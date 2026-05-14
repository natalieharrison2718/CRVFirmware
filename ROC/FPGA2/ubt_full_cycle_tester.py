#!/usr/bin/env python3
"""
ubt_full_cycle_tester.py  --  ROC full data-cycle tester
=========================================================
Tests the complete loop:
  FEB sends data → arrives in PhyRxBuff → DDR sequencer drains it
  → PhyRxBuff empty → P3 re-arms ReadyStatus
  → AutoTx sends UBT → FEB responds → repeating

Pauses at startup so you can open a PACR session in a separate screen:
  screen -S pacr
  # Inside screen: connect your FEB terminal and run PACR

Usage
-----
  python ubt_full_cycle_tester.py --ip 192.168.157.97 --port 5002 --ga 1
  python ubt_full_cycle_tester.py --ip 192.168.157.97 --port 5002 --ga 1 \\
         --cycles 20 --ports 0 1 2 --no-pause

  # Increase reply timeout if FEB is slow:
  python ubt_full_cycle_tester.py --ip 192.168.157.97 --ga 1 --reply-timeout 2.0

Transport
---------
  Uses uc_adapter_tcp (same as ubt_data_guard_monitor.py).

Register addresses from Proj_Defs.vhd:
  0x000  CSRRegAddr
  0x012  PhyTxCSRAddr
  0x013  PhyTxBuff_Count
  0x016  RxDAVAddr          (bit i = port i has data in PhyRxBuff)
  0x017  InputMaskAddr
  0x018..0x01F  PhyRxWdUsed[0..7]
  0x02F  FEBFMActiveAD      (bit i = FM link active on port i)
  0x049  LastTxTargetAddr   (one-hot: last PhyTxBuff word target)
  0x1A0  ReadyStatusAddr
  0x1A1  ReadyClearAddr
  0x1A2  ReadyForceAddr
"""

import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    import uc_adapter_tcp as uc
except ImportError as exc:
    print(f"ERROR: could not import uc_adapter_tcp: {exc}")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Register map  (10-bit addresses, from Proj_Defs.vhd)
# ---------------------------------------------------------------------------

class R:
    CSRRegAddr       = 0x000
    PhyTxCSRAddr     = 0x012
    PhyTxCntAddr     = 0x013
    RxDAVAddr        = 0x016
    InputMaskAddr    = 0x017
    PhyRxWdUsed      = [0x018 + i for i in range(8)]
    FEBFMActiveAD    = 0x02F
    LastTxTargetAddr = 0x049
    TxFifoRawEmpty   = 0x04C
    TxEnMaskAd       = 0x00E    # one-hot of last-loaded TxEn mask (0x00 = no PHY enabled)
    UBTTargetStatusAddr = 0x062 # bit 0 = UBTTarget_empty, bit 1 = UBTTarget_full
    ReadyStatusAddr  = 0x1A0
    ReadyClearAddr   = 0x1A1
    ReadyForceAddr   = 0x1A2

CSR_OPEN  = 0x00AC   # PHY_on | FMRxEn | DDRWrt_En | DDRRd_en
CSR_CLOSE = 0x000C   # FMRxEn only

# Phase timeouts (seconds)
T_UBT_FIRE   = 2.0    # max wait for LastTxTarget to change after forcing ReadyStatus
T_FEB_REPLY  = 1.0    # max wait for FEB data to arrive in PhyRxBuff   (overridden by --reply-timeout)
T_DDR_DRAIN  = 3.0    # max wait for PhyRxBuff to go empty (DDR drain)
T_REARM      = 5.0    # max wait for ReadyStatus bit to rise again (P3 re-arm)

# Polling intervals (seconds)
POLL_FAST = 0.005
POLL_MED  = 0.010
POLL_SLOW = 0.050

# ---------------------------------------------------------------------------
# Transport
# ---------------------------------------------------------------------------

class ROC:
    def __init__(self, host: str, port: int, ga: int,
                 timeout: float = 2.0, debug: bool = False):
        self._ga    = ga
        self._debug = debug
        uc.set_config(host=host, port=port, debug=debug)

    def _a16(self, addr10: int) -> int:
        return uc.compose_a16(self._ga, addr10 & 0x3FF)

    def read(self, addr10: int) -> Optional[int]:
        try:
            return uc.uc_read(self._a16(addr10))
        except Exception as e:
            if self._debug:
                print(f"  [uc] rd 0x{addr10:03X} err: {e}")
            return None

    def write(self, addr10: int, value: int) -> bool:
        try:
            ok, _ = uc.uc_write(self._a16(addr10), value & 0xFFFF)
            return bool(ok)
        except Exception as e:
            if self._debug:
                print(f"  [uc] wr 0x{addr10:03X}=0x{value:04X} err: {e}")
            return False

    def close(self):
        try:
            uc.close()
        except Exception:
            pass

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def one_hot_port(v: Optional[int]) -> Optional[int]:
    if v is None or v == 0:
        return None
    v &= 0xFF
    if v & (v - 1):
        return None
    for i in range(8):
        if v & (1 << i):
            return i
    return None

def mask_to_ports(m: int) -> List[int]:
    return [i for i in range(8) if m & (1 << i)]

def _decode_csr(v: int) -> str:
    b = []
    if v & 0x04: b.append("PHY_on")
    if v & 0x08: b.append("FMRxEn")
    if v & 0x20: b.append("DDRWrt_En")
    if v & 0x80: b.append("DDRRd_en")
    return " | ".join(b) if b else "(none)"

def _ts(t0: float) -> str:
    return f"[{(time.time()-t0)*1000:>8.1f}ms]"

def _wait_for(
    condition,          # callable() -> bool
    timeout_s: float,
    poll_s: float = POLL_FAST,
) -> Tuple[bool, float]:
    """Poll until condition() is True or timeout.  Returns (ok, elapsed_ms)."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if condition():
            return True, (time.time() - t0) * 1000.0
        time.sleep(poll_s)
    return False, timeout_s * 1000.0

# ---------------------------------------------------------------------------
# OPEN / CLOSE
# ---------------------------------------------------------------------------

def open_roc(roc: ROC, mask: int = 0xFF, settle_s: float = 0.3) -> bool:
    print(f"\n  → OPEN ROC  (mask=0x{mask:02X})")
    roc.write(R.InputMaskAddr, mask)
    time.sleep(0.01)
    roc.write(R.CSRRegAddr, CSR_OPEN)
    time.sleep(settle_s)
    roc.write(R.ReadyClearAddr, 0xFF)
    time.sleep(0.02)

    csr   = roc.read(R.CSRRegAddr)
    mkr   = roc.read(R.InputMaskAddr)
    rxact = roc.read(R.FEBFMActiveAD)

    ok = True
    if csr is not None and (csr & 0xAC) != 0xAC:
        print(f"  ⚠  CSR readback 0x{csr:04X}  (expected bits 0xAC set)")
        ok = False
    else:
        print(f"  ✓  CSR        = 0x{(csr or 0):04X}  ({_decode_csr(csr or 0)})")
    if mkr is not None and (mkr & 0xFF) != (mask & 0xFF):
        print(f"  ⚠  InputMask readback 0x{(mkr or 0)&0xFF:02X}  (wrote 0x{mask:02X})")
        ok = False
    else:
        print(f"  ✓  InputMask  = 0x{(mkr or 0)&0xFF:02X}")
    active = mask_to_ports((rxact or 0) & 0xFF)
    print(f"     FEBFMActive= 0x{(rxact or 0)&0xFF:02X}  active: {active}")
    if not active:
        print(f"  ⚠  No FM links detected — UBTs will time out")
    return ok

def close_roc(roc: ROC) -> None:
    print(f"\n  → CLOSE ROC")
    roc.write(R.ReadyClearAddr, 0xFF)
    time.sleep(0.01)
    roc.write(R.InputMaskAddr, 0x00)
    time.sleep(0.01)
    roc.write(R.CSRRegAddr, CSR_CLOSE)
    time.sleep(0.02)
    csr = roc.read(R.CSRRegAddr)
    mkr = roc.read(R.InputMaskAddr)
    print(f"     CSR=0x{(csr or 0):04X}  Mask=0x{(mkr or 0)&0xFF:02X}")

# ---------------------------------------------------------------------------
# PACR pause banner
# ---------------------------------------------------------------------------

PACR_BANNER = """
╔══════════════════════════════════════════════════════════════╗
║           PAUSE — open PACR now in a separate screen        ║
║                                                              ║
║  In a new terminal:                                          ║
║    screen -S pacr                                            ║
║    # connect to FEB  e.g.  telnet 192.168.x.x               ║
║    PACR 100                                                  ║
║                                                              ║
║  PACR will let you see each UBT packet as it arrives.       ║
║  When ready, press  ENTER  here to start the cycle test.    ║
╚══════════════════════════════════════════════════════════════╝
"""

def pacr_pause(skip: bool) -> None:
    if skip:
        print("  (--no-pause: skipping PACR screen setup prompt)")
        return
    print(PACR_BANNER)
    try:
        input("  Press ENTER to start ► ")
    except (EOFError, KeyboardInterrupt):
        pass
    print()

# ---------------------------------------------------------------------------
# Full-cycle test
# ---------------------------------------------------------------------------

# Result codes for each phase
PASS = "PASS"
TOUT = "TOUT"
SKIP = "SKIP"

class CycleResult:
    __slots__ = ["cycle", "port",
                 "ubt_ok",  "ubt_ms",
                 "data_ok", "data_ms",
                 "drain_ok","drain_ms",
                 "rearm_ok","rearm_ms",
                 "note"]
    def __init__(self, cycle, port):
        self.cycle    = cycle
        self.port     = port
        self.ubt_ok   = TOUT;  self.ubt_ms   = 0.0
        self.data_ok  = TOUT;  self.data_ms  = 0.0
        self.drain_ok = SKIP;  self.drain_ms = 0.0
        self.rearm_ok = SKIP;  self.rearm_ms = 0.0
        self.note     = ""

def _phase_tag(s: str) -> str:
    return {"PASS": "✓", "TOUT": "✗", "SKIP": "—"}.get(s, s)


def run_full_cycle_test(
    roc:            ROC,
    ports:          List[int],
    cycles_per_port: int,
    t_feb_reply:    float,
    t_ubt:          float,
    t_drain:        float,
    t_rearm:        float,
    t0:             float,
) -> List[CycleResult]:
    """
    For each port × cycle:
      Phase 1 – Force ReadyStatus[port] HIGH, wait for LastTxTarget → port
      Phase 2 – Wait for FEB data in PhyRxBuff (RxDAV or WdUsed > 0)
      Phase 3 – Wait for PhyRxBuff to drain to zero (DDR sequencer read it)
      Phase 4 – Wait for ReadyStatus[port] to go HIGH again (P3 re-arm)
    """
    results: List[CycleResult] = []

    print(f"\n  {'═'*68}")
    print(f"  FULL CYCLE TEST  ports={ports}  cycles={cycles_per_port}/port")
    print(f"  {'═'*68}")
    print(f"""
  Phases tested per cycle:
    1. Force ReadyStatus → UBT fires to FEB
    2. FEB responds → data arrives in PhyRxBuff
    3. DDR sequencer drains PhyRxBuff
    4. P3 logic re-arms ReadyStatus (no manual write needed)
""")
    hdr = (f"  {'Cycle':>6}  {'Port':>5}  "
           f"{'UBT':>6}  {'Data':>6}  {'Drain':>6}  {'Rearm':>6}  Note")
    sep = f"  {'':─>6}  {'':─>5}  {'':─>6}  {'':─>6}  {'':─>6}  {'':─>6}  {'':─>46}"
    print(hdr)
    print(sep)

    for port in ports:
        for cyc in range(1, cycles_per_port + 1):
            res = CycleResult(cyc, port)


       
            # ── Phase 1: clear LastTxTarget, force ReadyStatus, watch for UBT ──
            roc.write(R.LastTxTargetAddr, 0xFFFF)   # any write clears the latch
            time.sleep(0.015)                        # 3+ i50MHz sync cycles @ 50 MHz
            
            roc.write(R.ReadyClearAddr, 1 << port)
            time.sleep(0.005)
            roc.write(R.ReadyForceAddr, 1 << port)
            
            print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  ReadyStatus[{port}] forced HIGH")
            
            expected_mask = 1 << port

            def ubt_fired():
                # ROBUSTNESS: PASS only when BOTH LastTxTarget AND TxEnMask
                # show the expected one-hot bit.  This guards against the CDC
                # race in which AutoTx_Target latched LastTxTarget but the
                # UBTTarget tag had not yet propagated, so TxEnMask stayed
                # 0x00 and no PHY enable was actually asserted.
                last = roc.read(R.LastTxTargetAddr) or 0
                mask = roc.read(R.TxEnMaskAd) or 0
                return (one_hot_port(last) == port
                        and (mask & 0xFF) == expected_mask)

            ok, ms = _wait_for(ubt_fired, t_ubt, POLL_FAST)   # ← THIS WAS MISSING
            last_val = roc.read(R.LastTxTargetAddr) or 0
            mask_val = (roc.read(R.TxEnMaskAd) or 0) & 0xFF
            ubt_status = roc.read(R.UBTTargetStatusAddr) or 0
            ubt_empty  = bool(ubt_status & 0x1)
            ubt_full   = bool(ubt_status & 0x2)
            if ok:
                res.ubt_ok = PASS; res.ubt_ms = ms
                print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  "
                      f"✓ UBT fired  last=0x{last_val:02X} mask=0x{mask_val:02X}  ({ms:.1f}ms)")
            else:
                res.ubt_ok = TOUT; res.ubt_ms = ms
                # Distinguish failure modes so future regressions are not
                # misattributed to the FEB.
                if (mask_val & expected_mask) == 0 and one_hot_port(last_val) == port:
                    fail_kind = "PHY-ENABLE-MISSING"
                    detail    = (f"LastTxTarget=0x{last_val:02X} latched but "
                                 f"TxEnMask=0x{mask_val:02X} (expected 0x{expected_mask:02X}) — "
                                 f"UBT staged but PHY enable never asserted "
                                 f"(UBTTarget empty={int(ubt_empty)} full={int(ubt_full)})")
                elif one_hot_port(last_val) != port:
                    fail_kind = "UBT-NOT-STAGED"
                    detail    = (f"LastTxTarget=0x{last_val:02X} TxEnMask=0x{mask_val:02X} "
                                 f"UBTTarget empty={int(ubt_empty)} full={int(ubt_full)}")
                else:
                    fail_kind = "UBT-TIMEOUT"
                    detail    = (f"LastTxTarget=0x{last_val:02X} TxEnMask=0x{mask_val:02X}")
                res.note = f"⚠ {fail_kind}  {detail}"
                print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  "
                      f"✗ {fail_kind}   {detail}")
                _print_row(res)
                results.append(res)
                time.sleep(0.05)
                continue
        
            # ── Phase 2: Wait for FEB data ──────────────────────────────────────
            def feb_data():
                dav = roc.read(R.RxDAVAddr) or 0
                wc  = roc.read(R.PhyRxWdUsed[port]) or 0
                return bool(dav & (1 << port)) or (wc & 0xFFF) > 0
       

            
            ok, ms = _wait_for(feb_data, t_feb_reply, POLL_FAST)
            wc_now = (roc.read(R.PhyRxWdUsed[port]) or 0) & 0xFFF
            dav_now= (roc.read(R.RxDAVAddr) or 0) >> port & 1
            if ok:
                res.data_ok = PASS; res.data_ms = ms
                print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  "
                      f"✓ FEB data  WdUsed={wc_now}  DAV={dav_now}  ({ms:.1f}ms)")
            else:
                res.data_ok = TOUT; res.data_ms = ms
                res.note = f"⚠ FEB no reply  WdUsed={wc_now}  DAV={dav_now}"
                print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  "
                      f"✗ FEB DATA TIMEOUT   WdUsed={wc_now}  DAV={dav_now}")
                _print_row(res)
                results.append(res)
                time.sleep(0.05)
                continue

            # ── Phase 3: Wait for DDR drain (PhyRxBuff goes to zero) ─
            def buf_empty():
                wc = roc.read(R.PhyRxWdUsed[port]) or 0
                return (wc & 0xFFF) == 0

            ok, ms = _wait_for(buf_empty, t_drain, POLL_MED)
            wc_end = (roc.read(R.PhyRxWdUsed[port]) or 0) & 0xFFF
            if ok:
                res.drain_ok = PASS; res.drain_ms = ms
                print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  "
                      f"✓ PhyRxBuff drained  WdUsed={wc_end}  ({ms:.1f}ms)")
            else:
                res.drain_ok = TOUT; res.drain_ms = ms
                res.note = f"⚠ DDR drain timeout  WdUsed={wc_end}"
                print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  "
                      f"✗ DDR DRAIN TIMEOUT  WdUsed={wc_end}")
                _print_row(res)
                results.append(res)
                time.sleep(0.05)
                continue

            # ── Phase 4: Wait for P3 re-arm (ReadyStatus goes HIGH) ──
            # Do NOT write ReadyForceAddr here; we want to confirm the
            # firmware's own P3/P4 logic re-arms the bit automatically.
            def ready_set():
                rs = roc.read(R.ReadyStatusAddr) or 0
                return bool(rs & (1 << port))

            ok, ms = _wait_for(ready_set, t_rearm, POLL_SLOW)
            rs_val = (roc.read(R.ReadyStatusAddr) or 0) & 0xFF
            if ok:
                res.rearm_ok = PASS; res.rearm_ms = ms
                print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  "
                      f"✓ ReadyStatus[{port}] re-armed  RS=0x{rs_val:02X}  ({ms:.1f}ms)")
            else:
                res.rearm_ok = TOUT; res.rearm_ms = ms
                res.note = (f"⚠ P3 re-arm timeout  RS=0x{rs_val:02X}")
                print(f"  {_ts(t0)} port{port} cyc {cyc:>2}  "
                      f"✗ P3 RE-ARM TIMEOUT  RS=0x{rs_val:02X}")

            _print_row(res)
            results.append(res)

    return results


def _print_row(res: CycleResult) -> None:
    def _col(s, ms):
        tag = _phase_tag(s)
        if s == SKIP:
            return f"{'—':>6}"
        if s == PASS:
            return f"{tag}{ms:5.0f}"
        return f"{tag} TOUT"

    row = (f"  {res.cycle:>6}  {res.port:>5}  "
           f"{_col(res.ubt_ok,  res.ubt_ms):>6}  "
           f"{_col(res.data_ok, res.data_ms):>6}  "
           f"{_col(res.drain_ok,res.drain_ms):>6}  "
           f"{_col(res.rearm_ok,res.rearm_ms):>6}  "
           f"{res.note}")
    print(row)


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

def print_summary(results: List[CycleResult], ports: List[int]) -> bool:
    print(f"\n  {'═'*68}")
    print(f"  SUMMARY")
    print(f"  {'═'*68}")

    totals = {p: {k: 0 for k in ["n","ubt","data","drain","rearm","viol"]}
              for p in ports}
    grand = {k: 0 for k in ["n","ubt","data","drain","rearm","viol"]}

    for r in results:
        p = r.port
        totals[p]["n"] += 1
        grand["n"] += 1
        for phase, attr in [("ubt","ubt_ok"),("data","data_ok"),
                             ("drain","drain_ok"),("rearm","rearm_ok")]:
            val = getattr(r, attr)
            if val == PASS:
                totals[p][phase] += 1
                grand[phase]     += 1
            if val == TOUT:
                totals[p]["viol"] += 1
                grand["viol"]     += 1

    def _frac(n, d):
        return f"{n}/{d}" if d else "—"

    hdr2 = (f"  {'Port':>5}  {'Cycles':>7}  "
            f"{'UBT':>6}  {'FEBdata':>8}  {'Drain':>7}  {'Rearm':>7}  {'Viols':>7}")
    print(hdr2)
    print(f"  {'':─>5}  {'':─>7}  {'':─>6}  {'':─>8}  {'':─>7}  {'':─>7}  {'':─>7}")
    for p in ports:
        t = totals[p]
        print(f"  {p:>5}  {t['n']:>7}  "
              f"{_frac(t['ubt'],  t['n']):>6}  "
              f"{_frac(t['data'], t['ubt'] if t['ubt'] else 1):>8}  "
              f"{_frac(t['drain'],t['data'] if t['data'] else 1):>7}  "
              f"{_frac(t['rearm'],t['drain'] if t['drain'] else 1):>7}  "
              f"{t['viol']:>7}")

    g = grand
    print(f"  {'TOTAL':>5}  {g['n']:>7}  "
          f"{_frac(g['ubt'],  g['n']):>6}  "
          f"{_frac(g['data'], g['ubt'] if g['ubt'] else 1):>8}  "
          f"{_frac(g['drain'],g['data'] if g['data'] else 1):>7}  "
          f"{_frac(g['rearm'],g['drain'] if g['drain'] else 1):>7}  "
          f"{g['viol']:>7}")

    passed = g["viol"] == 0
    print(f"\n  Overall: {'PASS ✓' if passed else 'FAIL ✗  ('+str(g['viol'])+' violation(s))'}")
    return passed


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="ROC full data-cycle tester with PACR visibility",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Four-phase cycle test:
  Phase 1  Force ReadyStatus[port] HIGH → watch AutoTx fire UBT
  Phase 2  Wait for FEB data in PhyRxBuff (RxDAV or PhyRxWdUsed > 0)
  Phase 3  Wait for PhyRxBuff to drain (DDR sequencer consumed data)
  Phase 4  Wait for ReadyStatus[port] to go HIGH again (P3 re-arm, no help)

Before the test starts you are prompted to open PACR in a separate screen.
PACR shows each incoming UBT packet at the FEB side.

Example:
  screen -S pacr
  # in that screen: telnet <FEB_IP> → PACR 100

  # In your main terminal:
  python ubt_full_cycle_tester.py --ip 192.168.157.97 --ga 1 --cycles 10

Tip: if UBT fires but PACR shows nothing, the FEB is not seeing the packet
     (Ethernet MAC / UBTTarget FIFO issue).
     If PACR shows the UBT but Phase 2 times out, the FEB is not replying.
        """,
    )
    parser.add_argument("--ip",     default="192.168.157.97")
    parser.add_argument("--port",   type=int, default=5002,
                        help="Bridge TCP port (default 5002)")
    parser.add_argument("--ga",     type=int, default=1,
                        help="Geographic address 0-3 (default 1)")
    parser.add_argument("--ports",  type=int, nargs="+", default=None,
                        help="Port(s) to test (default: auto-detect from FEBFMActive)")
    parser.add_argument("--mask",   type=lambda x: int(x,0), default=None,
                        help="8-bit hex port mask for OPEN (default 0xFF)")
    parser.add_argument("--cycles", type=int, default=10,
                        help="Cycles per port (default 10)")
    parser.add_argument("--reply-timeout", type=float, default=1.0,
                        help="Seconds to wait for FEB reply (Phase 2, default 1.0)")
    parser.add_argument("--ubt-timeout",   type=float, default=2.0,
                        help="Seconds to wait for UBT to fire (Phase 1, default 2.0)")
    parser.add_argument("--drain-timeout", type=float, default=3.0,
                        help="Seconds to wait for DDR drain (Phase 3, default 3.0)")
    parser.add_argument("--rearm-timeout", type=float, default=5.0,
                        help="Seconds to wait for P3 re-arm (Phase 4, default 5.0)")
    parser.add_argument("--open-settle", type=float, default=0.3,
                        help="Seconds after CSR write for FM links to settle (default 0.3)")
    parser.add_argument("--no-pause",  action="store_true",
                        help="Skip the PACR screen setup prompt")
    parser.add_argument("--no-open",   action="store_true",
                        help="Skip OPEN sequence")
    parser.add_argument("--no-close",  action="store_true",
                        help="Skip CLOSE sequence")
    parser.add_argument("--debug",     action="store_true",
                        help="Print raw bridge I/O")
    args = parser.parse_args()

    roc = ROC(args.ip, args.port, args.ga, debug=args.debug)
    rc  = 0
    t0  = time.time()

    try:
        eff_mask = args.mask if args.mask is not None else 0xFF

        # ── OPEN ────────────────────────────────────────────────────────
        if not args.no_open:
            open_roc(roc, eff_mask, settle_s=args.open_settle)

        # ── Determine ports to test ──────────────────────────────────────
        if args.ports is not None:
            test_ports = args.ports
        else:
            rxact = roc.read(R.FEBFMActiveAD) or 0
            test_ports = mask_to_ports(rxact & 0xFF)
            print(f"\n  Auto-detected active ports: {test_ports}  "
                  f"(FEBFMActive=0x{rxact&0xFF:02X})")
            if not test_ports:
                print(f"  ⚠  No active FM links — exiting.")
                return 1

        # ── PACR pause ───────────────────────────────────────────────────
        pacr_pause(args.no_pause)

        # ── Run test ────────────────────────────────────────────────────
        results = run_full_cycle_test(
            roc           = roc,
            ports         = test_ports,
            cycles_per_port = args.cycles,
            t_feb_reply   = args.reply_timeout,
            t_ubt         = args.ubt_timeout,
            t_drain       = args.drain_timeout,
            t_rearm       = args.rearm_timeout,
            t0            = t0,
        )

        passed = print_summary(results, test_ports)
        rc = 0 if passed else 1

        # ── CLOSE ───────────────────────────────────────────────────────
        if not args.no_close:
            close_roc(roc)

    except KeyboardInterrupt:
        print("\n  Interrupted by user.")
        rc = 1
    finally:
        roc.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())
