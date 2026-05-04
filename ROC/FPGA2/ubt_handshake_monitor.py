#!/usr/bin/env python3
"""
ubt_handshake_monitor.py  —  ROC (Controller_FPGA2) sequential UBT handshake test tool
========================================================================================
Discovers active FEB ports automatically via FEBFMActiveAD (Rx_active bitmap)
and tests/monitors only those ports.  --test-port is now optional; if omitted
the first active port is used for the 'single' mode.
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
# Register addresses (10-bit addr10)
# ---------------------------------------------------------------------------
class R:
    CSRRegAddr          = 0x000
    InputMaskAddr       = 0x017
    PhyTxCSRAddr        = 0x012
    PhyTxCntAddr        = 0x013
    RxDAVAddr           = 0x016
    FEBFMActiveAD       = 0x02F   # NEW: Rx_active bitmap (bit i = port i active)
    LastTxTargetAddr    = 0x049
    TxCurrentTargetAddr = 0x04A
    TxFifoRawEmptyAddr  = 0x04C
    AutoTxKickAddr      = 0x04D
    TxFifoResetAddr     = 0x04E
    ReadyStatusAddr     = 0x1A0
    ReadyClearAddr      = 0x1A1
    ReadyForceAddr      = 0x1A2
    PhyRxWdUsed         = [0x018 + i for i in range(8)]
    PHYActivityCntAdd   = [0x070 + i for i in range(8)]

CSR_OPEN  = 0x00AC
CSR_CLOSE = 0x000C
UBT_TIMEOUT_MS = 10.0
TIMEOUT_GAP_MS = 9.0

# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------

def _a(ga: int, addr10: int) -> int:
    return uc.compose_a16(ga, addr10)

def _rd(ga: int, addr10: int) -> Optional[int]:
    try:
        return uc.uc_read(_a(ga, addr10))
    except Exception:
        return None

def _wr(ga: int, addr10: int, value: int) -> bool:
    try:
        ok, _ = uc.uc_write(_a(ga, addr10), value)
        return ok
    except Exception:
        return False

def one_hot_to_port(val: Optional[int]) -> Optional[int]:
    if val is None or val == 0:
        return None
    v = val & 0xFF
    if v & (v - 1):
        return None
    for i in range(8):
        if v & (1 << i):
            return i
    return None

def mask_to_ports(mask: int) -> List[int]:
    return [i for i in range(8) if mask & (1 << i)]

def _decode_csr(val: int) -> str:
    bits = []
    if val & 0x04: bits.append("PHY_on")
    if val & 0x08: bits.append("FMRxEn")
    if val & 0x20: bits.append("DDRWrt_En")
    if val & 0x80: bits.append("DDRRd_en")
    return " | ".join(bits) if bits else "(none)"

# ---------------------------------------------------------------------------
# Port discovery  — reads FEBFMActiveAD directly (Rx_active register)
# ---------------------------------------------------------------------------

def discover_active_ports(ga: int, settle_s: float = 0.0) -> List[int]:
    """
    Return sorted list of local port indices (0-7) whose FM link is live.

    Uses FEBFMActiveAD (0x02F), which mirrors the firmware's Rx_active signal.
    Each bit is set when the 10-us FM-edge-detector window sees >=15 transitions
    for 4 consecutive windows (~40 ms after the PHY comes up).

    settle_s > 0 adds an explicit wait before sampling, useful right after OPEN.
    """
    if settle_s > 0:
        time.sleep(settle_s)
    val = _rd(ga, R.FEBFMActiveAD)
    if val is None:
        print(f"  [GA={ga}] ⚠  FEBFMActiveAD read failed — no active ports assumed")
        return []
    active = mask_to_ports(val & 0xFF)
    print(f"  [GA={ga}] FEBFMActive=0x{val & 0xFF:02X}  "
          f"→ active local ports {active}")
    return active

def active_mask(ga: int) -> int:
    """Return the 8-bit Rx_active mask, or 0 on error."""
    val = _rd(ga, R.FEBFMActiveAD)
    return (val or 0) & 0xFF

# ---------------------------------------------------------------------------
# Status snapshot
# ---------------------------------------------------------------------------

def print_status(ga: int) -> None:
    print(f"\n{'='*60}")
    print(f"  ROC register snapshot  (GA={ga})")
    print(f"{'='*60}")

    csr   = _rd(ga, R.CSRRegAddr)
    mask  = _rd(ga, R.InputMaskAddr)
    rs    = _rd(ga, R.ReadyStatusAddr)
    dav   = _rd(ga, R.RxDAVAddr)
    rxact = _rd(ga, R.FEBFMActiveAD)
    txcsr = _rd(ga, R.PhyTxCSRAddr)
    txcnt = _rd(ga, R.PhyTxCntAddr)
    txraw = _rd(ga, R.TxFifoRawEmptyAddr)
    last  = _rd(ga, R.LastTxTargetAddr)
    curr  = _rd(ga, R.TxCurrentTargetAddr)

    def _h(v, w=4): return f"0x{v:0{w}X}" if v is not None else "N/A"
    def _b(v, w=2): return f"0x{v:0{w}X}" if v is not None else "N/A"

    print(f"  CSR             : {_h(csr)}  ({_decode_csr(csr or 0)})")
    print(f"  InputMask       : {_b(mask)}")
    print(f"  ReadyStatus     : {_b(rs)}")
    print(f"  RxDAV (~empty)  : {_b(dav)}")
    print(f"  FEBFMActive     : {_b(rxact)}  "
          f"(active: {mask_to_ports((rxact or 0) & 0xFF)})")
    print(f"  PhyTxCSR        : {_h(txcsr)}  "
          f"(TxEnAck={(txcsr or 0)&1}  RdEmpty={(txcsr or 0)>>1&1})")
    print(f"  PhyTxBuff_Count : {(txcnt or 0) & 0x7FF}")
    print(f"  TxFifoRawEmpty  : {(txraw or 0) & 1}")
    print(f"  LastTxTarget    : {_b(last)}  (port {one_hot_to_port(last)})")
    print(f"  TxCurrentTarget : {_b(curr)}  (port {one_hot_to_port(curr)})")

    if rxact is not None and mask is not None:
        dead = [p for p in mask_to_ports(mask & 0xFF)
                if not (rxact & (1 << p))]
        if dead:
            print(f"\n  ⚠  Masked but no FM link: {dead}  "
                  f"(these ports will always time out)")

    print(f"\n  {'Port':>4}  {'RxWds':>6}  {'DAV':>4}  "
          f"{'FMActive':>9}  {'Activity':>9}")
    for i in range(8):
        wc      = _rd(ga, R.PhyRxWdUsed[i])
        act_cnt = _rd(ga, R.PHYActivityCntAdd[i])
        dav_bit = ((dav or 0) >> i) & 1 if dav is not None else '?'
        fm_bit  = ((rxact or 0) >> i) & 1 if rxact is not None else '?'
        print(f"  {i:>4}  {(wc or 0) & 0xFFF:>6}  {dav_bit:>4}  "
              f"{fm_bit:>9}  {(act_cnt or 0) & 0xFFFF:>9}")
    print()

# ---------------------------------------------------------------------------
# OPEN / CLOSE sequences
# ---------------------------------------------------------------------------

def open_roc(ga: int, mask: int = 0xFF, settle_s: float = 0.05) -> bool:
    """
    Send the OPEN sequence.  settle_s gives the FM-edge-detector time to
    assert Rx_active on ports with live FM links before we sample them.
    """
    print(f"\n  → OPEN ROC (GA={ga}, mask=0x{mask:02X})")
    _wr(ga, R.InputMaskAddr, mask)
    time.sleep(0.01)
    _wr(ga, R.CSRRegAddr, CSR_OPEN)
    time.sleep(0.01)
    _wr(ga, R.ReadyClearAddr, 0xFF)
    time.sleep(settle_s)   # wait for FM links to stabilise

    csr_rb  = _rd(ga, R.CSRRegAddr)
    mask_rb = _rd(ga, R.InputMaskAddr)
    rs_rb   = _rd(ga, R.ReadyStatusAddr)
    rxact   = _rd(ga, R.FEBFMActiveAD)
    ok = True

    if csr_rb is not None and (csr_rb & 0xAC) != 0xAC:
        print(f"  ⚠  CSR readback 0x{csr_rb:04X} — expected bits 0xAC set")
        ok = False
    else:
        print(f"  ✓  CSR = 0x{(csr_rb or 0):04X}")
    if mask_rb is not None and (mask_rb & 0xFF) != (mask & 0xFF):
        print(f"  ⚠  InputMask readback 0x{mask_rb:04X}")
        ok = False
    else:
        print(f"  ✓  InputMask = 0x{(mask_rb or 0) & 0xFF:02X}")
    print(f"     ReadyStatus = 0x{(rs_rb or 0) & 0xFF:02X}")
    if rxact is not None:
        active = mask_to_ports(rxact & 0xFF)
        print(f"  ✓  FEBFMActive = 0x{rxact & 0xFF:02X}  active ports: {active}")
        if not active:
            print(f"  ⚠  No FM links detected.  Ports will time out in handshake tests.")
    return ok

def close_roc(ga: int) -> bool:
    print(f"\n  → CLOSE ROC (GA={ga})")
    _wr(ga, R.ReadyClearAddr, 0xFF)
    time.sleep(0.01)
    _wr(ga, R.InputMaskAddr, 0x00)
    time.sleep(0.01)
    _wr(ga, R.CSRRegAddr, CSR_CLOSE)
    time.sleep(0.02)
    csr_rb  = _rd(ga, R.CSRRegAddr)
    mask_rb = _rd(ga, R.InputMaskAddr)
    rs_rb   = _rd(ga, R.ReadyStatusAddr)
    ok = True
    print(f"  CSR=0x{(csr_rb or 0):04X}  "
          f"Mask=0x{(mask_rb or 0) & 0xFF:02X}  "
          f"RS=0x{(rs_rb or 0) & 0xFF:02X}")
    if (csr_rb or 0) & 0xFF != 0x0C: ok = False
    if (mask_rb or 0) & 0xFF != 0x00: ok = False
    if (rs_rb or 0) & 0xFF != 0x00:   ok = False
    return ok

# ---------------------------------------------------------------------------
# Polling snapshot
# ---------------------------------------------------------------------------

class Snapshot:
    __slots__ = ("t_ms","rs","dav","rxact","last_tgt","curr_tgt",
                 "txcnt","txcsr","txraw")
    def __init__(self, t_ms: float, ga: int):
        self.t_ms     = t_ms
        self.rs       = _rd(ga, R.ReadyStatusAddr)
        self.dav      = _rd(ga, R.RxDAVAddr)
        self.rxact    = _rd(ga, R.FEBFMActiveAD)
        self.last_tgt = _rd(ga, R.LastTxTargetAddr)
        self.curr_tgt = _rd(ga, R.TxCurrentTargetAddr)
        self.txcnt    = _rd(ga, R.PhyTxCntAddr)
        self.txcsr    = _rd(ga, R.PhyTxCSRAddr)
        self.txraw    = _rd(ga, R.TxFifoRawEmptyAddr)

# ---------------------------------------------------------------------------
# Monitor mode  — auto-uses whatever ports are alive
# ---------------------------------------------------------------------------

def monitor(ga: int, duration_s: float, interval_ms: float) -> int:
    """
    Poll for UBT transmissions and FEB replies.
    Active port set is determined live from FEBFMActiveAD each tick so the
    monitor naturally follows ports that come up or go down mid-run.
    Returns the violation count.
    """
    print(f"\n  Monitoring for {duration_s:.0f}s  "
          f"(poll {interval_ms:.0f}ms, GA={ga})")
    print(f"\n  {'Time(ms)':>10}  {'Event':<22}  {'Port':>5}  Detail")
    print(f"  {'':─>10}  {'':─>22}  {'':─>5}  {'':─>44}")

    t0           = time.time()
    prev_last    : Optional[int]        = None
    prev_dav     : int                  = 0
    ubt_times    : Dict[int, float]     = {}
    replied      : Dict[int, bool]      = {}
    last_ubt_port: Optional[int]        = None
    last_ubt_t   : float                = 0.0
    violations   = 0
    ubt_count    = 0
    reply_count  = 0
    rtts         : Dict[int, List[float]] = {i: [] for i in range(8)}
    interval_s   = interval_ms / 1000.0

    while True:
        now  = time.time()
        t_ms = (now - t0) * 1000.0
        if t_ms > duration_s * 1000.0:
            break

        snap = Snapshot(t_ms, ga)
        dav   = snap.dav or 0
        rxact = snap.rxact or 0

        # ── UBT detection: LastTxTarget changed ────────────────────────
        lt      = snap.last_tgt
        lt_port = one_hot_to_port(lt)
        if lt is not None and lt != prev_last and lt != 0 and lt_port is not None:
            ubt_count += 1
            fm_ok  = bool(rxact & (1 << lt_port))
            tag    = "" if fm_ok else "⚠ no FM link on this port"
            status_tag = ""

            if last_ubt_port is not None and not replied.get(last_ubt_port, False):
                gap = t_ms - last_ubt_t
                if gap < TIMEOUT_GAP_MS:
                    violations += 1
                    if lt_port == last_ubt_port:
                        status_tag = f"⚠ SAME-PORT RE-TX (gap={gap:.1f}ms)"
                    else:
                        status_tag = (f"⚠ VIOLATION gap={gap:.1f}ms "
                                      f"(prev=port{last_ubt_port})")
                else:
                    status_tag = f"timeout OK {gap:.1f}ms"

            detail = (f"RS=0x{(snap.rs or 0)&0xFF:02X}  "
                      f"RxDAV=0x{dav&0xFF:02X}  "
                      f"FMActive=0x{rxact&0xFF:02X}  {tag}  {status_tag}")
            print(f"  {t_ms:>10.1f}  {'UBT →':<22}  "
                  f"{'port '+str(lt_port):>5}  {detail}")

            ubt_times[lt_port] = t_ms
            replied[lt_port]   = False
            last_ubt_port      = lt_port
            last_ubt_t         = t_ms

        prev_last = lt

        # ── Reply detection: RxDAV bit rose ────────────────────────────
        new_bits = dav & ~prev_dav & 0xFF
        if new_bits:
            for p in range(8):
                if new_bits & (1 << p):
                    reply_count += 1
                    replied[p]   = True
                    rtt          = t_ms - ubt_times.get(p, t_ms)
                    rtts[p].append(rtt)
                    print(f"  {t_ms:>10.1f}  {'← FEB reply':<22}  "
                          f"{'port '+str(p):>5}  RTT={rtt:.1f}ms  "
                          f"FMActive=0x{rxact&0xFF:02X}")
        prev_dav = dav
        time.sleep(interval_s)

    # ── Summary ────────────────────────────────────────────────────────
    final_rxact = active_mask(ga)
    print(f"\n  {'='*60}")
    print(f"  MONITOR SUMMARY  ({duration_s:.0f}s, GA={ga})")
    print(f"  {'='*60}")
    print(f"  Active FEB ports (at end) : {mask_to_ports(final_rxact)}")
    print(f"  UBT sent   : {ubt_count}")
    print(f"  FEB replies: {reply_count}")
    print(f"  VIOLATIONS : {violations}")
    _print_rtt_table(rtts)
    return violations

# ---------------------------------------------------------------------------
# Single-port test  — auto-selects first active port if none given
# ---------------------------------------------------------------------------

def test_single_port(
    ga: int,
    port: Optional[int] = None,
    timeout_s: float = 5.0,
) -> bool:
    """
    Test one port.  If port is None, auto-select the first port with a live
    FM link.  Falls back to port 0 with a warning if no FM link is detected.
    """
    rxact  = active_mask(ga)
    active = mask_to_ports(rxact)

    if port is None:
        if not active:
            print(f"  ⚠  No active FM links detected "
                  f"(FEBFMActive=0x{rxact:02X}).  "
                  f"Defaulting to port 0 — test will likely time out.")
            port = 0
        else:
            port = active[0]
            print(f"  Auto-selected port {port} from active ports {active}")
    else:
        if not (rxact & (1 << port)):
            print(f"  ⚠  Port {port} has no active FM link "
                  f"(FEBFMActive=0x{rxact:02X}) — test may time out")

    print(f"\n  ── Single-port test: port {port}  (GA={ga}) ──")

    _wr(ga, R.ReadyClearAddr, 0xFF)
    time.sleep(0.02)

    _wr(ga, R.ReadyForceAddr, 1 << port)
    time.sleep(0.01)
    _wr(ga, R.AutoTxKickAddr, 1 << port)
    time.sleep(0.01)

    t0        = time.time()
    ubt_seen  = False
    other_ubt = False
    reply_seen= False
    prev_dav  = _rd(ga, R.RxDAVAddr) or 0
    prev_last = _rd(ga, R.LastTxTargetAddr)
    ubt_t     = 0.0

    print(f"\n  {'Time(ms)':>10}  {'Event':<24}  Detail")
    print(f"  {'':─>10}  {'':─>24}  {'':─>42}")

    while (time.time() - t0) < timeout_s:
        t_ms = (time.time() - t0) * 1000.0
        lt   = _rd(ga, R.LastTxTargetAddr)
        dav  = _rd(ga, R.RxDAVAddr) or 0

        if lt is not None and lt != prev_last and lt != 0:
            lt_port = one_hot_to_port(lt)
            if lt_port == port:
                if not ubt_seen:
                    print(f"  {t_ms:>10.1f}  {'✓ UBT sent':<24}  port {port}")
                    ubt_seen = True
                    ubt_t    = t_ms
                else:
                    print(f"  {t_ms:>10.1f}  {'⚠ RE-TX same port':<24}  "
                          f"gap={t_ms-ubt_t:.1f}ms — VIOLATION")
                    other_ubt = True
            elif lt_port is not None:
                gap = t_ms - ubt_t
                if gap < TIMEOUT_GAP_MS:
                    print(f"  {t_ms:>10.1f}  {'⚠ OTHER port UBT':<24}  "
                          f"port {lt_port}  gap={gap:.1f}ms — VIOLATION")
                    other_ubt = True
                else:
                    print(f"  {t_ms:>10.1f}  {'(timeout OK)':<24}  "
                          f"port {lt_port}  gap={gap:.1f}ms")
        prev_last = lt

        if (dav & (1 << port)) and not (prev_dav & (1 << port)):
            rtt = t_ms - ubt_t
            print(f"  {t_ms:>10.1f}  {'✓ FEB reply':<24}  "
                  f"port {port}  RTT={rtt:.1f}ms")
            reply_seen = True
            break
        prev_dav = dav
        time.sleep(0.005)

    passed = ubt_seen and not other_ubt
    print(f"\n  Result: {'PASS ✓' if passed else 'FAIL ✗'}  "
          f"(UBT_seen={ubt_seen}  other_ubt={other_ubt}  "
          f"FEB_reply={reply_seen})")
    return passed

# ---------------------------------------------------------------------------
# Sequential handshake test  — auto-discovers active ports
# ---------------------------------------------------------------------------

def test_sequential_handshake(
    ga: int,
    mask: Optional[int] = None,
    rounds: int = 2,
    timeout_s: float = 60.0,
) -> bool:
    """
    Run `rounds` complete cycles over all active FEB ports.

    If mask is None, the active port set is read from FEBFMActiveAD.
    Ports with no FM link are automatically excluded — they would always time
    out and distort the statistics.

    Returns True if no violations found.
    """
    # Determine effective mask
    rxact = active_mask(ga)
    if mask is None:
        eff_mask = rxact        # only ports with live FM links
        print(f"\n  Auto-detected active ports: "
              f"{mask_to_ports(eff_mask)}  (FEBFMActive=0x{eff_mask:02X})")
    else:
        dead = [p for p in mask_to_ports(mask) if not (rxact & (1 << p))]
        if dead:
            print(f"  ⚠  Ports {dead} in mask have no FM link "
                  f"(FEBFMActive=0x{rxact:02X}) — they will time out")
        eff_mask = mask & 0xFF

    if eff_mask == 0:
        print(f"  ✗  No active FEB ports found (FEBFMActive=0x{rxact:02X}) "
              f"— aborting sequential test.")
        return False

    ports_in_test = mask_to_ports(eff_mask)
    print(f"\n  ── Sequential handshake test  GA={ga}  "
          f"ports={ports_in_test}  rounds={rounds} ──")
    print(f"\n  {'Time(ms)':>10}  {'Event':<22}  {'Port':>5}  Detail")
    print(f"  {'':─>10}  {'':─>22}  {'':─>5}  {'':─>44}")

    violations    = 0
    total_ubts    = 0
    total_replies = 0
    rtts: Dict[int, List[float]] = {i: [] for i in range(8)}

    _wr(ga, R.ReadyClearAddr, 0xFF)
    time.sleep(0.02)

    for rnd in range(rounds):
        print(f"\n  ── Round {rnd+1}/{rounds} ──")

        # Re-sample active ports at the start of each round so the test
        # adapts if a FEB comes up or goes away between rounds.
        current_rxact = active_mask(ga)
        round_mask    = eff_mask & current_rxact
        if round_mask == 0:
            print(f"  ⚠  No active ports at round start — skipping.")
            continue
        if round_mask != eff_mask:
            dropped = mask_to_ports(eff_mask & ~current_rxact)
            print(f"  ⚠  Ports {dropped} lost FM link, skipping them this round")

        _wr(ga, R.ReadyForceAddr, round_mask)
        time.sleep(0.01)

        t0        = time.time()
        prev_last = _rd(ga, R.LastTxTargetAddr)
        prev_dav  = _rd(ga, R.RxDAVAddr) or 0
        last_ubt_port: Optional[int] = None
        last_ubt_t:    float         = 0.0
        ubt_times: Dict[int, float] = {}
        replied:   Dict[int, bool]  = {}
        served:    set              = set()

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            t_ms  = (time.time() - t0) * 1000.0
            lt    = _rd(ga, R.LastTxTargetAddr)
            dav   = _rd(ga, R.RxDAVAddr) or 0
            rxact_now = _rd(ga, R.FEBFMActiveAD) or 0

            # ── UBT detection ───────────────────────────────────────────
            if lt is not None and lt != prev_last and lt != 0:
                lt_port = one_hot_to_port(lt)
                if lt_port is not None:
                    total_ubts += 1
                    served.add(lt_port)
                    fm_ok  = bool(rxact_now & (1 << lt_port))
                    tag    = "" if fm_ok else "⚠ no FM link"

                    if (last_ubt_port is not None
                            and not replied.get(last_ubt_port, False)):
                        gap = t_ms - last_ubt_t
                        if gap < TIMEOUT_GAP_MS:
                            violations += 1
                            if lt_port == last_ubt_port:
                                tag += f"  ⚠ SAME-PORT RE-TX gap={gap:.1f}ms"
                            else:
                                tag += (f"  ⚠ VIOLATION gap={gap:.1f}ms "
                                        f"(prev=port{last_ubt_port})")
                        else:
                            tag += f"  timeout OK {gap:.1f}ms"

                    rs_v = (_rd(ga, R.ReadyStatusAddr) or 0) & 0xFF
                    detail = (f"RS=0x{rs_v:02X}  "
                              f"RxDAV=0x{dav&0xFF:02X}  "
                              f"FMActive=0x{rxact_now&0xFF:02X}  {tag}")
                    print(f"  {t_ms:>10.1f}  {'UBT →':<22}  "
                          f"{'port '+str(lt_port):>5}  {detail}")

                    ubt_times[lt_port] = t_ms
                    replied[lt_port]   = False
                    last_ubt_port      = lt_port
                    last_ubt_t         = t_ms
            prev_last = lt

            # ── Reply detection ─────────────────────────────────────────
            new_bits = dav & ~prev_dav & 0xFF
            if new_bits:
                for p in range(8):
                    if new_bits & (1 << p):
                        total_replies += 1
                        replied[p]     = True
                        rtt            = t_ms - ubt_times.get(p, t_ms)
                        rtts[p].append(rtt)
                        print(f"  {t_ms:>10.1f}  {'← FEB reply':<22}  "
                              f"{'port '+str(p):>5}  RTT={rtt:.1f}ms")
            prev_dav = dav

            # Stop round when all active ports served and replied / timed out
            round_ports = mask_to_ports(round_mask)
            if all(p in served for p in round_ports):
                if all(
                    replied.get(p, False)
                    or (t_ms - ubt_times.get(p, 0)) > UBT_TIMEOUT_MS * 1.2
                    for p in round_ports
                ):
                    break
            time.sleep(0.005)

        # Per-round diagnostics
        for p in mask_to_ports(round_mask):
            if p not in served:
                print(f"  ⚠  Round {rnd+1}: port {p} never served")
            elif not replied.get(p, False):
                print(f"  ⚠  Round {rnd+1}: port {p} served but no reply received")

    # ── Summary ────────────────────────────────────────────────────────
    final_rxact = active_mask(ga)
    print(f"\n  {'='*60}")
    print(f"  SEQUENTIAL TEST SUMMARY  ({rounds} rounds, GA={ga})")
    print(f"  {'='*60}")
    print(f"  Active FEB ports tested: {ports_in_test}")
    print(f"  FEBFMActive at end     : 0x{final_rxact:02X}  "
          f"({mask_to_ports(final_rxact)})")
    print(f"  UBT sent   : {total_ubts}")
    print(f"  FEB replies: {total_replies}")
    print(f"  VIOLATIONS : {violations}")
    _print_rtt_table(rtts)

    passed = (violations == 0)
    print(f"\n  Overall: "
          f"{'PASS ✓' if passed else 'FAIL ✗  (' + str(violations) + ' violations)'}")
    return passed

# ---------------------------------------------------------------------------
# RTT table
# ---------------------------------------------------------------------------

def _print_rtt_table(rtts: Dict[int, List[float]]) -> None:
    if not any(rtts.values()):
        return
    print(f"\n  Per-port RTT (ms):")
    print(f"  {'Port':>4}  {'Replies':>7}  "
          f"{'Min RTT':>8}  {'Max RTT':>8}  {'Avg RTT':>8}")
    for p in range(8):
        rlist = rtts.get(p, [])
        if rlist:
            mn = min(rlist); mx = max(rlist); avg = sum(rlist)/len(rlist)
            print(f"  {p:>4}  {len(rlist):>7}  "
                  f"{mn:>8.1f}  {mx:>8.1f}  {avg:>8.1f}")
        else:
            print(f"  {p:>4}  {0:>7}  "
                  f"{'—':>8}  {'—':>8}  {'—':>8}")

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="ROC Controller_FPGA2 UBT sequential handshake test/monitor",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Active-port discovery:
  The script reads FEBFMActiveAD (0x02F) which mirrors the firmware's Rx_active
  signal.  A port is 'active' when its FM edge-detector sees transitions for
  4 consecutive 10-us windows (~40 ms after link-up).

  --mask can override this; ports with no FM link are flagged as warnings.

Modes:
  status       Print register snapshot (includes FEBFMActive bitmap)
  open         Send OPEN sequence
  close        Send CLOSE sequence
  monitor      Poll; log UBT + FEB-reply events for --duration s
  single       Test one port (auto-selects first active port by default)
  sequential   Cycle all active ports for --rounds rounds

Examples:
  python ubt_handshake_monitor.py --ga 1 --mode status
  python ubt_handshake_monitor.py --ga 1 --mode monitor --duration 30
  python ubt_handshake_monitor.py --ga 1 --mode single          # auto port
  python ubt_handshake_monitor.py --ga 1 --mode single --test-port 0
  python ubt_handshake_monitor.py --ga 1 --mode sequential      # auto ports
  python ubt_handshake_monitor.py --ga 1 --mode sequential --mask 0x0F
        """,
    )
    parser.add_argument("--ip",       default="192.168.157.97")
    parser.add_argument("--port",     type=int,                 default=5002)
    parser.add_argument("--ga",       type=int,                 default=1)
    parser.add_argument("--mode",     default="monitor",
                        choices=["status","open","close",
                                 "monitor","single","sequential"])
    parser.add_argument("--duration", type=float,               default=30.0)
    parser.add_argument("--poll-ms",  type=float,               default=5.0)
    parser.add_argument("--mask",
                        type=lambda x: None if x.lower() == "auto" else int(x, 0),
                        default=None,
                        help="8-bit hex port mask, or 'auto' (default: auto-detect "
                             "via FEBFMActiveAD)")
    parser.add_argument("--test-port", type=int, default=None,
                        help="Local port 0-7 for single-port test.  "
                             "Omit to auto-select the first active port.")
    parser.add_argument("--rounds",   type=int,                 default=2)
    parser.add_argument("--no-open",  action="store_true")
    parser.add_argument("--no-close", action="store_true")
    parser.add_argument("--debug",    action="store_true")
    args = parser.parse_args()

    uc.set_config(host=args.ip, port=args.port, debug=args.debug)

    ga   = args.ga
    mode = args.mode
    rc   = 0

    try:
        if mode == "status":
            if not args.no_open:
                open_roc(ga, args.mask or 0xFF)
            print_status(ga)

        elif mode == "open":
            rc = 0 if open_roc(ga, args.mask or 0xFF) else 1

        elif mode == "close":
            rc = 0 if close_roc(ga) else 1

        elif mode == "monitor":
            if not args.no_open:
                open_roc(ga, args.mask or 0xFF)
            viols = monitor(ga, args.duration, args.poll_ms)
            if not args.no_close:
                close_roc(ga)
            rc = 1 if viols > 0 else 0

        elif mode == "single":
            if not args.no_open:
                open_roc(ga, args.mask or 0xFF)
            passed = test_single_port(ga, args.test_port)
            if not args.no_close:
                close_roc(ga)
            rc = 0 if passed else 1

        elif mode == "sequential":
            if not args.no_open:
                open_roc(ga, args.mask or 0xFF)
            passed = test_sequential_handshake(
                ga,
                mask=args.mask,   # None → auto-discover
                rounds=args.rounds,
            )
            if not args.no_close:
                close_roc(ga)
            rc = 0 if passed else 1

    except KeyboardInterrupt:
        print("\nInterrupted.")
        rc = 1
    finally:
        try:
            uc.close()
        except Exception:
            pass

    return rc


if __name__ == "__main__":
    sys.exit(main())