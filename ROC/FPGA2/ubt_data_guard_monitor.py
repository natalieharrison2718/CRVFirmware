#!/usr/bin/env python3
"""
ubt_data_guard_monitor.py  --  ROC UBT sequential-handshake data-guard checker
================================================================================
Connects to the ROC via the uc_adapter_tcp bridge (same transport used by
ubt_handshake_monitor.py) and verifies that, after each UBT is sent to a PHY
port, the FEB actually returns data (RxDAV bit rises and PhyRxWdUsed > 0)
BEFORE the firmware sends the next UBT to any port.

Register addresses come from Proj_Defs.vhd.

Usage
-----
  python ubt_data_guard_monitor.py --ip 192.168.157.97 --port 5002 --ga 1
  python ubt_data_guard_monitor.py --ip 192.168.157.97 --port 5002 --ga 1 \
         --mode sequential --rounds 3 --duration 120

Modes
-----
  monitor     : passive poll; log every UBT and every FEB reply; flag
                any UBT that fires before the previous port replied
  sequential  : force ReadyStatus for all active ports and run N rounds,
                counting ordering violations
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
# Register addresses  (lower 10 bits, from Proj_Defs.vhd)
# ---------------------------------------------------------------------------

class R:
    CSRRegAddr          = 0x000   # Control / Status Register
    InputMaskAddr       = 0x017   # Channel mask (bit i = port i enabled)
    PhyTxCSRAddr        = 0x012   # Phy TX CSR (bit0=TxEnAck, bit1=RdEmpty)
    PhyTxCntAddr        = 0x013   # PhyTxBuff_Count [10:0]
    RxDAVAddr           = 0x016   # RX data-available bitmap (bit i = port i)
    FEBFMActiveAD       = 0x02F   # Rx_active bitmap (FM edge-detect)
    LastTxTargetAddr    = 0x049   # One-hot port tag of the last transmitted word
    TxFifoRawEmptyAddr  = 0x04C   # Raw read-side empty flag of PhyTxBuff
    AutoTxKickAddr      = 0x04D   # Write one-hot mask to force-kick AutoTx
    TxFifoResetAddr     = 0x04E   # Write any value to reset PhyTx FIFO
    ReadyStatusAddr     = 0x1A0   # ReadyStatus sticky bits (read)
    ReadyClearAddr      = 0x1A1   # Write 1-bits to clear ReadyStatus
    ReadyForceAddr      = 0x1A2   # Write 1-bits to set  ReadyStatus
    # Per-port read-side word-count of PhyRxBuff  (0x018 .. 0x01F)
    PhyRxWdUsed         = [0x018 + i for i in range(8)]
    # Per-port Ethernet Rx word count (same range, aliased for clarity)
    PhyActivityCntAddr  = [0x070 + i for i in range(8)]

CSR_OPEN  = 0x00AC   # PHY on, FMRxEn, DDRWrt_En, DDRRd_en
CSR_CLOSE = 0x000C   # FMRxEn only

# Strict-ordering flag (any UBT before previous port has replied = violation)
REPLY_REQUIRED_BEFORE_NEXT_UBT = True

# ---------------------------------------------------------------------------
# uc_adapter_tcp transport wrapper
# ---------------------------------------------------------------------------

class ROC:
    """
    Thin wrapper around uc_adapter_tcp.

    Uses the same protocol the working ubt_handshake_monitor.py uses, so
    multi-byte writes (e.g. CSR=0x00AC, Mask=0xFF) actually land correctly
    on the µC bus.  The previous raw-Telnet implementation truncated writes
    and produced bogus CSR/Mask readbacks.
    """

    def __init__(self, host: str, port: int, ga: int,
                 timeout: float = 2.0, debug: bool = False):
        self._host    = host
        self._port    = port
        self._ga      = ga
        self._timeout = timeout
        self._debug   = debug
        uc.set_config(host=host, port=port, debug=debug)

    # ---- address packing --------------------------------------------------

    def _addr16(self, addr10: int) -> int:
        return uc.compose_a16(self._ga, addr10 & 0x3FF)

    # ---- public API -------------------------------------------------------

    def read(self, addr10: int) -> Optional[int]:
        try:
            return uc.uc_read(self._addr16(addr10))
        except Exception as e:
            if self._debug:
                print(f"  [uc] read 0x{addr10:03X} failed: {e}")
            return None

    def write(self, addr10: int, value: int) -> bool:
        try:
            ok, _ = uc.uc_write(self._addr16(addr10), value & 0xFFFF)
            return bool(ok)
        except Exception as e:
            if self._debug:
                print(f"  [uc] write 0x{addr10:03X}=0x{value:04X} failed: {e}")
            return False

    def close(self) -> None:
        try:
            uc.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------

def one_hot_to_port(val: Optional[int]) -> Optional[int]:
    """Convert a one-hot byte to a port index, or None if not one-hot."""
    if val is None or val == 0:
        return None
    v = val & 0xFF
    if v & (v - 1):        # more than one bit set
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
# Open / Close sequences
# ---------------------------------------------------------------------------

def open_roc(roc: ROC, mask: int = 0xFF, settle_s: float = 0.2) -> bool:
    """
    Send the OPEN sequence.

    settle_s gives the FM-edge-detector time to assert Rx_active on ports
    with live FM links before we sample FEBFMActive.  The detector requires
    4 consecutive 10 µs windows of activity (~40 µs) plus FM-link sync
    time, so 200 ms is comfortably enough.
    """
    print(f"\n  → OPEN ROC  (mask=0x{mask:02X})")
    roc.write(R.InputMaskAddr, mask)
    time.sleep(0.01)
    roc.write(R.CSRRegAddr, CSR_OPEN)
    time.sleep(settle_s)          # wait for FM links and Rx_active to assert
    roc.write(R.ReadyClearAddr, 0xFF)
    time.sleep(0.02)

    csr   = roc.read(R.CSRRegAddr)
    mkr   = roc.read(R.InputMaskAddr)
    rs    = roc.read(R.ReadyStatusAddr)
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
        print(f"  ✓  InputMask  = 0x{(mkr or 0) & 0xFF:02X}")
    print(f"     ReadyStatus= 0x{(rs  or 0) & 0xFF:02X}")
    active = mask_to_ports((rxact or 0) & 0xFF)
    print(f"     FEBFMActive= 0x{(rxact or 0) & 0xFF:02X}  active: {active}")
    if not active:
        print(f"  ⚠  No FM links detected — all ports will time out")
    return ok


def close_roc(roc: ROC) -> bool:
    print(f"\n  → CLOSE ROC")
    roc.write(R.ReadyClearAddr, 0xFF)
    time.sleep(0.01)
    roc.write(R.InputMaskAddr, 0x00)
    time.sleep(0.01)
    roc.write(R.CSRRegAddr, CSR_CLOSE)
    time.sleep(0.02)
    csr  = roc.read(R.CSRRegAddr)
    mkr  = roc.read(R.InputMaskAddr)
    rs   = roc.read(R.ReadyStatusAddr)
    print(f"  CSR=0x{(csr or 0):04X}  "
          f"Mask=0x{(mkr or 0)&0xFF:02X}  "
          f"RS=0x{(rs or 0)&0xFF:02X}")
    return ((csr or 0) & 0xA0) == 0 and (mkr or 0) == 0

# ---------------------------------------------------------------------------
# Status snapshot
# ---------------------------------------------------------------------------

def print_status(roc: ROC) -> None:
    print(f"\n{'='*62}")
    print(f"  ROC register snapshot")
    print(f"{'='*62}")
    csr   = roc.read(R.CSRRegAddr)
    mask  = roc.read(R.InputMaskAddr)
    rs    = roc.read(R.ReadyStatusAddr)
    dav   = roc.read(R.RxDAVAddr)
    rxact = roc.read(R.FEBFMActiveAD)
    txcsr = roc.read(R.PhyTxCSRAddr)
    txcnt = roc.read(R.PhyTxCntAddr)
    last  = roc.read(R.LastTxTargetAddr)

    def _h(v, w=4): return f"0x{v:0{w}X}" if v is not None else "N/A"
    def _b(v, w=2): return f"0x{v:0{w}X}" if v is not None else "N/A"

    print(f"  CSR            : {_h(csr)}  ({_decode_csr(csr or 0)})")
    print(f"  InputMask      : {_b(mask)}")
    print(f"  ReadyStatus    : {_b(rs)}")
    print(f"  RxDAV          : {_b(dav)}")
    print(f"  FEBFMActive    : {_b(rxact)}  "
          f"active: {mask_to_ports((rxact or 0) & 0xFF)}")
    print(f"  PhyTxCSR       : {_h(txcsr)}")
    print(f"  PhyTxBuff_Count: {(txcnt or 0) & 0x7FF}")
    print(f"  LastTxTarget   : {_b(last)}  "
          f"(port {one_hot_to_port(last)})")

    print(f"\n  {'Port':>4}  {'RxWdUsed':>9}  {'DAV':>4}  {'FMActive':>9}")
    for i in range(8):
        wc  = roc.read(R.PhyRxWdUsed[i])
        dav_b  = ((dav   or 0) >> i) & 1
        fm_b   = ((rxact or 0) >> i) & 1
        print(f"  {i:>4}  {(wc or 0) & 0xFFF:>9}  {dav_b:>4}  {fm_b:>9}")
    print()

# ---------------------------------------------------------------------------
# Core data-guard check helpers
# ---------------------------------------------------------------------------

def _feb_has_data(roc: ROC, port: int) -> bool:
    """
    Return True if the FEB on *port* has deposited at least one word into
    the ROC's PhyRxBuff since the last UBT.

    Two independent signals must agree:
      1. RxDAVAddr bit[port]     -- data-available latch
      2. PhyRxWdUsed[port] > 0   -- FIFO read-side word count
    Either alone is sufficient to declare 'data arrived'.
    """
    dav  = roc.read(R.RxDAVAddr)
    wc   = roc.read(R.PhyRxWdUsed[port])
    dav_set = (dav is not None) and bool(dav & (1 << port))
    wc_set  = (wc  is not None) and (wc & 0xFFF) > 0
    return dav_set or wc_set


def _poll_for_feb_data(
    roc:       ROC,
    port:      int,
    deadline:  float,
    poll_s:    float = 0.005,
) -> Tuple[bool, float]:
    """
    Poll until FEB data arrives on *port* or *deadline* (time.time()) is reached.
    Returns (data_arrived: bool, latency_ms: float).
    """
    t0 = time.time()
    while time.time() < deadline:
        if _feb_has_data(roc, port):
            return True, (time.time() - t0) * 1000.0
        time.sleep(poll_s)
    return False, (time.time() - t0) * 1000.0

# ---------------------------------------------------------------------------
# Monitor mode  (passive)
# ---------------------------------------------------------------------------

def monitor(
    roc:         ROC,
    duration_s:  float,
    poll_ms:     float,
) -> int:
    """
    Passively monitor for UBT transmissions and FEB replies.
    A VIOLATION is recorded whenever LastTxTarget changes to a new port
    *before* we have seen FEB data arrive for the previous port.

    Returns the total violation count.
    """
    print(f"\n  Passive monitor for {duration_s:.0f}s  "
          f"(poll {poll_ms:.0f} ms)")
    hdr = (f"  {'Time(ms)':>10}  {'Event':<26}  {'Port':>5}  "
           f"{'DAV':>5}  {'WdUsed':>7}  Note")
    print(f"\n{hdr}")
    print(f"  {'':─>10}  {'':─>26}  {'':─>5}  "
          f"{'':─>5}  {'':─>7}  {'':─>36}")

    t0         = time.time()
    poll_s     = poll_ms / 1000.0
    violations = 0
    ubt_count  = 0
    reply_count= 0

    # Tracking state
    prev_last  : Optional[int]       = None
    pending_port: Optional[int]      = None   # port waiting for FEB data
    pending_ubt_t: float             = 0.0
    prev_dav   : int                 = 0
    rtts       : Dict[int, List[float]] = {i: [] for i in range(8)}

    while (time.time() - t0) < duration_s:
        now   = time.time()
        t_ms  = (now - t0) * 1000.0

        last   = roc.read(R.LastTxTargetAddr)
        dav    = roc.read(R.RxDAVAddr) or 0
        rs     = roc.read(R.ReadyStatusAddr) or 0
        rxact  = roc.read(R.FEBFMActiveAD) or 0

        # ── Detect new UBT ────────────────────────────────────────────
        if last is not None and last != prev_last and last != 0:
            new_port = one_hot_to_port(last)
            if new_port is not None:
                ubt_count += 1
                wc = (roc.read(R.PhyRxWdUsed[new_port]) or 0) & 0xFFF
                fm = bool(rxact & (1 << new_port))
                note = "" if fm else "⚠ no FM link"

                # Check: was the previous port's reply received?
                if pending_port is not None:
                    if not _feb_has_data(roc, pending_port):
                        violations += 1
                        gap = t_ms - pending_ubt_t
                        note += (f"  ⚠ VIOLATION: UBT→port{new_port} before "
                                 f"FEB reply from port{pending_port} "
                                 f"(gap={gap:.1f}ms)")

                print(f"  {t_ms:>10.1f}  {'UBT →':<26}  "
                      f"{'port '+str(new_port):>5}  "
                      f"{(dav>>new_port)&1:>5}  {wc:>7}  "
                      f"RS=0x{rs&0xFF:02X}  {note}")

                pending_port  = new_port
                pending_ubt_t = t_ms

        prev_last = last

        # ── Detect FEB reply (RxDAV rising edge) ──────────────────────
        new_dav = dav & ~prev_dav & 0xFF
        if new_dav:
            for p in range(8):
                if new_dav & (1 << p):
                    reply_count += 1
                    wc  = (roc.read(R.PhyRxWdUsed[p]) or 0) & 0xFFF
                    rtt = t_ms - (pending_ubt_t
                                  if pending_port == p else t_ms)
                    rtts[p].append(rtt)
                    print(f"  {t_ms:>10.1f}  {'← FEB reply':<26}  "
                          f"{'port '+str(p):>5}  "
                          f"{1:>5}  {wc:>7}  RTT={rtt:.1f}ms")
        prev_dav = dav

        time.sleep(poll_s)

    # ── Summary ────────────────────────────────────────────────────────
    rxact_end = roc.read(R.FEBFMActiveAD) or 0
    print(f"\n  {'='*62}")
    print(f"  MONITOR SUMMARY  ({duration_s:.0f}s)")
    print(f"  {'='*62}")
    print(f"  Active FEB ports (end) : {mask_to_ports(rxact_end)}")
    print(f"  UBTs observed          : {ubt_count}")
    print(f"  FEB replies (DAV edge) : {reply_count}")
    print(f"  VIOLATIONS             : {violations}")
    _print_rtt_table(rtts)
    return violations

# ---------------------------------------------------------------------------
# Sequential data-guard test  (active)
# ---------------------------------------------------------------------------

def test_sequential(
    roc:       ROC,
    mask:      Optional[int],
    rounds:    int,
    reply_timeout_s: float = 0.5,
    poll_ms:   float = 5.0,
) -> bool:
    """
    Force ReadyStatus for all active ports, then wait for the firmware's
    AutoTx FSM to send UBTs and collect FEB replies.

    For each UBT detected:
      1. Record which port was targeted and the RxDAV + PhyRxWdUsed state
         at the moment of the UBT.
      2. Wait up to *reply_timeout_s* for FEB data to arrive on that port.
      3. Only after data has arrived (or timeout) allow the next UBT to
         be 'expected'.
      4. If the firmware fires a new UBT to any port while we are still
         waiting for the previous reply → VIOLATION.

    Returns True if no violations.
    """
    # Determine active ports
    rxact = roc.read(R.FEBFMActiveAD) or 0
    if mask is None:
        eff_mask = rxact & 0xFF
        print(f"\n  Auto-detected active ports: "
              f"{mask_to_ports(eff_mask)}  "
              f"(FEBFMActive=0x{eff_mask:02X})")
    else:
        eff_mask = mask & 0xFF
        dead = [p for p in mask_to_ports(eff_mask) if not (rxact & (1 << p))]
        if dead:
            print(f"  ⚠  Ports {dead} in mask have no FM link — "
                  f"will always time out")

    if eff_mask == 0:
        print(f"  ✗  No active ports — aborting.")
        return False

    ports = mask_to_ports(eff_mask)
    print(f"\n  ── Sequential data-guard test  "
          f"ports={ports}  rounds={rounds} ──")
    hdr = (f"  {'Time(ms)':>10}  {'Event':<30}  {'Port':>5}  "
           f"{'DAVbit':>7}  {'WdUsed':>7}  Note")
    print(f"\n{hdr}")
    print(f"  {'':─>10}  {'':─>30}  {'':─>5}  "
          f"{'':─>7}  {'':─>7}  {'':─>36}")

    poll_s     = poll_ms / 1000.0
    violations = 0
    total_ubts = 0
    total_ok   = 0
    total_tout = 0
    rtts: Dict[int, List[float]] = {i: [] for i in range(8)}

    # Clear any old state
    roc.write(R.ReadyClearAddr, 0xFF)
    time.sleep(0.02)

    t_origin = time.time()

    for rnd in range(rounds):
        print(f"\n  ── Round {rnd+1}/{rounds} ──")

        # Re-sample active ports
        live_mask = (roc.read(R.FEBFMActiveAD) or 0) & eff_mask
        if live_mask == 0:
            print(f"  ⚠  No live ports at round start — skipping.")
            continue
        dropped = mask_to_ports(eff_mask & ~live_mask)
        if dropped:
            print(f"  ⚠  Ports {dropped} lost FM link, skipping this round")

        # Assert ReadyStatus for all live ports this round
        roc.write(R.ReadyForceAddr, live_mask)
        time.sleep(0.01)

        # Now watch: for each UBT that fires we must see FEB data before
        # the *next* UBT fires.
        round_ports  = set(mask_to_ports(live_mask))
        served       : set            = set()
        replied_ok   : set            = set()
        prev_last    : Optional[int]  = None
        pending_port : Optional[int]  = None
        pending_t    : float          = 0.0
        prev_dav     : int            = 0

        round_deadline = time.time() + max(30.0, len(round_ports) * 15.0)

        while time.time() < round_deadline:
            t_ms = (time.time() - t_origin) * 1000.0

            last   = roc.read(R.LastTxTargetAddr)
            dav    = roc.read(R.RxDAVAddr) or 0
            rxact_now = roc.read(R.FEBFMActiveAD) or 0

            # ── New UBT? ──────────────────────────────────────────────
            if last is not None and last != prev_last and last != 0:
                new_port = one_hot_to_port(last)
                if new_port is not None:
                    total_ubts += 1
                    served.add(new_port)
                    wc    = (roc.read(R.PhyRxWdUsed[new_port]) or 0) & 0xFFF
                    dav_b = (dav >> new_port) & 1
                    fm    = bool(rxact_now & (1 << new_port))
                    note  = "" if fm else "⚠ no FM link"

                    # ── DATA-GUARD CHECK ──────────────────────────────
                    # If there was a previous pending port, verify its
                    # FEB data arrived BEFORE this UBT fired.
                    if pending_port is not None:
                        data_present = _feb_has_data(roc, pending_port)
                        gap = t_ms - pending_t
                        if not data_present:
                            violations += 1
                            note += (f"  ⚠ VIOLATION — no FEB data from "
                                     f"port{pending_port} before UBT "
                                     f"(gap={gap:.1f}ms)")
                        else:
                            note += f"  ✓ prev port{pending_port} replied"

                    print(f"  {t_ms:>10.1f}  {'UBT →':<30}  "
                          f"{'port '+str(new_port):>5}  "
                          f"{dav_b:>7}  {wc:>7}  {note}")

                    pending_port = new_port
                    pending_t    = t_ms
            prev_last = last

            # ── FEB reply (RxDAV rising edge) ────────────────────────
            new_dav = dav & ~prev_dav & 0xFF
            if new_dav:
                for p in range(8):
                    if new_dav & (1 << p):
                        wc  = (roc.read(R.PhyRxWdUsed[p]) or 0) & 0xFFF
                        rtt = t_ms - (pending_t if pending_port == p else t_ms)
                        rtts[p].append(rtt)
                        replied_ok.add(p)
                        total_ok += 1
                        print(f"  {t_ms:>10.1f}  {'← FEB data received':<30}  "
                              f"{'port '+str(p):>5}  "
                              f"{1:>7}  {wc:>7}  RTT={rtt:.1f}ms")
            prev_dav = dav

            # ── Round complete? ───────────────────────────────────────
            # All ports served and either replied or waited long enough
            all_served = round_ports.issubset(served)
            if all_served:
                all_done = all(
                    p in replied_ok
                    or (t_ms - (pending_t if pending_port == p else 0))
                       > reply_timeout_s * 1000.0
                    for p in round_ports
                )
                if all_done:
                    break

            time.sleep(poll_s)

        # Per-round diagnostics
        for p in round_ports:
            if p not in served:
                print(f"  ⚠  Round {rnd+1}: port {p} was never sent a UBT")
                total_tout += 1
            elif p not in replied_ok:
                print(f"  ⚠  Round {rnd+1}: port {p} got UBT but no FEB data")
                total_tout += 1

    # ── Final summary ─────────────────────────────────────────────────
    final_rxact = roc.read(R.FEBFMActiveAD) or 0
    print(f"\n  {'='*62}")
    print(f"  DATA-GUARD TEST SUMMARY  ({rounds} rounds)")
    print(f"  {'='*62}")
    print(f"  Ports tested           : {ports}")
    print(f"  FEBFMActive at end     : 0x{final_rxact:02X}  "
          f"({mask_to_ports(final_rxact)})")
    print(f"  Total UBTs observed    : {total_ubts}")
    print(f"  FEB data received      : {total_ok}")
    print(f"  Timeouts / no-reply    : {total_tout}")
    print(f"  VIOLATIONS             : {violations}")
    _print_rtt_table(rtts)

    passed = violations == 0
    print(f"\n  Overall: "
          f"{'PASS ✓' if passed else 'FAIL ✗  (' + str(violations) + ' violation(s))'}")
    return passed

# ---------------------------------------------------------------------------
# RTT table
# ---------------------------------------------------------------------------

def _print_rtt_table(rtts: Dict[int, List[float]]) -> None:
    if not any(rtts.values()):
        return
    print(f"\n  Per-port reply RTT (ms):")
    print(f"  {'Port':>4}  {'N':>5}  {'Min':>8}  {'Max':>8}  {'Avg':>8}")
    for p in range(8):
        lst = rtts.get(p, [])
        if lst:
            print(f"  {p:>4}  {len(lst):>5}  "
                  f"{min(lst):>8.1f}  {max(lst):>8.1f}  "
                  f"{sum(lst)/len(lst):>8.1f}")
        else:
            print(f"  {p:>4}  {0:>5}  {'—':>8}  {'—':>8}  {'—':>8}")

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="ROC UBT data-guard monitor (uc_adapter_tcp transport)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Registers used (from Proj_Defs.vhd):
  RxDAVAddr       0x016  -- bit i set when PhyRxBuff for port i is non-empty
  PhyRxWdUsed[i]  0x018+i-- 12-bit read-side word count of PhyRxBuff[i]
  LastTxTargetAddr 0x049 -- one-hot port tag latched at each PhyTxBuff read
  FEBFMActiveAD   0x02F  -- Rx_active bitmap (FM edge-detect, 1 bit/port)
  ReadyStatusAddr 0x1A0  -- sticky ready-status bits
  ReadyClearAddr  0x1A1  -- write 1s to clear ReadyStatus bits
  ReadyForceAddr  0x1A2  -- write 1s to set  ReadyStatus bits

What counts as a VIOLATION:
  The firmware's AutoTx FSM sends a UBT to port N.  Before the FEB on
  port N has deposited any data (RxDAV[N]=0 AND PhyRxWdUsed[N]=0), the
  firmware sends another UBT to any port.

Examples:
  python ubt_data_guard_monitor.py --ip 192.168.157.97 --ga 1
  python ubt_data_guard_monitor.py --ip 192.168.157.97 --ga 1 --duration 60
  python ubt_data_guard_monitor.py --ip 192.168.157.97 --ga 1 \\
         --mode sequential --rounds 5
  python ubt_data_guard_monitor.py --ip 192.168.157.97 --ga 1 \\
         --mode status
        """,
    )
    parser.add_argument("--ip",       default="192.168.157.97",
                        help="ROC IP address")
    parser.add_argument("--port",     type=int, default=5002,
                        help="Bridge TCP port")
    parser.add_argument("--ga",       type=int, default=1,
                        help="Geographic address (0-3)")
    parser.add_argument("--mode",     default="monitor",
                        choices=["status", "open", "close",
                                 "monitor", "sequential"],
                        help="Operation mode")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Monitor duration in seconds")
    parser.add_argument("--poll-ms",  type=float, default=5.0,
                        help="Poll interval in milliseconds")
    parser.add_argument("--mask",
                        type=lambda x: None if x.lower() == "auto"
                                            else int(x, 0),
                        default=None,
                        help="8-bit hex port mask, or 'auto' (default: "
                             "auto-detect via FEBFMActiveAD)")
    parser.add_argument("--rounds",   type=int, default=2,
                        help="Number of rounds for sequential mode")
    parser.add_argument("--reply-timeout", type=float, default=0.5,
                        help="Seconds to wait for FEB reply before "
                             "declaring timeout (sequential mode)")
    parser.add_argument("--open-settle", type=float, default=0.2,
                        help="Seconds to wait after CSR write for FM "
                             "links / Rx_active to assert (default 0.2)")
    parser.add_argument("--no-open",  action="store_true",
                        help="Skip OPEN sequence")
    parser.add_argument("--no-close", action="store_true",
                        help="Skip CLOSE sequence")
    parser.add_argument("--debug",    action="store_true",
                        help="Print raw bridge I/O")
    args = parser.parse_args()

    roc = ROC(args.ip, args.port, args.ga, debug=args.debug)
    rc  = 0

    try:
        mode = args.mode
        eff_mask = args.mask if args.mask is not None else 0xFF

        if mode == "status":
            if not args.no_open:
                open_roc(roc, eff_mask, settle_s=args.open_settle)
            print_status(roc)

        elif mode == "open":
            rc = 0 if open_roc(roc, eff_mask, settle_s=args.open_settle) else 1

        elif mode == "close":
            rc = 0 if close_roc(roc) else 1

        elif mode == "monitor":
            if not args.no_open:
                open_roc(roc, eff_mask, settle_s=args.open_settle)
            viols = monitor(roc, args.duration, args.poll_ms)
            if not args.no_close:
                close_roc(roc)
            rc = 1 if viols > 0 else 0

        elif mode == "sequential":
            if not args.no_open:
                open_roc(roc, eff_mask, settle_s=args.open_settle)
            passed = test_sequential(
                roc,
                mask=args.mask,
                rounds=args.rounds,
                reply_timeout_s=args.reply_timeout,
                poll_ms=args.poll_ms,
            )
            if not args.no_close:
                close_roc(roc)
            rc = 0 if passed else 1

    except KeyboardInterrupt:
        print("\n  Interrupted by user.")
        rc = 1
    finally:
        roc.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())