#!/usr/bin/env python3
"""
ubt_sequence_monitor.py
───────────────────────────────────────────────────────────────────────
Verify that Controller_FPGA2 AutoTx sends UBT requests ONE AT A TIME
and waits for the FEB to fill the ROC RX buffer before moving to the
next port.

Fixes vs. original version
───────────────────────────
  • Default port changed to 5001, default GA changed to 1
  • TCP read timeout reduced to 0.5 s (was 3.0 s → caused ~3 s polls)
  • CSR_OPEN corrected to 0x00AC (bit7 = DDRRd_en was missing)
  • read_snap() no longer reads 8× PHY_RX_WD_USED on every hot-path poll
    (saves 8 TCP round-trips per snap; those are read only on full snaps)
  • Re-arm throttled to at most once per REARM_INTERVAL_S (was every poll)
  • do_open() now prints Rx_active and FEB-FM-active so link health is
    visible before the monitor loop starts
  • Added --rearm-interval and --snap-full-every CLI options

Usage
─────
  python ubt_sequence_monitor.py                         # GA=1, all ports, 3 rounds
  python ubt_sequence_monitor.py --ga 2 --rounds 5
  python ubt_sequence_monitor.py --ga 1 --extra-ga 2 3  # open all three boards
  python ubt_sequence_monitor.py --ports 0x03 --rounds 2 --output r.json
"""

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Dict, List, Optional

try:
    import uc_adapter_tcp as uc
except ImportError as exc:
    print(f"ERROR: could not import uc_adapter_tcp: {exc}")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# Register map  (addr10 values)
#
# Address derivation from the VHDL iCD mux and the reference open/close cmds:
#   WR 5A2 FF  → raw_addr = 0x5A2, GA bits = (0x5A2 >> 10) & 3 = 1
#                addr10 = 0x5A2 & 0x3FF = 0x1A2  → ReadyForceAddr
#   RD 5A0     → GA=1, addr10=0x1A0 → ReadyStatusAddr
# ─────────────────────────────────────────────────────────────────────────────
class R:
    CSR             = 0x000   # bit2=PHY_on, bit3=FMRxEn, bit5=DDRWrtEn, bit7=DDRRdEn
    INPUT_MASK      = 0x017   # Port mask (8-bit)
    RX_DAV          = 0x016   # ~PhyRxBuff_Empty  (bit=1 means data present)
    PHY_TX_CSR      = 0x012   # bit0=TxEnAck, bit1=RdEmpty
    PHY_TX_CNT      = 0x013   # TX FIFO write-domain word count (11-bit)
    LAST_TX_TARGET  = 0x049   # Sticky last-TX target latch (one-hot, 8-bit)
    TX_CURR_TARGET  = 0x04A   # Current TX target (one-hot, 8-bit)
    READY_STATUS    = 0x1A0   # ReadyStatus (read)
    READY_CLEAR     = 0x1A1   # ReadyClear  (write-only)
    READY_FORCE     = 0x1A2   # ReadyForce  (write-only)
    TX_FIFO_RST     = 0x04E   # TX FIFO reset pulse
    FEB_FM_ACTIVE   = 0x02F   # Active FEB FM channels  (Rx_active)
    DEBUG_VERSION   = 0x099   # Firmware version tag (should read 0x0011)
    PHY_RX_WD_USED  = [0x018 + i for i in range(8)]   # per-port RX word count


# CSR bit layout
_CSR_DDR_RD   = (1 << 7)   # DDRRd_en
_CSR_DDR_WRT  = (1 << 5)   # DDRWrt_En
_CSR_FMRXEN   = (1 << 3)   # FMRxEn
_CSR_PHY_ON   = (1 << 2)   # ~PhyPDn

# OPEN:  DDRRd_en=1, DDRWrt_En=1, FMRxEn=1, PHY_on
# 0x80 | 0x20 | 0x08 | 0x04 = 0xAC
_CSR_OPEN  = _CSR_DDR_RD | _CSR_DDR_WRT | _CSR_FMRXEN | _CSR_PHY_ON  # 0xAC

# CLOSE: DDRRd_en=0, DDRWrt_En=0, FMRxEn=1, PHY_on
_CSR_CLOSE = _CSR_FMRXEN | _CSR_PHY_ON   # 0x0C


# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Snap:
    t:            float  = 0.0
    ready:        int    = 0
    tx_cnt:       int    = 0
    tx_ack:       int    = 0
    tx_rd_empty:  int    = 1
    last_target:  int    = 0
    curr_target:  int    = 0
    rx_dav:       int    = 0
    rx_wds:       List   = field(default_factory=lambda: [0] * 8)


@dataclass
class HandshakeEvent:
    port:           int   = 0
    ubt_sent_ms:    float = 0.0
    feb_replied_ms: float = 0.0
    round_trip_ms:  float = 0.0
    violation:      bool  = False
    violation_msg:  str   = ""


# ─────────────────────────────────────────────────────────────────────────────
# Low-level helpers
# ─────────────────────────────────────────────────────────────────────────────
def _rd(ga: int, addr10: int) -> Optional[int]:
    try:
        return uc.uc_read(uc.compose_a16(ga, addr10))
    except Exception:
        return None


def _wr(ga: int, addr10: int, val: int) -> bool:
    try:
        ok, _ = uc.uc_write(uc.compose_a16(ga, addr10), val)
        return ok
    except Exception:
        return False


def _popcount(x: int) -> int:
    return bin(x).count('1')


def _lowest_bit(x: int) -> int:
    if x == 0:
        return -1
    return (x & -x).bit_length() - 1


# ─────────────────────────────────────────────────────────────────────────────
# Snapshot  (fast path omits per-port RX word counts)
# ─────────────────────────────────────────────────────────────────────────────
def read_snap(ga: int, full: bool = False) -> Snap:
    """
    Fast snap: 6 TCP reads (ReadyStatus, PhyTxCSR, PhyTxCnt,
               LastTxTarget, TxCurrTarget, RxDAV).
    Full snap: additionally reads 8× PhyRxWdUsed (14 reads total).
    """
    s = Snap(t=time.time())

    r  = _rd(ga, R.READY_STATUS);   s.ready      = (r  or 0) & 0xFF
    c  = _rd(ga, R.PHY_TX_CSR);     s.tx_ack     = (c  or 0) & 1
    s.tx_rd_empty = ((c or 0) >> 1) & 1
    n  = _rd(ga, R.PHY_TX_CNT);     s.tx_cnt     = (n  or 0) & 0x7FF
    lt = _rd(ga, R.LAST_TX_TARGET); s.last_target = (lt or 0) & 0xFF
    ct = _rd(ga, R.TX_CURR_TARGET); s.curr_target = (ct or 0) & 0xFF
    dv = _rd(ga, R.RX_DAV);         s.rx_dav     = (dv or 0) & 0xFF

    if full:
        wds = []
        for i in range(8):
            v = _rd(ga, R.PHY_RX_WD_USED[i])
            wds.append((v or 0) & 0x0FFF)
        s.rx_wds = wds

    return s


# ─────────────────────────────────────────────────────────────────────────────
# OPEN / CLOSE
# ─────────────────────────────────────────────────────────────────────────────
def do_open(primary_ga: int,
            extra_gas: List[int],
            port_mask: int) -> bool:
    """
    Replicate the reference OPEN sequence.
    Returns False if Rx_active is 0 (FM links may be down).
    """
    all_gas = [primary_ga] + extra_gas
    print(f"\n[OPEN]  GAs={all_gas}  ports=0x{port_mask:02X}  CSR=0x{_CSR_OPEN:04X}")

    for ga in all_gas:
        _wr(ga, R.INPUT_MASK, port_mask)
        _wr(ga, R.CSR,        _CSR_OPEN)
    time.sleep(0.05)

    # Arm ReadyStatus on all GAs so AutoTx fires immediately
    for ga in all_gas:
        _wr(ga, R.READY_FORCE, port_mask)
    time.sleep(0.02)

    # ── verify primary GA ────────────────────────────────────────────────
    csr      = _rd(primary_ga, R.CSR)
    mask     = _rd(primary_ga, R.INPUT_MASK)
    rs       = _rd(primary_ga, R.READY_STATUS)
    rx_act   = _rd(primary_ga, R.FEB_FM_ACTIVE)
    rx_act_v = (rx_act or 0) & 0xFF

    print(f"         GA={primary_ga}: "
          f"CSR=0x{(csr or 0):04X}  "
          f"mask=0x{(mask or 0) & 0xFF:02X}  "
          f"ReadyStatus=0x{(rs or 0) & 0xFF:02X}  "
          f"Rx_active=0x{rx_act_v:02X}")

    # ── DDRRd_en check ───────────────────────────────────────────────────
    if csr is not None and not (csr & _CSR_DDR_RD):
        print("  ⚠  WARNING: DDRRd_en (bit7) is NOT set in CSR readback — "
              "P4 ReadyStatus path will not fire")

    # ── FM link health check ─────────────────────────────────────────────
    if rx_act_v == 0:
        print("  ⚠  WARNING: Rx_active = 0x00 — no FEB FM links are active.\n"
              "     FEB reply frames will be silently discarded by PhyRx_Proc.\n"
              "     Ensure FEBs are powered and FMRxEn has been asserted long\n"
              "     enough for the transition counter to saturate (≥15 edges).")
        return False

    active_ports = [i for i in range(8) if (rx_act_v >> i) & 1]
    inactive_ports = [i for i in range(8)
                      if (port_mask >> i) & 1 and not (rx_act_v >> i) & 1]
    print(f"         FM active ports  : {active_ports}")
    if inactive_ports:
        print(f"  ⚠  Masked but FM-inactive: {inactive_ports} "
              f"— replies from these ports will be dropped")
    return True


def do_close(primary_ga: int, extra_gas: List[int]) -> None:
    all_gas = [primary_ga] + extra_gas
    print(f"\n[CLOSE] GAs={all_gas}")
    for ga in all_gas:
        _wr(ga, R.READY_CLEAR, 0xFF)
    time.sleep(0.01)
    for ga in all_gas:
        _wr(ga, R.CSR,        _CSR_CLOSE)
        _wr(ga, R.INPUT_MASK, 0x00)
    time.sleep(0.02)

    csr  = _rd(primary_ga, R.CSR)
    mask = _rd(primary_ga, R.INPUT_MASK)
    rs   = _rd(primary_ga, R.READY_STATUS)
    print(f"         GA={primary_ga}: "
          f"CSR=0x{(csr or 0):04X}  "
          f"mask=0x{(mask or 0) & 0xFF:02X}  "
          f"ReadyStatus=0x{(rs or 0) & 0xFF:02X}")


def reset_tx_fifo(ga: int) -> None:
    _wr(ga, R.TX_FIFO_RST, 0x0001)
    time.sleep(0.025)
    _wr(ga, R.TX_FIFO_RST, 0x0000)
    time.sleep(0.06)


def wait_tx_idle(ga: int, timeout_s: float = 5.0) -> bool:
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        csr = _rd(ga, R.PHY_TX_CSR)
        if csr is not None:
            if (csr >> 1) & 1 and not (csr & 1):
                return True
        time.sleep(0.015)
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Core monitor
# ─────────────────────────────────────────────────────────────────────────────
def monitor(ga: int,
            port_mask: int,
            rounds: int,
            poll_s: float = 0.008,
            rearm_interval_s: float = 3.0,
            snap_full_every: int = 50) -> Dict:
    """
    Watch AutoTx cycle through the enabled ports.

    Parameters
    ──────────
    ga                 : primary board to monitor
    port_mask          : one-hot mask of ports to observe
    rounds             : number of full port sweeps to observe
    poll_s             : poll interval (seconds)
    rearm_interval_s   : minimum seconds between ReadyForce re-arms
    snap_full_every    : read per-port RX word counts every N snaps
    """
    num_ports     = _popcount(port_mask)
    target_events = rounds * num_ports
    events:  List[HandshakeEvent] = []
    history: List[dict]           = []
    violations: List[str]         = []
    t0 = time.time()

    # ── table header ──────────────────────────────────────────────────────
    hdr = (f"  {'ms':>10}  {'ReadyStat':>10}  {'LastTgt':>8}"
           f"  {'TxCnt':>6}  {'RxDAV':>7}  Event")
    sep = "  " + "─" * (len(hdr) - 2)
    print(f"\n{sep}\n{hdr}\n{sep}")

    def ms_now(t: float) -> float:
        return (t - t0) * 1_000

    def log(snap: Snap, note: str = "") -> None:
        print(f"  {ms_now(snap.t):>10.1f}  "
              f"0x{snap.ready:02X}{'':>8}  "
              f"0x{snap.last_target:02X}{'':>6}  "
              f"{snap.tx_cnt:>6}  "
              f"0x{snap.rx_dav:02X}{'':>5}  "
              f"{note}")

    # ── state ─────────────────────────────────────────────────────────────
    prev           = read_snap(ga, full=True)
    prev_target    = 0
    in_flight_port = -1
    in_flight_ms   = 0.0
    prev_rx_dav    = 0
    last_rearm_t   = time.time()   # throttle re-arms
    rearm_count    = 0
    snap_idx       = 0
    log(prev, "── monitor start ──")

    timeout_s = max(target_events * 2.0, 90.0)
    deadline  = time.time() + timeout_s

    while time.time() < deadline and len(events) < target_events:
        time.sleep(poll_s)
        snap_idx += 1
        s = read_snap(ga, full=(snap_idx % snap_full_every == 0))
        elapsed_ms = ms_now(s.t)

        # ── new UBT target ────────────────────────────────────────────────
        if s.last_target != 0 and s.last_target != prev_target:
            new_port = _lowest_bit(s.last_target)

            violation     = False
            violation_msg = ""
            if in_flight_port >= 0:
                if not ((prev_rx_dav >> in_flight_port) & 1):
                    violation_msg = (
                        f"NEW UBT to port {new_port} while port {in_flight_port} "
                        f"has NOT yet received a FEB reply  (t={elapsed_ms:.1f} ms)"
                    )
                    violations.append(violation_msg)
                    log(s, f"⚠  VIOLATION — {violation_msg}")
                    violation = True
                else:
                    log(s, f"→  UBT sent to port {new_port} "
                            f"(port {in_flight_port} already replied ✓)")
            else:
                log(s, f"→  UBT sent to port {new_port}")

            ev = HandshakeEvent(
                port          = new_port,
                ubt_sent_ms   = elapsed_ms,
                violation     = violation,
                violation_msg = violation_msg,
            )
            events.append(ev)
            in_flight_port = new_port
            in_flight_ms   = elapsed_ms
            prev_target    = s.last_target

        # ── FEB reply (RX DAV rising edge) ────────────────────────────────
        new_dav = s.rx_dav & ~prev_rx_dav
        if new_dav:
            for i in range(8):
                if (new_dav >> i) & 1:
                    rtt  = elapsed_ms - in_flight_ms if in_flight_port == i else None
                    note = f"←  FEB replied on port {i}"
                    if rtt is not None:
                        note += f"  RTT={rtt:.1f} ms"
                        for ev in reversed(events):
                            if ev.port == i and ev.feb_replied_ms == 0:
                                ev.feb_replied_ms = elapsed_ms
                                ev.round_trip_ms  = rtt
                                break
                        if in_flight_port == i:
                            in_flight_port = -1
                    else:
                        note += f"  (unexpected; in-flight={in_flight_port})"
                    log(s, note)

        prev_rx_dav = s.rx_dav

        # ── FEB silence warning ───────────────────────────────────────────
        # If a UBT was sent more than 500 ms ago and RX DAV still has not
        # risen for that port, warn once.
        if (in_flight_port >= 0
                and elapsed_ms - in_flight_ms > 500
                and not ((s.rx_dav >> in_flight_port) & 1)):
            log(s, f"⚠  FEB silent for {elapsed_ms - in_flight_ms:.0f} ms "
                   f"on port {in_flight_port} — "
                   f"check FM link (Rx_active), PHY Tx enable, cable")
            in_flight_port = -1   # stop spamming; advance FSM watch

        # ── re-arm (throttled) ────────────────────────────────────────────
        now = time.time()
        if (s.ready == 0
                and len(events) < target_events
                and now - last_rearm_t >= rearm_interval_s):
            _wr(ga, R.READY_FORCE, port_mask)
            last_rearm_t  = now
            rearm_count  += 1
            log(s, f"↺  ReadyStatus re-armed (sweep {rearm_count + 1})")

        # ── compact history ───────────────────────────────────────────────
        history.append({
            "t_ms":     round(elapsed_ms, 1),
            "ready":    f"0x{s.ready:02X}",
            "tx_cnt":   s.tx_cnt,
            "last_tgt": f"0x{s.last_target:02X}",
            "rx_dav":   f"0x{s.rx_dav:02X}",
        })

    log(read_snap(ga), "── monitor end ──")

    # ─────────────────────────────────────────────────────────────────────
    # Summary
    # ─────────────────────────────────────────────────────────────────────
    print(f"\n{sep}")
    print(f"  Handshakes observed : {len(events)} / {target_events} expected")
    print(f"  Violations          : {len(violations)}")

    if events:
        print(f"\n  {'Port':>6}  {'UBT sent(ms)':>14}  "
              f"{'FEB reply(ms)':>14}  {'RTT(ms)':>10}  Status")
        print(f"  {'─'*6}  {'─'*14}  {'─'*14}  {'─'*10}  {'─'*20}")
        for ev in events:
            replied = f"{ev.feb_replied_ms:.1f}" if ev.feb_replied_ms else "—  (no reply)"
            rtt     = f"{ev.round_trip_ms:.1f}"  if ev.round_trip_ms  else "—"
            status  = "⚠ VIOLATION" if ev.violation else (
                      "⚠ no FEB reply" if not ev.feb_replied_ms else "✓")
            print(f"  {ev.port:>6}  {ev.ubt_sent_ms:>14.1f}  "
                  f"{replied:>14}  {rtt:>10}  {status}")

    if violations:
        print(f"\n  VIOLATIONS DETAIL:")
        for v in violations:
            print(f"    ⚠  {v}")
    elif len(events) > 0:
        print(f"\n  ✓  All observed UBT handshakes were sequential")

    # Diagnose common failure modes
    no_replies = sum(1 for ev in events if not ev.feb_replied_ms)
    if no_replies > 0:
        print(f"\n  DIAGNOSIS  ({no_replies} UBTs without FEB reply):")
        print(f"    1. Check Rx_active — run: RD {uc.compose_a16(ga, R.FEB_FM_ACTIVE):03X}")
        print(f"       Should be 0x00FF. If 0x00, FM links are down.")
        print(f"    2. Check TxEn is actually toggling (oscilloscope on PHY TxEn pin)")
        print(f"    3. Verify AutoTx_CdcDelay >= 12 in Controller_FPGA2.vhd")
        print(f"       (was 6 — causes UBTTarget FIFO miss → TxEnMask = 0x00 → no TX)")

    passed = (len(violations) == 0 and len(events) > 0
              and no_replies == 0)

    return {
        "timestamp":   datetime.now().isoformat(),
        "ga":          ga,
        "port_mask":   f"0x{port_mask:02X}",
        "rounds":      rounds,
        "observed":    len(events),
        "expected":    target_events,
        "no_replies":  no_replies,
        "violations":  violations,
        "events":      [asdict(ev) for ev in events],
        "history_len": len(history),
        "pass":        passed,
    }


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────��──────────────
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Monitor UBT sequential handshake on Controller FPGA2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples
────────
  python ubt_sequence_monitor.py
  python ubt_sequence_monitor.py --ga 2 --rounds 5
  python ubt_sequence_monitor.py --ga 1 --extra-ga 2 3 --ports 0xFF
  python ubt_sequence_monitor.py --ports 0x03 --rounds 2 --output r.json
        """,
    )
    ap.add_argument("--ip",              default="192.168.157.97")
    ap.add_argument("--port",            type=int,              default=5001)
    ap.add_argument("--ga",              type=int,              default=1)
    ap.add_argument("--extra-ga",        type=int, nargs="*",   default=[],
                    dest="extra_gas")
    ap.add_argument("--ports",           type=lambda x: int(x, 0), default=0xFF,
                    help="One-hot port mask (default 0xFF)")
    ap.add_argument("--rounds",          type=int,              default=3,
                    help="Full port sweeps to observe (default 3)")
    ap.add_argument("--poll-ms",         type=float,            default=8.0,
                    help="Poll interval ms (default 8)")
    ap.add_argument("--rearm-interval",  type=float,            default=3.0,
                    dest="rearm_interval",
                    help="Min seconds between ReadyForce re-arms (default 3)")
    ap.add_argument("--snap-full-every", type=int,              default=50,
                    dest="snap_full_every",
                    help="Read per-port RX word counts every N snaps (default 50)")
    ap.add_argument("--output",          default=None)
    ap.add_argument("--debug",           action="store_true")
    args = ap.parse_args()

    # Reduced timeout is the key fix — 3.0 s made every poll take 3 s
    uc.set_config(host=args.ip, port=args.port,
                  debug=args.debug, timeout_s=0.5)

    print(f"\n{'═'*70}")
    print(f"  UBT Sequence Monitor  —  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Target    : {args.ip}:{args.port}")
    print(f"  Primary GA: {args.ga}   Extra GAs: {args.extra_gas}")
    print(f"  Ports     : 0x{args.ports:02X}   Rounds: {args.rounds}")
    print(f"  Poll      : {args.poll_ms} ms   Re-arm min interval: {args.rearm_interval} s")
    print(f"{'═'*70}")

    results: Dict = {"pass": False}
    try:
        print("\n[SETUP] Clearing ReadyStatus and resetting TX FIFO …")
        _wr(args.ga, R.READY_CLEAR, 0xFF)
        time.sleep(0.01)
        reset_tx_fifo(args.ga)
        idle = wait_tx_idle(args.ga, timeout_s=5.0)
        print(f"         TX FIFO idle: {'✓' if idle else '⚠ not idle — proceeding anyway'}")

        links_ok = do_open(args.ga, args.extra_gas, args.ports)
        if not links_ok:
            print("\n  ⚠  Proceeding despite FM link warning — "
                  "FEB replies will not arrive until Rx_active is non-zero.")

        # Allow FM transition counter to saturate before monitoring
        time.sleep(0.20)

        results = monitor(
            ga               = args.ga,
            port_mask        = args.ports,
            rounds           = args.rounds,
            poll_s           = args.poll_ms / 1_000.0,
            rearm_interval_s = args.rearm_interval,
            snap_full_every  = args.snap_full_every,
        )

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        results = {"pass": False, "error": "interrupted"}

    finally:
        do_close(args.ga, args.extra_gas)

        if args.output:
            with open(args.output, "w") as fh:
                json.dump(results, fh, indent=2)
            print(f"\n  Report saved → {args.output}")

        try:
            uc.close()
        except Exception:
            pass

    verdict = "PASS ✓" if results.get("pass") else "FAIL ✗"
    print(f"\n  Overall: {verdict}\n")
    return 0 if results.get("pass") else 1


if __name__ == "__main__":
    sys.exit(main())