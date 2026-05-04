#!/usr/bin/env python3
"""
ubt_lane_feb_response_test.py
═══════════════════════════════════════════════════════════════════════════════
Test that UBT requests are routed to the correct PHY lane AND that the FEB
responds by sending data back to the ROC.

For each enabled port p the script:
  1. Clears all ReadyStatus bits.
  2. Forces ReadyStatus for port p only → AutoTx should send UBT to lane p.
  3. Verifies LAST_TX_TARGET has exactly bit p set (one-hot, no cross-lane
     firing).
  4. Waits for RX_DAV bit p to rise (FEB replied on the correct lane).
  5. Checks PHY_RX_WD_USED[p] > 0 (data words actually arrived in the buffer).
  6. Optionally reads back a sample of RX words from the port FIFO.

Pass criteria
─────────────
  • LAST_TX_TARGET == 1<<p  (correct lane, no spurious bits)
  • RX_DAV bit p rises within --reply-timeout seconds
  • PHY_RX_WD_USED[p] > 0 after RX_DAV is set

Usage
─────
  python ubt_lane_feb_response_test.py                     # all ports, GA=1
  python ubt_lane_feb_response_test.py --ports 0x03        # ports 0 and 1 only
  python ubt_lane_feb_response_test.py --ga 2 --rounds 3   # 3 rounds per port
  python ubt_lane_feb_response_test.py --readback 8 --verbose
  python ubt_lane_feb_response_test.py --output results.json
"""

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Dict, List, Optional

try:
    import uc_adapter_tcp as uc
except ImportError as exc:
    print(f"ERROR: could not import uc_adapter_tcp: {exc}")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# Register map  (addr10 values — matches Proj_Defs.vhd and ubt_sequence_monitor.py)
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
    DEBUG_VERSION   = 0x099   # Firmware version tag
    PHY_RX_WD_USED  = [0x018 + i for i in range(8)]   # per-port RX word count
    PHY_RX_RD       = [0x020 + i for i in range(8)]   # per-port RX data FIFO read


# CSR bit layout
_CSR_DDR_RD   = (1 << 7)   # DDRRd_en
_CSR_DDR_WRT  = (1 << 5)   # DDRWrt_En
_CSR_FMRXEN   = (1 << 3)   # FMRxEn
_CSR_PHY_ON   = (1 << 2)   # ~PhyPDn

# OPEN:  DDRRd_en=1, DDRWrt_En=1, FMRxEn=1, PHY_on  →  0xAC
_CSR_OPEN  = _CSR_DDR_RD | _CSR_DDR_WRT | _CSR_FMRXEN | _CSR_PHY_ON

# CLOSE: DDRRd_en=0, DDRWrt_En=0, FMRxEn=1, PHY_on  →  0x0C
_CSR_CLOSE = _CSR_FMRXEN | _CSR_PHY_ON

# Maximum data words to read back from the RX FIFO in one test
_MAX_RX_READBACK = 16


# ─────────────────────────────────────────────────────────────────────────────
# Per-lane, per-round test result
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class LaneTestResult:
    port:               int   = 0
    round_idx:          int   = 0

    # Lane routing
    ubt_sent:           bool  = False
    last_target_raw:    int   = 0     # LAST_TX_TARGET value when UBT fires
    correct_lane:       bool  = False  # last_target_raw == 1<<port (one-hot, exact)
    spurious_lanes:     int   = 0     # extra bits set beyond the expected lane

    # FEB response
    feb_replied:        bool  = False
    reply_dav_raw:      int   = 0     # full RX_DAV byte when reply was detected
    rx_words:           int   = 0     # PHY_RX_WD_USED[port] after reply

    # Timing
    ubt_sent_ms:        float = 0.0
    feb_reply_ms:       float = 0.0
    round_trip_ms:      float = 0.0

    # Failure detail (empty string = pass)
    fail_reason:        str   = ""


# ─────────────────────────────────────────────────────────────────────────────
# Low-level register helpers
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


def _bit(value: int, bit: int) -> bool:
    return bool((value >> bit) & 1)


# ─────────────────────────────────────────────────────────────────────────────
# Hardware setup / teardown
# ─────────────────────────────────────────────────────────────────────────────
def do_open(ga: int, extra_gas: List[int], port_mask: int) -> bool:
    """
    Open the ROC: set CSR, INPUT_MASK.
    Returns False if Rx_active is 0 (FM links appear down).
    """
    all_gas = [ga] + extra_gas
    print(f"\n[OPEN]  GAs={all_gas}  ports=0x{port_mask:02X}  CSR=0x{_CSR_OPEN:04X}")

    for g in all_gas:
        _wr(g, R.INPUT_MASK, port_mask)
        _wr(g, R.CSR,        _CSR_OPEN)
    time.sleep(0.05)

    csr    = _rd(ga, R.CSR)
    mask   = _rd(ga, R.INPUT_MASK)
    rs     = _rd(ga, R.READY_STATUS)
    rx_act = (_rd(ga, R.FEB_FM_ACTIVE) or 0) & 0xFF
    ver    = _rd(ga, R.DEBUG_VERSION)

    print(f"         GA={ga}  CSR=0x{(csr or 0):04X}  "
          f"mask=0x{(mask or 0) & 0xFF:02X}  "
          f"ReadyStatus=0x{(rs or 0) & 0xFF:02X}  "
          f"Rx_active=0x{rx_act:02X}  "
          f"FW_ver=0x{(ver or 0):04X}")

    if csr is not None and not (csr & _CSR_DDR_RD):
        print("  ⚠  WARNING: DDRRd_en (bit7) not set in CSR — "
              "ReadyStatus path may not fire")

    if rx_act == 0:
        print("  ⚠  WARNING: Rx_active=0x00 — no FEB FM links active; "
              "FEB reply frames will be silently dropped")
        return False

    active  = [i for i in range(8) if _bit(rx_act, i)]
    missing = [i for i in range(8) if _bit(port_mask, i) and not _bit(rx_act, i)]
    print(f"         FM-active ports : {active}")
    if missing:
        print(f"  ⚠  Ports in mask but FM-inactive (no FEB reply expected): {missing}")
    return True


def do_close(ga: int, extra_gas: List[int]) -> None:
    all_gas = [ga] + extra_gas
    print(f"\n[CLOSE] GAs={all_gas}")
    for g in all_gas:
        _wr(g, R.READY_CLEAR, 0xFF)
    time.sleep(0.01)
    for g in all_gas:
        _wr(g, R.CSR,        _CSR_CLOSE)
        _wr(g, R.INPUT_MASK, 0x00)
    time.sleep(0.02)

    csr  = _rd(ga, R.CSR)
    mask = _rd(ga, R.INPUT_MASK)
    rs   = _rd(ga, R.READY_STATUS)
    print(f"         GA={ga}  CSR=0x{(csr or 0):04X}  "
          f"mask=0x{(mask or 0) & 0xFF:02X}  "
          f"ReadyStatus=0x{(rs or 0) & 0xFF:02X}")


def reset_tx_fifo(ga: int) -> None:
    _wr(ga, R.TX_FIFO_RST, 0x0001)
    time.sleep(0.025)
    _wr(ga, R.TX_FIFO_RST, 0x0000)
    time.sleep(0.06)


def wait_tx_idle(ga: int, timeout_s: float = 5.0) -> bool:
    """Wait until the TX FIFO is empty and TxEnAck is de-asserted."""
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        csr = _rd(ga, R.PHY_TX_CSR)
        if csr is not None:
            fifo_empty = _bit(csr, 1)
            tx_ack     = _bit(csr, 0)
            if fifo_empty and not tx_ack:
                return True
        time.sleep(0.015)
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Core per-lane test
# ─────────────────────────────────────────────────────────────────────────────
def test_lane(ga: int, port: int, round_idx: int,
              ubt_timeout_s: float = 1.0,
              reply_timeout_s: float = 3.0,
              readback_words: int = 0,
              verbose: bool = False) -> LaneTestResult:
    """
    Test lane routing and FEB response for a single port in one round.

    Steps
    ─────
    1. Clear ALL ReadyStatus bits.
    2. Clear the LAST_TX_TARGET sticky latch.
    3. Force ReadyStatus for port p only so AutoTx targets exactly lane p.
    4. Poll LAST_TX_TARGET until non-zero (AutoTx fired the UBT).
    5. Verify LAST_TX_TARGET == 1<<p (correct lane, no spurious bits).
    6. Poll RX_DAV bit p until it rises (FEB sent data back).
    7. Read PHY_RX_WD_USED[p] to confirm data words arrived.
    8. Optionally read sample words from PHY_RX_RD[p].
    """
    res = LaneTestResult(port=port, round_idx=round_idx)
    t0  = time.time()
    expected_target = (1 << port) & 0xFF

    def elapsed_ms() -> float:
        return (time.time() - t0) * 1_000

    # ── 1. Clear ReadyStatus for all ports ──────────────────────────────────
    _wr(ga, R.READY_CLEAR, 0xFF)
    time.sleep(0.005)

    # ── 2. Clear the sticky LAST_TX_TARGET latch ────────────────────────────
    _wr(ga, R.LAST_TX_TARGET, 0x0000)
    time.sleep(0.005)

    lt_pre = (_rd(ga, R.LAST_TX_TARGET) or 0) & 0xFF
    if lt_pre != 0 and verbose:
        print(f"    [p{port}] ⚠ LAST_TX_TARGET not cleared "
              f"(reads 0x{lt_pre:02X} after write)")

    # ── 3. Force ReadyStatus for port p only ────────────────────────────────
    _wr(ga, R.READY_FORCE, expected_target)
    if verbose:
        rs = (_rd(ga, R.READY_STATUS) or 0) & 0xFF
        print(f"    [p{port}] ReadyStatus after force = 0x{rs:02X}  "
              f"(expected 0x{expected_target:02X})")

    # ── 4. Poll LAST_TX_TARGET until AutoTx fires ────────────────────────────
    t_ubt = time.time() + ubt_timeout_s
    while time.time() < t_ubt:
        lt = (_rd(ga, R.LAST_TX_TARGET) or 0) & 0xFF
        if lt:
            res.ubt_sent        = True
            res.last_target_raw = lt
            res.ubt_sent_ms     = elapsed_ms()
            break
        time.sleep(0.008)

    if not res.ubt_sent:
        res.fail_reason = (
            f"LAST_TX_TARGET never set within {ubt_timeout_s:.1f} s — "
            "AutoTx FSM did not fire "
            "(check ReadyStatus, INPUT_MASK, TX FIFO, CSR bit7)"
        )
        if verbose:
            csr_val = _rd(ga, R.PHY_TX_CSR) or 0
            cnt_val = _rd(ga, R.PHY_TX_CNT)  or 0
            rs_val  = (_rd(ga, R.READY_STATUS) or 0) & 0xFF
            print(f"    [p{port}] FAIL: AutoTx did not fire")
            print(f"           PhyTxCSR=0x{csr_val:04X}  "
                  f"TxCnt={cnt_val & 0x7FF}  "
                  f"ReadyStatus=0x{rs_val:02X}")
        return res

    # ── 5. Check lane routing (exact one-hot match) ──────────────────────────
    res.correct_lane   = (res.last_target_raw == expected_target)
    res.spurious_lanes = (res.last_target_raw & ~expected_target) & 0xFF

    if verbose:
        ok_mark = "✓" if res.correct_lane else "✗"
        spurious_note = (f"  ⚠ spurious bits=0x{res.spurious_lanes:02X}"
                         if res.spurious_lanes else "")
        print(f"    [p{port}] {ok_mark} UBT at {res.ubt_sent_ms:.1f} ms  "
              f"LAST_TX_TARGET=0x{res.last_target_raw:02X}  "
              f"expected=0x{expected_target:02X}{spurious_note}")

    if not res.correct_lane and not res.fail_reason:
        actual_ports = [i for i in range(8) if _bit(res.last_target_raw, i)]
        res.fail_reason = (
            f"Lane routing error: LAST_TX_TARGET=0x{res.last_target_raw:02X} "
            f"(ports {actual_ports}) != expected=0x{expected_target:02X} "
            f"(port {port})"
        )

    # ── 6. Poll RX_DAV for port p (FEB reply) ───────────────────────────────
    t_reply = time.time() + reply_timeout_s
    while time.time() < t_reply:
        dav = (_rd(ga, R.RX_DAV) or 0) & 0xFF
        if _bit(dav, port):
            res.feb_replied   = True
            res.reply_dav_raw = dav
            res.feb_reply_ms  = elapsed_ms()
            res.round_trip_ms = res.feb_reply_ms - res.ubt_sent_ms
            break
        time.sleep(0.010)

    if not res.feb_replied:
        reply_fail = (
            f"FEB did not reply on port {port} within {reply_timeout_s:.1f} s "
            f"(RX_DAV bit {port} never set)"
        )
        if verbose:
            dav    = (_rd(ga, R.RX_DAV) or 0) & 0xFF
            wds    = (_rd(ga, R.PHY_RX_WD_USED[port]) or 0) & 0x0FFF
            rx_act = (_rd(ga, R.FEB_FM_ACTIVE) or 0) & 0xFF
            print(f"    [p{port}] FAIL: {reply_fail}")
            print(f"           RX_DAV=0x{dav:02X}  "
                  f"WdUsed[{port}]={wds}  "
                  f"Rx_active=0x{rx_act:02X}")
        if not res.fail_reason:
            res.fail_reason = reply_fail
        return res

    # ── 7. Read PHY_RX_WD_USED[p] ───────────────────────────────────────────
    res.rx_words = (_rd(ga, R.PHY_RX_WD_USED[port]) or 0) & 0x0FFF

    if verbose:
        other_dav = res.reply_dav_raw & ~(1 << port) & 0xFF
        other_note = ""
        if other_dav:
            other_ports = [i for i in range(8) if _bit(other_dav, i)]
            other_note = f"  (also set on ports {other_ports})"
        print(f"    [p{port}] ← FEB replied at {res.feb_reply_ms:.1f} ms  "
              f"RTT={res.round_trip_ms:.1f} ms  "
              f"RxWords={res.rx_words}  "
              f"RX_DAV=0x{res.reply_dav_raw:02X}{other_note}")

    if res.rx_words == 0 and not res.fail_reason:
        res.fail_reason = (
            f"RX_DAV bit {port} set but PHY_RX_WD_USED[{port}]=0 "
            "— data may have been consumed or the word counter was not updated"
        )

    # ── 8. Optional RX data readback ────────────────────────────────────────
    if readback_words > 0 and res.rx_words > 0:
        n = min(readback_words, _MAX_RX_READBACK, res.rx_words)
        sample = []
        for _ in range(n):
            w = _rd(ga, R.PHY_RX_RD[port])
            if w is not None:
                sample.append(w & 0xFFFF)
        if verbose and sample:
            words_hex = " ".join(f"0x{w:04X}" for w in sample)
            print(f"    [p{port}]    RX sample ({len(sample)} words): {words_hex}")

    return res


# ─────────────────────────────────────────────────────────────────────────────
# Full test sweep across all enabled ports
# ─────────────────────────────────────────────────────────────────────────────
def run_test(ga: int,
             port_mask: int,
             rounds: int,
             ubt_timeout_s: float,
             reply_timeout_s: float,
             readback_words: int,
             verbose: bool) -> Dict:
    """
    For each round, iterate over every enabled port and call test_lane().
    Returns a results dict ready for JSON serialisation.
    """
    ports = [i for i in range(8) if _bit(port_mask, i)]
    total = rounds * len(ports)
    all_results: List[LaneTestResult] = []
    t0 = time.time()

    print(f"\n  Ports under test : {ports}")
    print(f"  Rounds           : {rounds}")
    print(f"  Total lane tests : {total}\n")

    hdr = (f"  {'Round':>6}  {'Port':>5}  {'Routing':>9}  "
           f"{'FEB rply':>9}  {'RTT ms':>8}  {'RxWords':>8}  Status")
    sep = "  " + "─" * (len(hdr) - 2)
    print(sep)
    print(hdr)
    print(sep)

    for rnd in range(rounds):
        for port in ports:
            # Wait for any residual TX activity to finish before each test
            wait_tx_idle(ga, timeout_s=2.0)

            res = test_lane(
                ga              = ga,
                port            = port,
                round_idx       = rnd,
                ubt_timeout_s   = ubt_timeout_s,
                reply_timeout_s = reply_timeout_s,
                readback_words  = readback_words,
                verbose         = verbose,
            )
            all_results.append(res)

            # Table row
            routing = "✓" if res.correct_lane else ("—" if not res.ubt_sent else "✗")
            replied = "✓" if res.feb_replied  else "✗"
            rtt     = f"{res.round_trip_ms:.1f}" if res.feb_replied else "—"
            wds     = str(res.rx_words) if res.feb_replied else "—"

            test_passed = res.correct_lane and res.feb_replied and res.rx_words > 0
            if test_passed:
                status = "PASS"
            elif res.correct_lane and res.feb_replied and res.rx_words == 0:
                status = "⚠ no data"
            else:
                status = "FAIL"

            print(f"  {rnd + 1:>6}  {port:>5}  {routing:>9}  "
                  f"{replied:>9}  {rtt:>8}  {wds:>8}  {status}")
            if res.fail_reason and not verbose:
                # Always print a brief failure hint even without --verbose
                print(f"         ↳ {res.fail_reason}")

            # Short delay between port tests to let FIFO drain fully
            time.sleep(0.05)

    print(sep)
    elapsed_s = time.time() - t0

    # ── Summary ──────────────────────────────────────────────────────────────
    passed      = sum(1 for r in all_results
                      if r.correct_lane and r.feb_replied and r.rx_words > 0)
    route_fails = sum(1 for r in all_results if r.ubt_sent and not r.correct_lane)
    no_ubt      = sum(1 for r in all_results if not r.ubt_sent)
    no_reply    = sum(1 for r in all_results if r.ubt_sent and not r.feb_replied)
    no_data     = sum(1 for r in all_results
                      if r.feb_replied and r.rx_words == 0)

    rtts    = [r.round_trip_ms for r in all_results if r.round_trip_ms > 0]
    avg_rtt = sum(rtts) / len(rtts) if rtts else 0.0
    max_rtt = max(rtts)             if rtts else 0.0

    print(f"\n  Tests run        : {total}")
    print(f"  PASS             : {passed}")
    print(f"  Routing errors   : {route_fails}")
    print(f"  AutoTx no-fire   : {no_ubt}")
    print(f"  FEB no reply     : {no_reply}")
    print(f"  DAV set, no data : {no_data}")
    if rtts:
        print(f"  RTT  avg / max   : {avg_rtt:.1f} ms / {max_rtt:.1f} ms")
    print(f"  Elapsed          : {elapsed_s:.1f} s")

    overall_pass = (passed == total)
    verdict = "PASS ✓" if overall_pass else "FAIL ✗"
    print(f"\n  Overall: {verdict}\n")

    if route_fails:
        print("  ROUTING FAILURES:")
        for r in all_results:
            if r.ubt_sent and not r.correct_lane:
                print(f"    Round {r.round_idx + 1} port {r.port}: {r.fail_reason}")
        print()

    if no_reply:
        print("  FEB NO-REPLY  — common causes:")
        print("    1. Check Rx_active (should equal port_mask when all FEBs active)")
        print(f"       RD {uc.compose_a16(ga, R.FEB_FM_ACTIVE):03X}")
        print("    2. Verify FM cable connections and FEB power")
        print("    3. Check AutoTx_CdcDelay >= 12 in Controller_FPGA2.vhd")
        print("       (too small → UBTTarget FIFO miss → TxEnMask=0x00 → no TX)")
        print()

    if no_ubt:
        print("  AUTOTX NO-FIRE  — common causes:")
        print("    1. INPUT_MASK excludes the port (mask must have bit p set)")
        print("    2. CSR bit7 (DDRRd_en) not set — ReadyStatus path blocked")
        print("    3. TX FIFO not drained — try --reset-fifo")
        print()

    return {
        "timestamp":    datetime.now().isoformat(),
        "ga":           ga,
        "port_mask":    f"0x{port_mask:02X}",
        "rounds":       rounds,
        "total":        total,
        "passed":       passed,
        "route_fails":  route_fails,
        "no_ubt":       no_ubt,
        "no_reply":     no_reply,
        "no_data":      no_data,
        "avg_rtt_ms":   round(avg_rtt, 2),
        "max_rtt_ms":   round(max_rtt, 2),
        "elapsed_s":    round(elapsed_s, 2),
        "pass":         overall_pass,
        "results":      [asdict(r) for r in all_results],
    }


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Test UBT lane routing and FEB→ROC data response",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples
────────
  python ubt_lane_feb_response_test.py
  python ubt_lane_feb_response_test.py --ports 0x03 --rounds 5
  python ubt_lane_feb_response_test.py --ga 2 --reply-timeout 5
  python ubt_lane_feb_response_test.py --readback 8 --verbose
  python ubt_lane_feb_response_test.py --output results.json
        """,
    )
    ap.add_argument("--ip",             default="192.168.157.97",
                    help="ROC IP address  (default 192.168.157.97)")
    ap.add_argument("--port",           type=int,                  default=5001,
                    help="TCP port  (default 5001)")
    ap.add_argument("--ga",             type=int,                  default=1,
                    help="Primary GA (geographic address)  (default 1)")
    ap.add_argument("--extra-ga",       type=int, nargs="*",       default=[],
                    dest="extra_gas",
                    help="Additional GAs to open alongside the primary GA")
    ap.add_argument("--ports",          type=lambda x: int(x, 0), default=0xFF,
                    help="One-hot port mask  (default 0xFF = all 8 ports)")
    ap.add_argument("--rounds",         type=int,                  default=1,
                    help="Test rounds per port  (default 1)")
    ap.add_argument("--ubt-timeout",    type=float,                default=1.0,
                    dest="ubt_timeout",
                    help="Seconds to wait for AutoTx to fire after ReadyForce "
                         "(default 1.0)")
    ap.add_argument("--reply-timeout",  type=float,                default=3.0,
                    dest="reply_timeout",
                    help="Seconds to wait for the FEB reply (RX_DAV rising) "
                         "(default 3.0)")
    ap.add_argument("--readback",       type=int,                  default=0,
                    help="Read back N RX words per port for inspection "
                         "(default 0 = disabled)")
    ap.add_argument("--verbose",        action="store_true",
                    help="Print per-step diagnostics for every port test")
    ap.add_argument("--reset-fifo",     action="store_true",
                    dest="reset_fifo",
                    help="Reset the TX FIFO before running tests")
    ap.add_argument("--output",         default=None,
                    help="Write JSON report to FILE")
    ap.add_argument("--debug",          action="store_true",
                    help="Enable uc_adapter_tcp debug logging")
    args = ap.parse_args()

    # Reduced timeout keeps polls snappy; 3.0 s caused ~3 s-per-poll latency
    uc.set_config(host=args.ip, port=args.port,
                  debug=args.debug, timeout_s=0.5)

    print(f"\n{'═' * 70}")
    print(f"  UBT Lane & FEB Response Test  —  "
          f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  Target       : {args.ip}:{args.port}")
    print(f"  Primary GA   : {args.ga}   Extra GAs: {args.extra_gas}")
    print(f"  Ports        : 0x{args.ports:02X}   Rounds: {args.rounds}")
    print(f"  Timeouts     : UBT={args.ubt_timeout} s  "
          f"FEB-reply={args.reply_timeout} s")
    print(f"{'═' * 70}")

    results: Dict = {"pass": False}
    try:
        print("\n[SETUP] Clearing ReadyStatus …")
        _wr(args.ga, R.READY_CLEAR, 0xFF)
        time.sleep(0.01)

        if args.reset_fifo:
            print("[SETUP] Resetting TX FIFO …")
            reset_tx_fifo(args.ga)

        idle = wait_tx_idle(args.ga, timeout_s=5.0)
        print(f"         TX FIFO idle: "
              f"{'✓' if idle else '⚠ not idle — proceeding anyway'}")

        links_ok = do_open(args.ga, args.extra_gas, args.ports)
        if not links_ok:
            print("\n  ⚠  FM links not all active — "
                  "FEB reply tests may fail for inactive ports.")

        # Allow FM transition counter to saturate before testing
        time.sleep(0.20)

        results = run_test(
            ga              = args.ga,
            port_mask       = args.ports,
            rounds          = args.rounds,
            ubt_timeout_s   = args.ubt_timeout,
            reply_timeout_s = args.reply_timeout,
            readback_words  = args.readback,
            verbose         = args.verbose,
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
