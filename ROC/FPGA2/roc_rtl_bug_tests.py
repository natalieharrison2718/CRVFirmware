#!/usr/bin/env python3
"""
roc_rtl_bug_tests.py
====================
Targeted RTL symptom checks for five known Controller_FPGA2.vhd issues.

Usage:
  python roc_rtl_bug_tests.py --ip 192.168.157.97 --ga 1
  python roc_rtl_bug_tests.py --ip 192.168.157.97 --ga 1 --port 0 --trials 20
  python roc_rtl_bug_tests.py --ip 192.168.157.97 --ga 1 --skip 3,5
  python roc_rtl_bug_tests.py --ip 192.168.157.97 --ga 1 --bug 2
  python roc_rtl_bug_tests.py --ip 192.168.157.97 --ga 1 --dry-run
"""

import argparse
import sys
import time
from collections import namedtuple
from typing import Dict, Optional, Set

try:
    import uc_adapter_tcp as uc
except ImportError as exc:
    print(f"ERROR: could not import uc_adapter_tcp: {exc}")
    sys.exit(1)


class C:
    RESET = "\033[0m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    CYAN = "\033[36m"
    BOLD = "\033[1m"


BugResult = namedtuple("BugResult", ["bug_num", "name", "verdict", "detail", "evidence"])


class R:
    TxEnMaskAd          = 0x00E
    PhyTxCSRAddr        = 0x012  # bit0=TxEnAck, bit1=PhyTxBuff_Empty
    PhyTxCntAddr        = 0x013
    InputMaskAddr       = 0x017
    PhyRxWdUsed         = [0x018 + i for i in range(8)]
    LastTxTargetAddr    = 0x049
    TxCurrentTargetAddr = 0x04A
    TxFifoResetAddr     = 0x04E
    CRCErrAddr          = 0x060
    PhyActivityCntAddr  = [0x070 + i for i in range(8)]

    ReadyStatusAddr     = 0x1A0
    ReadyClearAddr      = 0x1A1
    ReadyForceAddr      = 0x1A2

    # Script-defined defaults for timeout counters; update if your Proj_Defs differs.
    AutoTxTimeoutCntAd  = [0x0B0 + i for i in range(8)]
    AutoTxTimeoutClrAd  = 0x0B8


class ROC:
    def __init__(self, host: str, port: int, ga: int, debug: bool = False):
        self._ga = ga
        self._debug = debug
        uc.set_config(host=host, port=port, debug=debug)

    def _a16(self, addr10: int) -> int:
        return uc.compose_a16(self._ga, addr10 & 0x3FF)

    def read(self, addr10: int) -> Optional[int]:
        try:
            return uc.uc_read(self._a16(addr10))
        except Exception as exc:
            if self._debug:
                print(f"  [uc] rd 0x{addr10:03X} err: {exc}")
            return None

    def write(self, addr10: int, value: int) -> bool:
        try:
            ok, _ = uc.uc_write(self._a16(addr10), value & 0xFFFF)
            return bool(ok)
        except Exception as exc:
            if self._debug:
                print(f"  [uc] wr 0x{addr10:03X}=0x{value:04X} err: {exc}")
            return False

    def close(self):
        try:
            uc.close()
        except Exception:
            pass


def color_verdict(v: str) -> str:
    if v == "PASS":
        return f"{C.GREEN}{v}{C.RESET}"
    if v == "FAIL":
        return f"{C.RED}{v}{C.RESET}"
    if v == "SUSPICIOUS":
        return f"{C.YELLOW}{v}{C.RESET}"
    return f"{C.CYAN}{v}{C.RESET}"


def _h(v: Optional[int], w: int = 4) -> str:
    return f"0x{v:0{w}X}" if v is not None else "N/A"


def _mask(port: int) -> int:
    return (1 << port) & 0xFF


def _rd(roc: Optional[ROC], addr10: int, dry_run: bool) -> Optional[int]:
    if dry_run or roc is None:
        return None
    return roc.read(addr10)


def _wr(roc: Optional[ROC], addr10: int, val: int, dry_run: bool) -> bool:
    if dry_run or roc is None:
        return True
    return roc.write(addr10, val)


def _wait_last_tx_change(roc: ROC, dry_run: bool, timeout_s: float = 1.0) -> Dict:
    before = _rd(roc, R.LastTxTargetAddr, dry_run)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        cur = _rd(roc, R.LastTxTargetAddr, dry_run)
        if cur is not None and cur != 0 and cur != before:
            return {"latched": True, "elapsed_ms": (time.monotonic() - t0) * 1000.0, "value": cur, "before": before}
        time.sleep(0.002)
    return {"latched": False, "elapsed_ms": (time.monotonic() - t0) * 1000.0, "value": _rd(roc, R.LastTxTargetAddr, dry_run), "before": before}


def _cleanup(roc: Optional[ROC], port: int, dry_run: bool) -> None:
    m = _mask(port)
    _wr(roc, R.ReadyClearAddr, m, dry_run)
    _wr(roc, R.LastTxTargetAddr, 0xFFFF, dry_run)
    _wr(roc, R.AutoTxTimeoutClrAd, m, dry_run)
    _wr(roc, R.AutoTxTimeoutCntAd[port], 0x0000, dry_run)  # best-effort fallback if cnt register is writable


def _dry_result(bug_num: int, name: str, detail: str) -> BugResult:
    return BugResult(bug_num, name, "SKIPPED", f"DRY-RUN: {detail}", {"dry_run": True})


def test_bug1_cdc_autotx_target(roc: Optional[ROC], port: int, trials: Optional[int], dry_run: bool) -> BugResult:
    name = "AutoTx_Target CDC into i50MHz"
    if dry_run:
        return _dry_result(1, name, "Would force ReadyStatus, poll TxEnMask/CurrentTarget for >500 ms mismatch, and cross-check LastTxTarget vs TxEnMask over repeated trials.")

    print(f"{C.YELLOW}Warning: Bug 1 is metastability-related; software can only detect observable mismatch symptoms.{C.RESET}")
    evidence = {}
    try:
        m = _mask(port)
        _wr(roc, R.ReadyForceAddr, m, dry_run)

        mismatch_start = None
        suspicious_persist = False
        t0 = time.monotonic()
        samples = 0
        while time.monotonic() - t0 < 1.5:
            txen = _rd(roc, R.TxEnMaskAd, dry_run)
            cur = _rd(roc, R.TxCurrentTargetAddr, dry_run)
            last = _rd(roc, R.LastTxTargetAddr, dry_run)
            samples += 1

            if txen is not None and cur is not None and (txen & 0xFF) != 0 and (cur & 0xFF) == 0:
                if mismatch_start is None:
                    mismatch_start = time.monotonic()
                elif time.monotonic() - mismatch_start > 0.5:
                    suspicious_persist = True
                    evidence["persistent_mismatch"] = {
                        "txen": txen & 0xFF,
                        "current_target": cur & 0xFF,
                        "last_tx_target": (last or 0) & 0xFF,
                    }
                    break
            else:
                mismatch_start = None
            time.sleep(0.005)

        n_trials = trials if trials is not None else 6
        disagree = 0
        valid = 0
        for _ in range(max(3, n_trials)):
            _wr(roc, R.ReadyForceAddr, m, dry_run)
            time.sleep(0.01)
            txen = _rd(roc, R.TxEnMaskAd, dry_run)
            last = _rd(roc, R.LastTxTargetAddr, dry_run)
            if txen is None or last is None:
                continue
            txb = txen & 0xFF
            ltb = last & 0xFF
            if txb != 0 and ltb != 0:
                valid += 1
                if (txb & ltb) == 0:
                    disagree += 1
            time.sleep(0.01)

        evidence.update({
            "samples": samples,
            "disagree_trials": disagree,
            "valid_trials": valid,
        })

        if valid >= 3 and disagree >= max(2, valid // 2):
            return BugResult(1, name, "FAIL", f"LastTxTarget and TxEnMask disagreed in {disagree}/{valid} valid trials.", evidence)
        if suspicious_persist:
            return BugResult(1, name, "SUSPICIOUS", "Observed TxEnMask!=0 while TxCurrentTarget==0 persisting >500 ms.", evidence)
        return BugResult(1, name, "PASS", "No persistent TxEnMask/CurrentTarget mismatch observed.", evidence)
    finally:
        _cleanup(roc, port, dry_run)


def test_bug2_rxfilled_sticky(roc: Optional[ROC], port: int, trials: Optional[int], dry_run: bool) -> BugResult:
    name = "RxFilled_sticky only in AT_WaitRxFill"
    if dry_run:
        return _dry_result(2, name, "Would measure latency from UBT fire to PhyActivityCnt change and correlate with AutoTxTimeoutCnt increments.")

    evidence = {}
    try:
        m = _mask(port)
        base_act = _rd(roc, R.PhyActivityCntAddr[port], dry_run) or 0
        base_to = _rd(roc, R.AutoTxTimeoutCntAd[port], dry_run) or 0
        base_wd = _rd(roc, R.PhyRxWdUsed[port], dry_run) or 0

        _wr(roc, R.ReadyForceAddr, m, dry_run)
        latch = _wait_last_tx_change(roc, dry_run, timeout_s=1.0)

        t0 = time.monotonic()
        reply_ms = None
        timeout_ms = None
        first_wd_nonzero_ms = None
        while time.monotonic() - t0 < 1.0:
            act = _rd(roc, R.PhyActivityCntAddr[port], dry_run)
            wd = _rd(roc, R.PhyRxWdUsed[port], dry_run)
            toc = _rd(roc, R.AutoTxTimeoutCntAd[port], dry_run)

            if reply_ms is None and act is not None and act != base_act:
                reply_ms = (time.monotonic() - t0) * 1000.0
            if first_wd_nonzero_ms is None and wd is not None and wd > 0:
                first_wd_nonzero_ms = (time.monotonic() - t0) * 1000.0
            if timeout_ms is None and toc is not None and ((toc - base_to) & 0xFF) > 0:
                timeout_ms = (time.monotonic() - t0) * 1000.0
            if reply_ms is not None and timeout_ms is not None:
                break
            time.sleep(0.001)

        second_fire_ok = False
        if timeout_ms is None:
            before2 = _rd(roc, R.LastTxTargetAddr, dry_run)
            _wr(roc, R.ReadyForceAddr, m, dry_run)
            time.sleep(0.02)
            after2 = _rd(roc, R.LastTxTargetAddr, dry_run)
            second_fire_ok = (before2 is not None and after2 is not None and after2 != before2 and after2 != 0)

        evidence.update({
            "base_activity": base_act,
            "base_timeout_cnt": base_to,
            "base_rxwdused": base_wd,
            "ubt_latch": latch,
            "reply_ms": reply_ms,
            "first_rxwdused_nonzero_ms": first_wd_nonzero_ms,
            "timeout_ms": timeout_ms,
            "second_ubt_fired": second_fire_ok,
        })

        if reply_ms is not None and timeout_ms is not None and reply_ms < timeout_ms:
            return BugResult(2, name, "FAIL", "FEB activity arrived before timeout counter increment, indicating missed RxFilled_sticky capture.", evidence)
        if timeout_ms is None and second_fire_ok:
            return BugResult(2, name, "PASS", "No timeout increment observed and a second UBT fired normally.", evidence)
        return BugResult(2, name, "SUSPICIOUS", "Could not prove dropped reply latch; observed timing was inconclusive.", evidence)
    finally:
        _cleanup(roc, port, dry_run)


def test_bug3_fifo_reset_race(roc: Optional[ROC], port: int, trials: Optional[int], dry_run: bool) -> BugResult:
    name = "FIFO reset races with AT_WriteWords"
    if dry_run:
        return _dry_result(3, name, "Would force ReadyStatus, pulse TxFifoResetAddr during activity, then force again and correlate Tx count / timeout / LastTxTarget.")

    evidence = {}
    try:
        m = _mask(port)
        base_to = _rd(roc, R.AutoTxTimeoutCntAd[port], dry_run) or 0

        _wr(roc, R.ReadyForceAddr, m, dry_run)
        first = _wait_last_tx_change(roc, dry_run, timeout_s=0.6)

        _wr(roc, R.TxFifoResetAddr, 0x0001, dry_run)
        _wr(roc, R.ReadyForceAddr, m, dry_run)

        t0 = time.monotonic()
        zero_streak = 0
        saw_nonzero = False
        second_latched = False
        first_val = first.get("value")
        txcnt_samples = []
        while time.monotonic() - t0 < 0.8:
            txcnt = _rd(roc, R.PhyTxCntAddr, dry_run)
            last = _rd(roc, R.LastTxTargetAddr, dry_run)
            if txcnt is not None:
                c = txcnt & 0x7FF
                txcnt_samples.append(c)
                if c == 0:
                    zero_streak += 1
                else:
                    saw_nonzero = True
                    zero_streak = 0
            if last is not None and last != 0 and first_val is not None and last != first_val:
                second_latched = True
            time.sleep(0.002)

        end_to = _rd(roc, R.AutoTxTimeoutCntAd[port], dry_run) or base_to
        timeout_rose = ((end_to - base_to) & 0xFF) > 0

        evidence.update({
            "first_latch": first,
            "txcnt_min": min(txcnt_samples) if txcnt_samples else None,
            "txcnt_max": max(txcnt_samples) if txcnt_samples else None,
            "saw_nonzero_txcnt": saw_nonzero,
            "zero_streak_samples": zero_streak,
            "second_lasttx_latched": second_latched,
            "timeout_before": base_to,
            "timeout_after": end_to,
        })

        if timeout_rose and not second_latched:
            return BugResult(3, name, "FAIL", "Timeout counter increased without matching second LastTxTarget update after reset pulse.", evidence)
        if second_latched and not timeout_rose:
            return BugResult(3, name, "PASS", "Second force latched target and timeout counter did not rise.", evidence)
        return BugResult(3, name, "SUSPICIOUS", "Observed partial race symptoms but not a definitive loss pattern.", evidence)
    finally:
        _cleanup(roc, port, dry_run)


def test_bug4_txenreq_cdc(roc: Optional[ROC], port: int, trials: Optional[int], dry_run: bool) -> BugResult:
    name = "TxEnReq CDC SysClk->i50MHz"
    if dry_run:
        return _dry_result(4, name, "Would run repeated UBT trials, then measure TxEnAck assertion latency (5 ms threshold) after LastTxTarget latch.")

    print(f"{C.YELLOW}Warning: Bug 4 is metastability-related; software can only detect observable missed/late TxEnAck symptoms.{C.RESET}")
    evidence = {"trial_latencies_ms": []}
    try:
        m = _mask(port)
        n_trials = trials if trials is not None else 20
        missed = 0
        worst_ms = 0.0

        for _ in range(max(1, n_trials)):
            _wr(roc, R.LastTxTargetAddr, 0xFFFF, dry_run)
            _wr(roc, R.ReadyForceAddr, m, dry_run)
            l = _wait_last_tx_change(roc, dry_run, timeout_s=1.0)
            if not l["latched"]:
                evidence["trial_latencies_ms"].append(None)
                continue

            csr_at_latch = _rd(roc, R.PhyTxCSRAddr, dry_run)
            empty_at_latch = None if csr_at_latch is None else ((csr_at_latch >> 1) & 0x1)

            t0 = time.monotonic()
            lat = None
            while time.monotonic() - t0 < 0.200:
                csr = _rd(roc, R.PhyTxCSRAddr, dry_run)
                if csr is not None and (csr & 0x1):
                    lat = (time.monotonic() - t0) * 1000.0
                    break
                if time.monotonic() - t0 > 0.005:
                    break

            evidence["trial_latencies_ms"].append(lat)
            if lat is not None:
                worst_ms = max(worst_ms, lat)
            elif empty_at_latch == 0:
                missed += 1
            time.sleep(0.005)

        evidence["worst_latency_ms"] = worst_ms
        evidence["missed_ack_trials"] = missed

        if missed > 0:
            return BugResult(4, name, "SUSPICIOUS", f"TxEnAck did not assert within 5 ms in {missed} trial(s) while FIFO was non-empty.", evidence)
        return BugResult(4, name, "PASS", f"TxEnAck asserted in all observable trials; worst latency {worst_ms:.3f} ms.", evidence)
    finally:
        _cleanup(roc, port, dry_run)


def test_bug5_missing_fcs(roc: Optional[ROC], port: int, trials: Optional[int], dry_run: bool) -> BugResult:
    name = "No TX Ethernet FCS appended"
    if dry_run:
        return _dry_result(5, name, "Would run repeated UBT trials and check PhyActivityCnt deltas plus CRCErrAddr as an indirect acceptance signal.")

    evidence = {}
    try:
        m = _mask(port)
        n_trials = trials if trials is not None else 10
        fired = 0
        success = 0
        crc_samples = []

        act0 = _rd(roc, R.PhyActivityCntAddr[port], dry_run) or 0
        prev_act = act0
        for _ in range(max(1, n_trials)):
            _wr(roc, R.ReadyForceAddr, m, dry_run)
            latch = _wait_last_tx_change(roc, dry_run, timeout_s=1.0)
            if latch["latched"]:
                fired += 1
            time.sleep(0.05)
            act = _rd(roc, R.PhyActivityCntAddr[port], dry_run)
            crc = _rd(roc, R.CRCErrAddr, dry_run)
            if crc is not None:
                crc_samples.append(crc)
            if act is not None and act != prev_act:
                success += 1
                prev_act = act
            time.sleep(0.02)

        evidence.update({
            "trials": max(1, n_trials),
            "ubt_fired": fired,
            "activity_start": act0,
            "activity_end": prev_act,
            "activity_success_trials": success,
            "crc_err_samples": crc_samples,
            "note": "Indirect test only. Logic analyzer or Wireshark on MII needed for definitive FCS diagnosis.",
        })

        if fired > 0 and success == 0:
            return BugResult(5, name, "FAIL", "FEB activity never advanced across all fired trials — possible FCS bug or PHY link issue. (Indirect test)", evidence)
        return BugResult(5, name, "PASS", "Observed FEB activity in at least one fired trial. (Indirect test)", evidence)
    finally:
        _cleanup(roc, port, dry_run)


def parse_selection(args) -> Set[int]:
    selected = {1, 2, 3, 4, 5}
    if args.bug is not None:
        selected = {args.bug}
    if args.skip:
        for tok in args.skip.split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                selected.discard(int(tok))
            except ValueError:
                pass
    return selected


def print_result(res: BugResult) -> None:
    print(f"\n[{res.bug_num}] {res.name}")
    print(f"  verdict: {color_verdict(res.verdict)}")
    print(f"  detail : {res.detail}")
    if res.evidence:
        print("  evidence:")
        for k, v in res.evidence.items():
            print(f"    - {k}: {v}")


def print_summary(results):
    print("\n" + "=" * 86)
    print("RTL bug symptom summary")
    print("=" * 86)
    print(f"{'Bug':<5} {'Name':<42} {'Verdict':<12} Detail")
    print("-" * 86)
    for r in results:
        print(f"{r.bug_num:<5} {r.name[:42]:<42} {r.verdict:<12} {r.detail.splitlines()[0]}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Controller_FPGA2 RTL bug symptom tests")
    ap.add_argument("--ip", default="192.168.157.97", help="ROC IP")
    ap.add_argument("--port", type=int, default=0, help="PHY port index 0..7 (default 0)")
    ap.add_argument("--tcp-port", type=int, default=5002, help="ROC TCP port")
    ap.add_argument("--ga", type=int, required=True, help="GA (0..3)")
    ap.add_argument("--bug", type=int, choices=[1, 2, 3, 4, 5], default=None, help="Run only one bug test")
    ap.add_argument("--skip", default="", help="Comma-separated bug numbers to skip, e.g. 3,5")
    ap.add_argument("--trials", type=int, default=None, help="Override trial count for repeated tests")
    ap.add_argument("--debug", action="store_true", help="Enable uc_adapter_tcp debug")
    ap.add_argument("--dry-run", action="store_true", help="Print actions without hardware access")
    args = ap.parse_args()

    if not (0 <= args.port <= 7):
        print("ERROR: --port must be in [0..7]")
        sys.exit(2)

    selected = parse_selection(args)

    print(f"{C.BOLD}Controller_FPGA2 RTL bug checks{C.RESET}")
    print(f"Target: ip={args.ip} tcp_port={args.tcp_port} ga={args.ga} phy_port={args.port}")
    if args.dry_run:
        print(f"{C.CYAN}DRY-RUN mode enabled: no hardware reads/writes will be issued.{C.RESET}")

    roc = None if args.dry_run else ROC(args.ip, args.tcp_port, args.ga, debug=args.debug)

    tests = {
        1: test_bug1_cdc_autotx_target,
        2: test_bug2_rxfilled_sticky,
        3: test_bug3_fifo_reset_race,
        4: test_bug4_txenreq_cdc,
        5: test_bug5_missing_fcs,
    }

    results = []
    try:
        for bug_num in [1, 2, 3, 4, 5]:
            if bug_num not in selected:
                results.append(BugResult(bug_num, f"Bug {bug_num}", "SKIPPED", "Skipped by CLI selection.", {}))
                continue
            res = tests[bug_num](roc, args.port, args.trials, args.dry_run)
            results.append(res)
            print_result(res)
    finally:
        if roc is not None:
            roc.close()

    print_summary(results)

    hard_fail = any(r.verdict == "FAIL" for r in results)
    sys.exit(1 if hard_fail else 0)
