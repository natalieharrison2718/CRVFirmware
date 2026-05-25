#!/usr/bin/env python3
"""
feb_ubtrig_handshake_test.py  --  FEB ↔ ROC UBTRIG handshake diagnostic
========================================================================
Tests the full round-trip UBT handshake:

    ROC fires UBTRIG (UBT packet via PhyTxBuff)
      └─► FEB receives UBTRIG over Ethernet
            └─► FEB sends data back (Ethernet Rx → PhyRxBuff fills)
                  └─► DDR write sequencer drains PhyRxBuff → LPDDR
                        └─► AutoTx_ReArm fires → ROC sends next UBTRIG

Evidence collected at each step:
  Step 1 – LastTxTarget latches a one-hot value for the target port
  Step 2 – PhyActivityCounter[port] increments (counts PhyRxBuff_wreq pulses)
  Step 3 – DDR write pointer (SDWrtAd) advances, PhyRxBuff drains to zero
  Step 4 – LastTxTarget latches again for the same port

Multiple consecutive cycles can be measured with --cycles N.
Per-cycle timing and word counts are reported in a summary table.

Prerequisites
-------------
• uc_adapter_tcp.py (TCP bridge to µC bus) must be importable.
• ROC firmware must be running (post-reset, post-calibration).
• At least one FEB must be connected and responding to UBTRIG.

Usage
-----
  python feb_ubtrig_handshake_test.py --ip 192.168.157.97 --ga 1
  python feb_ubtrig_handshake_test.py --ip 192.168.157.97 --ga 1 --port 0 --cycles 5
  python feb_ubtrig_handshake_test.py --ip 192.168.157.97 --ga 1 --port 0 --cycles 3 \\
        --ubt-timeout 3 --fill-timeout 8 --drain-timeout 15 --ubt2-timeout 8
  python feb_ubtrig_handshake_test.py --ip 192.168.157.97 --ga 1 --scan-ports
"""

import argparse
import dataclasses
import sys
import time
from typing import List, Optional, Tuple

try:
    import uc_adapter_tcp as uc
except ImportError as exc:
    print(f"ERROR: could not import uc_adapter_tcp: {exc}")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Register map  (10-bit addresses from Proj_Defs.vhd)
# ---------------------------------------------------------------------------
class R:
    CSRRegAddr          = 0x000
    SDRamWrtPtrHiAd     = 0x002   # SDWrtAd[29:16] in lower 14 bits
    SDRamWrtPtrLoAd     = 0x003   # SDWrtAd[15:0]
    SDRamRdPtrHiAd      = 0x004
    SDRamRdPtrLoAd      = 0x005
    DDRStatAddr         = 0x008
    DDRRdStatAd         = 0x00C   # [15:12]=DDRWrtStat  [2:0]=DDRRdStat
    TxEnMaskAd          = 0x00E
    PhyTxCSRAddr        = 0x012   # [0]=TxEnAck  [1]=PhyTxBuff_Empty
    PhyTxCntAddr        = 0x013   # PhyTxBuff wr_data_count (11 bits)
    RxDAVAddr           = 0x016   # bit i = port i has data in PhyRxBuff
    InputMaskAddr       = 0x017   # MaskReg
    PhyRxWdUsed         = [0x018 + i for i in range(8)]
    FEBFMActiveAD       = 0x02F   # bit i = FM link active on port i
    SDRdPtrAddrHi       = 0x046
    SDRdPtrAddrLo       = 0x047
    LastTxTargetAddr    = 0x049   # one-hot lane last driven; write any value to clear
    TxCurrentTargetAddr = 0x04A   # live CurrentTarget (i50MHz domain)
    TxFifoRawEmptyAddr  = 0x04C   # bit 0 = raw PhyTxBuff_Empty
    TxFifoResetAddr     = 0x04E   # write pulse to reset TX FIFO
    DebugVersion        = 0x099
    PhyActivityCnt      = [0x070 + i for i in range(8)]
    ReadyStatusAddr     = 0x1A0
    ReadyClearAddr      = 0x1A1
    ReadyForceAddr      = 0x1A2
    AutoTxTimeoutCntAd  = [0x0B0 + i for i in range(8)]
    AutoTxTimeoutClrAddr= 0x0B8

# CSR read-back bit positions
CSR_RD_FMRX_EN    = (1 << 2)
CSR_RD_DDR_WRT_EN = (1 << 4)
CSR_RD_DDR_RD_EN  = (1 << 6)
DDR_STAT_CAL_DN   = (1 << 1)

# ---------------------------------------------------------------------------
# ANSI colours
# ---------------------------------------------------------------------------
OK    = "\033[32m✓\033[0m"
WARN  = "\033[33m⚠\033[0m"
FAIL  = "\033[31m✗\033[0m"
INFO  = "  "
ARROW = "\033[36m→\033[0m"


# ---------------------------------------------------------------------------
# Transport wrapper
# ---------------------------------------------------------------------------
class ROC:
    def __init__(self, host: str, port: int, ga: int, debug: bool = False):
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
def _one_hot_port(v: int) -> Optional[int]:
    v &= 0xFF
    if v == 0 or (v & (v - 1)):
        return None
    for i in range(8):
        if v & (1 << i):
            return i
    return None


def _read_ddr_wrt_addr(roc: ROC) -> int:
    hi = (roc.read(R.SDRamWrtPtrHiAd) or 0) & 0x3FFF
    lo = (roc.read(R.SDRamWrtPtrLoAd) or 0) & 0xFFFF
    return (hi << 16) | lo


def _ports(mask: int) -> List[int]:
    return [i for i in range(8) if mask & (1 << i)]


def _print_sep(char: str = "─", width: int = 68) -> None:
    print(f"  {char * width}")


# ---------------------------------------------------------------------------
# Result dataclass for one cycle
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class CycleResult:
    cycle_num:          int
    port:               int
    ubt1_fired:         bool   = False
    ubt1_latency_ms:    float  = 0.0
    feb_data_arrived:   bool   = False
    fill_latency_ms:    float  = 0.0
    fill_via:           str    = ""       # "WdUsed" | "ActivityCounter"
    peak_words:         int    = 0
    activity_delta:     int    = 0
    ddr_drained:        bool   = False
    drain_latency_ms:   float  = 0.0
    words_written:      int    = 0        # 32-bit words written to LPDDR
    ubt2_fired:         bool   = False
    ubt2_latency_ms:    float  = 0.0
    timeout_count:      int    = 0        # AutoTx_TimeoutCnt at end
    notes:              str    = ""

    @property
    def fully_ok(self) -> bool:
        return (self.ubt1_fired and self.feb_data_arrived
                and self.ddr_drained and self.ubt2_fired)


# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
def preflight(roc: ROC, port: int) -> List[str]:
    """
    Quick sanity checks before running handshake cycles.
    Returns a list of warning strings (empty = all OK).
    """
    warnings = []

    ver = roc.read(R.DebugVersion)
    if ver is None:
        warnings.append("FATAL: Cannot read DebugVersion — ROC not reachable")
        return warnings
    print(f"  {OK}  DebugVersion = 0x{ver:04X}")

    csr = (roc.read(R.CSRRegAddr) or 0)
    if not (csr & CSR_RD_FMRX_EN):
        warnings.append("CSR FMRxEn=0 (read bit 2). Write bit 3=1 to CSRRegAddr to enable.")
    if not (csr & CSR_RD_DDR_WRT_EN):
        warnings.append("CSR DDRWrt_En=0 (read bit 4). Write bit 5=1 to CSRRegAddr to enable.")
    if not (csr & CSR_RD_DDR_RD_EN):
        warnings.append("CSR DDRRd_en=0 (read bit 6). Write bit 7=1 to CSRRegAddr to enable.")
    if not warnings:
        print(f"  {OK}  CSR: FMRxEn, DDRWrt_En, DDRRd_en all enabled  "
              f"(read=0x{csr:04X})")

    ddr = (roc.read(R.DDRStatAddr) or 0)
    if not (ddr & DDR_STAT_CAL_DN):
        warnings.append("DDR not calibrated (SDCalDn=0). Firmware may need reset/init.")
    else:
        print(f"  {OK}  DDR calibrated")

    mk = (roc.read(R.InputMaskAddr) or 0) & 0xFF
    if mk == 0:
        warnings.append("MaskReg=0x00 — all ports masked. Write 0xFF to InputMaskAddr (0x017).")
    elif not (mk & (1 << port)):
        warnings.append(f"MaskReg[{port}]=0 — target port masked. "
                        f"Write 0x{mk | (1<<port):02X} to InputMaskAddr (0x017).")
    else:
        print(f"  {OK}  MaskReg = 0x{mk:02X}  (port {port} enabled)")





        
    act = (roc.read(R.FEBFMActiveAD) or 0) & 0xFF
    if not (act & (1 << port)):
        warnings.append(f"FM link inactive on port {port} (FEBFMActiveAD=0x{act:02X}). "
                        f"Check FEB connection, FMRxEn=1, and cable.")
    else:
        print(f"  {OK}  FM link active on port {port}  (FEBFMActiveAD=0x{act:02X})")


    if not (act & (1 << port)):
    warnings.append(
        f"FATAL: Rx_active[{port}]=0 (FEBFMActiveAD=0x{act:02X}). "
        f"DDR write sequencer will abort at CheckActive0 even if PhyRxBuff fills. "
        f"Check FM LVDS cable, FMRxEn=1 in CSR, and MaskReg[{port}]=1."
    )
    
    cnt  = (roc.read(R.PhyTxCntAddr) or 0) & 0x7FF
    raw  = (roc.read(R.TxFifoRawEmptyAddr) or 0) & 1
    tcsr = (roc.read(R.PhyTxCSRAddr) or 0)
    ack  = bool(tcsr & 0x01)
    if cnt > 0 and not raw:
        warnings.append(f"PhyTxBuff not empty at start (Count={cnt}, raw_empty={bool(raw)}). "
                        f"Pulse TxFifoResetAddr (0x{R.TxFifoResetAddr:03X}) to clear.")
    if ack:
        warnings.append("TxEnAck=1 at start — SMI_Proc busy. Wait or reset.")

    tout = (roc.read(R.AutoTxTimeoutCntAd[port]) or 0) & 0xFF
    if tout > 0:
        print(f"  {WARN}  AutoTx timeout counter[{port}] = {tout} "
              f"(pre-existing timeouts — clear with write to "
              f"AutoTxTimeoutClrAddr 0x{R.AutoTxTimeoutClrAddr:03X})")

    return warnings


# ---------------------------------------------------------------------------
# Single handshake cycle
# ---------------------------------------------------------------------------
def run_cycle(
    roc:             ROC,
    port:            int,
    cycle_num:       int,
    ubt_timeout_s:   float,
    fill_timeout_s:  float,
    drain_timeout_s: float,
    ubt2_timeout_s:  float,
    verbose:         bool,
) -> CycleResult:
    """
    Execute and measure one complete UBT → FEB reply → DDR drain → UBT cycle.
    Returns a populated CycleResult.
    """
    res       = CycleResult(cycle_num=cycle_num, port=port)
    one_hot   = 1 << port

    # ------------------------------------------------------------------
    # Step 1: Clear LastTxTarget, force ReadyStatus[port], wait for UBT
    # ------------------------------------------------------------------
    if verbose:
        print(f"\n    Step 1 — Force ReadyStatus[{port}], wait for UBT fire")

    roc.write(R.LastTxTargetAddr, 0xFFFF)   # clear diagnostic latch
    time.sleep(0.025)
    roc.write(R.ReadyClearAddr, one_hot)    # ensure bit is low
    time.sleep(0.005)

    # Sample baseline activity counter before force
    act_base = (roc.read(R.PhyActivityCnt[port]) or 0) & 0xFFFF
    wrt_base = _read_ddr_wrt_addr(roc)

    roc.write(R.ReadyForceAddr, one_hot)    # arm the port
    t_force = time.time()

    fired = False
    while time.time() - t_force < ubt_timeout_s:
        last = (roc.read(R.LastTxTargetAddr) or 0) & 0xFF
        if _one_hot_port(last) == port:
            res.ubt1_fired      = True
            res.ubt1_latency_ms = (time.time() - t_force) * 1000
            cnt = (roc.read(R.PhyTxCntAddr) or 0) & 0x7FF
            if verbose:
                print(f"    {OK}  UBT fired in {res.ubt1_latency_ms:.1f} ms  "
                      f"LastTxTarget=0x{last:02X}  PhyTxBuff_Count={cnt}")
            fired = True
            break

    if not fired:
        last_val = (roc.read(R.LastTxTargetAddr) or 0) & 0xFF
        rs       = (roc.read(R.ReadyStatusAddr)  or 0) & 0xFF
        mk       = (roc.read(R.InputMaskAddr)    or 0) & 0xFF
        cnt      = (roc.read(R.PhyTxCntAddr)     or 0) & 0x7FF
        raw      = (roc.read(R.TxFifoRawEmptyAddr) or 0) & 1
        tcsr     = (roc.read(R.PhyTxCSRAddr)     or 0)

        res.notes = (f"UBT did not fire. LastTxTarget=0x{last_val:02X} "
                     f"RS=0x{rs:02X} MaskReg=0x{mk:02X} "
                     f"TxCnt={cnt} TxRawEmpty={bool(raw)} "
                     f"TxEnAck={bool(tcsr & 0x01)}")
        if verbose:
            print(f"    {FAIL}  UBT did NOT fire within {ubt_timeout_s:.1f}s")
            print(f"           LastTxTarget=0x{last_val:02X}  "
                  f"ReadyStatus=0x{rs:02X}  MaskReg=0x{mk:02X}")
            print(f"           PhyTxBuff_Count={cnt}  RawEmpty={bool(raw)}  "
                  f"TxEnAck={bool(tcsr & 0x01)}")
            if not (rs & one_hot):
                print(f"           → ReadyStatus[{port}] not set — may have "
                      f"been consumed before first poll (very fast FSM)")
            if not (mk & one_hot):
                print(f"           → MaskReg[{port}]=0 — AT_Idle guard blocks UBT")
            if cnt > 0 and not raw:
                print(f"           → PhyTxBuff not empty (cnt={cnt}) — "
                      f"AT_Idle guard PhyTxBuff_Empty_s=1 FAILS")
                print(f"             Pulse TxFifoResetAddr "
                      f"(0x{R.TxFifoResetAddr:03X}) to clear stale data")
        return res

    t_ubt_fired = time.time()

    # ------------------------------------------------------------------
    # Step 2: Wait for FEB data — PhyActivityCounter or PhyRxWdUsed
    # ------------------------------------------------------------------
    if verbose:
        print(f"\n    Step 2 — Wait for FEB data reply (timeout {fill_timeout_s:.0f}s)")
        print(f"             PhyActivityCounter[{port}] baseline = {act_base}")

    peak_words  = 0
    filled      = False
    t0          = t_ubt_fired

    while time.time() - t0 < fill_timeout_s:
        wc  = (roc.read(R.PhyRxWdUsed[port])    or 0) & 0xFFF
        dav = (roc.read(R.RxDAVAddr)              or 0) & 0xFF
        act = (roc.read(R.PhyActivityCnt[port])  or 0) & 0xFFFF

        if wc > peak_words:
            peak_words = wc

        # Primary: FIFO visibly non-empty this poll cycle
        if wc > 0:
            elapsed = (time.time() - t0) * 1000
            res.feb_data_arrived = True
            res.fill_latency_ms  = elapsed
            res.fill_via         = "WdUsed"
            res.peak_words       = wc
            res.activity_delta   = act - act_base
            if verbose:
                print(f"    {OK}  FEB data arriving — WdUsed={wc}  "
                      f"DAV[{port}]={bool(dav & one_hot)}  "
                      f"ActivityCnt={act} (+{act - act_base})  "
                      f"({elapsed:.1f} ms after UBT)")
            filled = True
            break

        # Secondary: activity counter advanced (buffer filled & drained between polls)
        if act != act_base:
            elapsed = (time.time() - t0) * 1000
            delta   = act - act_base
            res.feb_data_arrived = True
            res.fill_latency_ms  = elapsed
            res.fill_via         = "ActivityCounter"
            res.peak_words       = peak_words
            res.activity_delta   = delta
            if verbose:
                print(f"    {OK}  FEB data received (drained before poll) — "
                      f"PhyActivityCounter +{delta}  "
                      f"({elapsed:.1f} ms after UBT)")
            act_base = act
            filled = True
            break

    if not filled:
        act_final = (roc.read(R.PhyActivityCnt[port]) or 0) & 0xFFFF
        delta     = act_final - act_base
        dav_final = (roc.read(R.RxDAVAddr) or 0) & 0xFF
        mk        = (roc.read(R.InputMaskAddr) or 0) & 0xFF
        fm_act    = (roc.read(R.FEBFMActiveAD) or 0) & 0xFF

        res.activity_delta = delta
        res.peak_words     = peak_words
        reason_parts = []
        if delta == 0:
            reason_parts.append("PhyActivityCounter did NOT advance — FEB sent NO data")
            if not (fm_act & one_hot):
                reason_parts.append(f"FM link lost (FEBFMActiveAD=0x{fm_act:02X})")
            if not (mk & one_hot):
                reason_parts.append(f"MaskReg[{port}]=0 gates PhyRxBuff_wreq")
        else:
            reason_parts.append(
                f"Counter advanced +{delta} but secondary check missed it "
                f"(increase --fill-timeout)")
        res.notes = "; ".join(reason_parts)

        if verbose:
            print(f"    {FAIL}  No FEB data within {fill_timeout_s:.0f}s")
            for r in reason_parts:
                print(f"           {ARROW} {r}")
            print(f"           Diagnosis snapshot:")
            print(f"             FEBFMActiveAD = 0x{fm_act:02X}  "
                  f"MaskReg = 0x{mk:02X}  "
                  f"DAV = 0x{dav_final:02X}")
        return res

    t_data_arrived = time.time()

    # ------------------------------------------------------------------
    # Step 3: Confirm PhyRxBuff drains into LPDDR via SDWrtAd delta
    # ------------------------------------------------------------------
    if verbose:
        print(f"\n    Step 3 — Wait for DDR drain (timeout {drain_timeout_s:.0f}s)")
        print(f"             DDR write ptr baseline = "
              f"0x{wrt_base:08X}  ({wrt_base // 4} words)")

    drained       = False
    prev_wrt      = wrt_base
    prev_wc       = (roc.read(R.PhyRxWdUsed[port]) or 0) & 0xFFF
    t0            = t_data_arrived

    while time.time() - t0 < drain_timeout_s:
        wrt_now  = _read_ddr_wrt_addr(roc)
        wc       = (roc.read(R.PhyRxWdUsed[port]) or 0) & 0xFFF
        dav      = (roc.read(R.RxDAVAddr)          or 0) & 0xFF
        ddr_stat = (roc.read(R.DDRRdStatAd) or 0)
        wrt_seq  = (ddr_stat >> 12) & 0xF

        if verbose and wrt_now != prev_wrt:
            delta = (wrt_now - prev_wrt) // 4
            print(f"    {INFO}  DDR ptr advancing "
                  f"0x{prev_wrt:08X} → 0x{wrt_now:08X}  "
                  f"(+{delta} words)  WdUsed={wc}  DDRWrtStat=0x{wrt_seq:X}")
            prev_wrt = wrt_now

        if verbose and wc < prev_wc and wc == 0:
            elapsed = (time.time() - t0) * 1000
            print(f"    {INFO}  PhyRxBuff[{port}] reached zero  ({elapsed:.1f} ms)")
        prev_wc = wc

        # Success A: ptr advanced AND buffer visibly empty
        if wrt_now > wrt_base and wc == 0 and not bool(dav & one_hot):
            elapsed = (time.time() - t0) * 1000
            res.ddr_drained      = True
            res.drain_latency_ms = elapsed
            res.words_written    = (wrt_now - wrt_base) // 4
            if verbose:
                print(f"    {OK}  PhyRxBuff[{port}] fully drained —")
                print(f"           DDR write ptr: "
                      f"0x{wrt_base:08X} → 0x{wrt_now:08X}  "
                      f"(+{res.words_written} × 32-bit words = "
                      f"{res.words_written * 4} bytes)")
                print(f"           Time to drain: {elapsed:.1f} ms")
            drained = True
            break

        # Success B: ptr advanced AND DDR_Write_Seq back at Idle
        if wrt_now > wrt_base and wrt_seq == 0:
            elapsed = (time.time() - t0) * 1000
            res.ddr_drained      = True
            res.drain_latency_ms = elapsed
            res.words_written    = (wrt_now - wrt_base) // 4
            if verbose:
                print(f"    {OK}  DDR drain complete (Idle, "
                      f"drain faster than poll) —")
                print(f"           DDR write ptr: "
                      f"0x{wrt_base:08X} → 0x{wrt_now:08X}  "
                      f"(+{res.words_written} × 32-bit words)")
                print(f"           Time to drain: {elapsed:.1f} ms")
            drained = True
            break

    if not drained:
        wrt_final   = _read_ddr_wrt_addr(roc)
        wc_final    = (roc.read(R.PhyRxWdUsed[port]) or 0) & 0xFFF
        ddr_stat_f  = roc.read(R.DDRRdStatAd) or 0
        csr_f       = roc.read(R.CSRRegAddr)  or 0

        res.words_written = (wrt_final - wrt_base) // 4
        if wrt_final == wrt_base:
            reason = (f"DDR write ptr did NOT advance. "
                      f"DDRWrt_En={bool(csr_f & CSR_RD_DDR_WRT_EN)} "
                      f"DDRWrtStat=0x{(ddr_stat_f>>12)&0xF:X} "
                      f"WdUsed={wc_final}")
        else:
            reason = (f"Partial drain: +{res.words_written} words but "
                      f"WdUsed={wc_final} remaining. "
                      f"DDRWrtStat=0x{(ddr_stat_f>>12)&0xF:X}")
        res.notes = (res.notes + "; " if res.notes else "") + reason

        if verbose:
            print(f"    {FAIL}  PhyRxBuff[{port}] did not drain within "
                  f"{drain_timeout_s:.0f}s")
            print(f"           DDR write ptr: "
                  f"0x{wrt_base:08X} → 0x{wrt_final:08X}  "
                  f"(delta = {res.words_written} words)")
            print(f"           WdUsed={wc_final}  "
                  f"DDRWrtStat=0x{(ddr_stat_f>>12)&0xF:X}")
            if wrt_final == wrt_base:
                print(f"           {ARROW} DDR_Write_Seq never ran. Possible:")
                print(f"               • DDRWrt_En=0  "
                      f"(CSR read bit 4 = "
                      f"{bool(csr_f & CSR_RD_DDR_WRT_EN)})")
                print(f"               • EventRdy never asserted — "
                      f"Rx_active and PhyRxBuff_RdStat mismatch")
                print(f"               • DDR not calibrated  "
                      f"(re-run with --preflight-only and check DDR status)")
        return res

    t_drained = time.time()

    # ------------------------------------------------------------------
    # Step 4: Clear LastTxTarget, wait for AutoTx_ReArm to fire next UBT
    # ------------------------------------------------------------------
    if verbose:
        print(f"\n    Step 4 — Wait for next (automatic) UBT "
              f"(timeout {ubt2_timeout_s:.0f}s)")

    roc.write(R.LastTxTargetAddr, 0xFFFF)
    time.sleep(0.025)
    t0 = t_drained

    while time.time() - t0 < ubt2_timeout_s:
        last = (roc.read(R.LastTxTargetAddr) or 0) & 0xFF
        cnt  = (roc.read(R.PhyTxCntAddr)     or 0) & 0x7FF
        if _one_hot_port(last) == port:
            res.ubt2_fired      = True
            res.ubt2_latency_ms = (time.time() - t0) * 1000
            if verbose:
                print(f"    {OK}  Second UBT fired in {res.ubt2_latency_ms:.1f} ms  "
                      f"LastTxTarget=0x{last:02X}  PhyTxBuff_Count={cnt}")
            break
        time.sleep(0.005)

    if not res.ubt2_fired:
        rs   = (roc.read(R.ReadyStatusAddr)            or 0) & 0xFF
        busy = (roc.read(R.PhyTxCntAddr)               or 0) & 0x7FF
        tout = (roc.read(R.AutoTxTimeoutCntAd[port])   or 0) & 0xFF
        note = (f"Second UBT not fired. RS=0x{rs:02X} "
                f"TxCnt={busy} TimeoutCnt={tout}")
        res.notes = (res.notes + "; " if res.notes else "") + note
        if verbose:
            print(f"    {FAIL}  Second UBT did not fire within {ubt2_timeout_s:.1f}s")
            print(f"           ReadyStatus=0x{rs:02X}  "
                  f"PhyTxBuff_Count={busy}  "
                  f"AutoTxTimeoutCnt[{port}]={tout}")
            print(f"           Possible causes:")
            print(f"             • AT_WaitDdrDrain did not set AutoTx_ReArm "
                  f"(check VHDL success branch)")
            print(f"             • AutoTx_Busy[{port}] not cleared after drain")
            print(f"             • DDR drain timeout fired  (tout={tout})")

    # Sample timeout counter at end
    res.timeout_count = (roc.read(R.AutoTxTimeoutCntAd[port]) or 0) & 0xFF

    return res


# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------
def print_summary(results: List[CycleResult]) -> None:
    _print_sep("═")
    print("  CYCLE SUMMARY")
    _print_sep("═")

    hdr = (f"  {'Cyc':>3}  {'UBT1':>5}  {'UBT1 ms':>8}  "
           f"{'FEB data':>9}  {'via':>16}  {'peak wds':>9}  "
           f"{'actv Δ':>7}  "
           f"{'DDR drain':>10}  {'wds wr':>7}  "
           f"{'drain ms':>9}  "
           f"{'UBT2':>5}  {'UBT2 ms':>8}  {'tout':>5}")
    print(hdr)
    _print_sep()

    for r in results:
        def _fmt(ok: bool, val: str) -> str:
            return f"\033[32m{val}\033[0m" if ok else f"\033[31m{val}\033[0m"

        print(
            f"  {r.cycle_num:>3}  "
            f"{_fmt(r.ubt1_fired, 'yes' if r.ubt1_fired else 'NO'):>14}  "
            f"{r.ubt1_latency_ms:>8.1f}  "
            f"{_fmt(r.feb_data_arrived, 'yes' if r.feb_data_arrived else 'NO'):>18}  "
            f"{r.fill_via:>16}  "
            f"{r.peak_words:>9}  "
            f"{r.activity_delta:>7}  "
            f"{_fmt(r.ddr_drained, 'yes' if r.ddr_drained else 'NO'):>19}  "
            f"{r.words_written:>7}  "
            f"{r.drain_latency_ms:>9.1f}  "
            f"{_fmt(r.ubt2_fired, 'yes' if r.ubt2_fired else 'NO'):>14}  "
            f"{r.ubt2_latency_ms:>8.1f}  "
            f"{r.timeout_count:>5}"
        )
        if r.notes:
            # Wrap long notes at 80 chars
            words = r.notes.split()
            line  = "       "
            for w in words:
                if len(line) + len(w) + 1 > 80:
                    print(f"\033[33m{line}\033[0m")
                    line = "       " + w
                else:
                    line += " " + w
            if line.strip():
                print(f"\033[33m{line}\033[0m")

    _print_sep()
    ok_cycles    = sum(1 for r in results if r.fully_ok)
    total_cycles = len(results)
    ubt1_ok      = sum(1 for r in results if r.ubt1_fired)
    feb_ok       = sum(1 for r in results if r.feb_data_arrived)
    ddr_ok       = sum(1 for r in results if r.ddr_drained)
    ubt2_ok      = sum(1 for r in results if r.ubt2_fired)

    sym = OK if ok_cycles == total_cycles else (WARN if ok_cycles > 0 else FAIL)
    print(f"\n  {sym}  {ok_cycles}/{total_cycles} cycles fully OK")
    print(f"  {INFO}  Step success rates:  "
          f"UBT1={ubt1_ok}/{total_cycles}  "
          f"FEB reply={feb_ok}/{total_cycles}  "
          f"DDR drain={ddr_ok}/{total_cycles}  "
          f"UBT2={ubt2_ok}/{total_cycles}")

    if ok_cycles == total_cycles:
        ubt1_times  = [r.ubt1_latency_ms  for r in results if r.ubt1_fired]
        fill_times  = [r.fill_latency_ms  for r in results if r.feb_data_arrived]
        drain_times = [r.drain_latency_ms for r in results if r.ddr_drained]
        ubt2_times  = [r.ubt2_latency_ms  for r in results if r.ubt2_fired]
        for label, times in [
            ("UBT1 fire latency",  ubt1_times),
            ("FEB data latency",   fill_times),
            ("DDR drain latency",  drain_times),
            ("UBT2 fire latency",  ubt2_times),
        ]:
            if times:
                print(f"  {INFO}  {label:22s}: "
                      f"min={min(times):.1f} ms  "
                      f"avg={sum(times)/len(times):.1f} ms  "
                      f"max={max(times):.1f} ms")

    # Per-step failure analysis
    failed_steps = []
    if ubt1_ok < total_cycles:
        failed_steps.append(
            f"UBT1 failed {total_cycles - ubt1_ok}x: check MaskReg, "
            f"PhyTxBuff_Empty_s=1, TxEnAck=0, AT_Idle guard"
        )
    if feb_ok < total_cycles:
        failed_steps.append(
            f"FEB reply missing {total_cycles - feb_ok}x: check FEB "
            f"connection, FM link active, Ethernet cable, MaskReg"
        )
    if ddr_ok < total_cycles:
        failed_steps.append(
            f"DDR drain failed {total_cycles - ddr_ok}x: check DDRWrt_En, "
            f"EventRdy logic, DDR_Write_Seq state"
        )
    if ubt2_ok < total_cycles:
        failed_steps.append(
            f"UBT2 not fired {total_cycles - ubt2_ok}x: check "
            f"AutoTx_ReArm, AT_WaitDdrDrain success branch, timeout counters"
        )
    if failed_steps:
        print(f"\n  FAILED STEP ANALYSIS:")
        for s in failed_steps:
            print(f"  {FAIL}  {s}")


# ---------------------------------------------------------------------------
# Port scan mode
# ---------------------------------------------------------------------------
def scan_ports(
    roc:             ROC,
    ubt_timeout_s:   float,
    fill_timeout_s:  float,
    drain_timeout_s: float,
    ubt2_timeout_s:  float,
) -> None:
    """
    Run a single-cycle handshake test on every port where FM is active.
    Reports a one-line pass/fail summary per port.
    """
    print(f"\n{'═'*68}")
    print(f"  PORT SCAN — testing all FM-active ports")
    print(f"{'═'*68}")

    fm_act = (roc.read(R.FEBFMActiveAD) or 0) & 0xFF
    mk     = (roc.read(R.InputMaskAddr)  or 0) & 0xFF
    active = _ports(fm_act & mk)

    if not active:
        print(f"  {FAIL}  No ports are both FM-active and MaskReg-enabled.")
        print(f"         FEBFMActiveAD=0x{fm_act:02X}  MaskReg=0x{mk:02X}")
        return

    print(f"  Testing ports: {active}  "
          f"(FEBFMActiveAD=0x{fm_act:02X} & MaskReg=0x{mk:02X})\n")

    scan_results = []
    for p in range(8):
        if not (fm_act & (1 << p)) or not (mk & (1 << p)):
            print(f"  Port {p}: skipped  "
                  f"(FM={'active' if fm_act & (1<<p) else 'inactive'}  "
                  f"mask={'1' if mk & (1<<p) else '0'})")
            continue

        print(f"  Port {p}: running …", end="", flush=True)
        r = run_cycle(
            roc, p, cycle_num=1,
            ubt_timeout_s=ubt_timeout_s,
            fill_timeout_s=fill_timeout_s,
            drain_timeout_s=drain_timeout_s,
            ubt2_timeout_s=ubt2_timeout_s,
            verbose=False,
        )
        scan_results.append((p, r))
        steps = (
            f"UBT1={'✓' if r.ubt1_fired else '✗'}  "
            f"FEB={'✓' if r.feb_data_arrived else '✗'}  "
            f"DDR={'✓' if r.ddr_drained else '✗'}  "
            f"UBT2={'✓' if r.ubt2_fired else '✗'}"
        )
        sym = OK if r.fully_ok else FAIL
        print(f"\r  Port {p}: {sym}  {steps}"
              + (f"  NOTE: {r.notes}" if r.notes else ""))

    print()
    ok_ports = [p for p, r in scan_results if r.fully_ok]
    bad_ports = [p for p, r in scan_results if not r.fully_ok]
    if bad_ports:
        print(f"  {FAIL}  Failing ports: {bad_ports}")
    if ok_ports:
        print(f"  {OK}  Passing ports: {ok_ports}")


# ---------------------------------------------------------------------------
# Snapshot of current state (useful before first cycle)
# ---------------------------------------------------------------------------
def print_state_snapshot(roc: ROC, port: int) -> None:
    print(f"\n── State snapshot ──────────────────────────────────────────")
    csr      = (roc.read(R.CSRRegAddr)            or 0)
    tcsr     = (roc.read(R.PhyTxCSRAddr)          or 0)
    mk       = (roc.read(R.InputMaskAddr)          or 0) & 0xFF
    fm_act   = (roc.read(R.FEBFMActiveAD)          or 0) & 0xFF
    dav      = (roc.read(R.RxDAVAddr)              or 0) & 0xFF
    cnt      = (roc.read(R.PhyTxCntAddr)           or 0) & 0x7FF
    raw      = (roc.read(R.TxFifoRawEmptyAddr)     or 0) & 1
    rs       = (roc.read(R.ReadyStatusAddr)         or 0) & 0xFF
    last     = (roc.read(R.LastTxTargetAddr)        or 0) & 0xFF
    txm      = (roc.read(R.TxEnMaskAd)             or 0) & 0xFF
    wrt_addr = _read_ddr_wrt_addr(roc)
    ddr_stat = (roc.read(R.DDRRdStatAd)            or 0)

    print(f"  CSR (read)          = 0x{csr:04X}  "
          f"FMRxEn={bool(csr & CSR_RD_FMRX_EN)}  "
          f"DDRWrt_En={bool(csr & CSR_RD_DDR_WRT_EN)}  "
          f"DDRRd_en={bool(csr & CSR_RD_DDR_RD_EN)}")
    print(f"  PhyTxBuff_Empty     = {bool(tcsr & 0x02)}  "
          f"TxEnAck = {bool(tcsr & 0x01)}  "
          f"PhyTxBuff_Count = {cnt}  RawEmpty = {bool(raw)}")
    print(f"  MaskReg             = 0x{mk:02X}  "
          f"FEBFMActive = 0x{fm_act:02X}")
    print(f"  ReadyStatus         = 0x{rs:02X}  "
          f"LastTxTarget = 0x{last:02X}  TxEnMask = 0x{txm:02X}")
    print(f"  PhyRxDAV            = 0x{dav:02X}")
    print(f"  DDR write ptr       = 0x{wrt_addr:08X}  "
          f"({wrt_addr // 4} words written total)")
    print(f"  DDRWrtStat          = 0x{(ddr_stat>>12)&0xF:X}  "
          f"DDRRdStat = 0x{ddr_stat&0x7:X}")
    tout = (roc.read(R.AutoTxTimeoutCntAd[port]) or 0) & 0xFF
    print(f"  AutoTxTimeoutCnt[{port}] = {tout}")
    wc = (roc.read(R.PhyRxWdUsed[port]) or 0) & 0xFFF
    act = (roc.read(R.PhyActivityCnt[port]) or 0) & 0xFFFF
    print(f"  PhyRxWdUsed[{port}]     = {wc}  "
          f"PhyActivityCnt[{port}] = {act}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="FEB ↔ ROC UBTRIG handshake diagnostic",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--ip",              default="192.168.157.97",
                        help="ROC IP address (default 192.168.157.97)")
    parser.add_argument("--port",            type=int, default=5002,
                        help="TCP bridge port (default 5002)")
    parser.add_argument("--ga",              type=int, default=1,
                        help="Geographic address 0–3 (default 1)")
    parser.add_argument("--feb-port",        type=int, default=0,
                        help="FEB port index 0–7 to test (default 0)")
    parser.add_argument("--cycles",          type=int, default=1,
                        help="Number of consecutive handshake cycles (default 1)")
    parser.add_argument("--ubt-timeout",     type=float, default=2.0,
                        help="Seconds to wait for UBT to fire (default 2.0)")
    parser.add_argument("--fill-timeout",    type=float, default=5.0,
                        help="Seconds to wait for FEB data (default 5.0)")
    parser.add_argument("--drain-timeout",   type=float, default=10.0,
                        help="Seconds to wait for DDR drain (default 10.0)")
    parser.add_argument("--ubt2-timeout",    type=float, default=5.0,
                        help="Seconds to wait for second UBT (default 5.0)")
    parser.add_argument("--scan-ports",      action="store_true",
                        help="Test all FM-active & masked ports, one cycle each")
    parser.add_argument("--preflight-only",  action="store_true",
                        help="Run pre-flight checks only, do not run cycles")
    parser.add_argument("--no-preflight",    action="store_true",
                        help="Skip pre-flight checks")
    parser.add_argument("--snapshot",        action="store_true",
                        help="Print a state snapshot before running cycles")
    parser.add_argument("--quiet",           action="store_true",
                        help="Suppress per-step verbose output; show summary only")
    parser.add_argument("--debug",           action="store_true",
                        help="Print raw TCP bridge I/O")
    args = parser.parse_args()

    roc = ROC(args.ip, args.port, args.ga, debug=args.debug)

    print(f"\nFEB ↔ ROC UBTRIG Handshake Test")
    print(f"  ROC: {args.ip}:{args.port}  GA={args.ga}  "
          f"Port: {args.feb_port}  Cycles: {args.cycles}")
    _print_sep("═")

    try:
        # ── Pre-flight ───────────────────────────────────────────────────
        if not args.no_preflight:
            print("\n── Pre-flight checks ───────────────────────────────────────")
            warnings = preflight(roc, args.feb_port)
            if warnings:
                print()
                for w in warnings:
                    print(f"  {WARN}  {w}")
                if any("FATAL" in w for w in warnings):
                    return 1
                print()
                if not args.preflight_only:
                    print(f"  Pre-flight warnings present.  "
                          f"Continuing anyway (use --no-preflight to suppress).")
            else:
                print(f"  {OK}  All pre-flight checks passed.")

        if args.preflight_only:
            return 0

        # ── State snapshot ───────────────────────────────────────────────
        if args.snapshot:
            print_state_snapshot(roc, args.feb_port)

        # ── Port scan ────────────────────────────────────────────────────
        if args.scan_ports:
            scan_ports(
                roc,
                ubt_timeout_s   = args.ubt_timeout,
                fill_timeout_s  = args.fill_timeout,
                drain_timeout_s = args.drain_timeout,
                ubt2_timeout_s  = args.ubt2_timeout,
            )
            return 0

        # ── Handshake cycles ─────────────────────────────────────────────
        results: List[CycleResult] = []
        verbose = not args.quiet

        for cyc in range(1, args.cycles + 1):
            if verbose or args.cycles > 1:
                print(f"\n{'─'*68}")
                print(f"  CYCLE {cyc}/{args.cycles}  —  port {args.feb_port}")
                print(f"{'─'*68}")

            r = run_cycle(
                roc,
                port            = args.feb_port,
                cycle_num       = cyc,
                ubt_timeout_s   = args.ubt_timeout,
                fill_timeout_s  = args.fill_timeout,
                drain_timeout_s = args.drain_timeout,
                ubt2_timeout_s  = args.ubt2_timeout,
                verbose         = verbose,
            )
            results.append(r)

            if not r.fully_ok and args.cycles > 1:
                print(f"\n  Stopping multi-cycle run after first failure.")
                break

        # ── Summary ──────────────────────────────────────────────────────
        print()
        print_summary(results)

    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        roc.close()

    results_exist = 'results' in dir() and results
    all_ok = results_exist and all(r.fully_ok for r in results)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
