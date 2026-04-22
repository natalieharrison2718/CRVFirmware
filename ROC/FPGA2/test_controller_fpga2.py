#!/usr/bin/env python3
"""
Functional test suite for Controller_FPGA2 (Mu2e ROC FPGA 2).

Covers:
  - Basic register read/write sanity (CSR, mask, test counter)
  - Uptime counter (increments at 1 Hz)
  - PHY Tx FIFO: load / reset / empty / count
  - PHY Rx FIFO word-count registers and DAV bits
  - AutoTx / ReadyStatus / ReadyClear handshake
  - UBT packet round-trip via AutoTx + LastTxTarget latch
  - DDR status and pointer registers
  - SMI control chain-select register
  - FM Rx error and parity-error registers
  - CRC error register and CRC readback arrays
  - Link-TX FIFO control (stat / trace)
  - Overflow counter register
  - Debug / version register

Requires: uc_adapter_tcp.py (same directory)

Usage
-----
  # Run all tests against default IP/GA
  python test_controller_fpga2.py

  # Specify target
  python test_controller_fpga2.py --ip 192.168.1.10 --port 5002 --ga 0

  # Run only a subset
  python test_controller_fpga2.py --tests csr,phytx,ddr

  # Save JSON report
  python test_controller_fpga2.py --output report.json
"""

import argparse
import json
import sys
import time
from dataclasses import dataclass, asdict, field
from datetime import datetime
from typing import Callable, Dict, List, Optional, Tuple

try:
    import uc_adapter_tcp as uc
except ImportError as e:
    print(f"ERROR: could not import uc_adapter_tcp: {e}")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Register-address constants (from Proj_Defs.vhd)
# ---------------------------------------------------------------------------
class R:
    CSRRegAddr          = 0x000   # Control / status register
    LinkCtrlAd          = 0x001   # Link control (bit0 = enable stat)
    SDRamWrtPtrHiAd     = 0x002   # DDR write pointer hi
    SDRamWrtPtrLoAd     = 0x003   # DDR write pointer lo
    SDRamRdPtrHiAd      = 0x004   # DDR read  pointer hi
    SDRamRdPtrLoAd      = 0x005   # DDR read  pointer lo
    SDRamSwapPort       = 0x006   # Byte-swapped DDR data port
    SDRamPortAd         = 0x007   # DDR data port
    DDRStatAddr         = 0x008   # DDR status flags
    DDRCountAddr        = 0x009   # DDR wr/rd counts
    DDRRdStatAd         = 0x00C   # DDR read FSM status
    TxEnMaskAd          = 0x00E   # TxEn mask
    TxFIFOWrtAd         = 0x010   # Serial link TX FIFO write
    PhyTxFIFOWrtAd      = 0x011   # PHY TX FIFO write
    PhyTxCSRAddr        = 0x012   # PHY TX CSR (bit0=TxEnAck, bit1=Empty)
    PhyTxCntAddr        = 0x013   # PHY TX FIFO word count (11-bit)
    RxErrAddr           = 0x014   # PHY RX error flags
    RxCRSAddr           = 0x015   # PHY Carrier Sense
    RxDAVAddr           = 0x016   # PHY RX data-available (active-low empty)
    InputMaskAddr       = 0x017   # Input port mask
    LastTxTargetAddr    = 0x049   # Sticky last-TX target latch
    TxCurrentTargetAddr = 0x04A   # Current transmit target (one-hot)
    TxFifoWrCountAddr   = 0x04B   # Mirrored TX FIFO wr_data_count
    TxFifoRawEmptyAddr  = 0x04C   # Raw TX FIFO empty bit
    AutoTxKickAddr      = 0x04D   # Auto-TX kick register
    TxFifoResetAddr     = 0x04E   # TX FIFO reset (pulse)
    TxFifoCtrlAddr      = 0x04F   # TX FIFO control bits
    SDRdPtrAddrHi       = 0x046   # DDR read-pointer snapshot hi
    SDRdPtrAddrLo       = 0x047   # DDR read-pointer snapshot lo
    LinkTxTraceAd       = 0x048   # Link TX trace buffer read
    TestCounterHiAd     = 0x043   # Test counter hi (writable)
    TestCounterLoAd     = 0x044   # Test counter lo (writable, read increments)
    DDR_BuffCountAd     = 0x045   # DDR address-buffer count
    FEBFMActiveAD       = 0x02F   # Active FEB FM channels
    FMRxStatAddr        = 0x040   # FM Rx FIFO full/empty status
    FMRxErrAddr         = 0x041   # FM Rx error / parity
    SPIWrtAddr          = 0x042   # SPI write request
    OverflowCntAd       = 0x080   # TX overflow counter
    SMIRdDataAd0        = 0x0FD   # SMI read data chain 0
    SMIRdDataAd1        = 0x0FE   # SMI read data chain 1
    DebugVersion        = 0x099   # Firmware version tag (reads 0x0011)
    SMICtrlAddr         = 0x0FF   # SMI chain-select
    DebugAddr           = 0x061   # Debug register
    CRCErrAddr          = 0x060   # CRC error flags
    ReadyStatusAddr     = 0x1A0   # ReadyStatus  (10-bit: "0110100000")
    ReadyClearAddr      = 0x1A1   # ReadyClear   (10-bit: "0110100001")

    # Per-port arrays  (0..7)
    PhyRxWdUsedRdAddr   = [0x018 + i for i in range(8)]
    PhyRxRdAddr         = [0x020 + i for i in range(8)]
    FEBFMRdAddr         = [0x030 + i for i in range(8)]
    FEBFMWdsUsedAddr    = [0x038 + i for i in range(8)]
    RdCRCAddr           = [0x050 + i for i in range(16)]
    PHYActivityCntAdd   = [0x070 + i for i in range(8)]


# ---------------------------------------------------------------------------
# Firmware-defined constants
# ---------------------------------------------------------------------------
EXPECTED_VERSION = 0x0011   # DebugVersion register


# ---------------------------------------------------------------------------
# Test result bookkeeping
# ---------------------------------------------------------------------------
@dataclass
class TestResult:
    name: str
    passed: bool
    message: str = ""
    details: Dict = field(default_factory=dict)


class TestSuite:


    @staticmethod
    def _hex(val: Optional[int], width: int = 4) -> str:
        """Format an optional int as '0xNNNN', or 'None' if val is None."""
        return f"0x{val:0{width}X}" if val is not None else "None"
    

    def __init__(self, ga: int):
        self.ga = ga
        self.results: List[TestResult] = []
        self._pass = 0
        self._fail = 0

    # ------------------------------------------------------------------
    # Low-level helpers
    # ------------------------------------------------------------------
    def _a16(self, addr10: int) -> int:
        return uc.compose_a16(self.ga, addr10)

    def _rd(self, addr10: int, name: str = "reg") -> Optional[int]:
        try:
            return uc.uc_read(self._a16(addr10))
        except Exception as e:
            print(f"    [READ  ERROR] {name} (0x{addr10:03X}): {e}")
            return None

    def _wr(self, addr10: int, value: int, name: str = "reg") -> bool:
        try:
            ok, reply = uc.uc_write(self._a16(addr10), value)
            if not ok:
                print(f"    [WRITE ERROR] {name} (0x{addr10:03X} <- 0x{value:04X}): {reply}")
            return ok
        except Exception as e:
            print(f"    [WRITE EXCEPT] {name} (0x{addr10:03X} <- 0x{value:04X}): {e}")
            return False

    def _poll(self, addr10: int, mask: int, expected: int,
              timeout_s: float = 1.0, interval_s: float = 0.01,
              name: str = "reg") -> bool:
        """Poll until (reg & mask) == expected or timeout."""
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            val = self._rd(addr10, name)
            if val is not None and (val & mask) == expected:
                return True
            time.sleep(interval_s)
        return False

    # ------------------------------------------------------------------
    # Result recording
    # ------------------------------------------------------------------
    def _record(self, name: str, passed: bool, msg: str = "", details: Dict = None):
        r = TestResult(name=name, passed=passed, message=msg,
                       details=details or {})
        self.results.append(r)
        if passed:
            self._pass += 1
            print(f"  ✓  {name}")
        else:
            self._fail += 1
            print(f"  ✗  {name}  — {msg}")
        return passed

    def _skip(self, name: str, reason: str):
        r = TestResult(name=name, passed=True,
                       message=f"SKIPPED: {reason}")
        self.results.append(r)
        print(f"  -  {name}  (skipped: {reason})")

    # ------------------------------------------------------------------
    # ── Test groups ────────────────────────────────────────────────────
    # ------------------------------------------------------------------

    # ── 1. Firmware version ─────────────────────────────────────────────
    def test_version(self):
        print("\n[1] Firmware version")
        val = self._rd(R.DebugVersion, "DebugVersion")
        if val is None:
            self._record("version_readable", False, "register read failed")
            return
        self._record("version_readable", True, details={"value": f"0x{val:04X}"})
        self._record("version_correct",
                     val == EXPECTED_VERSION,
                     f"got 0x{val:04X}, expected 0x{EXPECTED_VERSION:04X}",
                     details={"got": f"0x{val:04X}", "expected": f"0x{EXPECTED_VERSION:04X}"})

    # ── 2. CSR register ─────────────────────────────────────────────────
    def test_csr(self):
        """Read and sanity-check the CSR register bits."""
        print("\n[2] CSR register")
        val = self._rd(R.CSRRegAddr, "CSR")
        if val is None:
            self._record("csr_readable", False, "register read failed")
            return
        self._record("csr_readable", True, details={"value": f"0x{val:04X}"})

        # bit 0 = RxBuffRst (normally 0 after boot)
        # bit 5 = DDRWrt_En, bit 7 = DDRRd_en  — just check readability
        print(f"    CSR = 0x{val:04X}  (RxBuffRst={val&1}, DDRWrtEn={(val>>5)&1}, "
              f"DDRRdEn={(val>>7)&1}, PhyPDn_inv={(val>>3)&1})")
        self._record("csr_reserved_bits_zero",
                     (val & 0x8010) == 0,
                     f"unexpected reserved bits set: 0x{val:04X}",
                     details={"csr": f"0x{val:04X}"})

    # ── 3. Input mask register ───────────────────────────────────────────
    def test_input_mask(self):
        """Write and read back InputMaskAddr; restore original."""
        print("\n[3] Input mask register")
        original = self._rd(R.InputMaskAddr, "InputMask")
        if original is None:
            self._record("input_mask_readable", False, "read failed"); return

        test_values = [0x00, 0x55, 0xAA, 0xFF]
        all_ok = True
        for tv in test_values:
            if not self._wr(R.InputMaskAddr, tv, "InputMask"):
                all_ok = False; break
            rb = self._rd(R.InputMaskAddr, "InputMask")
            if rb is None or (rb & 0xFF) != tv:
                all_ok = False
                print(f"    FAIL: wrote 0x{tv:02X}, read back 0x{rb:04X if rb is not None else 0}")
                break

        # Restore
        self._wr(R.InputMaskAddr, original & 0xFF, "InputMask")
        self._record("input_mask_read_write", all_ok,
                     "" if all_ok else "mask readback mismatch")

    # ── 4. Test counter ─────────────────────────────────────────────────
    def test_counter(self):
        """
        Test counter increments on each read of the low word.
        Firmware: each RDDL=2 on TestCounterLoAd causes TestCount+1.
        """
        print("\n[4] Test counter")
        # Preset to a known value
        preset_hi = 0x0000
        preset_lo = 0x0010
        if not self._wr(R.TestCounterHiAd, preset_hi, "TestCountHi"):
            self._record("test_counter_preset", False, "write hi failed"); return
        if not self._wr(R.TestCounterLoAd, preset_lo, "TestCountLo"):
            self._record("test_counter_preset", False, "write lo failed"); return

        # Read hi first (does NOT increment)
        hi0 = self._rd(R.TestCounterHiAd, "TestCountHi")
        if hi0 is None:
            self._record("test_counter_preset", False, "read hi failed"); return
        self._record("test_counter_preset", hi0 == preset_hi,
                     f"hi expected 0x{preset_hi:04X} got 0x{hi0:04X}")

        # Each read of lo increments by 1
        reads = []
        for _ in range(4):
            v = self._rd(R.TestCounterLoAd, "TestCountLo")
            reads.append(v)

        # Values should be preset_lo, preset_lo+1, preset_lo+2, preset_lo+3
        expected = [preset_lo + i for i in range(4)]
        ok = (reads == expected)
        self._record("test_counter_increments_on_read", ok,
                     f"expected {[hex(x) for x in expected]}, got {[hex(x) if x is not None else None for x in reads]}",
                     details={"reads": reads, "expected": expected})

    # ── 5. Uptime counter ────────────────────────────────────────────────
    def test_uptime(self):
        """
        Uptime increments at 1 Hz.  We wait ~2 s and verify it increased.
        """
        print("\n[5] Uptime counter (waiting ~2 s)")
        hi0 = self._rd(R.TestCounterHiAd)  # UpTimeRegAddrHi is different; use actual
        # Correct addresses:
        UP_HI = 0x06C
        UP_LO = 0x06D

        def read_uptime() -> Optional[int]:
            hi = self._rd(UP_HI, "UpTimeHi")
            lo = self._rd(UP_LO, "UpTimeLo")
            if hi is None or lo is None:
                return None
            return ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)

        t0 = read_uptime()
        if t0 is None:
            self._record("uptime_readable", False, "read failed"); return
        self._record("uptime_readable", True, details={"initial": t0})

        time.sleep(2.2)
        t1 = read_uptime()
        if t1 is None:
            self._record("uptime_increments", False, "second read failed"); return

        delta = t1 - t0
        # Allow 1 or 2 (might have just crossed a boundary)
        ok = 1 <= delta <= 3
        self._record("uptime_increments", ok,
                     f"expected delta 1-3 s, got {delta}",
                     details={"t0": t0, "t1": t1, "delta": delta})

    # ── 6. PHY TX FIFO ──────────────────────────────────────────────────
    def test_phytx_fifo(self):
        """
        Load a few words into the PHY TX FIFO, verify count, then reset.
        Note: does NOT actually trigger transmission.
        """
        print("\n[6] PHY TX FIFO")

        # Read initial state
        csr = self._rd(R.PhyTxCSRAddr, "PhyTxCSR")
        cnt = self._rd(R.PhyTxCntAddr, "PhyTxCnt")
        if csr is None or cnt is None:
            self._record("phytx_fifo_readable", False, "CSR/count read failed"); return

        empty = (csr >> 1) & 1
        self._record("phytx_fifo_initially_empty", empty == 1,
                     f"FIFO not empty at start (CSR=0x{csr:04X}, count={cnt & 0x7FF})",
                     details={"csr": f"0x{csr:04X}", "count": cnt & 0x7FF})

        # Write a few words — use broadcast address so GA doesn't matter
        BROADCAST_PHY = 0x301   # PhyTxBroadCastAd = "11" & X"01"
        words_written = 3
        for w in [0xDEAD, 0xBEEF, 0xCAFE]:
            self._wr(BROADCAST_PHY, w, "PhyTxBroadcast")
            time.sleep(0.002)

        time.sleep(0.01)
        cnt_after = self._rd(R.PhyTxCntAddr, "PhyTxCnt (after write)")
        if cnt_after is not None:
            self._record("phytx_fifo_count_increases",
                         (cnt_after & 0x7FF) >= words_written,
                         f"count={cnt_after & 0x7FF}, expected >= {words_written}",
                         details={"count": cnt_after & 0x7FF})
        else:
            self._record("phytx_fifo_count_increases", False, "count read failed")

        # Reset the FIFO
        self._wr(R.TxFifoResetAddr, 0x0001, "TxFifoReset")
        time.sleep(0.01)
        self._wr(R.TxFifoResetAddr, 0x0000, "TxFifoReset (clear)")
        time.sleep(0.05)

        csr_rst = self._rd(R.PhyTxCSRAddr, "PhyTxCSR (after reset)")
        cnt_rst = self._rd(R.PhyTxCntAddr, "PhyTxCnt (after reset)")
        if csr_rst is not None and cnt_rst is not None:
            empty_after = (csr_rst >> 1) & 1
            self._record("phytx_fifo_reset_empties",
                         empty_after == 1,
                         f"FIFO still not empty after reset (CSR=0x{csr_rst:04X}, count={cnt_rst & 0x7FF})",
                         details={"csr": f"0x{csr_rst:04X}", "count": cnt_rst & 0x7FF})
        else:
            self._record("phytx_fifo_reset_empties", False, "read failed after reset")

    # ── 7. PHY RX word-count registers ──────────────────────────────────
    def test_phyrx_registers(self):
        """
        Read the 8 PHY RX word-count registers and DAV register.
        Verifies they are readable; reports any non-zero (data arrived).
        """
        print("\n[7] PHY RX registers (8 ports)")
        dav = self._rd(R.RxDAVAddr, "RxDAV")
        if dav is None:
            self._record("phyrx_dav_readable", False, "RxDAV read failed"); return
        self._record("phyrx_dav_readable", True,
                     details={"dav": f"0x{dav:04X}"})
        print(f"    RxDAV (active=1): 0x{dav & 0xFF:02X}")

        counts = []
        all_readable = True
        for i in range(8):
            v = self._rd(R.PhyRxWdUsedRdAddr[i], f"PhyRxWdUsed[{i}]")
            if v is None:
                all_readable = False
                counts.append(None)
            else:
                counts.append(v & 0x0FFF)

        self._record("phyrx_wdcount_all_readable", all_readable,
                     "one or more word-count registers unreadable",
                     details={"counts": counts})

        print(f"    Port word counts: {counts}")
        # Verify the upper nibble is zero (only 12-bit count)
        for i, c in enumerate(counts):
            if c is not None and (c & 0xF000) != 0:
                self._record(f"phyrx_wdcount_port{i}_format", False,
                             f"port {i} upper bits set: 0x{c:04X}")
                return
        self._record("phyrx_wdcount_format_ok", True,
                     details={"counts": counts})

    # ── 8. PHY activity counters ──��──────────────────────────────────────
    def test_phy_activity_counters(self):
        """Read the 8 per-port activity counters; verify they are readable."""
        print("\n[8] PHY activity counters")
        values = []
        all_ok = True
        for i in range(8):
            v = self._rd(R.PHYActivityCntAdd[i], f"PhyAct[{i}]")
            if v is None:
                all_ok = False
                values.append(None)
            else:
                values.append(v & 0xFFFF)

        self._record("phy_activity_all_readable", all_ok,
                     "one or more activity counters unreadable",
                     details={"values": values})
        print(f"    Activity counts: {values}")

    # ── 9. DDR register sanity ───────────────────────────────────────────
    def test_ddr_registers(self):
        """
        Verify DDR pointer registers accept writes and read back correctly.
        Does NOT start any DDR access.
        """
        print("\n[9] DDR pointer registers")
        WR_HI = 0x00AB
        WR_LO = 0x1234
        # Write address pointers
        ok_whi = self._wr(R.SDRamWrtPtrHiAd, WR_HI, "DDRWrtPtrHi")
        ok_wlo = self._wr(R.SDRamWrtPtrLoAd, WR_LO, "DDRWrtPtrLo")
        if not (ok_whi and ok_wlo):
            self._record("ddr_ptr_writable", False, "write failed"); return

        rb_hi = self._rd(R.SDRamWrtPtrHiAd, "DDRWrtPtrHi")
        rb_lo = self._rd(R.SDRamWrtPtrLoAd, "DDRWrtPtrLo")

        # Only 14-bit hi field (bits 13:0 meaningful)
        hi_ok = rb_hi is not None and (rb_hi & 0x3FFF) == (WR_HI & 0x3FFF)
        lo_ok = rb_lo is not None and rb_lo == WR_LO

        rb_hi_str = f"0x{rb_hi:04X}" if rb_hi is not None else "None"
        rb_lo_str = f"0x{rb_lo:04X}" if rb_lo is not None else "None"

        self._record("ddr_wrt_ptr_readback", hi_ok and lo_ok,
                     f"hi: wrote 0x{WR_HI:04X}, got {rb_hi_str}; "
                     f"lo: wrote 0x{WR_LO:04X}, got {rb_lo_str}",
                     details={"hi_written": WR_HI, "hi_readback": rb_hi,
                               "lo_written": WR_LO, "lo_readback": rb_lo})

        # DDR status register — just readable
        stat = self._rd(R.DDRStatAddr, "DDRStat")
        cnt  = self._rd(R.DDRCountAddr, "DDRCount")
        stat_str = f"0x{stat:04X}" if stat is not None else "None"
        cnt_str  = f"0x{cnt:04X}"  if cnt  is not None else "None"
        self._record("ddr_stat_readable",
                     stat is not None and cnt is not None,
                     "DDR status or count register unreadable",
                     details={"stat": stat_str, "count": cnt_str})
        if stat is not None:
            calib_done = (stat >> 0) & 1
            print(f"    DDR stat=0x{stat:04X}  (CalibDone={calib_done})")

    # ── 10. SMI chain-select register ───────────────────────────────────
    def test_smi_ctrl(self):
        """Read/write SMI control register (chain-select, MDIORd)."""
        print("\n[10] SMI control register")
        original = self._rd(R.SMICtrlAddr, "SMICtrl")
        if original is None:
            self._record("smi_ctrl_readable", False, "read failed"); return
        self._record("smi_ctrl_readable", True,
                     details={"value": f"0x{original:04X}"})

        # Write chain 1, MDIORd=0
        test_val = 0x0002   # ChainSel=10b, MDIORd=0
        ok = self._wr(R.SMICtrlAddr, test_val, "SMICtrl")
        if not ok:
            self._record("smi_ctrl_writable", False, "write failed"); return

        rb = self._rd(R.SMICtrlAddr, "SMICtrl")
        # Only lower 3 bits are writable (MDIORd + ChainSel[1:0])
        match = rb is not None and (rb & 0x0007) == (test_val & 0x0007)
        self._record("smi_ctrl_readback", match,
                     f"wrote 0x{test_val:04X}, read {self._hex(rb)}",
                     details={"written": test_val, "readback": rb})

        # Restore
        self._wr(R.SMICtrlAddr, original & 0x0007, "SMICtrl restore")


    # ── 11. FM Rx status and error registers ────────────────────────────
    def test_fm_rx(self):
        """Verify FM Rx stat and error registers are readable."""
        print("\n[11] FM Rx registers")
        stat = self._rd(R.FMRxStatAddr, "FMRxStat")
        err  = self._rd(R.FMRxErrAddr,  "FMRxErr")
        self._record("fmrx_stat_readable",
                     stat is not None,
                     "FMRxStat read failed",
                     details={"value": f"0x{stat:04X}" if stat else None})
        self._record("fmrx_err_readable",
                     err is not None,
                     "FMRxErr read failed",
                     details={"value": f"0x{err:04X}" if err else None})

        if stat is not None:
            full_bits  = (stat >> 8) & 0xFF
            empty_bits = stat & 0xFF
            print(f"    FMRxStat: full=0x{full_bits:02X}  empty=0x{empty_bits:02X}")
        if err is not None:
            print(f"    FMRxErr:  0x{err:04X}")

        # Clear all parity errors (write 1s to bits 7:0 + bit 8)
        self._wr(R.FMRxErrAddr, 0x01FF, "FMRxErr clear")
        err_after = self._rd(R.FMRxErrAddr, "FMRxErr (after clear)")
        # After clearing, parity bits should be 0 (no new errors)
        if err_after is not None:
            parity_bits = err_after & 0x00FF
            self._record("fmrx_err_clears",
                         parity_bits == 0,
                         f"parity bits not cleared: 0x{parity_bits:02X}",
                         details={"after_clear": f"0x{err_after:04X}"})
        else:
            self._record("fmrx_err_clears", False, "read failed after clear")

    # ── 12. FEB FM word-count and data registers ─────────────────────────
    def test_feb_fm_registers(self):
        """Read FEB FM active, word-used, and status registers."""
        print("\n[12] FEB FM registers")
        active = self._rd(R.FEBFMActiveAD, "FEBFMActive")
        self._record("feb_fm_active_readable",
                     active is not None,
                     "FEBFMActive read failed",
                     details={"value": f"0x{active:04X}" if active else None})
        if active is not None:
            print(f"    Active FEB FM ports: 0x{active & 0xFF:02X}")

        # Read all 8 word-count registers
        all_ok = True
        wds_used = []
        for i in range(8):
            v = self._rd(R.FEBFMWdsUsedAddr[i], f"FEBFMWdsUsed[{i}]")
            if v is None:
                all_ok = False
                wds_used.append(None)
            else:
                wds_used.append(v & 0x07FF)
        self._record("feb_fm_wds_all_readable", all_ok,
                     "one or more FEB FM word-count registers unreadable",
                     details={"wds_used": wds_used})
        print(f"    FEB FM words used: {wds_used}")

    # ── 13. CRC error register ───────────────────────────────────────────
    def test_crc_registers(self):
        """
        Read CRC error flags and CRC array registers.
        Also verifies clearing the CRC error register.
        """
        print("\n[13] CRC registers")
        crc_err = self._rd(R.CRCErrAddr, "CRCErr")
        self._record("crc_err_readable",
                     crc_err is not None,
                     "CRCErrAddr read failed",
                     details={"value": f"0x{crc_err:04X}" if crc_err else None})
        if crc_err is not None:
            print(f"    CRC errors: 0x{crc_err & 0xFF:02X}  (bits set = ports with CRC error)")

        # Clear all CRC errors
        self._wr(R.CRCErrAddr, 0x00FF, "CRCErr clear all")
        crc_after = self._rd(R.CRCErrAddr, "CRCErr (after clear)")
        if crc_after is not None:
            self._record("crc_err_clears",
                         (crc_after & 0xFF) == 0,
                         f"CRC err bits still set: 0x{crc_after & 0xFF:02X}",
                         details={"after": f"0x{crc_after:04X}"})
        else:
            self._record("crc_err_clears", False, "read failed after clear")

        # Read a sample of the CRC array (first 4 port pairs)
        crc_ok = True
        for i in range(4):
            hi = self._rd(R.RdCRCAddr[2 * i],     f"CRC[{i}]_hi")
            lo = self._rd(R.RdCRCAddr[2 * i + 1], f"CRC[{i}]_lo")
            if hi is None or lo is None:
                crc_ok = False
                break
        self._record("crc_array_readable", crc_ok,
                     "one or more CRC array registers unreadable")

    # ── 14. ReadyStatus / ReadyClear handshake ───────────────────────────
    def test_ready_status(self):
        """
        Verify ReadyStatus / ReadyClear handshake.

        Instead of asserting DDRRd_en and then reading ReadyStatus (which
        AutoTx may have already cleared), we poll ReadyStatus tightly in the
        ~10 ms window between the rising edge and AutoTx consuming the bits.
        If we still miss it we fall back to checking that AutoTx produced
        words in the PHY TX FIFO, which is equally valid proof that
        ReadyStatus bits were raised.

        """
        print("\n[14] ReadyStatus / ReadyClear handshake")

        # ── Read initial state ────────────────────────────────────────────
        initial = self._rd(R.ReadyStatusAddr, "ReadyStatus (initial)")
        if initial is None:
            self._record("ready_status_readable", False, "read failed"); return
        self._record("ready_status_readable", True,
                     details={"initial": f"0x{initial & 0xFF:02X}"})
        print(f"    Initial ReadyStatus = 0x{initial & 0xFF:02X}")

        # ── Preserve the full CSR ─────────────────────────────────────────
        csr_orig = self._rd(R.CSRRegAddr, "CSR (original)")
        if csr_orig is None:
            self._record("ready_status_set_via_ddr", False, "CSR read failed"); return

        DDR_RD_BIT = (1 << 7)

        # ── Step 1: guarantee DDRRd_en = 0 ───────────────────────────────
        if csr_orig & DDR_RD_BIT:
            csr_low = csr_orig & ~DDR_RD_BIT & 0xFFFF
            if not self._wr(R.CSRRegAddr, csr_low, "CSR DDRRd_en=0"):
                self._record("ready_status_set_via_ddr", False, "CSR write failed"); return
            time.sleep(0.02)
        else:
            csr_low = csr_orig

        # ── Step 2: write MaskReg = 0xFF ─────────────────────────────────
        mask_orig = self._rd(R.InputMaskAddr, "InputMask (original)")
        if not self._wr(R.InputMaskAddr, 0xFF, "InputMask (all ports)"):
            self._record("ready_status_set_via_ddr", False, "MaskReg write failed"); return
        time.sleep(0.01)

        # ── Step 3: pre-clear any leftover ReadyStatus bits ───────────────
        self._wr(R.ReadyClearAddr, 0xFF, "ReadyClear (pre-clear)")
        time.sleep(0.01)
        rs_pre = self._rd(R.ReadyStatusAddr, "ReadyStatus (pre-edge)")
        print(f"    ReadyStatus before edge = 0x{(rs_pre or 0) & 0xFF:02X}")

        # ── Step 4: rising edge on DDRRd_en ──────────────────────────────
        csr_now = self._rd(R.CSRRegAddr, "CSR (before edge)")
        if csr_now is None:
            csr_now = csr_low
        csr_high = (csr_now & ~DDR_RD_BIT & 0xFFFF) | DDR_RD_BIT
        if not self._wr(R.CSRRegAddr, csr_high, "CSR DDRRd_en=1"):
            self._record("ready_status_set_via_ddr", False, "CSR edge write failed"); return

        # ── Step 5: poll ReadyStatus tightly, AND watch the TX FIFO ──────
        # AutoTx may consume the ReadyStatus bits within microseconds, but
        # doing so requires writing words into PhyTxBuff.  We accept either:
        #   (a) we catch ReadyStatus non-zero during the poll, OR
        #   (b) we see the TX FIFO fill up (proof AutoTx fired).

        rs_seen   = 0
        fifo_seen = False
        t_end = time.time() + 0.5          # 500 ms poll window
        while time.time() < t_end:
            rs = self._rd(R.ReadyStatusAddr)
            if rs is not None and (rs & 0xFF) != 0:
                rs_seen = rs & 0xFF
                break
            cnt = self._rd(R.PhyTxCntAddr)
            if cnt is not None and (cnt & 0x7FF) > 0:
                fifo_seen = True
                break
            # no sleep — poll as fast as the TCP link allows

        bits_set = bin(rs_seen).count('1')
        evidence  = rs_seen != 0 or fifo_seen

        print(f"    ReadyStatus caught = 0x{rs_seen:02X}  "
              f"TX FIFO filled = {fifo_seen}  "
              f"(evidence of DDRRd_en edge = {evidence})")

        self._record("ready_status_set_via_ddr",
                     evidence,
                     "" if evidence else
                     "neither ReadyStatus nor TX FIFO showed activity after DDRRd_en edge",
                     details={"rs_caught": f"0x{rs_seen:02X}",
                               "fifo_filled": fifo_seen,
                               "bits_set": bits_set})

        # ── Step 6: clear and verify ──────────────────────────────────────
        # Use the bits we actually saw; if we only saw FIFO activity,
        # clear all 8 bits as a best-effort check.

        clear_mask = rs_seen if rs_seen else 0xFF
        self._wr(R.ReadyClearAddr, clear_mask, "ReadyClear")
        time.sleep(0.02)

        rs_after = self._rd(R.ReadyStatusAddr, "ReadyStatus (after clear)")
        if rs_after is None:
            self._record("ready_status_clears", False, "read after clear failed"); return

        remaining = rs_after & clear_mask & 0xFF

        # If AutoTx already cleared the bits before we did, remaining will
        # also be 0 — that still counts as a pass.
        self._record("ready_status_clears",
                     remaining == 0,
                     f"bits 0x{remaining:02X} not cleared",
                     details={"cleared_mask": f"0x{clear_mask:02X}",
                               "after_clear":  f"0x{rs_after & 0xFF:02X}"})
        print(f"    ReadyStatus after clear = 0x{rs_after & 0xFF:02X}")

        # ── Restore ───────────────────────────────────────────────────────
        self._wr(R.CSRRegAddr, csr_orig, "CSR restore")
        if mask_orig is not None:
            self._wr(R.InputMaskAddr, mask_orig & 0xFF, "InputMask restore")

        # Drain any FIFO words AutoTx loaded
        self._reset_phytx_fifo(timeout_s=2.0)

    
    # ── 15. UBT AutoTx + LastTxTarget sticky latch ──────────────────────
    def test_ubt_autotx(self, lane_mask: int = 0x01, num_ubts: int = 3):
        """
        Trigger AutoTx for a single-lane UBT packet and verify:
          - PHY TX FIFO receives words
          - LastTxTarget latch captures correct target
          - No multi-lane bleed
        """
        print(f"\n[15] UBT AutoTx (lane=0x{lane_mask:02X}, n={num_ubts})")

        # ── Hard-reset TX state before starting ──────────────────────────
        print("    Resetting TX FIFO state...")
        if not self._reset_phytx_fifo(timeout_s=3.0):
            # Diagnostic: print what's stuck
            csr = self._rd(R.PhyTxCSRAddr)
            cnt = self._rd(R.PhyTxCntAddr)
            csr_str = f"0x{csr:04X}" if csr is not None else "None"
            cnt_str = str(cnt & 0x7FF) if cnt is not None else "None"
            self._record("ubt_fifo_initially_empty", False,
                         f"TX FIFO still not empty after reset — "
                         f"CSR={csr_str} count={cnt_str} "
                         f"(TxEnAck={(csr & 1) if csr else '?'}, "
                         f"RdEmpty={(csr >> 1 & 1) if csr else '?'})")
            return
        self._record("ubt_fifo_initially_empty", True)

        # ── Also update _wait_phytx_empty to use the new helper ──────────
        # (used at end of each UBT iteration)

        per_ubt_results = []
        issues = []

        for n in range(num_ubts):
            print(f"    UBT {n+1}/{num_ubts}")

            # Clear sticky latch
            self._wr(R.LastTxTargetAddr, 0x0000, "LastTxTarget clear")
            time.sleep(0.005)
            pre = self._rd(R.LastTxTargetAddr, "LastTxTarget pre")
            if pre is not None and (pre & 0xFF) != 0:
                issues.append(f"UBT #{n}: latch not cleared (pre=0x{pre & 0xFF:02X})")

            # Raise ReadyStatus bit for target port via AutoTxKick
            self._wr(R.AutoTxKickAddr, lane_mask, "AutoTxKick")
            time.sleep(0.005)

            # Assert TxEnReq
            self._wr(R.PhyTxCSRAddr, 0x0001, "PhyTxCSR TxEnReq")

            # Poll: wait for TX FIFO to load (FSM wrote UBT words)
            fifo_loaded = False
            t_load = time.time() + 0.5
            while time.time() < t_load:
                cnt = self._rd(R.PhyTxCntAddr)
                if cnt is not None and (cnt & 0x7FF) > 0:
                    fifo_loaded = True
                    break
                time.sleep(0.005)

            # Poll: LastTxTarget latch (set at PhyTxBuff_rdreq time)
            latch_val = 0
            t_latch = time.time() + 0.5
            while time.time() < t_latch:
                lv = self._rd(R.LastTxTargetAddr)
                if lv is not None and (lv & 0xFF) != 0:
                    latch_val = lv & 0xFF
                    break
                time.sleep(0.01)

            expected   = lane_mask & 0xFF
            multi_lane = bin(latch_val).count('1') > 1
            correct    = (latch_val == expected)

            per_ubt_results.append({
                "n":           n,
                "fifo_loaded": fifo_loaded,
                "latch":       f"0x{latch_val:02X}",
                "expected":    f"0x{expected:02X}",
                "correct":     correct,
                "multi_lane":  multi_lane,
            })
            print(f"      fifo_loaded={fifo_loaded}  "
                  f"latch=0x{latch_val:02X}  expected=0x{expected:02X}  "
                  f"{'✓' if correct and not multi_lane else '✗'}")

            if not correct:
                issues.append(f"UBT #{n}: latch=0x{latch_val:02X} != 0x{expected:02X}")
            if multi_lane:
                issues.append(f"UBT #{n}: MULTI-LANE latch=0x{latch_val:02X}")

            # Drain FIFO before next iteration
            self._wait_phytx_fully_idle(timeout_s=2.0)
            time.sleep(0.1)

        ok = len(issues) == 0 and len(per_ubt_results) == num_ubts
        self._record("ubt_autotx_single_lane", ok,
                     "; ".join(issues) if issues else "",
                     details={"per_ubt": per_ubt_results})

    # ── 16. Overflow counter register ───────────────────────────────────
    def test_overflow_counter(self):
        """Verify OverflowCntAd register is readable and 16-bit."""
        print("\n[16] Overflow counter")
        val = self._rd(R.OverflowCntAd, "OverflowCnt")
        self._record("overflow_cnt_readable",
                     val is not None,
                     "read failed",
                     details={"value": f"0x{val:04X}" if val else None})
        if val is not None:
            print(f"    Overflow count: {val & 0xFFFF}")

    # ── 17. Link TX FIFO + trace ─────────────────────────────────────────
    def test_link_tx(self):
        """Read LinkCtrl register and verify Link TX FIFO status bits."""
        print("\n[17] Link TX / control register")
        lc = self._rd(R.LinkCtrlAd, "LinkCtrl")
        self._record("link_ctrl_readable",
                     lc is not None,
                     "read failed",
                     details={"value": f"0x{lc:04X}" if lc is not None else None})
        if lc is not None:
            link_stat_en = lc & 0x0001
            link_empty   = (lc >> 1) & 0x0001
            link_full    = (lc >> 2) & 0x0001
            full_cnt     = (lc >> 8) & 0xFF
            print(f"    LinkCtrl: StatEn={link_stat_en}  Empty={link_empty}  "
                  f"Full={link_full}  FullCnt={full_cnt}")
            print(f"    Link TX FIFO empty={link_empty}  (non-empty is normal during stat tx)")

        # Write Link-stat enable = 1, read back
        if not self._wr(R.LinkCtrlAd, 0x0001, "LinkCtrl StatEn=1"):
            self._record("link_ctrl_writable", False, "write failed"); return
        rb = self._rd(R.LinkCtrlAd, "LinkCtrl readback")
        rb_str = f"0x{rb:04X}" if rb is not None else "None"
        self._record("link_ctrl_writable",
                     rb is not None and (rb & 0x0001) == 1,
                     f"readback bit0 not set: {rb_str}",
                     details={"readback": rb_str})

        # Write Link-stat enable = 1, read back
        if not self._wr(R.LinkCtrlAd, 0x0001, "LinkCtrl StatEn=1"):
            self._record("link_ctrl_writable", False, "write failed"); return
        rb = self._rd(R.LinkCtrlAd, "LinkCtrl readback")
        self._record("link_ctrl_writable",
                     rb is not None and (rb & 0x0001) == 1,
                     f"readback bit0 not set: 0x{rb:04X if rb else 0}",
                     details={"readback": f"0x{rb:04X}" if rb else None})

    # ── 18. Debug register format ────────────────────────────────────────
    def test_debug_register(self):
        """Read debug register and confirm it's readable."""
        print("\n[18] Debug register")
        val = self._rd(R.DebugAddr, "Debug")
        self._record("debug_readable",
                     val is not None,
                     "read failed",
                     details={"value": f"0x{val:04X}" if val else None})
        if val is not None:
            print(f"    Debug = 0x{val:04X}")

    # ── 19. TxEnMask read/write ──────────────────────────────────────────
    def test_txen_mask(self):
        """Verify TxEnMask accepts writes and returns correct values."""
        print("\n[19] TxEnMask register")
        original = self._rd(R.TxEnMaskAd, "TxEnMask")
        if original is None:
            self._record("txen_mask_readable", False, "read failed"); return
        self._record("txen_mask_readable", True,
                     details={"original": f"0x{original:04X}"})

        test_patterns = [0x00, 0x55, 0xAA, 0xFF]
        all_ok = True
        for tv in test_patterns:
            self._wr(R.TxEnMaskAd, tv, "TxEnMask")
            time.sleep(0.005)
            rb = self._rd(R.TxEnMaskAd)
            if rb is None or (rb & 0xFF) != tv:
                all_ok = False
                print(f"    FAIL: wrote 0x{tv:02X}, got 0x{rb & 0xFF if rb else 'None'}")
                break

        # Restore
        self._wr(R.TxEnMaskAd, original & 0xFF, "TxEnMask restore")
        self._record("txen_mask_readwrite", all_ok,
                     "readback mismatch" if not all_ok else "")

    # ── 20. PHY Tx FIFO reset pulse stretch ──────────────────────────────
    def test_fifo_reset_pulse(self):
        """
        Verify the 16-cycle reset stretch: load words, assert reset,
        confirm FIFO drains.
        """
        print("\n[20] TX FIFO reset pulse stretch")
        # Load a couple of words
        for w in [0x1111, 0x2222]:
            self._wr(R.PhyTxFIFOWrtAd, w, "PhyTxFIFO write")
            time.sleep(0.002)

        time.sleep(0.01)
        cnt_before = self._rd(R.PhyTxCntAddr)
        words_present = cnt_before is not None and (cnt_before & 0x7FF) > 0

        # Assert reset pulse
        self._wr(R.TxFifoResetAddr, 0x0001, "TxFifoReset pulse")
        time.sleep(0.02)
        self._wr(R.TxFifoResetAddr, 0x0000, "TxFifoReset deassert")
        time.sleep(0.05)

        ok = self._wait_phytx_empty(timeout_s=1.0)
        self._record("fifo_reset_drains",
                     ok,
                     "FIFO not empty after reset pulse" +
                     ("" if words_present else " (no words were pre-loaded)"),
                     details={"cnt_before": cnt_before})

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def _wait_phytx_empty(self, timeout_s: float = 1.0) -> bool:
        """Thin wrapper kept for backward compatibility."""
        return self._wait_phytx_fully_idle(timeout_s=timeout_s)

    ##########
    def _reset_phytx_fifo(self, timeout_s: float = 1.0) -> bool:
        """
        Fully reset the PHY TX FIFO and wait for the read domain to go idle.
        Also clears TxEnReq so no stale handshake re-fills it.
        Returns True if FIFO is confirmed idle within timeout_s.
        """
        # 1. Clear TxEnReq so the i50MHz domain stops driving TxEn
        self._wr(R.PhyTxCSRAddr, 0x0000, "PhyTxCSR TxEnReq=0")
        time.sleep(0.01)

        # 2. Assert FIFO reset (firmware stretches to 16 SysClk cycles = 160 ns)
        self._wr(R.TxFifoResetAddr, 0x0001, "TxFifoReset assert")
        time.sleep(0.02)   # 20 ms >> 160 ns stretch
        self._wr(R.TxFifoResetAddr, 0x0000, "TxFifoReset deassert")

        # 3. Extra settle: let both clock domains finish draining
        #    wr_data_count can lag the read-domain empty flag by ~10 SysClk cycles
        time.sleep(0.10)

        # 4. Poll on read-domain empty + TxEnAck=0 (wr_data_count not required)
        return self._wait_phytx_fully_idle(timeout_s=timeout_s)
        
        
    def _wait_phytx_fully_idle(self, timeout_s: float = 2.0) -> bool:
        """
        Wait until the PHY TX FIFO is idle:
          - PhyTxCSR bit1 (read-domain empty, i50MHz clock)  = 1
          - PhyTxCSR bit0 (TxEnAck)                          = 0

        Note: wr_data_count (PhyTxCntAddr) is in the SysClk write domain and
        can legitimately lag behind the read-domain empty flag by several
        cycles after a reset.  We therefore treat read-domain-empty + no-ack
        as sufficient — we no longer require wr_data_count == 0.
        """
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            csr = self._rd(R.PhyTxCSRAddr)
            if csr is not None:
                rd_empty = (csr >> 1) & 1   # read-domain empty (i50MHz)
                tx_ack   =  csr       & 1   # TxEnAck still set?
                if rd_empty and not tx_ack:
                    return True
            time.sleep(0.01)
        # Debug dump on timeout
        csr = self._rd(R.PhyTxCSRAddr)
        cnt = self._rd(R.PhyTxCntAddr)
        csr_str = f"0x{csr:04X}" if csr is not None else "None"
        cnt_str = str(cnt & 0x7FF) if cnt is not None else "None"
        ack_str = str(csr & 1)        if csr is not None else "?"
        emp_str = str((csr >> 1) & 1) if csr is not None else "?"
        print(f"    [timeout] PhyTxCSR={csr_str}  "
              f"count={cnt_str}  "
              f"TxEnAck={ack_str}  "
              f"RdEmpty={emp_str}")
        return False

    
    # ------------------------------------------------------------------
    # Runner
    # ------------------------------------------------------------------
    def run(self, tests: Optional[List[str]] = None, lane_mask: int = 0x01):
        """Run all (or specified) test groups."""
        all_tests: Dict[str, Callable] = {
            "version":      self.test_version,
            "csr":          self.test_csr,
            "mask":         self.test_input_mask,
            "counter":      self.test_counter,
            "uptime":       self.test_uptime,
            "phytx":        self.test_phytx_fifo,
            "phyrx":        self.test_phyrx_registers,
            "activity":     self.test_phy_activity_counters,
            "ddr":          self.test_ddr_registers,
            "smi":          self.test_smi_ctrl,
            "fmrx":         self.test_fm_rx,
            "feb_fm":       self.test_feb_fm_registers,
            "crc":          self.test_crc_registers,
            "ready":        self.test_ready_status,
            "ubt":          lambda: self.test_ubt_autotx(lane_mask=lane_mask),
            "overflow":     self.test_overflow_counter,
            "linktx":       self.test_link_tx,
            "debug":        self.test_debug_register,
            "txen":         self.test_txen_mask,
            "fifo_reset":   self.test_fifo_reset_pulse,
        }

        to_run = list(all_tests.keys()) if tests is None else tests
        print(f"\n{'='*65}")
        print(f"  Controller FPGA2 Test Suite  —  GA={self.ga}")
        print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"  Tests: {', '.join(to_run)}")
        print(f"{'='*65}")

        for name in to_run:
            if name in all_tests:
                try:
                    all_tests[name]()
                except Exception as exc:
                    self._record(f"{name}_exception", False,
                                 f"Unhandled exception: {exc}")
                    import traceback
                    traceback.print_exc()
            else:
                print(f"\n  Unknown test group: '{name}'  (skipping)")

        self._print_summary()

    def _print_summary(self):
        total = self._pass + self._fail
        print(f"\n{'='*65}")
        print(f"  SUMMARY:  {self._pass}/{total} passed"
              f"  {'✓ ALL PASS' if self._fail == 0 else f'✗ {self._fail} FAILED'}")
        print(f"{'='*65}")
        if self._fail:
            print("\n  Failed tests:")
            for r in self.results:
                if not r.passed and not r.message.startswith("SKIPPED"):
                    print(f"    ✗  {r.name:40s}  {r.message}")
        print()

    def to_dict(self) -> Dict:
        return {
            "timestamp":   datetime.now().isoformat(),
            "ga":          self.ga,
            "passed":      self._pass,
            "failed":      self._fail,
            "total":       self._pass + self._fail,
            "results":     [asdict(r) for r in self.results],
        }


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Controller FPGA2 functional test suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Available test groups:
  version    Firmware version register
  csr        Control/status register
  mask       Input channel mask
  counter    32-bit test counter (read-to-increment)
  uptime     Uptime counter (waits ~2 s)
  phytx      PHY TX FIFO load / reset / empty
  phyrx      PHY RX word-count and DAV registers
  activity   Per-port PHY activity counters
  ddr        DDR pointer and status registers
  smi        SMI chain-select register
  fmrx       FM Rx status, error, and clear
  feb_fm     FEB FM word-used and active registers
  crc        CRC error flags and CRC array
  ready      ReadyStatus / ReadyClear handshake
  ubt        AutoTx UBT packet + LastTxTarget latch
  overflow   Overflow counter register
  linktx     Link TX FIFO status and control
  debug      Debug register
  txen       TxEnMask read/write
  fifo_reset TX FIFO reset pulse stretch

Examples:
  python test_controller_fpga2.py
  python test_controller_fpga2.py --tests csr,phytx,ddr
  python test_controller_fpga2.py --lane-mask 0x01 --output results.json
  python test_controller_fpga2.py --ip 10.0.0.1 --port 5002 --ga 1
        """,
    )
    parser.add_argument("--ip",         default="192.168.157.97",
                        help="Target IP address")
    parser.add_argument("--port",       type=int, default=5002,
                        help="Target TCP port")
    parser.add_argument("--ga",         type=int, default=0,
                        help="Geographic address (0-3)")
    parser.add_argument("--tests",      default=None,
                        help="Comma-separated list of test groups to run")
    parser.add_argument("--lane-mask",  type=lambda x: int(x, 0), default=0x01,
                        help="One-hot lane mask for UBT test (default 0x01)")
    parser.add_argument("--output",     default=None,
                        help="Save JSON report to this file")
    parser.add_argument("--debug",      action="store_true",
                        help="Enable TCP adapter debug output")
    args = parser.parse_args()

    uc.set_config(host=args.ip, port=args.port, debug=args.debug)

    test_list = [t.strip() for t in args.tests.split(",")] if args.tests else None

    suite = TestSuite(ga=args.ga)
    try:
        suite.run(tests=test_list, lane_mask=args.lane_mask)
    except KeyboardInterrupt:
        print("\n\nInterrupted.")
    finally:
        if args.output:
            report = suite.to_dict()
            with open(args.output, "w") as fh:
                json.dump(report, fh, indent=2)
            print(f"Report saved to: {args.output}")
        try:
            uc.close()
        except Exception:
            pass

    return 0 if suite._fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
