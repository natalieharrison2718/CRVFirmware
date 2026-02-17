#!/usr/bin/env python3
"""
Send Auto-TX UBT and verify single-lane transmit on Controller FPGA2.

Defaults prefilled from Proj_Defs:
- AutoTxKickAddr     = 0x04D
- TxEnMaskAd         = 0x00E
- PhyTxCSRAddr       = 0x012
- PhyTxCntAddr       = 0x013
- TxCurrentTargetAddr= 0x04A

What it does:
1) Set TxEnMask to select exactly one FEB lane (one-hot)
2) Kick Auto-TX via AutoTxKickAddr
3) Assert TxEnReq (write bit0=1 to PhyTxCSRAddr)
4) Verify:
   - PhyTx FIFO count increases (UBT words queued)
   - TxEnAck asserts
   - CurrentTarget is one-hot, matching chosen lane

Requires uc_adapter_telnet.py providing:
  - uc_write(a16, data) -> (ok:bool, reply:any)
  - uc_read(a16) or uc_read16(a16) -> (ok:bool, value:int)
"""

import argparse
import sys
import time

# Import telnet adapter
try:
    import uc_adapter_telnet as uc
except Exception as e:
    print(f"Error: could not import uc_adapter_telnet: {e}")
    sys.exit(1)


def compose_a16(ga: int, addr10: int) -> int:
    """12-bit uC address: [GA(11:10)] : [addr(9:0)]"""
    return ((ga & 0x3) << 10) | (addr10 & 0x3FF)


def set_host_port(host: str, port: int) -> None:
    """Patch uc_adapter_telnet HOST/PORT if present."""
    if hasattr(uc, "HOST"):
        uc.HOST = host
    if hasattr(uc, "PORT"):
        uc.PORT = port

def _normalize_read_result(res):
    """
    Normalize different adapter return styles to (ok: bool, value: int).
    Accepts:
      - int (direct register value)
      - str (e.g. "123" or "0x7B")
      - (ok: bool, value: int)
      - (value,) single-element tuple
    """
    # (ok, value) tuple
    if isinstance(res, tuple):
        if len(res) == 2 and isinstance(res[0], bool):
            ok, val = res
            return ok, int(val) if isinstance(val, (int,)) else (ok, None)
        if len(res) == 1:
            # treat single-element tuple as direct value
            try:
                return True, int(res[0])
            except Exception:
                return False, None

    # direct int
    if isinstance(res, int):
        return True, res

    # string value (decimal or hex)
    if isinstance(res, str):
        s = res.strip()
        try:
            val = int(s, 0)  # auto base (handles "123" or "0x7B")
            return True, val
        except Exception:
            return False, None

    # unknown type
    return False, None


def uc_read16(a16: int):
    """
    Read 16-bit register via adapter; tolerant to different adapter APIs.
    Tries uc.uc_read(a16) then uc.uc_read16(a16). Normalizes the result.
    """
    # Try uc_read first
    if hasattr(uc, "uc_read"):
        try:
            res = uc.uc_read(a16)
            return _normalize_read_result(res)
        except Exception:
            pass

    # Fallback to uc_read16
    if hasattr(uc, "uc_read16"):
        try:
            res = uc.uc_read16(a16)
            return _normalize_read_result(res)
        except Exception:
            pass

    return False, None

def popcount8(x: int) -> int:
    x &= 0xFF
    return bin(x).count("1")


def lowest_one_bit_index(mask: int) -> int:
    for i in range(8):
        if mask & (1 << i):
            return i
    return -1


def test_autotx_send_ubt(args):
    set_host_port(args.ip, args.port)

    ga     = args.ga
    mask   = args.kick_mask & 0xFF
    lane   = lowest_one_bit_index(mask)

    if lane < 0 or popcount8(mask) != 1:
        print("ERROR: --kick-mask must be one-hot (choose exactly one lane).")
        return 2

    # Compose addresses
    a_cnt    = compose_a16(ga, args.cnt_addr)     # PhyTxCntAddr
    a_csr    = compose_a16(ga, args.csr_addr)     # PhyTxCSRAddr (bit0=TxEnAck, bit1=Empty)
    a_tgt    = compose_a16(ga, args.target_addr)  # TxCurrentTargetAddr (lower byte = one-hot)
    a_kick   = compose_a16(ga, args.kick_addr)    # AutoTxKickAddr
    a_txmask = compose_a16(ga, args.txmask_addr)  # TxEnMaskAd (lower byte lane enable)

    print("--- Auto-TX UBT Send & Verify ---")
    print(f"GA:             {ga}")
    print(f"Lane (one-hot): bit {lane} (mask 0x{mask:02X})")
    print(f"Kick Addr:      0x{args.kick_addr:03X}")
    print(f"TxEnMask Addr:  0x{args.txmask_addr:03X}")
    print(f"PhyTxCSR Addr:  0x{args.csr_addr:03X}")
    print(f"PhyTxCnt Addr:  0x{args.cnt_addr:03X}")
    print(f"Target Addr:    0x{args.target_addr:03X}")
    print(f"Endpoint:       {args.ip}:{args.port}")
    print("---------------------------------")

    # 0) Set TxEnMask to exactly one lane (ensure single-lane TX)
    ok, reply = uc.uc_write(a_txmask, mask)
    if not ok:
        print(f"FAIL: write TxEnMaskAd addr=0x{a_txmask:03X} data=0x{mask:02X} reply={reply}")
        return 2
    print(f"TxEnMask set OK: 0x{mask:02X}")

    # 1) Read initial FIFO count
    ok, cnt0 = uc_read16(a_cnt)
    if not ok or cnt0 is None:
        print("FAIL: read PhyTxCntAddr before kick")
        return 2
    cnt0 &= 0x07FF  # 11-bit count per VHDL
    print(f"Initial FIFO count: {cnt0}")

    # 2) Kick Auto-TX to queue UBT words
    ok, reply = uc.uc_write(a_kick, mask)
    if not ok:
        print(f"FAIL: kick write addr=0x{a_kick:03X}, data=0x{mask:02X}, reply={reply}")
        return 2
    print(f"Kick OK: addr=0x{a_kick:03X} data=0x{mask:02X} reply={reply}")

    # 3) Assert TxEnReq (write bit0=1 to PhyTxCSRAddr)
    ok, reply = uc.uc_write(a_csr, 0x0001)
    if not ok:
        print(f"FAIL: assert TxEnReq addr=0x{a_csr:03X} data=0x0001 reply={reply}")
        return 2
    print("TxEnReq asserted")

    # 4a) Poll for FIFO count increase (UBT words queued)
    fifo_ok = False
    cnt1 = cnt0
    t0 = time.time()
    while time.time() - t0 < args.timeout_s:
        ok, val = uc_read16(a_cnt)
        if not ok or val is None:
            time.sleep(args.poll_ms / 1000.0)
            continue
        cnt1 = (val & 0x07FF)
        if cnt1 > cnt0:
            fifo_ok = True
            break
        time.sleep(args.poll_ms / 1000.0)
    print(f"Post-kick FIFO count: {cnt1} (delta {cnt1 - cnt0})")

    # 4b) Poll TxEnAck
    ack_ok = False
    csr = None
    t1 = time.time()
    while time.time() - t1 < args.timeout_s:
        ok, csr = uc_read16(a_csr)
        if not ok or csr is None:
            time.sleep(args.poll_ms / 1000.0)
            continue
        tx_en_ack = (csr & 0x0001) != 0      # bit0 = TxEnAck
        tx_empty  = (csr & 0x0002) != 0      # bit1 = PhyTxBuff_Empty
        if tx_en_ack:
            ack_ok = True
            break
        time.sleep(args.poll_ms / 1000.0)
    print(f"TxEnAck: {'YES' if ack_ok else 'NO'} (Empty={'YES' if (csr and ((csr & 0x2)!=0)) else 'NO'})")

    # 4c) Sample CurrentTarget; require one-hot match to requested lane
    samples = []
    for _ in range(args.samples):
        ok, tgt = uc_read16(a_tgt)
        if not ok or tgt is None:
            samples.append(None)
        else:
            tgt_mask = tgt & 0x00FF
            samples.append(tgt_mask)
        time.sleep(args.poll_ms / 1000.0)

    mismatches = 0
    nonzero_seen = 0
    for m in samples:
        if m is None:
            mismatches += 1
            continue
        if m == 0:
            continue
        nonzero_seen += 1
        if popcount8(m) != 1 or m != (1 << lane):
            mismatches += 1

    print(f"Target samples: {samples}")
    print(f"Non-zero target samples: {nonzero_seen}, mismatches: {mismatches}")
    if nonzero_seen == 0:
        print("WARN: did not catch an active transmit target; sampling may have missed the TX window.")

    # Decide PASS/FAIL
    pass_fifo = fifo_ok or (cnt1 > cnt0)
    pass_ack  = ack_ok
    pass_lane = mismatches == 0

    if not pass_fifo:
        print("FAIL: FIFO did not increase (UBT words not queued)")
    if not pass_ack:
        print("FAIL: TxEnAck did not assert")
    if not pass_lane:
        print("FAIL: CurrentTarget was not one-hot matching requested lane during samples")

    if pass_fifo and pass_ack and pass_lane:
        print("PASS: UBT queued and single-lane transmit confirmed.")
        return 0
    return 1


def parse_args():
    p = argparse.ArgumentParser(description="Send Auto-TX UBT and verify single-lane transmit")
    p.add_argument("--ga", type=int, default=0, help="Geographic address (0..3)")
    # Defaults from Proj_Defs
    p.add_argument("--kick-addr",    type=lambda s: int(s, 0), default=0x04D, help="AutoTxKickAddr (10-bit)")
    p.add_argument("--kick-mask",    type=lambda s: int(s, 0), required=True, help="One-hot kick mask (lower byte)")
    p.add_argument("--txmask-addr",  type=lambda s: int(s, 0), default=0x00E, help="TxEnMaskAd (10-bit)")
    p.add_argument("--csr-addr",     type=lambda s: int(s, 0), default=0x012, help="PhyTxCSRAddr (10-bit)")
    p.add_argument("--cnt-addr",     type=lambda s: int(s, 0), default=0x013, help="PhyTxCntAddr (10-bit)")
    p.add_argument("--target-addr",  type=lambda s: int(s, 0), default=0x04A, help="TxCurrentTargetAddr (10-bit)")
    p.add_argument("-i", "--ip", default="192.168.157.97", help="Target IP")
    p.add_argument("-p", "--port", type=int, default=5002, help="Target Port")
    p.add_argument("--timeout-s", type=float, default=1.0, help="Poll timeout seconds")
    p.add_argument("--poll-ms", type=int, default=20, help="Poll interval milliseconds")
    p.add_argument("--samples", type=int, default=12, help="Number of CurrentTarget samples after kick")
    return p.parse_args()


def main():
    args = parse_args()
    rc = test_autotx_send_ubt(args)
    sys.exit(rc)


if __name__ == "__main__":
    main()
