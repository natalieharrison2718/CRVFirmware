Trace the full lifecycle of TxEnReq and TxEnAck in ROC/FPGA2/Controller_FPGA2.vhd.

I want:
- where each signal is produced
- what clock domain owns it
- where it is consumed
- whether each consumption is CDC-safe
- reset behavior
- handshake timing assumptions
- exact line references

Then tell me whether the handshake is structurally safe, and if not, give the smallest safe fix.