Using the CDC findings for ROC/FPGA2/Controller_FPGA2.vhd, generate the smallest safe patch for the TxEnAck crossing only.

Requirements:
- preserve existing naming/style
- add a 2-FF synchronizer in the correct destination clock domain
- update only unsafe consumers
- document reset-safe default behavior
- do not refactor unrelated logic
- explain why the latency is acceptable