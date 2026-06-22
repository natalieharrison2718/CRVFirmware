Review the microcontroller register interface in ROC/FPGA2/Controller_FPGA2.vhd.

I want you to:
- identify all register write decodes and readback mux entries
- trace which internal signals each CSR bit controls
- flag any write path that directly affects a non-SysClk domain without proper CDC handling
- flag any status readback that exposes raw async or unsynchronized signals
- include exact line references

Output:
1. Register map summary
2. Unsafe write-side control paths
3. Unsafe readback/status paths
4. Minimal corrective actions