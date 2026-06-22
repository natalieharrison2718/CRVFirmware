Review reset handling in ROC/FPGA2/Controller_FPGA2.vhd.

Specifically:
- identify all resets and reset-like control signals
- show which are async assert vs sync deassert
- identify which processes use each reset
- flag reset-domain crossings or inconsistent reset assumptions
- flag signals whose reset value is unsafe for hardware behavior
- include exact line references

Output:
1. Reset inventory
2. Safe reset structures
3. Risky reset usage
4. Recommended minimal fixes