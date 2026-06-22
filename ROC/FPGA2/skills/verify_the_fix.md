Re-review ROC/FPGA2/Controller_FPGA2.vhd after the TxEnAck CDC fix.

Verify:
- all SysClk consumers now use the synchronized copy
- no i50MHz-local logic was incorrectly changed
- reset behavior remains safe
- no new CDC issues were introduced by the fix

Then list:
1. verified resolved items
2. remaining CDC issues
3. follow-up cleanup opportunities