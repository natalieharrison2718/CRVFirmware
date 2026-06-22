Review this PR as a firmware/FPGA reviewer.

Focus on:
- clock domain crossings
- reset behavior
- state machine changes
- register interface changes
- hidden timing assumptions
- unsafe status/control crossings

Please separate findings into:
- blocking issues
- medium-risk concerns
- low-risk cleanup suggestions

For each finding, include file and line references and explain the hardware risk.