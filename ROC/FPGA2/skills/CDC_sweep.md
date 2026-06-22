Do a firmware-focused CDC review of ROC/FPGA2/Controller_FPGA2.vhd.

Requirements:
- Identify every clock domain and reset domain in the file.
- List all cross-domain signals.
- For each crossing, classify it as:
  - safe
  - unsafe
  - ambiguous/manual review
- Distinguish between:
  - 2-FF synchronizers
  - pulse/level handshakes
  - async FIFO crossings
  - raw unsynchronized status/control reads
  - multi-bit bus crossings
- Include:
  - source process/domain
  - destination process/domain
  - reset/default behavior
  - exact signal names
  - exact file/line references

Output format:
1. Domain map
2. Verified safe crossings
3. CDC violations
4. Manual-review items
5. Prioritized minimal fixes

Do not propose broad refactors.