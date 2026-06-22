I’m debugging this symptom in CRVFirmware:
“PHY transmit sometimes never starts after data is queued.”

Please investigate likely structural causes in the HDL, prioritizing:
- missing or broken handshakes
- CDC issues
- reset release issues
- FIFO empty/full misuse
- state machine wait conditions

Do not guess broadly. Trace the concrete logic paths and cite exact files/lines.
Then rank the top likely root causes.