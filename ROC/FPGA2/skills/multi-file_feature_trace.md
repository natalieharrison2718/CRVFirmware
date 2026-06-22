Trace how PHY transmit enable flows across the CRVFirmware repo.

Start from the CSR write/readback path and follow:
- register decode
- handshake generation
- CDC into PHY-side logic
- FIFO dependencies
- output gating

I want a structured explanation with:
- file-by-file path
- signal names
- domain ownership
- where the flow could stall or misbehave