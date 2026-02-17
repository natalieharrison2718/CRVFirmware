# Quick smoke test on the raw TCP CLI: read DebugVersion at addr10=0x99, GA=1, base=0x4000
from uc_adapter_tcp import set_config, compose_a16, uc_read, close

# Always use fixed IP 192.168.157.97 and port 5002
set_config(host="192.168.157.97", port=5002, timeout_s=2.0, eol="\r", addr_base=0x4000, debug=True)
try:
    a16 = compose_a16(1, 0x99)  # GA=1
    val = uc_read(a16)
    print(f"DebugVersion (A16=0x{a16:04X}): 0x{val:04X}")
finally:
    close()