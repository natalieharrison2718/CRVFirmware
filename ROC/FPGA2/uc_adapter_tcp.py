import socket
import time
import re
import select
from typing import Optional, Tuple

# Fixed target IP and default port
HOST = "192.168.157.97"
PORT = 5002
READ_TIMEOUT_S = 3.0
DEBUG = False

# CLI formatting
EOL = "\r"                            # CR line ending (matches programROC)
CMD_READ_FMT  = "RD {addr}"           # e.g., RD 4412
CMD_WRITE_FMT = "WR {addr} {val}"     # e.g., WR 440E 0001
VAL_FORMAT = "hex"                    # "hex", "hexH", or "dec"

# A16 address base (upper nibble); your logs show 0x4000
ADDR_BASE = 0x4000

_sock: Optional[socket.socket] = None

def set_config(host: Optional[str] = None,
               port: Optional[int] = None,
               timeout_s: float = 3.0,
               eol: str = "\r",
               addr_base: int = 0x4000,
               cmd_read_fmt: str = "RD {addr}",
               cmd_write_fmt: str = "WR {addr} {val}",
               val_format: str = "hex",
               debug: bool = False) -> None:
    """Configure TCP CLI adapter. If host/port not provided, keep fixed defaults."""
    global HOST, PORT, READ_TIMEOUT_S, EOL, ADDR_BASE, CMD_READ_FMT, CMD_WRITE_FMT, VAL_FORMAT, DEBUG
    HOST = HOST if host is None else host
    PORT = PORT if port is None else port
    READ_TIMEOUT_S = timeout_s
    EOL = eol
    ADDR_BASE = addr_base & 0xF000
    CMD_READ_FMT = cmd_read_fmt
    CMD_WRITE_FMT = cmd_write_fmt
    VAL_FORMAT = val_format.lower()
    DEBUG = debug

def compose_a16(ga_bits: int, addr10: int) -> int:
    """Compose A16H: ADDR_BASE | (GA<<10) | addr10."""
    return (ADDR_BASE | ((ga_bits & 0x3) << 10) | (addr10 & 0x3FF))

def _format_addr_a16(addr16: int) -> str:
    return f"{addr16 & 0xFFFF:04X}"

def _format_val_d16(val16: int) -> str:
    v = val16 & 0xFFFF
    if VAL_FORMAT == "hexh":
        return f"{v:04X}H"
    elif VAL_FORMAT == "dec":
        return str(v)
    else:
        return f"{v:04X}"

def _ensure_open():
    global _sock
    if _sock is not None:
        return
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.settimeout(READ_TIMEOUT_S)
    s.connect((HOST, PORT))
    _sock = s
    if DEBUG:
        print(f"[TCP] connected to {HOST}:{PORT}")

def _read_reply(deadline: float) -> bytes:
    """
    Read until timeout; prefer a 4-hex-digit word.
    Do NOT stop on '>' alone; keep reading until we see hex or timeout.
    """
    buf = b""
    while time.time() < deadline:
        rlist, _, _ = select.select([_sock], [], [], 0.05)
        if not rlist:
            continue
        try:
            chunk = _sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            if DEBUG:
                try:
                    print("[TCP] recv:", chunk.decode(errors="ignore"))
                except Exception:
                    print("[TCP] recv bytes:", chunk)
            # Stop early only if a hex word appears
            if re.search(rb"\b[0-9A-Fa-f]{4}\b", buf):
                break
        except socket.timeout:
            continue
    return buf

def send_cli(cmd: str) -> str:
    """Send a raw CLI command and return reply text (no prompt management)."""
    _ensure_open()
    if DEBUG:
        print("[TCP] send:", cmd)
    _sock.sendall((cmd + EOL).encode())
    deadline = time.time() + READ_TIMEOUT_S
    data = _read_reply(deadline)
    text = data.replace(b"\r", b"").decode(errors="ignore").strip()
    # Strip echoed command if present
    if text.startswith(cmd):
        text = text[len(cmd):].strip()
    if DEBUG:
        print("[TCP] reply:", text)
    return text

def uc_read(addr16: int) -> int:
    """Read a 16-bit register via TCP CLI; reply must contain a 4-hex-digit word."""
    _ensure_open()
    a16 = _format_addr_a16(addr16)
    cmd = CMD_READ_FMT.format(addr=a16)

    # First attempt
    text = send_cli(cmd)
    m = re.search(r"\b([0-9A-Fa-f]{4})\b", text)
    if m:
        return int(m.group(1), 16) & 0xFFFF

    # One retry (device sometimes prints '>' first, then number)
    time.sleep(0.05)
    text2 = send_cli(cmd)
    m2 = re.search(r"\b([0-9A-Fa-f]{4})\b", text2)
    if m2:
        return int(m2.group(1), 16) & 0xFFFF

    # Decimal fallback
    m = re.search(r"\b([0-9]{1,5})\b", text) or re.search(r"\b([0-9]{1,5})\b", text2)
    if m:
        return int(m.group(1)) & 0xFFFF

    raise RuntimeError(f"Read failed: {text2 or text}")

def uc_write(addr16: int, value16: int) -> Tuple[bool, str]:
    """Write a 16-bit value via TCP CLI; returns (success, reply_text)."""
    _ensure_open()
    a16 = _format_addr_a16(addr16)
    d16 = _format_val_d16(value16)
    text = send_cli(CMD_WRITE_FMT.format(addr=a16, val=d16))
    # Treat any reply with ERR/?PARM as failure; '>' or blank are OK
    if text.upper().find("ERR") >= 0 or text.upper().find("?PARM") >= 0:
        return (False, text)
    return (True, text)

def close():
    global _sock
    try:
        if _sock is not None:
            _sock.close()
    finally:
        _sock = None