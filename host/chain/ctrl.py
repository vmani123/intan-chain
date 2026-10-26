"""UART control interface, CONTRACTS.md section 4.

Text protocol: one line out, one line back. The board is authoritative; this
module only formats requests and refuses to guess when a reply is not exactly
what section 4 promises.
"""
from . import contract as C

READ_ONLY = 0x10  # addresses 0x10..0x1f are counters, status and the ID
# The diagnostic list registers (0x20..0x23) sit above the counters but are
# read/write; everything else at or above READ_ONLY answers `err` to a write.
WRITABLE = frozenset(a for a in C.REG.values() if a < READ_ONLY) | frozenset(
    C.REG[n] for n in ("DIAG_PTR", "DIAG_CMD", "DIAG_LEN", "DIAG_CTRL") if n in C.REG)
COUNTERS = ("FRAMES", "DGRAMS", "FIFO_HIGH", "OVERRUNS", "STALLS")


class CtrlError(RuntimeError):
    """The board said no, said nothing, or said something unparseable."""


def addr_of(reg) -> int:
    """Accept either 'SAMPLE_PERIOD' or 0x01, so scripts can use whichever reads better."""
    if isinstance(reg, str):
        try:
            return C.REG[reg]
        except KeyError:
            raise CtrlError(f"unknown register name {reg!r}; known: {sorted(C.REG)}") from None
    if not 0 <= int(reg) <= 0xFF:
        raise CtrlError(f"register address {reg} out of range 0x00..0xff")
    return int(reg)


class Ctrl:
    def __init__(self, port: str = None, baud: int = C.UART_BAUD, timeout: float = 1.0, ser=None):
        if ser is not None:
            self.ser = ser  # FakeSerial, or anything with write/readline
        else:
            # Imported here so report.py, metrics.py and the tests work on a
            # machine that has never seen a serial port.
            import serial
            self.ser = serial.Serial(port, baud, timeout=timeout)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        self.ser.close()

    def _cmd(self, line: str) -> str:
        self.ser.reset_input_buffer()  # the board never speaks unsolicited, so anything pending is stale
        self.ser.write((line + "\n").encode("ascii"))
        raw = self.ser.readline()
        if not raw.endswith(b"\n"):
            raise CtrlError(f"no complete reply to {line!r} before the serial timeout (got {raw!r})")
        reply = raw.decode("ascii", "replace").strip()
        if reply == "err":
            raise CtrlError(f"board rejected {line!r}")
        return reply

    def read(self, reg) -> int:
        addr = addr_of(reg)
        reply = self._cmd(f"R {addr:02x}")
        got, sep, val = reply.partition("=")
        try:
            if sep != "=" or len(val) != 8 or len(got) != 2:
                raise ValueError
            if int(got, 16) != addr:
                raise CtrlError(f"reply is for register 0x{got}, asked for 0x{addr:02x}")
            return int(val, 16)
        except ValueError:
            raise CtrlError(f"malformed reply to read of 0x{addr:02x}: {reply!r} "
                            f"(want 'aa=vvvvvvvv')") from None

    def write(self, reg, value: int) -> None:
        addr = addr_of(reg)
        reply = self._cmd(f"W {addr:02x} {value & 0xFFFFFFFF:08x}")
        if reply != "ok":
            raise CtrlError(f"unexpected reply to write of 0x{addr:02x}: {reply!r} (want 'ok')")

    def check_id(self) -> int:
        got = self.read("ID")
        if got != C.ID_EXPECTED:
            raise CtrlError(f"ID register reads 0x{got:08x}, expected 0x{C.ID_EXPECTED:08x}: "
                            f"wrong bitstream, or the board is not talking to us")
        return got

    def counters(self) -> dict:
        return {name: self.read(name) for name in COUNTERS}

    def read_many(self, regs) -> dict:
        """{name or address: value}, one round trip each.

        Deliberately sequential: the board parses a line only while it is not
        sending a reply, so lines sent back to back would be dropped.
        """
        return {reg: self.read(reg) for reg in regs}

    def resync(self, settle: float = 0.05) -> None:
        """Terminate any half-typed line on the board and drop what it says back.

        Someone at `screen` may have left bytes in the board's line buffer; the
        first real command would then be parsed as garbage and answer `err`.
        """
        import time
        self.ser.write(b"\n")
        if settle:
            time.sleep(settle)
        self.ser.reset_input_buffer()

    def clear_counters(self) -> None:
        self.write("CTRL", self.read("CTRL") | 0x2)  # bit1 is self-clearing

    def set_run(self, on: bool) -> None:
        ctrl = self.read("CTRL")
        self.write("CTRL", ctrl | 0x1 if on else ctrl & ~0x1)


class FakeSerial:
    """The board side of section 4, in memory.

    Kept next to the host side rather than in the tests so that this file shows
    both halves of the protocol: if the FPGA disagrees with this class, one of
    the two is wrong about CONTRACTS.md.
    """

    def __init__(self):
        self.regs = {addr: 0 for addr in C.REG.values()}
        self.regs[C.REG["ID"]] = C.ID_EXPECTED
        self.log = []
        self.reply_override = None  # test hook: answer the next line with this instead
        self._out = b""

    # --- pyserial-shaped surface -------------------------------------------
    def reset_input_buffer(self):
        self._out = b""

    def write(self, data: bytes) -> int:
        for line in data.decode("ascii", "replace").split("\n"):
            if line.strip():
                self._out += (self._handle(line.strip()) + "\n").encode("ascii")
        return len(data)

    def readline(self) -> bytes:
        line, nl, rest = self._out.partition(b"\n")
        if not nl:
            return b""  # empty means timeout, same as pyserial
        self._out = rest
        return line + nl

    def close(self):
        pass

    # --- board behaviour ----------------------------------------------------
    def _handle(self, line: str) -> str:
        self.log.append(line)
        if self.reply_override is not None:
            reply, self.reply_override = self.reply_override, None
            return reply
        parts = line.split()
        # Hex in is accepted in either case (a human on `screen` should not be
        # tripped by shift); hex out is always lowercase, as section 4 says.
        try:
            if len(parts) == 2 and parts[0] == "R" and len(parts[1]) == 2:
                addr = int(parts[1], 16)
                if addr not in self.regs:
                    return "err"
                return f"{addr:02x}={self.regs[addr]:08x}"
            if len(parts) == 3 and parts[0] == "W" and len(parts[1]) == 2 and len(parts[2]) == 8:
                addr, val = int(parts[1], 16), int(parts[2], 16)
                if addr not in self.regs or addr not in WRITABLE:
                    return "err"  # counters and ID are read-only
                if addr == C.REG["CTRL"] and val & 0x2:
                    for name in COUNTERS:
                        self.regs[C.REG[name]] = 0
                    val &= ~0x2  # self-clearing
                self.regs[addr] = val
                return "ok"
        except ValueError:
            return "err"
        return "err"
