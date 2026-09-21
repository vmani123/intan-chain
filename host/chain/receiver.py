"""UDP data receiver and the echo probe, host side.

The data socket is deliberately dumb: timestamp the datagram and put it in a
list. Parsing during capture would put Python work between two packets at
several thousand datagrams per second and turn host jitter into apparent loss.
"""
import socket
import struct
import threading
import time

from . import contract as C

PROBE_FMT = "<IQ"  # seq, host monotonic_ns at send
PROBE_BYTES = struct.calcsize(PROBE_FMT)


def capture(duration_s: float, port: int = C.UDP_PORT, sock=None,
            bufsize: int = 2048) -> list[tuple[float, bytes]]:
    """Collect (t_host_s, datagram) for duration_s. Pass `sock` to use your own."""
    own = sock is None
    if own:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # A big kernel buffer absorbs a Wi-Fi burst while this loop is in
        # Python; without it a scheduling hiccup reads as radio loss.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        sock.bind(("", port))
    out = []
    sock.settimeout(0.2)
    deadline = time.monotonic() + duration_s
    try:
        while time.monotonic() < deadline:
            try:
                data = sock.recv(bufsize)
            except socket.timeout:
                continue
            out.append((time.monotonic(), data))
    finally:
        if own:
            sock.close()
    return out


def echo_probe(host: str, port: int = C.ECHO_PORT, count: int = 20,
               interval: float = 0.05, timeout: float = 0.5) -> list[float]:
    """One-way air latency estimates, in microseconds.

    Half the round trip. That assumes a symmetric path, and a loaded Wi-Fi
    uplink is not symmetric: the uplink queues behind our own data stream while
    the downlink is nearly idle, so this number is biased low for the leg we
    actually care about. It is reported separately from fpga_us and esp_us for
    exactly that reason - those two are exact, this one is an estimate.
    """
    out = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        for seq in range(count):
            t0 = time.monotonic_ns()
            sock.sendto(struct.pack(PROBE_FMT, seq, t0), (host, port))
            while True:
                try:
                    reply, _ = sock.recvfrom(PROBE_BYTES)
                except socket.timeout:
                    break  # dropped probe; a lost probe is not a lost sample
                if len(reply) != PROBE_BYTES:
                    continue
                rseq, rt0 = struct.unpack(PROBE_FMT, reply)
                if rseq != seq:
                    continue  # a late reply to an earlier probe
                out.append((time.monotonic_ns() - rt0) / 2000.0)
                break
            time.sleep(interval)
    finally:
        sock.close()
    return out


def probe_in_background(host: str, **kw) -> tuple[threading.Thread, list]:
    """Run echo_probe next to a capture, so air latency is measured under load."""
    out = []
    th = threading.Thread(target=lambda: out.extend(echo_probe(host, **kw)), daemon=True)
    th.start()
    return th, out


class EchoResponder(threading.Thread):
    """What the ESP does with ECHO_PORT; also what the tests and --dry-run use."""

    def __init__(self, port: int = C.ECHO_PORT, bind: str = ""):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.1)
        self.port = self.sock.getsockname()[1]  # resolves port 0 for tests
        self._done = threading.Event()  # not _stop: Thread already owns that name

    def run(self):
        while not self._done.is_set():
            try:
                data, peer = self.sock.recvfrom(PROBE_BYTES)
            except socket.timeout:
                continue
            except OSError:
                break
            self.sock.sendto(data, peer)  # verbatim: the probe times its own packet
        self.sock.close()

    def stop(self):
        self._done.set()
        self.join(timeout=1.0)
