"""Loopback only: no hardware, no traffic off this machine."""
import socket
import threading
import time

from chain import contract as C
from chain import frame as F
from chain import pattern, receiver


def test_capture_collects_datagrams_with_timestamps():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    payloads = [F.build_datagram(
        [F.build_frame(d * 5 + i, pattern.expected_sample(d * 5 + i)) for i in range(5)])
        for d in range(3)]

    def send():
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for p in payloads:
            time.sleep(0.01)
            tx.sendto(p, ("127.0.0.1", port))
        tx.close()

    th = threading.Thread(target=send)
    th.start()
    got = receiver.capture(1.0, sock=sock)
    th.join()
    sock.close()

    assert [data for _t, data in got] == payloads
    assert all(t0 <= t1 for (t0, _a), (t1, _b) in zip(got, got[1:]))
    frames, _trailer = F.parse_datagram(got[0][1])
    assert len(frames) == C.FRAMES_PER_DGRAM


def test_echo_probe_estimates_one_way_latency():
    responder = receiver.EchoResponder(port=0, bind="127.0.0.1")
    responder.start()
    try:
        est = receiver.echo_probe("127.0.0.1", port=responder.port, count=5, interval=0.0)
    finally:
        responder.stop()
    assert len(est) == 5
    assert all(0 < e < 100000 for e in est)  # loopback: sane microseconds, not nanos or seconds


def test_probe_in_background_runs_next_to_a_capture():
    responder = receiver.EchoResponder(port=0, bind="127.0.0.1")
    responder.start()
    th, out = receiver.probe_in_background("127.0.0.1", port=responder.port,
                                           count=3, interval=0.0)
    th.join(timeout=5.0)
    responder.stop()
    assert len(out) == 3


def test_a_socket_opened_before_the_run_keeps_the_first_datagrams():
    # The sweep opens the data socket before it starts the FPGA. Opening it
    # inside capture() lost the first 4 datagrams of a bench run (seq 0..19):
    # they arrived before the socket existed.
    import socket
    sock = receiver.open_socket(port=0)
    port = sock.getsockname()[1]
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(b"early", ("127.0.0.1", port))
    tx.close()
    got = receiver.capture(0.2, sock=sock)
    sock.close()
    assert [d for _t, d in got] == [b"early"]
