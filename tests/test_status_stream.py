"""The status stream: every frame is used, and each sample is timed by the controller's frame counter."""
import socket
import struct
import threading
import time
from types import SimpleNamespace

import pytest
from conftest import wait_until

from fairino_gcode import fairino_CNDE_listener as cnde
from fairino_gcode import robot_monitor as rm

PERIOD = 0.05


def data_frame(count, x):
    """An output-data frame carrying one DOUBLE_6 TCP pose."""
    return cnde.build_frame(cnde.FT_OUTPUT_DATA, struct.pack("<6d", x, 0, 0, 180, 0, 0), count)


def test_split_frames_keeps_every_frame_and_the_unfinished_rest():
    a, b, c = data_frame(1, 1.0), data_frame(2, 2.0), data_frame(3, 3.0)
    frames, rest = rm.split_frames(a + b + c[:10])
    assert [f[0] for f in frames] == [1, 2] and rest == c[:10]
    frames, rest = rm.split_frames(rest + c[10:])
    assert [f[0] for f in frames] == [3] and rest == b""
    frames, rest = rm.split_frames(b"junk" + a + b"\x5a")            # leading junk; half a header at the end
    assert [f[0] for f in frames] == [1] and rest == b"\x5a"


def test_several_frames_in_one_datagram_are_all_read():
    """CNDEClient.recv_frame() returned only the first of them: that is how half the samples got lost."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.bind(("127.0.0.1", 0))
    reader = rm.FrameReader(SimpleNamespace(sock=rx, use_tcp=False, ip="127.0.0.1"))
    tx.sendto(data_frame(1, 1.0) + data_frame(2, 2.0) + data_frame(3, 3.0), rx.getsockname())
    tx.sendto(data_frame(4, 4.0), rx.getsockname())
    got = [reader.next(1.0) for _ in range(4)]
    assert [g[1] for g in got] == [1, 2, 3, 4]
    assert got[0][0] == got[1][0] == got[2][0]                        # read in one go
    assert [g[4] for g in got] == [3, 2, 1, 0]          # how many came after it (both datagrams were waiting)
    assert reader.next(0.05) is None
    rx.close()
    tx.close()


def test_a_frame_split_over_two_tcp_reads_is_put_back_together():
    a, b = socket.socketpair()
    reader = rm.FrameReader(SimpleNamespace(sock=a, use_tcp=True, ip="127.0.0.1"))
    stream = data_frame(7, 7.0) + data_frame(8, 8.0) + data_frame(9, 9.0)

    def send():
        for chunk in (stream[:20], stream[20:61], stream[61:]):       # cut through the middle of frames
            b.sendall(chunk)
            time.sleep(0.05)

    threading.Thread(target=send, daemon=True).start()
    got = [reader.next(1.0) for _ in range(3)]
    assert [g[1] for g in got] == [7, 8, 9]
    assert struct.unpack("<6d", got[2][3])[0] == 9.0
    b.close()
    with pytest.raises(ConnectionError):
        reader.next(1.0)
    a.close()


def test_samples_read_in_a_batch_are_timed_a_period_apart():
    clock = rm.SampleClock(PERIOD)
    assert clock.stamp(10, 100.0) == 100.0
    # the program was busy for 0.2 s: four frames are read in one go, the last of them just sent
    times = [clock.stamp(c, 100.25, behind) for c, behind in ((11, 3), (12, 2), (13, 1), (14, 0))]
    assert times == pytest.approx([100.10, 100.15, 100.20, 100.25])
    # a frame missing on the network is counted
    assert clock.stamp(16, 100.35) == pytest.approx(100.35) and clock.lost == 1
    # the counter wraps round after 255
    clock = rm.SampleClock(PERIOD)
    clock.stamp(255, 5.0)
    assert clock.stamp(0, 5.05) == pytest.approx(5.05) and clock.lost == 0
    # never before the sample before it
    assert clock.stamp(1, 5.06, behind=3) > 5.05
    # a message frame uses up a number without being a sample
    clock = rm.SampleClock(PERIOD)
    clock.stamp(1, 1.0)
    clock.counted(2)
    assert clock.stamp(3, 1.05) == pytest.approx(1.05) and clock.lost == 0


def test_datagrams_that_queued_up_are_read_as_one_batch():
    """The program was busy: several datagrams wait on the socket. They are all read at once, and each knows
    how many came after it."""
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for k in range(4):
        tx.sendto(data_frame(k, float(k)), rx.getsockname())
    time.sleep(0.1)
    reader = rm.FrameReader(SimpleNamespace(sock=rx, use_tcp=False, ip="127.0.0.1"))
    got = [reader.next(1.0) for _ in range(4)]
    assert [g[1] for g in got] == [0, 1, 2, 3] and [g[4] for g in got] == [3, 2, 1, 0]
    rx.close()
    tx.close()


def test_the_monitor_gets_every_frame_of_a_bunched_stream(qapp):
    """A controller (or network) that delivers the frames three at a time: every one reaches the GUI, a
    period apart."""
    port_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port_sock.bind(("127.0.0.1", 0))
    port = port_sock.getsockname()[1]
    stop = threading.Event()

    def controller():
        names, client, count = [], None, 0
        port_sock.settimeout(0.005)
        sent, batch, next_t = 0, b"", None
        while not stop.is_set():
            try:
                data, addr = port_sock.recvfrom(65535)
                for _n, ftype, content in cnde.iter_frames(data):
                    count = (count + 1) & 0xFF
                    if ftype == cnde.FT_OUTPUT_CFG:
                        names, client = content[2:].decode().split(","), addr
                        text = ",".join("DOUBLE_6" if n == rm.TCP else "UINT8" for n in names)
                        port_sock.sendto(cnde.build_frame(cnde.FT_MESSAGE, bytes([cnde.MSG_SUCCESS]) + text.encode(),
                                                          count), addr)
                    elif ftype in (cnde.FT_OUTPUT_START, cnde.FT_OUTPUT_STOP):
                        port_sock.sendto(cnde.build_frame(cnde.FT_MESSAGE, bytes([cnde.MSG_SUCCESS]), count), addr)
                        if ftype == cnde.FT_OUTPUT_START:
                            sent, next_t = 0, time.monotonic()
            except socket.timeout:
                pass
            if next_t is not None and sent < 30 and time.monotonic() >= next_t:   # one frame every period...
                next_t += PERIOD
                payload = b"".join(struct.pack("<6d", float(sent), 0, 0, 180, 0, 0) if n == rm.TCP
                                   else struct.pack("<B", 0) for n in names)
                count = (count + 1) & 0xFF
                batch += cnde.build_frame(cnde.FT_OUTPUT_DATA, payload, count)
                sent += 1
                if sent % 3 == 0:                      # ...sent three to a datagram
                    port_sock.sendto(batch, client)
                    batch = b""

    threading.Thread(target=controller, daemon=True).start()
    monitor = rm.RobotMonitor(rm.Target("127.0.0.1", port=port, period_ms=50))
    seen = []
    monitor.sample.connect(seen.append)
    monitor.start()
    try:
        assert wait_until(lambda: len(seen) >= 30, 10), len(seen)
        assert [s[rm.TCP][0] for s in seen[:30]] == [float(k) for k in range(30)]
        gaps = [b[rm.RECEIVED] - a[rm.RECEIVED] for a, b in zip(seen, seen[1:30])]
        assert gaps == pytest.approx([PERIOD] * 29, abs=0.005)        # (the fake sends with a few ms jitter)
        assert monitor.clock.lost == 0
    finally:
        monitor.stop()
        stop.set()
        port_sock.close()
