"""Camera tile: the ROS subscription lives exactly as long as a viewer streams.

Pure transport (CameraRelay + the HTTP stream) and the node wiring, over real
sockets and a real ROS subscription (isolated domain, see test_leader_keys).
"""
import os
import re
import socket
import struct
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_leader_keys import wnode  # noqa: E402,F401  (fixture; sets the ROS isolation)

from gripperx_teleop.web_server import CameraRelay, TeleopWebServer  # noqa: E402

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'gripperx_teleop', 'web')
JPEG = b'\xff\xd8\xff\xc0' + bytes(range(256)) * 40 + b'\xff\xd9'


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def until(cond, timeout=4.0):
    end = time.monotonic() + timeout
    while not cond() and time.monotonic() < end:
        time.sleep(0.02)
    return cond()


def open_stream(port):
    s = socket.create_connection(('127.0.0.1', port), timeout=3)
    s.sendall(b'GET /api/camera/stream HTTP/1.1\r\nHost: x\r\n\r\n')
    return s


def read_until(s, marker, limit=200000):
    buf = b''
    while marker not in buf and len(buf) < limit:
        chunk = s.recv(65536)
        if not chunk:
            break
        buf += chunk
    return buf


def abrupt_close(s):
    # RST instead of FIN: what a killed browser or a dropped link looks like.
    s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))
    s.close()


@pytest.fixture
def server():
    calls = []
    relay = CameraRelay(lambda: calls.append('sub'), lambda: calls.append('unsub'))
    port = free_port()
    srv = TeleopWebServer(sink=lambda *a: None, snapshot=lambda: {}, port=port, camera=relay)
    srv.start()
    yield srv, relay, calls, port
    srv.stop()


def test_subscription_only_while_viewers(server):
    _, relay, calls, port = server
    assert calls == [] and relay.viewers == 0          # nobody streaming: no subscription
    a = open_stream(port)
    assert until(lambda: calls == ['sub'])
    b = open_stream(port)
    assert until(lambda: relay.viewers == 2)
    assert calls == ['sub']                            # one subscription for both
    a.close()
    assert until(lambda: relay.viewers == 1)
    assert calls == ['sub']                            # still one viewer: stays
    b.close()
    assert until(lambda: calls == ['sub', 'unsub'])    # last one gone (quiet stream: no frames)
    assert relay.viewers == 0


def test_abrupt_disconnect_while_frames_flow(server):
    _, relay, calls, port = server
    s = open_stream(port)
    assert until(lambda: calls == ['sub'])
    relay.push(JPEG)
    assert JPEG in read_until(s, JPEG)
    abrupt_close(s)
    for _ in range(40):                                # keep feeding: the write must fail
        relay.push(JPEG)
        time.sleep(0.02)
    assert until(lambda: calls == ['sub', 'unsub'])


def test_jpeg_bytes_unchanged_and_only_newest_frame(server):
    _, relay, calls, port = server
    s = open_stream(port)
    assert until(lambda: relay.viewers == 1)
    old, new = JPEG + b'old', JPEG + b'new'
    relay.push(old)
    relay.push(new)                                    # burst before the viewer wakes up
    data = read_until(s, b'new')
    assert b'multipart/x-mixed-replace' in data and b'image/jpeg' in data
    assert new in data
    s.close()


def test_relay_resets_when_resubscribed():
    relay = CameraRelay(lambda: None, lambda: None)
    t = relay.acquire()
    relay.push(b'x')
    relay.release(t)
    t = relay.acquire()
    assert relay.status()['age_sec'] is None           # no frame from the previous session
    relay.release(t)


def test_failing_subscribe_does_not_leave_a_phantom_viewer(server):
    """M3: create_subscription raising must not keep the viewer count above zero."""
    _, relay, calls, port = server
    boom = {'on': True}

    def subscribe():
        if boom['on']:
            raise RuntimeError('create_subscription failed')
        calls.append('sub')

    relay._subscribe = subscribe
    s = open_stream(port)
    assert b'503' in s.recv(200)
    s.close()
    assert relay.viewers == 0 and calls == []
    boom['on'] = False
    s = open_stream(port)                              # recovers: first viewer subscribes
    assert until(lambda: calls == ['sub'] and relay.viewers == 1)
    s.close()
    assert until(lambda: relay.viewers == 0)


def test_failing_unsubscribe_still_drops_the_viewer():
    def bad():
        raise RuntimeError('destroy failed')

    relay = CameraRelay(lambda: None, bad)
    t = relay.acquire()
    with pytest.raises(RuntimeError):
        relay.release(t)
    assert relay.viewers == 0
    relay.release(t)                                   # second release is a no-op
    assert relay.viewers == 0


FRAME_BYTES = 12000


def stamped_frame():
    """JPEG-looking frame whose payload carries the (monotonic) time it was pushed."""
    head = b'\xff\xd8\xff\xc0' + struct.pack('d', time.monotonic())
    return head + b'.' * (FRAME_BYTES - len(head) - 2) + b'\xff\xd9'


def feed(relay, stop, fps=15.0):
    while not stop.is_set():
        relay.push(stamped_frame())
        time.sleep(1.0 / fps)


def test_slow_reader_sees_bounded_frame_age(server):
    """S1. Produced ~180 kB/s, read ~100 kB/s. Without the bounded send buffer the
    kernel queues the excess (auditor: age 0.4 -> 6 s in 25 s, connection never closes).
    Now at most ~(server sndbuf 2x32 kB + client rcvbuf + one frame) / read rate stays
    in flight: about (64 + 32 + 12) kB / 100 kB/s = 1.1 s, so after warm-up the age of
    every frame the client completes must stay below 2.0 s; unbounded growth would
    reach ~5 s by the end of this 8 s run."""
    _, relay, calls, port = server
    stop = threading.Event()
    threading.Thread(target=feed, args=(relay, stop), daemon=True).start()
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16384)   # before connect
    s.connect(('127.0.0.1', port))
    s.sendall(b'GET /api/camera/stream HTTP/1.1\r\nHost: x\r\n\r\n')
    ages, buf, t0, state_ages = [], b'', time.monotonic(), []
    try:
        while time.monotonic() - t0 < 8.0:
            chunk = s.recv(1000)                                 # ~100 kB/s
            time.sleep(0.01)
            buf += chunk
            while True:
                i = buf.find(b'\xff\xd8\xff\xc0')
                if i < 0 or len(buf) < i + FRAME_BYTES:
                    break
                sent = struct.unpack('d', buf[i + 4:i + 12])[0]
                ages.append((time.monotonic() - t0, time.monotonic() - sent))
                buf = buf[i + FRAME_BYTES:]
            if time.monotonic() - t0 > 2.0:
                state_ages.append(relay.status()['age_sec'])
    finally:
        stop.set()
        s.close()
    late = [a for t, a in ages if t > 2.0]
    assert len(late) > 3
    assert max(late) < 2.0, max(late)
    # the tile's age (frame last handed over) is bounded too, i.e. not "live" by luck
    assert max(a for a in state_ages if a is not None) < 2.0


def test_connected_but_never_reading_client_releases_the_subscription(server, monkeypatch):
    """R1. A client that stays connected and never reads (frozen tab, half-open link)
    must not hold the subscription for ever. Timeout patched to 2 s; the bound asserted
    is timeout + 3 s: ~1 s for the buffers to fill at 180 kB/s, the rest is margin."""
    import gripperx_teleop.web_server as ws
    monkeypatch.setattr(ws, 'CAMERA_WRITE_TIMEOUT_SEC', 2.0)
    _, relay, calls, port = server
    stop = threading.Event()
    threading.Thread(target=feed, args=(relay, stop), daemon=True).start()
    s = open_stream(port)                                       # connected, never read
    t0 = time.monotonic()
    try:
        assert until(lambda: calls == ['sub'])
        assert until(lambda: relay.viewers == 0, timeout=5.0), 'viewer still held'
        assert time.monotonic() - t0 < 2.0 + 3.0
        assert until(lambda: calls == ['sub', 'unsub'])
    finally:
        stop.set()
        s.close()


def test_tile_age_does_not_under_report_the_kernel_queue(server):
    """Minor A. Slow reader (~40 kB/s, 4 kB receive buffer): the age the relay reports
    must not be younger than what the client really gets by more than 0.7 s. Counting a
    frame as delivered at write() time under-reports by the whole send queue
    (64 kB / 40 kB/s = 1.6 s); counting it when the queue drained leaves only the
    client's own receive buffer (about 8 kB = 0.2 s) plus sampling slack."""
    _, relay, calls, port = server
    stop = threading.Event()
    threading.Thread(target=feed, args=(relay, stop), daemon=True).start()
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    s.connect(('127.0.0.1', port))
    s.sendall(b'GET /api/camera/stream HTTP/1.1\r\nHost: x\r\n\r\n')
    buf, t0, gaps = b'', time.monotonic(), []
    try:
        while time.monotonic() - t0 < 9.0:
            buf += s.recv(400)
            time.sleep(0.01)                                    # ~40 kB/s
            while True:
                i = buf.find(b'\xff\xd8\xff\xc0')
                if i < 0 or len(buf) < i + FRAME_BYTES:
                    break
                sent = struct.unpack('d', buf[i + 4:i + 12])[0]
                client_age = time.monotonic() - sent
                relay_age = relay.status()['age_sec']
                if time.monotonic() - t0 > 3.0 and relay_age is not None:
                    gaps.append(client_age - relay_age)
                buf = buf[i + FRAME_BYTES:]
    finally:
        stop.set()
        s.close()
    assert len(gaps) > 3
    assert max(gaps) < 0.7, max(gaps)


def test_viewer_that_stops_reading_shows_stale(server):
    """The tile's age follows what the viewer was handed, not what the camera produces."""
    _, relay, calls, port = server
    stop = threading.Event()
    threading.Thread(target=feed, args=(relay, stop), daemon=True).start()
    s = open_stream(port)
    try:
        assert until(lambda: relay.status()['age_sec'] is not None)
        time.sleep(3.0)                                         # never read: buffers fill
        assert relay.status()['age_sec'] > 1.0                  # camera live, viewer not served
        assert relay.status()['arrival_age_sec'] < 0.5
    finally:
        stop.set()
        s.close()


def test_node_creates_and_destroys_the_ros_subscription(wnode):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage

    n, _ = wnode
    topic = n._camera_topic
    ex = SingleThreadedExecutor()
    ex.add_node(n)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    pub = n.create_publisher(
        CompressedImage, topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        assert n._camera_sub is None
        assert n._camera_view()['state'] == 'off'
        port = free_port()
        srv = TeleopWebServer(sink=lambda *a: None, snapshot=n._telemetry, port=port,
                              camera=n._camera, camera_profile=n.request_camera_profile)
        srv.start()
        try:
            s = open_stream(port)
            assert until(lambda: n._camera_sub is not None)
            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = list(JPEG)
            got = b''
            end = time.monotonic() + 5
            while JPEG not in got and time.monotonic() < end:
                pub.publish(msg)
                s.settimeout(0.3)
                try:
                    got += s.recv(65536)
                except socket.timeout:
                    pass
            assert JPEG in got                         # unchanged bytes, ROS -> HTTP
            # 'live' once the viewer's TCP peer has acknowledged a frame (next 0.1 s tick)
            assert until(lambda: n._camera_view()['state'] == 'live', timeout=3.0)
            abrupt_close(s)
            assert until(lambda: n._camera_sub is None)
            assert until(lambda: n._camera_view()['state'] == 'off')
        finally:
            srv.stop()
    finally:
        ex.shutdown()
        t.join(timeout=2)


def test_profile_request_without_camera_node_fails_fast(wnode):
    n, _ = wnode
    t0 = time.monotonic()
    r = n.request_camera_profile('detail')
    assert not r['ok'] and 'not running' in r['detail']
    assert time.monotonic() - t0 < 0.5
    assert not n.request_camera_profile('')['ok']
    # N1: no profile list is kept here; a name is passed on and the camera node refuses
    assert 'camera_profiles' not in {p for p in n._parameters}
    assert 'not running' in n.request_camera_profile('bogus')['detail']
    assert any('camera profile detail' in e['text'] for e in n._telemetry()['events'])


def test_page_wiring():
    html = open(os.path.join(WEB, 'index.html')).read()
    js = open(os.path.join(WEB, 'app.js')).read()
    for ident in ('cam-power', 'cam-view', 'cam-detail', 'cam-img', 'cam-stats', 'cam-msg'):
        assert f'id="{ident}"' in html
    assert re.search(r"'/api/camera/stream", js) and '/api/camera/profile' in js
    assert 'removeAttribute(\'src\')' in js                # Off really closes the stream
    assert not re.search(r'<img[^>]*\bsrc=', html)         # default OFF: no stream in the markup


def test_unanswered_profile_read_times_out(wnode):
    """N3: a GetParameters that never answers is dropped by the watchdog."""
    n, _ = wnode
    removed = []

    class Client:
        def remove_pending_request(self, fut):
            removed.append(fut)

    n._camera_get = Client()
    n._camera_read = ('fut', time.monotonic() - 10.0)
    n._camera_watchdog()
    assert removed == ['fut'] and n._camera_read is None
