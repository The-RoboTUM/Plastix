#!/usr/bin/env python3
"""
HTTP transport for the browser teleop UI — no ROS, no third-party packages.

Why plain HTTP and not a WebSocket
----------------------------------
A WebSocket would need either a third-party package (`websockets` is on the
laptop but was never a declared dependency of this workspace, and is not
guaranteed on the Pi) or a hand-rolled RFC 6455 framer. Neither belongs in the
path that carries a dead-man switch. What is actually needed here is one
short POST per input change and a one-way status stream, and the standard
library does both:

  POST /api/input      operator -> node   (held-key set + one-shot events)
  GET  /api/telemetry  node -> operator   (Server-Sent Events, text/event-stream)
  GET  /api/camera/stream   camera JPEGs, unchanged (multipart/x-mixed-replace, one
                            connection = one viewer; closing it = viewer gone)
  POST /api/camera/profile  {"profile": "view"|"detail"} -> camera node parameter

Latency on localhost is well under a millisecond per POST, and HTTP/1.1
keep-alive means no TCP handshake per beat.

The safety-relevant idea: the browser does NOT send "key pressed" and later
"key released" and hope both arrive. Every beat carries the COMPLETE set of
keys currently held, and the node treats a beat as a dead-man refresh. A lost
beat, a closed laptop lid, a crashed tab or a dropped Wi-Fi link all look the
same to the node — the set stops being refreshed and goes stale, which is
exactly the terminal node's existing "key repeat stopped" condition (SR-3).
There is no message whose loss can leave the robot driving.

This module knows nothing about ROS or about teleop. It owns the socket, the
session bookkeeping and the exclusive-control rule; the caller supplies a
`sink` (what to do with operator input) and a `snapshot` (what to stream back).
"""
import collections
import fcntl
import json
import mimetypes
import os
import select
import socket
import struct
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

# Largest POST body accepted. A beat is a few hundred bytes; anything above
# this is not our client.
MAX_BODY_BYTES = 8192

# How long a control holder may go silent before another session may take over.
# Deliberately longer than the dead-man window: taking control away from an
# operator who is merely mid-beat would be worse than waiting.
DEFAULT_TAKEOVER_SEC = 2.0

# Camera stream socket: kernel send buffer (the OS doubles it) and the time one frame
# may take to be written before the viewer counts as dead. TO-VERIFY on the robot link.
CAMERA_SNDBUF = 32768
CAMERA_WRITE_TIMEOUT_SEC = 5.0

_STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')


class ControlRegistry:
    """Who is allowed to drive, and who is only watching.

    Two browser tabs pointed at the same node would otherwise fight over the
    dead-man set: tab A holds W and refreshes it, tab B sends an empty set at
    the same rate, and the robot stutters. So exactly one session at a time
    holds control; every other session gets a read-only view of the same
    telemetry.

    The emergency stop is deliberately NOT gated on holding control. Anyone
    looking at the page may stop the robot — that asymmetry is the point.
    """

    def __init__(self, takeover_sec: float = DEFAULT_TAKEOVER_SEC):
        self._lock = threading.Lock()
        self._holder = None
        self._seen = 0.0
        self._takeover_sec = float(takeover_sec)

    def claim(self, session: str, now: float, *, force: bool = False) -> bool:
        with self._lock:
            stale = (now - self._seen) > self._takeover_sec
            if self._holder in (None, session) or stale or force:
                self._holder = session
                self._seen = now
                return True
            return False

    def holder_at(self, now: float):
        """The holder, or None when its lease has lapsed (no refresh for > takeover_sec).

        A lapsed holder is reported free so a page that decided to observe sees
        the seat open and claims it; claim() already grants it after the same
        interval, so the two views cannot disagree.
        """
        with self._lock:
            if (now - self._seen) > self._takeover_sec:
                return None
            return self._holder

    def refresh(self, session: str, now: float) -> bool:
        """True if this session holds control (and its lease was renewed)."""
        with self._lock:
            if self._holder != session:
                return False
            self._seen = now
            return True

    def release(self, session: str) -> None:
        with self._lock:
            if self._holder == session:
                self._holder = None
                self._seen = 0.0

    @property
    def holder(self):
        return self.holder_at(time.monotonic())


class CameraRelay:
    """Newest-frame relay between one ROS subscription and any number of HTTP viewers.

    The subscription exists only while viewers exist: the first acquire() calls
    `subscribe`, the last release() calls `unsubscribe`. Frames are kept as the
    original JPEG bytes; wait_frame() always returns the newest one. That alone
    does NOT keep a slow browser from falling behind: bytes already handed to the
    kernel sit in socket buffers. The stream handler therefore bounds those buffers
    (small SO_SNDBUF, skip a frame when the socket is not writable), and the relay
    tracks per viewer which frame was last handed over completely (delivered()), so
    the tile's frame age is that of what the viewer was actually given.
    """

    def __init__(self, subscribe, unsubscribe):
        self._subscribe = subscribe
        self._unsubscribe = unsubscribe
        self._viewers = 0
        self._next_token = 0
        self._delivered = {}    # token -> rx time of the last frame fully handed over, or None
        self._vlock = threading.Lock()
        self._cv = threading.Condition()
        self._seq = 0
        self._frame = None      # (seq, data, t_rx)
        self._times = []        # rx times of the last frames, for fps

    @property
    def viewers(self) -> int:
        return self._viewers

    def acquire(self):
        """Register a viewer and return its token. If subscribing fails the viewer is
        not counted (an exception here must not leave the count above zero for good)."""
        with self._vlock:
            first = self._viewers == 0
            if first:
                with self._cv:
                    self._frame = None
                    self._times = []
                self._subscribe()   # may raise: nothing is counted yet
            self._viewers += 1
            self._next_token += 1
            token = self._next_token
            self._delivered[token] = None
            return token

    def release(self, token):
        """Unregister a viewer. The count drops before `unsubscribe` runs, so a failing
        unsubscribe cannot keep a phantom viewer alive."""
        with self._vlock:
            if token not in self._delivered:
                return
            del self._delivered[token]
            self._viewers -= 1
            if self._viewers == 0:
                self._unsubscribe()

    def delivered(self, token, t_rx: float):
        """The viewer was handed the frame that arrived at t_rx, completely."""
        with self._vlock:
            if token in self._delivered:
                self._delivered[token] = t_rx

    def push(self, data: bytes):
        now = time.monotonic()
        with self._cv:
            self._seq += 1
            self._frame = (self._seq, data, now)
            self._times.append(now)
            del self._times[:-30]
            self._cv.notify_all()

    def wait_frame(self, last_seq: int, timeout: float):
        """Newest frame with seq > last_seq as (seq, data, t_rx), or None on timeout."""
        with self._cv:
            self._cv.wait_for(
                lambda: self._frame is not None and self._frame[0] > last_seq, timeout)
            if self._frame is not None and self._frame[0] > last_seq:
                return self._frame
            return None

    def status(self) -> dict:
        """fps/bytes describe what arrives from ROS; age_sec is the age of the frame the
        most-behind viewer was last handed, acknowledged by its TCP peer (None until one was), so a viewer
        that is not being served shows as stale even while the camera is live."""
        now = time.monotonic()
        with self._cv:
            frame = self._frame
            times = [t for t in self._times if now - t <= 3.0]
        with self._vlock:
            seen = [t for t in self._delivered.values() if t is not None]
        fps = ((len(times) - 1) / (times[-1] - times[0])
               if len(times) >= 2 and times[-1] > times[0] else 0.0)
        return {
            'viewers': self._viewers,
            'fps': round(fps, 1),
            'age_sec': round(now - min(seen), 2) if seen else None,
            'arrival_age_sec': None if frame is None else round(now - frame[2], 2),
            'bytes': None if frame is None else len(frame[1]),
        }


def _writable(sock) -> bool:
    """True if a send would be accepted without waiting."""
    try:
        return bool(select.select([], [sock], [], 0)[1])
    except (OSError, ValueError):
        return False


def _unsent_bytes(sock) -> int:
    """Bytes still in the kernel send queue (unsent or not yet acknowledged), 0 if unknown."""
    try:
        buf = bytearray(4)
        fcntl.ioctl(sock.fileno(), termios.TIOCOUTQ, buf, True)
        return struct.unpack('i', bytes(buf))[0]
    except (OSError, ValueError):
        return 0


def _peer_closed(sock) -> bool:
    """True if the client has closed its end (a quiet stream cannot notice that by writing)."""
    try:
        ready, _, _ = select.select([sock], [], [], 0)
        if ready:
            return sock.recv(1, socket.MSG_PEEK) == b''
    except (OSError, ValueError):
        return True
    return False


class TeleopWebServer:
    """Serves the UI and bridges browser <-> caller-supplied callbacks.

    `sink(session, keys, events, has_control)` is called for every beat, with
    `keys` the complete set of keys the browser reports as held right now.
    `snapshot()` returns a JSON-serialisable dict streamed to every client.
    """

    def __init__(
        self,
        sink,
        snapshot,
        host: str = '127.0.0.1',
        port: int = 8080,
        stream_hz: float = 20.0,
        takeover_sec: float = DEFAULT_TAKEOVER_SEC,
        logger=None,
        camera=None,
        camera_profile=None,
    ):
        """camera: a CameraRelay, or None (no camera routes).
        camera_profile(name) -> dict, must not block: starts the parameter change.
        """
        self._camera = camera
        self._camera_profile = camera_profile
        self._sink = sink
        self._snapshot = snapshot
        self._host = host
        self._port = int(port)
        self._stream_period = 1.0 / max(1.0, float(stream_hz))
        self._log = logger
        self.control = ControlRegistry(takeover_sec)

        server = self  # closed over by the handler below

        class Handler(BaseHTTPRequestHandler):
            # Keep-alive: without HTTP/1.1 every beat would pay a fresh TCP
            # handshake, which on Wi-Fi is worth more than the whole payload.
            protocol_version = 'HTTP/1.1'

            def log_message(self, fmt, *args):
                pass  # the access log would drown the ROS log at 20 beats/s

            # -- helpers ---------------------------------------------------

            def _send_json(self, payload, status=200):
                body = json.dumps(payload).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def _send_static(self, path):
                name = 'index.html' if path in ('', '/') else path.lstrip('/')
                # Refuse anything that climbs out of the asset directory.
                full = os.path.normpath(os.path.join(_STATIC_DIR, name))
                if not full.startswith(_STATIC_DIR) or not os.path.isfile(full):
                    self._send_json({'error': 'not found'}, status=404)
                    return
                ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
                with open(full, 'rb') as handle:
                    body = handle.read()
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            # -- routes ----------------------------------------------------

            def do_GET(self):
                route = urlparse(self.path).path
                if route == '/api/telemetry':
                    self._stream_telemetry()
                elif route == '/api/camera/stream':
                    self._stream_camera()
                else:
                    self._send_static(route)

            def do_POST(self):
                route = urlparse(self.path).path
                if route not in ('/api/input', '/api/camera/profile'):
                    self._send_json({'error': 'not found'}, status=404)
                    return
                try:
                    length = int(self.headers.get('Content-Length') or 0)
                except ValueError:
                    self._send_json({'error': 'bad length'}, status=400)
                    return
                if length <= 0 or length > MAX_BODY_BYTES:
                    self._send_json({'error': 'bad length'}, status=400)
                    return
                try:
                    message = json.loads(self.rfile.read(length).decode('utf-8'))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self._send_json({'error': 'bad json'}, status=400)
                    return

                if route == '/api/camera/profile':
                    self._camera_profile_post(message)
                    return

                session = str(message.get('session') or '')[:64]
                if not session:
                    self._send_json({'error': 'no session'}, status=400)
                    return
                keys = [str(k)[:16] for k in (message.get('keys') or [])][:16]
                events = [str(e)[:32] for e in (message.get('events') or [])][:16]

                now = time.monotonic()
                if message.get('claim'):
                    granted = server.control.claim(
                        session, now, force=bool(message.get('force'))
                    )
                elif message.get('release'):
                    server.control.release(session)
                    granted = False
                else:
                    granted = server.control.refresh(session, now)

                try:
                    server._sink(session, keys, events, granted)
                except Exception as exc:  # noqa: BLE001 — never kill the socket
                    server._warn(f'teleop web input handler failed: {exc}')

                self._send_json({
                    'ok': True,
                    'control': bool(granted),
                    'holder': server.control.holder,
                })

            def _camera_profile_post(self, message):
                if server._camera_profile is None:
                    self._send_json({'error': 'no camera'}, status=404)
                    return
                try:
                    result = server._camera_profile(str(message.get('profile') or '')[:32])
                except Exception as exc:  # noqa: BLE001
                    server._warn(f'camera profile handler failed: {exc}')
                    result = {'ok': False, 'detail': str(exc)}
                self._send_json(result)

            # -- camera ------------------------------------------------------

            def _stream_camera(self):
                relay = server._camera
                if relay is None:
                    self._send_json({'error': 'no camera'}, status=404)
                    return
                try:
                    token = relay.acquire()
                except Exception as exc:  # noqa: BLE001 -- e.g. the subscription failed
                    server._warn(f'camera stream refused: {exc}')
                    self._send_json({'error': 'camera subscription failed'}, status=503)
                    return
                sock = self.connection
                try:
                    # Delay must not build in the kernel: a small send buffer, and a
                    # frame is only started when the socket can take data now, else it
                    # is skipped (the next wait picks the then-newest one).
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, CAMERA_SNDBUF)
                    # One frame must be written within this, or the viewer is dropped.
                    sock.settimeout(CAMERA_WRITE_TIMEOUT_SEC)
                    boundary = 'gxframe'
                    self.send_response(200)
                    self.send_header(
                        'Content-Type', f'multipart/x-mixed-replace; boundary={boundary}')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.close_connection = True
                    seq = 0
                    sent_total = 0
                    pending = collections.deque()   # (end offset in the byte stream, t_rx)
                    blocked_since = None
                    while not server._stop.is_set():
                        got = relay.wait_frame(seq, 0.1)
                        # A frame counts as delivered when the kernel has no more of its
                        # bytes queued (acknowledged by the peer), not when write() returned.
                        if pending:
                            acked = sent_total - _unsent_bytes(sock)
                            while pending and pending[0][0] <= acked:
                                relay.delivered(token, pending.popleft()[1])
                        if got is None:
                            if _peer_closed(sock):
                                break
                            continue
                        if not _writable(sock):
                            # A connected client that stops reading (frozen tab, half-open
                            # link) must not hold the camera subscription for ever: no
                            # progress for CAMERA_WRITE_TIMEOUT_SEC ends the stream.
                            now = time.monotonic()
                            if blocked_since is None:
                                blocked_since = now
                            elif now - blocked_since > CAMERA_WRITE_TIMEOUT_SEC:
                                break
                            if _peer_closed(sock):
                                break
                            time.sleep(0.02)   # retry with the then-newest frame
                            continue
                        blocked_since = None
                        seq, data, t_rx = got
                        head = (f'--{boundary}\r\nContent-Type: image/jpeg\r\n'
                                f'Content-Length: {len(data)}\r\n\r\n').encode('ascii')
                        part = head + data + b'\r\n'
                        self.wfile.write(part)
                        self.wfile.flush()
                        sent_total += len(part)
                        pending.append((sent_total, t_rx))
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    pass  # tab closed / link dropped / browser not reading
                finally:
                    try:
                        relay.release(token)
                    except Exception as exc:  # noqa: BLE001
                        server._warn(f'camera unsubscribe failed: {exc}')

            # -- SSE -------------------------------------------------------

            def _stream_telemetry(self):
                # No Content-Length: an SSE body ends when the connection does.
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'close')
                self.end_headers()
                self.close_connection = True
                try:
                    while not server._stop.is_set():
                        try:
                            frame = server._snapshot()
                        except Exception as exc:  # noqa: BLE001
                            server._warn(f'teleop web snapshot failed: {exc}')
                            frame = {'error': str(exc)}
                        frame['holder'] = server.control.holder
                        blob = json.dumps(frame)
                        self.wfile.write(f'data: {blob}\n\n'.encode('utf-8'))
                        self.wfile.flush()
                        time.sleep(server._stream_period)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass  # operator closed the tab — nothing to report

        self._handler = Handler
        self._httpd = None
        self._thread = None
        self._stop = threading.Event()

    # -- lifecycle ------------------------------------------------------

    def _warn(self, text):
        if self._log is not None:
            self._log.warning(text)

    @property
    def url(self) -> str:
        shown = 'localhost' if self._host in ('127.0.0.1', '0.0.0.0', '') else self._host
        return f'http://{shown}:{self._port}/'

    def start(self):
        self._httpd = ThreadingHTTPServer((self._host, self._port), self._handler)
        # Otherwise a live SSE stream would hold shutdown open for its full
        # frame period on every quit.
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, kwargs={'poll_interval': 0.2}, daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
