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
import json
import mimetypes
import os
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
        with self._lock:
            return self._holder


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
    ):
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
                else:
                    self._send_static(route)

            def do_POST(self):
                route = urlparse(self.path).path
                if route != '/api/input':
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
