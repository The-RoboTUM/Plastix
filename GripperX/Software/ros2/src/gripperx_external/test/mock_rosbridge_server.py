#!/usr/bin/env python3
"""Pure-python rosbridge stand-in. No ROS, no rclpy, no DDS.

Exists so rollout stages 0-3 need neither the Octopus machine nor a network. It
speaks the same verb subset ``rosbridge_client`` speaks - ``subscribe``,
``unsubscribe``, ``advertise``, ``unadvertise``, ``publish`` - and loops published
frames back to every client subscribed to that topic, which is what a real
rosbridge does via the ROS graph.

Same library pin as the client: **python3-websockets 10.4 from apt**. Legacy
``websockets.server.serve`` only; 10.4 has neither ``websockets.asyncio`` nor
``websockets.sync``. See the long note in ``rosbridge_client.py``.

DELIBERATE FIDELITY CHOICES - each one is a place where a convenient mock would
lie about the counterpart:

* **No latched replay by default.** Real rosbridge subscribes to the ROS graph with
  its own default (VOLATILE) QoS, so a TRANSIENT_LOCAL publisher's latched sample is
  *not* replayed to a client that connects late - the Octopus's own interface doc
  says so. ``--latch-replay`` exists only to demonstrate the difference; leaving it
  off is the honest default, and it is why ``fake_octopus.py`` republishes at 1 Hz
  instead of relying on a latch.
* **Publish without advertise is an error status, not a silent accept.** rosbridge
  cannot infer a message type it was never told, so accepting it would hide a client
  bug until integration day.
* **The publisher receives its own message** if it also subscribed, matching ROS
  rather than a naive "broadcast to others" loop.
* **Status frames only for warnings and errors.** Real rosbridge is quiet on
  success unless the client raises its status level. ``--verbose-status`` adds
  info-level acks for debugging.

Test control surface (used by ``check_rosbridge_client.py`` and ``fake_octopus.py``):
:meth:`MockRosbridgeServer.drop_all_clients`, :meth:`close_listener` /
:meth:`open_listener`, :meth:`inject_publish` and :meth:`add_local_subscriber`.
Together they cover "force a disconnect", "hold the port shut so the backoff ladder
is observable", and "act as a ROS publisher/subscriber on the far side".

Standalone:

    python3 src/gripperx_external/test/mock_rosbridge_server.py --port 9090
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set

try:
    from websockets.server import serve as _ws_serve
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "python3-websockets (apt, noble/universe 10.4) is required. A version >= 14 "
        "removed websockets.server.serve, which this stand-in is pinned to."
    ) from exc

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9090
DEFAULT_MAX_SIZE = 65536

LocalCallback = Callable[[str, Any], None]

_LOG = logging.getLogger("gripperx_external.mock_rosbridge")


@dataclass
class ServerStats:
    clients_accepted: int = 0
    clients_closed: int = 0
    frames_received: int = 0
    frames_sent: int = 0
    subscribes: int = 0
    unsubscribes: int = 0
    advertises: int = 0
    unadvertises: int = 0
    publishes_in: int = 0
    publishes_delivered: int = 0
    status_errors_sent: int = 0
    forced_drops: int = 0
    bad_frames: int = 0
    #: How many times each topic was subscribed to, across all connections. This
    #: is the number that proves a client re-subscribed after a reconnect.
    subscribes_by_topic: Dict[str, int] = field(default_factory=dict)
    advertises_by_topic: Dict[str, int] = field(default_factory=dict)


class _Client:
    __slots__ = ("ws", "subscriptions", "advertised", "peer")

    def __init__(self, ws: Any) -> None:
        self.ws = ws
        self.subscriptions: Set[str] = set()
        #: topic -> message type, as declared by ``advertise``.
        self.advertised: Dict[str, str] = {}
        self.peer = getattr(ws, "remote_address", None)


class MockRosbridgeServer:
    """A rosbridge-shaped WebSocket server with an in-process control surface."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        *,
        max_size: int = DEFAULT_MAX_SIZE,
        ping_interval_sec: Optional[float] = 5.0,
        ping_timeout_sec: Optional[float] = 5.0,
        close_timeout_sec: float = 2.0,
        latch_replay: bool = False,
        verbose_status: bool = False,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.max_size = int(max_size)
        self.ping_interval_sec = ping_interval_sec
        self.ping_timeout_sec = ping_timeout_sec
        self.close_timeout_sec = float(close_timeout_sec)
        self.latch_replay = bool(latch_replay)
        self.verbose_status = bool(verbose_status)
        self._log = logger or _LOG

        self.stats = ServerStats()
        self._clients: Set[_Client] = set()
        self._server: Any = None
        #: topic -> last message, kept only to serve ``--latch-replay``.
        self._latched: Dict[str, Any] = {}
        self._local_subs: Dict[str, List[LocalCallback]] = {}
        # Retained so a fire-and-forget send is not garbage-collected mid-flight
        # ("Task was destroyed but it is pending" would show up as noise in a soak).
        self._pending: Set[asyncio.Future] = set()

    # -- lifecycle --------------------------------------------------------
    async def start(self) -> None:
        await self.open_listener()

    async def open_listener(self) -> None:
        """Bind and accept. Idempotent."""
        if self._server is not None:
            return
        self._server = await _ws_serve(
            self._handle_client,
            self.host,
            self.port,
            max_size=self.max_size,
            ping_interval=self.ping_interval_sec,
            ping_timeout=self.ping_timeout_sec,
            close_timeout=self.close_timeout_sec,
        )
        self._log.info("mock rosbridge listening on ws://%s:%d", self.host, self.port)

    async def close_listener(self) -> None:
        """Stop accepting new connections, leave existing ones alone.

        Combined with :meth:`drop_all_clients` this produces a real outage - the
        client's connect attempts are refused, so the backoff ladder becomes
        observable instead of being short-circuited by an instant success.

        DO NOT "SIMPLIFY" THIS TO ``WebSocketServer.close()`` + ``wait_closed()``.
        That deadlocks forever on this exact stack, and it is a library bug, not a
        misuse: ``websockets`` 10.4's ``WebSocketServer._close()`` awaits the
        underlying ``asyncio.Server.wait_closed()`` *before* it closes the open
        WebSocket connections, and Python 3.12's ``wait_closed()`` (rewritten in
        3.12.1, see its own docstring) does not return until every connection has
        been dropped. Each waits for the other. Ubuntu 24.04 ships exactly this
        pair - python3-websockets 10.4 on Python 3.12.3 - so the deadlock is the
        default behaviour here, not an edge case.

        Closing the ``asyncio.Server`` directly avoids the cycle entirely: it frees
        the listening sockets synchronously and does not touch live connections,
        which is precisely the semantics this method promises. Full teardown is
        :meth:`stop`, which drops the connections first and therefore never needs
        to wait for them here.
        """
        if self._server is None:
            return
        ws_server, self._server = self._server, None
        asyncio_server = getattr(ws_server, "server", None)
        if asyncio_server is not None:
            asyncio_server.close()
        # Let the loop run the socket teardown, so an immediate open_listener()
        # can re-bind the same port.
        await asyncio.sleep(0)
        self._log.info("mock rosbridge stopped listening")

    async def stop(self) -> None:
        # Connections first, listener second - the reverse order is what walks
        # into the 10.4/3.12 deadlock described in close_listener().
        await self.drop_all_clients(code=1001, reason="server shutting down")
        await self.close_listener()

    async def drop_all_clients(
        self, code: int = 1001, reason: str = "forced disconnect"
    ) -> int:
        """Close every open connection. Returns how many were closed."""
        clients = list(self._clients)

        async def _close(client: _Client) -> None:
            try:
                await client.ws.close(code=code, reason=reason)
            except Exception:  # already gone; nothing to do
                pass

        if clients:
            self.stats.forced_drops += len(clients)
            # Concurrently and bounded: a peer that stops answering the closing
            # handshake must cost us close_timeout once, not once per client.
            await asyncio.wait_for(
                asyncio.gather(*(_close(c) for c in clients), return_exceptions=True),
                timeout=self.close_timeout_sec * 2 + 1.0,
            )
            self._log.info("mock rosbridge dropped %d client(s): %s", len(clients), reason)
        return len(clients)

    @property
    def client_count(self) -> int:
        return len(self._clients)

    @property
    def listening(self) -> bool:
        return self._server is not None

    # -- far-side ROS graph simulation ------------------------------------
    def add_local_subscriber(self, topic: str, callback: LocalCallback) -> None:
        """Subscribe *on the server side*, i.e. play a ROS node behind the bridge.

        This is how ``fake_octopus.py`` hears ``/octopus/trash_goal_done`` without
        opening a second WebSocket back to itself.
        """
        self._local_subs.setdefault(topic, []).append(callback)

    def inject_publish(self, topic: str, msg: Any, *, latch: bool = False) -> int:
        """Publish as if a ROS node behind the bridge had. Returns the delivery count."""
        if latch or self.latch_replay:
            self._latched[topic] = msg
        return self._deliver(topic, msg)

    # -- protocol ---------------------------------------------------------
    async def _handle_client(self, ws: Any, path: str = "") -> None:
        client = _Client(ws)
        self._clients.add(client)
        self.stats.clients_accepted += 1
        self._log.info("mock rosbridge client connected (%s)", client.peer)
        try:
            async for raw in ws:
                self.stats.frames_received += 1
                await self._handle_frame(client, raw)
        except Exception as exc:
            self._log.debug("client session ended: %s", exc)
        finally:
            self._clients.discard(client)
            self.stats.clients_closed += 1
            self._log.info("mock rosbridge client disconnected (%s)", client.peer)

    async def _handle_frame(self, client: _Client, raw: Any) -> None:
        try:
            frame = json.loads(raw)
        except (ValueError, TypeError) as exc:
            self.stats.bad_frames += 1
            await self._send_status(client, "error", f"not JSON: {exc}", None)
            return
        if not isinstance(frame, dict):
            self.stats.bad_frames += 1
            await self._send_status(client, "error", "frame is not an object", None)
            return

        op = frame.get("op")
        frame_id = frame.get("id")
        topic = frame.get("topic")

        if op == "subscribe":
            if not isinstance(topic, str) or not topic:
                self.stats.bad_frames += 1
                await self._send_status(client, "error", "subscribe without topic", frame_id)
                return
            client.subscriptions.add(topic)
            self.stats.subscribes += 1
            self.stats.subscribes_by_topic[topic] = (
                self.stats.subscribes_by_topic.get(topic, 0) + 1
            )
            if self.verbose_status:
                await self._send_status(client, "info", f"subscribed {topic}", frame_id)
            # Only with --latch-replay. Off by default because real rosbridge does
            # not replay a TRANSIENT_LOCAL sample to a late client.
            if self.latch_replay and topic in self._latched:
                await self._send_publish(client, topic, self._latched[topic])

        elif op == "unsubscribe":
            client.subscriptions.discard(topic if isinstance(topic, str) else "")
            self.stats.unsubscribes += 1
            if self.verbose_status:
                await self._send_status(client, "info", f"unsubscribed {topic}", frame_id)

        elif op == "advertise":
            msg_type = frame.get("type")
            if not isinstance(topic, str) or not isinstance(msg_type, str):
                self.stats.bad_frames += 1
                await self._send_status(
                    client, "error", "advertise needs topic and type", frame_id
                )
                return
            client.advertised[topic] = msg_type
            self.stats.advertises += 1
            self.stats.advertises_by_topic[topic] = (
                self.stats.advertises_by_topic.get(topic, 0) + 1
            )
            if self.verbose_status:
                await self._send_status(
                    client, "info", f"advertised {topic} as {msg_type}", frame_id
                )

        elif op == "unadvertise":
            client.advertised.pop(topic if isinstance(topic, str) else "", None)
            self.stats.unadvertises += 1
            if self.verbose_status:
                await self._send_status(client, "info", f"unadvertised {topic}", frame_id)

        elif op == "publish":
            if not isinstance(topic, str) or topic not in client.advertised:
                await self._send_status(
                    client,
                    "error",
                    f"cannot publish on {topic!r}: not advertised by this client",
                    frame_id,
                )
                return
            msg = frame.get("msg")
            self.stats.publishes_in += 1
            self._latched[topic] = msg
            self._deliver(topic, msg)

        else:
            # Real rosbridge answers an unknown op with an error status. Keeping
            # that behaviour is what exercises the client's status handling.
            await self._send_status(client, "error", f"unsupported op {op!r}", frame_id)

    def _deliver(self, topic: str, msg: Any) -> int:
        """Fan a message out to ws subscribers and to server-side subscribers."""
        delivered = 0
        for client in list(self._clients):
            if topic not in client.subscriptions:
                continue
            # The publisher is not excluded: in ROS a node that publishes and
            # subscribes to the same topic receives its own message.
            task = asyncio.ensure_future(self._send_publish(client, topic, msg))
            self._pending.add(task)
            task.add_done_callback(self._pending.discard)
            delivered += 1
        for callback in list(self._local_subs.get(topic, ())):
            try:
                callback(topic, msg)
            except Exception as exc:
                self._log.exception("local subscriber for %s raised: %s", topic, exc)
        self.stats.publishes_delivered += delivered
        return delivered

    async def _send_publish(self, client: _Client, topic: str, msg: Any) -> None:
        await self._send(client, {"op": "publish", "topic": topic, "msg": msg})

    async def _send_status(
        self, client: _Client, level: str, text: str, frame_id: Optional[Any]
    ) -> None:
        if level == "error":
            self.stats.status_errors_sent += 1
        await self._send(client, {"op": "status", "level": level, "msg": text, "id": frame_id})

    async def _send(self, client: _Client, frame: Dict[str, Any]) -> None:
        try:
            await client.ws.send(json.dumps(frame))
            self.stats.frames_sent += 1
        except Exception as exc:
            self._log.debug("send to %s failed: %s", client.peer, exc)


# ---------------------------------------------------------------------------
# standalone
# ---------------------------------------------------------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pure-python rosbridge stand-in.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--max-size", type=int, default=DEFAULT_MAX_SIZE)
    parser.add_argument(
        "--latch-replay",
        action="store_true",
        help="replay the last message to late subscribers - NOT what real "
        "rosbridge does; for demonstrating the difference only",
    )
    parser.add_argument("--verbose-status", action="store_true")
    parser.add_argument("--stats-interval", type=float, default=10.0)
    parser.add_argument("--log-level", default="INFO")
    return parser


async def _main(args: argparse.Namespace) -> int:
    server = MockRosbridgeServer(
        args.host,
        args.port,
        max_size=args.max_size,
        latch_replay=args.latch_replay,
        verbose_status=args.verbose_status,
    )
    await server.start()
    try:
        while True:
            await asyncio.sleep(max(0.1, args.stats_interval))
            print(
                f"[mock] clients={server.client_count} "
                f"subs={server.stats.subscribes} advs={server.stats.advertises} "
                f"pub_in={server.stats.publishes_in} "
                f"pub_out={server.stats.publishes_delivered} "
                f"errors={server.stats.status_errors_sent}",
                flush=True,
            )
    except (KeyboardInterrupt, asyncio.CancelledError):
        return 0
    finally:
        await server.stop()


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )
    try:
        return asyncio.run(_main(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
