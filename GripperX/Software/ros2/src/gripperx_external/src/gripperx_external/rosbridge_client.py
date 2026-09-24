"""asyncio rosbridge-protocol client. No rclpy, no ROS message types.

This is the transport seam. Everything above it speaks the Octopus contract
(``octopus_protocol``); everything below it is a WebSocket carrying JSON. Swapping
rosbridge for something else means replacing this file and nothing else.

LIBRARY PIN - READ BEFORE "MODERNISING" ANYTHING HERE
=====================================================
The dependency is **python3-websockets from apt (noble/universe), version 10.4**.
Deliberately apt and not pip: PEP-668 blocks a system pip install, and a venv would
shadow ``rclpy`` for the node that wraps this client.

10.4 has **neither** ``websockets.asyncio`` (added in 13) **nor** ``websockets.sync``
(added in 12). The only API that exists is the legacy one::

    from websockets.client import connect      # a coroutine/async-context-manager

Rewriting this module against ``websockets.asyncio.client.connect``,
``ClientConnection``, ``websockets.connect(...)`` as a plain function, or any
``.recv(timeout=...)`` signature will raise ``ModuleNotFoundError`` or ``TypeError``
on the target machine. The keyword arguments used below - ``ping_interval``,
``ping_timeout``, ``max_size``, ``close_timeout``, ``open_timeout`` - are all present
in 10.4 and are the reason those particular names were chosen.

WHY THE RECONNECT IS HAND-ROLLED AND LOUD
=========================================
``roslibpy`` was rejected for exactly this: it hides reconnection inside a twisted
reactor, and reconnect semantics are what **NFR-4** and the **SR-15** arming gate are
built on. ``LINK_LOST`` must be a decision this process makes on evidence it can
show - a connected flag, a reconnect count, and the age of the last received frame -
not a state a library keeps to itself. Every one of those is a field on
:class:`LinkStats` and is meant to be published in telemetry and ``/diagnostics``.

WHAT THIS CLIENT DOES NOT DO
============================
It has no notion of goals, arming or motion. It cannot publish on a motion-chain
topic because it has no idea such topics exist; the topics it touches are whatever
the caller registers. **SR-15 rule 6** (never publish ``/teleop/set_mode``) is
enforced by the gateway that owns the topic names, not here.

TWO SIZE LIMITS, ON PURPOSE
===========================
``ws_max_size`` (65536) is the library's own guard: exceeding it makes *websockets*
close the connection with 1009, which costs a reconnect. ``max_msg_bytes`` (8192) is
ours and sits below it, so an oversized frame is **dropped and counted while the
connection stays up** - an observable event rather than a mystery reconnect. Frames
between the two limits are our drop; only something above 65536 kills the socket.

THREADING
=========
:meth:`RosbridgeClient.run` owns an asyncio event loop, which the wrapping rclpy node
will run in its own thread. :meth:`subscribe`, :meth:`unsubscribe`, :meth:`advertise`,
:meth:`unadvertise`, :meth:`publish` and :meth:`stats` are therefore **thread-safe**.
Registered callbacks are invoked **on the asyncio thread** - hand work off to the ROS
executor rather than blocking in them.
"""

from __future__ import annotations

import asyncio
import json
import logging
import random
import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, Iterable, Mapping, Optional

try:
    import websockets  # noqa: F401  (imported for the version banner below)
    from websockets.client import connect as _ws_connect
    from websockets.exceptions import WebSocketException
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    raise ImportError(
        "gripperx_external.rosbridge_client needs python3-websockets "
        "(apt: `sudo apt install python3-websockets`, noble/universe 10.4). "
        "Do NOT pip-install it: PEP-668 blocks the system install and a venv "
        "shadows rclpy. If the import failed on `websockets.client`, a version "
        ">= 14 is installed, which removed the legacy API this module is pinned to."
    ) from exc

#: Frames larger than this are dropped by us, in both directions, and counted.
DEFAULT_MAX_MSG_BYTES = 8192
#: Handed to websockets. Above this the *library* closes the connection (1009).
DEFAULT_WS_MAX_SIZE = 65536

DEFAULT_PING_INTERVAL_SEC = 5.0
DEFAULT_PING_TIMEOUT_SEC = 5.0
DEFAULT_CLOSE_TIMEOUT_SEC = 2.0
DEFAULT_OPEN_TIMEOUT_SEC = 5.0

DEFAULT_BACKOFF_INITIAL_SEC = 1.0
DEFAULT_BACKOFF_MAX_SEC = 30.0
#: +/- 20 %. Without it, several clients restarted together retry in lockstep.
DEFAULT_BACKOFF_JITTER = 0.2
#: A connection that survived this long resets the backoff ladder to its first
#: rung. Shorter than this counts as flapping and keeps the ladder climbing, so a
#: server that accepts and immediately drops does not get hammered at 1 Hz.
DEFAULT_BACKOFF_RESET_AFTER_SEC = 10.0

#: Bound on frames waiting to be written. Reached only if the socket stalls while
#: the caller keeps publishing; dropping the newest and counting it is preferable
#: to unbounded growth in a node that is meant to run for hours.
DEFAULT_OUTBOUND_QUEUE_SIZE = 64

#: 2 ** this caps the ladder arithmetic long before the max_sec clamp matters.
_MAX_BACKOFF_STEP = 20

MessageCallback = Callable[[str, Any], None]
StateCallback = Callable[[bool], None]


# ---------------------------------------------------------------------------
# observable state
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class LinkStats:
    """Immutable snapshot of the link. Every field is meant to be publishable.

    ``last_receive_sec`` and ``last_connect_sec`` are on the client's injected
    clock (``time.monotonic`` by default), so they are only meaningful relative to
    ``now``; :meth:`RosbridgeClient.last_receive_age_sec` does that subtraction.
    """

    connected: bool = False
    #: Successful connections. ``connections - 1`` of them were reconnects.
    connections: int = 0
    reconnects: int = 0
    connect_attempts: int = 0
    connect_failures: int = 0
    messages_received: int = 0
    messages_sent: int = 0
    #: Inbound ``publish`` frames handed to a registered callback.
    publishes_routed: int = 0
    #: Inbound ``publish`` frames for a topic nobody subscribed to.
    publishes_unrouted: int = 0
    #: Inbound ``status`` frames, by rosbridge level.
    status_error: int = 0
    status_warning: int = 0
    status_info: int = 0
    #: Inbound frames with an ``op`` this client does not implement.
    unhandled_ops: int = 0
    parse_errors: int = 0
    #: Exceptions raised *inside* a caller's callback. Caught, never propagated.
    callback_errors: int = 0
    oversize_dropped_in: int = 0
    oversize_dropped_out: int = 0
    #: Outbound frames discarded because the link was down when they were offered.
    dropped_disconnected: int = 0
    #: Outbound frames discarded because the write queue was full.
    dropped_queue_full: int = 0
    #: ``publish`` calls for a topic that was never advertised. rosbridge cannot
    #: infer the type, so sending them would be a silent no-op on the far side.
    dropped_unadvertised: int = 0
    #: ``subscribe`` / ``advertise`` frames sent, first connection *and* replays.
    #: With a static registry these divide by the topic count to give the number
    #: of connections that re-registered - which is how a soak proves that
    #: re-subscribe after reconnect actually happened.
    subscribe_frames_sent: int = 0
    advertise_frames_sent: int = 0
    #: The jittered delay currently being waited out, 0.0 while connected.
    current_backoff_sec: float = 0.0
    backoff_step: int = 0
    last_receive_sec: Optional[float] = None
    last_connect_sec: Optional[float] = None
    last_disconnect_sec: Optional[float] = None
    last_error: str = ""
    subscribed_topics: tuple = ()
    advertised_topics: tuple = ()


@dataclass
class _Subscription:
    topic: str
    msg_type: str
    callback: MessageCallback
    throttle_rate_ms: int = 0
    queue_length: int = 1
    compression: str = "none"

    def frame(self) -> Dict[str, Any]:
        return {
            "op": "subscribe",
            # Stable per topic, so a replay after reconnect re-uses the same id
            # rather than accumulating subscriptions on the far side.
            "id": f"sub:{self.topic}",
            "topic": self.topic,
            "type": self.msg_type,
            "throttle_rate": int(self.throttle_rate_ms),
            "queue_length": int(self.queue_length),
            "compression": self.compression,
        }


@dataclass
class _Advertisement:
    topic: str
    msg_type: str
    latch: bool = False
    queue_size: int = 1

    def frame(self) -> Dict[str, Any]:
        return {
            "op": "advertise",
            "id": f"adv:{self.topic}",
            "topic": self.topic,
            "type": self.msg_type,
            "latch": bool(self.latch),
            "queue_size": int(self.queue_size),
        }


# ---------------------------------------------------------------------------
# client
# ---------------------------------------------------------------------------
class RosbridgeClient:
    """Reconnecting rosbridge client over a single WebSocket.

    Protocol subset, and only this subset: ``subscribe``, ``unsubscribe``,
    ``advertise``, ``unadvertise``, ``publish`` outbound; ``publish`` and
    ``status`` inbound. No ``rosapi``, no services, no ``png``/``fragment``
    compression, no parameter access - the counterpart's rosbridge is meant to run
    with ``services_glob:="[]"`` precisely so none of that is reachable.
    """

    def __init__(
        self,
        url: str,
        *,
        max_msg_bytes: int = DEFAULT_MAX_MSG_BYTES,
        ws_max_size: int = DEFAULT_WS_MAX_SIZE,
        ping_interval_sec: float = DEFAULT_PING_INTERVAL_SEC,
        ping_timeout_sec: float = DEFAULT_PING_TIMEOUT_SEC,
        close_timeout_sec: float = DEFAULT_CLOSE_TIMEOUT_SEC,
        open_timeout_sec: float = DEFAULT_OPEN_TIMEOUT_SEC,
        backoff_initial_sec: float = DEFAULT_BACKOFF_INITIAL_SEC,
        backoff_max_sec: float = DEFAULT_BACKOFF_MAX_SEC,
        backoff_jitter: float = DEFAULT_BACKOFF_JITTER,
        backoff_reset_after_sec: float = DEFAULT_BACKOFF_RESET_AFTER_SEC,
        outbound_queue_size: int = DEFAULT_OUTBOUND_QUEUE_SIZE,
        on_state_change: Optional[StateCallback] = None,
        logger: Optional[logging.Logger] = None,
        time_fn: Callable[[], float] = time.monotonic,
        rng: Optional[random.Random] = None,
    ) -> None:
        self.url = url
        self.max_msg_bytes = int(max_msg_bytes)
        self.ws_max_size = int(ws_max_size)
        if self.max_msg_bytes > self.ws_max_size:
            raise ValueError(
                "max_msg_bytes must stay below ws_max_size, otherwise an oversized "
                "frame kills the connection instead of being dropped and counted"
            )
        self.ping_interval_sec = float(ping_interval_sec)
        self.ping_timeout_sec = float(ping_timeout_sec)
        self.close_timeout_sec = float(close_timeout_sec)
        self.open_timeout_sec = float(open_timeout_sec)
        self.backoff_initial_sec = float(backoff_initial_sec)
        self.backoff_max_sec = float(backoff_max_sec)
        self.backoff_jitter = float(backoff_jitter)
        self.backoff_reset_after_sec = float(backoff_reset_after_sec)
        self.outbound_queue_size = int(outbound_queue_size)

        self._on_state_change = on_state_change
        self._log = logger or logging.getLogger("gripperx_external.rosbridge")
        self._now = time_fn
        self._rng = rng or random.Random()

        self._subscriptions: Dict[str, _Subscription] = {}
        self._advertisements: Dict[str, _Advertisement] = {}
        self._publish_seq = 0

        self._stats = LinkStats()
        # The counters are written from two threads: the asyncio loop (inbound
        # frames, connect/disconnect) and whatever thread calls publish() or
        # subscribe(). Without this lock a concurrent read-modify-write silently
        # loses increments, and these counters are exactly what the arming gate
        # and the /diagnostics output are supposed to be trusted on.
        self._stats_lock = threading.Lock()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._outbound: Optional[asyncio.Queue] = None
        self._stop_event: Optional[asyncio.Event] = None
        self._stopping = False
        self._backoff_step = 0

    # -- observable state ------------------------------------------------
    def stats(self) -> LinkStats:
        """Thread-safe snapshot. Dataclass replace, so the caller cannot mutate us."""
        with self._stats_lock:
            current = self._stats
        return replace(
            current,
            subscribed_topics=tuple(sorted(self._subscriptions)),
            advertised_topics=tuple(sorted(self._advertisements)),
        )

    @property
    def connected(self) -> bool:
        return self._stats.connected

    def last_receive_age_sec(self, now: Optional[float] = None) -> float:
        """Seconds since the last **application** frame, ``-1.0`` if never.

        WebSocket pongs deliberately do not count. A far side whose ROS graph has
        gone quiet still answers pings, and treating that as a live link is
        precisely the failure ``LINK_LOST`` exists to catch.
        """
        last = self._stats.last_receive_sec
        if last is None:
            return -1.0
        return max(0.0, (self._now() if now is None else now) - last)

    def link_healthy(self, max_silence_sec: float, now: Optional[float] = None) -> bool:
        """Connected **and** something arrived within ``max_silence_sec``.

        Both Octopus producers publish at 1 Hz unconditionally, so silence on a
        connected socket means their stack, not their traffic, has stopped.
        """
        if not self._stats.connected:
            return False
        age = self.last_receive_age_sec(now)
        return 0.0 <= age <= max_silence_sec

    def _bump(self, **deltas: Any) -> None:
        """Add to counters, or assign with a ``set_`` prefix. Atomic across threads."""
        with self._stats_lock:
            current = self._stats
            updates: Dict[str, Any] = {}
            for name, delta in deltas.items():
                if name.startswith("set_"):
                    updates[name[4:]] = delta
                else:
                    updates[name] = getattr(current, name) + delta
            self._stats = replace(current, **updates)

    # -- registration (thread-safe, valid before and during a connection) --
    def subscribe(
        self,
        topic: str,
        msg_type: str,
        callback: MessageCallback,
        *,
        throttle_rate_ms: int = 0,
        queue_length: int = 1,
        compression: str = "none",
    ) -> None:
        """Register a subscription and send it if the link is up.

        Registration is the durable thing: the registry is replayed on **every**
        connection, so a subscribe issued while disconnected is not lost and a
        reconnect does not silently leave us deaf.
        """
        sub = _Subscription(
            topic=topic,
            msg_type=msg_type,
            callback=callback,
            throttle_rate_ms=throttle_rate_ms,
            queue_length=queue_length,
            compression=compression,
        )
        self._subscriptions[topic] = sub
        if self._stats.connected:
            if self._offer(sub.frame()):
                self._bump(subscribe_frames_sent=1)

    def unsubscribe(self, topic: str) -> None:
        sub = self._subscriptions.pop(topic, None)
        if sub is None:
            return
        if self._stats.connected:
            self._offer({"op": "unsubscribe", "id": f"sub:{topic}", "topic": topic})

    def advertise(
        self, topic: str, msg_type: str, *, latch: bool = False, queue_size: int = 1
    ) -> None:
        adv = _Advertisement(topic=topic, msg_type=msg_type, latch=latch, queue_size=queue_size)
        self._advertisements[topic] = adv
        if self._stats.connected:
            if self._offer(adv.frame()):
                self._bump(advertise_frames_sent=1)

    def unadvertise(self, topic: str) -> None:
        adv = self._advertisements.pop(topic, None)
        if adv is None:
            return
        if self._stats.connected:
            self._offer({"op": "unadvertise", "id": f"adv:{topic}", "topic": topic})

    def publish(self, topic: str, msg: Any) -> bool:
        """Queue a ``publish``. Returns ``False`` if it was dropped, and why is a counter.

        Dropped rather than buffered while disconnected, deliberately: everything
        we publish is either a timely acknowledgement or a telemetry sample, and
        replaying either one minutes later is worse than not sending it.
        """
        if topic not in self._advertisements:
            # rosbridge cannot infer the type, so this would be a silent no-op on
            # the far side. Loud here instead.
            self._log.error("publish on un-advertised topic %s - dropped", topic)
            self._bump(dropped_unadvertised=1, set_last_error=f"UNADVERTISED:{topic}")
            return False
        self._publish_seq += 1
        frame = {
            "op": "publish",
            "id": f"pub:{topic}:{self._publish_seq}",
            "topic": topic,
            "msg": msg,
        }
        return self._offer(frame)

    # -- outbound plumbing ------------------------------------------------
    def _offer(self, frame: Mapping[str, Any]) -> bool:
        try:
            text = json.dumps(frame)
        except (TypeError, ValueError) as exc:
            self._log.error("frame for %s is not JSON-serialisable: %s", frame.get("topic"), exc)
            self._bump(parse_errors=1, set_last_error=f"ENCODE:{exc}")
            return False

        size = len(text.encode("utf-8"))
        if size > self.max_msg_bytes:
            self._log.error(
                "outbound frame for %s is %d bytes, over max_msg_bytes=%d - dropped",
                frame.get("topic"),
                size,
                self.max_msg_bytes,
            )
            self._bump(oversize_dropped_out=1, set_last_error="OVERSIZE_OUT")
            return False

        loop = self._loop
        if loop is None or not self._stats.connected:
            self._bump(dropped_disconnected=1)
            return False
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            self._enqueue(text)
        else:
            loop.call_soon_threadsafe(self._enqueue, text)
        return True

    def _enqueue(self, text: str) -> None:
        """Runs on the asyncio thread only. Never raises - a QueueFull escaping
        here would surface as a loop-level 'unhandled exception' instead of a
        counter, which is the opposite of what this module is for."""
        queue = self._outbound
        if queue is None:
            self._bump(dropped_disconnected=1)
            return
        try:
            queue.put_nowait(text)
        except asyncio.QueueFull:
            self._log.error("outbound queue full (%d) - frame dropped", self.outbound_queue_size)
            self._bump(dropped_queue_full=1, set_last_error="QUEUE_FULL")

    # -- supervisor -------------------------------------------------------
    def _backoff_delay(self) -> float:
        step = min(self._backoff_step, _MAX_BACKOFF_STEP)
        base = min(self.backoff_max_sec, self.backoff_initial_sec * (2.0**step))
        jitter = self._rng.uniform(-self.backoff_jitter, self.backoff_jitter)
        return max(0.0, base * (1.0 + jitter))

    async def run(self) -> None:
        """Connect, serve, reconnect, forever - until :meth:`stop`.

        This is the whole reconnect policy, in one readable loop, on purpose.
        """
        self._loop = asyncio.get_running_loop()
        self._outbound = asyncio.Queue(maxsize=self.outbound_queue_size)
        self._stop_event = asyncio.Event()
        self._stopping = False

        while not self._stopping:
            self._bump(connect_attempts=1)
            connected_at: Optional[float] = None
            try:
                async with _ws_connect(
                    self.url,
                    ping_interval=self.ping_interval_sec,
                    ping_timeout=self.ping_timeout_sec,
                    close_timeout=self.close_timeout_sec,
                    open_timeout=self.open_timeout_sec,
                    max_size=self.ws_max_size,
                ) as ws:
                    connected_at = self._now()
                    await self._on_connected(ws, connected_at)
                    await self._session(ws)
            except asyncio.CancelledError:
                raise
            except (OSError, WebSocketException, asyncio.TimeoutError) as exc:
                self._note_failure(exc, connected_at)
            except Exception as exc:  # pragma: no cover - defensive
                self._log.exception("unexpected error in rosbridge session: %s", exc)
                self._note_failure(exc, connected_at)
            finally:
                if connected_at is not None:
                    self._on_disconnected(connected_at)

            if self._stopping:
                break
            delay = self._backoff_delay()
            self._bump(set_current_backoff_sec=delay, set_backoff_step=self._backoff_step)
            self._log.warning(
                "rosbridge link down (%s) - reconnecting in %.2f s (step %d)",
                self._stats.last_error or "closed",
                delay,
                self._backoff_step,
            )
            self._backoff_step = min(self._backoff_step + 1, _MAX_BACKOFF_STEP)
            try:
                assert self._stop_event is not None
                await asyncio.wait_for(self._stop_event.wait(), timeout=delay)
            except asyncio.TimeoutError:
                pass
            self._bump(set_current_backoff_sec=0.0)

        self._loop = None

    def stop(self) -> None:
        """Ask :meth:`run` to return. Thread-safe."""
        self._stopping = True
        loop, event = self._loop, self._stop_event
        if loop is not None and event is not None:
            loop.call_soon_threadsafe(event.set)

    def _note_failure(self, exc: BaseException, connected_at: Optional[float]) -> None:
        text = f"{type(exc).__name__}: {exc}".strip().rstrip(":")
        self._bump(set_last_error=text)
        if connected_at is None:
            self._bump(connect_failures=1)
            self._log.debug("rosbridge connect to %s failed: %s", self.url, text)

    async def _on_connected(self, ws: Any, connected_at: float) -> None:
        first = self._stats.connections == 0
        self._bump(
            connections=1,
            reconnects=0 if first else 1,
            set_connected=True,
            set_last_connect_sec=connected_at,
            set_last_error="",
        )
        self._log.info(
            "rosbridge connected to %s (connection #%d)", self.url, self._stats.connections
        )
        await self._replay_registry(ws)
        self._fire_state(True)

    def _on_disconnected(self, connected_at: float) -> None:
        uptime = max(0.0, self._now() - connected_at)
        was_connected = self._stats.connected
        self._bump(set_connected=False, set_last_disconnect_sec=self._now())
        # Drain whatever never made it onto the wire, and count it, rather than
        # letting it arrive on a later connection out of context.
        queue = self._outbound
        if queue is not None:
            stale = 0
            while not queue.empty():
                queue.get_nowait()
                stale += 1
            if stale:
                self._bump(dropped_disconnected=stale)
        if uptime >= self.backoff_reset_after_sec:
            self._backoff_step = 0
        if was_connected:
            self._log.warning("rosbridge disconnected after %.1f s", uptime)
            self._fire_state(False)

    def _fire_state(self, connected: bool) -> None:
        if self._on_state_change is None:
            return
        try:
            self._on_state_change(connected)
        except Exception as exc:  # callbacks are the caller's, never fatal here
            self._log.exception("on_state_change callback raised: %s", exc)
            self._bump(callback_errors=1)

    async def _replay_registry(self, ws: Any) -> None:
        """Re-send every subscribe and advertise on a fresh connection.

        Sent straight down the socket rather than through the outbound queue, so
        they cannot end up behind a publish that was queued a moment earlier - the
        far side would reject that publish for an unknown type.
        """
        for adv in list(self._advertisements.values()):
            await ws.send(json.dumps(adv.frame()))
            self._bump(advertise_frames_sent=1, messages_sent=1)
        for sub in list(self._subscriptions.values()):
            await ws.send(json.dumps(sub.frame()))
            self._bump(subscribe_frames_sent=1, messages_sent=1)

    async def _session(self, ws: Any) -> None:
        assert self._stop_event is not None
        reader = asyncio.ensure_future(self._reader(ws))
        writer = asyncio.ensure_future(self._writer(ws))
        stopper = asyncio.ensure_future(self._stop_event.wait())
        tasks: Iterable[asyncio.Future] = (reader, writer, stopper)
        try:
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in (reader, writer, stopper):
                if not task.done():
                    task.cancel()
            await asyncio.gather(reader, writer, stopper, return_exceptions=True)
        for task in done:
            if task is stopper:
                continue
            exc = task.exception()
            if exc is not None:
                raise exc

    async def _writer(self, ws: Any) -> None:
        assert self._outbound is not None
        while True:
            text = await self._outbound.get()
            await ws.send(text)
            self._bump(messages_sent=1)

    async def _reader(self, ws: Any) -> None:
        async for raw in ws:
            self._handle_raw(raw)

    # -- inbound ----------------------------------------------------------
    def _handle_raw(self, raw: Any) -> None:
        payload = raw if isinstance(raw, (bytes, bytearray)) else str(raw).encode("utf-8")
        if len(payload) > self.max_msg_bytes:
            self._log.error(
                "inbound frame of %d bytes exceeds max_msg_bytes=%d - dropped "
                "(connection kept)",
                len(payload),
                self.max_msg_bytes,
            )
            self._bump(oversize_dropped_in=1, set_last_error="OVERSIZE_IN")
            return

        self._bump(messages_received=1, set_last_receive_sec=self._now())
        try:
            frame = json.loads(payload)
        except (ValueError, TypeError) as exc:
            self._log.error("inbound frame is not JSON: %s", exc)
            self._bump(parse_errors=1, set_last_error=f"DECODE:{exc}")
            return
        if not isinstance(frame, Mapping):
            self._bump(parse_errors=1, set_last_error="DECODE:not an object")
            return

        op = frame.get("op")
        if op == "publish":
            self._handle_publish(frame)
        elif op == "status":
            self._handle_status(frame)
        else:
            # png/fragment/service ops and rosapi replies all land here. We
            # neither request nor need them; counting keeps a chatty far side
            # visible without turning it into an error.
            self._bump(unhandled_ops=1)
            self._log.debug("ignoring unsupported rosbridge op %r", op)

    def _handle_publish(self, frame: Mapping[str, Any]) -> None:
        topic = frame.get("topic")
        sub = self._subscriptions.get(topic) if isinstance(topic, str) else None
        if sub is None:
            # Normal for one frame after an unsubscribe; a persistent count means
            # the far side is sending something we never asked for.
            self._bump(publishes_unrouted=1)
            return
        self._bump(publishes_routed=1)
        try:
            sub.callback(sub.topic, frame.get("msg"))
        except Exception as exc:
            self._log.exception("callback for %s raised: %s", sub.topic, exc)
            self._bump(callback_errors=1, set_last_error=f"CALLBACK:{type(exc).__name__}")

    def _handle_status(self, frame: Mapping[str, Any]) -> None:
        level = str(frame.get("level", "")).lower()
        text = frame.get("msg", "")
        if level == "error":
            self._bump(status_error=1, set_last_error=f"STATUS:{text}")
            self._log.error("rosbridge status error (%s): %s", frame.get("id"), text)
        elif level == "warning":
            self._bump(status_warning=1)
            self._log.warning("rosbridge status warning (%s): %s", frame.get("id"), text)
        else:
            self._bump(status_info=1)
            self._log.debug("rosbridge status %s (%s): %s", level, frame.get("id"), text)
