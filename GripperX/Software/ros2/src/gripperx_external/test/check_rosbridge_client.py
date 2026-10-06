#!/usr/bin/env python3
"""Stage-0 verification: rosbridge client + mock server + fake Octopus.

Pure python, nothing running - no ROS, no DDS, no Octopus machine. Mirrors the
style of ``check_geodesy.py`` / ``check_validation.py``.

    python3 src/gripperx_external/test/check_rosbridge_client.py
    python3 src/gripperx_external/test/check_rosbridge_client.py --soak-sec 600

Part 1  protocol round trip (subscribe / advertise / publish / status)
Part 2  the two size limits, and that only ours keeps the connection up
Part 3  reconnect: re-subscribe, re-advertise, backoff ladder with jitter
Part 4  the fake Octopus's five real behaviours
Part 5  ``--soak-sec``: a long run with forced disconnects, counters asserted

THE SOAK IS THE GATE for build-order step 5. It asserts, over its whole run:
zero unhandled asyncio exceptions, zero logged tracebacks, zero callback errors,
one re-subscribe and one re-advertise per connection, and a backoff ladder that
climbs 1 -> 2 -> 4 ... and clamps at 30 s with visible jitter.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import math
import os
import re
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))
sys.path.insert(0, _HERE)

from gripperx_external import octopus_protocol as proto  # noqa: E402
from gripperx_external.geodesy import Datum, latlon_to_map  # noqa: E402
from gripperx_external.rosbridge_client import RosbridgeClient  # noqa: E402
from fake_octopus import FakeOctopus  # noqa: E402
from mock_rosbridge_server import MockRosbridgeServer  # noqa: E402

_failures: List[str] = []
_BACKOFF_RE = re.compile(r"reconnecting in ([0-9.]+) s \(step ([0-9]+)\)")


def check(condition: bool, label: str) -> bool:
    print(f"  [{'ok ' if condition else 'FAIL'}] {label}")
    if not condition:
        _failures.append(label)
    return bool(condition)


class Watchdog(logging.Handler):
    """Counts what must be zero, and keeps the backoff log for inspection.

    A traceback logged anywhere in the package is a failure of the soak even if
    nothing crashed - the whole point of the hand-rolled client is that failures
    surface as counters, not as stack traces.
    """

    def __init__(self) -> None:
        super().__init__(level=logging.DEBUG)
        self.tracebacks: List[str] = []
        self.errors: List[str] = []
        self.backoffs: List[Tuple[float, int]] = []
        self.loop_exceptions: List[str] = []

    def emit(self, record: logging.LogRecord) -> None:
        try:
            text = record.getMessage()
        except Exception:  # pragma: no cover
            text = str(record.msg)
        if record.exc_info:
            self.tracebacks.append(f"{record.name}: {text}")
        if record.levelno >= logging.ERROR:
            self.errors.append(f"{record.name}: {text}")
        match = _BACKOFF_RE.search(text)
        if match:
            self.backoffs.append((float(match.group(1)), int(match.group(2))))

    def install_loop_hook(self, loop: asyncio.AbstractEventLoop) -> None:
        def handler(_loop: asyncio.AbstractEventLoop, context: Dict[str, Any]) -> None:
            self.loop_exceptions.append(str(context.get("message") or context))

        loop.set_exception_handler(handler)


async def _wait_for(predicate, timeout: float, interval: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        await asyncio.sleep(interval)
    return predicate()


def _free_port(base: int) -> int:
    import socket

    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


# ---------------------------------------------------------------------------
# Part 1-3: the client against the bare mock
# ---------------------------------------------------------------------------
async def part_protocol_and_limits(watchdog: Watchdog) -> None:
    port = _free_port(9090)
    server = MockRosbridgeServer("127.0.0.1", port)
    await server.start()

    received: List[Tuple[str, Any]] = []
    states: List[bool] = []

    client = RosbridgeClient(
        f"ws://127.0.0.1:{port}",
        backoff_initial_sec=0.2,
        backoff_max_sec=1.0,
        backoff_reset_after_sec=0.5,
        on_state_change=states.append,
    )
    client.subscribe("/echo", "std_msgs/String", lambda t, m: received.append((t, m)))
    client.advertise("/echo", "std_msgs/String")
    runner = asyncio.ensure_future(client.run())

    print("\n1. protocol round trip")
    ok = await _wait_for(lambda: client.connected, 5.0)
    check(ok, "the client connects and reports connected=True")
    check(states == [True], "on_state_change fired once with True")
    await _wait_for(lambda: server.stats.subscribes >= 1 and server.stats.advertises >= 1, 2.0)
    check(server.stats.subscribes == 1, "exactly one subscribe reached the server")
    check(server.stats.advertises == 1, "exactly one advertise reached the server")

    check(client.publish("/echo", {"data": "hello"}), "publish on an advertised topic is accepted")
    await _wait_for(lambda: len(received) >= 1, 3.0)
    check(
        received and received[0] == ("/echo", {"data": "hello"}),
        "the published frame loops back to our own subscriber unchanged",
    )
    check(
        client.last_receive_age_sec() >= 0.0 and client.last_receive_age_sec() < 3.0,
        "last_receive_age_sec reports a fresh age after a delivery",
    )
    check(
        client.link_healthy(max_silence_sec=5.0) and not client.link_healthy(0.0),
        "link_healthy needs both a connection and recent traffic",
    )

    check(
        not client.publish("/never-advertised", {"data": "x"}),
        "publish on an un-advertised topic is refused locally, not sent",
    )
    check(
        client.stats().dropped_unadvertised == 1,
        "the un-advertised publish is counted, not silently dropped",
    )

    # Deliberately reaching past the public API: the client offers no way to send
    # a bad op, which is exactly why the inbound `status` path needs provoking
    # from here to be covered at all.
    client._offer({"op": "definitely-not-a-rosbridge-op", "id": "probe"})
    await _wait_for(lambda: client.stats().status_error >= 1, 3.0)
    check(client.stats().status_error == 1, "an error-level rosbridge status is counted")
    check(
        "unsupported op" in client.stats().last_error,
        "the status text is kept in last_error for telemetry",
    )

    boom: List[int] = []

    def exploding(topic: str, msg: Any) -> None:
        boom.append(1)
        raise RuntimeError("callback blew up on purpose")

    client.subscribe("/boom", "std_msgs/String", exploding)
    client.advertise("/boom", "std_msgs/String")
    await _wait_for(lambda: server.stats.subscribes >= 2, 2.0)
    client.publish("/boom", {"data": "x"})
    await _wait_for(lambda: client.stats().callback_errors >= 1, 3.0)
    check(client.stats().callback_errors == 1, "an exception inside a callback is caught+counted")
    check(client.connected, "a raising callback does not take the connection down")
    watchdog.tracebacks.clear()  # the traceback above is the expected outcome

    print("\n2. the two size limits")
    stats_before = client.stats()
    big_out = {"data": "x" * 9000}
    check(not client.publish("/echo", big_out), "an outbound frame over max_msg_bytes is refused")
    check(
        client.stats().oversize_dropped_out == stats_before.oversize_dropped_out + 1,
        "the oversized outbound frame is counted",
    )

    # Between max_msg_bytes (8192) and ws_max_size (65536): our drop, not the
    # library's close. The connection must survive it.
    connections_before = client.stats().connections
    server.inject_publish("/echo", {"data": "y" * 20000})
    await _wait_for(lambda: client.stats().oversize_dropped_in >= 1, 3.0)
    check(client.stats().oversize_dropped_in == 1, "an inbound frame over max_msg_bytes is dropped")
    check(
        client.connected and client.stats().connections == connections_before,
        "dropping an oversized inbound frame does NOT cost a reconnect",
    )
    check(
        len(received) == 1,
        "the oversized frame never reaches the callback",
    )

    print("\n3. reconnect, re-subscribe, re-advertise, backoff")
    subs_before = dict(server.stats.subscribes_by_topic)
    advs_before = dict(server.stats.advertises_by_topic)
    connections_before = client.stats().connections
    await server.drop_all_clients(reason="check: forced disconnect")
    check(await _wait_for(lambda: not client.connected, 3.0), "the client notices the disconnect")

    check(
        not client.publish("/echo", {"data": "while down"}),
        "a publish while disconnected is dropped, not buffered for later",
    )
    check(client.stats().dropped_disconnected >= 1, "the disconnected publish is counted")

    check(
        await _wait_for(lambda: client.stats().connections > connections_before, 10.0),
        "the client reconnects on its own",
    )
    check(client.stats().reconnects >= 1, "the reconnect counter advanced")
    await _wait_for(
        lambda: server.stats.subscribes_by_topic.get("/echo", 0) > subs_before.get("/echo", 0),
        3.0,
    )
    check(
        server.stats.subscribes_by_topic.get("/echo", 0) == subs_before.get("/echo", 0) + 1
        and server.stats.subscribes_by_topic.get("/boom", 0) == subs_before.get("/boom", 0) + 1,
        "every subscription is re-subscribed exactly once after the reconnect",
    )
    check(
        server.stats.advertises_by_topic.get("/echo", 0) == advs_before.get("/echo", 0) + 1
        and server.stats.advertises_by_topic.get("/boom", 0) == advs_before.get("/boom", 0) + 1,
        "every advertisement is re-advertised exactly once after the reconnect",
    )

    received.clear()
    client.publish("/echo", {"data": "after reconnect"})
    check(
        await _wait_for(lambda: len(received) >= 1, 3.0),
        "publish/subscribe work again over the new connection",
    )
    check(states[-1] is True and states.count(False) == 1,
          "on_state_change reported exactly one down and then up again")

    # Backoff ladder: refuse connections for a while and watch the delays grow.
    # The sleep is load-bearing, not padding: the ladder only resets to its first
    # rung once a connection has survived backoff_reset_after_sec (0.5 s here), so
    # without it the ladder would still be climbing from the drop above and the
    # "starts at step 0" assertion below would be testing nothing.
    await asyncio.sleep(0.6)
    watchdog.backoffs.clear()
    await server.close_listener()
    await server.drop_all_clients(reason="check: outage")
    await asyncio.sleep(4.0)
    await server.open_listener()
    check(
        await _wait_for(lambda: client.connected, 15.0),
        "the client recovers once the listener comes back",
    )
    ladder = [step for _, step in watchdog.backoffs]
    delays = [delay for delay, _ in watchdog.backoffs]
    check(len(ladder) >= 3, f"the outage produced several retries (steps observed: {ladder})")
    check(
        ladder == sorted(ladder) and ladder[0] == 0,
        f"a long-lived connection resets the ladder, which then climbs from 0: {ladder}",
    )
    expected = [min(1.0, 0.2 * 2**s) for s in ladder]
    within = all(
        abs(d - e) <= e * 0.2 + 1e-6 for d, e in zip(delays, expected)
    )
    check(within, f"each delay is its rung +/-20% jitter: {[round(d, 3) for d in delays]}")
    check(
        any(abs(d - e) > 1e-9 for d, e in zip(delays, expected)),
        "the jitter is actually applied (delays are not the bare rungs)",
    )

    client.stop()
    await asyncio.wait_for(runner, timeout=5.0)
    await server.stop()
    check(not client.connected, "stop() leaves the client disconnected and run() returns")


# ---------------------------------------------------------------------------
# Part 4: the fake Octopus's real behaviours
# ---------------------------------------------------------------------------
class _OctopusConsumer:
    """Minimal stand-in for what ``octopus_link_node`` will do in step 6."""

    def __init__(self, client: RosbridgeClient) -> None:
        self.client = client
        self.datum: Optional[Datum] = None
        self.goals: List[Tuple[float, float]] = []
        self.reports: List[proto.TrashGpsReport] = []
        self.parse_errors = 0

        client.subscribe(proto.TOPIC_DATUM, "sensor_msgs/NavSatFix", self._on_datum)
        client.subscribe(proto.TOPIC_TRASH_GOAL, "sensor_msgs/NavSatFix", self._on_goal)
        client.subscribe(proto.TOPIC_TRASH_GPS, "std_msgs/String", self._on_gps)
        client.advertise(proto.TOPIC_TRASH_GOAL_DONE, "std_msgs/String")

    def _on_datum(self, topic: str, msg: Any) -> None:
        try:
            fix = proto.parse_navsatfix(msg)
        except proto.ProtocolError:
            self.parse_errors += 1
            return
        self.datum = Datum(fix.latitude_deg, fix.longitude_deg, from_topic=True)

    def _on_goal(self, topic: str, msg: Any) -> None:
        try:
            fix = proto.parse_navsatfix(msg)
        except proto.ProtocolError:
            self.parse_errors += 1
            return
        self.goals.append((fix.latitude_deg, fix.longitude_deg))

    def _on_gps(self, topic: str, msg: Any) -> None:
        try:
            self.reports.append(proto.parse_trash_gps(msg))
        except proto.ProtocolError:
            self.parse_errors += 1

    def ack(self, target_id: str) -> bool:
        return self.client.publish(
            proto.TOPIC_TRASH_GOAL_DONE, {"data": proto.build_goal_done(target_id)}
        )

    @property
    def last_report(self) -> Optional[proto.TrashGpsReport]:
        return self.reports[-1] if self.reports else None


async def part_fake_octopus(watchdog: Watchdog) -> None:
    port = _free_port(9091)
    server = MockRosbridgeServer("127.0.0.1", port)
    await server.start()
    fake = FakeOctopus(server, rate_hz=10.0)  # 10 Hz so the checks do not take minutes

    client = RosbridgeClient(f"ws://127.0.0.1:{port}", backoff_initial_sec=0.2)
    consumer = _OctopusConsumer(client)
    runner = asyncio.ensure_future(client.run())
    publisher = asyncio.ensure_future(fake.run())

    print("\n4. the fake Octopus contract")
    await _wait_for(lambda: consumer.last_report is not None and consumer.datum is not None, 5.0)
    report = consumer.last_report
    check(report is not None and consumer.datum is not None, "datum and trash_gps arrive")
    assert report is not None and consumer.datum is not None

    check(
        report.datum is not None and report.datum.from_topic,
        "trash_gps carries datum{lat,lon,from_topic} and says the datum is live",
    )
    check(report.open_count == 3 and len(report.targets) == 3, "open_count and targets[] agree")
    check(
        report.goal_target() is not None and report.goal_id is not None,
        "exactly one target is flagged is_goal and goal_id names it",
    )

    # Round trip: their lat/lon expanded back through our inverse must land on the
    # map metres the fake stored. This is the whole geodetic agreement in one line.
    worst = 0.0
    for target in report.targets:
        x, y = latlon_to_map(consumer.datum, target.latitude_deg, target.longitude_deg)
        assert target.x is not None and target.y is not None
        worst = max(worst, math.hypot(x - target.x, y - target.y))
    check(worst < 1e-6, f"lat/lon round-trips to the stored map metres (worst {worst:.2e} m)")

    goal_target = report.goal_target()
    assert goal_target is not None
    check(
        goal_target.id == min(report.targets, key=lambda t: math.hypot(t.x or 0, t.y or 0)).id,
        "goal_selection is nearest-to-the-datum, as on their side",
    )

    # confidence may be null
    fake.add_target(0.5, 0.5, confidence=None)
    await _wait_for(lambda: len(consumer.reports[-1].targets) == 4, 3.0)
    check(
        any(t.confidence is None for t in consumer.reports[-1].targets),
        "a null confidence survives the wire as None rather than 0.0",
    )
    check(
        all(isinstance(t.last_seen, float) for t in consumer.reports[-1].targets),
        "last_seen is an epoch float",
    )

    # datum move -> immediate full republish burst
    reports_before = len(consumer.reports)
    bursts_before = fake.datum_bursts
    moved = fake.move_datum_m(0.30, -0.20)
    check(moved and fake.datum_bursts == bursts_before + 1, "a datum drag counts as a change")
    await _wait_for(lambda: len(consumer.reports) > reports_before, 2.0)
    latest = consumer.reports[-1]
    check(len(latest.targets) == 4, "the burst republishes ALL targets, not just the goal")

    # A sub-threshold move must NOT count (their threshold is 1e-9 deg).
    bursts_before = fake.datum_bursts
    tiny = fake.move_datum(
        fake.datum.latitude_deg + 1e-12, fake.datum.longitude_deg + 1e-12
    )
    check(
        not tiny and fake.datum_bursts == bursts_before,
        "a move below the 1e-9 deg threshold is not a change",
    )

    # acknowledge -> the goal advances
    goal_id = consumer.reports[-1].goal_id
    assert goal_id is not None
    acks_before = fake.acks_received
    check(consumer.ack(goal_id), "trash_goal_done is publishable on the advertised topic")
    check(
        await _wait_for(lambda: fake.acks_received > acks_before, 3.0),
        "the fake receives and parses the bare-id acknowledgement",
    )
    check(
        await _wait_for(
            lambda: consumer.reports[-1].goal_id not in (None, goal_id), 3.0
        ),
        "the goal advances to a different target after the acknowledgement",
    )
    check(consumer.reports[-1].open_count == 3, "open_count drops as targets are collected")

    # The {"id": n} form must work too - their parser accepts both.
    goal_id = consumer.reports[-1].goal_id
    assert goal_id is not None
    acks_before = fake.acks_received
    client.publish(
        proto.TOPIC_TRASH_GOAL_DONE, {"data": proto.build_goal_done(goal_id, as_json_object=True)}
    )
    check(
        await _wait_for(lambda: fake.acks_received > acks_before, 3.0),
        'the {"id": n} acknowledgement form is accepted as well',
    )

    # goal publishing stops entirely when nothing is open
    for target in list(consumer.reports[-1].targets):
        if not target.collected:
            consumer.ack(target.id)
            await asyncio.sleep(0.15)
    check(
        await _wait_for(lambda: consumer.reports[-1].open_count == 0, 5.0),
        "open_count reaches 0 once every target is acknowledged",
    )
    goals_before = fake.goal_publishes
    suppressed_before = fake.goal_suppressed_ticks
    await asyncio.sleep(0.8)
    check(
        fake.goal_publishes == goals_before and fake.goal_suppressed_ticks > suppressed_before,
        "trash_goal goes SILENT with nothing open - no sentinel, no empty message",
    )
    check(
        consumer.reports[-1].mission_complete and consumer.reports[-1].goal_id is None,
        "open_count == 0 is the only thing distinguishing 'done' from 'stale'",
    )

    # unreachable target: well-formed, but nothing can drive to it
    unreachable = fake.add_target(0.0, 0.0)
    fake.make_unreachable(unreachable.id)
    await _wait_for(lambda: consumer.reports[-1].open_count == 1, 3.0)
    stalled = consumer.reports[-1]
    far = [t for t in stalled.targets if t.id == str(unreachable.id)][0]
    check(
        far.x is not None and math.hypot(far.x, far.y or 0.0) > 100.0,
        "an unreachable target is expressed as a far-away position, their only option",
    )
    goals_before = fake.goal_publishes
    await asyncio.sleep(0.5)
    check(
        fake.goal_publishes > goals_before and consumer.reports[-1].goal_id == str(unreachable.id),
        "the mission stalls on it forever - the goal never advances without an ack",
    )

    # ids restart at 1, collected flags lost
    reports_before = len(consumer.reports)
    fake.restart_ids()
    # Waiting for a *newer* report, not for id "1": target 1 already existed, so
    # the obvious predicate would be satisfied by the pre-restart report.
    await _wait_for(lambda: len(consumer.reports) > reports_before, 3.0)
    restarted = consumer.reports[-1]
    check(
        [t.id for t in restarted.targets] == [str(i + 1) for i in range(len(restarted.targets))],
        "a restart renumbers the ids from 1",
    )
    check(
        restarted.open_count == len(restarted.targets),
        "a restart loses every collected flag - collected trash comes back",
    )

    publisher.cancel()
    client.stop()
    await asyncio.wait_for(runner, timeout=5.0)
    await asyncio.gather(publisher, return_exceptions=True)
    await server.stop()


# ---------------------------------------------------------------------------
# Part 5: soak
# ---------------------------------------------------------------------------
async def part_soak(watchdog: Watchdog, duration_sec: float, outage_sec: float) -> None:
    port = _free_port(9092)
    server = MockRosbridgeServer("127.0.0.1", port)
    await server.start()
    fake = FakeOctopus(server, rate_hz=1.0)  # their real rate

    client = RosbridgeClient(f"ws://127.0.0.1:{port}")
    consumer = _OctopusConsumer(client)
    runner = asyncio.ensure_future(client.run())
    publisher = asyncio.ensure_future(fake.run())

    n_topics_sub = 3
    n_topics_adv = 1
    start = time.monotonic()
    # Three forced drops with the server still up (instant recovery) plus one real
    # outage, which is what makes the backoff ladder observable at all.
    events: List[Tuple[float, str]] = [
        (duration_sec * 0.10, "drop"),
        (duration_sec * 0.30, "drop"),
        (duration_sec * 0.50, "outage"),
        (duration_sec * 0.75, "drop"),
    ]
    next_event = 0
    acked: set = set()
    added_targets = 0
    forced_disconnects = 0

    print(
        f"\n5. soak: {duration_sec:.0f} s, {len(events)} forced disconnects "
        f"(one of them a {outage_sec:.0f} s outage)"
    )
    while True:
        elapsed = time.monotonic() - start
        if elapsed >= duration_sec:
            break
        if next_event < len(events) and elapsed >= events[next_event][0]:
            _, kind = events[next_event]
            next_event += 1
            # A scheduled event that lands while the client is still climbing the
            # backoff ladder would drop nothing and quietly weaken the gate, so
            # wait for a live connection first. Bounded, so a genuinely stuck
            # client fails the run instead of stalling it.
            if not await _wait_for(lambda: server.client_count > 0, 90.0, 0.25):
                print(f"    t={elapsed:6.1f}s  NO CLIENT to disconnect - skipped")
                continue
            if kind == "drop":
                n = await server.drop_all_clients(reason="soak forced disconnect")
                forced_disconnects += n > 0
                print(f"    t={elapsed:6.1f}s  forced disconnect ({n} client)")
            else:
                await server.close_listener()
                n = await server.drop_all_clients(reason="soak outage")
                forced_disconnects += n > 0
                print(f"    t={elapsed:6.1f}s  outage begins ({outage_sec:.0f} s)")
                await asyncio.sleep(outage_sec)
                await server.open_listener()
                print(f"    t={time.monotonic() - start:6.1f}s  outage over")

        # Keep the mission moving: acknowledge the current goal, and top the field
        # up so the run exercises both the busy and the idle (open_count == 0) path.
        report = consumer.last_report
        if report is not None and report.goal_id and report.goal_id not in acked:
            if consumer.ack(report.goal_id):
                acked.add(report.goal_id)
        if report is not None and report.open_count == 0 and added_targets < 12:
            fake.add_target(0.5 + 0.1 * added_targets, -0.4)
            added_targets += 1
            acked.clear()
        await asyncio.sleep(0.5)

    stats = client.stats()
    elapsed = time.monotonic() - start
    client.stop()
    await asyncio.wait_for(runner, timeout=10.0)
    final = client.stats()
    publisher.cancel()
    await asyncio.gather(publisher, return_exceptions=True)
    await server.stop()

    ladder = watchdog.backoffs
    print()
    print(f"    duration                 : {elapsed:.1f} s")
    print(f"    connections / reconnects : {stats.connections} / {stats.reconnects}")
    print(f"    connect attempts/failures: {stats.connect_attempts} / {stats.connect_failures}")
    print(f"    frames in / out          : {stats.messages_received} / {stats.messages_sent}")
    print(f"    publishes routed/unrouted: {stats.publishes_routed} / {stats.publishes_unrouted}")
    print(f"    subscribe/advertise sent : {stats.subscribe_frames_sent} / "
          f"{stats.advertise_frames_sent}")
    print(f"    server saw subs/advs     : {server.stats.subscribes} / {server.stats.advertises}")
    print(f"    oversize dropped in/out  : {stats.oversize_dropped_in} / "
          f"{stats.oversize_dropped_out}")
    print(f"    dropped disconnected/full: {stats.dropped_disconnected} / "
          f"{stats.dropped_queue_full}")
    print(f"    status err/warn/info     : {stats.status_error} / {stats.status_warning} / "
          f"{stats.status_info}")
    print(f"    parse / callback errors  : {stats.parse_errors} / {stats.callback_errors}")
    print(f"    unhandled ops            : {stats.unhandled_ops}")
    print(f"    acks / goal publishes    : {fake.acks_received} / {fake.goal_publishes}")
    print(f"    goal-silent ticks        : {fake.goal_suppressed_ticks}")
    print(f"    backoff waits (s, step)  : {[(round(d, 2), s) for d, s in ladder]}")
    print(f"    logged tracebacks        : {len(watchdog.tracebacks)}")
    print(f"    asyncio loop exceptions  : {len(watchdog.loop_exceptions)}")
    print()

    check(len(watchdog.loop_exceptions) == 0,
          f"zero unhandled asyncio exceptions ({watchdog.loop_exceptions[:2]})")
    check(len(watchdog.tracebacks) == 0,
          f"zero logged tracebacks ({watchdog.tracebacks[:2]})")
    check(stats.callback_errors == 0, "zero callback errors")
    check(stats.parse_errors == 0, "zero parse errors")
    check(stats.oversize_dropped_in == 0 and stats.oversize_dropped_out == 0,
          "no frame of the real contract came near max_msg_bytes")
    check(stats.dropped_queue_full == 0, "the outbound queue never filled")
    check(forced_disconnects >= 3, f"at least three disconnects were really forced "
                                   f"({forced_disconnects})")
    check(stats.reconnects >= forced_disconnects,
          f"every forced disconnect was recovered ({stats.reconnects} >= "
          f"{forced_disconnects})")
    check(stats.connections == stats.reconnects + 1, "connections == reconnects + 1")
    check(
        stats.subscribe_frames_sent == stats.connections * n_topics_sub,
        f"re-subscribe on every connection: {stats.subscribe_frames_sent} == "
        f"{stats.connections} x {n_topics_sub}",
    )
    check(
        stats.advertise_frames_sent == stats.connections * n_topics_adv,
        f"re-advertise on every connection: {stats.advertise_frames_sent} == "
        f"{stats.connections} x {n_topics_adv}",
    )
    check(
        server.stats.subscribes == stats.subscribe_frames_sent
        and server.stats.advertises == stats.advertise_frames_sent,
        "the server counted exactly the registrations the client says it sent",
    )
    check(stats.publishes_unrouted == 0,
          "every inbound publish was routed to a registered callback")
    check(final.connected is False, "stop() left the link closed and run() returned")
    check(fake.acks_received > 0 and fake.unknown_acks == 0,
          "acknowledgements arrived and all of them were known ids")
    check(fake.goal_suppressed_ticks > 0,
          "the idle path (open_count == 0, goal topic silent) was exercised")

    steps = [s for _, s in ladder]
    check(len(ladder) >= 4, f"a backoff wait was logged for every disconnect ({len(ladder)})")
    check(max(steps) >= 3, f"the outage drove the ladder up to step {max(steps)}")
    delays_ok = True
    for delay, step in ladder:
        rung = min(30.0, 1.0 * 2**step)
        if not (rung * 0.8 - 1e-6 <= delay <= rung * 1.2 + 1e-6):
            delays_ok = False
    check(delays_ok, "every wait sat within +/-20% of its rung (1 s -> 30 s ladder)")
    check(any(abs(d - min(30.0, 2.0**s)) > 1e-9 for d, s in ladder), "jitter was applied")
    check(all(d <= 36.0 for d, _ in ladder), "no wait exceeded the 30 s cap plus jitter")


# ---------------------------------------------------------------------------
async def _run(args: argparse.Namespace) -> int:
    watchdog = Watchdog()
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.addHandler(watchdog)
    watchdog.install_loop_hook(asyncio.get_running_loop())

    print("=" * 78)
    print("gripperx_external - stage 0: rosbridge client, mock server, fake Octopus")
    print("=" * 78)

    if not args.soak_only:
        await part_protocol_and_limits(watchdog)
        await part_fake_octopus(watchdog)
        watchdog.tracebacks.clear()
        watchdog.loop_exceptions.clear()
        watchdog.backoffs.clear()

    if args.soak_sec > 0:
        await part_soak(watchdog, args.soak_sec, args.outage_sec)

    print()
    print("=" * 78)
    if _failures:
        print(f"FAILED: {len(_failures)} check(s)")
        for label in _failures:
            print(f"  - {label}")
        return 1
    print("All rosbridge stage-0 checks passed.")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Stage-0 checks for the rosbridge link.")
    parser.add_argument(
        "--soak-sec",
        type=float,
        default=0.0,
        help="run the long soak for this many seconds (gate: 600)",
    )
    parser.add_argument("--outage-sec", type=float, default=40.0)
    parser.add_argument("--soak-only", action="store_true")
    parser.add_argument("--log-file", default="")
    args = parser.parse_args(argv)

    if args.log_file:
        logging.basicConfig(
            filename=args.log_file,
            level=logging.DEBUG,
            format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        )
    return asyncio.run(_run(args))


if __name__ == "__main__":
    sys.exit(main())
