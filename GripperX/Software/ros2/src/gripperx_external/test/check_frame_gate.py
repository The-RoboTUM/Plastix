#!/usr/bin/env python3
"""The Octopus-frame dispatch gate and the operator's blacklist door.

Two behaviours added on 2026-08-24, both driven by things that actually
happened on 2026-08-21 against the real counterpart:

  * THE FRAME GATE. Their `/octopus/flight_camera_transform/status` says whether
    their own map frame is usable. If they say it is not, dispatching would send
    the robot to a pose derived from a frame its owner does not vouch for. The
    hard part is not the gate, it is that SILENCE MUST NOT GATE: a counterpart
    that never publishes the topic has to keep working exactly as before.

  * `clear_blacklist`. `_maybe_drop_blacklist` drops the blacklist only on
    evidence - an id we acknowledged, watched turn `collected: true`, and then
    saw come back `collected: false`. That evidence CANNOT EXIST when their
    targets are deleted rather than collected, and their ids restart at 1 anyway.
    Observed live: a fresh target arrived as id 1, we still held id 1 from the
    previous id space, and every reachable target behind it was stuck with no
    error on either side. The only cure was restarting the node, which on the
    real robot costs the arming state and a fresh SR-1 approval.

Needs a ROS context (it builds the real node) but starts nothing, spins nothing,
dispatches nothing and moves nothing.

    python3 test/check_frame_gate.py
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

import rclpy  # noqa: E402

from gripperx_external_msgs.msg import ExternalLinkStatus  # noqa: E402

FAILURES = []


def check(label, condition):
    print(f"  [{'ok ' if condition else 'FAIL'}] {label}", flush=True)
    if not condition:
        FAILURES.append(label)


def link_status(*, enabled, seen, ready, relocks=0):
    msg = ExternalLinkStatus()
    msg.connected = True
    msg.last_message_age_sec = 0.1
    msg.reconnect_count = 0
    msg.frame_status_enabled = enabled
    msg.frame_status_seen = seen
    msg.frame_ready = ready
    msg.frame_relocks = relocks
    return msg


def frame_block(node):
    return [b for b in node._dispatch_blocks() if "map frame is not ready" in b]


def main():
    rclpy.init(
        args=["--ros-args", "-p", "expected_domain_id:=220", "-p", "use_sim_time:=false"]
    )
    from gripperx_external.goal_gateway_node import GoalGatewayNode

    node = GoalGatewayNode()

    print("\n-- 1. the frame gate never fires on an absence -------------------")
    node._on_link_status(link_status(enabled=False, seen=False, ready=False))
    check(
        "not subscribed at all -> no frame block (an Octopus without the topic "
        "keeps working)",
        not frame_block(node),
    )
    node._on_link_status(link_status(enabled=True, seen=False, ready=False))
    check(
        "subscribed but nothing received yet -> no frame block (silence is not "
        "evidence that their frame is bad)",
        not frame_block(node),
    )

    print("\n-- 2. an OBSERVED not-ready does gate ----------------------------")
    node._on_link_status(link_status(enabled=True, seen=True, ready=False))
    blocks = frame_block(node)
    check("observed not-ready -> exactly one frame block", len(blocks) == 1)
    check(
        "the block names what to look at",
        blocks and "transform status" in blocks[0],
    )

    print("\n-- 3. it recovers by itself, with no operator act ----------------")
    node._on_link_status(link_status(enabled=True, seen=True, ready=True))
    check("their state returns to ready -> block gone", not frame_block(node))
    check(
        "and nothing else was disturbed: still disarmed, still dry_run",
        any("disarmed" in b for b in node._dispatch_blocks())
        and any("dry_run" in b for b in node._dispatch_blocks()),
    )

    print("\n-- 4. the gate is independent of the arming gate -----------------")
    node._on_link_status(link_status(enabled=True, seen=True, ready=False))
    both = node._dispatch_blocks()
    check(
        "a not-ready frame and a closed arming gate are reported as TWO blocks, "
        "not one",
        any("map frame is not ready" in b for b in both)
        and any("disarmed" in b for b in both),
    )
    node._on_link_status(link_status(enabled=True, seen=True, ready=True))

    print("\n-- 5. clear_blacklist on an empty blacklist ----------------------")
    from std_srvs.srv import Trigger

    resp = node._on_clear_blacklist(Trigger.Request(), Trigger.Response())
    check("succeeds rather than erroring", resp.success)
    check("and says so plainly", "already empty" in resp.message)

    print("\n-- 6. clear_blacklist on the state the id reset actually left ----")
    # Exactly the live case: we blacklisted id "1" and counted its attempts,
    # then their node restarted and handed us a NEW object carrying id "1".
    node._blacklist.append("1")
    node._attempts["1"] = 2
    node._attempts["4"] = 1
    resp = node._on_clear_blacklist(Trigger.Request(), Trigger.Response())
    check("succeeds", resp.success)
    check("the blacklist is empty afterwards", not node._blacklist)
    check(
        "the ATTEMPT COUNTS go too - a target at max attempts would otherwise "
        "be blacklisted again on its next failure with no attempt left",
        not node._attempts,
    )
    check("the message names what was dropped", "1" in resp.message)

    print("\n-- 7. it permits nothing -----------------------------------------")
    with node._arming_lock:
        armed = node._arming.is_armed(node._safety_now())
    check("clearing did NOT arm anything", not armed)
    check(
        "and dispatch is still blocked for every reason it was blocked before",
        any("disarmed" in b for b in node._dispatch_blocks()),
    )

    print("\n-- 8. a re-lock CANCELS what is running, and does not disarm ----")
    from gripperx_external_msgs.msg import ExternalTarget, ExternalTargetList
    from gripperx_external import arming as arm_mod
    from gripperx_external.geodesy import Datum, map_to_latlon

    cancels = []
    node._cancel_mission = lambda reason, now, error=True: cancels.append(reason) or True

    node._frame_relocks_seen = None
    node._on_link_status(link_status(enabled=True, seen=True, ready=True, relocks=3))
    check(
        "a gateway that starts after the link node inherits its count without "
        "cancelling anything",
        not cancels,
    )
    node._on_link_status(link_status(enabled=True, seen=True, ready=True, relocks=3))
    check("an unchanged count cancels nothing", not cancels)
    # The real assertion: arm the machine, THEN re-lock, and the window has to
    # survive it. Driven on the arming machine directly - the service refuses
    # while allow_arm is false, and what is under test here is the consequence
    # of a re-lock, not the authority gate.
    # `allow_arm` is false in this node's default configuration and that is not
    # what is under test, so it is permitted on the machine directly. Nothing
    # else about the gate is bypassed.
    node._arming.allow_arm = True
    with node._arming_lock:
        granted = node._arming.arm(120.0, "check_frame_gate", node._safety_now())
    check("armed for the test", granted.granted)
    with node._arming_lock:
        armed_before = node._arming.is_armed(node._safety_now())
    check("...and the machine agrees it is armed", armed_before)

    node._on_link_status(link_status(enabled=True, seen=True, ready=True, relocks=4))
    check("the re-lock count moving up CANCELS", cancels == ["OCTOPUS_FRAME_RELOCK"])
    with node._arming_lock:
        armed_after = node._arming.is_armed(node._safety_now())
    check(
        "and the arming window SURVIVES it - a re-lock is geometry, not time, "
        "and re-arming would not repair it",
        armed_after,
    )
    with node._arming_lock:
        node._arming.disarm(arm_mod.TRIGGER_OPERATOR, node._safety_now(), "end of check")

    print("\n-- 9. an id that names a different object loses its blacklist ---")
    DAT = Datum(48.2650, 11.6710)
    node._datum_tracker.update(DAT)

    def target_at(target_id, x, y):
        lat, lon = map_to_latlon(DAT, x, y)
        t = ExternalTarget()
        t.id = target_id
        t.latitude_deg = lat
        t.longitude_deg = lon
        t.source_x, t.source_y = x, y
        t.collected = False
        t.is_goal = False
        return t

    def target_list(targets):
        m = ExternalTargetList()
        m.targets = targets
        return m

    # blacklisted at (0.40, 0.10)
    anchor_lat, anchor_lon = map_to_latlon(DAT, 0.40, 0.10)
    node._blacklist.append("1")
    node._blacklist_latlon["1"] = (anchor_lat, anchor_lon)
    node._attempts["1"] = 2

    node._drop_blacklist_on_identity(target_list([target_at("1", 0.44, 0.12)]))
    check(
        "the same id 0.045 m away is the same object - entry KEPT (inside 0.15)",
        "1" in node._blacklist,
    )

    node._drop_blacklist_on_identity(target_list([target_at("1", 0.10, -0.20)]))
    check(
        "the same id 0.42 m away is NOT the same object - entry dropped",
        "1" not in node._blacklist,
    )
    check("its anchor went with it", "1" not in node._blacklist_latlon)
    check(
        "and its attempt count too, or it would be re-blacklisted on the next "
        "failure with nothing left to spend",
        "1" not in node._attempts,
    )

    print("\n-- 10. the inference never runs without a datum ------------------")
    node._blacklist.append("7")
    node._blacklist_latlon["7"] = (anchor_lat, anchor_lon)
    saved_topic = node._datum_tracker._topic_datum
    saved_fallback = node._datum_tracker._fallback
    # No datum at all -> no metres -> nothing to compare, so nothing may be
    # inferred. `datum` falls back to the configured value, so both have to go.
    node._datum_tracker._topic_datum = None
    node._datum_tracker._fallback = None
    node._drop_blacklist_on_identity(target_list([target_at("7", 5.0, 5.0)]))
    check(
        "with no datum the entry is kept rather than guessed at",
        "7" in node._blacklist,
    )
    node._datum_tracker._topic_datum = saved_topic
    node._datum_tracker._fallback = saved_fallback
    node._blacklist.clear()
    node._blacklist_latlon.clear()

    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 74)
    if FAILURES:
        print(f"{len(FAILURES)} FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print("All frame-gate and clear_blacklist checks passed.")
    print(
        "NOT PROVEN HERE: that their `state` field behaves as their document "
        "says. This exercises OUR gate against crafted link statuses."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
