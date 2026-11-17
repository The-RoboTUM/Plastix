#!/usr/bin/env python3
"""Space must always abort P - the gateway's side (user decision 2026-10-12; audit r10 H1).

A Space reaches the gateway twice: as a new /arm/stop_epoch from the arm, and as the
keyboard mode from teleop_mux, up to a mux tick later. A pick sent in between would
carry the NEW epoch and the arm would accept it. So a CHANGE of the epoch is itself an
operator stop: disarm, cancel the mission and any pick in flight, no resume without a
new arming. The REAL gateway node with the REAL arming machine; the action clients are
recorders (check_reaim_node harness), nothing is sent to any server, nothing moves.

    ROS_DOMAIN_ID=220 python3 test/check_stop_epoch.py
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import rclpy  # noqa: E402
from std_msgs.msg import Int64  # noqa: E402

import check_reaim_node as h  # noqa: E402

FAILURES = []


def check(label, condition, detail=""):
    print(f"  [{'PASS' if condition else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""),
          flush=True)
    if not condition:
        FAILURES.append(label)


def armed_mission(gw, node):
    """A mission at the grasp point (no re-aim), the REAL arming machine, armed."""
    mission = h.reset(gw, node)
    del node._arming.is_armed                    # the harness stubs it to True
    mission.object_xy = (0.40, 0.0)
    res = node._arming.arm(60.0, "check_stop_epoch", node._safety_now())
    assert res.granted, res.message
    return mission


def epoch(node, value):
    node._on_arm_stop_epoch(Int64(data=value))


def run(gw, node):
    print("1. the first epoch after start only initialises (disarmed, no mission)")
    node._arm_stop_epoch = None
    h.reset(gw, node)
    del node._arming.is_armed
    node._mission = None
    epoch(node, 1000)
    check("stored, nothing disarmed", node._arm_stop_epoch == 1000
          and node._arming.last_disarm_detail.find("stop epoch") < 0)
    mission = armed_mission(gw, node)
    epoch(node, 1000)
    check("the same value again changes nothing", node._is_armed() and not mission.cancelling)

    print("2. an epoch CHANGE (a Space at the arm, or a restart) stops the gateway (r10 G1)")
    epoch(node, 1001)
    check("disarmed by MODE_CHANGE, an ERROR diagnostic, the detail names both causes",
          not node._is_armed() and node._arming.last_disarm_trigger == "MODE_CHANGE"
          and node._last_disarm_error is True
          and "changed 1000 -> 1001 (Space handled by the arm, or arm node restarted)"
          in node._arming.last_disarm_detail, node._arming.last_disarm_detail)
    check("the mission is cancelled", mission.cancelling and "MODE_CHANGE" in mission.cancel_reason,
          mission.cancel_reason)
    node._on_arrival(mission)
    check("... and an arrival after it sends no pick", h.picked(node) == 0)

    print("3. no automatic resume; arming again does resume")
    mission = armed_mission(gw, node)
    node._on_arrival(mission)
    check("re-armed: the pick is sent, with the new epoch",
          h.picked(node) == 1 and node._pick_client.goals[0].stop_epoch == 1001)

    print("4. a pick in flight is cancelled by the epoch change")
    handle = h.FakeHandle()
    node._pick_client.futures[0].resolve(handle)
    epoch(node, 1002)
    check("disarmed and the pick goal cancelled", not node._is_armed() and handle.cancels == 1,
          f"cancels {handle.cancels}")

    print("5. already disarmed: a mission still in flight is withdrawn as well")
    mission = armed_mission(gw, node)
    with node._arming_lock:                      # the machine only: no cancel issued
        node._arming.disarm("OPERATOR", node._safety_now(), "test: operator disarm")
    check("fixture: disarmed, mission not cancelling", not mission.cancelling)
    epoch(node, 1003)
    check("the mission is cancelled", mission.cancelling)

    print("6. the FIRST epoch while armed (or on a mission) stops like a change (r10b)")
    node._arm_stop_epoch = None
    mission = armed_mission(gw, node)
    epoch(node, 2000)
    check("disarmed, mission cancelled, value stored",
          not node._is_armed() and mission.cancelling and node._arm_stop_epoch == 2000
          and "first arm stop epoch 2000" in node._arming.last_disarm_detail,
          node._arming.last_disarm_detail)

    print("7. G3: the epoch changes between the arrival's cancel check and its send")
    import threading
    mission = armed_mission(gw, node)
    real_goal, fired = gw.PickPlastic.Goal, []

    def goal_ctor(*a, **k):
        if not fired:
            fired.append(1)
            t = threading.Thread(target=epoch, args=(node, 2001))
            t.start()
            t.join()
        return real_goal(*a, **k)
    gw.PickPlastic.Goal = goal_ctor
    try:
        node._on_arrival(mission)
    finally:
        gw.PickPlastic.Goal = real_goal
    sent = [g.stop_epoch for g in node._pick_client.goals]
    check("the pick that left carries the OLD epoch (the arm refuses it)",
          fired and sent == [2000] and not node._is_armed(), f"sent {sent}")

    print("8. the arrival is past its armed check while the epoch handler runs")
    mission = armed_mission(gw, node)
    paused, go = threading.Event(), threading.Event()
    real_refusal, real_disarm = node._state_refusal, node._arming.disarm

    def refusal(*a, **k):                    # after the armed check, before the lock
        paused.set()
        go.wait(5.0)
        return real_refusal(*a, **k)
    node._state_refusal = refusal
    arrival = threading.Thread(target=node._on_arrival, args=(mission,))
    arrival.start()
    assert paused.wait(5.0)

    def disarm(*a, **k):                     # inside _on_arm_stop_epoch, mid-way
        go.set()
        arrival.join(0.5)                    # blocked on the mission lock (right)
        return real_disarm(*a, **k)
    node._arming.disarm = disarm
    try:
        epoch(node, 2002)
    finally:
        del node._arming.disarm
        del node._state_refusal
    arrival.join(5.0)
    sent = [g.stop_epoch for g in node._pick_client.goals]
    check("no pick carries the new epoch", not arrival.is_alive() and 2002 not in sent
          and not node._is_armed(), f"sent {sent}")

    print("9. no epoch when a pick is due: one clear diagnostic, not one per attempt")
    errors = []
    real_logger = node.get_logger

    class Log:
        def __init__(self, inner):
            self.inner = inner

        def error(self, text, *a, **k):
            errors.append(text)
            return self.inner.error(text, *a, **k)

        def __getattr__(self, name):
            return getattr(self.inner, name)
    log = Log(real_logger())
    node.get_logger = lambda: log
    try:
        node._arm_stop_epoch = None
        node._arm_stop_epoch_warned = False
        for _ in range(2):
            mission = armed_mission(gw, node)
            node._on_arrival(mission)
        diag = [e for e in errors if "stop_epoch 0" in e]
        check("sent with 0 (the arm refuses), the diagnostic once",
              node._pick_client.goals[0].stop_epoch == 0 and len(diag) == 1, f"{len(diag)}")
    finally:
        node.get_logger = real_logger

    print("10. QoS: the arm publishes /arm/stop_epoch RELIABLE + TRANSIENT_LOCAL depth 1")
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    subs = [s for s in node.subscriptions if s.topic_name == "/arm/stop_epoch"]
    q = subs[0].qos_profile if len(subs) == 1 else None
    check("one subscriber, RELIABLE + TRANSIENT_LOCAL, depth 1 (a late joiner gets it)",
          q is not None and q.reliability == ReliabilityPolicy.RELIABLE
          and q.durability == DurabilityPolicy.TRANSIENT_LOCAL and q.depth == 1, str(q))


def main():
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=false"])
    gw, node = h.build()
    original_validate = gw.val.validate_dispatch
    try:
        run(gw, node)
    finally:
        gw.val.validate_dispatch = original_validate
        node.destroy_node()
        rclpy.shutdown()
    if FAILURES:
        print(f"FAILED: {len(FAILURES)} check(s)")
        for label in FAILURES:
            print(f"  - {label}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
