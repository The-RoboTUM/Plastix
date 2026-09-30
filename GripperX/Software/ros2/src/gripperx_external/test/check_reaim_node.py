#!/usr/bin/env python3
"""Behaviour of the re-aim on the REAL gateway node, with the world stubbed.

Needs a ROS context (it builds the real node) but spins nothing, sends nothing
to any server and moves nothing: both action clients are replaced by recorders,
TF, arming and validation are stubbed per case. It exists because the twin
harness cannot place an event inside a window that is microseconds wide, and a
structural check cannot tell whether a guard does anything (audit F-48: removing
the second `withdrawn` re-check in `_start_reaim` left check_reaim.py green -
case 1 here fails on that mutation).

    ROS_DOMAIN_ID=220 python3 test/check_reaim_node.py

Every number below is a FIXTURE (the user's 0.360 m grasp offset and an
arbitrary 0.10 m tolerance), none is a measurement.
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_PKG, "src"))

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

REAIM_BT = os.path.abspath(
    os.path.join(_PKG, "..", "gripperx_planning", "config", "reaim_in_place.xml")
)
#: 8.5 deg to the left of a robot at the origin facing +x: a re-aim is due, and
#: the unrotated grasp point is 0.072 m away (inside the 0.10 fixture tolerance).
OBJECT = (0.40, 0.06)
FAILURES = []


def check(label, condition, detail=""):
    print(f"  [{'PASS' if condition else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""),
          flush=True)
    if not condition:
        FAILURES.append(label)


class FakeFuture:
    def __init__(self, value=None):
        self.value = value
        self.callbacks = []

    def add_done_callback(self, fn):
        self.callbacks.append(fn)

    def result(self):
        return self.value

    def resolve(self, value):
        self.value = value
        for fn in self.callbacks:
            fn(self)


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancels = 0
        self.result_future = FakeFuture()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancels += 1
        return FakeFuture()


class Outcome:
    def __init__(self, status):
        self.status = status
        self.result = type("R", (), {"error_code": 0, "error_msg": ""})()


class Recorder:
    def __init__(self):
        self.goals = []
        self.futures = []

    def send_goal_async(self, goal):
        self.goals.append(goal)
        future = FakeFuture()
        self.futures.append(future)
        return future

    def server_is_ready(self):
        return True


def build():
    from gripperx_external import goal_gateway_node as gw

    overrides = [
        Parameter("expected_domain_id", value=220),
        Parameter("goal_ingress_enabled", value=True),
        Parameter("allow_arm", value=True),
        Parameter("dry_run", value=False),
        Parameter("auto_pick", value=True),
        Parameter("grasp.offset_x_m", value="0.360"),
        Parameter("grasp.offset_y_m", value="0.000"),
        Parameter("grasp.tolerance_m", value="0.10"),
        Parameter("grasp.reaim_enabled", value=True),
        Parameter("grasp.reaim_behavior_tree", value=REAIM_BT),
        Parameter("arming.max_consecutive_aborts", value=3),
    ]
    node = gw.GoalGatewayNode(parameter_overrides=overrides)
    return gw, node


#: A controllable ROS clock: settle ticks advance it by one dispatch period, so
#: the TF stamp derived from a constant `age` advances like a live transform.
CLOCK = [1000.0]


def reset(gw, node, pose=(0.0, 0.0, 0.0), age=0.05):
    node._ros_now = lambda: CLOCK[0]
    node._nav_client = Recorder()
    node._pick_client = Recorder()
    node._pick_available = True
    node._nav2_available = True
    node._auto_pick = True
    node._arming.is_armed = lambda _now: True
    node._arming.consecutive_aborts = 0
    node._robot_pose = lambda: (pose, age, "")
    # The stamped sibling follows whatever `_robot_pose` stub a case installs;
    # with the test clock fixed during a tick, CLOCK - age IS the stamp.
    node._robot_pose_stamped = lambda: (
        lambda p, a, e: (p, a, e, None if a is None else CLOCK[0] - a)
    )(*node._robot_pose())
    node._correlation_holds = lambda _m: (True, "fixture")
    node._recorrelate_mission = lambda _m: gw.corr.CorrelationResult(
        gw.corr.UNIQUE, target_id="7"
    )
    node._mission_dispatch_context = lambda _m, _p: None
    node._make_context = lambda *_a, **_k: None
    node._attempts.clear()
    node._blacklist.clear()
    gw.val.validate_dispatch = lambda _d, _c: gw.val.ValidationResult(
        verdict=gw.val.VERDICT_ACCEPTED
    )
    mission = gw.Mission(
        target_id="7",
        object_xy=OBJECT,
        pose=(0.04, 0.0, 0.0),
        datum_lat=float("nan"),
        datum_lon=float("nan"),
        started_at_sec=0.0,
        incoming=object(),
    )
    node._mission = mission
    return mission


def decision(gw, node):
    return gw.decide_reaim(OBJECT, (0.0, 0.0, 0.0), node._grasp_offset(), True,
                           math.radians(5.0), math.radians(20.0))


def picked(node):
    return len(node._pick_client.goals)


def settle(node, mission, ticks=2):
    """Dispatch ticks while the base stands still (the stubbed pose is constant)."""
    for _ in range(ticks):
        CLOCK[0] += 0.5
        if node._mission is mission:
            node._supervise_mission(mission, node._ros_now())


def reaim_sent(node):
    return [g for g in node._nav_client.goals if g.behavior_tree]


def main():
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=false"])
    gw, node = build()
    original_validate = gw.val.validate_dispatch
    try:
        run(gw, node)
    finally:
        gw.val.validate_dispatch = original_validate
        node.destroy_node()
        rclpy.shutdown()
    print("=" * 78)
    if FAILURES:
        print(f"FAILED: {len(FAILURES)} check(s)")
        for label in FAILURES:
            print(f"  - {label}")
        return 1
    print("All re-aim node checks passed.")
    return 0


def run(gw, node):
    print("1. SR-15 rule 13: a disarm landing between validate_dispatch and the send")
    mission = reset(gw, node)

    def validate_then_disarm(_d, _c):
        node._cancel_mission("disarm:OPERATOR", node._ros_now(), error=True)
        return gw.val.ValidationResult(verdict=gw.val.VERDICT_ACCEPTED)

    gw.val.validate_dispatch = validate_then_disarm
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    check("no re-aim goal is sent after the gate closed", not reaim_sent(node))
    check("  ... and the mission slot is released", node._mission is None)

    print("2. a cancel between arrival and the re-aim")
    mission = reset(gw, node)
    mission.cancel_requested_at_sec = 1.0
    mission.cancel_reason = "disarm:MODE_CHANGE"
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    check("no re-aim goal, slot released", not reaim_sent(node) and node._mission is None)

    print("3. a disarm during the send window (goal accepted after the gate closed)")
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    check("the re-aim goal is sent with the re-aim tree",
          len(reaim_sent(node)) == 1 and reaim_sent(node)[0].behavior_tree == REAIM_BT)
    node._cancel_mission("disarm:OPERATOR", node._ros_now(), error=True)
    handle = FakeHandle()
    node._nav_client.futures[0].resolve(handle)
    check("the late-accepted goal is cancelled the moment it has a handle", handle.cancels == 1)
    handle.result_future.resolve(Outcome(GoalStatus.STATUS_CANCELED))
    check("  ... and its CANCELED result ends without a pick, slot released",
          picked(node) == 0 and node._mission is None)

    print("4. re-aim REJECTED -> arrival gate on the unmoved pose -> pick")
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    node._nav_client.futures[0].resolve(FakeHandle(accepted=False))
    check("F-49: even the unmoved fallback settles first (no pick at once)",
          picked(node) == 0 and mission.state == gw.NAV_SETTLING)
    settle(node, mission)
    check("the gate passes on the pose held, so it picks", picked(node) == 1,
          mission.reaim_outcome)

    print("5. re-aim ABORTED -> no pick, counted")
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    handle = FakeHandle()
    node._nav_client.futures[0].resolve(handle)
    handle.result_future.resolve(Outcome(GoalStatus.STATUS_ABORTED))
    check("no pick after an abort", picked(node) == 0)
    check("  ... a failed attempt, and one abort on the budget",
          node._attempts.get("7") == 1 and node._arming.consecutive_aborts == 1)

    print("6. own timeout cancel -> confirmed -> arrival gate -> pick; superseded -> no pick")
    for supersede in (False, True):
        mission = reset(gw, node)
        node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
        handle = FakeHandle()
        node._nav_client.futures[0].resolve(handle)
        mission.reaim_started_at_sec = node._ros_now() - 100.0
        node._supervise_mission(mission, node._ros_now())
        check(f"[supersede={supersede}] the timeout cancels the re-aim", handle.cancels == 1)
        if supersede:
            node._cancel_mission("disarm:OPERATOR", node._ros_now(), error=True)
        handle.result_future.resolve(Outcome(GoalStatus.STATUS_CANCELED))
        settle(node, mission)
        if supersede:
            check("  ... a disarm landing on it supersedes: no pick", picked(node) == 0)
        else:
            check("  ... the confirmed cancel ends in the arrival gate, which picks",
                  picked(node) == 1 and mission.reaim_outcome == "timed out")
        check("  ... and the timeout counted against the abort budget (F-47)",
              node._arming.consecutive_aborts == 1)

    print("7. F-47: the approach's success does not reset the budget before a re-aim")
    mission = reset(gw, node)
    node._arming.consecutive_aborts = 2
    node._on_arrival(mission)
    check("first pass starts a re-aim instead of picking",
          len(reaim_sent(node)) == 1 and picked(node) == 0)
    check("  ... and leaves the consecutive-abort count at 2", node._arming.consecutive_aborts == 2)

    print("8. F-15/F-44: no fresh pose -> no re-aim, no pick")
    for pose, age in (((0.0, 0.0, 0.0), 5.0), (None, None)):
        mission = reset(gw, node, pose=pose, age=age)
        node._on_arrival(mission)
        check(f"pose={pose} age={age}: fails closed",
              picked(node) == 0 and not reaim_sent(node) and node._attempts.get("7") == 1)
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    handle = FakeHandle()
    node._nav_client.futures[0].resolve(handle)
    node._robot_pose = lambda: ((0.0, 0.0, 0.0), 5.0, "")
    handle.result_future.resolve(Outcome(GoalStatus.STATUS_SUCCEEDED))
    settle(node, mission, ticks=3)
    mission.settle_started_at_sec = node._ros_now() - 100.0
    settle(node, mission, ticks=1)
    check("a stale pose after the re-aim never settles and never picks (second pass)",
          picked(node) == 0 and node._attempts.get("7") == 1)

    print("9. correlation in the re-aim phase (user decision 2026-09-29): absence only")
    corr = gw.corr
    cases = (
        (corr.CorrelationResult(corr.NO_MATCH, absence=True), True),
        (corr.CorrelationResult(corr.NO_MATCH), False),
        (corr.CorrelationResult(corr.NO_TARGETS, absence=True), True),
        (corr.CorrelationResult(corr.NO_TARGETS), False),
        (corr.CorrelationResult(corr.TARGETS_STALE), False),
        (corr.CorrelationResult(corr.UNIQUE, target_id="7"), True),
        (corr.CorrelationResult(corr.UNIQUE, target_id="8"), False),
        (corr.CorrelationResult(corr.AMBIGUOUS), False),
        (corr.CorrelationResult(corr.ID_MISMATCH), False),
    )
    for result, expected in cases:
        mission = reset(gw, node)
        node._recorrelate_mission = lambda _m, r=result: r
        holds, _ = node._reaim_correlation_holds(mission)
        check(f"{result.status} {result.target_id or ''} absence={result.absence}: "
              f"holds={expected}", holds == expected)
    mission = reset(gw, node)

    def boom(_m):
        raise RuntimeError("fixture")

    node._recorrelate_mission = boom
    check("a failed re-correlation refuses", node._reaim_correlation_holds(mission)[0] is False)

    for result, cancels in ((corr.CorrelationResult(corr.NO_MATCH, absence=True), False),
                            (corr.CorrelationResult(corr.AMBIGUOUS), True),
                            (corr.CorrelationResult(corr.TARGETS_STALE), True)):
        mission = reset(gw, node)
        node._recorrelate_mission = lambda _m, r=result: r
        node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
        handle = FakeHandle()
        node._nav_client.futures[0].resolve(handle)
        node._supervise_mission(mission, node._ros_now())
        check(f"re-aim in flight, {result.status}: cancelled={cancels}",
              (handle.cancels == 1) == cancels)
        if not cancels:
            handle.result_future.resolve(Outcome(GoalStatus.STATUS_SUCCEEDED))
            settle(node, mission)
            check("  ... and the pick after the re-aim is NOT blocked by the absence",
                  picked(node) == 1)
    for status in (corr.AMBIGUOUS, corr.TARGETS_STALE):
        mission = reset(gw, node)
        node._recorrelate_mission = lambda _m, st=status: corr.CorrelationResult(st)
        mission.reaim_done, mission.reaim_outcome = True, "succeeded"
        node._on_arrival(mission)
        check(f"second pass with a {status} list: no pick", picked(node) == 0)

    print("10. F-46: switching the re-aim OFF ends one in flight at the arrival gate")
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    handle = FakeHandle()
    node._nav_client.futures[0].resolve(handle)
    result = node._on_set_parameters([Parameter("grasp.reaim_enabled", value=False)])
    check("switch-OFF is accepted and cancels the re-aim",
          result.successful and handle.cancels == 1)
    handle.result_future.resolve(Outcome(GoalStatus.STATUS_CANCELED))
    settle(node, mission)
    check("  ... whose confirmed cancel ends in the arrival gate (pick)",
          picked(node) == 1 and mission.reaim_outcome == "switched off")

    print("12. settling: no judgement and no pick while the base still turns")
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    handle = FakeHandle()
    node._nav_client.futures[0].resolve(handle)
    handle.result_future.resolve(Outcome(GoalStatus.STATUS_SUCCEEDED))
    check("the re-aim's success does NOT pick at once", picked(node) == 0
          and mission.state == gw.NAV_SETTLING)
    yaws = iter([0.10, 0.20, 0.25] + [0.26] * 20)
    node._robot_pose = lambda: ((0.0, 0.0, next(yaws)), 0.05, "")
    settle(node, mission, ticks=3)
    check("  ... nor while the pose keeps turning between ticks", picked(node) == 0)
    settle(node, mission, ticks=2)
    check("  ... and judges and picks once it stands still", picked(node) == 1)
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    handle = FakeHandle()
    node._nav_client.futures[0].resolve(handle)
    handle.result_future.resolve(Outcome(GoalStatus.STATUS_SUCCEEDED))
    counter = iter(range(1000))
    node._robot_pose = lambda: ((0.0, 0.0, 0.05 * next(counter)), 0.05, "")
    settle(node, mission, ticks=2)
    mission.settle_started_at_sec = node._ros_now() - 100.0
    settle(node, mission, ticks=1)
    check("a base that never stands still: REAIM_NOT_SETTLED, no pick",
          picked(node) == 0 and node._attempts.get("7") == 1)
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    handle = FakeHandle()
    node._nav_client.futures[0].resolve(handle)
    handle.result_future.resolve(Outcome(GoalStatus.STATUS_SUCCEEDED))
    node._cancel_mission("disarm:OPERATOR", node._ros_now(), error=True)
    settle(node, mission, ticks=1)
    check("a disarm while settling releases the slot, no pick",
          picked(node) == 0 and node._mission is None)

    print("13. the APPROACH's success settles first; the re-aim decision uses the settled pose")
    mission = reset(gw, node)
    # Nav2 result at yaw 0 (object 8.5 deg left), then the base coasts to
    # +8.5 deg and stops - aimed by the coast alone, so NO re-aim is due.
    poses = iter([0.0, 0.0, 0.10, math.atan2(0.06, 0.40)] + [math.atan2(0.06, 0.40)] * 20)
    node._robot_pose = lambda: ((0.0, 0.0, next(poses)), 0.05, "")
    node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
    check("approach success does not judge at once",
          mission.state == gw.NAV_SETTLING and not reaim_sent(node) and picked(node) == 0)
    settle(node, mission, ticks=4)
    check("  ... once still, the gate judges the SETTLED pose: no re-aim needed, pick",
          not reaim_sent(node) and picked(node) == 1)
    check("  ... and the coast is measured for the log",
          "coast +8.5 deg" in mission.coast_summary, mission.coast_summary)
    mission = reset(gw, node)
    node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
    settle(node, mission, ticks=2)
    check("a still base after the approach: the re-aim is decided on it (sent)",
          len(reaim_sent(node)) == 1 and picked(node) == 0)
    mission = reset(gw, node)
    counter = iter(range(1000))
    node._robot_pose = lambda: ((0.0, 0.0, 0.05 * next(counter)), 0.05, "")
    node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
    settle(node, mission, ticks=2)
    mission.settle_started_at_sec = node._ros_now() - 100.0
    settle(node, mission, ticks=1)
    check("never still after the approach: NOT_SETTLED, attempt counted, no pick, no re-aim",
          picked(node) == 0 and not reaim_sent(node) and node._attempts.get("7") == 1)

    print("14. F-50: the gate's STATE at re-aim send, while settling, at the pick point")
    rejected = lambda reason: (lambda _d, _c: gw.val.ValidationResult(  # noqa: E731
        verdict=gw.val.VERDICT_REJECTED, reason=reason, detail="fixture"))
    mission = reset(gw, node)
    gw.val.validate_dispatch = rejected(gw.val.DATUM_CHANGED)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    settle(node, mission, ticks=3)
    check("DATUM_CHANGED at the re-aim send: withdrawn, never a fallback pick",
          not reaim_sent(node) and picked(node) == 0 and node._mission is None)
    mission = reset(gw, node)
    gw.val.validate_dispatch = rejected(gw.val.COST_TOO_HIGH)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    settle(node, mission)
    check("a POSE refusal (COST_TOO_HIGH) at the re-aim send falls back and picks",
          not reaim_sent(node) and picked(node) == 1)
    mission = reset(gw, node)
    node._start_reaim(mission, (0.0, 0.0, 0.0), decision(gw, node))
    handle = FakeHandle()
    node._nav_client.futures[0].resolve(handle)
    handle.result_future.resolve(Outcome(GoalStatus.STATUS_SUCCEEDED))
    gw.val.validate_dispatch = rejected(gw.val.DATUM_CHANGED)
    settle(node, mission, ticks=3)
    check("DATUM_CHANGED while settling: no pick, mission released",
          picked(node) == 0 and node._mission is None)
    for reason in (gw.val.LINK_LOST, gw.val.MODE_STALE):
        mission = reset(gw, node)
        gw.val.validate_dispatch = rejected(reason)
        node._on_arrival(mission)
        check(f"{reason} at the pick point: no pick, no re-aim, withdrawn",
              picked(node) == 0 and not reaim_sent(node) and node._mission is None)

    print("14b. F-55: the state closes at the PICK point (object on axis, no re-aim)")
    mission = reset(gw, node)
    mission.object_xy = (0.40, 0.0)
    gw.val.validate_dispatch = rejected(gw.val.LINK_LOST)
    node._on_arrival(mission)
    check("LINK_LOST at the pick point with no re-aim due: no pick, withdrawn",
          picked(node) == 0 and not reaim_sent(node) and node._mission is None)

    print("14c. F-55: the state closes ONLY during a settle tick, then reopens")
    mission = reset(gw, node)
    mission.object_xy = (0.40, 0.0)
    node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
    accepted_result = gw.val.ValidationResult(verdict=gw.val.VERDICT_ACCEPTED)

    def closed_while_settling(_d, _c):
        # Closed exactly while the base settles, open again by the time the
        # pick point is reached - so ONLY the settle-tick check can catch it.
        if node._mission is not None and node._mission.settling:
            return gw.val.ValidationResult(
                verdict=gw.val.VERDICT_REJECTED, reason=gw.val.DATUM_CHANGED, detail="x")
        return accepted_result

    gw.val.validate_dispatch = closed_while_settling
    settle(node, mission, ticks=3)
    check("a DATUM_CHANGED seen on one settle tick ends the arrival: no pick, released",
          picked(node) == 0 and node._mission is None)

    print("15. F-51: a FROZEN transform never reads as a settled base")
    mission = reset(gw, node)
    node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
    frozen_stamp = CLOCK[0]
    node._robot_pose = lambda: ((0.0, 0.0, 0.0), CLOCK[0] - frozen_stamp, "")
    # Two ticks: both samples are still inside max_tf_age_sec (0.5 s, 1.0 s),
    # so only the stamp check can tell them apart from a still, live base.
    settle(node, mission, ticks=2)
    check("identical poses from an unchanged stamp: not settled, no pick",
          picked(node) == 0 and mission.state == gw.NAV_SETTLING)
    mission = reset(gw, node)
    node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
    future_stamp = CLOCK[0] + 100.0
    node._robot_pose_stamped = lambda: ((0.0, 0.0, 0.0), 0.0, "", future_stamp)
    settle(node, mission, ticks=3)
    check("F-54: a FUTURE-stamped frozen transform (age clamped to 0): not settled",
          picked(node) == 0 and mission.state == gw.NAV_SETTLING)

    print("16. F-52: which NO_MATCH / NO_TARGETS results are 'absence'")
    tp = corr.TargetPosition
    genuine = corr.correlate((0.0, 0.0), [tp(id="1", x=5.0, y=5.0, collected=False)], 0.25)
    collected = corr.correlate((0.0, 0.0), [tp(id="1", x=0.1, y=0.0, collected=True)], 0.25)
    bad_tol = corr.correlate((0.0, 0.0), [tp(id="1", x=0.1, y=0.0, collected=False)], 0.0)
    empty = corr.correlate((0.0, 0.0), [], 0.25)
    no_datum = node._correlate(48.0, 11.0, None, True, cross_check_reported_id=False)
    for label, result, expected in (
        ("no target within tolerance", genuine, True),
        ("empty list", empty, True),
        ("the only match flagged collected", collected, False),
        ("unusable goal_match_tolerance_m", bad_tol, False),
        ("no datum in force", no_datum, False),
    ):
        mission = reset(gw, node)
        node._recorrelate_mission = lambda _m, r=result: r
        holds, _ = node._reaim_correlation_holds(mission)
        check(f"{label}: {result.status} absence={result.absence} -> holds={expected}",
              holds == expected and result.absence == expected)

    print("17. F-53: an unconfirmed PICK cancel is reported (SR-15 rule 9)")
    mission = reset(gw, node)
    node.get_parameter("grasp.reaim_enabled")  # unchanged; object on axis:
    mission.object_xy = (0.40, 0.0)
    mission.cancel_confirmed = True  # as left by the navigation's result
    before = node._cancel_failures
    node._on_arrival(mission)
    check("the pick is sent and cancel_confirmed is reset for the new goal",
          picked(node) == 1 and mission.cancel_confirmed is False)
    node._pick_client.futures[0].resolve(FakeHandle())
    node._cancel_mission("disarm:OPERATOR", node._ros_now(), error=True)
    node._check_cancel_timeout(node._ros_now() + 100.0)
    check("  ... and a pick cancel nobody confirms is escalated to the report",
          node._cancel_failures == before + 1)

    print("11. the mission slot is released when a cancel lands before the pick")
    mission = reset(gw, node)
    mission.reaim_done, mission.reaim_outcome = True, "succeeded"
    mission.cancel_requested_at_sec, mission.cancel_reason = 1.0, "disarm:OPERATOR"
    node._on_arrival(mission)
    check("no pick, slot released", picked(node) == 0 and node._mission is None)


if __name__ == "__main__":
    sys.exit(main())
