#!/usr/bin/env python3
"""The aimed pick on the REAL gateway node, with the world stubbed (FR-16 stage 1;
SR-16 extension E1-E13, DRAFT). Needs a ROS context (it builds the real node), spins
nothing, sends nothing to any server and moves nothing: both action clients are
recorders, TF, correlation and validation are stubbed per case.

    ROS_DOMAIN_ID=220 python3 test/check_aimed_pick_node.py

What it pins down:
  * grasp.aimed_pick off: the PickPlastic goal is exactly today's (no aim, no preview);
  * automatic: aimed at the DISPATCHED position, in the map frame;
  * every arm refusal counts one attempt and acknowledges nothing; OUT_OF_REACH,
    ENVELOPE_REJECTED and NO_PATH leave the target to the ordinary dispatch (re-approach
    through every gate, while attempts remain); STALE_TF and BASE_MOVING hold it off;
    exhausted attempts blacklist as today;
  * confirm_by_rearm: preview -> disarm (PICK_CONFIRM) -> ONE pending pick -> the re-arm
    sends exactly one aimed pick; age, moved robot, a cancel or a refused preview
    discard it; the bounds come from the configuration;
  * confirm_by_rearm after the re-arm: every failed gate of the confirmation pass and
    every non-success of the confirmed pick discards like a pending pick (counted, held
    off, disarmed PICK_CONFIRM, no re-approach); only a success stays armed;
  * the mode's startup refusals.
Every number is a FIXTURE (the user's 0.360 m grasp offset, a 0.10 m tolerance, the
pending bounds), none is a measurement.
"""

import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_PKG, "src"))

import rclpy  # noqa: E402
import rclpy.duration  # noqa: E402,F401
from action_msgs.msg import GoalStatus  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from gripperx_arm_msgs.action import PickPlastic  # noqa: E402

OBJECT = (0.37, 0.02)            # 0.024 m from the 0.360 grasp point of a robot at 0
FAILURES = []
CLOCK = [1000.0]


def check(label, condition, detail=""):
    print(f"  [{'PASS' if condition else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""),
          flush=True)
    if not condition:
        FAILURES.append(label)


class FakeFuture:
    def __init__(self, value=None):
        self.value, self.callbacks = value, []

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
        self.accepted, self.cancels, self.result_future = accepted, 0, FakeFuture()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancels += 1
        return FakeFuture()


class Recorder:
    def __init__(self):
        self.goals, self.futures = [], []

    def send_goal_async(self, goal, **_kw):
        self.goals.append(goal)
        f = FakeFuture()
        self.futures.append(f)
        return f

    def server_is_ready(self):
        return True


class Outcome:
    def __init__(self, status, success=False, reason="", message="", tib=(0.0, 0.0)):
        self.status = status
        self.result = PickPlastic.Result(success=success, reason=reason, message=message)
        self.result.target_in_base.point.x, self.result.target_in_base.point.y = tib


def build(mode="off", **extra):
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
        Parameter("grasp.reaim_enabled", value=False),
        Parameter("max_attempts_per_target", value=2),
        Parameter("grasp.aimed_pick", value=mode),
        Parameter("grasp.localisation_settle_sec", value="1.0"),
        Parameter("grasp.localisation_settle_timeout_sec", value="8.0"),
    ] + _shared_tf() + [Parameter(k, value=v) for k, v in extra.items()]
    return gw, gw.GoalGatewayNode(parameter_overrides=overrides)


def _shared_tf():
    """The shared TF bound from the ACTIVE file (absent before audit F4 (a))."""
    from shared_tf import SHARED_TF_FILE, shared_tf_overrides
    return shared_tf_overrides() if os.path.isfile(SHARED_TF_FILE) else []


BOUNDS = {"grasp.pending_pick_max_age_sec": "60.0",
          "grasp.pending_pick_max_target_shift_m": "0.03"}


def reset(gw, node, pose=(0.0, 0.0, 0.0), armed=True):
    node._ros_now = lambda: CLOCK[0]
    node._nav_client, node._pick_client = Recorder(), Recorder()
    node._pick_available = node._nav2_available = True
    node._auto_pick = True
    node._arm_stop_epoch = 41
    node.__dict__.pop("_robot_pose", None)
    node._robot_pose = lambda: (pose, 0.05, "")
    node._robot_pose_stamped = lambda: (
        lambda p, a, e: (p, a, e, None if a is None else CLOCK[0] - a))(*node._robot_pose())
    node._correlation_holds = lambda _m, **_k: (True, "fixture")
    node._recorrelate_mission = lambda _m: gw.corr.CorrelationResult(
        gw.corr.UNIQUE, target_id="7", distance_m=0.004)
    node._mission_dispatch_context = lambda _m, p: gw.val.DispatchContext(
        armed=node._is_armed(), dry_run=False, link_alive=True, teleop_mode="autonomous",
        teleop_mode_age_sec=0.1, nav2_available=True, datum_unchanged=True,
        line_calibration_unchanged=True, pose=p or (0.0, 0.0, 0.0), arm_ready=True)
    node._make_context = lambda *_a, **_k: None
    node._attempts.clear()
    node._blacklist.clear()
    node._acknowledged.clear()
    gw.val.validate_dispatch = lambda d, _c: (
        gw.val.ValidationResult(verdict=gw.val.VERDICT_ACCEPTED) if d is None or d.armed
        else gw.val.ValidationResult(verdict=gw.val.VERDICT_PREVIEW, reason=gw.val.NOT_ARMED))
    with node._arming_lock:
        node._arming.disarm(gw.TRIGGER_OPERATOR, node._safety_now(), "reset")
        if armed:
            node._arming.arm(60.0, "check", node._safety_now())
    mission = gw.Mission(target_id="7", object_xy=OBJECT, pose=(0.0, 0.0, 0.0),
                         datum_lat=float("nan"), datum_lon=float("nan"), started_at_sec=0.0,
                         incoming=object())
    node._mission = mission
    return mission


def goals(node):
    return node._pick_client.goals


def answer(node, outcome, index=-1):
    """The arm's answer to the last pick goal: accepted, then this outcome."""
    h = FakeHandle()
    node._pick_client.futures[index].resolve(h)
    h.result_future.resolve(outcome)


def settle(node, mission, ticks=6):
    for _ in range(ticks):
        CLOCK[0] += 0.5
        if node._mission is mission:
            node._supervise_mission(mission, node._ros_now())


def setarm(node, arm):
    from gripperx_external_msgs.srv import SetArming
    req = SetArming.Request()
    req.arm, req.duration_sec, req.requested_by = arm, 60.0, "check"
    node._clock_proven, node._clock_stalled = True, False
    return node._on_set_arming(req, SetArming.Response())


def run_off_and_automatic():
    print("1. grasp.aimed_pick off: today's goal, nothing aimed")
    gw, node = build("off")
    try:
        mission = reset(gw, node)
        node._on_arrival(mission)
        g = goals(node)
        check("one pick goal", len(g) == 1)
        ref = PickPlastic.Goal(execute=True, stop_epoch=41)
        check("  ... field for field the fixed sequence's goal (no aim, no preview, no target)",
              g and g[0] == ref, repr(g[0]) if g else "")
        answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason="OUT_OF_REACH", message="x"))
        check("  ... and its failure is handled as before (PICK_FAILED, counted)",
              node._attempts.get("7") == 1 and node._last_reason == gw.ACK_PICK_FAILED)
        mission = reset(gw, node)
        node._goal_done_pub = RaisingPub()
        node._on_arrival(mission)
        try:
            answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
            escaped = None
        except Exception as exc:  # noqa: BLE001
            escaped = exc
        check("off, trash_goal_done publish raises: nothing escapes, unacknowledged as "
              "the other structural refusals (held off, still armed, no NavigateToPose)",
              escaped is None and node._goal_done_pub.calls == 1
              and node._ack_suppressed_reason == gw.ACK_PUBLISH_FAILED
              and "7" in node._blacklist and node._is_armed() and node._mission is None
              and not node._acknowledged and not node._nav_client.goals
              and "7" not in node._attempts, repr(escaped))
    finally:
        node.destroy_node()

    print("1b. one standoff for every mode (user decision 2026-10-13): grasp.offset_x_m")
    for mode, extra in (("off", {}), ("automatic", {}), ("confirm_by_rearm", BOUNDS)):
        gw, node = build(mode, **extra)
        try:
            check(f"{mode}: grasp point x = grasp.offset_x_m",
                  node._grasp_offset().x == float(node.get_parameter("grasp.offset_x_m").value))
            check("  ... and there is no separate aimed offset any more",
                  not node.has_parameter("grasp.aimed_offset_x_m"))
        finally:
            node.destroy_node()

    print("2. automatic: aimed at the DISPATCHED position, map frame")
    gw, node = build("automatic")
    try:
        mission = reset(gw, node)
        node._on_arrival(mission)
        g = goals(node)
        ok = len(g) == 1 and g[0].aimed and not g[0].preview
        check("one aimed goal, not a preview", ok)
        check("  ... target = mission.object_xy in the map frame, z 0",
              ok and g[0].target.header.frame_id == node._map_frame
              and (g[0].target.point.x, g[0].target.point.y, g[0].target.point.z)
              == (OBJECT[0], OBJECT[1], 0.0))

        for reason in ("OUT_OF_REACH", "ENVELOPE_REJECTED", "NO_PATH"):
            mission = reset(gw, node)
            node._on_arrival(mission)
            answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason=reason, message="arm"))
            check(f"{reason}: counted, released, NOT blacklisted, nothing acknowledged "
                  "(the ordinary dispatch re-approaches)",
                  node._attempts.get("7") == 1 and node._mission is None
                  and "7" not in node._blacklist and not node._acknowledged
                  and node._last_reason == f"AIMED_{reason}")
            mission = reset(gw, node)
            node._attempts["7"] = 1
            node._on_arrival(mission)
            answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason=reason, message="arm"))
            check("  ... the last attempt exhausted blacklists as today",
                  node._attempts.get("7") == 2 and "7" in node._blacklist)
        for reason in ("STALE_TF", "BASE_MOVING", "STEERING_UNKNOWN", "REFUSED_AFTER_MOTION",
                       "BASE_MOVED_DURING_PICK", "TARGET_SHIFTED"):
            mission = reset(gw, node)
            node._on_arrival(mission)
            answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason=reason, message="arm"))
            check(f"{reason}: counted and held off (no re-approach), nothing acknowledged",
                  node._attempts.get("7") == 1 and "7" in node._blacklist
                  and not node._acknowledged)
        mission = reset(gw, node)
        node._on_arrival(mission)
        answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason="REFUSED", message="busy"))
        check("any other failure: PICK_FAILED as today",
              node._last_reason == gw.ACK_PICK_FAILED and "7" not in node._blacklist)
        import inspect
        src = inspect.getsource(gw.GoalGatewayNode._aimed_refusal)
        check("a re-approach is never sent from the refusal path (C-5: one dispatch path)",
              "_send_nav_goal" not in src and "send_goal_async" not in src)
    finally:
        node.destroy_node()


def run_confirm():
    print("3. confirm_by_rearm: preview, disarm, ONE pending pick, the re-arm sends it")
    gw, node = build("confirm_by_rearm", **BOUNDS)
    try:
        mission = reset(gw, node)
        node._on_arrival(mission)
        g = goals(node)
        check("arrival sends a PREVIEW only", len(g) == 1 and g[0].aimed and g[0].preview)
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "PREVIEW_OK", "ok", (0.37, 0.02)))
        check("  ... the gateway disarmed itself with PICK_CONFIRM",
              not node._is_armed() and node._arming.last_disarm_trigger == "PICK_CONFIRM")
        check("  ... one pending pick, nothing else sent",
              mission.state == gw.NAV_PENDING_CONFIRM and mission.pending is not None
              and len(goals(node)) == 1 and node._mission is mission)
        settle(node, mission, ticks=2)
        check("  ... waiting sends nothing", len(goals(node)) == 1 and node._mission is mission)
        with node._arming_lock:
            node._arming.arm(60.0, "operator", node._safety_now())
        node._on_rearm()
        check("the re-arm starts the confirmation pass (settle), no pick yet",
              mission.state == gw.NAV_SETTLING and len(goals(node)) == 1)
        settle(node, mission)
        g = goals(node)
        check("  ... then exactly ONE aimed pick, not a preview",
              len(g) == 2 and g[1].aimed and not g[1].preview)
        check("  ... the arm checks the shift on ITS pick-start transform (SR-16 E11 item "
              "4, audit F5): the preview's target_in_base and the configured bound ride "
              "with the confirmed pick; the preview carried none",
              len(g) == 2 and g[1].check_target_shift and not g[0].check_target_shift
              and (g[1].preview_target_in_base.x, g[1].preview_target_in_base.y)
              == (0.37, 0.02)
              and g[1].max_target_shift_m
              == float(node.get_parameter("grasp.pending_pick_max_target_shift_m").value),
              f"{g[1].check_target_shift if len(g) == 2 else None}")
        node._on_rearm()
        settle(node, mission)
        check("a second re-arm sends nothing more", len(goals(node)) == 2)
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
        check("  ... its success is acknowledged as any pick", node._acknowledged == ["7"])

        def pending_mission(node=node):
            m = reset(gw, node)
            node._on_arrival(m)
            answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "PREVIEW_OK", "", (0.37, 0.02)))
            return m

        for label, spoil in (
            ("age bound exceeded", lambda m: setattr(m.pending, "recorded_mono",
                                                     m.pending.recorded_mono - 120.0)),
            ("robot moved 5 cm", lambda m: setattr(node, "_robot_pose",
                                                   lambda: ((0.05, 0.0, 0.0), 0.05, ""))),
            ("robot turned 5 deg", lambda m: setattr(node, "_robot_pose",
                                                     lambda: ((0.0, 0.0, 0.087), 0.05, ""))),
            ("stale pose at the re-arm", lambda m: setattr(node, "_robot_pose",
                                                           lambda: ((0.0, 0.0, 0.0), 9.0, ""))),
        ):
            m = pending_mission()
            spoil(m)
            with node._arming_lock:
                node._arming.arm(60.0, "operator", node._safety_now())
            node._on_rearm()
            settle(node, m)
            check(f"{label}: discarded at the re-arm, counted, no pick",
                  len(goals(node)) == 1 and node._attempts.get("7") == 1
                  and node._mission is None and not node._acknowledged)
            check("  ... and held off (no re-approach) with the gateway DISARMED (AP1 M3)",
                  "7" in node._blacklist and not node._is_armed())

        m = pending_mission()
        node._cancel_mission("disarm:MODE_CHANGE (arm stop epoch)", node._ros_now())
        settle(node, m, ticks=1)
        check("a cancel (Space via the stop epoch, any trigger) while pending discards it",
              node._mission is None and node._attempts.get("7") == 1 and len(goals(node)) == 1)

        m = pending_mission()
        m.pending.recorded_mono -= 120.0
        settle(node, m, ticks=1)
        check("the age bound expiring while pending discards it without a re-arm",
              node._mission is None and node._attempts.get("7") == 1)

        m = pending_mission()
        node._mission_dispatch_context = lambda _m, p: gw.val.DispatchContext(
            armed=node._is_armed(), dry_run=False, link_alive=False, teleop_mode="autonomous",
            teleop_mode_age_sec=0.1, nav2_available=True, datum_unchanged=True,
            line_calibration_unchanged=True, pose=p, arm_ready=True)
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        settle(node, m, ticks=1)
        check("a lost link while pending discards it (state gate, the disarm aside)",
              node._mission is None and node._attempts.get("7") == 1)

        print("   AP1 M1/L4/L5")
        m = pending_mission()
        r = setarm(node, False)
        check("an operator disarm while ALREADY disarmed discards the pending pick (M1)",
              r.success and node._mission is None and "7" in node._blacklist)
        r = setarm(node, True)
        settle(node, m)
        check("  ... and the next arming sends nothing", len(goals(node)) == 1)

        m = pending_mission()
        with node._arming_lock:
            node._arming.disarm(gw.TRIGGER_OPERATOR, node._safety_now(), "x")
            node._arming.arm(60.0, "operator", node._safety_now())
        node._on_rearm()
        settle(node, m)
        check("a disarm REQUEST between record and re-arm voids it, even if the "
              "re-arm follows before any tick (M1)",
              len(goals(node)) == 1 and node._mission is None)

        m = pending_mission()
        with node._arming_lock:
            node._arming.arm(60.0, "operator", node._safety_now())
        node._on_rearm()
        node._robot_pose = lambda: ((0.0, 0.0, 0.1), 0.05, "")     # ~37 mm shift
        settle(node, m)
        check("a target shifted from the preview beyond the bound is not sent (L4)",
              len(goals(node)) == 1 and node._mission is None and "7" in node._blacklist,
              f"goals {len(goals(node))}")

        # L5: an arming granted in the window between preview result and the disarm
        m = c_reset = reset(gw, node)
        node._on_arrival(m)
        orig = node._arming.disarm

        def hooked(trigger, now, detail=""):
            if trigger == "PICK_CONFIRM":
                node._arming.arm(60.0, "early", node._safety_now())
                node._on_rearm()
            return orig(trigger, now, detail)
        node._arming.disarm = hooked
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "PREVIEW_OK", "", (0.37, 0.02)))
        node._arming.disarm = orig
        settle(node, m)
        check("an arming BEFORE the gateway's disarm approves nothing (L5)",
              len(goals(node)) == 1 and m.state == gw.NAV_PENDING_CONFIRM
              and not node._is_armed(), m.state)
        del c_reset

        # AP2 (1): an arming between the gateway's disarm and the record
        m = reset(gw, node)
        node._on_arrival(m)
        orig_pub = node._publish_arming_state

        def arm_in_window():
            node._publish_arming_state = orig_pub
            with node._arming_lock:
                node._arming.arm(60.0, "between", node._safety_now())
            orig_pub()
        node._publish_arming_state = arm_in_window
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "PREVIEW_OK", "", (0.37, 0.02)))
        node._publish_arming_state = orig_pub
        with node._arming_lock:
            node._arming.arm(60.0, "operator", node._safety_now())
        node._on_rearm()
        settle(node, m)
        check("an arming between the disarm and the record: nothing pending, discarded, "
              "the next arming sends nothing (AP2)",
              len(goals(node)) == 1 and node._mission is None and "7" in node._blacklist)

        reset(gw, node)
        node._mission = None
        with node._arming_lock:
            node._arming.arm(60.0, "operator", node._safety_now())
        node._on_rearm()
        check("a re-arm with nothing pending does nothing arm-related", not goals(node))

        m = reset(gw, node)
        node._on_arrival(m)
        answer(node, Outcome(GoalStatus.STATUS_ABORTED, False, "OUT_OF_REACH", "outside"))
        check("a refused preview: counted, no disarm, no pending",
              node._is_armed() and node._mission is None and node._attempts.get("7") == 1)
        import inspect
        src = inspect.getsource(gw.GoalGatewayNode._on_set_arming)
        check("SetArming's grant path is what calls the re-arm hook", "_on_rearm()" in src)
    finally:
        node.destroy_node()

    print("3b. auto_pick and the measured tolerance: not in confirm_by_rearm, kept in automatic")
    for mode, extra, expect in (("confirm_by_rearm", BOUNDS, 1), ("automatic", {}, 0)):
        gw, node = build(mode, **extra)
        try:
            m = reset(gw, node)
            node._auto_pick = False
            node._on_arrival(m)
            check(f"{mode}, auto_pick false: {expect} goal(s)", len(goals(node)) == expect)
            m = reset(gw, node)
            node._pick_needs_measured_tolerance = True
            node.set_parameters([Parameter("grasp.tolerance_m", value="TO-VERIFY")])
            node._on_arrival(m)
            check(f"{mode}, real domain, tolerance unmeasured: {expect} goal(s)",
                  len(goals(node)) == expect)
        finally:
            node.destroy_node()

    print("4. the bounds come from the configuration (a duplicated constant would not move)")
    for age, expect_pick in (("60.0", True), ("0.5", False)):
        gw, node = build("confirm_by_rearm", **{**BOUNDS, "grasp.pending_pick_max_age_sec": age})
        try:
            m = reset(gw, node)
            node._on_arrival(m)
            answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "PREVIEW_OK", "", (0.37, 0.02)))
            time.sleep(1.0)
            with node._arming_lock:
                node._arming.arm(60.0, "operator", node._safety_now())
            node._on_rearm()
            settle(node, m)
            check(f"pending_pick_max_age_sec {age}, re-arm after 1 s: pick={expect_pick}",
                  (len(goals(node)) == 2) == expect_pick)
        finally:
            node.destroy_node()
    for shift, expect_pick in (("0.10", True), ("0.02", False)):
        gw, node = build("confirm_by_rearm",
                         **{**BOUNDS, "grasp.pending_pick_max_target_shift_m": shift})
        try:
            m = reset(gw, node)
            node._on_arrival(m)
            answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "PREVIEW_OK", "", (0.37, 0.02)))
            node._robot_pose = lambda: ((0.0, 0.0, 0.1), 0.05, "")
            with node._arming_lock:
                node._arming.arm(60.0, "operator", node._safety_now())
            node._on_rearm()
            settle(node, m)
            check(f"pending_pick_max_target_shift_m {shift}, target shifted ~37 mm: "
                  f"pick={expect_pick}", (len(goals(node)) == 2) == expect_pick)
        finally:
            node.destroy_node()


class PubRecorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class RaisingPub:
    """trash_goal_done's publisher failing (rcl error)."""

    def __init__(self):
        self.msgs, self.calls = [], 0

    def publish(self, _msg):
        self.calls += 1
        raise RuntimeError("fixture: rcl publish failed")


def run_confirm_failures():
    print("3c. confirm_by_rearm: EVERY non-success after the re-arm discards (F1, user "
          "2026-11-09 '1 a')")
    from types import SimpleNamespace
    gw, node = build("confirm_by_rearm", **BOUNDS)
    tolerance = float(node.get_parameter("grasp.tolerance_m").value)
    real_on_arrival = node._on_arrival
    routes = []
    real_confirm_pass_failed = node._confirm_pass_failed

    def spy(mission, reason, why):
        handled = real_confirm_pass_failed(mission, reason, why)
        if handled:
            routes.append((reason, why))
        return handled
    node._confirm_pass_failed = spy

    def routed(label, expect):
        got = routes[-1][0] if routes else None
        check(f"  ... through the {expect} branch", got == expect, f"{label}: {routes[-1:]}")

    def confirmed():
        """A mission whose pending pick the operator has just approved (settle running)."""
        m = reset(gw, node)
        node.__dict__.pop("_on_arrival", None)
        node._goal_done_pub = PubRecorder()
        node._arm_stop_epoch = 41
        node._on_arrival(m)
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "PREVIEW_OK", "", (0.37, 0.02)))
        with node._arming_lock:
            node._arming.arm(60.0, "operator", node._safety_now())
        node._on_rearm()
        ok = m.state == gw.NAV_SETTLING and m.confirm_pass and len(goals(node)) == 1
        check("  (fixture: re-arm accepted, confirmation pass settling)", ok, m.state)
        return m

    def discarded(label, picks, trigger="PICK_CONFIRM"):
        check(f"{label}: disarmed ({trigger}), held off, counted once, no NavigateToPose, "
              "no trash_goal_done",
              not node._is_armed() and node._arming.last_disarm_trigger == trigger
              and "7" in node._blacklist and node._attempts.get("7") == 1
              and node._mission is None and not node._nav_client.goals
              and not node._acknowledged and not node._goal_done_pub.msgs
              and node._last_reason == "PENDING_PICK_DISCARDED"
              and len(goals(node)) == picks,
              f"armed={node._is_armed()} trigger={node._arming.last_disarm_trigger} "
              f"bl={node._blacklist} att={node._attempts} reason={node._last_reason} "
              f"picks={len(goals(node))}")

    try:
        print("   gates of the confirmation pass")
        m = confirmed()
        # Off the grasp point by twice the CONFIGURED tolerance, unmoved during the settle.
        off = (-2.0 * tolerance, 0.0, 0.0)
        node._robot_pose = lambda: (off, 0.05, "")
        settle(node, m)
        discarded("reached check fails at the confirmation", picks=1)

        m = confirmed()
        node._correlation_holds = lambda _m, **_k: (False, "two objects within the radius")
        settle(node, m)
        discarded("correlation no longer holds at the confirmation", picks=1)

        m = confirmed()
        stale = float(node.get_parameter("max_tf_age_sec").value) + 1.0

        def stale_at_arrival(mission):
            node._robot_pose = lambda: ((0.0, 0.0, 0.0), stale, "")
            real_on_arrival(mission)
        node._on_arrival = stale_at_arrival
        settle(node, m)
        node.__dict__.pop("_on_arrival", None)
        discarded("pose stale at the arrival pass (TF older than max_tf_age_sec)", picks=1)

        m = confirmed()
        node._robot_pose = lambda: (None, None, "no transform")
        settle(node, m, ticks=int(2 * float(
            node.get_parameter("grasp.localisation_settle_timeout_sec").value) / 0.5) + 2)
        discarded("no TF during the confirmation settle (never settles)", picks=1)

        m = confirmed()
        node._mission_dispatch_context = lambda _m, p: gw.val.DispatchContext(
            armed=node._is_armed(), dry_run=False, link_alive=False, teleop_mode="autonomous",
            teleop_mode_age_sec=0.1, nav2_available=True, datum_unchanged=True,
            line_calibration_unchanged=True, pose=p, arm_ready=True)
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        settle(node, m, ticks=2)
        discarded("link lost during the confirmation settle (state gate)", picks=1)

        m = confirmed()
        settle(node, m, ticks=1)
        node._on_arm_stop_epoch(SimpleNamespace(data=42))
        settle(node, m, ticks=2)
        discarded("Space (stop epoch) during the confirmation settle", picks=1,
                  trigger="MODE_CHANGE")

        print("   the confirmed pick's own result")
        codes = sorted(gw.AIM_REAPPROACH | gw.AIM_NO_REAPPROACH
                       | {"REFUSED_AFTER_MOTION", "REFUSED", ""})
        for code in codes:
            m = confirmed()
            settle(node, m)
            g = goals(node)
            check(f"  (fixture: confirmed pick sent for {code or 'abort'})",
                  len(g) == 2 and g[1].aimed and not g[1].preview)
            answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason=code, message="arm"))
            discarded(f"arm result {code or 'ABORTED, no reason'}", picks=2)

        m = confirmed()
        settle(node, m)
        h = FakeHandle(accepted=False)
        node._pick_client.futures[-1].resolve(h)
        discarded("confirmed pick rejected by the arm server", picks=2)

        m = confirmed()
        settle(node, m)
        h = FakeHandle()
        node._pick_client.futures[-1].resolve(h)
        node._on_arm_stop_epoch(SimpleNamespace(data=42))
        check("Space during the confirmed pick: disarmed MODE_CHANGE and the pick "
              "cancelled at once, as before",
              h.cancels == 1 and not node._is_armed()
              and node._arming.last_disarm_trigger == "MODE_CHANGE", h.cancels)
        h.result_future.resolve(Outcome(GoalStatus.STATUS_CANCELED))
        discarded("  ... its CANCELED result", picks=2, trigger="MODE_CHANGE")
        check("  ... Space during a confirmed pick HOLDS THE TARGET OFF (USER DECISION "
              "2026-11-11)", "7" in node._blacklist and not node._nav_client.goals)

        print("   transport failures, gates of the arrival pass, cancels (re-audit item 5)")

        def fail(future):
            def boom():
                raise RuntimeError("fixture: transport failure")
            future.result = boom
            for fn in future.callbacks:
                fn(future)

        m = confirmed()
        settle(node, m)
        fail(node._pick_client.futures[-1])
        discarded("PICK_SEND_FAILED (the goal response raised)", picks=2)
        routed("send", "PICK_SEND_FAILED")

        m = confirmed()
        settle(node, m)
        h = FakeHandle()
        node._pick_client.futures[-1].resolve(h)
        fail(h.result_future)
        discarded("PICK_RESULT_FAILED (the result future raised)", picks=2)
        routed("result", "PICK_RESULT_FAILED")

        m = confirmed()
        node._pick_available = False
        settle(node, m)
        discarded("NO_PICK_CLIENT at the confirmation's arrival pass", picks=1)
        routed("no client", gw.ACK_NO_PICK_CLIENT)

        m = confirmed()

        def keyboard_at_arrival(mission):
            node._mission_dispatch_context = lambda _m, p: gw.val.DispatchContext(
                armed=node._is_armed(), dry_run=False, link_alive=True,
                teleop_mode="keyboard", teleop_mode_age_sec=0.1, nav2_available=True,
                datum_unchanged=True, line_calibration_unchanged=True, pose=p,
                arm_ready=True)
            gw.val.validate_dispatch = ORIGINAL_VALIDATE
            real_on_arrival(mission)
        node._on_arrival = keyboard_at_arrival
        settle(node, m)
        node.__dict__.pop("_on_arrival", None)
        discarded("state refusal at the confirmation's arrival pass (mode left autonomous)",
                  picks=1)
        routed("state refusal", gw.val.MODE_NOT_AUTONOMOUS)

        m = confirmed()

        def cancel_lands(mission, _pose):
            node._cancel_mission("fixture: a cancel lands after the gates", node._ros_now())
            return None
        node._state_refusal = cancel_lands
        settle(node, m)
        node.__dict__.pop("_state_refusal", None)
        discarded("a cancel landing at the confirmation's arrival pass", picks=1)
        routed("cancel at arrival", "CANCELLED")

        m = confirmed()
        with node._mission_lock:
            m.settling = False
            m.cancel_requested_at_sec = node._ros_now()
            m.cancel_reason = "fixture: cancelled before the settle began"
        node._begin_settle(m, phase="confirm")
        discarded("a cancel before the confirmation's settle began", picks=1)
        routed("cancel before settle", "CANCELLED")
        check("  ... and named as such", routes[-1][1].startswith("before the settle"),
              routes[-1][1])

        m = confirmed()
        settle(node, m)
        h = FakeHandle()
        node._pick_client.futures[-1].resolve(h)
        r = setarm(node, False)
        check("operator disarm during the confirmed pick: the pick is cancelled",
              r.success and h.cancels == 1, h.cancels)
        h.result_future.resolve(Outcome(GoalStatus.STATUS_CANCELED))
        discarded("  ... its CANCELED result (OPERATOR kept, not overwritten)", picks=2,
                  trigger="OPERATOR")

        print("   a successful pick whose acknowledgement is refused (USER DECISION "
              "2026-11-11, '3 a')")
        m = confirmed()
        settle(node, m)
        h = FakeHandle()
        node._pick_client.futures[-1].resolve(h)
        setarm(node, False)
        h.result_future.resolve(Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
        discarded("operator disarm during the pick, then a LATE SUCCESS", picks=2,
                  trigger="OPERATOR")
        routed("late success", f"ACK_REFUSED_{gw.ACK_DISARMED}")

        m = confirmed()
        settle(node, m)
        h = FakeHandle()
        node._pick_client.futures[-1].resolve(h)
        # The window runs out with no poll in between (white-box: the expiry instant).
        node._arming._expires_at_sec = node._safety_now() - 0.01
        h.result_future.resolve(Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
        discarded("arming window expired during the pick, then a success", picks=2,
                  trigger="TIMEOUT")
        routed("expiry", f"ACK_REFUSED_{gw.ACK_DISARMED}")

        m = confirmed()
        settle(node, m)
        node._correlation_holds = lambda _m, **_k: (False, "fixture: a second object appeared")
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
        discarded("success, correlation changed at the acknowledgement", picks=2)
        routed("correlation", f"ACK_REFUSED_{gw.ACK_CORRELATION_CHANGED}")

        m = confirmed()
        settle(node, m)
        pub, node._goal_done_pub = node._goal_done_pub, None
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
        node._goal_done_pub = pub
        discarded("success, no acknowledgement publisher", picks=2)
        routed("no publisher", "ACK_REFUSED_NO_ACK_PUBLISHER")

        m = confirmed()
        settle(node, m)
        m.target_id = ""
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
        check("success, no id to name: disarmed PICK_CONFIRM, released, nothing "
              "acknowledged, no NavigateToPose",
              not node._is_armed() and node._arming.last_disarm_trigger == "PICK_CONFIRM"
              and node._mission is None and not node._acknowledged
              and not node._goal_done_pub.msgs and not node._nav_client.goals
              and m.discarded)
        routed("no id", f"ACK_REFUSED_{gw.ACK_NOT_CORRELATED}")

        print("   the release of a cancelled confirmed pick (re-audit item 1)")
        m = confirmed()
        settle(node, m)
        h = FakeHandle()
        node._pick_client.futures[-1].resolve(h)
        node._cancel_mission("fixture: cancel, never confirmed", node._ros_now())
        node._nav2_available = False
        CLOCK[0] += 2.0 * node._cancel_confirm_timeout_sec + 1.0
        node._check_cancel_timeout(node._ros_now())
        node._supervise_mission(m, node._ros_now())
        discarded("cancelling, Nav2 gone, no pick result: released as a discard", picks=2)
        routed("release", "CANCEL_UNCONFIRMED")
        h.result_future.resolve(Outcome(GoalStatus.STATUS_CANCELED))
        check("  ... a result arriving after the release counts nothing more",
              node._attempts.get("7") == 1 and m.discarded)

        print("   two callers discard concurrently")
        import threading
        m = confirmed()
        settle(node, m)
        calls = []
        real_disarm = node._arming.disarm

        def slow_disarm(trigger, now, detail=""):
            calls.append(trigger)
            time.sleep(0.05)
            return real_disarm(trigger, now, detail)
        node._arming.disarm = slow_disarm
        gate = threading.Barrier(2)

        def caller(tag):
            gate.wait()
            node._discard_pending(m, f"fixture caller {tag}")
        threads = [threading.Thread(target=caller, args=(t,)) for t in "AB"]
        for t in threads:
            t.start()
        for t in threads:
            t.join(5.0)
        node._arming.disarm = real_disarm
        check("two concurrent discards: ONE disarm request", calls == ["PICK_CONFIRM"], calls)
        discarded("  ... counted once", picks=2)

        print("   the arm's STOPPED result before its new stop epoch (USER DECISION "
              "2026-11-11, SR-13)")
        m = confirmed()
        settle(node, m)
        node._last_disarm_error = False
        answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason="STOPPED", message="Space"))
        discarded("STOPPED before the epoch: held off, labelled a Space", picks=2,
                  trigger="MODE_CHANGE")
        check("  ... with the ERROR diagnostic raised", node._last_disarm_error is True)
        routed("stopped", "AIMED_STOPPED")
        node._on_arm_stop_epoch(SimpleNamespace(data=42))
        check("  ... the late epoch changes nothing and sends nothing",
              node._arming.last_disarm_trigger == "MODE_CHANGE" and len(goals(node)) == 2
              and node._attempts.get("7") == 1 and not node._nav_client.goals)

        print("   trash_goal_done publish raises (USER DECISION 2026-11-11)")
        m = confirmed()
        settle(node, m)
        node._goal_done_pub = RaisingPub()
        try:
            answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
            escaped = None
        except Exception as exc:  # noqa: BLE001
            escaped = exc
        check("publish raises: nothing escapes the result callback (the executor and "
              "the gateway keep spinning)", escaped is None, repr(escaped))
        discarded("  ... publish raised", picks=2)
        routed("publish raised", f"ACK_REFUSED_{gw.ACK_PUBLISH_FAILED}")
        check("  ... publish attempted exactly once, nothing recorded as acknowledged",
              node._goal_done_pub.calls == 1 and not node._acknowledged)

        print("   success")
        m = confirmed()
        settle(node, m)
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
        check("a SUCCESSFUL confirmed pick: acknowledged, mission continues ARMED (E11 item 6)",
              node._is_armed() and node._acknowledged == ["7"]
              and [x.data for x in node._goal_done_pub.msgs] == ["7"]
              and "7" not in node._blacklist and node._mission is None)
    finally:
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        node.destroy_node()

    print("3d. automatic: OUT_OF_REACH still re-approaches (E8 regression)")
    gw, node = build("automatic")
    try:
        m = reset(gw, node)
        node._on_arrival(m)
        answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason="OUT_OF_REACH", message="arm"))
        check("automatic, OUT_OF_REACH: still ARMED, not blacklisted, slot released for "
              "the ordinary dispatch",
              node._is_armed() and "7" not in node._blacklist and node._mission is None
              and node._last_reason == "AIMED_OUT_OF_REACH")
        m2 = reset(gw, node)
        node._correlation_holds = lambda _m, **_k: (False, "x")
        node._on_arrival(m2)
        check("automatic, correlation lost at arrival: counted as before, still armed",
              node._is_armed() and "7" not in node._blacklist
              and node._last_reason == gw.ACK_CORRELATION_CHANGED)
    finally:
        node.destroy_node()


def real_dispatch_world(gw, node):
    """The §3e fixture: the node's own resolution and dispatch path; only the robot
    pose and the clock stay stubbed (reset). Returns feed_world(), which re-feeds target
    7, its goal, a live link, teleop mode "autonomous" and an idle arm at home."""
    from types import SimpleNamespace
    from gripperx_arm_msgs.msg import ArmState
    from gripperx_external import line_frame as lf
    from gripperx_external.geodesy import Datum
    from gripperx_external_msgs.msg import ExternalGoal, ExternalTarget, ExternalTargetList
    reset(gw, node)
    node._mission = None
    # The real resolution and dispatch path: only the robot pose and the clock stay
    # stubbed (reset); context, correlation, both gates are the node's own.
    for name in ("_make_context", "_mission_dispatch_context", "_correlation_holds",
                 "_recorrelate_mission"):
        node.__dict__.pop(name, None)
    gw.val.validate_dispatch = ORIGINAL_VALIDATE
    gid = "fixture-map-publisher"
    node.get_publishers_info_by_topic = lambda *_a, **_k: [
        SimpleNamespace(endpoint_gid=gid)]
    datum = Datum(48.2000000, 11.6000000, from_topic=True, stamp_sec=time.time())
    node._datum_tracker.update(datum)
    node._line_cal = lf.compute_calibration(
        (-2.0, 0.0), (2.0, 0.0), session="check_aimed_pick_node", calibration_id=1,
        map_session=lf.map_session_key([gid]))
    lat, lon = lf.our_map_to_latlon(datum, node._line_cal, *OBJECT)
    from nav_msgs.msg import OccupancyGrid
    grid = OccupancyGrid()           # a free 4 m x 4 m costmap around the map origin
    grid.info.resolution, grid.info.width, grid.info.height = 0.05, 80, 80
    grid.info.origin.position.x = grid.info.origin.position.y = -2.0
    grid.data = [0] * (80 * 80)
    node._on_costmap(grid)

    def feed_world():
        t = ExternalTarget(id="7", latitude_deg=lat, longitude_deg=lon,
                           collected=False, is_goal=True)
        node._on_targets(ExternalTargetList(goal_id="7", targets=[t]))
        g = ExternalGoal(target_id="7", object_latitude_deg=lat,
                         object_longitude_deg=lon)
        g.header.stamp.sec = int(time.time())
        node._on_goal(g)
        node._link_connected = True
        node._teleop_mode, node._teleop_mode_mono_sec = "autonomous", node._safety_now()
        s = ArmState(known=True, idle=True, at_home=True, gripper_at_home=True)
        s.header.stamp = node.get_clock().now().to_msg()
        node._on_arm_state(s)

    return feed_world


def run_dispatch_after_discard():
    print("3e. a FULL _dispatch_tick after a discard sends no NavigateToPose (re-audit "
          "item 5)")
    gw, node = build("confirm_by_rearm", arm_state_ready_hold_sec=0.0, **BOUNDS)
    try:
        feed_world = real_dispatch_world(gw, node)
        feed_world()
        node._dispatch_tick()
        sent = node._nav_client.goals
        res = node._resolution
        check("fixture: armed with a free slot, the real tick DOES dispatch target 7",
              len(sent) == 1 and node._mission is not None
              and node._mission.target_id == "7",
              f"goals={len(sent)} verdict={res.result.verdict if res else None} "
              f"reason={res.result.reason if res else None} "
              f"block={node._last_dispatch_block}")
        mission = node._mission
        if mission is None:
            return
        # Drive the dispatched mission through the confirm flow to a discard: arrival,
        # preview, PICK_CONFIRM disarm, re-arm, confirmed pick, an arm refusal.
        node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
        settle(node, mission, ticks=8)
        check("  (fixture: the dispatched mission previews at arrival)",
              len(goals(node)) == 1 and goals(node)[0].preview, mission.state)
        answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "PREVIEW_OK", "", OBJECT))
        feed_world()
        setarm(node, True)
        settle(node, mission, ticks=8)
        check("  (fixture: the re-arm sends the confirmed pick)",
              len(goals(node)) == 2 and not goals(node)[1].preview, mission.state)
        answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason="OUT_OF_REACH", message="x"))
        check("  (fixture: discarded, slot free, held off)",
              node._mission is None and "7" in node._blacklist and mission.discarded)

        for label, arm in (("disarmed", False), ("operator armed again", True)):
            if arm:
                setarm(node, True)
            feed_world()
            for _ in range(3):
                CLOCK[0] += 0.5
                node._dispatch_tick()
            res = node._resolution
            check(f"after the discard, {label}: three full ticks, NO NavigateToPose "
                  "(no re-approach)",
                  len(node._nav_client.goals) == 1 and node._mission is None,
                  f"goals={len(node._nav_client.goals)}")
            check("  ... the target resolves and is refused as BLACKLISTED",
                  res is not None and res.result.reason == gw.val.BLACKLISTED,
                  res.result.reason if res else None)
    finally:
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        node.destroy_node()


def run_stopped_every_mode():
    print("3f. a STOPPED pick result disarms (MODE_CHANGE) in EVERY mode, before the slot "
          "is released (USER DECISION 2026-11-11)")
    for mode in ("off", "automatic"):
        gw, node = build(mode, arm_state_ready_hold_sec=0.0)
        try:
            feed_world = real_dispatch_world(gw, node)
            feed_world()
            node._dispatch_tick()
            mission = node._mission
            check(f"{mode} (fixture: the real tick dispatches target 7)",
                  len(node._nav_client.goals) == 1 and mission is not None,
                  f"block={node._last_dispatch_block}")
            if mission is None:
                continue
            node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
            settle(node, mission, ticks=8)
            check(f"{mode} (fixture: one real pick in flight, no preview)",
                  len(goals(node)) == 1 and not goals(node)[0].preview, mission.state)
            if len(goals(node)) != 1:
                continue
            armed_at_release = []
            real_clear = node._clear_mission

            def spy_clear(m):
                armed_at_release.append(node._is_armed())
                return real_clear(m)
            node._clear_mission = spy_clear
            epoch = node._arm_stop_epoch
            node._last_disarm_error = False
            feed_world()                      # teleop held "autonomous", arm idle at home
            answer(node, Outcome(GoalStatus.STATUS_ABORTED, reason=gw.ARM_REASON_STOPPED,
                                 message="Space"))
            node.__dict__.pop("_clear_mission", None)
            max_attempts = int(node.get_parameter("max_attempts_per_target").value)
            check(f"{mode}, STOPPED with mode still autonomous and no new stop epoch: "
                  "disarmed MODE_CHANGE, ERROR diagnostic",
                  not node._is_armed() and node._arming.last_disarm_trigger == "MODE_CHANGE"
                  and node._last_disarm_error is True and node._arm_stop_epoch == epoch
                  and node._teleop_mode == "autonomous")
            check("  ... the gate closed BEFORE the mission slot was released",
                  armed_at_release == [False], armed_at_release)
            check("  ... attempt accounting as before: one attempt, PICK_FAILED, not held "
                  "off below max_attempts_per_target, nothing acknowledged",
                  node._attempts.get("7") == 1 < max_attempts and node._mission is None
                  and node._last_reason == gw.ACK_PICK_FAILED
                  and "7" not in node._blacklist and not node._acknowledged)
            for _ in range(3):
                feed_world()
                CLOCK[0] += 0.5
                node._dispatch_tick()
            check("  ... three full ticks afterwards: NO NavigateToPose, slot stays free",
                  len(node._nav_client.goals) == 1 and node._mission is None,
                  f"goals={len(node._nav_client.goals)} block={node._last_dispatch_block}")
        finally:
            gw.val.validate_dispatch = ORIGINAL_VALIDATE
            node.destroy_node()

    print("3h. the gateway cancels a pick while armed and a Space lands in that cancel: "
          "the CANCELED result with reason STOPPED disarms like STOPPED")

    def frame_relock(node, _mission):
        node._note_frame_relock(1)
        node._note_frame_relock(2)               # their re-lock: cancels, never disarms

    def correlation_lost(node, _mission):
        # The supervisor does not re-take the correlation while the arm picks (its
        # cancel is the navigation goal's); this is that cancel's call, as it would land.
        node._cancel_mission("CORRELATION_LOST: target 7 can no longer be named "
                             "uniquely by its own fix - fixture", node._safety_now())
    for mode, why, trigger in (("off", "OCTOPUS_FRAME_RELOCK", frame_relock),
                               ("automatic", "OCTOPUS_FRAME_RELOCK", frame_relock),
                               ("automatic", "CORRELATION_LOST", correlation_lost)):
        gw, node = build(mode, arm_state_ready_hold_sec=0.0)
        try:
            feed_world = real_dispatch_world(gw, node)
            feed_world()
            node._dispatch_tick()
            mission = node._mission
            if mission is None:
                check(f"{mode} (fixture: the real tick dispatches target 7)", False,
                      f"block={node._last_dispatch_block}")
                continue
            node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
            settle(node, mission, ticks=8)
            if len(goals(node)) != 1:
                check(f"{mode} (fixture: one real pick in flight)", False, mission.state)
                continue
            h = FakeHandle()
            node._pick_client.futures[-1].resolve(h)
            trigger(node, mission)
            check(f"{mode} (fixture: {why} cancelled the pick, still armed)",
                  h.cancels == 1 and node._is_armed()
                  and mission.cancel_reason.startswith(why),
                  f"cancels={h.cancels} reason={mission.cancel_reason!r}")
            armed_at_release = []
            real_clear = node._clear_mission

            def spy_clear(m):
                armed_at_release.append(node._is_armed())
                return real_clear(m)
            node._clear_mission = spy_clear
            feed_world()                      # teleop held "autonomous", arm idle at home
            h.result_future.resolve(Outcome(GoalStatus.STATUS_CANCELED,
                                            reason=gw.ARM_REASON_STOPPED, message="Space"))
            node.__dict__.pop("_clear_mission", None)
            check(f"{mode}/{why}, CANCELED with reason STOPPED: disarmed MODE_CHANGE BEFORE the "
                  "slot was released",
                  not node._is_armed() and node._arming.last_disarm_trigger == "MODE_CHANGE"
                  and armed_at_release == [False] and node._mission is None,
                  f"armed={node._is_armed()} trigger={node._arming.last_disarm_trigger} "
                  f"release={armed_at_release}")
            for _ in range(3):
                feed_world()
                CLOCK[0] += 0.5
                node._dispatch_tick()
            check("  ... three full ticks afterwards: NO NavigateToPose, slot stays free",
                  len(node._nav_client.goals) == 1 and node._mission is None,
                  f"goals={len(node._nav_client.goals)} block={node._last_dispatch_block}")
        finally:
            gw.val.validate_dispatch = ORIGINAL_VALIDATE
            node.destroy_node()

    print("3g. the arm's STOPPED literal has one value on both sides")
    from gripperx_external import goal_gateway_node as gw
    import re
    src_root = os.path.dirname(_PKG)
    arm = os.path.join(src_root, "gripperx_arm", "gripperx_arm", "arm_action_server.py")
    action = os.path.join(src_root, "gripperx_arm_msgs", "action", "PickPlastic.action")
    if not (os.path.isfile(arm) and os.path.isfile(action)):
        print(f"  [SKIP] arm sources not beside this package ({arm})")
        return
    with open(arm) as f:
        set_reasons = re.findall(r"result\.reason\s*=\s*['\"]([A-Z_]+)['\"]", f.read())
    with open(action) as f:
        documented = f.read()
    check("the arm sets result.reason to the gateway's ARM_REASON_STOPPED",
          gw.ARM_REASON_STOPPED in set_reasons, sorted(set(set_reasons)))
    check("  ... and PickPlastic.action documents that value",
          re.search(rf"\b{re.escape(gw.ARM_REASON_STOPPED)}\b", documented) is not None)


def run_canceled_no_reapproach():
    print("3i. audit N6: a CANCELED pick whose reason never allows a re-approach is counted "
          "and held off, as ABORTED")
    from gripperx_external import goal_gateway_node as gw
    gw, node = build("automatic")
    try:
        for reason in sorted(gw.AIM_NO_REAPPROACH):
            mission = reset(gw, node)
            node._on_arrival(mission)
            answer(node, Outcome(GoalStatus.STATUS_CANCELED, reason=reason, message="arm"))
            check(f"CANCELED + {reason}: counted and held off, nothing acknowledged",
                  node._attempts.get("7") == 1 and "7" in node._blacklist
                  and node._mission is None and not node._acknowledged
                  and node._last_reason == f"AIMED_{reason}",
                  f"attempts={node._attempts} blacklist={node._blacklist} "
                  f"last={node._last_reason}")
        for reason in ("", "CANCELED", "OUT_OF_REACH"):
            mission = reset(gw, node)
            node._on_arrival(mission)
            answer(node, Outcome(GoalStatus.STATUS_CANCELED, reason=reason, message="arm"))
            check(f"  ... CANCELED + {reason or '<no reason>'}: released as before, not "
                  "counted, not held off",
                  node._attempts.get("7") is None and "7" not in node._blacklist
                  and node._mission is None)
    finally:
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        node.destroy_node()

    # The auditor's race on the real path: our frame re-lock cancel (gate stays open)
    # is pending when the arm's base watch fires.
    gw, node = build("automatic", arm_state_ready_hold_sec=0.0)
    try:
        feed_world = real_dispatch_world(gw, node)
        feed_world()
        node._dispatch_tick()
        mission = node._mission
        if mission is None:
            check("N6 (fixture: the real tick dispatches target 7)", False,
                  f"block={node._last_dispatch_block}")
            return
        node._on_nav_result(mission, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
        settle(node, mission, ticks=8)
        if len(goals(node)) != 1:
            check("N6 (fixture: one real pick in flight)", False, mission.state)
            return
        h = FakeHandle()
        node._pick_client.futures[-1].resolve(h)
        node._note_frame_relock(1)
        node._note_frame_relock(2)
        check("N6 (fixture: the re-lock cancelled the pick, still armed)",
              h.cancels == 1 and node._is_armed(), f"cancels={h.cancels}")
        feed_world()
        h.result_future.resolve(Outcome(GoalStatus.STATUS_CANCELED,
                                        reason="BASE_MOVED_DURING_PICK", message="base"))
        check("CANCELED + BASE_MOVED_DURING_PICK during a re-lock cancel: counted, held off",
              node._attempts.get("7") == 1 and "7" in node._blacklist
              and node._mission is None,
              f"attempts={node._attempts} blacklist={node._blacklist}")
        for _ in range(3):
            feed_world()
            CLOCK[0] += 0.5
            node._dispatch_tick()
        check("  ... three full ticks afterwards: target 7 is NOT dispatched again",
              len(node._nav_client.goals) == 1,
              f"goals={len(node._nav_client.goals)} block={node._last_dispatch_block}")
    finally:
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        node.destroy_node()


def run_localisation_and_arm_gate():
    print("6. aimed: the pick is judged only on a SETTLED localisation (user 2026-10-13)")
    gw, node = build("automatic")
    try:
        m = reset(gw, node)
        node._on_nav_result(m, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
        settle(node, m, ticks=2)
        check("still base, window not yet over: no pick", len(goals(node)) == 0)
        settle(node, m, ticks=3)
        check("  ... once the pose held for grasp.localisation_settle_sec: the pick",
              len(goals(node)) == 1)

        m = reset(gw, node)
        node._on_nav_result(m, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
        settle(node, m, ticks=3)
        node._robot_pose = lambda: ((0.006, 0.0, 0.0), 0.05, "")     # a late correction
        settle(node, m, ticks=2)
        check("a late localisation correction (6 mm) restarts the window: no pick yet",
              len(goals(node)) == 0)
        settle(node, m, ticks=3)
        check("  ... and the pick follows once it held again", len(goals(node)) == 1)

        m = reset(gw, node)
        node._on_nav_result(m, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
        k = iter(range(1000))
        node._robot_pose = lambda: ((0.006 * (next(k) % 2), 0.0, 0.0), 0.05, "")
        settle(node, m, ticks=20)
        check("a localisation that never holds: LOCALISATION_NOT_SETTLED, no pick, held "
              "off (no re-approach)",
              len(goals(node)) == 0 and node._last_reason == "LOCALISATION_NOT_SETTLED"
              and "7" in node._blacklist)
    finally:
        node.destroy_node()
    gw, node = build("off")
    try:
        m = reset(gw, node)
        node._on_nav_result(m, FakeFuture(Outcome(GoalStatus.STATUS_SUCCEEDED)))
        settle(node, m, ticks=2)
        check("mode off: the settle step is today's (no window)", len(goals(node)) == 1)
    finally:
        node.destroy_node()

    print("7. ARM_NOT_HOME: no dispatch unless /arm/state is fresh, idle and at home")
    from gripperx_arm_msgs.msg import ArmState

    def st(node, idle=True, home=True, gripper=True, known=True, age=0.0):
        m = ArmState(known=known, idle=idle, at_home=home, gripper_at_home=gripper,
                     owner="" if idle else "pick")
        m.header.stamp = (node.get_clock().now()
                          - rclpy.duration.Duration(seconds=age)).to_msg()
        return m

    gw, node = build("off", arm_state_ready_hold_sec=0.0)
    try:
        node._arm_state = None
        check("no state: not ready", node._arm_ready()[0] is False)
        for kw, ok in (({}, True), ({"idle": False}, False), ({"home": False}, False),
                       ({"known": False}, False)):
            node._on_arm_state(st(node, **kw))
            check(f"{kw or 'idle, at home, known'}: ready={ok}", node._arm_ready()[0] is ok,
                  node._arm_ready()[1])
        node._on_arm_state(st(node))
        node._arm_state = (node._arm_state[0], time.monotonic() - 5.0)
        check("stale by receive time: not ready", node._arm_ready()[0] is False)
        node._on_arm_state(st(node, age=5.0))
        check("a latched OLD state (stamp 5 s old, just received): not ready (F3)",
              node._arm_ready()[0] is False, node._arm_ready()[1])
        reset(gw, node)
        node._arm_state = None
        real = gw.GoalGatewayNode._mission_dispatch_context
        d = real(node, node._mission, (0.0, 0.0, 0.0))
        check("the gateway's own dispatch context carries it",
              d.arm_ready is False and "no /arm/state" in d.arm_detail)
        import dataclasses
        r = ORIGINAL_VALIDATE(dataclasses.replace(
            d, armed=True, dry_run=False, link_alive=True, teleop_mode="autonomous",
            teleop_mode_age_sec=0.1, nav2_available=True, datum_unchanged=True,
            line_calibration_unchanged=True), None)
        check("  ... and the one dispatch gate refuses with ARM_NOT_HOME",
              r.reason == gw.val.ARM_NOT_HOME, r.reason)

        print("   in flight (F6): a goal driving is cancelled with its own reason")
        m = reset(gw, node)
        m.state = gw.NAV_NAVIGATING
        node._mission_dispatch_context = lambda _m, p: dataclasses.replace(
            d, armed=True, dry_run=False, link_alive=True, teleop_mode="autonomous",
            teleop_mode_age_sec=0.1, nav2_available=True, datum_unchanged=True,
            line_calibration_unchanged=True, pose=p,
            arm_ready=node._arm_ready()[0], arm_detail=node._arm_ready()[1])
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        node._on_arm_state(st(node, idle=False))
        node._supervise_mission(m, node._ros_now())
        check("ARM_NOT_HOME while navigating cancels the goal, counted",
              m.cancelling and m.cancel_reason.startswith("ARM_NOT_HOME")
              and node._arm_not_home_cancels == 1, m.cancel_reason)
    finally:
        node.destroy_node()

    print("   hysteresis (F2): ready only after arm_state_ready_hold_sec, flapping resets")
    gw, node = build("off", arm_state_ready_hold_sec=0.6)
    try:
        node._on_arm_state(st(node))
        check("ready message just arrived: not yet", node._arm_ready()[0] is False)
        time.sleep(0.4)
        node._on_arm_state(st(node, home=False))
        time.sleep(0.1)
        node._on_arm_state(st(node))
        time.sleep(0.3)
        node._on_arm_state(st(node))
        check("a flap inside the window restarts it", node._arm_ready()[0] is False)
        time.sleep(0.4)
        node._on_arm_state(st(node))
        check("  ... continuously ready for the hold: ready", node._arm_ready()[0] is True)
    finally:
        node.destroy_node()

    print("   fixed P end-to-end, gate on, object in the jaws after the pick (F1 a/b)")
    for variant, home in (("b (gripper part of at_home)", False), ("a (limbs only)", True)):
        gw, node = build("off", arm_state_ready_hold_sec=0.0)
        try:
            m = reset(gw, node)
            node._on_arrival(m)
            answer(node, Outcome(GoalStatus.STATUS_SUCCEEDED, True, "", "done"))
            ok_ack = node._acknowledged == ["7"]
            # what the arm node publishes after it: limbs home, gripper on the object
            node._on_arm_state(st(node, home=home, gripper=False))
            ready, why = node._arm_ready()
            check(f"variant {variant}: picked and acknowledged; next dispatch "
                  + ("allowed" if home else "BLOCKED (ARM_NOT_HOME)"),
                  ok_ack and ready is home, why)
        finally:
            node.destroy_node()
    real_sim = gw.is_simulation_domain
    gw.is_simulation_domain = lambda: False
    try:
        _gw, node = build("off", dispatch_requires_arm_home=False)
        node.destroy_node()
        check("dispatch_requires_arm_home false outside simulation: refused", False)
    except Exception as exc:  # noqa: BLE001
        check("dispatch_requires_arm_home false outside simulation: refused",
              "dispatch_requires_arm_home" in str(exc), str(exc)[:80])
    finally:
        gw.is_simulation_domain = real_sim


def run_startup_refusals():
    print("5. startup refusals of the mode")
    for mode, extra, why in (("sometimes", {}, "unknown value"),
                             ("confirm_by_rearm", {}, "pending bounds unset")):
        try:
            _gw, node = build(mode, **extra)
            node.destroy_node()
            check(f"{why}: refused", False)
        except Exception as exc:  # noqa: BLE001
            check(f"{why}: refused", "grasp.aimed_pick" in str(exc), str(exc)[:90])
    from gripperx_external import goal_gateway_node as gw
    real = gw.is_simulation_domain
    real_logger = gw.GoalGatewayNode.get_logger
    warns = []

    class _Spy:
        def __init__(self, inner):
            self._inner = inner

        def warn(self, msg, *a, **k):
            warns.append(str(msg))
            return self._inner.warn(msg, *a, **k)

        def __getattr__(self, name):
            return getattr(self._inner, name)

    gw.is_simulation_domain = lambda: False
    gw.GoalGatewayNode.get_logger = lambda self: _Spy(real_logger(self))
    try:
        _gw, node = build("automatic")
        mode = node._aimed_mode
        node.destroy_node()
        check("automatic outside a simulation domain: starts (user decision 2026-11-16)",
              mode == "automatic")
        check("... with the loud prototype-assumption WARN",
              any("NON-SIMULATION" in w and "TO-VERIFY" in w and "0.361" in w
                  for w in warns), str(warns)[:90])
    except Exception as exc:  # noqa: BLE001
        check("automatic outside a simulation domain: starts", False, str(exc)[:90])
    finally:
        gw.is_simulation_domain = real
        gw.GoalGatewayNode.get_logger = real_logger


def run_real_config_files():
    print("5b. the shipped real-robot configs (read from the files, not constants)")
    import os
    import yaml
    src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    ext = os.path.join(src, "gripperx_external", "config", "octopus_link_real.yaml")
    p = yaml.safe_load(open(ext))["/gripperx/external/goal_gateway_node"]["ros__parameters"]
    check("real yaml: auto_pick true", p["auto_pick"] is True)
    check("real yaml: grasp.aimed_pick automatic", p["grasp.aimed_pick"] == "automatic")
    arm = os.path.join(src, "gripperx_arm", "config", "arm_safety_filter.yaml")
    a = yaml.safe_load(open(arm))["arm_action_server"]["ros__parameters"]["aimed"]
    check("real arm yaml: aimed.region_margin_m 0.0 (N7 stays satisfied)",
          a.get("region_margin_m") == 0.0)


def _launch_nodes_with_params(path, executables):
    """{executable: parameters source text} of every Node(...) call in a launch file
    (parsed, not run: the bringup launch files need the robot's packages)."""
    import ast
    with open(path) as f:
        text = f.read()
    tree = ast.parse(text)
    assigned = {}                    # name -> source of what it is assigned (one hop)
    for a in ast.walk(tree):
        if isinstance(a, ast.Assign):
            for t in a.targets:
                if isinstance(t, ast.Name):
                    assigned[t.id] = ast.get_source_segment(text, a.value)
    found = {}
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and getattr(call.func, "id", "") == "Node"):
            continue
        kw = {k.arg: k.value for k in call.keywords}
        exe = kw.get("executable")
        if isinstance(exe, ast.Constant) and exe.value in executables:
            par = kw.get("parameters")
            src = ast.get_source_segment(text, par) if par is not None else ""
            names = [n.id for n in ast.walk(par) if isinstance(n, ast.Name)] if par else []
            found.setdefault(exe.value, []).append(
                src + "".join(" " + assigned.get(n, "") for n in names))
    return found


def run_tf_age_one_bound():
    print("8. SR-16 E3 / audit F4 (user decision 2026-11-13, option a): ONE shared, "
          "startup-only TF age bound for the gateway and the arm")
    import yaml
    from shared_tf import SHARED_TF_FILE, shared_tf_params
    shared = shared_tf_params() if os.path.isfile(SHARED_TF_FILE) else {}
    check("the shared file carries max_tf_age_sec under /** (one value for both nodes)",
          isinstance(shared.get("max_tf_age_sec"), float), SHARED_TF_FILE)
    gw, node = build("off")
    try:
        check("the gateway runs on the ACTIVE shared value",
              float(node.get_parameter("max_tf_age_sec").value)
              == shared.get("max_tf_age_sec"))
        before = node.get_parameter("max_tf_age_sec").value
        res = node.set_parameters([Parameter("max_tf_age_sec", value=float(before) + 5.0)])[0]
        check("max_tf_age_sec cannot be changed on a running gateway (SR-15 rule 14)",
              not res.successful and node.get_parameter("max_tf_age_sec").value == before,
              res.reason)
    finally:
        node.destroy_node()
    try:
        gw.GoalGatewayNode(parameter_overrides=[
            Parameter("expected_domain_id", value=220),
            Parameter("grasp.aimed_pick", value="off")]).destroy_node()
        check("without the shared file the gateway refuses to start (no default)", False)
    except Exception as exc:  # noqa: BLE001
        check("without the shared file the gateway refuses to start (no default)",
              "max_tf_age_sec" in str(exc), str(exc)[:120])
    src_root = os.path.dirname(_PKG)

    def carries(doc, key):
        if isinstance(doc, dict):
            return key in doc or any(carries(v, key) for v in doc.values())
        return False
    for path in (os.path.join(_PKG, "config", "octopus_link_real.yaml"),
                 os.path.join(_PKG, "config", "octopus_link_twin.yaml"),
                 os.path.join(src_root, "gripperx_arm", "config", "arm_safety_filter.yaml")):
        with open(path) as f:
            doc = yaml.safe_load(f)
        check(f"{os.path.basename(path)} carries no max_tf_age_sec of its own",
              not carries(doc, "max_tf_age_sec"))
    for rel, exes in (
        ("gripperx_external/launch/octopus_link.launch.py", {"goal_gateway_node"}),
        ("gripperx_arm/launch/arm.launch.py", {"arm_action_server"}),
        ("gripperx_bringup/launch/real_robot.launch.py", {"arm_action_server"}),
        ("gripperx_bringup/launch/teleop_real.launch.py", {"arm_action_server"}),
    ):
        nodes = _launch_nodes_with_params(os.path.join(src_root, rel), exes)
        check(f"{rel}: every {sorted(exes)} Node loads tf_freshness.yaml",
              set(nodes) == exes and all("tf_freshness.yaml" in p
                                         for ps in nodes.values() for p in ps),
              repr(nodes)[:160])


ORIGINAL_VALIDATE = None


def main():
    global ORIGINAL_VALIDATE
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=false"])
    from gripperx_external import goal_gateway_node as gw
    ORIGINAL_VALIDATE = gw.val.validate_dispatch
    try:
        run_off_and_automatic()
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        run_confirm()
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        run_confirm_failures()
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        run_dispatch_after_discard()
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        run_stopped_every_mode()
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        run_canceled_no_reapproach()
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        run_startup_refusals()
        run_real_config_files()
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        run_localisation_and_arm_gate()
        run_tf_age_one_bound()
    finally:
        gw.val.validate_dispatch = ORIGINAL_VALIDATE
        rclpy.shutdown()
    print("=" * 78)
    if FAILURES:
        print(f"FAILED: {len(FAILURES)} check(s)")
        for label in FAILURES:
            print(f"  - {label}")
        return 1
    print("All aimed-pick node checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
