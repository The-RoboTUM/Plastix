#!/usr/bin/env python3
"""The LIST-LOSS HOLD of the correlation gate, on the REAL gateway node.

USER DECISION 2026-11-16 (REQUIREMENTS.md SR-16, DECIDED block): an EMPTY or
STALE target list is held for `correlation_list_loss_hold_sec` after the
mission's last unique correlation, as long as no grasp was requested. The hold
width and `occlusion_latch_enabled` are READ FROM THE ACTIVE CONFIG
(octopus_link_real.yaml), never duplicated here.

Builds the node but spins nothing, sends nothing and moves nothing: both action
clients are recorders, the safety clock and the correlation are stubbed.

    ROS_DOMAIN_ID=220 python3 test/check_correlation_hold.py
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_PKG, "src"))

import rclpy  # noqa: E402
import yaml  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from shared_tf import shared_tf_overrides  # noqa: E402

REAL_CONFIG = os.path.join(_PKG, "config", "octopus_link_real.yaml")
OBJECT = (0.40, 0.06)   # fixture: 0.072 m from the grasp point of a robot at the origin
FAILURES = []


def check(label, condition, detail=""):
    print(f"  [{'PASS' if condition else 'FAIL'}] {label}" + (f"  ({detail})" if detail else ""),
          flush=True)
    if not condition:
        FAILURES.append(label)


def active_config():
    with open(REAL_CONFIG) as f:
        return yaml.safe_load(f)["/gripperx/external/goal_gateway_node"]["ros__parameters"]


class FakeFuture:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, fn):
        self.callbacks.append(fn)


class FakeHandle:
    def __init__(self):
        self.accepted = True
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1
        return FakeFuture()


class Recorder:
    def __init__(self):
        self.goals = []

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return FakeFuture()

    def server_is_ready(self):
        return True


class Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


#: The safety clock (`_safety_now`), driven by the checks.
NOW = [5000.0]


def build(cfg):
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
        Parameter("occlusion_latch_enabled", value=cfg["occlusion_latch_enabled"]),
        Parameter("correlation_list_loss_hold_sec",
                  value=cfg["correlation_list_loss_hold_sec"]),
    ] + shared_tf_overrides()
    node = gw.GoalGatewayNode(parameter_overrides=overrides)
    return gw, node


def reset(gw, node, armed=True):
    node._safety_now = lambda: NOW[0]
    node._ros_now = lambda: NOW[0]
    node._nav_client = Recorder()
    node._pick_client = Recorder()
    node._pick_available = True
    node._nav2_available = True
    node._auto_pick = True
    node._goal_done_pub = Publisher()
    node._arming.is_armed = lambda _now: armed
    node._robot_pose = lambda: ((0.0, 0.0, 0.0), 0.05, "")
    node._robot_pose_stamped = lambda: ((0.0, 0.0, 0.0), 0.05, "", NOW[0] - 0.05)
    node._mission_dispatch_context = lambda _m, _p: None
    node._make_context = lambda *_a, **_k: None
    node._attempts.clear()
    node._blacklist.clear()
    node._acknowledged.clear()
    gw.val.validate_dispatch = lambda _d, _c: gw.val.ValidationResult(
        verdict=gw.val.VERDICT_ACCEPTED
    )
    mission = gw.Mission(
        target_id="7", object_xy=OBJECT, pose=(0.04, 0.0, 0.0),
        datum_lat=float("nan"), datum_lon=float("nan"), started_at_sec=0.0,
        incoming=object(),
    )
    mission.nav_handle = FakeHandle()
    mission.nav_accepted = True
    node._mission = mission
    return mission


def feed(node, result):
    node._recorrelate_mission = lambda _m, r=result: r


def seen(gw, node, mission):
    """One unique correlation naming the mission's own id, at NOW."""
    feed(node, gw.corr.CorrelationResult(gw.corr.UNIQUE, target_id="7"))
    return node._correlation_holds(mission, hold=gw.CORR_HOLD_DRIVE)[0]


def run(gw, node, window):
    corr = gw.corr
    empty = corr.CorrelationResult(corr.NO_TARGETS, absence=True, detail="empty")
    stale = corr.CorrelationResult(corr.TARGETS_STALE, detail="stale")
    others = corr.CorrelationResult(corr.NO_MATCH, absence=True, detail="others only")
    drive, pick = gw.CORR_HOLD_DRIVE, gw.CORR_HOLD_PICK

    print(f"0. the window comes from the active config: {window} s")
    check("the node runs with the configured window", node._list_loss_hold_sec == window)

    print("1. empty list, inside the window: held for driving, latch NOT voided")
    mission = reset(gw, node, armed=False)
    check("unique once", seen(gw, node, mission))
    NOW[0] += window - 0.1
    feed(node, empty)
    holds, detail = node._correlation_holds(mission, hold=drive)
    check("held (unarmed too)", holds, detail)
    check("  ... latch not voided", mission.latch_void_reason == "")
    check("  ... diagnostics say it is held", "held" in node._mission_correlation_hold)
    node._supervise_mission(mission, NOW[0])
    check("  ... the goal in flight is NOT cancelled", mission.nav_handle.cancels == 0)

    print("2. empty list past the window: CORRELATION_LOST cancel, latch voided")
    NOW[0] += 0.2
    node._supervise_mission(mission, NOW[0])
    check("cancelled", mission.nav_handle.cancels == 1)
    check("  ... reason starts CORRELATION_LOST and says the hold expired",
          mission.cancel_reason.startswith("CORRELATION_LOST")
          and f"hold expired after {window:.1f} s" in mission.cancel_reason,
          mission.cancel_reason)
    check("  ... latch voided", mission.latch_void_reason != "")

    print("3. the window counts from the LAST unique correlation")
    mission = reset(gw, node)
    seen(gw, node, mission)
    NOW[0] += window - 1.0
    seen(gw, node, mission)
    NOW[0] += window - 1.0
    feed(node, empty)
    check("held", node._correlation_holds(mission, hold=drive)[0])

    print("4. no hold once a grasp was requested")
    mission = reset(gw, node)
    seen(gw, node, mission)
    mission.grasp_requested = True
    NOW[0] += 1.0
    feed(node, empty)
    check("refused", not node._correlation_holds(mission, hold=drive)[0])
    check("  ... latch voided", mission.latch_void_reason != "")

    print("5. no hold if the mission was never uniquely correlated")
    mission = reset(gw, node)
    feed(node, empty)
    check("refused", not node._correlation_holds(mission, hold=drive)[0])
    check("  ... 'no list yet' (NO_TARGETS without absence) is no hold either",
          not node._list_loss_remaining(mission, corr.CorrelationResult(corr.NO_TARGETS)))

    print("6. stale list: held for driving, never for a pick or an acknowledgement")
    mission = reset(gw, node)
    seen(gw, node, mission)
    NOW[0] += 1.0
    feed(node, stale)
    check("drive: held", node._correlation_holds(mission, hold=drive)[0])
    check("pick: refused", not node._correlation_holds(mission, hold=pick)[0])
    check("acknowledgement (default): refused", not node._correlation_holds(mission)[0])
    check("  ... latch still not voided inside the window", mission.latch_void_reason == "")
    node._on_arrival(mission)
    check("  ... the arrival gate sends NO pick on a stale list",
          len(node._pick_client.goals) == 0)
    mission = reset(gw, node)
    seen(gw, node, mission)
    NOW[0] += 1.0
    feed(node, empty)
    check("acknowledgement on an empty list (default): refused",
          not node._correlation_holds(mission)[0])
    mission.grasp_requested = True
    node._acknowledge(mission)
    check("  ... _acknowledge publishes nothing", node._goal_done_pub.sent == [])

    print("7. arrival gate on an EMPTY list inside the window: picks, then no more hold")
    mission = reset(gw, node)
    seen(gw, node, mission)
    NOW[0] += 2.0
    feed(node, empty)
    node._on_arrival(mission)
    check("pick sent", len(node._pick_client.goals) == 1)
    check("  ... and the mission is marked grasp_requested", mission.grasp_requested)
    check("  ... so the acknowledgement refuses on the empty list",
          not node._correlation_holds(mission)[0])

    print("8. AMBIGUOUS / another id: immediate refusal and void, inside the window")
    for result in (corr.CorrelationResult(corr.AMBIGUOUS, detail="two"),
                   corr.CorrelationResult(corr.UNIQUE, target_id="8", distance_m=0.1)):
        mission = reset(gw, node)
        seen(gw, node, mission)
        NOW[0] += 1.0
        feed(node, result)
        node._supervise_mission(mission, NOW[0])
        check(f"{result.status} {result.target_id}: cancelled at once",
              mission.nav_handle.cancels == 1
              and mission.cancel_reason.startswith("CORRELATION_LOST"))
        check("  ... latch voided", mission.latch_void_reason != "")
    mission = reset(gw, node, armed=False)
    seen(gw, node, mission)
    NOW[0] += 1.0
    feed(node, corr.CorrelationResult(corr.NO_MATCH, detail="only match collected"))
    check("NO_MATCH that is not absence (unarmed, so no latch): refused, no list-loss hold",
          not node._correlation_holds(mission, hold=drive)[0]
          and node._mission_correlation_hold == "")

    print("9. empty inside the window does not void the NO_MATCH latch")
    if not cfg_latch(node):
        check("occlusion_latch_enabled is true in the active config", False)
        return
    mission = reset(gw, node, armed=True)
    seen(gw, node, mission)
    NOW[0] += 3.0
    feed(node, empty)
    check("empty: held", node._correlation_holds(mission, hold=drive)[0])
    NOW[0] += window   # past the list-loss window: only the latch can hold now
    feed(node, others)
    holds, detail = node._correlation_holds(mission, hold=drive)
    check("other targets back -> NO_MATCH: the armed latch still holds", holds, detail)
    node._arming.is_armed = lambda _now: False
    check("  ... and not when unarmed (latch unchanged, D1)",
          not node._correlation_holds(mission, hold=drive)[0])


def cfg_latch(node):
    return bool(node.get_parameter("occlusion_latch_enabled").value)


def main():
    cfg = active_config()
    window = float(cfg["correlation_list_loss_hold_sec"])
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=false"])
    gw, node = build(cfg)
    original_validate = gw.val.validate_dispatch
    try:
        run(gw, node, window)
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
    print("All list-loss hold checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
