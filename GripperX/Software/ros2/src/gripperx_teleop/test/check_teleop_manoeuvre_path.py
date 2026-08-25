#!/usr/bin/env python3
"""End-to-end check: key press -> cmd_vel that swerve_cmd_node would consume.

The pure-python companion (check_manoeuvres.py) proves the twists are
kinematically reachable. This one proves they actually SURVIVE the wiring:
the real KeyboardTeleopNode and the real TeleopMuxNode are started, keys are
injected, and the resulting `/cmd_vel` — the exact topic swerve_cmd_node
subscribes to — is recorded. Nothing is asserted about code that is not run.

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 src/gripperx_teleop/test/check_teleop_manoeuvre_path.py

SAFETY (SR-1 / SR-8)
--------------------
This publishes on `/cmd_vel`. On the robot's domain that is a movement command.
The module therefore pins itself, BEFORE rclpy is imported, to
ROS_DOMAIN_ID=221 (twin range, deliberately != 20 = GripperX-1) and to
localhost-only discovery, and refuses to run on domain 20. No hardware, no Pi,
no servo is involved: `/hw/steer_states` is fed by a stub node in this process
that stands in for steer_servo_node.
"""

from __future__ import annotations

import os
import sys

# Must happen before rclpy/rmw read the environment.
os.environ["ROS_DOMAIN_ID"] = "221"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
if os.environ["ROS_DOMAIN_ID"] == "20":
    raise SystemExit("refusing to run on the real robot's DDS domain (SR-8)")

import math  # noqa: E402
import time  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402

from gripperx_teleop.keyboard_teleop_node import KeyboardTeleopNode  # noqa: E402
from gripperx_teleop.key_input import PRESS, read_sequence  # noqa: E402
from gripperx_teleop.teleop_mux_node import TeleopMuxNode  # noqa: E402
from gripperx_teleop.manoeuvre import (  # noqa: E402
    CRAB_LEFT,
    SPIN_CW,
    manoeuvre_pose,
)
from gripperx_control.steering_limits import (  # noqa: E402
    DEFAULT_INWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_SIGN,
    SteeringLimits,
)
from gripperx_control.swerve_kinematic_model import FourWIS4WIDKinematicModel  # noqa: E402
from gripperx_geometry.constants import (  # noqa: E402
    HALF_TRACK_KINGPIN as B,
    HALF_WHEELBASE_KINGPIN as A,
    WHEEL_RADIUS_EFFECTIVE as WHEEL_RADIUS,
)

CRAB_SPEED = 0.25
SPIN_SPEED = 0.60
POSE_SCALE = 0.02
LINEAR_VEL = 0.5
RELEASE_SEC = 0.30
ALIGN_TIMEOUT = 5.0          # long on purpose: arming must come from feedback
DRIVE_HOLD = 0.30

# a/b CORRECTED 2026-08-21. They were 0.203 / 0.16556, the pre-2026-08-13
# obsolete-CAD pair, so this check has been asserting against a geometry the
# robot does not have. The live values come from ros2_controllers.yaml, which
# is what swerve_controller actually runs on.
# NOTE b is the half KING-PIN track, not the half contact-point track: the
# kinematics computes at the steering point. 0.16556 was a CONTACT-POINT figure,
# which is why the two looked like a 34 % contradiction and were not one.
# Both stay TO-VERIFY for a much smaller reason - 0.110 against the CAD king-pin
# half-track 0.1087, i.e. 1.2 %, unresolved.
MODEL = FourWIS4WIDKinematicModel(a=A, b=B, wheel_radius=WHEEL_RADIUS)
LIMITS = SteeringLimits.from_outward_inward(
    math.radians(DEFAULT_OUTWARD_LIMIT_DEG),
    math.radians(DEFAULT_INWARD_LIMIT_DEG),
    DEFAULT_OUTWARD_SIGN,
).in_model_order()

CRAB_POSE = manoeuvre_pose(CRAB_LEFT, MODEL, LIMITS, CRAB_SPEED, SPIN_SPEED, [0.0] * 4)
SPIN_POSE = manoeuvre_pose(SPIN_CW, MODEL, LIMITS, CRAB_SPEED, SPIN_SPEED, [0.0] * 4)


class FakeSteerServo(Node):
    """Stands in for steer_servo_node's /hw/steer_states feedback."""

    def __init__(self) -> None:
        super().__init__("fake_steer_servo")
        self.angles = [0.0] * 4
        self._pub = self.create_publisher(Float64MultiArray, "/hw/steer_states", 10)
        self.create_timer(0.02, self._tick)

    def _tick(self) -> None:
        msg = Float64MultiArray()
        msg.data = list(self.angles)
        self._pub.publish(msg)


class Recorder(Node):
    """Stands in for swerve_cmd_node: records exactly what reaches /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("manoeuvre_recorder")
        self.cmd_vel = Twist()
        self.direct_steer = None
        self.direct_steer_count = 0
        self.manoeuvre = ""
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(
            Float64MultiArray, "/teleop/direct_steer", self._on_direct, 10
        )
        self.create_subscription(
            String,
            "/teleop/manoeuvre",
            self._on_manoeuvre,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_vel = msg

    def _on_direct(self, msg: Float64MultiArray) -> None:
        self.direct_steer = list(msg.data)
        self.direct_steer_count += 1

    def _on_manoeuvre(self, msg: String) -> None:
        self.manoeuvre = msg.data


class Harness:
    def __init__(self) -> None:
        rclpy.init(
            args=[
                "--ros-args",
                "-p", "publish_rate_hz:=50.0",
                "-p", f"a:={A}",
                "-p", f"b:={B}",
                "-p", f"wheel_radius:={WHEEL_RADIUS}",
                "-p", f"drive_hold_sec:={DRIVE_HOLD}",
                "-p", f"direct_release_sec:={RELEASE_SEC}",
                "-p", f"align_timeout_sec:={ALIGN_TIMEOUT}",
                "-p", f"crab_speed_m_s:={CRAB_SPEED}",
                "-p", f"spin_speed_rad_s:={SPIN_SPEED}",
                "-p", f"manoeuvre_pose_scale:={POSE_SCALE}",
                "-p", f"linear_vel_m_s:={LINEAR_VEL}",
                "-p", "cmd_timeout_sec:=0.5",
            ]
        )
        self.teleop = KeyboardTeleopNode()
        self.mux = TeleopMuxNode()
        self.servo = FakeSteerServo()
        self.recorder = Recorder()
        self.executor = SingleThreadedExecutor()
        for node in (self.teleop, self.mux, self.servo, self.recorder):
            self.executor.add_node(node)
        self.pump(0.6)                      # discovery

    def pump(self, seconds: float, key: str = "", repeat: float = 0.04) -> None:
        """Spin for `seconds`, re-pressing `key` at a typical auto-repeat rate."""
        end = time.monotonic() + seconds
        last = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if key and (now - last) >= repeat:
                self.teleop.press(key)
                last = now
            self.executor.spin_once(timeout_sec=0.005)

    def shutdown(self) -> None:
        for node in (self.teleop, self.mux, self.servo, self.recorder):
            self.executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    failures = []

    def check(name: str, condition: bool, detail: str = "") -> None:
        print(f"  {name}{(' — ' + detail) if detail else ''} -> "
              f"{'PASS' if condition else 'FAIL'}")
        if not condition:
            failures.append(name)

    def close(a: float, b: float, tol: float = 1e-6) -> bool:
        return abs(a - b) <= tol

    print(f"DDS domain {os.environ['ROS_DOMAIN_ID']}, discovery "
          f"{os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']} — no robot reachable.")
    print(f"crab pose (joint order FL,FR,BL,BR): "
          f"{[round(math.degrees(v), 1) for v in CRAB_POSE]}")
    print(f"spin pose (joint order FL,FR,BL,BR): "
          f"{[round(math.degrees(v), 1) for v in SPIN_POSE]}")
    print()

    h = Harness()
    rec = h.recorder
    try:
        # === 1. Regression: the existing keys are untouched ===============
        print("=== 1. Existing keys unchanged (no arrow ever pressed) ===")
        h.pump(0.5, key="w")
        check(
            "W drives: /cmd_vel linear.x at full speed, no vy/omega",
            close(rec.cmd_vel.linear.x, LINEAR_VEL)
            and close(rec.cmd_vel.linear.y, 0.0)
            and close(rec.cmd_vel.angular.z, 0.0),
            f"linear.x={rec.cmd_vel.linear.x:.3f} linear.y={rec.cmd_vel.linear.y:.3f} "
            f"angular.z={rec.cmd_vel.angular.z:.3f}",
        )
        check(
            "W drives immediately, no alignment wait on a fresh node",
            rec.manoeuvre.startswith("CORNERING") and "armed" in rec.manoeuvre,
            rec.manoeuvre,
        )
        before = rec.direct_steer_count
        h.pump(0.4, key="a")
        check(
            "A/D still travel via /teleop/direct_steer with the counter-rotating pattern",
            rec.direct_steer_count > before
            and rec.direct_steer[0] > 0.0
            and close(rec.direct_steer[0], rec.direct_steer[1])
            and close(rec.direct_steer[2], -rec.direct_steer[0]),
            f"direct_steer={[round(math.degrees(v), 1) for v in rec.direct_steer]} deg",
        )
        h.pump(0.5)
        print()

        # === 2. Crab: linear.y survives the whole path ====================
        print("=== 2. Arrow LEFT — crab left ===")
        h.servo.angles = [0.0] * 4
        h.pump(0.4)
        h.pump(RELEASE_SEC * 0.6, key="left")
        check(
            "RELEASING: zero twist while the direct-steer override still owns the servos",
            close(rec.cmd_vel.linear.y, 0.0) and close(rec.cmd_vel.linear.x, 0.0),
            f"linear.y={rec.cmd_vel.linear.y:.4f}",
        )
        # Counted from here, i.e. entirely inside the manoeuvre — before the
        # first press the node is still cornering and legitimately publishing.
        direct_before = rec.direct_steer_count
        h.pump(0.6, key="left")
        expected_hold = CRAB_SPEED * POSE_SCALE
        check(
            "ALIGNING: pose commanded at the hold scale — linear.y is NON-ZERO at "
            "swerve_cmd_node's input, so it survived teleop_mux",
            close(rec.cmd_vel.linear.y, expected_hold, 1e-4),
            f"linear.y={rec.cmd_vel.linear.y:.4f} (expected {expected_hold:.4f})",
        )
        check(
            "ALIGNING: traction withheld (wheels still measured at 0 deg)",
            abs(rec.cmd_vel.linear.y) < CRAB_SPEED * 0.1 and "aligning" in rec.manoeuvre,
            rec.manoeuvre,
        )
        check(
            "/teleop/direct_steer publishing has STOPPED, so the override lapses",
            rec.direct_steer_count == direct_before,
            f"{rec.direct_steer_count - direct_before} extra messages",
        )
        h.servo.angles = list(CRAB_POSE)          # servos report arrival
        h.pump(0.5, key="left")
        check(
            "wheels arrive -> ARMED: full crab speed on linear.y",
            close(rec.cmd_vel.linear.y, CRAB_SPEED, 1e-4)
            and close(rec.cmd_vel.linear.x, 0.0)
            and close(rec.cmd_vel.angular.z, 0.0),
            f"linear.y={rec.cmd_vel.linear.y:.4f}, state {rec.manoeuvre}",
        )
        # release: dead-man must end the manoeuvre
        h.pump(DRIVE_HOLD + 0.3)
        check(
            "arrow released: manoeuvre ends within the dead-man window (SR-3)",
            close(rec.cmd_vel.linear.y, 0.0) and rec.manoeuvre.startswith("CORNERING"),
            rec.manoeuvre,
        )
        h.servo.angles = [0.0] * 4
        h.pump(0.3)
        print()

        # === 3. Spin: angular.z survives ==================================
        # KEY 0, not arrow up, since 2026-08-24: the arrows now steer an active
        # crab and the two spins moved to 0 (CW) and 9 (CCW).
        print("=== 3. Key 0 — spin clockwise ===")
        h.pump(RELEASE_SEC + 0.5, key="0")
        check(
            "ALIGNING: angular.z non-zero at swerve_cmd_node's input "
            "(teleop_mux no longer zeroes it)",
            close(rec.cmd_vel.angular.z, -SPIN_SPEED * POSE_SCALE, 1e-4),
            f"angular.z={rec.cmd_vel.angular.z:.4f}",
        )
        h.servo.angles = list(SPIN_POSE)
        h.pump(0.5, key="0")
        check(
            "wheels arrive -> ARMED: full spin rate, NEGATIVE omega = clockwise "
            "(key 0 = CW as specified)",
            close(rec.cmd_vel.angular.z, -SPIN_SPEED, 1e-4)
            and rec.cmd_vel.angular.z < 0.0,
            f"angular.z={rec.cmd_vel.angular.z:.4f}",
        )
        print()

        # === 4. Mutual exclusion =========================================
        # STILL MUTUALLY EXCLUSIVE, but between the four MANOEUVRE keys, which
        # are now left/right/0/9. Arrow up/down left that set on 2026-08-24 and
        # are tested as crab MODIFIERS in section 4b instead — holding left and
        # up together is now a supported combination, not a conflict.
        print("=== 4. Two manoeuvre keys at once ===")
        h.teleop.press("left")
        h.teleop.press("0")
        h.pump(0.4, key="0")
        check(
            "left + 0: the later press wins, exactly one manoeuvre is active",
            "SPIN CW" in rec.manoeuvre and close(rec.cmd_vel.linear.y, 0.0),
            f"{rec.manoeuvre}, linear.y={rec.cmd_vel.linear.y:.4f}",
        )
        h.teleop.press("0")
        h.teleop.press("left")
        h.pump(0.4, key="left")
        check(
            "up + left: reverse order, crab wins",
            "CRAB LEFT" in rec.manoeuvre and close(rec.cmd_vel.angular.z, 0.0),
            f"{rec.manoeuvre}, angular.z={rec.cmd_vel.angular.z:.4f}",
        )
        h.pump(0.4, key="w")
        check(
            "W while crabbing: cancels the manoeuvre, back to cornering",
            rec.manoeuvre.startswith("CORNERING") and close(rec.cmd_vel.linear.y, 0.0),
            rec.manoeuvre,
        )
        h.pump(DRIVE_HOLD + 0.2)
        print()

        # === 5. Space bar in every mode ==================================
        print("=== 5. Space bar — unconditional emergency stop (SR-2) ===")
        h.servo.angles = [0.0] * 4
        h.pump(RELEASE_SEC + 0.3, key="left")
        h.servo.angles = list(CRAB_POSE)
        h.pump(0.4, key="left")
        armed_crab = close(rec.cmd_vel.linear.y, CRAB_SPEED, 1e-4)
        direct_before = rec.direct_steer_count
        h.teleop.center()
        h.pump(0.3)
        check(
            "space out of an armed crab: /cmd_vel zeroed on all axes",
            armed_crab
            and close(rec.cmd_vel.linear.x, 0.0)
            and close(rec.cmd_vel.linear.y, 0.0)
            and close(rec.cmd_vel.angular.z, 0.0),
            f"was linear.y={CRAB_SPEED} armed; now "
            f"({rec.cmd_vel.linear.x:.3f}, {rec.cmd_vel.linear.y:.3f}, "
            f"{rec.cmd_vel.angular.z:.3f})",
        )
        check(
            "space: /teleop/direct_steer resumes at straight ahead — this is what "
            "swings the wheels back",
            rec.direct_steer_count > direct_before
            and all(close(v, 0.0) for v in rec.direct_steer),
            f"direct_steer={rec.direct_steer}",
        )
        check(
            "space: back to keyboard mode + cornering",
            rec.manoeuvre.startswith("CORNERING"),
            rec.manoeuvre,
        )
        h.pump(0.4, key="w")
        check(
            "space out of a crab: W does NOT drive while the wheels are still at 90 deg",
            close(rec.cmd_vel.linear.x, 0.0),
            f"linear.x={rec.cmd_vel.linear.x:.3f}, {rec.manoeuvre}",
        )
        h.servo.angles = [0.0] * 4
        h.pump(0.4, key="w")
        check(
            "…and drives again as soon as the wheels are measurably straight",
            close(rec.cmd_vel.linear.x, LINEAR_VEL),
            f"linear.x={rec.cmd_vel.linear.x:.3f}",
        )
        h.pump(DRIVE_HOLD + 0.2)

        # space in plain cornering must behave exactly as before the feature
        h.pump(0.3, key="w")
        h.teleop.center()
        h.pump(0.2)
        h.pump(0.3, key="w")
        check(
            "space in plain cornering: W drives again immediately (unchanged SR-2/SR-3)",
            close(rec.cmd_vel.linear.x, LINEAR_VEL),
            f"linear.x={rec.cmd_vel.linear.x:.3f}",
        )
        print()
    finally:
        h.shutdown()

    # === 6. Escape-sequence parsing (no ROS) =============================
    # Moved to gripperx_teleop.key_input.read_sequence, which additionally
    # carries the kitty-protocol EVENT TYPE. The pre-protocol cases below are
    # kept verbatim: a terminal without the protocol still sends exactly these,
    # and they must keep decoding identically. The event-type cases live in
    # test/check_key_input.py.
    print("=== 6. Escape-sequence parsing ===")
    for payload, expected, label in (
        (b"[A", ("up", PRESS), "CSI up"),
        (b"[B", ("down", PRESS), "CSI down"),
        (b"[C", ("right", PRESS), "CSI right"),
        (b"[D", ("left", PRESS), "CSI left"),
        (b"OA", ("up", PRESS), "SS3 up (application cursor mode)"),
        (b"", None, "bare ESC — must not block on bytes that never come"),
        (b"[H", None, "Home key — not an arrow"),
        (b"[1;5A", None, "ctrl+up — modified, dropped, not read as 'a'"),
        (b"[119;1:3u", ("w", 3), "W RELEASE under the kitty protocol"),
        (b"\x1b", None, "ESC ESC"),
    ):
        read_fd, write_fd = os.pipe()
        if payload:
            os.write(write_fd, payload)
        with os.fdopen(read_fd, "rb", buffering=0) as reader:
            start = time.monotonic()
            result = read_sequence(reader, 0.05)
            elapsed = time.monotonic() - start
            leftover = b""
            import select as _select
            if _select.select([reader], [], [], 0.0)[0]:
                leftover = os.read(read_fd, 64)
        os.close(write_fd)
        ok = result == expected and not leftover and elapsed < 1.0
        print(
            f"  {label}: {payload!r} -> {result!r} "
            f"(leftover {leftover!r}, {elapsed * 1000:.0f} ms) -> "
            f"{'PASS' if ok else 'FAIL'}"
        )
        if not ok:
            failures.append(f"escape parsing {label}")
    print()

    if failures:
        print("FAILURES: " + "; ".join(failures))
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
