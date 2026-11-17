#!/usr/bin/env python3
"""Does a wheel-position reset still produce a /wheel/odom yaw-rate spike?

Reset shapes, from the code that produces them (firmware main.cpp /
motor_controller.cpp, GripperXInterface on_init/read). localization_input_node
runs in gripperx-mapping.service and survives a bringup restart.
REACHABLE on the current stack:
  - Controller / bringup restart: GripperXInterface publishes 0.0 until its first
    /hw/joint_states, then the ESP32's absolute count reappears at once.
  - controller_manager stall longer than state_timeout_sec without a read()
    ERROR: a /joint_states gap, the count continuing.
NOT REACHABLE as a resumed stream (kept as defensive cases): an ESP32 reboot.
It takes > 2 s, so read() returns ERROR and ros2_control deactivates the hardware
component and every controller until a full restart (SR-13, no auto-recovery).
A micro-ROS reconnect without reboot keeps the count: no reset at all.

reset_gap_sec must equal state_timeout_sec in the live gripperx_description
xacro; this check reads that file and fails on drift.

The real LocalizationInputNode is used with the package's own config file. The
motion is a synthetic spin at OMEGA, so every honest rate is exactly OMEGA.
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
import yaml  # noqa: E402

from gripperx_control.swerve_kinematic_model import BodyTwist  # noqa: E402
from gripperx_localization.localization_input_node import LocalizationInputNode  # noqa: E402

CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config", "localization.yaml"
)
T = 1.0 / 30.0
OMEGA = 0.8
XACRO = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "gripperx_description", "urdf",
    "gripperx_v1.ros2_control.xacro",
)
COUNTER_AT = 150.0  # rad of accumulated wheel travel before the reset
FAILURES = []


def check(label, ok, detail):
    print(f"    {'ok  ' if ok else 'FAIL'} {label}: {detail}")
    if not ok:
        FAILURES.append(label)


class Run:
    def __init__(self, omega=OMEGA):
        self.node = LocalizationInputNode()
        self.out = []
        self.node.publish_wheel_odometry = (
            lambda stamp, vx, vy, wz: self.out.append((self.t, wz, self.node.pose_yaw))
        )
        commands = self.node.kinematic_model.inverse_kinematics(BodyTwist(0.0, 0.0, omega))
        self.angles = [c.steering_angle for c in commands]
        self.rates = [
            c.linear_speed / self.node.effective_wheel_radius / m
            for c, m in zip(commands, self.node.drive_joint_multipliers)
        ]
        self.t = 100.0

    def send(self, positions, moving=True):
        msg = JointState()
        msg.name = list(self.node.drive_joint_names) + list(self.node.steer_joint_names)
        msg.position = list(positions) + list(self.angles)
        msg.velocity = (list(self.rates) if moving else [0.0] * 4) + [0.0] * 4
        msg.header.stamp = Time(sec=int(self.t), nanosec=int(round((self.t - int(self.t)) * 1e9)))
        self.node.joint_state_callback(msg)

    def spin(self, reads, start):
        """`reads` reads of the spin; wheel i at start[i] + rate_i * T * k."""
        for k in range(reads):
            self.t += T
            self.send([s + r * T * k for s, r in zip(start, self.rates)])

    def zeros(self, reads):
        for _ in range(reads):
            self.t += T
            self.send([0.0] * 4, moving=False)

    def gap(self, seconds):
        self.t += seconds

    def finish(self, label):
        wz = [w for _, w, _ in self.out]
        if not any(abs(w) > 1e-12 for w in wz):
            check(f"{label}: moving samples published", False,
                  "no moving /wheel/odom sample at all (everything re-based?)")
            self.node.destroy_node()
            return
        worst = max(abs(w - OMEGA) for w in wz if abs(w) > 1e-12)
        spikes = [w for w in wz if abs(w) > 1.05 * OMEGA]
        check(f"{label}: no yaw-rate spike", not spikes,
              f"max |vyaw| {max(abs(w) for w in wz):.2f} rad/s")
        check(f"{label}: every moving rate correct", worst < 1e-6,
              f"worst |error| {worst:.2e} rad/s")
        # Pose continuity: with every published rate exactly OMEGA, the yaw can only
        # have moved by OMEGA per published read; a jump at the reset breaks this.
        moving = sum(1 for w in wz if abs(w) > 1e-12)
        expected = OMEGA * T * moving
        delta = self.node.pose_yaw - expected
        err = math.atan2(math.sin(delta), math.cos(delta))
        check(f"{label}: pose continuous", abs(err) < 1e-6,
              f"yaw error {err:+.2e} rad over {moving} moving samples")
        self.node.destroy_node()


def main():
    print("\n--- params file carries the keys ---")
    with open(CONFIG) as handle:
        params = yaml.safe_load(handle)["localization_input_node"]["ros__parameters"]
    for key in ("reset_detection", "reset_gap_sec"):
        check(f"{key} in localization.yaml", key in params, repr(params.get(key)))
    try:
        with open(XACRO) as handle:
            match = re.search(r"state_timeout_sec:=([0-9.eE+-]+)", handle.read())
    except OSError as error:
        match = None
        print(f"    cannot read {XACRO}: {error}")
    timeout = float(match.group(1)) if match else None
    check("reset_gap_sec == xacro state_timeout_sec",
          timeout is not None and params.get("reset_gap_sec") == timeout,
          f"reset_gap_sec {params.get('reset_gap_sec')!r}, state_timeout_sec {timeout!r}")

    rclpy.init(args=["--ros-args", "--params-file", CONFIG])
    far = [COUNTER_AT * (1 if r > 0 else -1) for r in Run().rates]

    print("\n--- DEFENSIVE (not reachable now): ESP32 reboot, gap, 0.0 frames, count from 0 ---")
    run = Run()
    run.spin(30, far)
    run.gap(3.0)
    run.zeros(5)
    run.spin(30, [0.0] * 4)
    run.finish("defensive reboot")

    print("\n--- DEFENSIVE (not reachable now): ESP32 reboot, gap, count near 0, no 0.0 frame ---")
    run = Run()
    run.spin(30, far)
    run.gap(3.0)
    run.spin(30, [0.05 * r for r in run.rates])
    run.finish("defensive reboot, no zero frame")

    print("\n--- controller restart at startup: 0.0 until feedback, then the count ---")
    run = Run()
    run.zeros(10)
    run.spin(30, far)
    run.finish("startup")

    print("\n--- bringup restart mid-run: gap, 0.0 frames, same count continues ---")
    run = Run()
    run.spin(30, far)
    run.gap(5.0)
    run.zeros(3)
    run.spin(30, far)
    run.finish("bringup restart")

    print("\n--- controller_manager stall > state_timeout_sec, no ERROR: gap, count continues ---")
    run = Run()
    run.spin(30, far)
    run.gap(3.0)
    run.spin(30, [f + r * (T * 30 + 3.0) for f, r in zip(far, run.rates)])
    run.finish("controller_manager stall")

    print("\n--- genuine fast motion is NOT a reset ---")
    # Fastest wheel at the command cap, gripperx_control ros2_controllers.yaml
    # max_wheel_angular_speed (12.0 rad/s).
    probe = Run()
    fast_omega = OMEGA * 12.0 / max(abs(r) for r in probe.rates)
    probe.node.destroy_node()
    fast = Run(omega=fast_omega)
    # Wheel 0 passes EXACTLY through 0.0 at read 15; the others cross zero a few
    # counts apart. (All four at exactly 0.0 in the same read is read as a reset and
    # costs two reads of motion, no spike -- see localization.yaml.)
    start = [-r * T * 15 + 0.004 * i for i, r in enumerate(fast.rates)]
    fast.spin(30, start)
    fast.gap(0.5)  # a dropout shorter than reset_gap_sec: the average rate is correct
    fast.spin(30, [s + r * (T * 30 + 0.5) for s, r in zip(start, fast.rates)])
    wz = [w for _, w, _ in fast.out]
    check("all reads published", len(wz) == 59, f"{len(wz)} of 59")
    worst = max((abs(w - fast_omega) for w in wz), default=math.inf)
    check("rates correct at the command cap", worst < 1e-6,
          f"{fast_omega:.2f} rad/s, worst |error| {worst:.2e}")
    fast.node.destroy_node()

    rclpy.shutdown()
    print()
    if FAILURES:
        for failure in FAILURES:
            print("FAILURE:", failure)
        sys.exit(1)
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
