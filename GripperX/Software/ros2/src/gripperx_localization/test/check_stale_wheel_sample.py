#!/usr/bin/env python3
"""Does a stale /joint_states read still corrupt the /wheel/odom yaw rate?

The case: 2026-10-07 21:02 bag. /joint_states is stamped with the controller's read
time, and the ESP32 feedback arrives at the same ~30 Hz on an unrelated clock. With
the two phases aligned, read() copied the previous feedback message again on 21.7 %
of moving samples (all four wheel positions bit-identical) and repaid each repeat
with one double step later. Old result while spinning at ~0.8 rad/s: a 0 / 1.7 /
0.8 rad/s sawtooth with 21 % zeros, and the pose integral still exact.

READ_STEPS below is NOT invented: it is the per-read feedback-index step sequence of
a 255-read moving stretch of that bag (/joint_states samples matched bit-for-bit to
/hw/joint_states messages; 0 = repeat, 2 = double step), reads 2-151. Read 1 is
left out because it is the double step repaying a repeat that lies before the
stretch; starting there would test an unbalanced window, not the mechanism. The wheel
motion around it is a synthetic spin in place, so the ground truth is known exactly.

The real LocalizationInputNode is used with the package's own config file, so a
retuned parameter in localization.yaml is what gets tested, not a copy of it.
"""
import math
import os
import statistics
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
READ_PERIOD = 1.0 / 30.0
OMEGA = 0.8
READ_STEPS = (
    "110120111210111111210201120112110120201120111111120211011120111111120210120111"
    "120120111201210111111111120120112110201120210112110120210210120210111120"
)
FAILURES = []


def check(label, ok, detail):
    print(f"    {'ok  ' if ok else 'FAIL'} {label}: {detail}")
    if not ok:
        FAILURES.append(label)


def make_node():
    node = LocalizationInputNode()
    node.published = []
    node.publish_wheel_odometry = (
        lambda stamp, vx, vy, wz: node.published.append(wz)
    )
    return node


def spin_wheels(node):
    """Steering angles and raw wheel rates (rad/s) of a pure spin at OMEGA."""
    commands = node.kinematic_model.inverse_kinematics(BodyTwist(0.0, 0.0, OMEGA))
    angles = [c.steering_angle for c in commands]
    rates = [
        c.linear_speed / node.effective_wheel_radius / m
        for c, m in zip(commands, node.drive_joint_multipliers)
    ]
    return angles, rates


def feed(node, steps, angles, rates, positions_frozen=False):
    """One /joint_states per read; position = feedback index * one feedback step."""
    feedback_index = 0
    for read, step in enumerate([1] + list(steps)):
        feedback_index += step
        msg = JointState()
        msg.name = list(node.drive_joint_names) + list(node.steer_joint_names)
        position_index = 0 if positions_frozen else feedback_index
        msg.position = [r * READ_PERIOD * position_index for r in rates] + list(angles)
        msg.velocity = list(rates) + [0.0] * 4
        t = 100.0 + read * READ_PERIOD
        msg.header.stamp = Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
        node.joint_state_callback(msg)
    return feedback_index


def main():
    print("\n--- params file carries the keys (a typo would fall back to the node default) ---")
    with open(CONFIG) as handle:
        params = yaml.safe_load(handle)["localization_input_node"]["ros__parameters"]
    for key in ("stale_sample_check", "stale_sample_min_wheel_rate",
                "stale_sample_step_tolerance"):
        check(f"{key} in localization.yaml", key in params, repr(params.get(key)))

    rclpy.init(args=["--ros-args", "--params-file", CONFIG])
    steps = [int(c) for c in READ_STEPS]

    print("\n--- run-3 repeat pattern, spin at %.2f rad/s ---" % OMEGA)
    node = make_node()
    angles, rates = spin_wheels(node)
    last_index = feed(node, steps, angles, rates)
    wz = node.published
    zeros = sum(1 for w in wz if abs(w) < 1e-9)
    print("    %d reads, %d repeats, %d double steps, %d odom samples"
          % (len(steps), steps.count(0), steps.count(2), len(wz)))
    check("no zero-rate dropouts", zeros == 0, f"{zeros} zero samples")
    worst = max(abs(w - OMEGA) for w in wz)
    check("every rate within 5 % of truth", worst < 0.05 * OMEGA,
          f"worst |error| {worst:.3f} rad/s, scatter {statistics.pstdev(wz):.3f} rad/s")
    # The first message only seeds the reference; it sits at feedback index 1.
    truth_yaw = OMEGA * READ_PERIOD * (last_index - 1)
    delta = node.pose_yaw - truth_yaw
    yaw_error = math.atan2(math.sin(delta), math.cos(delta))
    check("pose integral exact", abs(yaw_error) < 1e-9,
          f"pose_yaw {node.pose_yaw:.9f} vs truth {truth_yaw:.9f}")
    node.destroy_node()

    print("\n--- clean stream: the compensation must change nothing ---")
    node = make_node()
    feed(node, [1] * 60, angles, rates)
    worst = max(abs(w - OMEGA) for w in node.published)
    check("one sample per read", len(node.published) == 60, f"{len(node.published)} samples")
    check("rate equals truth", worst < 1e-6, f"worst |error| {worst:.2e} rad/s")
    node.destroy_node()

    print("\n--- frozen encoders, velocity = command echo: must stay zero, not silent ---")
    node = make_node()
    feed(node, [1] * 60, angles, rates, positions_frozen=True)
    moving = sum(1 for w in node.published if abs(w) > 1e-12)
    check("no fabricated motion", moving == 0, f"{moving} non-zero samples")
    check("still published (at least every 2nd read)", len(node.published) >= 30,
          f"{len(node.published)} samples for 60 reads")
    node.destroy_node()

    rclpy.shutdown()
    print()
    if FAILURES:
        for failure in FAILURES:
            print("FAILURE:", failure)
        sys.exit(1)
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
