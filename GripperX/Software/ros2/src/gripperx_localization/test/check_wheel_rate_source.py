#!/usr/bin/env python3
"""Does wheel_rate_source=position actually make a command echo unreachable?

No ROS, no Gazebo: the extraction method is exercised directly against synthetic
JointState frames. The decisive case is the last one — a wheel whose encoder is
unplugged, so /joint_states velocity carries the COMMAND while the position does
not move. That is the state BL/BR were found in on 2026-08-21.
"""
import os
import sys
import math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from sensor_msgs.msg import JointState
from gripperx_localization.localization_input_node import LocalizationInputNode

NAMES = ["f_leftwheel", "b_leftwheel", "b_rightwheel", "f_rightwheel"]

class Stub:
    """Only what _extract_wheel_angular_rates touches."""
    def __init__(self, source):
        self.wheel_rate_source = source
        self.drive_joint_names = NAMES
        self.drive_joint_multipliers = [1.0, 1.0, 1.0, 1.0]
        self.previous_drive_positions = {n: 0.0 for n in NAMES}
        self._logged = []
    def get_logger(self):
        outer = self
        class L:
            def error(self, msg, **kw): outer._logged.append(msg)
        return L()

extract = LocalizationInputNode._extract_wheel_angular_rates

def frame(pos, vel):
    m = JointState(); m.name = list(NAMES)
    m.position = list(pos); m.velocity = list(vel)
    return m

def run(label, pos, vel, dt, expect_truth, expect_ok):
    idx = {n: i for i, n in enumerate(NAMES)}
    print(f"\n--- {label} ---")
    print(f"    ground truth: {expect_truth:+.3f} rad/s")
    for src in ("velocity_if_valid", "velocity", "position"):
        st = Stub(src)
        rates = extract(st, m := frame(pos, vel), idx, {n: pos[i] for i, n in enumerate(NAMES)}, dt)
        err = rates[0] - expect_truth
        ok = abs(err) < 1e-6
        expected_ok = expect_ok[src]
        print(f"    {src:18s} -> {rates[0]:+.3f} rad/s   "
              f"{'matches ground truth' if ok else f'WRONG by {err:+.3f}'}")
        if ok != expected_ok:
            FAILURES.append(f"{label} / {src}: expected "
                            f"{'agreement' if expected_ok else 'disagreement'}, got the opposite")

FAILURES = []
DT = 1.0 / 30.0
# 1) healthy wheel: encoder turning, velocity and position agree
run("healthy wheel, encoder turning", [2.0 * DT] * 4, [2.0] * 4, DT, 2.0,
    {"velocity_if_valid": True, "velocity": True, "position": True})
# 2) unplugged encoder: position frozen, velocity carries the COMMAND echo
# THE CASE THIS PARAMETER EXISTS FOR. Encoder unplugged: firmware leaves the wheel
# position untouched and getRPM() hands the COMMAND back on the velocity index. The
# robot was found in exactly this state on BL and BR on 2026-08-21.
run("encoder unplugged, velocity carries the command echo",
    [0.0] * 4, [2.35] * 4, DT, 0.0,
    {"velocity_if_valid": False, "velocity": False, "position": True})

print()
if FAILURES:
    for f in FAILURES:
        print("FAILURE:", f)
    sys.exit(1)
print("ALL CHECKS PASSED")
