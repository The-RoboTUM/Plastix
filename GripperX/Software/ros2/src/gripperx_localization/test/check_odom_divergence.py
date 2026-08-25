#!/usr/bin/env python3
"""Replay recorded runs through the divergence monitor.

The point of the monitor is that it would have caught the 2026-08-21 case where
Nav2 reported SUCCEEDED with the robot 2.47 m off target and zero recoveries. So
the test is not "does the arithmetic work" but "does it fire on the run that went
wrong, and stay quiet on the run that did not".

Traces are the per-cycle recordings from that investigation: t_sim plus the wheel
odometry pose (wo_x, wo_y) and the laser odometry pose (lo_x, lo_y).
"""
import csv
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from gripperx_localization.odom_divergence_monitor import DivergenceWindow  # noqa: E402

WINDOW_SEC = 2.0
MIN_TRAVEL_M = 0.10
MAX_DIVERGENCE_M = 0.25


def replay(path):
    w = DivergenceWindow(WINDOW_SEC, MIN_TRAVEL_M, MAX_DIVERGENCE_M)
    first_alarm_t = None
    alarm_cycles = 0
    worst = 0.0
    with open(path) as handle:
        for row in csv.DictReader(handle):
            try:
                t = float(row["t_sim"])
                w.push_a(t, float(row["wo_x"]), float(row["wo_y"]))
                w.push_b(t, float(row["lo_x"]), float(row["lo_y"]))
            except (ValueError, KeyError):
                continue
            state, _ta, _tb, div = w.evaluate()
            worst = max(worst, div)
            if state == DivergenceWindow.DIVERGED:
                alarm_cycles += 1
                if first_alarm_t is None:
                    first_alarm_t = t
    return first_alarm_t, alarm_cycles, worst


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else None
    if not base:
        print("usage: check_odom_divergence.py <directory with run*_trace.csv>")
        print("(traces are investigation artefacts, not committed to the repo)")
        return 0

    cases = [
        ("run3", "healthy run, laser odometry fused, no lock-up", False),
        ("run1", "the failure: scan matcher locked up, Nav2 said SUCCEEDED", True),
    ]
    failures = []
    for run, label, expect_alarm in cases:
        path = os.path.join(base, f"{run}_trace.csv")
        if not os.path.exists(path):
            print(f"  {run}: trace not present, skipped")
            continue
        first, cycles, worst = replay(path)
        fired = cycles > 0
        print(f"\n--- {run}: {label} ---")
        print(f"    worst divergence in a {WINDOW_SEC:.0f} s window: {worst:.4f} m "
              f"(threshold {MAX_DIVERGENCE_M:.2f})")
        if fired:
            print(f"    ALARM from t={first:.1f}s, sustained over {cycles} cycles")
        else:
            print("    no alarm")
        if fired != expect_alarm:
            failures.append(
                f"{run}: expected {'an alarm' if expect_alarm else 'silence'}, got the opposite")

    print()
    if failures:
        for f in failures:
            print("FAILURE:", f)
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
