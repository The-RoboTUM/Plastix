#!/usr/bin/env python3
"""Fail if the robot's EKF gates either of its yaw-rate sources.

Since 2.4 (IMU yaw-rate fusion, LIDAR/IMU issue draft 2026-10-22) the robot's EKF
has two yaw-rate sources: imu0 (gyro, primary) and odom0 (encoders, fallback --
user decision D1 = (a), 2026-10-29). Neither may carry a Mahalanobis gate:

- odom0: it is the only yaw-rate source whenever the IMU is silent, and the only
  vx/vy source without laser fusion. A gate rejects sawtooth wheel-rate samples and
  replaces them with nothing, so the filter integrates a stale rate (2026-10-07
  coast bags: +46.8 deg drift with the gate, -0.3 deg without).
- imu0: measured gyro changes between consecutive samples reach 0.59 rad/s in runs
  of 3-5 samples at every spin start/stop (2026-10-22 drive15 bag); the former
  threshold 1.5 would reject exactly those runs and hand vyaw to the encoder until
  the gate has grown (rationale in localization.yaml above imu0).

Reads the ACTIVE configuration, not file text: evaluates localization.launch.py
with the arguments gripperx-mapping.sh passes on the robot, for every EKF
definition reachable from there, and every combination real_autonomy.launch.py can
forward, and checks the merged parameters each EKF actually receives (the launch
evaluation of check_imu_wiring.py). A gate re-added by a launch override fails here
as well as one re-added in localization.yaml. Without launch/launch_ros it falls
back to localization.yaml alone and says so.

Usage: check_odom0_no_gate.py [path/to/localization.yaml]   (path: fallback only)
"""

import importlib.util
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve()
_DEFAULT = HERE.parent.parent / "config" / "localization.yaml"
YAW_RATE_INDEX = 11
FORBIDDEN = {
    "odom0": ("odom0_twist_rejection_threshold", "odom0_pose_rejection_threshold"),
    "imu0": ("imu0_twist_rejection_threshold",),
}


def _module(name):
    spec = importlib.util.spec_from_file_location(f"_{name}", HERE.parent / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def problems(params: dict) -> list:
    found = []
    for source, keys in FORBIDDEN.items():
        config = [bool(v) for v in params.get(f"{source}_config", [])]
        if len(config) != 15 or not config[YAW_RATE_INDEX]:
            found.append(f"{source} does not fuse the yaw rate (index {YAW_RATE_INDEX}); "
                         "this check's premise (two ungated yaw-rate sources) is stale")
        found += [f"{key} = {params[key]!r}" for key in keys if key in params]
    return found


def main(argv):
    failures = []
    try:
        import launch_ros  # noqa: F401
        have_launch = True
    except ImportError:
        have_launch = False

    if have_launch:
        imu = _module("check_imu_wiring")
        wiring = _module("check_autonomy_launch_wiring")
        _, _, robot_args = wiring.parse_mapping_script()
        cases = []
        for fuse in ("true", "false"):
            for gps in ("true", "false"):
                combo = {"fuse_laser_odometry": fuse, "enable_gps": gps}
                cases.append((f"gripperx-mapping.sh args + {combo}", {**robot_args, **combo}))
        if imu.REAL_AUTONOMY.is_file():
            for laser in ("true", "false"):
                for fuse in ("true", "false"):
                    for gps in ("true", "false"):
                        overrides = {"enable_laser_odometry": laser,
                                     "fuse_laser_odometry": fuse, "enable_gps": gps}
                        active, args = imu.localization_include_args(
                            imu.REAL_AUTONOMY, overrides)
                        if active:
                            cases.append((f"real_autonomy {overrides}", args))
        definitions = set()
        for where, args in cases:
            ekf = imu.effective_parameters(args, wanted=("ekf_filter_node",)).get(
                "ekf_filter_node")
            if ekf is None:
                failures.append(f"{where}: no ekf_filter_node started")
                continue
            definitions.add(ekf["__entity__"])
            failures += [f"{where}: {p}" for p in problems(ekf)]
        if len(definitions) != 4:
            failures.append(f"reached {len(definitions)} of the four EKF definitions")
        where = f"{len(cases)} robot launch configurations"
    else:
        path = Path(argv[1]) if len(argv) > 1 else _DEFAULT
        print("SKIP launch evaluation (launch/launch_ros not importable): checking "
              f"{path} alone. Source /opt/ros/jazzy for the full check.")
        params = yaml.safe_load(path.read_text())["ekf_filter_node"]["ros__parameters"]
        failures += [f"{path}: {p}" for p in problems(params)]
        where = str(path)

    if failures:
        print("FAIL: the robot EKF gates a yaw-rate source, or its premise changed:")
        for failure in failures:
            print(f"  {failure}")
        return 1
    print(f"OK {where}: odom0 and imu0 both fuse the yaw rate, neither is gated")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
