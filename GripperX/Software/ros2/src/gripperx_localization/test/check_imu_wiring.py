#!/usr/bin/env python3
"""Fail if the IMU wiring or the robot's imu0 fusion drift from what was decided.

Reads the ACTIVE configuration, not a copy of it: Part 1 evaluates
localization.launch.py from the source tree with the exact arguments
gripperx-mapping.sh passes on the robot, resolves every parameter file and
override the EKF and localization_input_node actually receive, in launch order,
and checks the merged result. The cross-file parts compare two configs with each
other, never a config with a constant written here.

Part 1  robot EKF: imu0 fuses the yaw rate (index 11, step 2.4) and nothing else,
        never absolute yaw (index 5, user decision D2, 2026-10-22); the encoder
        yaw rate stays fused as fallback (D1 = (a), 2026-10-29) with a variance of
        no less than 0.08 (rad/s)^2, and the gyro's variance is the tighter one
Part 2  robot input: localization_input_node reads the topic the BNO085 driver
        publishes, stamps the frame the driver stamps, that frame is a URDF link,
        and imu0 listens on what localization_input_node publishes
Part 3  twin: the Gazebo IMU reaches ROS on the same raw topic, from a sensor on the
        same frame; for every argument combination sim_navigation.launch.py can
        forward, the twin EKF's imu0 set and the twin's yaw-rate variances equal
        the robot's (2.4: one filter structure on both platforms)
Part 4  every EKF variant: with the robot's arguments (gripperx-mapping.sh, and every
        combination real_autonomy.launch.py can forward) imu0 obeys Part 1

The launch evaluation needs `launch`/`launch_ros` (sourcing /opt/ros/jazzy is
enough). Without them Part 1 falls back to localization.yaml alone and says so.

Usage: check_imu_wiring.py
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve()
SRC = HERE.parents[2]                                  # .../Software/ros2/src
LOC = SRC / "gripperx_localization"
LOCALIZATION_YAML = LOC / "config" / "localization.yaml"
LAUNCH = LOC / "launch" / "localization.launch.py"
BNO085_YAML = SRC / "gripperx_sensors" / "config" / "bno085.yaml"
CORE_XACRO = SRC / "gripperx_description" / "urdf" / "gripperx_v1.core.xacro"
GAZEBO_XACRO = SRC / "gripperx_description" / "urdf" / "gripperx_v1.gazebo.xacro"
GZ_BRIDGE = SRC / "gripperx_gazebo" / "config" / "gz_bridge.yaml"
SIM_NAV = SRC / "gripperx_gazebo" / "launch" / "sim_navigation.launch.py"
REAL_AUTONOMY = SRC / "gripperx_bringup" / "launch" / "real_autonomy.launch.py"

YAW_INDEX = 5        # robot_localization state order: x y z roll pitch YAW ...
YAW_RATE_INDEX = 11  # ... vx vy vz vroll vpitch VYAW ax ay az
# User decision D1 = (a), 2026-10-29: the encoder yaw rate stays fused "with a
# large variance", at NO LESS THAN this (YAW_AGREEMENT_REPORT 5.1 variant A). A
# decided floor, not a copy of the configured value.
ENCODER_YAW_RATE_VARIANCE_FLOOR = 0.08
BOOLS = ("true", "false")

FAILURES: list[str] = []


def check(condition: bool, label: str, detail: str = "") -> None:
    print(f"  {'ok  ' if condition else 'FAIL'}  {label}")
    if not condition:
        for line in detail.splitlines():
            print(f"          {line}")
        FAILURES.append(label)


def _wiring_module():
    spec = importlib.util.spec_from_file_location(
        "_autonomy_wiring", HERE.parent / "check_autonomy_launch_wiring.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def effective_parameters(launch_args: dict[str, str],
                         wanted=("ekf_filter_node", "localization_input_node")) -> dict[str, dict]:
    """{node_name: merged parameters} for the wanted nodes the launch starts with these args."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import normalize_to_list_of_substitutions as norm
    from launch.utilities import perform_substitutions
    import launch_ros.actions as lra
    from launch_ros.utilities import evaluate_parameters

    spec = importlib.util.spec_from_file_location("_imu_launch_under_test", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Source tree, not the ament index: an installed copy may predate the change.
    module.get_package_share_directory = lambda name: str(SRC / name)
    ld = module.generate_launch_description()

    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            ctx.launch_configurations[entity.name] = perform_substitutions(
                ctx, entity.default_value)
    unknown = sorted(set(launch_args) - set(ctx.launch_configurations))
    if unknown:
        raise AssertionError(f"arguments not declared by {LAUNCH.name}: {unknown}")
    ctx.launch_configurations.update(launch_args)

    nodes: dict[str, dict] = {}
    for position, entity in enumerate(ld.entities):
        if not isinstance(entity, (lra.Node, lra.LifecycleNode)):
            continue
        if entity.condition is not None and not entity.condition.evaluate(ctx):
            continue
        name = perform_substitutions(ctx, norm(entity._Node__node_name))
        if name not in wanted:
            continue
        # Position in the launch description: stable across evaluations, unlike
        # id(), which differs per call and may be reused after garbage collection.
        merged: dict = {"__entity__": position}
        for item in evaluate_parameters(ctx, entity._Node__parameters or []):
            if isinstance(item, dict):
                merged.update(item)
                continue
            data = yaml.safe_load(Path(item).read_text()) or {}
            for key in ("/**", name, f"/{name}"):
                merged.update((data.get(key) or {}).get("ros__parameters", {}))
        nodes[name] = merged
    return nodes


def fused(config) -> list[int]:
    return [i for i, v in enumerate(config) if bool(v)]


def localization_include_args(parent: Path, overrides: dict[str, str]):
    """(condition, {arg: value}) the parent passes to localization.launch.py.

    Evaluated from the parent launch file itself with its declared defaults plus
    `overrides`, so a renamed, dropped or added forwarded argument is seen.
    """
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions as norm
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location(f"_parent_{parent.stem}", parent)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda name: str(SRC / name)
    ld = module.generate_launch_description()
    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            ctx.launch_configurations[entity.name] = perform_substitutions(
                ctx, entity.default_value)
    unknown = sorted(set(overrides) - set(ctx.launch_configurations))
    if unknown:
        raise AssertionError(f"arguments not declared by {parent.name}: {unknown}")
    ctx.launch_configurations.update(overrides)
    found = []
    for entity in ld.entities:
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        location = perform_substitutions(
            ctx, norm(entity.launch_description_source._LaunchDescriptionSource__location))
        if Path(location).name != LAUNCH.name:
            continue
        args = {perform_substitutions(ctx, norm(k)): perform_substitutions(ctx, norm(v))
                for k, v in entity.launch_arguments}
        condition = entity.condition is None or entity.condition.evaluate(ctx)
        found.append((condition, args))
    if len(found) != 1:
        raise AssertionError(f"{parent.name}: expected one include of {LAUNCH.name}, "
                             f"found {len(found)}")
    return found[0]


def check_imu0(config, where: str) -> None:
    config = [bool(v) for v in config]
    check(len(config) == 15, f"{where}: imu0_config has 15 entries", f"got {config}")
    check(len(config) == 15 and not config[YAW_INDEX],
          f"{where}: imu0 does NOT fuse absolute yaw (index {YAW_INDEX})",
          "User decision D2 (2026-10-22): the IMU contributes its yaw RATE only. "
          "Absolute BNO085 yaw (Game Rotation Vector) is out of scope; fusing it on "
          "the robot is exactly what the IMU wiring (2.2) must not switch on.")
    others = [i for i, v in enumerate(config) if v and i != YAW_RATE_INDEX]
    check(not others, f"{where}: imu0 fuses nothing but the yaw rate (index {YAW_RATE_INDEX})",
          f"also true: indices {others}")
    check(len(config) == 15 and config[YAW_RATE_INDEX],
          f"{where}: imu0 fuses the yaw rate (index {YAW_RATE_INDEX})",
          "Step 2.4 (<<FR+4>>): the gyro yaw rate is the EKF's primary yaw-rate source.")


def check_yaw_rate_sources(ekf: dict, lin: dict, where: str) -> None:
    """D1 = (a): encoder yaw rate fused as fallback, loosely; the gyro tighter."""
    odom0 = [bool(v) for v in ekf.get("odom0_config", [])]
    check(len(odom0) == 15 and odom0[YAW_RATE_INDEX],
          f"{where}: odom0 still fuses the encoder yaw rate (fallback, D1 = (a))",
          "robot_localization has no automatic sensor fallback: with imu0 the only "
          "yaw-rate source, a silent IMU leaves the filter integrating its last rate.")
    enc = lin.get("wheel_odom_yaw_rate_variance")
    gyro = lin.get("imu_yaw_rate_variance")
    check(isinstance(enc, (int, float)) and enc >= ENCODER_YAW_RATE_VARIANCE_FLOOR,
          f"{where}: encoder yaw-rate variance >= {ENCODER_YAW_RATE_VARIANCE_FLOOR} (D1 = (a))",
          f"wheel_odom_yaw_rate_variance={enc!r}. A tighter value lets the encoder's "
          "systematic error (+6.4...+9.0 % in spin) pull the fused yaw.")
    check(isinstance(gyro, (int, float)) and isinstance(enc, (int, float))
          and 0.0 < gyro < enc,
          f"{where}: the gyro yaw-rate variance is set, and tighter than the encoder's",
          f"imu_yaw_rate_variance={gyro!r}, wheel_odom_yaw_rate_variance={enc!r}. "
          "0 keeps the BNO085 datasheet value (2.9e-3), which leaves the encoder ~2 % "
          "of the weight -- about 1.8 deg on 3 turns, at the D3 limit (report 5.1).")


def xacro_imu_sensor() -> tuple[str | None, str | None]:
    """(reference frame, <topic>) of the Gazebo sensor of type imu."""
    text = GAZEBO_XACRO.read_text()
    m = re.search(r'<gazebo\s+reference="([^"]+)">\s*<sensor[^>]*type="imu"[^>]*>(.*?)</sensor>',
                  text, re.S)
    if not m:
        return None, None
    topic = re.search(r"<topic>\s*([^<\s]+)\s*</topic>", m.group(2))
    return m.group(1), topic.group(1) if topic else None


def main() -> int:
    bno = yaml.safe_load(BNO085_YAML.read_text())["bno085_imu"]["ros__parameters"]
    try:
        import launch_ros  # noqa: F401
        have_launch = True
    except ImportError:
        have_launch = False
    wiring = _wiring_module()

    print("Part 1 - robot EKF imu0 (the launch gripperx-mapping.sh runs)")
    if have_launch:
        _, _, robot_args = wiring.parse_mapping_script()
        print(f"  ..    robot launch arguments: {robot_args}")
        robot = effective_parameters(robot_args)
        ekf = robot.get("ekf_filter_node")
        lin = robot.get("localization_input_node")
        check(ekf is not None and lin is not None,
              "the robot launch starts ekf_filter_node and localization_input_node",
              f"started: {sorted(robot)}")
        if ekf is None or lin is None:
            return 1
        check_imu0(ekf.get("imu0_config", []), "robot")
        check_yaw_rate_sources(ekf, lin, "robot")
    else:
        print("  SKIP  launch/launch_ros not importable: checking localization.yaml "
              "alone, without the launch overrides. Source /opt/ros/jazzy for the full check.")
        params = yaml.safe_load(LOCALIZATION_YAML.read_text())
        ekf = params["ekf_filter_node"]["ros__parameters"]
        lin = params["localization_input_node"]["ros__parameters"]
        check_imu0(ekf.get("imu0_config", []), "localization.yaml")
        check_yaw_rate_sources(ekf, lin, "localization.yaml")

    print("Part 2 - robot IMU input, frame and EKF topic")
    check(lin.get("imu_input_topic") == bno["imu_topic"],
          "localization_input_node reads the topic the BNO085 driver publishes",
          f"imu_input_topic={lin.get('imu_input_topic')!r}, bno085.yaml "
          f"imu_topic={bno['imu_topic']!r}")
    check(lin.get("imu_frame_id") == bno["frame_id"],
          "localization_input_node stamps the frame the BNO085 driver stamps",
          f"imu_frame_id={lin.get('imu_frame_id')!r}, bno085.yaml frame_id={bno['frame_id']!r}")
    check(re.search(rf'<link\s+name="{re.escape(str(lin.get("imu_frame_id")))}"',
                    CORE_XACRO.read_text()) is not None,
          "that frame is a link in the URDF (gripperx_v1.core.xacro)",
          "Without it robot_localization cannot transform the IMU into base_footprint "
          "and drops every sample once imu0 fuses anything.")
    check(ekf.get("imu0") == lin.get("imu_output_topic"),
          "EKF imu0 listens on localization_input_node's output topic",
          f"imu0={ekf.get('imu0')!r}, imu_output_topic={lin.get('imu_output_topic')!r}")

    print("Part 3 - twin")
    bridge = yaml.safe_load(GZ_BRIDGE.read_text())
    imu_entries = [e for e in bridge if e.get("ros_type_name") == "sensor_msgs/msg/Imu"]
    check(len(imu_entries) == 1, "gz_bridge.yaml bridges exactly one IMU", f"{imu_entries}")
    if imu_entries:
        entry = imu_entries[0]
        ros_topic = entry.get("ros_topic_name", entry.get("topic_name"))
        gz_topic = entry.get("gz_topic_name", entry.get("topic_name"))
        check(ros_topic == lin.get("imu_input_topic"),
              "the twin's IMU reaches ROS on the topic localization_input_node reads",
              f"bridge ROS side {ros_topic!r}, imu_input_topic {lin.get('imu_input_topic')!r}. "
              "Different raw topics on twin and robot make the 2.5 A/B replay "
              "compare two configurations instead of one.")
        frame, sensor_topic = xacro_imu_sensor()
        check(sensor_topic == gz_topic,
              "the bridge's Gazebo side is the topic the Gazebo IMU sensor publishes",
              f"gazebo.xacro <topic> {sensor_topic!r}, bridge gz side {gz_topic!r}")
        check(frame == lin.get("imu_frame_id"),
              "the Gazebo IMU sensor sits on the frame localization_input_node stamps",
              f"gazebo reference {frame!r}, imu_frame_id {lin.get('imu_frame_id')!r}")
    if have_launch:
        # Every combination sim_navigation.launch.py can forward. It fixes
        # enable_gps:=false itself; that is part of what is evaluated. Compared
        # with the robot's EFFECTIVE values (Part 1), not with a constant: the 2.5
        # replay and any twin result only speak for the robot while both run one
        # filter structure (decided in 2.4; before it the twin alone fused IMU yaw).
        robot_imu0 = fused(robot["ekf_filter_node"].get("imu0_config", []))
        robot_lin = robot["localization_input_node"]
        variance_keys = ("imu_yaw_rate_variance", "wheel_odom_yaw_rate_variance")
        for laser in BOOLS:
            for fuse in BOOLS:
                overrides = {"odom_source": "ekf", "enable_laser_odometry": laser,
                             "fuse_laser_odometry": fuse}
                active, args = localization_include_args(SIM_NAV, overrides)
                where = f"twin (sim_navigation {overrides})"
                check(active, f"{where}: the localization include is active")
                twin = effective_parameters(args)
                ekf = twin.get("ekf_filter_node")
                twin_lin = twin.get("localization_input_node")
                check(ekf is not None and twin_lin is not None,
                      f"{where}: an EKF and localization_input_node are started",
                      f"args {args}")
                got = fused(ekf.get("imu0_config", [])) if ekf else None
                check(got == robot_imu0,
                      f"{where}: twin imu0 fuses what the robot's fuses ({robot_imu0})",
                      f"twin fused indices {got}")
                got = {k: (twin_lin or {}).get(k) for k in variance_keys}
                want = {k: robot_lin.get(k) for k in variance_keys}
                check(got == want, f"{where}: twin yaw-rate variances equal the robot's",
                      f"twin {got}, robot {want}")
        active, _ = localization_include_args(SIM_NAV, {"odom_source": "ground_truth"})
        check(not active, "twin with odom_source:=ground_truth starts no localization EKF")

        print("Part 4 - every EKF variant")
        _, _, robot_args = wiring.parse_mapping_script()
        variants_robot = set()
        for fuse in BOOLS:
            for gps in BOOLS:
                combo = {"fuse_laser_odometry": fuse, "enable_gps": gps}
                params = effective_parameters({**robot_args, **combo})
                ekf = params["ekf_filter_node"]
                variants_robot.add(ekf["__entity__"])
                check_imu0(ekf.get("imu0_config", []), f"robot args + {combo}")
                check_yaw_rate_sources(ekf, params["localization_input_node"],
                                       f"robot args + {combo}")
        check(len(variants_robot) == 4,
              "the combinations above reach all four EKF definitions",
              f"robot reached {len(variants_robot)}")
        if REAL_AUTONOMY.is_file():
            for laser in BOOLS:
                for fuse in BOOLS:
                    for gps in BOOLS:
                        overrides = {"enable_laser_odometry": laser,
                                     "fuse_laser_odometry": fuse, "enable_gps": gps}
                        active, args = localization_include_args(REAL_AUTONOMY, overrides)
                        where = f"real_autonomy {overrides}"
                        check(active, f"{where}: the localization include is active")
                        if active:
                            params = effective_parameters(args)
                            ekf = params["ekf_filter_node"]
                            check_imu0(ekf.get("imu0_config", []), where)
                            check_yaw_rate_sources(
                                ekf, params["localization_input_node"], where)
    else:
        print("  SKIP  Parts 3-4 launch evaluation: launch/launch_ros not importable.")

    if FAILURES:
        print(f"\nFAIL: {len(FAILURES)} check(s) failed")
        return 1
    print("\nOK: IMU wiring and imu0 yaw-rate fusion as decided")
    return 0


if __name__ == "__main__":
    sys.exit(main())
