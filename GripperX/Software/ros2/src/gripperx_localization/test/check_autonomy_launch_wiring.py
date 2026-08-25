#!/usr/bin/env python3
"""Verification of the wiring between the robot's autostart, the localization stack
and Nav2 — i.e. of the invariant that was BROKEN until 2026-08-24.

Run from the workspace source tree:

    python3 src/gripperx_localization/test/check_autonomy_launch_wiring.py

Parts 1-3 need `launch`/`launch_ros` (sourcing /opt/ros/jazzy is enough) and are
SKIPPED with a loud notice if they are unavailable. Parts 4-7 are pure python and
always run.

WHAT THIS EXISTS TO CATCH
-------------------------
`gripperx_planning/config/nav2.yaml` points `controller_server` and `bt_navigator` at
`odom_topic: /odometry/filtered`. That topic is published by `ekf_filter_node`, which
lives in `gripperx_localization/launch/localization.launch.py`. Until 2026-08-24 the
robot's autostart chain

    gripperx-bringup.service   -> real_robot.launch.py       (no localization at all)
    gripperx-mapping.service   -> mapping.launch.py          (rf2o + slam_toolbox, no EKF)
    gripperx-navigation.service-> gripperx_planning/navigation.launch.py  (Nav2)

started NO EKF anywhere, so `/odometry/filtered` had no publisher and Nav2 came up
green with permanently zero velocity feedback. Nothing failed, nothing logged, and the
defect was found by reading launch files rather than by running them.

The test therefore does not check "is the code as written today". It checks the
PROPERTY: whatever the mapping service starts must publish the topic Nav2 reads its
odometry from. Repointing the service somewhere else in the future is fine — dropping
the EKF out of the chain again is not.

Part 1  Nav2's odom topic has a publisher in the chain the autostart actually starts
Part 2  exactly one node in that chain claims odom -> base_footprint (rf2o is out)
Part 3  the SLAM config that gets loaded is the TUNED one, not the bringup copy
Part 4  fuse_laser_odometry is declared AND forwarded by real_autonomy.launch.py
Part 5  enable_saved_map_localization's default is consistent with the map on disk
Part 6  the script's lifecycle-state match is anchored ("inactive" is not "active")
Part 7  the script is syntactically valid and carries its rollback route
"""

from __future__ import annotations

import ast
import importlib.util
import re
import shutil
import subprocess
import sys
from pathlib import Path

# --------------------------------------------------------------------------------
# Locations. The test is run from the source tree, so everything is resolved relative
# to this file rather than through the ament index — an installed share directory can
# be stale, and a stale copy is exactly what this test must not be fooled by.
# --------------------------------------------------------------------------------
HERE = Path(__file__).resolve()
SRC = HERE.parents[2]                                  # .../Software/ros2/src
REPO = SRC.parents[2]                                  # repository root
SYSTEMD = REPO / "Software" / "pi_env" / "systemd"

MAPPING_SH = SYSTEMD / "scripts" / "gripperx-mapping.sh"
REAL_AUTONOMY = SRC / "gripperx_bringup" / "launch" / "real_autonomy.launch.py"
NAV2_YAML = SRC / "gripperx_planning" / "config" / "nav2.yaml"
LOCALIZATION_YAML = SRC / "gripperx_localization" / "config" / "localization.yaml"
MAPS_DIR = SRC / "gripperx_localization" / "maps"

FAILURES: list[str] = []
CHECKS = 0


def check(condition: bool, label: str, detail: str = "") -> None:
    global CHECKS
    CHECKS += 1
    if condition:
        print(f"  ok    {label}")
    else:
        print(f"  FAIL  {label}")
        if detail:
            for line in detail.splitlines():
                print(f"          {line}")
        FAILURES.append(label)


def section(title: str) -> None:
    print(f"\n{title}\n{'-' * len(title)}")


# --------------------------------------------------------------------------------
# Reading the autostart script. The launch file and its arguments are PARSED OUT OF
# THE SCRIPT rather than hardcoded here, so the test cannot drift away from what the
# robot actually runs — if someone changes the script, the test follows it and then
# judges it.
# --------------------------------------------------------------------------------
def parse_mapping_script() -> tuple[str, str, dict[str, str]]:
    """Return (package, launch_file, {arg: value}) from the ros2 launch invocation."""
    text = MAPPING_SH.read_text()
    # Join backslash-continued lines, then drop comments, then find the invocation.
    joined = re.sub(r"\\\n\s*", " ", text)
    lines = [ln for ln in joined.splitlines() if not ln.lstrip().startswith("#")]
    matches = [ln for ln in lines if "ros2 launch" in ln]
    if len(matches) != 1:
        raise AssertionError(
            f"expected exactly one 'ros2 launch' invocation in {MAPPING_SH.name}, "
            f"found {len(matches)}: {matches!r}"
        )
    invocation = matches[0]
    m = re.search(r"ros2 launch\s+(\S+)\s+(\S+)", invocation)
    if not m:
        raise AssertionError(f"could not parse the launch invocation: {invocation!r}")
    args = dict(re.findall(r"(\w+):=(\S+)", invocation))
    return m.group(1), m.group(2), args


def yaml_scalar(path: Path, node: str, key: str) -> str | None:
    """Read one scalar from a flat `node: / ros__parameters: / key: value` block.

    Deliberately not PyYAML: this must run on a machine with nothing installed, and
    the values wanted here are plain scalars. Trailing `# ...` comments are stripped,
    which matters because localization.yaml writes `0.05 #0.10`.
    """
    inside = False
    for line in path.read_text().splitlines():
        if re.match(rf"^{re.escape(node)}\s*:", line):
            inside = True
            continue
        # A block ends at the next key in column 0 — but NOT at a column-0 comment.
        # nav2.yaml puts long rationale comments at column 0 inside node blocks, and
        # treating those as the end silently truncated the search (found the hard way:
        # bt_navigator's odom_topic sits behind one).
        if inside and re.match(r"^[^\s#]", line):
            break
        if inside:
            m = re.match(rf"^\s+{re.escape(key)}\s*:\s*(.+?)\s*$", line)
            if m:
                return m.group(1).split("#")[0].strip().strip('"').strip("'")
    return None


# --------------------------------------------------------------------------------
# Parts 1-3 — the live launch description
# --------------------------------------------------------------------------------
def live_parts(package: str, launch_file: str, script_args: dict[str, str]) -> None:
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import normalize_to_list_of_substitutions as norm
    from launch.utilities import perform_substitutions
    import launch_ros.actions as lra

    pkg_dir = SRC / package
    launch_path = pkg_dir / "launch" / launch_file
    check(launch_path.is_file(), f"{package}/launch/{launch_file} exists in the source tree",
          f"looked at {launch_path}")
    if not launch_path.is_file():
        return

    spec = importlib.util.spec_from_file_location("_wiring_under_test", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # Resolve package share directories to the SOURCE tree instead of through the
    # ament index. Two reasons, and the second is the important one:
    #   - the test then needs nothing built, so it runs on a fresh clone and on the Pi
    #     before the first colcon build;
    #   - the ament index would resolve to install/, i.e. to a COPY of the launch file
    #     that may predate the change under test. A test that silently reads the last
    #     build instead of the working tree is worse than no test.
    # The layout is a faithful stand-in: a package's share directory is its launch/,
    # config/, maps/ and rviz/ subtrees, which is all these launch files ask for.
    def share_from_source(package_name: str) -> str:
        candidate = SRC / package_name
        if not candidate.is_dir():
            raise FileNotFoundError(
                f"package {package_name!r} not found in the source tree at {candidate}"
            )
        return str(candidate)

    module.get_package_share_directory = share_from_source
    ld = module.generate_launch_description()

    ctx = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            ctx.launch_configurations[entity.name] = perform_substitutions(ctx, entity.default_value)
    unknown = sorted(set(script_args) - set(ctx.launch_configurations))
    check(not unknown, "every argument the autostart passes is declared by the launch file",
          f"undeclared: {unknown}\nthese would abort the launch at boot")
    ctx.launch_configurations.update(script_args)

    live: list[tuple[str, str, str]] = []   # (node_name, package, executable)
    for entity in ld.entities:
        if not isinstance(entity, (lra.Node, lra.LifecycleNode)):
            continue
        if entity.condition is not None and not entity.condition.evaluate(ctx):
            continue
        live.append((
            perform_substitutions(ctx, norm(entity._Node__node_name)),
            perform_substitutions(ctx, norm(entity.node_package)),
            perform_substitutions(ctx, norm(entity.node_executable)),
        ))
    names = {n for n, _, _ in live}
    print(f"  ..    nodes started by the autostart with its own arguments: {sorted(names)}")

    section("Part 1 — Nav2's odometry topic has a publisher in the autostart chain")
    nav_odom = {
        "controller_server": yaml_scalar(NAV2_YAML, "controller_server", "odom_topic"),
        "bt_navigator": yaml_scalar(NAV2_YAML, "bt_navigator", "odom_topic"),
    }
    check(len(set(nav_odom.values())) == 1 and None not in nav_odom.values(),
          "controller_server and bt_navigator agree on one odom_topic",
          f"read from nav2.yaml: {nav_odom}")
    check(all(v is not None for v in nav_odom.values()),
          "no Nav2 odometry consumer relies on its package default",
          f"read from nav2.yaml: {nav_odom}\n"
          "An absent odom_topic makes the node subscribe to whatever its package "
          "default happens to be. That was survivable only while the autostart ran "
          "rf2o (publishing the bare /odom the default points at); it is not "
          "survivable now. Name the topic explicitly on every consumer.")
    topic = nav_odom["controller_server"] or nav_odom["bt_navigator"]
    ekf = [e for e in live if e[1] == "robot_localization" and e[2] == "ekf_node"]
    check(len(ekf) == 1,
          f"exactly one robot_localization/ekf_node is started (Nav2 reads {topic})",
          "THIS IS THE DEFECT THIS FILE EXISTS FOR: with no EKF in the chain, "
          f"{topic} has no publisher and Nav2 runs with permanently zero velocity "
          "feedback while reporting healthy.\n"
          f"started nodes were: {sorted(names)}")
    check(topic == "/odometry/filtered",
          "Nav2's odom_topic is the EKF's default output topic",
          f"nav2.yaml says {topic!r}; robot_localization publishes /odometry/filtered "
          "and localization.launch.py adds no remap for the EKF. If the topic is "
          "changed on either side, change it on both.")

    section("Part 2 — exactly one node claims odom -> base_footprint")
    check(not any(pkg == "rf2o_laser_odometry" for _, pkg, _ in live),
          "rf2o_laser_odometry is NOT in the chain",
          "rf2o runs with publish_tf: true, odom_frame_id: odom, "
          "base_frame_id: base_footprint — the same TF edge the EKF publishes. "
          "Both at once is a TF fight; the old chain avoided it only by having no EKF.")
    check(yaml_scalar(LOCALIZATION_YAML, "ekf_filter_node", "publish_tf") == "true",
          "the EKF is the one that publishes it (ekf_filter_node.publish_tf: true)")
    # Check EVERY node in the chain, not a hand-picked pair. The chain grew a third
    # candidate (laser_odometry_node, enabled by the autostart) after the first version
    # of this test was written, and a check that names its suspects individually would
    # not have noticed. ros2_laser_scan_matcher defaults publish_tf to false and
    # localization.yaml sets it false again — but "it happens to be false today" is not
    # something a test should leave to a reader to re-derive.
    tf_publishers = {}
    for node_name, _, _ in live:
        value = yaml_scalar(LOCALIZATION_YAML, node_name, "publish_tf")
        if value is not None and value.lower() == "true":
            tf_publishers[node_name] = value
    check(set(tf_publishers) == {"ekf_filter_node"},
          "exactly one node in the chain has publish_tf: true, and it is the EKF",
          f"nodes with publish_tf: true = {sorted(tf_publishers) or 'none'}\n"
          f"nodes in the chain = {sorted(names)}\n"
          "Two publishers on odom -> base_footprint make the transform alternate between "
          "two estimates at their respective rates. TF has no arbitration; the consumer "
          "sees whichever arrived last.")
    for node_name in ("localization_input_node", "laser_odometry_node"):
        if node_name in names:
            value = yaml_scalar(LOCALIZATION_YAML, node_name, "publish_tf")
            check(value is not None and value.lower() == "false",
                  f"{node_name} explicitly declines the TF (publish_tf: false)",
                  f"got {value!r}. Both nodes default to false, so an absent key is not a "
                  "defect — but it is then only correct by inheritance. State it.")

    section("Part 3 — the SLAM parameters that get loaded are the tuned ones")
    check("slam_toolbox" in names, "slam_toolbox is started")
    tuned = {
        "minimum_travel_distance": "0.05",
        "minimum_travel_heading": "0.05",
        "angle_variance_penalty": "2.0",
        "distance_variance_penalty": "0.8",
    }
    for key, expected in tuned.items():
        got = yaml_scalar(LOCALIZATION_YAML, "slam_toolbox", key)
        check(got == expected,
              f"localization.yaml slam_toolbox.{key} == {expected}",
              f"got {got!r}. The 2026-08-21 real-robot scan-matcher tuning (4f788e1) "
              "lives ONLY in this file. gripperx_bringup/config/slam_toolbox.yaml is "
              "the older, coarser copy — if the chain loads that one instead, mapping "
              "silently regresses.")
    bringup_slam = SRC / "gripperx_bringup" / "config" / "slam_toolbox.yaml"
    if bringup_slam.is_file():
        other = yaml_scalar(bringup_slam, "slam_toolbox", "minimum_travel_distance")
        check(other != tuned["minimum_travel_distance"],
              "the two SLAM configs are still genuinely different (so this test means something)",
              f"both now read {other!r} — if they were deliberately unified, delete one "
              "of them rather than leaving two files that must be kept in step.")


# --------------------------------------------------------------------------------
# Parts 4-7 — pure python, no ROS
# --------------------------------------------------------------------------------
def declared_arguments(path: Path) -> dict[str, str | None]:
    """{argument name: default_value as a literal string} from DeclareLaunchArgument."""
    tree = ast.parse(path.read_text())
    out: dict[str, str | None] = {}
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and getattr(node.func, "id", None) == "DeclareLaunchArgument"):
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        name = node.args[0].value
        default = None
        for kw in node.keywords:
            if kw.arg == "default_value" and isinstance(kw.value, ast.Constant):
                default = kw.value.value
        out[name] = default
    return out


def forwarded_to_include(path: Path, marker: str) -> set[str]:
    """Keys of the launch_arguments dict of the include whose source mentions `marker`."""
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and getattr(node.func, "id", None) == "IncludeLaunchDescription"):
            continue
        if marker not in ast.dump(node):
            continue
        for kw in node.keywords:
            if kw.arg != "launch_arguments":
                continue
            target = kw.value
            if isinstance(target, ast.Call) and getattr(target.func, "attr", None) == "items":
                target = target.func.value
            if isinstance(target, ast.Dict):
                return {k.value for k in target.keys if isinstance(k, ast.Constant)}
    return set()


def static_parts(script_args: dict[str, str]) -> None:
    section("Part 4 — fuse_laser_odometry is reachable from real_autonomy.launch.py")
    declared = declared_arguments(REAL_AUTONOMY)
    check("fuse_laser_odometry" in declared,
          "real_autonomy.launch.py declares fuse_laser_odometry",
          "without it the EKF can only ever run as ekf_without_laser from this entry "
          "point, which on a robot with no IMU leaves /wheel/odom as its single source.")
    check(declared.get("fuse_laser_odometry") == "false",
          "its default is false",
          f"got {declared.get('fuse_laser_odometry')!r}. A scan matcher can lock up "
          "silently; letting it into the filter is a per-run decision, not a default.")
    forwarded = forwarded_to_include(REAL_AUTONOMY, "localization")
    check("fuse_laser_odometry" in forwarded,
          "and it is forwarded to the localization include",
          f"forwarded arguments are {sorted(forwarded)}. Declaring without forwarding "
          "is worse than not declaring: the argument is accepted and silently ignored.")
    check("enable_laser_odometry" in forwarded,
          "enable_laser_odometry is still forwarded too")

    section("Part 5 — enable_saved_map_localization's default matches the map on disk")
    default_map = re.search(r'default_map_yaml\s*=\s*os\.path\.join\((.+?)\)',
                            REAL_AUTONOMY.read_text(), re.S)
    map_name = None
    if default_map:
        parts = re.findall(r'"([^"]+)"', default_map.group(1))
        map_name = parts[-1] if parts else None
    check(map_name is not None, "the default map filename can be read from the launch file")
    exists = bool(map_name) and (MAPS_DIR / map_name).is_file()
    print(f"  ..    default map is {map_name!r}; present in {MAPS_DIR.name}/: {exists}")
    if not exists:
        check(declared.get("enable_saved_map_localization") == "false",
              "the default map does not exist, so saved-map localization must default to false",
              f"got {declared.get('enable_saved_map_localization')!r}. A true default "
              "here asks map_server to load a file that is not there on every "
              "defaults-only launch.")
    else:
        check(True, f"the default map {map_name} exists — either default is defensible")

    section("Part 6 — the autostart's lifecycle-state match is anchored")
    text = MAPPING_SH.read_text()
    m = re.search(r"ros2 lifecycle get \S+ 2>/dev/null \| grep -q ('|\")(.+?)\1", text)
    check(m is not None, "the script matches slam_toolbox's lifecycle state with a grep pattern",
          "if the shape of that line changed, update this test rather than deleting it")
    if m:
        pattern = m.group(2)
        print(f"  ..    pattern in the script: {pattern!r}")
        # `ros2 lifecycle get` prints e.g. "active [3]" / "inactive [2]". The whole point
        # of the anchor is that the second must NOT satisfy the first. Run the real grep.
        def matches(sample: str) -> bool:
            return subprocess.run(["grep", "-q", pattern], input=sample,
                                  text=True).returncode == 0
        check(matches("active [3]\n"), "it matches 'active [3]'")
        check(not matches("inactive [2]\n"),
              "it does NOT match 'inactive [2]'",
              "an unanchored `grep -q active` reports an INACTIVE node as active, the "
              "safety net never fires, and the service claims to be mapping while "
              "slam_toolbox sits unconfigured.")
        check(not matches("unconfigured [1]\n"), "it does NOT match 'unconfigured [1]'")

    section("Part 7 — the autostart script is valid and carries its rollback route")
    if shutil.which("bash"):
        rc = subprocess.run(["bash", "-n", str(MAPPING_SH)], capture_output=True, text=True)
        check(rc.returncode == 0, "bash -n accepts the script", rc.stderr.strip())
    check("mapping.launch.py" in text,
          "the script names the path it was repointed away from",
          "the rollback comment must keep naming gripperx_bringup/mapping.launch.py, "
          "otherwise the way back is only recoverable from git history.")
    check("ROLLBACK" in text, "a ROLLBACK route is spelled out in the header")
    for key in ("enable_slam", "enable_saved_map_localization", "enable_laser_odometry"):
        check(key in script_args, f"the autostart passes {key} explicitly",
              f"arguments parsed from the script: {script_args}")
    check(script_args.get("enable_slam") == "true"
          and script_args.get("enable_saved_map_localization") == "false",
          "live SLAM, no saved map (user decision 2026-08-24)",
          f"got {script_args}")
    check(script_args.get("use_sim_time") == "false",
          "use_sim_time is false — the YAML hardcodes true and only this wins",
          "localization.yaml's slam_toolbox block sets use_sim_time: true. The launch "
          "file passes sim_time_param AFTER the YAML so the argument wins, but only if "
          "it is actually passed.")


def main() -> int:
    print(__doc__.split("WHAT THIS EXISTS TO CATCH")[0].strip())
    package, launch_file, script_args = parse_mapping_script()
    print(f"\nautostart starts: ros2 launch {package} {launch_file}")
    print(f"with arguments:   {script_args}")

    try:
        import launch  # noqa: F401
        import launch_ros  # noqa: F401
    except ImportError as exc:
        print(f"\n!! Parts 1-3 SKIPPED — launch/launch_ros unavailable ({exc}).")
        print("!! Source /opt/ros/jazzy/setup.bash to run the parts that matter most.")
    else:
        live_parts(package, launch_file, script_args)

    static_parts(script_args)

    print(f"\n{'=' * 72}")
    if FAILURES:
        print(f"FAILED — {len(FAILURES)} of {CHECKS} checks:")
        for f in FAILURES:
            print(f"  - {f}")
        return 1
    print(f"PASSED — {CHECKS} checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
