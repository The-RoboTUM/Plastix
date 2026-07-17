import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

# Digital-twin only: when the Nav2 core packages come from the locally-extracted
# .rosdeps_local debs (laptop, no system-wide Nav2), those debs were built
# against a newer diagnostic_updater ABI than this laptop's system ROS. Giving
# ONLY the Nav2 C++ nodes an LD_LIBRARY_PATH that prepends the .rosdeps_local
# libs lets them load their matching diagnostic_updater, while gz sim /
# controller_manager (separate process) keep the system one. The env var is set
# by scripts/sim_env_nav2.sh (scratch helper). Unset on the real robot / a
# proper Nav2 install -> empty dict -> behavior byte-identical.
_ROSDEPS_LIB = os.environ.get("GRIPPERX_ROSDEPS_LIB", "")
_NAV2_ENV = (
    {"LD_LIBRARY_PATH": _ROSDEPS_LIB + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")}
    if _ROSDEPS_LIB
    else {}
)


def generate_launch_description():
    planning_share = Path(get_package_share_directory("gripperx_planning"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_terrain_cost = LaunchConfiguration("enable_terrain_cost")
    terrain_cost_yaml = LaunchConfiguration("terrain_cost_yaml")
    params_file = LaunchConfiguration("params_file")
    default_nav_to_pose_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml")
    default_nav_through_poses_bt_xml = LaunchConfiguration("default_nav_through_poses_bt_xml")

    nav2_params = str(planning_share / "config" / "nav2.yaml")
    default_terrain_yaml = str(planning_share / "maps" / "terrain_cost.yaml")
    nav_to_pose_bt_xml = str(
        planning_share / "config" / "navigate_to_pose_w_replanning_and_recovery.xml"
    )
    nav_through_poses_bt_xml = str(
        planning_share / "config" / "navigate_through_poses_w_replanning_and_recovery.xml"
    )

    sim_time_param = {"use_sim_time": use_sim_time}

    planner_server = LifecycleNode(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace="",
        output="screen",
        parameters=[params_file, sim_time_param],
        additional_env=_NAV2_ENV,
    )

    # GAP-1 (DT-2): teleop_mux is the single owner of /cmd_vel. Nav2's
    # controller_server (path following) and behavior_server (spin/backup/
    # drive-on-heading recoveries) both publish cmd_vel, so their output must
    # go to the mux's autonomous input (/teleop/autonomous/cmd_vel), NOT
    # directly to /cmd_vel — otherwise two publishers fight on /cmd_vel (mux
    # zeros vs. Nav2) and the mux mode/override/stop chain cannot gate autonomy.
    # The mux must be in "autonomous" mode for Nav2 to reach /cmd_vel (wired via
    # initial_mode:=autonomous in the integrated bringup, sim_navigation.launch.py).
    autonomous_cmd_vel_remap = [("cmd_vel", "/teleop/autonomous/cmd_vel")]

    controller_server = LifecycleNode(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace="",
        output="screen",
        parameters=[params_file, sim_time_param],
        remappings=autonomous_cmd_vel_remap,
        additional_env=_NAV2_ENV,
    )

    behavior_server = LifecycleNode(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace="",
        output="screen",
        parameters=[params_file, sim_time_param],
        remappings=autonomous_cmd_vel_remap,
        additional_env=_NAV2_ENV,
    )

    bt_navigator = LifecycleNode(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace="",
        output="screen",
        parameters=[
            params_file,
            sim_time_param,
            {
                "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                "default_nav_through_poses_bt_xml": default_nav_through_poses_bt_xml,
            },
        ],
        additional_env=_NAV2_ENV,
    )

    terrain_cost_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="terrain_cost_map_server",
        output="screen",
        parameters=[
            sim_time_param,
            {"yaml_filename": terrain_cost_yaml, "frame_id": "map"},
        ],
        remappings=[("/map", "/terrain_costmap")],
        condition=IfCondition(enable_terrain_cost),
        additional_env=_NAV2_ENV,
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            sim_time_param,
            {
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            },
        ],
        additional_env=_NAV2_ENV,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock (set false on real hardware).",
            ),
            DeclareLaunchArgument(
                "enable_terrain_cost",
                default_value="false",
                description="Load optional terrain_cost map on /terrain_costmap (needs terrain_layer in nav2.yaml).",
            ),
            DeclareLaunchArgument(
                "terrain_cost_yaml",
                default_value=default_terrain_yaml,
                description="Absolute path to terrain_cost.yaml (same origin as arena_map).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=nav2_params,
                description="Absolute path to the Nav2 parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=nav_to_pose_bt_xml,
                description="Behavior tree XML used for NavigateToPose goals.",
            ),
            DeclareLaunchArgument(
                "default_nav_through_poses_bt_xml",
                default_value=nav_through_poses_bt_xml,
                description="Behavior tree XML used for NavigateThroughPoses goals.",
            ),
            terrain_cost_map_server,
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager_navigation,
        ]
    )
