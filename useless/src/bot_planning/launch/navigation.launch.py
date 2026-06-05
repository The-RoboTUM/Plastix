from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    planning_share = Path(get_package_share_directory("bot_planning"))
    bt_navigator_share = Path(get_package_share_directory("nav2_bt_navigator"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_terrain_cost = LaunchConfiguration("enable_terrain_cost")
    terrain_cost_yaml = LaunchConfiguration("terrain_cost_yaml")
    params_file = LaunchConfiguration("params_file")
    default_nav_to_pose_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml")
    default_nav_through_poses_bt_xml = LaunchConfiguration("default_nav_through_poses_bt_xml")

    nav2_params = str(planning_share / "config" / "nav2.yaml")
    default_terrain_yaml = str(planning_share / "maps" / "terrain_cost.yaml")
    # Local copy from Nav2 docs (see config/navigate_to_pose_w_replanning_and_recovery.xml).
    nav_to_pose_bt_xml = str(
        planning_share / "config" / "navigate_to_pose_w_replanning_and_recovery.xml"
    )
    nav_through_poses_bt_xml = str(
        bt_navigator_share
        / "behavior_trees"
        / "navigate_through_poses_w_replanning_and_recovery.xml"
    )

    sim_time_param = {"use_sim_time": use_sim_time}

    planner_server = LifecycleNode(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace="",
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    controller_server = LifecycleNode(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace="",
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    behavior_server = LifecycleNode(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace="",
        output="screen",
        parameters=[params_file, sim_time_param],
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
