import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gripperx_control = get_package_share_directory("gripperx_control")
    use_sim_time = LaunchConfiguration("use_sim_time")
    # Fix 7 / NFR-1 (#11): switches the joint_command_bridge mapping from a
    # separate node into swerve_cmd_node (one DDS node + one hop less per
    # cycle). Default false -> exactly previous behavior (separate node
    # remains the standard and fallback path). Details: gripperx_control/docs/FIX7_DEPLOY.md
    use_integrated_bridge = LaunchConfiguration("use_integrated_bridge")
    # bridge_config: allows the sim path to load its own joint_command_bridge
    # configuration with inverted right-wheel multipliers
    # (sim's URDF axis convention, see joint_command_bridge.sim.yaml).
    # Default = real configuration -> real_robot.launch.py stays unchanged.
    bridge_config = LaunchConfiguration("bridge_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=os.path.join(gripperx_control, "config", "joint_command_bridge.yaml"),
                description=(
                    "joint_command_bridge config. Default: real config "
                    "([1,1,1,1]). The sim path (simulate_robot.launch.py) sets "
                    "joint_command_bridge.sim.yaml here ([1,-1,1,-1]) because of the "
                    "inverted right wheel axes in the sim URDF."
                ),
            ),
            DeclareLaunchArgument(
                "use_integrated_bridge",
                default_value="false",
                description=(
                    "true: joint_command_bridge mapping directly in swerve_cmd_node "
                    "(separate bridge node is no longer needed). false (default): "
                    "unchanged behavior with separate joint_command_bridge node."
                ),
                choices=["true", "false"],
            ),
            Node(
                package="gripperx_control",
                executable="swerve_cmd_node",
                name="swerve_cmd_node",
                output="screen",
                parameters=[
                    os.path.join(gripperx_control, "config", "swerve_cmd.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "enable_integrated_bridge": use_integrated_bridge,
                    },
                ],
            ),
            Node(
                package="gripperx_control",
                executable="joint_command_bridge",
                name="joint_command_bridge",
                output="screen",
                condition=UnlessCondition(use_integrated_bridge),
                parameters=[
                    bridge_config,
                    {
                        "use_sim_time": use_sim_time,
                        "command_topic": "/swerve_cmd_joint_states",
                    },
                ],
            ),
        ]
    )
