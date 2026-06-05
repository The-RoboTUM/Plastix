import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bot_control = get_package_share_directory("bot_control")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="bot_control",
                executable="swerve_cmd_node",
                name="swerve_cmd_node",
                output="screen",
                parameters=[
                    os.path.join(bot_control, "config", "swerve_cmd.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="bot_control",
                executable="joint_command_bridge",
                name="joint_command_bridge",
                output="screen",
                parameters=[
                    os.path.join(bot_control, "config", "joint_command_bridge.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "command_topic": "/swerve_cmd_joint_states",
                    },
                ],
            ),
        ]
    )
