import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_controller = get_package_share_directory("robot_controller")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="robot_controller",
                executable="swerve_cmd_node",
                name="swerve_cmd_node",
                output="screen",
                parameters=[
                    os.path.join(robot_controller, "config", "swerve_cmd.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="robot_controller",
                executable="joint_command_bridge",
                name="joint_command_bridge",
                output="screen",
                parameters=[
                    os.path.join(robot_controller, "config", "joint_command_bridge.yaml"),
                    {
                        "use_sim_time": use_sim_time,
                        "command_topic": "/swerve_cmd_joint_states",
                    },
                ],
            ),
        ]
    )
