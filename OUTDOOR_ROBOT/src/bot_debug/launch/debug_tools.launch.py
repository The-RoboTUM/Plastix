from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    logger = Node(
        package="bot_debug",
        executable="debug_log_node",
        name="debug_log_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([logger])
