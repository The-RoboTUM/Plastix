"""Starts the teleop multiplexer (runs on the Pi, part of bringup)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gripperx_teleop'), 'config', 'teleop_mux.yaml'
    )
    return LaunchDescription([
        Node(
            package='gripperx_teleop',
            executable='teleop_mux_node',
            name='teleop_mux',
            output='screen',
            parameters=[config],
        ),
    ])
