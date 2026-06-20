from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sharx_communication',
            executable='thruster_mixer',
            name='thruster_mixer',
            output='screen',
        ),
        Node(
            package='sharx_communication',
            executable='movement_status',
            name='movement_status',
            output='screen',
        ),
    ])
