from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sharx_communication',
            executable='dummy_octopus',
            name='dummy_octopus',
            output='screen',
        ),
    ])