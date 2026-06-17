from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('sharx_communication'),
        'config',
        'sharx.yaml',
    )

    return LaunchDescription([
        Node(
            package='sharx_communication',
            executable='sharx_receiver',
            name='sharx_receiver',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='sharx_communication',
            executable='dummy_octopus',
            name='dummy_octopus',
            output='screen',
        ),
    ])