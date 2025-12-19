"""
Octopus Main Launch File
Launches all brain nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octopus_package',
            executable='octopus_main_node',
            name='octopus_main_node',
            output='screen',
        ),
        Node(
            package='octopus_package',
            executable='task_database_node',
            name='task_database_node',
            output='screen',
        ),
        Node(
            package='octopus_package',
            executable='location_database_node',
            name='location_database_node',
            output='screen',
        ),
        Node(
            package='octopus_package',
            executable='rtk_gps_node',
            name='rtk_gps_node',
            output='screen',
        ),
    ])

