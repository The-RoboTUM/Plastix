#!/usr/bin/env python3
"""
Launch file for Octopus Fleet ROS 2 nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'db_path',
            default_value='octopusfinal.db',
            description='Path to SQLite database'
        ),
        
        # Bridge node - connects database to ROS 2
        Node(
            package='octopus_fleet',
            executable='bridge_node',
            name='octopus_bridge',
            output='screen',
            parameters=[{
                'db_path': LaunchConfiguration('db_path'),
            }]
        ),
        
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '$(find octopus_fleet)/config/octopus.rviz'],
            output='screen'
        ),
        
        # Static transform for map frame (laptop origin)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])