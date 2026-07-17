"""
Starts the keyboard teleop on the laptop.

Prerequisite (set once in the terminal):
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash

Start:
  ros2 launch gripperx_teleop laptop_teleop.launch.py

Switch mode (from the laptop or Pi):
  ros2 topic pub --once /teleop/set_mode std_msgs/String '{data: "controller"}'
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('linear_speed',  default_value='0.4',
                              description='Forward/backward speed [m/s]'),
        DeclareLaunchArgument('angular_speed', default_value='0.8',
                              description='Rotational speed [rad/s]'),
        Node(
            package='gripperx_teleop',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            parameters=[{
                'linear_speed':    LaunchConfiguration('linear_speed'),
                'angular_speed':   LaunchConfiguration('angular_speed'),
                'publish_rate_hz': 20.0,
                'output_topic':    '/teleop/keyboard/cmd_vel',
            }],
        ),
    ])
