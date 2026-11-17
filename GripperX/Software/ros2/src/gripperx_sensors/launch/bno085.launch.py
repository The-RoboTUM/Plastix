"""Start the BNO085 IMU driver on its own.

The real-robot bringup (gripperx_bringup real_robot.launch.py, use_imu) starts
the same node itself since 2026-10-20; this file is for running it standalone
with the bringup's driver disabled (use_imu:=false). Two instances cannot share
the sensor: the second waits on the lock. Publishes on the topic set in
config/bno085.yaml (/imu/bno085/data), which localization_input_node
conditions onto /imu/data/filtered; what the EKF fuses from it is set in
gripperx_localization's localization.yaml. Starts no controller and commands
nothing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("gripperx_sensors"), "config", "bno085.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=params),
            Node(
                package="gripperx_sensors",
                executable="bno085_imu",
                name="bno085_imu",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
