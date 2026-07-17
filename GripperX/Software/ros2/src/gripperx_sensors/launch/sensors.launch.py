"""Start LiDAR, IMU, and GPS publishers for the real robot.

Default: sensor_mocks (bench testing).
Set use_mock_sensors:=false and launch your hardware drivers separately
or extend this file with your driver nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sensors_share = get_package_share_directory("gripperx_sensors")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_sensors = LaunchConfiguration("use_mock_sensors")
    sensors_params = os.path.join(sensors_share, "config", "sensors.yaml")

    sensor_mocks = Node(
        package="gripperx_sensors",
        executable="sensor_mocks",
        name="sensor_mocks",
        output="screen",
        condition=IfCondition(use_mock_sensors),
        parameters=[sensors_params, {"use_sim_time": use_sim_time}],
    )

    real_sensors_hint = LogInfo(
        msg=(
            "use_mock_sensors:=false — start your LiDAR/IMU/GPS drivers so they publish "
            "/scan, /imu/data, and optionally /gps/fix (see gripperx_sensors/config/sensors.yaml)."
        ),
        condition=UnlessCondition(use_mock_sensors),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "use_mock_sensors",
                default_value="true",
                description="Publish mock /scan, /imu/data, /gps/fix for bench testing.",
                choices=["true", "True", "false", "False"],
            ),
            sensor_mocks,
            real_sensors_hint,
        ]
    )
