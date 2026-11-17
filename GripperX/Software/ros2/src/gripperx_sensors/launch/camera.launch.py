"""Start the front USB camera (jpeg_camera_node) on its own.

Not part of the bringup. Publishes /camera/front/image_raw/compressed (JPEG
from the camera, undecoded) and /camera/front/camera_info. Starts no
controller and commands nothing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_DEVICE = (
    "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_1080P_Low_Light_UC684-video-index0"
)


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("gripperx_sensors"), "config", "camera_front.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=params),
            DeclareLaunchArgument("video_device", default_value=DEFAULT_DEVICE),
            Node(
                package="gripperx_sensors",
                executable="jpeg_camera",
                namespace="camera/front",
                name="camera_front",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"video_device": LaunchConfiguration("video_device")},
                ],
            ),
        ]
    )
