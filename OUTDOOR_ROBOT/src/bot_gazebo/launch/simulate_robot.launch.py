import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    bot_gazebo = get_package_share_directory("bot_gazebo")
    bot_control = get_package_share_directory("bot_control")

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "use_lidar": LaunchConfiguration("use_lidar"),
            "use_camera": LaunchConfiguration("use_camera"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_roll": LaunchConfiguration("spawn_roll"),
            "spawn_pitch": LaunchConfiguration("spawn_pitch"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
        }.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_control, "launch", "control.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[
            {"config_file": os.path.join(bot_gazebo, "config", "gz_bridge.yaml")}
        ],
    )

    return LaunchDescription(
        [
            SetUseSimTime(True),
            spawn_robot,
            control,
            gz_bridge,
        ]
    )
