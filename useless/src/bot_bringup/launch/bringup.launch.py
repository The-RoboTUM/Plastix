"""Real robot bringup only. For Gazebo use bot_gazebo/simulation.launch.py."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = Path(get_package_share_directory("bot_description"))
    control_share = Path(get_package_share_directory("bot_control"))
    use_sim_time = LaunchConfiguration("use_sim_time")

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(description_share / "launch" / "load_urdf.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_lidar": "false",
            "use_camera": "false",
            "urdf_file": str(description_share / "urdf" / "bot_v1.urdf.xacro"),
            "controllers_file": "",
        }.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(control_share / "launch" / "control.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (false on real hardware).",
                choices=["true", "True", "false", "False"],
            ),
            load_urdf,
            control,
            # TODO: ros2_control spawners when bot_hardware_interfaces exists.
        ]
    )
