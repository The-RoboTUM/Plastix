from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("bot_description"))
    rviz_config = package_share / "rviz" / "display.rviz"
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "load_urdf.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_lidar": "false",
            "use_camera": "false",
            "urdf_file": str(package_share / "urdf" / "bot_v1.urdf.xacro"),
            "controllers_file": "",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_joint_state_publisher_gui",
                default_value="true",
                description="Whether to start joint_state_publisher_gui.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock in RViz and robot_state_publisher.",
                choices=["true", "True", "false", "False"],
            ),
            load_urdf,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(use_joint_state_publisher_gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
