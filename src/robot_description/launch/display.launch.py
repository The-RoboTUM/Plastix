from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_share = Path(get_package_share_directory("robby_description"))
    xacro_path = package_share / "urdf" / "Robby_v1.urdf.xacro"
    rviz_config = package_share / "rviz" / "display.rviz"
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    robot_description = xacro.process_file(
        str(xacro_path),
        mappings={"mesh_dir": "package://robby_description/meshes"},
    ).toxml()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_joint_state_publisher_gui",
                default_value="true",
                description="Whether to start joint_state_publisher_gui.",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(use_joint_state_publisher_gui),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_config)],
            ),
        ]
    )
