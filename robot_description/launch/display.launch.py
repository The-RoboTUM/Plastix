import os 
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
##full directry to the package 
    robot_description_dir = get_package_share_directory("robot_description")

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    model_arg = DeclareLaunchArgument(
        name= "model", default_value= os.path.join(
            robot_description_dir, "urdf", "robot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )

## convert urdf model into ros2 for rviz
    robot_state_publisher_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters = [{"robot_description": robot_description,
                     "use_sim_time": True}]

    )

# it permits to move the joints
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )


    rviz_node = Node (
        package= "rviz2",
        executable="rviz2",
        name= "rviz2",
        output= "screen",
        arguments=["-d", os.path.join(
                get_package_share_directory("robot_description"),
                "rviz",
                "display.rviz"
            )
        ]
    )



    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,     
        rviz_node   
    ])

