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

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    path_description = os.path.join(get_package_share_directory("robot_description"), "launch")
    path_controller = os.path.join( get_package_share_directory("robot_controller"),  "launch")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_description, "gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),

    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(path_controller, "controller.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),

    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )


    return LaunchDescription([
        use_sim_time_arg,
        gazebo_launch,
        controller_launch,   
        #rviz_node,     
    ])

