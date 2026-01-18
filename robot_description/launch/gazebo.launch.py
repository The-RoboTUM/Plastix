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
    
    robot_description_dir = get_package_share_directory("robot_description")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value= "True"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")]),
                                 value_type=str
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="small_house")

    world_path = PathJoinSubstitution([
            robot_description_dir,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )
    
    model_path = str(Path(robot_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("robot_description"), 'models')


    model_arg = DeclareLaunchArgument(
        name= "model", default_value= os.path.join(
            robot_description_dir, "urdf", "robot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file"
    )

    robot_state_publisher_node = Node(
        package= "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters = [{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}]

    )
#parent directory 
# gazebo_resource_path is an instance of SetEnvironmentVarible
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

#To start gazebo simulation we need to call another launch file 
# Include other launch file that we want to start inside of this launch file 

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "robot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )


    return LaunchDescription([
        model_arg,
        world_name_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])

