import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    declare_map_yaml = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            get_package_share_directory("robot_mapping"),
            "maps",
            "map",
            "map.yaml"
        ),
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("robot_mapping"),
            "config",
            "nav2_params.yaml"
        ),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py"
            )
        ),
        launch_arguments={'slam': "False",
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': 'true',
                          'use_composition': 'True',
                          'use_respawn': 'False'}.items()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("robot_mapping"),
                "rviz",
                "nav.rviz"
            )
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_params_file,
        nav2_bringup,
        rviz,
    ])
