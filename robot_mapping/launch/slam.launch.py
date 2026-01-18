from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(
        get_package_share_directory('robot_mapping'),
        'config',
        'slam.yaml'
    )

    map_file = os.path.join(get_package_share_directory("robot_mapping"),"maps/map/map.yaml" )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'map': map_file,
                'params_file': params_file,
                'autostart': 'true',
                'use_navigation': 'true',
                'use_localization': 'true',
            }.items(),
        ),
    ])
