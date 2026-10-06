import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gripperx_arm = get_package_share_directory('gripperx_arm')
    # Startup homing drives the arm without being asked. Keep it switchable from the
    # command line so a re-teach session can start the node without moving anything.
    home_on_startup = LaunchConfiguration('home_on_startup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'home_on_startup',
            default_value='true',
            description='Move the arm to poses.home on node start. Set false while '
                        'the home pose is being re-taught.',
            choices=['true', 'false'],
        ),
        Node(
            package='gripperx_arm',
            # Installed via CMakeLists RENAME, i.e. without the .py suffix.
            executable='arm_action_server',
            name='arm_action_server',
            output='screen',
            parameters=[
                os.path.join(gripperx_arm, 'config', 'arm_poses.yaml'),
                {
                    'use_sim_time': False,
                    # Explicit bool: the node declares this as a bool and the
                    # substitution arrives as a string.
                    'home_on_startup': ParameterValue(home_on_startup, value_type=bool),
                },
            ],
        )
    ])
