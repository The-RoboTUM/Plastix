from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripperx_arm',
            executable='arm_action_server.py',
            name='arm_action_server',
            output='screen',
            parameters=[{'use_sim_time': False}],
        )
    ])
