"""nav2 navigation stack.

Prerequisite: gripperx-bringup.service + gripperx-mapping.sh are running.
Start: gripperx-navigation.sh
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('gripperx_bringup'), 'config', 'nav2_params.yaml'
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': lifecycle_nodes,
        }],
    )

    # Command chain: controller/behavior -> /nav/cmd_vel_raw -> velocity_smoother
    # -> /teleop/autonomous/cmd_vel -> teleop_mux. The smoother's input and output
    # MUST be different topics, otherwise it recursively smooths its own signal.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/nav/cmd_vel_raw')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/nav/cmd_vel_raw')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel', '/nav/cmd_vel_raw'),
            ('cmd_vel_smoothed', '/teleop/autonomous/cmd_vel'),
        ],
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        nav2_lifecycle_manager,
    ])
