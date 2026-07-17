"""Real-robot teleop: micro-ROS + steering servos + optional arm + optional keyboard/joystick.

Started on the Pi:
  - teleop_joint_commands_node  (/cmd_vel → /hw/joint_commands)
  - steer_servo_node             (ST3215 steering servos on /dev/steering_servo)
  - arm_action_server            (only if use_arm:=true, /dev/arm_servo must be present)

The micro_ros_agent is started via gripperx-agent.service (systemd).
The teleop input runs on the development machine (ROS_DOMAIN_ID=42).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    gripperx_control = get_package_share_directory('gripperx_control')

    teleop     = LaunchConfiguration('teleop')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_arm    = LaunchConfiguration('use_arm')

    use_keyboard = PythonExpression(["'", teleop, "' == 'keyboard'"])
    use_joy      = PythonExpression(["'", teleop, "' == 'joy'"])

    teleop_bridge = Node(
        package='gripperx_control',
        executable='teleop_joint_commands_node',
        name='teleop_joint_commands_node',
        output='screen',
        parameters=[
            os.path.join(gripperx_control, 'config', 'teleop_joint_commands.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    steer_servo = Node(
        package='gripperx_control',
        executable='steer_servo_node',
        name='steer_servo_node',
        output='screen',
        parameters=[
            os.path.join(gripperx_control, 'config', 'steer_servo.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    arm_server = Node(
        package='gripperx_arm',
        executable='arm_action_server',
        name='arm_action_server',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_arm),
    )

    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        condition=IfCondition(use_keyboard),
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'autorepeat_rate': 20.0}],
        condition=IfCondition(use_joy),
    )

    joy_teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[{
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear.x': 0.5,
            'scale_angular.yaw': -1.0,
            'require_enable_button': False,
        }],
        condition=IfCondition(use_joy),
    )

    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='none',
            description='Local teleop input: none, keyboard, joy.',
            choices=['none', 'keyboard', 'joy']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_arm', default_value='false',
            description='Start arm action server (only if /dev/arm_servo is present)'),
        teleop_bridge,
        steer_servo,
        arm_server,
        keyboard_teleop,
        joy_node,
        joy_teleop,
    ])
