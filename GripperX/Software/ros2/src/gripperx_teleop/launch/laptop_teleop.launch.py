"""
Starts the keyboard teleop on the laptop.

Prerequisite (set once in the terminal):
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=20
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash

Domain 20 is the real GripperX-1 (PlastiX convention: gripper robots 20-29, their
digital twin +200, i.e. 220-229). The laptop must be on the SAME domain as the robot's
services to reach it — the twin domain deliberately cannot see the real robot (SR-8).

Start:
  ros2 launch gripperx_teleop laptop_teleop.launch.py

Switch mode (from the laptop or Pi):
  ros2 topic pub --once /teleop/set_mode std_msgs/String '{data: "controller"}'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # NOTE (pre-existing, not fixed here): keyboard_teleop_node declares
        # neither `linear_speed` nor `angular_speed` nor `output_topic` — the
        # names are `linear_vel_m_s`, `steer_rate_rad_s` and `cmd_vel_topic`.
        # The three overrides below are therefore silently ignored and the
        # node's own defaults apply. Correcting them CHANGES THE DRIVE SPEED
        # (0.5 -> 0.4 m/s), so it is a user decision, not a drive-by fix.
        DeclareLaunchArgument('linear_speed',  default_value='0.4',
                              description='Forward/backward speed [m/s] (see note: not applied)'),
        DeclareLaunchArgument('angular_speed', default_value='0.8',
                              description='Rotational speed [rad/s] (see note: not applied)'),
        # Crab walk / in-place spin on the arrow keys (FR-7). These names ARE
        # declared by the node.
        DeclareLaunchArgument('crab_speed_m_s', default_value='0.25',
                              description='Sideways speed for arrow left/right [m/s]'),
        # Stiction bound: the spin radius is only hypot(a, b) = 0.211 m, so a
        # low spin_speed_rad_s asks for very little linear speed at the wheel,
        # and the firmware's open-loop feedforward (PWM = |rpm| * 0.85,
        # motor_controller.cpp) can land at or below stiction duty -- while a
        # spin needs MORE torque than driving straight, and there is no
        # PID/integrator to push through it (encoders feed odometry only), so
        # a stalled wheel stays stalled. 1.4 rad/s keeps the wheel duty at
        # roughly the same level as normal straight driving at 0.3 m/s.
        # Headroom: swerve_cmd.yaml max_wheel_angular_speed=12.0 only binds at
        # 3.98 rad/s. TO-VERIFY on the ground; override without editing via
        # spin_speed_rad_s:=<value>.
        DeclareLaunchArgument('spin_speed_rad_s', default_value='1.4',
                              description='In-place rotation rate for arrow up/down [rad/s]'),
        DeclareLaunchArgument('use_steer_feedback', default_value='true',
                              description='Arm manoeuvres on measured /hw/steer_states '
                                          'instead of a timeout'),
        Node(
            package='gripperx_teleop',
            executable='keyboard_teleop_node',
            name='keyboard_teleop_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('gripperx_teleop'),
                    'config', 'keyboard_teleop.yaml'),
                {
                'linear_speed':    LaunchConfiguration('linear_speed'),
                'angular_speed':   LaunchConfiguration('angular_speed'),
                'publish_rate_hz': 20.0,
                'output_topic':    '/teleop/keyboard/cmd_vel',
                # Typed explicitly: a LaunchConfiguration is a string, and the
                # node declares these as double/bool — an untyped override
                # fails at node startup with a ParameterTypeException.
                'crab_speed_m_s': ParameterValue(
                    LaunchConfiguration('crab_speed_m_s'), value_type=float),
                'spin_speed_rad_s': ParameterValue(
                    LaunchConfiguration('spin_speed_rad_s'), value_type=float),
                'use_steer_feedback': ParameterValue(
                    LaunchConfiguration('use_steer_feedback'), value_type=bool),
            }],
        ),
    ])
