"""
Starts the browser teleop UI on the laptop.

Same prerequisites as laptop_teleop.launch.py:
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=20
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash

Start:
  ros2 launch gripperx_teleop web_teleop.launch.py
  ros2 launch gripperx_teleop web_teleop.launch.py open_browser:=true

then open http://localhost:8080/ .

Run EITHER this or laptop_teleop.launch.py, never both: they publish the same
cmd_vel and would race.

Reachability is a deliberate choice, not a default. `web_host` stays on
127.0.0.1 so the page is only reachable from the laptop that runs it. Setting
it to 0.0.0.0 puts a live drive interface for this robot on the network, where
anyone who can reach the port can drive it -- there is no password. Do that
only on a network you control, and expect the node to say so in its log.
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
        DeclareLaunchArgument('web_host', default_value='127.0.0.1',
                              description='Bind address. 0.0.0.0 exposes the '
                                          'drive controls to the network'),
        DeclareLaunchArgument('web_port', default_value='8080',
                              description='TCP port for the UI'),
        DeclareLaunchArgument('open_browser', default_value='false',
                              description='Open the page automatically on start'),
        # Ride-through for a hiccup on the (unstable, NFR-8) link. Once it
        # expires the held keys are released, so the robot stops within about
        # this long plus one publish tick.
        DeclareLaunchArgument('client_timeout_sec', default_value='0.5',
                              description='How long the page may go silent '
                                          'before the robot is stopped [s]'),
        # Same values laptop_teleop.launch.py passes, so switching input device
        # does not silently change how the robot behaves. See the reasoning for
        # 1.4 rad/s there (below stiction at 0.6).
        DeclareLaunchArgument('crab_speed_m_s', default_value='0.25',
                              description='Sideways speed for arrow left/right [m/s]'),
        DeclareLaunchArgument('spin_speed_rad_s', default_value='1.4',
                              description='In-place rotation rate for arrow up/down [rad/s]'),
        DeclareLaunchArgument('use_steer_feedback', default_value='true',
                              description='Arm manoeuvres on measured '
                                          '/hw/steer_states instead of a timeout'),
        Node(
            package='gripperx_teleop',
            executable='web_teleop_node',
            name='web_teleop_node',
            output='screen',
            parameters=[
                # GEOMETRY SOURCE, added 2026-08-25 when this branch was merged.
                # keyboard_teleop_node declares a, b and wheel_radius WITHOUT a default
                # (Parameter.Type.DOUBLE) so they can only come from the single source of
                # truth. web_teleop_node builds on KeyboardTeleopNode and inherits that
                # declaration, so WITHOUT this file the node raises on startup instead of
                # running -- which is the intended loud failure, but it means the web UI
                # would simply not come up. laptop_teleop.launch.py already passes it; this
                # launch file was written before that fix landed and had the same gap the
                # geometry work found there: an inline dict that silently omits the geometry.
                os.path.join(
                    get_package_share_directory('gripperx_teleop'),
                    'config', 'keyboard_teleop.yaml'),
                {
                # NOTE: linear_vel_m_s is deliberately NOT overridden here.
                # laptop_teleop.launch.py passes `linear_speed`, which the node
                # does not declare, so the terminal teleop has always driven at
                # the node default of 0.5 m/s. Leaving it alone keeps the two
                # front-ends at the same speed; changing it is a user decision.
                # 20 Hz, matching laptop_teleop.launch.py. The node's own
                # default is 50, but the terminal teleop has always run at 20,
                # and "same teleop, different input device" has to include the
                # rate the twist is published at -- it is what teleop_mux and
                # the controller see. Raising it is a separate decision.
                'publish_rate_hz': 20.0,
                'web_host': LaunchConfiguration('web_host'),
                'web_port': ParameterValue(
                    LaunchConfiguration('web_port'), value_type=int),
                'open_browser': ParameterValue(
                    LaunchConfiguration('open_browser'), value_type=bool),
                'client_timeout_sec': ParameterValue(
                    LaunchConfiguration('client_timeout_sec'), value_type=float),
                'crab_speed_m_s': ParameterValue(
                    LaunchConfiguration('crab_speed_m_s'), value_type=float),
                'spin_speed_rad_s': ParameterValue(
                    LaunchConfiguration('spin_speed_rad_s'), value_type=float),
                'use_steer_feedback': ParameterValue(
                    LaunchConfiguration('use_steer_feedback'), value_type=bool),
            },
            ],
        ),
    ])
