"""Real robot bringup: ros2_control + swerve control + optional firmware mock.

Simulation is unchanged — use:
  ros2 launch gripperx_gazebo simulation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# ── Arm ──────────────────────────────────────────────────────────────────────
arm_action_server = Node(
    package="gripperx_arm",
    executable="arm_action_server",
    name="arm_action_server",
    output="screen",
    # Self-healing: dies if /dev/arm_servo is missing at startup (USB is
    # often only plugged in after boot) — retry until the hardware is present.
    respawn=True,
    respawn_delay=5.0,
    parameters=[{"use_sim_time": False}],
)


# ── LiDAR ────────────────────────────────────────────────────────────────────
def make_lidar_node(use_lidar):
    return Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        condition=IfCondition(use_lidar),
        # Self-healing: dies e.g. on port/permission errors; retry
        respawn=True,
        respawn_delay=5.0,
        parameters=[
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'frame_id': 'lidar_link'},
            {'port_name': '/dev/lidar'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
        ],
    )

# ── Lenkservos ───────────────────────────────────────────────────────────────
def make_steer_servo_node(gripperx_control):
    return Node(
        package='gripperx_control',
        executable='steer_servo_node',
        name='steer_servo_node',
        output='screen',
        # Self-healing: due to the boot kernel-panic workaround, USB is only
        # plugged in after boot; the node dies until then (require_all_servos)
        # and then comes back up on its own.
        respawn=True,
        respawn_delay=5.0,
        parameters=[os.path.join(gripperx_control, 'config', 'steer_servo.yaml')],
    )

from ament_index_python.packages import get_package_share_directory as _get_pkg
from launch_ros.parameter_descriptions import ParameterValue


def make_teleop_mux_node():
    import os as _os
    gripperx_teleop = _get_pkg('gripperx_teleop')
    return Node(
        package='gripperx_teleop',
        executable='teleop_mux_node',
        name='teleop_mux',
        output='screen',
        parameters=[_os.path.join(gripperx_teleop, 'config', 'teleop_mux.yaml')],
    )


def generate_launch_description():
    gripperx_description = get_package_share_directory("gripperx_description")
    gripperx_control = get_package_share_directory("gripperx_control")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_firmware = LaunchConfiguration("use_mock_firmware")
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    use_rviz = LaunchConfiguration("use_rviz")

    lidar_node = make_lidar_node(use_lidar)
    steer_servo_node = make_steer_servo_node(gripperx_control)
    teleop_mux_node = make_teleop_mux_node()

    controllers_file = os.path.join(gripperx_control, "config", "ros2_controllers.yaml")
    urdf_file = os.path.join(gripperx_description, "urdf", "gripperx_v1.urdf.xacro")

    robot_description = Command(
        [
            "xacro ",
            urdf_file,
            " mesh_dir:=package://gripperx_description/meshes",
            " use_lidar:=",
            use_lidar,
            " use_camera:=",
            use_camera,
            " enable_ros2_control:=true",
            " use_hw_topics:=true",
        ]
    )

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_description, "launch", "load_urdf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_lidar": use_lidar,
            "use_camera": use_camera,
            "urdf_file": urdf_file,
            "controllers_file": "",
            "enable_ros2_control": "true",
            "use_hw_topics": "true",
        }.items(),
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            controllers_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "steering_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    hw_firmware_mock = Node(
        package="gripperx_control",
        executable="hw_firmware_mock",
        output="screen",
        condition=IfCondition(use_mock_firmware),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_control, "launch", "control.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_description, "launch", "rviz.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock (false on real hardware).",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "use_mock_firmware",
                default_value="true",
                description="Run hw_firmware_mock instead of ESP32/micro-ROS.",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument("use_lidar", default_value="false"),
            DeclareLaunchArgument("use_camera", default_value="false"),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                choices=["true", "True", "false", "False"],
            ),
            arm_action_server,
            lidar_node,
            steer_servo_node,
            teleop_mux_node,
            load_urdf,
            hw_firmware_mock,
            ros2_control_node,
            control,
            rviz,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=ros2_control_node,
                    on_start=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[steering_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[wheel_controller],
                )
            ),
        ]
    )
