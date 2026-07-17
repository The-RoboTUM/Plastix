import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    gripperx_gazebo = get_package_share_directory("gripperx_gazebo")
    gripperx_control = get_package_share_directory("gripperx_control")
    gripperx_teleop = get_package_share_directory("gripperx_teleop")

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "use_lidar": LaunchConfiguration("use_lidar"),
            "use_camera": LaunchConfiguration("use_camera"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_roll": LaunchConfiguration("spawn_roll"),
            "spawn_pitch": LaunchConfiguration("spawn_pitch"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
        }.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_control, "launch", "control.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            # Sim URDF: right wheel joints have axis "0 0 -1" -> right wheels
            # must be inverted in the bridge so that driving straight actually
            # goes straight (DT-4/M2 investigation). Real robot stays unchanged.
            "bridge_config": os.path.join(
                gripperx_control, "config", "joint_command_bridge.sim.yaml"
            ),
        }.items(),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[
            {"config_file": os.path.join(gripperx_gazebo, "config", "gz_bridge.yaml")}
        ],
    )

    # DT-10: sim counterpart to steer_servo_node. Maps /teleop/direct_steer
    # ([FL,FR,BL,BR] = [angle,angle,-angle,-angle]) directly onto the
    # steering_position_controller (real servo steering path, same axis
    # kinematics as the real robot) and arbitrates -- as on the real robot --
    # against the swerve-derived steering (/swerve_cmd_joint_states) for the
    # Nav2/autonomous path. Replaces the earlier angular.z workaround steering
    # (swerve IK), which produced diverging angles on left/right rotation
    # ("fighting each other"). joint_command_bridge no longer publishes
    # steering for this (publish_steering=false in
    # joint_command_bridge.sim.yaml), only wheels.
    sim_steer_bridge = Node(
        package="gripperx_control",
        executable="sim_steer_bridge",
        name="sim_steer_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # DT-3/DT-4 (M1/M2): teleop_mux has so far been completely missing from
    # the sim -- on the real robot it's only started via
    # gripperx_bringup/real_robot.launch.py. Without it, /teleop/keyboard/cmd_vel
    # never reaches /cmd_vel. Hooked in here (instead of a dedicated launch
    # file) because simulate_robot.launch.py is already the sim counterpart to
    # real_robot.launch.py, which likewise starts teleop_mux directly
    # alongside control.
    # DT-10: keyboard_pass_angular_z=false (= real-robot behavior). A/D
    # steering now runs via the real servo steering path
    # (/teleop/direct_steer -> sim_steer_bridge ->
    # steering_position_controller), NOT via the angular.z workaround steering
    # through the swerve IK anymore. In keyboard mode, teleop_mux therefore
    # only passes through linear.x (W/S drive).
    teleop_mux = Node(
        package="gripperx_teleop",
        executable="teleop_mux_node",
        name="teleop_mux",
        output="screen",
        parameters=[
            os.path.join(gripperx_teleop, "config", "teleop_mux.yaml"),
            {
                "initial_mode": LaunchConfiguration("initial_mode"),
                "keyboard_pass_angular_z": False,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_mode",
                default_value="keyboard",
                description="teleop_mux start mode: keyboard | controller | autonomous.",
                choices=["keyboard", "controller", "autonomous"],
            ),
            SetUseSimTime(True),
            spawn_robot,
            control,
            gz_bridge,
            sim_steer_bridge,
            teleop_mux,
        ]
    )
