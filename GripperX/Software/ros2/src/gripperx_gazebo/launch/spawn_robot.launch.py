import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gripperx_description = get_package_share_directory("gripperx_description")
    gripperx_control = get_package_share_directory("gripperx_control")

    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_description, "launch", "load_urdf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_lidar": use_lidar,
            "use_camera": use_camera,
            "urdf_file": os.path.join(gripperx_description, "urdf", "gripperx_v1.gazebo.xacro"),
            "controllers_file": os.path.join(gripperx_control, "config", "ros2_controllers.yaml"),
            "mesh_dir": f"file://{os.path.join(gripperx_description, 'meshes')}",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        parameters=[
            {
                "topic": "/robot_description",
                "name": "bot",
                "allow_renaming": False,
                "x": spawn_x,
                "y": spawn_y,
                "z": spawn_z,
                "R": spawn_roll,
                "P": spawn_pitch,
                "Y": spawn_yaw,
            }
        ],
    )

    # /clock ORDERING GATE — §3.1.7 "Clock-source hazard", user decision
    # 2026-08-19 part (a), deviation row D17. Sim bringup MUST guarantee the
    # /clock source is up before any controller activates: with use_sim_time and
    # a clock at zero every `now() - stamp` in the chain is 0 and every freshness
    # test reads "always fresh" (the A2 override never expires,
    # cmd_vel_timeout_sec never fires, the watchdog's command_timeout_sec would
    # not either). It fails toward "keep commanding", and silently.
    #
    # The gate is a PROCESS, not a wait inside a node, because that is what makes
    # the guarantee expressible in launch: the spawners hang off its
    # OnProcessExit, so no controller is even LOADED until /clock has been
    # observed to ADVANCE — not merely to exist, since a paused Gazebo publishes
    # a constant clock and freezes the timeouts just as effectively.
    #
    # use_sim_time is forced False here even though the sim launch sets it
    # globally: the gate's own deadline must not be measured against the clock it
    # is waiting for. (It uses time.monotonic() internally regardless; this makes
    # the intent visible at the launch site.)
    #
    # It lives in this file rather than in simulate_robot.launch.py because the
    # spawners live here and launch event handlers do not reach across included
    # launch descriptions. If this file is ever used without a /clock source in
    # front of it, the gate times out, exits non-zero, and NO controller is
    # spawned — the deliberate direction.
    clock_ready_gate = Node(
        package="gripperx_gazebo",
        executable="clock_ready_gate",
        name="clock_ready_gate",
        output="screen",
        parameters=[{"use_sim_time": False}],
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

    # NFR-10 / §3.1 variant B — THE SWITCH-OVER, sim half. Replaces
    # steering_position_controller + wheel_velocity_controller; the topics they
    # carried (/steering_position_controller/commands,
    # /wheel_velocity_controller/commands) cease to exist, and sim_steer_bridge
    # with them — its only output was the first of the two, so it cannot survive
    # this switch even in principle. Arbitration point A2 now lives in
    # swerve_controller for real and sim alike (OP-23 / A2-b, §3.1.6: one
    # implementation, no code fork).
    #
    # TWO --param-file arguments, in this order, and the order is load-bearing.
    # swerve_controller.sim.yaml carries ONLY the difference
    # (wheel_command_multipliers [1,-1,1,-1] for the sim URDF's inverted
    # right-side wheel axes). Passing it alone would make it the controller's
    # COMPLETE parameter set and silently drop geometry, steering windows and
    # timeouts to their compiled defaults. Later files win, so [base, sim] means
    # base everywhere except that one value.
    swerve_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "swerve_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--param-file",
            os.path.join(gripperx_control, "config", "ros2_controllers.yaml"),
            "--param-file",
            os.path.join(gripperx_control, "config", "swerve_controller.sim.yaml"),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            load_urdf,
            spawn_robot,
            RegisterEventHandler(
                event_handler=OnProcessExit(target_action=spawn_robot, on_exit=[clock_ready_gate])
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=clock_ready_gate,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[swerve_controller],
                )
            ),
        ]
    )
