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
def make_arm_action_server(arm_home_on_startup):
    gripperx_arm = get_package_share_directory("gripperx_arm")
    return Node(
        package="gripperx_arm",
        executable="arm_action_server",
        name="arm_action_server",
        output="screen",
        # Self-healing: dies if /dev/arm_servo is missing at startup (USB is
        # often only plugged in after boot) — retry until the hardware is present.
        respawn=True,
        respawn_delay=5.0,
        parameters=[
            os.path.join(gripperx_arm, "config", "arm_poses.yaml"),
            {
                "use_sim_time": False,
                # Startup homing repeats on every respawn. Set false while the home
                # pose is being re-taught, otherwise each restart drives the arm onto
                # stale tick values. value_type=bool is explicit because the node
                # declares this as a bool and the substitution arrives as a string —
                # a type mismatch here would be a respawn crash loop.
                "home_on_startup": ParameterValue(arm_home_on_startup, value_type=bool),
            },
        ],
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
            # The driver publishes RAW; scan_range_filter republishes /scan with
            # the self-returns removed (see gripperx_sensors/scan_range_filter.py
            # and the node started below). Every consumer keeps reading /scan and
            # needs no change. If the filter is not running there is no /scan at
            # all - a loud failure, chosen over silently feeding unfiltered data
            # to consumers that cannot tell.
            {'topic_name': 'scan_raw'},
            {'frame_id': 'lidar_link'},
            {'port_name': '/dev/lidar'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            # SELF-OCCLUSION MASK (arm/gripper, CAD 2026-08-18). The arm and
            # gripper are permanently inside the scan plane BEHIND the sensor;
            # structurally unavoidable. The driver masks data INSIDE the interval
            # (see ldlidar_stl_ros2/launch/ld19.launch.py header): everything with
            # angle_crop_min <= angle <= angle_crop_max is set to NaN. (This said
            # "zeroed" until 2026-08-24; demo.cpp:199-201 sets quiet_NaN(), and
            # the difference matters -- scan_range_filter only rewrites FINITE
            # values, so crop-masked points pass through it untouched.)
            #
            # Occluders at the 260 mm scan plane, all within +-14 deg of the rear:
            # gripper_base (0.064-0.145 m), gripper_connector (0.142-0.148 m),
            # l_mid/r_mid (0.145-0.154 m). limb1_1 reaches +-27 deg but sits at
            # 0.032-0.072 m, below the declared min_range of 0.10, so it is
            # filtered anyway. +-20 deg therefore covers the persistent returns
            # with margin. FALLBACK: widen to 150.0/210.0 if limb1 shows up in a
            # real scan.
            #
            # CAVEAT 1: the crop angle is in the SENSOR's own frame, not the
            # robot's. A rotated mount means 180 deg is not the robot's rear —
            # validate against a real scan before trusting this.
            # CAVEAT 2: the masked sector is BLIND, not filtered. Reversing is
            # unguarded there and relies purely on costmap memory.
            {'enable_angle_crop_func': True},
            {'angle_crop_min': 160.0},
            {'angle_crop_max': 200.0},
        ],
    )

# ── Scan range filter ────────────────────────────────────────────────────────
# Sits between the driver and every consumer: driver -> /scan_raw -> here ->
# /scan. The LD06 reports from 2 cm and the arm and gripper are permanently in
# the scan plane, so without this the robot maps its own structure. Measured
# 2026-08-21: every self-return below 0.10 m, nothing at all between 0.10 and
# 0.35 m. The costmaps have obstacle_min_range/raytrace_min_range for this, but
# slam_toolbox has no equivalent parameter, which is why filtering at the source
# is the only place that covers every consumer at once.
def make_scan_range_filter(use_lidar):
    return Node(
        package='gripperx_sensors',
        executable='scan_range_filter',
        name='scan_range_filter',
        output='screen',
        condition=IfCondition(use_lidar),
        # respawn like the driver above it. Audit 2026-08-24 found this was the
        # only node in the lidar chain without it: with the driver on /scan_raw,
        # this node dying takes /scan away from slam_toolbox and both costmaps
        # for the rest of the session. The loud failure is intended, a permanent
        # one is not, and the driver it depends on already respawns.
        respawn=True,
        respawn_delay=5.0,
        parameters=[
            {'input_topic': '/scan_raw'},
            {'output_topic': '/scan'},
            {'min_range': 0.10},
        ],
    )


# ── LiDAR power branch (HWR-23) ──────────────────────────────────────────────
def make_lidar_power_node(gripperx_control, use_lidar):
    return Node(
        package='gripperx_control',
        executable='lidar_power_node',
        name='lidar_power_node',
        output='screen',
        condition=IfCondition(use_lidar),
        # respawn=False ON PURPOSE. The relay is normally-closed, so releasing the
        # GPIO line already re-powers the LiDAR; a respawn would then re-claim it
        # and drive ON again. Both halves would silently undo an operator's
        # deliberate power-off. A crash of this node must stay visible.
        respawn=False,
        parameters=[os.path.join(gripperx_control, 'config', 'lidar_power.yaml')],
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
    arm_home_on_startup = LaunchConfiguration("arm_home_on_startup")

    arm_action_server = make_arm_action_server(arm_home_on_startup)
    lidar_node = make_lidar_node(use_lidar)
    scan_range_filter = make_scan_range_filter(use_lidar)
    lidar_power_node = make_lidar_power_node(gripperx_control, use_lidar)
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

    # respawn: a hardware component that refuses to activate (SR-14 gate, no valid
    # /hw/steer_states inside steer_states_activation_timeout_sec) aborts this process, and
    # without a respawn the whole controller_manager is simply gone — no CM services, two
    # spawners failing with a message that never names the cause. Respawning restores the
    # diagnosable surface. It cannot itself cause motion: every activation first waits for a
    # measured steering angle and then commands exactly that angle with the wheels at zero
    # (SR-14), so a respawned activation is a hold, not a move. Note what it does NOT do:
    # controller state lives in this process, so a respawn comes back with no controllers
    # loaded and the one-shot spawners are long gone — the chain is diagnosable again, not
    # working again. Recovery stays operator-triggered (SR-13 / NFR-12 / OP-21).
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            controllers_file,
            {"use_sim_time": use_sim_time},
        ],
        respawn=True,
        respawn_delay=5.0,
    )

    # --controller-manager-timeout 150 (was 30). It MUST exceed the SR-14 activation gate
    # (steer_states_activation_timeout_sec, user-set to 120 s in gripperx_v1.ros2_control.xacro)
    # plus the rest of controller_manager startup: the CM does not serve the spawners while
    # the hardware component's on_activate() is still blocked on /hw/steer_states, so a
    # spawner timeout below the gate kills the spawners while the gate is still legitimately
    # waiting — and the gate then never gets to do its job or report why it failed.
    # 150 = 120 + 30 s margin. If the gate value changes, change these three with it.
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "150",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # NFR-10 / §3.1 variant B — THE SWITCH-OVER. One controller replaces
    # steering_position_controller + wheel_velocity_controller AND the two nodes
    # that used to feed them (swerve_cmd_node, joint_command_bridge; see the
    # removed control.launch.py include below).
    #
    # ALL THREE claim the same eight command interfaces, so ros2_control refuses
    # to activate a second claimant — a spawner list containing swerve_controller
    # must never also contain the other two (ros2_controllers.yaml says the same
    # at the declaration site).
    #
    # --param-file is deliberately NOT passed here: on the real robot the
    # swerve_controller parameters come from ros2_controllers.yaml, which is
    # already a parameters file of ros2_control_node above, and the controller
    # inherits it. The sim path DOES pass --param-file, because it layers
    # swerve_controller.sim.yaml on top (spawn_robot.launch.py).
    swerve_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "swerve_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "150",
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

    # control.launch.py (swerve_cmd_node + joint_command_bridge) IS DELIBERATELY
    # NOT INCLUDED ANY MORE — NFR-10 / SR-10, and this removal is load-bearing,
    # not tidiness.
    #
    # joint_command_bridge publishes /hw/joint_commands, i.e. the HARDWARE
    # BOUNDARY. GripperXInterface publishes it too, as soon as swerve_controller
    # writes the command interfaces. Leaving the include in place would make the
    # bridge a THIRD publisher there; the documented normal state is EXACTLY 2
    # (GripperXInterface main + its watchdog, D4 / OP-18b / SR-10 (3)).
    #
    # The guarantee is STRUCTURAL, not an ordering trick: the two are not both in
    # this launch description at all, so there is no window — not even a
    # transient one — in which the bridge and the controller both write
    # /hw/joint_commands. There is nothing to sequence.
    #
    # Both nodes stay on disk (control.launch.py, swerve_cmd_node.py,
    # joint_command_bridge.py) so that reverting one commit restores the old
    # chain; deletion is the step after hardware sign-off.

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
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_camera", default_value="false"),
            DeclareLaunchArgument(
                "arm_home_on_startup",
                # PERMANENTLY false — user decision 2026-08-25. Not an interim
                # state: startup homing means unattended arm motion, repeated on
                # every respawn (respawn=True), and a correct pose does not make an
                # unannounced move safe. Rationale in full in arm_poses.yaml.
                #
                # NOTE this argument is listed AFTER arm_poses.yaml in
                # make_arm_action_server's parameters list, so it OVERRIDES the file.
                # Setting home_on_startup in the YAML alone is inert here — both must
                # agree, and they do.
                default_value="false",
                description=(
                    "Move the arm to poses.home when arm_action_server starts. "
                    "Defaults to false permanently (2026-08-25): the node respawns, "
                    "so this means unattended arm motion at unpredictable moments. "
                    "Pass true only for a deliberate, attended test."
                ),
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                choices=["true", "True", "false", "False"],
            ),
            arm_action_server,
            lidar_node,
            scan_range_filter,
            lidar_power_node,
            steer_servo_node,
            teleop_mux_node,
            load_urdf,
            hw_firmware_mock,
            ros2_control_node,
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
                    on_exit=[swerve_controller],
                )
            ),
        ]
    )
