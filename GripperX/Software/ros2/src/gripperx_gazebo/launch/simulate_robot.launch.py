import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    gripperx_gazebo = get_package_share_directory("gripperx_gazebo")
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

    # control.launch.py (swerve_cmd_node + joint_command_bridge) IS DELIBERATELY
    # NOT INCLUDED ANY MORE — NFR-10 / §3.1.2. Both nodes are replaced by
    # swerve_controller, which is spawned in spawn_robot.launch.py and writes the
    # eight command interfaces directly. The right-wheel inversion that used to
    # live in joint_command_bridge.sim.yaml survives as
    # swerve_controller.sim.yaml's wheel_command_multipliers [1,-1,1,-1], layered
    # onto the spawner there — dropping it makes the twin veer off on a
    # straight-ahead command.
    #
    # Structural, not an ordering trick: on the real robot joint_command_bridge
    # is the node that would become a THIRD publisher on /hw/joint_commands
    # (SR-10, exactly 2 permitted). Keeping real and sim on the same launch
    # structure is §3.1.6's no-code-fork rule, so it comes out here too.

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[
            {"config_file": os.path.join(gripperx_gazebo, "config", "gz_bridge.yaml")}
        ],
    )

    # SAME FILTER AS THE REAL ROBOT, on purpose. The bridge delivers /scan_raw
    # and this republishes /scan with returns below the floor removed. In the
    # twin there is nothing to remove — the simulated LD06 does not see the arm —
    # so it is a pass-through here. It runs anyway because a filter present on
    # one platform and absent on the other is exactly the kind of sim/real split
    # that cost this project the axis-mirror confusion: every consumer should
    # read a topic that was produced the same way on both machines.
    scan_range_filter = Node(
        package="gripperx_sensors",
        executable="scan_range_filter",
        name="scan_range_filter",
        output="screen",
        parameters=[
            {"input_topic": "/scan_raw"},
            {"output_topic": "/scan"},
            {"min_range": 0.10},
            {"use_sim_time": True},
        ],
    )

    # sim_steer_bridge IS GONE FROM THE ACTIVE PATH — DT-10 / OP-23 / A2-b.
    # It was the sim's duplicate of arbitration point A2, and its only output
    # topic was /steering_position_controller/commands. That topic ceases to
    # exist with this rebuild (§3.1.2), so the node cannot survive the switch
    # even in principle. A2 now lives in swerve_controller and is therefore ONE
    # implementation shared by real and sim (§3.1.6, no code fork) instead of the
    # two copies it was (steer_servo_node L648 + sim_steer_bridge L161).
    # DT-10's requirement stands; its implementation does not.
    # The file stays on disk until after hardware sign-off.

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
            # gz_bridge FIRST in the list: it is the /clock source (gz_bridge.yaml
            # bridges gz Clock -> /clock GZ_TO_ROS), and spawn_robot's
            # clock_ready_gate blocks the controller spawners until that clock
            # ADVANCES (§3.1.7 / D17 part (a)). List order is not itself a
            # guarantee — the gate is — but starting the source first means the
            # gate is not paying for it.
            gz_bridge,
            scan_range_filter,
            spawn_robot,
            teleop_mux,
        ]
    )
