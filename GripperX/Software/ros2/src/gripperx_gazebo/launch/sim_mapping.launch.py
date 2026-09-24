import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """DT-4 (M2) / DT-2 preliminary stage: ONE launch for sim (testworld_v1) + SLAM
    (slam_toolbox from /scan) in the digital twin.

    Deliberately does NOT use `gripperx_localization/launch/localization.launch.py`:
    that one always starts an `ekf_filter_node` from the ROS system package
    `robot_localization`, which is missing on this laptop (see the
    `ground_truth_odom_bridge.py` docstring) and could not be installed
    without root/`sudo` password. Instead, the ground-truth odometry already
    intended for M1-M3 per DT-7 (status: clarified) is passed through
    directly as TF odom->base_footprint via `ground_truth_odom_bridge`
    (gripperx_localization); slam_toolbox builds the map on top of that.

    The full Stack-B EKF chain (`localization.launch.py`: wheel odom +
    laser_scan_matcher + IMU) is AVAILABLE as of 2026-08-21 — `robot_localization`
    and `ros2_laser_scan_matcher` are both built and present, and
    `sim_navigation.launch.py` wires it as `odom_source:=ekf`. The precondition
    this comment used to state ("once ros-jazzy-robot-localization is installed")
    has lapsed. Switching THIS launch over is still open work, not a blocked one:
    the ground-truth bridge stays the default here on purpose, because a mapping
    run wants a clean odom source rather than one under test.
    """
    gripperx_gazebo = get_package_share_directory("gripperx_gazebo")
    gripperx_localization_share = Path(get_package_share_directory("gripperx_localization"))

    world_file = LaunchConfiguration("world_file")
    headless = LaunchConfiguration("headless")
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_rviz = LaunchConfiguration("use_rviz")

    localization_params = str(gripperx_localization_share / "config" / "localization.yaml")
    rviz_config = str(gripperx_localization_share / "rviz" / "localization.rviz")
    sim_time_param = {"use_sim_time": True}

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_gazebo, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world_file": world_file,
            "headless": headless,
            "use_lidar": use_lidar,
            "use_camera": use_camera,
            # gripperx_description/rviz.launch.py (model/TF only) stays off; the
            # SLAM-capable RViz config (map/scan/TF) comes further below from
            # gripperx_localization/rviz/localization.rviz, so that two RViz
            # instances don't compete with each other.
            "use_rviz": "false",
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_roll": spawn_roll,
            "spawn_pitch": spawn_pitch,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    ground_truth_odom_bridge = Node(
        package="gripperx_localization",
        executable="ground_truth_odom_bridge",
        name="ground_truth_odom_bridge",
        output="screen",
        parameters=[sim_time_param],
    )

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[localization_params, sim_time_param],
    )

    configure_slam = LifecycleTransition(
        lifecycle_node_names=["/slam_toolbox"],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
    )

    activate_slam_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state="inactive",
            entities=[
                LifecycleTransition(
                    lifecycle_node_names=["/slam_toolbox"],
                    transition_ids=[Transition.TRANSITION_ACTIVATE],
                )
            ],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_mapping",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[sim_time_param],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(gripperx_gazebo, "worlds", "testworld_v1.world.sdf"),
                description="Gazebo world file (default: new DT-4 test world).",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description=(
                    "true (default): gz sim -s -r, no Gazebo window. "
                    "false: gz sim -r without -s, GUI window (desktop session/DISPLAY needed)."
                ),
                choices=["true", "false"],
            ),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="Camera not needed for pure SLAM/mapping.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="-5.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="RViz with map/scan/TF (gripperx_localization/rviz/localization.rviz).",
                choices=["true", "false"],
            ),
            simulation,
            ground_truth_odom_bridge,
            slam_toolbox_node,
            configure_slam,
            activate_slam_on_inactive,
            rviz,
        ]
    )
