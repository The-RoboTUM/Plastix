"""Offline replay stack of sub-issue 2.5: input node, EKF and slam_toolbox, nothing else.

Started by gripperx_localization.imu_replay, which also forces the DDS isolation.
Same nodes, node names and parameter layering as the "ekf_without_laser" and SLAM
branches of localization.launch.py, plus one file of per-run overrides applied last.
localization.yaml itself is never edited for an experiment. laser_scan_matcher is
not started: /laser/odom is not fused (fuse_laser_odometry defaults to false on the
robot) and comes from the bag. No controller, no bringup, nothing that commands.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.parameter_descriptions import ParameterFile
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = Path(get_package_share_directory("gripperx_localization"))
    base = str(share / "config" / "localization.yaml")
    override = ParameterFile(LaunchConfiguration("override_params"), allow_substs=False)
    sim_time = {"use_sim_time": True}

    input_node = Node(
        package="gripperx_localization",
        executable="localization_input_node",
        name="localization_input_node",
        output="screen",
        parameters=[base, sim_time, override],
    )
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        # odom1 off exactly as in localization.launch.py's ekf_without_laser.
        parameters=[base, sim_time, {"odom1": "", "odom1_config": [False] * 15}, override],
    )
    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[base, sim_time, override],
    )
    configure = LifecycleTransition(
        lifecycle_node_names=["/slam_toolbox"],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            goal_state="inactive",
            entities=[
                LifecycleTransition(
                    lifecycle_node_names=["/slam_toolbox"],
                    transition_ids=[Transition.TRANSITION_ACTIVATE],
                )
            ],
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "override_params",
                description="Absolute path of the per-run override YAML, applied last.",
            ),
            input_node,
            ekf,
            slam,
            configure,
            activate,
        ]
    )
