from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import LifecycleTransition
from lifecycle_msgs.msg import Transition


def config_is_true(config: LaunchConfiguration) -> PythonExpression:
    return PythonExpression(["'", config, "' == 'true'"])


def config_is_false(config: LaunchConfiguration) -> PythonExpression:
    return PythonExpression(["'", config, "' != 'true'"])


def all_configs(*conditions: PythonExpression) -> PythonExpression:
    expression = []
    for index, condition in enumerate(conditions):
        if index > 0:
            expression.append(" and ")
        expression.append(condition)
    return PythonExpression(expression)


def generate_launch_description():
    localization_share = Path(get_package_share_directory("gripperx_localization"))
    enable_laser_odometry = LaunchConfiguration("enable_laser_odometry")
    fuse_laser_odometry = LaunchConfiguration("fuse_laser_odometry")
    enable_gps = LaunchConfiguration("enable_gps")
    enable_slam = LaunchConfiguration("enable_slam")
    enable_saved_map_localization = LaunchConfiguration("enable_saved_map_localization")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    localization_params = str(localization_share / "config" / "localization.yaml")
    # DT-4: default points to the map generated via SLAM in the digital twin
    # for the new testworld_v1 (replaces the old, discarded arena_map.yaml).
    default_map_yaml = str(localization_share / "maps" / "testworld_v1_map.yaml")
    default_rviz_config = str(localization_share / "rviz" / "localization.rviz")
    sim_time_param = {"use_sim_time": use_sim_time}

    slam_condition = IfCondition(
        all_configs(
            config_is_true(enable_slam),
            config_is_false(enable_saved_map_localization),
        )
    )
    saved_map_localization_condition = IfCondition(config_is_true(enable_saved_map_localization))

    gps_odom_fusion = {
        "odom2": "/odometry/gps",
        "odom2_config": [
            True,
            True,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
        ],
        "odom2_queue_size": 10,
        "odom2_differential": False,
        "odom2_relative": False,
    }

    localization_input = Node(
        package="gripperx_localization",
        executable="localization_input_node",
        name="localization_input_node",
        output="screen",
        parameters=[localization_params, sim_time_param],
    )

    laser_odometry = Node(
        package="ros2_laser_scan_matcher",
        executable="laser_scan_matcher",
        name="laser_odometry_node",
        output="screen",
        condition=IfCondition(enable_laser_odometry),
        parameters=[localization_params, sim_time_param],
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        condition=IfCondition(enable_gps),
        parameters=[localization_params, sim_time_param],
        remappings=[
            ("/imu", "/imu/data/filtered"),
            ("/gps/fix", "/gps/fix"),
            ("/odometry/filtered", "/wheel/odom"),
        ],
    )

    ekf_with_laser = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(
            all_configs(config_is_true(fuse_laser_odometry), config_is_false(enable_gps))
        ),
        parameters=[localization_params, sim_time_param],
    )

    ekf_with_laser_and_gps = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(
            all_configs(config_is_true(fuse_laser_odometry), config_is_true(enable_gps))
        ),
        parameters=[localization_params, sim_time_param, gps_odom_fusion],
    )

    ekf_without_laser = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(
            all_configs(config_is_false(fuse_laser_odometry), config_is_false(enable_gps))
        ),
        parameters=[
            localization_params,
            sim_time_param,
            {
                "odom1": "",
                "odom1_config": [
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                ],
            },
        ],
    )

    ekf_without_laser_with_gps = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(
            all_configs(config_is_false(fuse_laser_odometry), config_is_true(enable_gps))
        ),
        parameters=[
            localization_params,
            sim_time_param,
            gps_odom_fusion,
            {
                "odom1": "",
                "odom1_config": [
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                ],
            },
        ],
    )

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        condition=slam_condition,
        parameters=[localization_params, sim_time_param],
    )

    configure_slam = LifecycleTransition(
        lifecycle_node_names=["/slam_toolbox"],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
        condition=slam_condition,
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
        ),
        condition=slam_condition,
    )

    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        condition=saved_map_localization_condition,
        parameters=[
            localization_params,
            sim_time_param,
            {
                "yaml_filename": map_yaml_file,
            },
        ],
    )

    amcl = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace="",
        output="screen",
        condition=saved_map_localization_condition,
        parameters=[localization_params, sim_time_param],
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        condition=saved_map_localization_condition,
        parameters=[
            sim_time_param,
            {
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["map_server", "amcl"],
            },
        ],
    )

    # THE ONE CHECK IN THE STACK THAT IS NOT SELF-REFERENTIAL. Every Nav2 tolerance
    # compares the robot pose against the goal in the SAME estimated frame, so an
    # error in the estimate cancels out of the comparison exactly -- that is how a
    # run ended 2.47 m off target with SUCCEEDED and zero recoveries on 2026-08-21.
    # Ground truth does not exist outside simulation, but wheel odometry and laser
    # odometry measure the same motion by physically different means and are
    # therefore independent OF EACH OTHER. This node says when they part company.
    # It reports; it never decides which source is right and never touches a command.
    #
    # Deliberately placed HERE rather than in a sim-only launch: the defect it
    # catches is in laser_scan_matcher and robot_localization, not in Gazebo, and
    # the real robot has no ground truth to fall back on. Same file, both platforms.
    odom_divergence_monitor = Node(
        package="gripperx_localization",
        executable="odom_divergence_monitor",
        name="odom_divergence_monitor",
        output="screen",
        condition=IfCondition(enable_laser_odometry),
        parameters=[localization_params, sim_time_param],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_localization",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[sim_time_param],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_laser_odometry",
                default_value="true",
                description=(
                    "Run the laser scan matcher, i.e. publish /laser/odom. Since "
                    "2026-08-21 this NO LONGER implies fusing it — see "
                    "fuse_laser_odometry. Keep it true even when not fusing: the "
                    "topic is the second, independent source odom_divergence_monitor "
                    "cross-checks the wheel odometry against, and with it off the "
                    "monitor has nothing to compare and stays silent."
                ),
            ),
            DeclareLaunchArgument(
                "fuse_laser_odometry",
                default_value="false",
                description=(
                    "Fuse /laser/odom into the EKF as odom1. DEFAULT false since "
                    "2026-08-21: localization.yaml fuses it as an ABSOLUTE POSE while "
                    "laser_scan_matcher fills neither pose nor twist covariance, so "
                    "robot_localization reads it as near-certainty with no rejection "
                    "threshold to gate it. A silent scan-matcher lock-up then drove "
                    "the estimate 2.62 m off over one 5.4 s straight leg while Nav2 "
                    "reported SUCCEEDED; the same case without the fusion ended 0.10 m "
                    "out. Safe to re-enable only once the covariance is real."
                ),
            ),
            DeclareLaunchArgument(
                "enable_gps",
                default_value="false",
                description="Fuse GPS via navsat_transform_node (/odometry/gps into EKF).",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="false",
                description="Start slam_toolbox mapping from /scan.",
            ),
            DeclareLaunchArgument(
                "enable_saved_map_localization",
                default_value="true",
                description=(
                    "Start nav2_map_server + AMCL (publishes /map for RViz and Nav2). "
                    "Set false for SLAM-only or EKF-only."
                ),
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value=default_map_yaml,
                description="Absolute path to the occupancy-grid YAML used by nav2_map_server.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock (false on real hardware).",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with localization.rviz (map, scan, Nav2 goal).",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Absolute path to the RViz config file.",
            ),
            localization_input,
            laser_odometry,
            navsat_transform,
            ekf_with_laser_and_gps,
            ekf_with_laser,
            ekf_without_laser_with_gps,
            ekf_without_laser,
            odom_divergence_monitor,
            slam_toolbox_node,
            configure_slam,
            activate_slam_on_inactive,
            map_server,
            amcl,
            lifecycle_manager_localization,
            rviz,
        ]
    )
