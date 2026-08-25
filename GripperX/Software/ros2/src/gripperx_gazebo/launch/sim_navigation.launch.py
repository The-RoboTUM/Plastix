import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node, SetUseSimTime
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

# See navigation.launch.py: the .rosdeps_local Nav2 debs (map_server, amcl,
# lifecycle_manager) need their newer diagnostic_updater ABI prepended to
# LD_LIBRARY_PATH, while gz sim / controllers / slam_toolbox (system) must keep
# the system one. Set by scripts/sim_env_nav2.sh; empty when unset (no-op).
_ROSDEPS_LIB = os.environ.get("GRIPPERX_ROSDEPS_LIB", "")
_NAV2_ENV = (
    {"LD_LIBRARY_PATH": _ROSDEPS_LIB + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")}
    if _ROSDEPS_LIB
    else {}
)


def generate_launch_description():
    """DT-2 / M3 (DT-5): ONE launch for the integrated sim autonomy stack —
    Gazebo sim + stack-B localization + Nav2 — in the correct order and with
    use_sim_time:=true consistently.

    Localization (odom source, DT-7), two mutually exclusive modes:

    `odom_source:=ground_truth` (default, OP-12, binding for M1-M3): the perfect
    ground-truth odometry (`/ground_truth/odom`) passed through as TF
    odom->base_footprint by `ground_truth_odom_bridge` and republished on
    `/odometry/filtered` (the stack-B odom topic Nav2 expects).

    `odom_source:=ekf` (M4/DT-9, wired 2026-08-21): the realistic chain — wheel
    forward kinematics (`localization_input_node` -> /wheel/odom),
    laser_scan_matcher (/laser/odom) and the IMU (/imu/data -> /imu/data/filtered)
    fused by robot_localization, which then owns the odom->base_footprint TF and
    publishes /odometry/filtered itself. Nav2 needs no change either way.

    This mode was blocked on "ros2_laser_scan_matcher, not installed on this
    laptop" until 2026-08-21; the package is built and present in `install/`, so
    the reason had lapsed. Two things to know before reading results from it:

    - **Gazebo's IMU carries no noise model** (`gripperx_v1.gazebo.xacro`: no
      `<noise>` on the sensor), so this exercises the wiring and the kinematics,
      NOT robustness against IMU bias or drift.
    - `/ground_truth/odom` keeps publishing in EKF mode — the gz OdometryPublisher
      plugin is unconditional. It is the REFERENCE the fused estimate can be
      measured against, which is the point of the mode: the wheel-odometry scale
      moved +35 % (0.052 -> 0.070) and no drive test has validated it
      (internal `NAV2_HANDOVER` §3).

    Note that `imu0` is the filter's ONLY absolute yaw source — `odom0` (wheel)
    contributes rates only and `odom1` (laser) does not take yaw. On hardware,
    where the IMU does not exist yet, that is a decision to make, not a setting.

    Map -> odom: default `localization:=slam` runs slam_toolbox online (proven
    M2 chain), building the map as the robot drives — required for full-arena M3
    goals because the saved M2 map only covers the southern strip. `localization
    :=amcl` loads that saved map (`gripperx_localization/maps/testworld_v1_map.yaml`)
    into nav2_map_server + AMCL, with the initial pose overridden to the actual
    spawn; goals are then confined to the mapped strip.

    `odom_source:=ekf` DOES reuse `gripperx_localization/launch/localization.launch.py`
    rather than restating the EKF chain here, but only its odometry half: it is
    included with `enable_slam:=false`, `enable_saved_map_localization:=false`
    and `use_rviz:=false`, because map->odom and RViz are owned by this file. The
    competing-TF hazard that kept the two apart is handled by the conditions —
    `ground_truth` and `ekf` can never both be active.

    RViz: started standalone per DIGITAL_TWIN_PLAN.md §9.4 (use_rviz default
    false); launch-started RViz is a known flakiness issue.
    """
    gripperx_gazebo = get_package_share_directory("gripperx_gazebo")
    gripperx_planning = get_package_share_directory("gripperx_planning")
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
    odom_source = LaunchConfiguration("odom_source")
    enable_laser_odometry = LaunchConfiguration("enable_laser_odometry")
    fuse_laser_odometry = LaunchConfiguration("fuse_laser_odometry")
    localization = LaunchConfiguration("localization")
    map_yaml_file = LaunchConfiguration("map_yaml_file")

    localization_params = str(gripperx_localization_share / "config" / "localization.yaml")
    rviz_config = str(gripperx_localization_share / "rviz" / "localization.rviz")
    default_map_yaml = str(gripperx_localization_share / "maps" / "testworld_v1_map.yaml")
    sim_time_param = {"use_sim_time": True}

    use_ground_truth = IfCondition(
        PythonExpression(["'", odom_source, "' == 'ground_truth'"])
    )
    use_ekf = IfCondition(PythonExpression(["'", odom_source, "' == 'ekf'"]))
    use_amcl = IfCondition(PythonExpression(["'", localization, "' == 'amcl'"]))
    use_slam = IfCondition(PythonExpression(["'", localization, "' == 'slam'"]))

    # --- Gazebo sim + control/teleop chain (mux started in autonomous mode) ---
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_gazebo, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world_file": world_file,
            "headless": headless,
            "use_lidar": use_lidar,
            "use_camera": use_camera,
            "use_rviz": "false",
            "initial_mode": "autonomous",
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_roll": spawn_roll,
            "spawn_pitch": spawn_pitch,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    # --- Odometry (DT-7): ground truth for M1-M3, republished on the stack-B
    #     topic /odometry/filtered so nav2.yaml (odom_topic) works unchanged. ---
    ground_truth_odom_bridge = Node(
        package="gripperx_localization",
        executable="ground_truth_odom_bridge",
        name="ground_truth_odom_bridge",
        output="screen",
        condition=use_ground_truth,
        parameters=[sim_time_param, {"output_topic": "/odometry/filtered"}],
    )

    # --- Odometry alternative (DT-9): the real EKF chain, odometry half only.
    #     map->odom (slam_toolbox / map_server+AMCL) and RViz stay with THIS
    #     file, so the include is told to start neither. ---
    localization_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gripperx_localization_share / "launch" / "localization.launch.py")
        ),
        condition=use_ekf,
        launch_arguments={
            "use_sim_time": "true",
            "enable_laser_odometry": enable_laser_odometry,
            "fuse_laser_odometry": fuse_laser_odometry,
            "enable_gps": "false",
            "enable_slam": "false",
            "enable_saved_map_localization": "false",
            "use_rviz": "false",
        }.items(),
    )

    # --- Map -> odom option A: saved map + AMCL ---
    # NOTE: the saved M2 map (testworld_v1_map) is world-aligned but only covers
    # the SOUTHERN strip (map y in [-5.93, -2.23]); localization.yaml's AMCL
    # initial_pose (0,0,0) is outside it. Override the initial pose to the actual
    # spawn (map == world for this map): (0, -5, 90 deg). AMCL mode is therefore
    # only usable for goals inside that strip; for full-arena M3 use
    # localization:=slam (default).
    amcl_initial_pose = {
        "set_initial_pose": True,
        "initial_pose.x": 0.0,
        "initial_pose.y": -5.0,
        "initial_pose.z": 0.0,
        "initial_pose.yaw": 1.5708,
    }
    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        condition=use_amcl,
        parameters=[sim_time_param, {"yaml_filename": map_yaml_file}],
        additional_env=_NAV2_ENV,
    )

    amcl = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace="",
        output="screen",
        condition=use_amcl,
        parameters=[localization_params, sim_time_param, amcl_initial_pose],
        additional_env=_NAV2_ENV,
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        condition=use_amcl,
        parameters=[
            sim_time_param,
            {
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["map_server", "amcl"],
            },
        ],
        additional_env=_NAV2_ENV,
    )

    # --- Map -> odom option B: online slam_toolbox ---
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        condition=use_slam,
        parameters=[localization_params, sim_time_param],
    )
    configure_slam = LifecycleTransition(
        lifecycle_node_names=["/slam_toolbox"],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
        condition=use_slam,
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
        condition=use_slam,
    )

    # --- Nav2 (gripperx_planning): planner/controller/behavior/bt + lifecycle mgr ---
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_planning, "launch", "navigation.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_navigation",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[sim_time_param],
    )

    return LaunchDescription(
        [
            SetUseSimTime(True),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(gripperx_gazebo, "worlds", "testworld_v1.world.sdf"),
                description="Gazebo world (default: testworld_v1, matches the saved M2 map).",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                choices=["true", "false"],
                description="true: gz sim -s -r (no window). false: GUI window.",
            ),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="Camera not needed for Nav2.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="-5.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="1.5708",
                description="Must match the M2 mapping start pose for AMCL init to align.",
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="ground_truth",
                choices=["ground_truth", "ekf"],
                description=(
                    "ground_truth (DT-7, M1-M3, default): /ground_truth/odom -> TF "
                    "odom->base_footprint + /odometry/filtered. ekf (M4/DT-9): "
                    "wheel + laser + IMU fused by robot_localization, which owns "
                    "the TF instead; /ground_truth/odom stays available as the "
                    "reference to measure against. The sim IMU has no noise model."
                ),
            ),
            DeclareLaunchArgument(
                "enable_laser_odometry",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "odom_source:=ekf only: run the laser scan matcher, i.e. publish "
                    "/laser/odom. This no longer implies fusing it — see "
                    "fuse_laser_odometry. Leave it true even when not fusing: the topic "
                    "is what odom_divergence_monitor cross-checks the wheel odometry "
                    "against, and the monitor is silent without it."
                ),
            ),
            DeclareLaunchArgument(
                "fuse_laser_odometry",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "odom_source:=ekf only: fuse /laser/odom into the EKF as odom1. "
                    "DEFAULT false since 2026-08-21 — a silent scan-matcher lock-up "
                    "drove the estimate 2.62 m off over one 5.4 s straight leg while "
                    "Nav2 reported SUCCEEDED with zero recoveries; the same case "
                    "without the fusion ended 0.10 m out. Cause: fused as an absolute "
                    "pose with an all-zero covariance and no rejection threshold. See "
                    "NAV2_HANDOVER.md section 9."
                ),
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="slam",
                choices=["amcl", "slam"],
                description=(
                    "slam (default): online slam_toolbox, builds the map while "
                    "driving — needed for full-arena M3 goals since the saved M2 "
                    "map only covers the southern strip. amcl: saved M2 map + "
                    "nav2_amcl (goals limited to that strip)."
                ),
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value=default_map_yaml,
                description="Occupancy-grid YAML for nav2_map_server (localization:=amcl).",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                choices=["true", "false"],
                description="Prefer standalone RViz (DIGITAL_TWIN_PLAN.md §9.4).",
            ),
            simulation,
            ground_truth_odom_bridge,
            localization_ekf,
            map_server,
            amcl,
            lifecycle_manager_localization,
            slam_toolbox_node,
            configure_slam,
            activate_slam_on_inactive,
            navigation,
            rviz,
        ]
    )
