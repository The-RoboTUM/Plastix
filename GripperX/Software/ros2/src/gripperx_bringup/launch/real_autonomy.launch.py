"""Full real-robot stack: hardware + sensors + localization + optional Nav2.

Modular toggles let you run the same pieces separately while developing.

Examples:
  # Base robot only (same as real_robot.launch.py)
  ros2 launch gripperx_bringup real_autonomy.launch.py \\
    enable_sensors:=false enable_localization:=false enable_navigation:=false

  # Mapping outdoors (SLAM, no saved map)
  ros2 launch gripperx_bringup real_autonomy.launch.py \\
    enable_slam:=true enable_saved_map_localization:=false enable_navigation:=false

  # Navigate on a saved map (map_yaml_file MUST name a map that exists — the
  # arena_map.yaml default does not, see the enable_saved_map_localization comment)
  ros2 launch gripperx_bringup real_autonomy.launch.py \\
    enable_slam:=false enable_saved_map_localization:=true enable_navigation:=true \\
    map_yaml_file:=/absolute/path/to/some_map.yaml

  # Live SLAM + Nav2 on the real robot — the combination the robot's own autostart
  # runs (Software/pi_env/systemd/scripts/gripperx-mapping.sh) and the one the
  # 2026-08-25 Nav2 deploy/test list drives by hand (tracked internally, not in
  # this repository)
  ros2 launch gripperx_bringup real_autonomy.launch.py \\
    enable_slam:=true enable_saved_map_localization:=false \\
    enable_laser_odometry:=true enable_navigation:=true

Sensor reality check (2026-08-13) — read before trusting any localisation output.
The EKF in gripperx_localization/config/localization.yaml fuses three inputs, and on
the real robot none of them currently carries usable data:

  odom0  /wheel/odom            flat until the encoder firmware is flashed AND
                                COUNTS_PER_OUTPUT_REV is bench-measured
                                (Software/microros/firmware/include/motor_controller.hpp)
  odom1  /laser/odom            TWO switches, and both must be true for the EKF to
                                see it: enable_laser_odometry (runs the matcher and
                                publishes the topic) and fuse_laser_odometry (lets it
                                into the filter). Both default to false; the second
                                was not forwarded from this file at all until
                                2026-08-24
  imu0   /imu/data/filtered     BNO085 is not connected (documentation/ASBUILT.md)

Until 2026-08-13 this was masked: use_mock_sensors defaulted to true, so sensor_mocks
fabricated /imu/data and a second /scan, and the stack looked functional. It is not.
Fix the inputs before drawing conclusions from a map or a pose estimate.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _reject_mock_sensors(context, *args, **kwargs):
    """Refuse to start with mock sensors on the real robot.

    This launch file hardwires use_lidar:="true" (see the `robot` include), so the real
    LD06 always runs. sensor_mocks publishes /scan on the same topic AND the same frame
    (lidar_link, gripperx_sensors/config/sensors.yaml) — two publishers whose output is
    indistinguishable downstream.

    The IMU is the worse half: sensor_mocks also publishes /imu/data, which
    localization_input_node filters into /imu/data/filtered and robot_localization fuses
    as imu0 (gripperx_localization/config/localization.yaml). The BNO085 is not connected
    (ASBUILT), so that path fabricates the only attitude input the EKF has. A map built
    on it looks plausible and is wrong, which is why this refuses rather than warns.

    For bench runs without hardware use the simulation
    (gripperx_gazebo/simulation.launch.py) or sensors.launch.py on its own.
    """
    if LaunchConfiguration("use_mock_sensors").perform(context).lower() in ("true", "1"):
        raise RuntimeError(
            "use_mock_sensors:=true is not valid in real_autonomy.launch.py — this launch "
            "always starts the real LD06, so mock /scan collides with it and mock "
            "/imu/data would be fused into the EKF as if it were a real sensor. "
            "Use gripperx_gazebo/simulation.launch.py for a sensor-free bench run."
        )
    return []


def generate_launch_description():
    bringup_share = get_package_share_directory("gripperx_bringup")
    sensors_share = get_package_share_directory("gripperx_sensors")
    localization_share = get_package_share_directory("gripperx_localization")
    planning_share = get_package_share_directory("gripperx_planning")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_firmware = LaunchConfiguration("use_mock_firmware")
    use_mock_sensors = LaunchConfiguration("use_mock_sensors")
    enable_sensors = LaunchConfiguration("enable_sensors")
    enable_localization = LaunchConfiguration("enable_localization")
    enable_navigation = LaunchConfiguration("enable_navigation")
    enable_laser_odometry = LaunchConfiguration("enable_laser_odometry")
    fuse_laser_odometry = LaunchConfiguration("fuse_laser_odometry")
    enable_gps = LaunchConfiguration("enable_gps")
    enable_slam = LaunchConfiguration("enable_slam")
    enable_saved_map_localization = LaunchConfiguration("enable_saved_map_localization")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    use_rviz = LaunchConfiguration("use_rviz")

    default_map_yaml = os.path.join(localization_share, "maps", "arena_map.yaml")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "real_robot.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_mock_firmware": use_mock_firmware,
            "use_lidar": "true",
            "use_camera": "false",
            "use_rviz": "false",
        }.items(),
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_share, "launch", "sensors.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_mock_sensors": use_mock_sensors,
        }.items(),
        condition=IfCondition(enable_sensors),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "enable_laser_odometry": enable_laser_odometry,
            "fuse_laser_odometry": fuse_laser_odometry,
            "enable_gps": enable_gps,
            "enable_slam": enable_slam,
            "enable_saved_map_localization": enable_saved_map_localization,
            "map_yaml_file": map_yaml_file,
        }.items(),
        condition=IfCondition(enable_localization),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(enable_navigation),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "use_mock_firmware",
                default_value="true",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "use_mock_sensors",
                # Was "true" until 2026-08-13, which silently stacked mock /scan and
                # mock /imu/data on top of the real LD06. Kept as an argument only so
                # the guard below can give a useful message instead of an unknown-arg
                # error; any true value is rejected.
                default_value="false",
                description=(
                    "Must stay false here — this launch always runs the real LiDAR. "
                    "See _reject_mock_sensors()."
                ),
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "enable_sensors",
                default_value="true",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "enable_localization",
                default_value="true",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "enable_navigation",
                default_value="false",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "enable_laser_odometry",
                default_value="false",
                description="Laser scan matcher (needs a moving /scan; disable with static mocks).",
            ),
            # ADDED 2026-08-24. localization.launch.py has carried this switch since the
            # /laser/odom lock-up was found, but this file did not forward it, so from
            # here the EKF could only ever run WITHOUT the laser (ekf_without_laser,
            # odom1 overridden to ""). With the BNO085 absent that left /wheel/odom as
            # the EKF's single live source — and /wheel/odom is the topic measured on
            # 2026-08-21 reporting 1 % of the commanded linear velocity and a phantom
            # -0.2594 rad/s yaw. The capability existed and was unreachable.
            #
            # DEFAULT STAYS false, and that is not timidity: a scan matcher can lock up
            # SILENTLY (twin, 2026-08-21 — 2.47 m off with Nav2 reporting SUCCEEDED and
            # zero recoveries, because every Nav2 tolerance compares two quantities
            # carrying the same error). Turning this on is a decision taken per run,
            # with odom_divergence_monitor watched.
            #
            # PARTIAL MITIGATION ONLY: odom1 contributes x, y, vx, vy. Yaw and yaw rate
            # still come from odom0 (/wheel/odom), so a phantom yaw SURVIVES the fusion.
            DeclareLaunchArgument(
                "fuse_laser_odometry",
                default_value="false",
                description=(
                    "Let /laser/odom into the EKF. Independent of enable_laser_odometry, "
                    "which only runs the matcher and publishes the topic."
                ),
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "enable_gps",
                default_value="false",
                description="Fuse /gps/fix via navsat_transform_node.",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="false",
                description="Online mapping with slam_toolbox (publishes map→odom).",
            ),
            # DEFAULT FLIPPED true -> false, 2026-08-24. It could not have been working:
            # map_yaml_file defaults to <gripperx_localization>/maps/arena_map.yaml and
            # THAT FILE DOES NOT EXIST (the maps directory holds testworld_v1_map.* and a
            # README). So the previous default asked map_server to load a missing file on
            # every defaults-only launch, which means no caller can have been depending on
            # it. false also makes the default agree with the robot's own autostart, which
            # runs live SLAM (user decision 2026-08-24, gripperx-mapping.sh).
            DeclareLaunchArgument(
                "enable_saved_map_localization",
                default_value="false",
                description=(
                    "map_server + AMCL on map_yaml_file. Requires map_yaml_file to point at "
                    "a map that exists; mutually exclusive with enable_slam."
                ),
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value=default_map_yaml,
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                choices=["true", "True", "false", "False"],
            ),
            # Runs before anything is spawned, so a rejected combination aborts the
            # launch instead of bringing half the stack up first.
            OpaqueFunction(function=_reject_mock_sensors),
            robot,
            sensors,
            localization,
            navigation,
        ]
    )
