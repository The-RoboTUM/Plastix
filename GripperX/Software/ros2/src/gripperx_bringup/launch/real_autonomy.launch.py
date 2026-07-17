"""Full real-robot stack: hardware + sensors + localization + optional Nav2.

Modular toggles let you run the same pieces separately while developing.

Examples:
  # Base robot only (same as real_robot.launch.py)
  ros2 launch gripperx_bringup real_autonomy.launch.py \\
    enable_sensors:=false enable_localization:=false enable_navigation:=false

  # Mapping outdoors (SLAM, no saved map)
  ros2 launch gripperx_bringup real_autonomy.launch.py \\
    enable_slam:=true enable_saved_map_localization:=false enable_navigation:=false

  # Navigate on a saved map
  ros2 launch gripperx_bringup real_autonomy.launch.py \\
    enable_slam:=false enable_saved_map_localization:=true enable_navigation:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
                default_value="true",
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
            DeclareLaunchArgument(
                "enable_saved_map_localization",
                default_value="true",
                description="map_server + AMCL on map_yaml_file.",
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
            robot,
            sensors,
            localization,
            navigation,
        ]
    )
