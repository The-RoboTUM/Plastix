import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetUseSimTime


def generate_launch_description():
    bot_gazebo = get_package_share_directory("bot_gazebo")
    bot_description = get_package_share_directory("bot_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_file = LaunchConfiguration("world_file")
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    use_rviz = LaunchConfiguration("use_rviz")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    ros_lib = os.path.join(f"/opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}", "lib")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    simulate_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_gazebo, "launch", "simulate_robot.launch.py")
        ),
        launch_arguments={
            "use_lidar": use_lidar,
            "use_camera": use_camera,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_roll": spawn_roll,
            "spawn_pitch": spawn_pitch,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_description, "launch", "rviz.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            SetUseSimTime(True),
            SetEnvironmentVariable(name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=ros_lib),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(bot_gazebo, "worlds", "mapping_arena.world.sdf"),
                description="Gazebo world file.",
            ),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="-6.0",
                description="Must lie inside maps/arena_map.yaml (x roughly -2..14).",
            ),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            gazebo,
            simulate_robot,
            rviz,
        ]
    )
