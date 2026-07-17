import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import SetUseSimTime


def generate_launch_description():
    gripperx_gazebo = get_package_share_directory("gripperx_gazebo")
    gripperx_description = get_package_share_directory("gripperx_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_file = LaunchConfiguration("world_file")
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    ros_lib = os.path.join(f"/opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}", "lib")

    # DT-4: headless (default true, unchanged behavior) controls "-s"
    # (server-only, no Gazebo window). headless:=false omits "-s" ->
    # gz sim starts server+GUI in one process (sim visible to the user).
    gz_args = PythonExpression(
        ["'-s -r ' if '", headless, "' == 'true' else '-r '"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [gz_args, world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    simulate_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_gazebo, "launch", "simulate_robot.launch.py")
        ),
        launch_arguments={
            "use_lidar": use_lidar,
            "use_camera": use_camera,
            # GAP-1 (DT-2): forward teleop_mux start mode so the integrated
            # autonomy bringup can start the mux in "autonomous" mode (Nav2 runs).
            "initial_mode": LaunchConfiguration("initial_mode"),
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
            os.path.join(gripperx_description, "launch", "rviz.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            SetUseSimTime(True),
            SetEnvironmentVariable(name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=ros_lib),
            DeclareLaunchArgument(
                # DT-4: mapping_arena.world.sdf was dropped as inaccurate (user
                # decision); testworld_v1 is the new baseline for SLAM/Nav2 tuning.
                # mapping_arena.world.sdf was deleted 2026-07-15 (unreferenced,
                # superseded by testworld_v1.world.sdf).
                "world_file",
                default_value=os.path.join(gripperx_gazebo, "worlds", "testworld_v1.world.sdf"),
                description="Gazebo world file.",
            ),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument(
                "initial_mode",
                default_value="keyboard",
                description="teleop_mux start mode: keyboard | controller | autonomous.",
                choices=["keyboard", "controller", "autonomous"],
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description=(
                    "true (default): gz sim -s -r, no Gazebo window. "
                    "false: gz sim -r without -s, starts Gazebo WITH a GUI window "
                    "(needs a desktop session/DISPLAY)."
                ),
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.0",
                description=(
                    "Must lie within testworld_v1 (x/y roughly -5.5..5.5). "
                    "Default: middle of the north-south corridor, southern end."
                ),
            ),
            DeclareLaunchArgument("spawn_y", default_value="-5.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="1.5708",
                description="Facing north (into the corridor).",
            ),
            gazebo,
            simulate_robot,
            rviz,
        ]
    )
