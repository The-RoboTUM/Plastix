import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gripperx_description = get_package_share_directory("gripperx_description")
    gripperx_control = get_package_share_directory("gripperx_control")

    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gripperx_description, "launch", "load_urdf.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_lidar": use_lidar,
            "use_camera": use_camera,
            "urdf_file": os.path.join(gripperx_description, "urdf", "gripperx_v1.gazebo.xacro"),
            "controllers_file": os.path.join(gripperx_control, "config", "ros2_controllers.yaml"),
            "mesh_dir": f"file://{os.path.join(gripperx_description, 'meshes')}",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        parameters=[
            {
                "topic": "/robot_description",
                "name": "bot",
                "allow_renaming": False,
                "x": spawn_x,
                "y": spawn_y,
                "z": spawn_z,
                "R": spawn_roll,
                "P": spawn_pitch,
                "Y": spawn_yaw,
            }
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "steering_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            load_urdf,
            spawn_robot,
            RegisterEventHandler(
                event_handler=OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[steering_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[wheel_controller],
                )
            ),
        ]
    )
