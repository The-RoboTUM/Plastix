from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf_file = LaunchConfiguration("urdf_file")
    controllers_file = LaunchConfiguration("controllers_file")
    mesh_dir = LaunchConfiguration("mesh_dir")

    default_gazebo_xacro = PathJoinSubstitution(
        [FindPackageShare("bot_description"), "urdf", "bot_v1.gazebo.xacro"]
    )
    default_controllers = PathJoinSubstitution(
        [FindPackageShare("bot_control"), "config", "ros2_controllers.yaml"]
    )

    robot_description = Command(
        [
            "xacro ",
            urdf_file,
            " mesh_dir:=",
            mesh_dir,
            " controllers_file:=",
            controllers_file,
            " use_lidar:=",
            use_lidar,
            " use_camera:=",
            use_camera,
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Pass use_sim_time to robot_state_publisher.",
                choices=["true", "True", "false", "False"],
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="false",
                description="Include lidar links and Gazebo sensor when using gazebo xacro.",
            ),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="Include camera links and Gazebo sensor when using gazebo xacro.",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=default_gazebo_xacro,
                description="Path to the robot xacro file.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=default_controllers,
                description="ros2_control parameters file passed into the xacro.",
            ),
            DeclareLaunchArgument(
                "mesh_dir",
                default_value="package://bot_description/meshes",
                description="Mesh URI root passed to xacro (file://... for Gazebo).",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(robot_description, value_type=str),
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
