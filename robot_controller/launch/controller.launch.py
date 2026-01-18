
from  launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sime_time_arg",
        default_value= "True"
    )

    use_sim_time = LaunchConfiguration("use_sime_time_arg")
    

    joint_state_broadcaster_spawner = Node( 
        package= "controller_manager",
        executable= "spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    
    simple_controller = Node(
        package=  "controller_manager",
        executable="spawner",
        arguments=[
            "kid_steering_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]

    )

    skid_controller_node =Node (
        package="robot_controller",
        executable="robot_controller"
    )



    return LaunchDescription([
        use_sim_time_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        skid_controller_node

    ])