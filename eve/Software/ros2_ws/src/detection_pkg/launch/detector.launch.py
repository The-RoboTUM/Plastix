from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    repo = DeclareLaunchArgument(
        'detect_localize_path',
        default_value='/home/victor-tipkemper/projects/robotics/PlastiX/eve/Software/detect-and-localize',
        description='Path to the detect-and-localize repo (holds the shared pipeline + models).',
    )
    tags = DeclareLaunchArgument(
        'tags',
        default_value='',
        description='AprilTag CSV (relative to the repo). Empty -> normalized image coordinates.',
    )
    input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='camera/image_raw/compressed',
        description='CompressedImage topic to subscribe to.',
    )

    detector = Node(
        package='detection_pkg',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{
            'detect_localize_path': LaunchConfiguration('detect_localize_path'),
            'tags': LaunchConfiguration('tags'),
            'input_topic': LaunchConfiguration('input_topic'),
        }],
    )

    return LaunchDescription([repo, tags, input_topic, detector])
