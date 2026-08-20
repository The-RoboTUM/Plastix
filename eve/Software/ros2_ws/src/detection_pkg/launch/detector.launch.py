from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    debug_image_jpeg_quality = DeclareLaunchArgument(
        'debug_image_jpeg_quality',
        default_value='80',
        description='JPEG quality (1-100) of ~/debug_image/compressed, the frame the '
                    'dashboard shows. Raise it (92-98) when the feed looks blocky.',
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
            # A launch argument is a string; the node declares this one as an int, so
            # the type has to be stated explicitly or the node refuses to start.
            'debug_image_jpeg_quality': ParameterValue(
                LaunchConfiguration('debug_image_jpeg_quality'), value_type=int
            ),
        }],
    )

    return LaunchDescription([repo, tags, input_topic, debug_image_jpeg_quality, detector])
