import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class VideoPublisher(Node):
    """Replays a video file onto the camera topic for testing.

    Publishes the same ``camera/image_raw/compressed`` JPEG stream as the real
    camera node, so downstream nodes (e.g. the detector) cannot tell the
    difference. Handy when the live camera sees nothing worth detecting.
    """

    def __init__(self):
        super().__init__('video_publisher')

        self.declare_parameter('video_path', '')
        self.declare_parameter('frame_rate', 0.0)   # 0 -> use the video's native fps
        self.declare_parameter('loop', True)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('topic', 'camera/image_raw/compressed')

        video_path = self.get_parameter('video_path').value
        self.loop = self.get_parameter('loop').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        topic = self.get_parameter('topic').value

        if not video_path:
            self.get_logger().error("Set the 'video_path' parameter to a video file.")
            raise RuntimeError('video_path not provided')

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video file: {video_path}')
            raise RuntimeError('video not available')

        frame_rate = self.get_parameter('frame_rate').value
        if frame_rate <= 0.0:
            frame_rate = self.cap.get(cv2.CAP_PROP_FPS) or 30.0

        self.publisher = self.create_publisher(
            CompressedImage, topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.timer = self.create_timer(1.0 / frame_rate, self.publish_frame)

        self.get_logger().info(
            f"Replaying '{video_path}' on '{topic}' @ {frame_rate:.1f} fps "
            f"(loop={self.loop})."
        )

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # rewind and keep going
                return
            self.get_logger().info('End of video reached, stopping.')
            self.timer.cancel()
            return

        _, jpeg_buf = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.format = 'jpeg'
        msg.data = np.array(jpeg_buf).tobytes()
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
