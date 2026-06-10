import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('verbose', False)
        self.declare_parameter('publish_raw', False)

        device_index = self.get_parameter('device_index').value
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.verbose = self.get_parameter('verbose').value
        self.publish_raw = self.get_parameter('publish_raw').value

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.raw_publisher = self.create_publisher(Image, 'camera/image_raw', qos)
        self.compressed_publisher = self.create_publisher(
            CompressedImage, 'camera/image_raw/compressed', qos
        )
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, frame_rate)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at device index {device_index}')
            raise RuntimeError('Camera not available')

        topics = 'camera/image_raw and camera/image_raw/compressed' if self.publish_raw else 'camera/image_raw/compressed only'
        self.get_logger().info(
            f'Camera opened at /dev/video{device_index} ({int(width)}x{int(height)} @ {frame_rate} fps)'
        )
        self.get_logger().info(f'Publishing on {topics}')

        self.timer = self.create_timer(1.0 / frame_rate, self.publish_frame)

        self.storage_dir = '/tmp/camera_images'
        os.makedirs(self.storage_dir, exist_ok=True)
        self.frame_count = 0

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            if self.verbose:
                self.get_logger().warn('Failed to capture frame')
            return

        stamp = self.get_clock().now().to_msg()

        if self.publish_raw:
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = 'camera_frame'
            self.raw_publisher.publish(raw_msg)

        _, jpeg_buf = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = stamp
        compressed_msg.header.frame_id = 'camera_frame'
        compressed_msg.format = 'jpeg'
        compressed_msg.data = np.array(jpeg_buf).tobytes()
        self.compressed_publisher.publish(compressed_msg)

        if self.frame_count % 10 == 0:
            path = os.path.join(self.storage_dir, f'frame_{self.frame_count:06d}.jpg')
            cv2.imwrite(path, frame)
        self.frame_count += 1

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
