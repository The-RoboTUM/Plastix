import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        device_index = self.get_parameter('device_index').value
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value

        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, frame_rate)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera at device index {device_index}')
            raise RuntimeError('Camera not available')

        self.get_logger().info(
            f'Camera opened at /dev/video{device_index} ({int(width)}x{int(height)} @ {frame_rate} fps)'
        )

        self.timer = self.create_timer(1.0 / frame_rate, self.publish_frame)
        
        # Setup image storage directory
        self.storage_dir = '/tmp/camera_images'
        os.makedirs(self.storage_dir, exist_ok=True)
        self.frame_count = 0

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.publisher.publish(msg)
        
        # Save image every 10 frames
        if self.frame_count % 10 == 0:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            cv2.imwrite(f'{self.storage_dir}/frame_{timestamp}.jpg', frame)
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
