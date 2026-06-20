import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class WasteFollower(Node):

    def __init__(self):
        super().__init__('waste_follower')

        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('turn_speed', 0.22)
        self.declare_parameter('center_tolerance', 0.12)
        self.declare_parameter('stop_area_ratio', 0.20)
        self.declare_parameter('detection_timeout', 0.7)

        self.forward_speed = float(
            self.get_parameter('forward_speed').value
        )
        self.turn_speed = float(
            self.get_parameter('turn_speed').value
        )
        self.center_tolerance = float(
            self.get_parameter('center_tolerance').value
        )
        self.stop_area_ratio = float(
            self.get_parameter('stop_area_ratio').value
        )
        self.detection_timeout = float(
            self.get_parameter('detection_timeout').value
        )

        self.last_detection_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            String,
            '/sharx/waste_detection',
            self.detection_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/sharx/cmd_vel_auto',
            10
        )

        self.watchdog_timer = self.create_timer(
            0.1,
            self.watchdog_callback
        )

        self.get_logger().info('Waste follower started')
        self.get_logger().info(
            'Publishing autonomous commands to /sharx/cmd_vel_auto'
        )

    def publish_stop(self):
        command = Twist()
        self.publisher.publish(command)

    def detection_callback(self, message):
        try:
            detection = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Invalid detection JSON')
            self.publish_stop()
            return

        self.last_detection_time = self.get_clock().now()

        if not detection.get('detected', False):
            self.publish_stop()
            return

        image_width = float(detection.get('image_width', 0))
        image_height = float(detection.get('image_height', 0))
        center_x = float(detection.get('center_x', 0))
        box_width = float(detection.get('box_width', 0))
        box_height = float(detection.get('box_height', 0))

        if image_width <= 0 or image_height <= 0:
            self.publish_stop()
            return

        image_center_x = image_width / 2.0

        horizontal_error = (
            center_x - image_center_x
        ) / image_center_x

        box_area = box_width * box_height
        image_area = image_width * image_height
        area_ratio = box_area / image_area

        command = Twist()

        if area_ratio >= self.stop_area_ratio:
            action = 'target_close_stop'

        elif horizontal_error < -self.center_tolerance:
            command.angular.z = self.turn_speed
            action = 'turn_left'

        elif horizontal_error > self.center_tolerance:
            command.angular.z = -self.turn_speed
            action = 'turn_right'

        else:
            command.linear.x = self.forward_speed
            action = 'move_forward'

        self.publisher.publish(command)

        self.get_logger().info(
            f'action={action} '
            f'error={horizontal_error:.2f} '
            f'area_ratio={area_ratio:.3f}'
        )

    def watchdog_callback(self):
        elapsed = (
            self.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9

        if elapsed > self.detection_timeout:
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)

    node = WasteFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
