#!/usr/bin/env python3
"""
Object Detection Node - Detects objects in drone images
Template: Eve team adds implementation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Subscriber: Receive drone images
        self.image_sub = self.create_subscription(
            Image,
            '/drone/image/raw',
            self.image_callback,
            10
        )
        
        # Publisher: Publish detection results
        self.detection_pub = self.create_publisher(
            String,
            '/drone/image/detection',
            10
        )
        
        self.get_logger().info('Object Detection Node started - Hello World!')

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        # TODO: Add your implementation
        
        result = String()
        result.data = 'detection_result'
        self.detection_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

