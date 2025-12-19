#!/usr/bin/env python3
"""
Surface Detection Node - Detects surface type from images
Template: Eve team adds implementation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SurfaceDetectionNode(Node):
    def __init__(self):
        super().__init__('surface_detection_node')
        
        # Subscriber: Receive detection results
        self.detection_sub = self.create_subscription(
            String,
            '/drone/image/detection',
            self.detection_callback,
            10
        )
        
        # Publisher: Publish surface type
        self.surface_pub = self.create_publisher(
            String,
            '/perception/surface',
            10
        )
        
        self.get_logger().info('Surface Detection Node started - Hello World!')

    def detection_callback(self, msg):
        self.get_logger().info(f'Received detection: {msg.data}')
        # TODO: Add your implementation
        
        result = String()
        result.data = 'surface_type'
        self.surface_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

