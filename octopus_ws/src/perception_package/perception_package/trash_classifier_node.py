#!/usr/bin/env python3
"""
Trash Classifier Node - Determines trash kind from detections
Template: Eve team adds implementation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TrashClassifierNode(Node):
    def __init__(self):
        super().__init__('trash_classifier_node')
        
        # Subscriber: Receive detection results
        self.detection_sub = self.create_subscription(
            String,
            '/drone/image/detection',
            self.detection_callback,
            10
        )
        
        # Publisher: Publish trash kind
        self.trash_pub = self.create_publisher(
            String,
            '/perception/trash_kind',
            10
        )
        
        self.get_logger().info('Trash Classifier Node started - Hello World!')

    def detection_callback(self, msg):
        self.get_logger().info(f'Received detection: {msg.data}')
        # TODO: Add your implementation
        
        result = String()
        result.data = 'trash_kind'
        self.trash_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = TrashClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

