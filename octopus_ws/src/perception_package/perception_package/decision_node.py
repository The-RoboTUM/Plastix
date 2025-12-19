#!/usr/bin/env python3
"""
Decision Node - Decides which robot should handle trash
Template: Eve team adds implementation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        
        # Subscriber: Receive trash kind
        self.trash_sub = self.create_subscription(
            String,
            '/perception/trash_kind',
            self.trash_callback,
            10
        )
        
        # Subscriber: Receive surface type
        self.surface_sub = self.create_subscription(
            String,
            '/perception/surface',
            self.surface_callback,
            10
        )
        
        # Publisher: Publish robot decision
        self.decision_pub = self.create_publisher(
            String,
            '/perception/robot_decision',
            10
        )
        
        self.get_logger().info('Decision Node started - Hello World!')

    def trash_callback(self, msg):
        self.get_logger().info(f'Received trash kind: {msg.data}')
        # TODO: Add your implementation

    def surface_callback(self, msg):
        self.get_logger().info(f'Received surface: {msg.data}')
        # TODO: Add your implementation


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

