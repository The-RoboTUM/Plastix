#!/usr/bin/env python3
"""
RTK GPS Node - Publishes location corrections
Template: Add your implementation here
"""

import rclpy
from rclpy.node import Node
from octopus_msgs.msg import Location


class RTKGPSNode(Node):
    def __init__(self):
        super().__init__('rtk_gps_node')
        
        # Publisher: Broadcast GPS locations
        self.location_pub = self.create_publisher(
            Location,
            '/rtk_gps/locations',
            10
        )
        
        # Timer: Simulate GPS updates
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('RTK GPS Node started - Hello World!')

    def timer_callback(self):
        msg = Location()
        msg.robot_name = 'test'
        msg.latitude = 0.0
        msg.longitude = 0.0
        msg.altitude = 0.0
        msg.timestamp = self.get_clock().now().to_msg()
        self.location_pub.publish(msg)
        # TODO: Add your implementation


def main(args=None):
    rclpy.init(args=args)
    node = RTKGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

