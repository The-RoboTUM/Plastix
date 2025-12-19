#!/usr/bin/env python3
"""
Octopus Main Node - Central brain coordinator
Template: Add your implementation here
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from octopus_msgs.msg import RobotStatus
from octopus_msgs.srv import GetRobotStatus


class OctopusMainNode(Node):
    def __init__(self):
        super().__init__('octopus_main_node')
        
        # Publisher: Send commands to robots
        self.command_pub = self.create_publisher(
            String,
            '/octopus/commands',
            10
        )
        
        # Subscriber: Receive status from robots
        self.status_sub = self.create_subscription(
            RobotStatus,
            '/octopus/robot_status',
            self.robot_status_callback,
            10
        )
        
        # Service: Get robot status
        self.status_srv = self.create_service(
            GetRobotStatus,
            '/octopus/get_robot_status',
            self.get_robot_status_callback
        )
        
        # Timer: Heartbeat
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info('Octopus Main Node started - Hello World!')

    def robot_status_callback(self, msg):
        self.get_logger().info(f'Received status from: {msg.robot_name}')
        # TODO: Add your implementation

    def get_robot_status_callback(self, request, response):
        self.get_logger().info(f'Status requested for: {request.robot_name}')
        # TODO: Add your implementation
        response.success = True
        return response

    def timer_callback(self):
        msg = String()
        msg.data = 'Octopus brain heartbeat'
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OctopusMainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

