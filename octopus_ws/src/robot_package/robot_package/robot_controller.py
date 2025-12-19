#!/usr/bin/env python3
"""
Robot Controller Template
Usage: ros2 run robot_package robot_controller --ros-args -p robot_name:=eve
       ros2 run robot_package robot_controller --ros-args -p robot_name:=robby
       ros2 run robot_package robot_controller --ros-args -p robot_name:=gripperx
       ros2 run robot_package robot_controller --ros-args -p robot_name:=sharx

Template: Each team should add their own implementation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from octopus_msgs.msg import RobotStatus
from octopus_msgs.srv import GetRobotStatus


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Parameter: robot_name (eve, robby, gripperx, sharx)
        self.declare_parameter('robot_name', 'robot')
        self.robot_name = self.get_parameter('robot_name').value
        
        # Publisher: Send status to brain
        self.status_pub = self.create_publisher(
            RobotStatus,
            '/octopus/robot_status',
            10
        )
        
        # Subscriber: Receive commands from brain
        self.command_sub = self.create_subscription(
            String,
            '/octopus/commands',
            self.command_callback,
            10
        )
        
        # Service: Provide robot status
        self.status_srv = self.create_service(
            GetRobotStatus,
            f'/{self.robot_name}/get_status',
            self.get_status_callback
        )
        
        # Timer: Publish status
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(f'Robot Controller started: {self.robot_name} - Hello World!')

    def command_callback(self, msg):
        self.get_logger().info(f'[{self.robot_name}] Received command: {msg.data}')
        # TODO: Add your implementation

    def get_status_callback(self, request, response):
        self.get_logger().info(f'[{self.robot_name}] Status requested')
        # TODO: Add your implementation
        response.status.robot_name = self.robot_name
        response.status.battery_level = 100.0
        response.status.is_available = True
        response.success = True
        return response

    def timer_callback(self):
        msg = RobotStatus()
        msg.robot_name = self.robot_name
        msg.battery_level = 100.0
        msg.is_available = True
        msg.current_task = ''
        self.status_pub.publish(msg)
        # TODO: Add your implementation


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

