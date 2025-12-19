#!/usr/bin/env python3
"""
Task Database Node - Manages task storage
Template: Add your implementation here
"""

import rclpy
from rclpy.node import Node
from octopus_msgs.msg import TaskStatus
from octopus_msgs.srv import CreateTask


class TaskDatabaseNode(Node):
    def __init__(self):
        super().__init__('task_database_node')
        
        # Publisher: Broadcast active tasks
        self.task_pub = self.create_publisher(
            TaskStatus,
            '/task_db/active_tasks',
            10
        )
        
        # Service: Create task
        self.create_srv = self.create_service(
            CreateTask,
            '/task_db/create',
            self.create_task_callback
        )
        
        self.get_logger().info('Task Database Node started - Hello World!')

    def create_task_callback(self, request, response):
        self.get_logger().info(f'Creating task: {request.task_type} at ({request.location.x}, {request.location.y})')
        # TODO: Add your implementation
        response.task_id = 'TASK_001'
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskDatabaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

