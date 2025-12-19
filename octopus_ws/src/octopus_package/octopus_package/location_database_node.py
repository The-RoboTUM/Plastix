#!/usr/bin/env python3
"""
Location Database Node - Stores location data from RTK GPS
Template: Add your implementation here
"""

import rclpy
from rclpy.node import Node
from octopus_msgs.msg import Location
from octopus_msgs.srv import LocationQuery


class LocationDatabaseNode(Node):
    def __init__(self):
        super().__init__('location_database_node')
        
        # Subscriber: Receive RTK GPS data
        self.location_sub = self.create_subscription(
            Location,
            '/rtk_gps/locations',
            self.location_callback,
            10
        )
        
        # Service: Query locations
        self.query_srv = self.create_service(
            LocationQuery,
            '/location_db/query',
            self.query_callback
        )
        
        self.get_logger().info('Location Database Node started - Hello World!')

    def location_callback(self, msg):
        self.get_logger().info(f'Received location for: {msg.robot_name}')
        # TODO: Add your implementation

    def query_callback(self, request, response):
        self.get_logger().info(f'Location query: radius={request.radius}, lat={request.lat}, lon={request.lon}')
        # TODO: Add your implementation
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocationDatabaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

