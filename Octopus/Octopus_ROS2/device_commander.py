#!/usr/bin/env python3
"""
Device Commander Node - Sends commands to devices via ROS 2
Can be controlled via command line or web dashboard
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import sqlite3
import json
from datetime import datetime

DB_PATH = "octopusfinal.db"

class DeviceCommanderNode(Node):
    def __init__(self):
        super().__init__('device_commander')
        
        self.get_logger().info('📡 Device Commander Node Starting...')
        
        # Command subscriber - receives commands from dashboard/CLI
        self.command_sub = self.create_subscription(
            String,
            '/octopus/commands',
            self.command_callback,
            10
        )
        
        # Goal publishers for each device
        self.goal_publishers = {}
        
        # Service for sending navigation goals
        # Will add action server for long-running tasks later
        
        self.get_logger().info('✅ Commander node ready for commands')
    
    def command_callback(self, msg):
        """Process incoming commands"""
        try:
            cmd = json.loads(msg.data)
            cmd_type = cmd.get('type')
            device_id = cmd.get('device_id')
            
            self.get_logger().info(f'Received command: {cmd_type} for {device_id}')
            
            if cmd_type == 'goto':
                self.send_goto_command(device_id, cmd.get('x'), cmd.get('y'), cmd.get('z', 0.0))
            elif cmd_type == 'return_home':
                self.send_return_home(device_id)
            elif cmd_type == 'stop':
                self.send_stop_command(device_id)
            else:
                self.get_logger().warn(f'Unknown command type: {cmd_type}')
                
        except Exception as e:
            self.get_logger().error(f'Command processing error: {e}')
    
    def send_goto_command(self, device_id, x, y, z):
        """Send navigation goal to device"""
        # Create publisher if doesn't exist
        if device_id not in self.goal_publishers:
            self.goal_publishers[device_id] = self.create_publisher(
                PoseStamped,
                f'/octopus/devices/{device_id}/goal',
                10
            )
        
        # Create goal message
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        
        self.goal_publishers[device_id].publish(goal)
        self.get_logger().info(f'Sent goto command to {device_id}: ({x}, {y}, {z})')
        
        # Also write to database for web dashboard
        self.write_command_to_db(device_id, 'goto', {'x': x, 'y': y, 'z': z})
    
    def send_return_home(self, device_id):
        """Send return to base command"""
        # Home is at (0, 0) in local coordinates
        self.send_goto_command(device_id, 0.0, 0.0, 0.0)
        self.get_logger().info(f'{device_id} returning to base')
    
    def send_stop_command(self, device_id):
        """Emergency stop command"""
        stop_pub = self.create_publisher(
            String,
            f'/octopus/devices/{device_id}/emergency_stop',
            10
        )
        
        msg = String()
        msg.data = 'STOP'
        stop_pub.publish(msg)
        
        self.get_logger().warn(f'Emergency stop sent to {device_id}')
    
    def write_command_to_db(self, device_id, cmd_type, params):
        """Write command to database for tracking"""
        try:
            conn = sqlite3.connect(DB_PATH)
            cur = conn.cursor()
            
            # You might want to create a commands table
            cur.execute("""
                CREATE TABLE IF NOT EXISTS commands (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    device_id TEXT NOT NULL,
                    command_type TEXT NOT NULL,
                    parameters TEXT,
                    timestamp TEXT NOT NULL,
                    status TEXT DEFAULT 'sent'
                )
            """)
            
            cur.execute("""
                INSERT INTO commands (device_id, command_type, parameters, timestamp)
                VALUES (?, ?, ?, ?)
            """, (device_id, cmd_type, json.dumps(params), datetime.utcnow().isoformat()))
            
            conn.commit()
            conn.close()
        except Exception as e:
            self.get_logger().error(f'Database write error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DeviceCommanderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()