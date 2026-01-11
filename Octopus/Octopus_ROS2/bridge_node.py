#!/usr/bin/env python3
"""
ROS 2 Bridge Node - Connects SQLite database to ROS 2 ecosystem
Reads device positions and status from database and publishes to ROS topics
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster
import sqlite3
import json
from datetime import datetime
import math

# You'll replace these with your custom messages once built
# from octopus_fleet.msg import DevicePosition, DeviceStatus, FleetStatus

DB_PATH = "octopusfinal.db"

class OctopusBridgeNode(Node):
    def __init__(self):
        super().__init__('octopus_bridge')
        
        self.get_logger().info('🤖 Octopus Fleet Bridge Node Starting...')
        
        # Publishers for each device
        self.device_publishers = {}
        self.battery_publishers = {}
        
        # TF broadcaster for coordinate frames
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Fleet status publisher
        self.fleet_status_pub = self.create_publisher(
            String,  # Will be FleetStatus when custom msg is built
            '/octopus/fleet_status',
            10
        )
        
        # Store laptop position (origin)
        self.laptop_position = None
        
        # Timer to poll database
        self.create_timer(0.5, self.publish_device_data)  # 2 Hz
        
        self.get_logger().info('✅ Bridge node initialized')
    
    def query_db(self, query, args=()):
        """Query SQLite database"""
        try:
            conn = sqlite3.connect(DB_PATH)
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()
            cur.execute(query, args)
            rows = [dict(r) for r in cur.fetchall()]
            conn.close()
            return rows
        except Exception as e:
            self.get_logger().error(f'Database error: {e}')
            return []
    
    def publish_device_data(self):
        """Read from database and publish to ROS topics"""
        # Get all device locations
        locations = self.query_db(
            "SELECT origin_id, type, latitude, longitude, altitude, timestamp FROM locations"
        )
        
        # Get battery data
        battery_data = self.query_db(
            "SELECT device_id, battery_percent, state FROM battery"
        )
        battery_dict = {b['device_id']: b for b in battery_data}
        
        # Find laptop position first
        laptop = next((loc for loc in locations if loc['type'] == 'laptop'), None)
        if laptop:
            self.laptop_position = laptop
        
        # Publish data for each device
        for loc in locations:
            device_id = loc['origin_id']
            device_type = loc['type']
            
            # Create publisher if it doesn't exist
            if device_id not in self.device_publishers:
                self.device_publishers[device_id] = self.create_publisher(
                    PoseStamped,
                    f'/octopus/devices/{device_id}/pose',
                    10
                )
                self.battery_publishers[device_id] = self.create_publisher(
                    BatteryState,
                    f'/octopus/devices/{device_id}/battery',
                    10
                )
                self.get_logger().info(f'Created publishers for {device_id}')
            
            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            # Convert to local coordinates if laptop position is known
            if self.laptop_position and device_type != 'laptop':
                x, y = self.gps_to_local(
                    loc['latitude'], loc['longitude'],
                    self.laptop_position['latitude'], self.laptop_position['longitude']
                )
            else:
                x, y = 0.0, 0.0
            
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = loc['altitude'] if loc['altitude'] else 0.0
            pose_msg.pose.orientation.w = 1.0  # No rotation for now
            
            self.device_publishers[device_id].publish(pose_msg)
            
            # Publish TF transform
            self.publish_tf(device_id, x, y, loc['altitude'] or 0.0)
            
            # Publish battery status
            if device_id in battery_dict:
                battery_msg = BatteryState()
                battery_msg.header.stamp = self.get_clock().now().to_msg()
                battery_msg.header.frame_id = device_id
                battery_msg.percentage = battery_dict[device_id]['battery_percent'] / 100.0
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                
                state = battery_dict[device_id]['state']
                if state == 'charging':
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                elif state == 'idle':
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
                
                self.battery_publishers[device_id].publish(battery_msg)
        
        # Publish fleet status
        self.publish_fleet_status(len(locations), battery_dict)
    
    def publish_tf(self, device_id, x, y, z):
        """Publish TF transform for device"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = device_id
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_fleet_status(self, device_count, battery_dict):
        """Publish overall fleet status"""
        # Count device types
        robots = sum(1 for d in battery_dict.values() if d.get('state') == 'active')
        
        # For now, publish as JSON string (will use custom message later)
        status = {
            'total_devices': device_count,
            'active_devices': robots,
            'timestamp': datetime.now().isoformat()
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.fleet_status_pub.publish(msg)
    
    def gps_to_local(self, lat, lon, origin_lat, origin_lon):
        """Convert GPS to local coordinates"""
        R = 6371000  # Earth radius in meters
        lat_factor = math.radians(1) * R
        lon_factor = math.radians(1) * R * math.cos(math.radians(origin_lat))
        
        dlat = lat - origin_lat
        dlon = lon - origin_lon
        
        y = dlat * lat_factor
        x = dlon * lon_factor
        
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = OctopusBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()