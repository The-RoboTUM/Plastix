#!/usr/bin/env python3
"""
Multi-Team ROS 2 Bridge Node
Publishes your team's data and subscribes to other teams' data
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import sqlite3
import json
from datetime import datetime
import math
import os

DB_PATH = "octopusfinal.db"
TEAM_ID = os.getenv('TEAM_ID', 'team_garching')  # Your team identifier

class MultiTeamBridgeNode(Node):
    def __init__(self):
        super().__init__(f'{TEAM_ID}_bridge')
        
        self.get_logger().info(f'🤖 Multi-Team Bridge Starting for {TEAM_ID}...')
        
        # Publishers for YOUR team's devices
        self.device_publishers = {}
        self.battery_publishers = {}
        
        # Subscribers for OTHER teams' devices
        self.other_teams_data = {}
        
        # TF broadcaster
        from tf2_ros import TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Fleet status publisher for YOUR team
        self.fleet_status_pub = self.create_publisher(
            String,
            f'/{TEAM_ID}/fleet_status',
            10
        )
        
        # Position the local frame is anchored on. Eve (the drone) is that point:
        # the collector robot is started on her spot, so the whole Octopus stack
        # calls it map (0, 0) - see docs/octopus_to_robot_interface.md. The laptop
        # is only the fallback for a database that has no drone row yet.
        self.datum_position = None
        
        # Timer to publish YOUR team's data
        self.create_timer(0.5, self.publish_own_team_data)
        
        # Timer to discover and subscribe to other teams
        self.create_timer(5.0, self.discover_other_teams)
        
        # Timer to aggregate all teams' data
        self.create_timer(2.0, self.publish_aggregated_data)
        
        self.get_logger().info(f'✅ {TEAM_ID} bridge initialized')
    
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
    
    def publish_own_team_data(self):
        """Publish YOUR team's robot data"""
        locations = self.query_db(
            "SELECT origin_id, type, latitude, longitude, altitude, timestamp FROM locations"
        )
        
        battery_data = self.query_db(
            "SELECT device_id, battery_percent, state FROM battery"
        )
        battery_dict = {b['device_id']: b for b in battery_data}
        
        # Find the datum first: Eve if she is in the table, else the laptop.
        datum = self.find_datum(locations)
        if datum:
            self.datum_position = datum
        
        # Publish each device
        for loc in locations:
            device_id = loc['origin_id']
            
            # Create publishers with TEAM namespace
            if device_id not in self.device_publishers:
                self.device_publishers[device_id] = self.create_publisher(
                    PoseStamped,
                    f'/{TEAM_ID}/devices/{device_id}/pose',
                    10
                )
                self.battery_publishers[device_id] = self.create_publisher(
                    BatteryState,
                    f'/{TEAM_ID}/devices/{device_id}/battery',
                    10
                )
            
            # Local coordinates relative to the datum. The datum device itself is
            # 0, 0 - it is what the frame is anchored on, so it cannot report a
            # position within its own frame.
            x, y = self.local_position(loc)
            
            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = f'{TEAM_ID}_map'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = loc['altitude'] if loc['altitude'] else 0.0
            pose_msg.pose.orientation.w = 1.0
            
            self.device_publishers[device_id].publish(pose_msg)
            
            # Publish battery
            if device_id in battery_dict:
                battery_msg = BatteryState()
                battery_msg.header.stamp = self.get_clock().now().to_msg()
                battery_msg.header.frame_id = f'{TEAM_ID}_{device_id}'
                battery_msg.percentage = battery_dict[device_id]['battery_percent'] / 100.0
                
                state = battery_dict[device_id]['state']
                if state == 'charging':
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                elif state == 'idle':
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
                else:
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                
                self.battery_publishers[device_id].publish(battery_msg)
        
        # Publish fleet status
        status = {
            'team_id': TEAM_ID,
            'device_count': len(locations),
            'timestamp': datetime.now().isoformat()
        }
        msg = String()
        msg.data = json.dumps(status)
        self.fleet_status_pub.publish(msg)
    
    def discover_other_teams(self):
        """Discover and subscribe to other teams' topics"""
        topics = self.get_topic_names_and_types()
        
        for topic, msg_types in topics:
            # Look for other teams' device poses
            if '/devices/' in topic and '/pose' in topic and TEAM_ID not in topic:
                # Extract team_id from topic
                parts = topic.split('/')
                if len(parts) >= 3:
                    other_team = parts[1]
                    
                    if topic not in self.other_teams_data:
                        self.get_logger().info(f'🔍 Discovered {other_team} - subscribing to {topic}')
                        
                        # Subscribe to other team's devices
                        self.create_subscription(
                            PoseStamped,
                            topic,
                            lambda msg, t=topic: self.other_team_callback(msg, t),
                            10
                        )
                        self.other_teams_data[topic] = {'last_msg': None, 'team': other_team}
    
    def other_team_callback(self, msg, topic):
        """Receive data from other teams"""
        if topic in self.other_teams_data:
            self.other_teams_data[topic]['last_msg'] = msg
    
    def publish_aggregated_data(self):
        """Publish aggregated data from all teams"""
        if not self.other_teams_data:
            return
        
        # Create aggregated view
        all_teams = {TEAM_ID: {'device_count': len(self.device_publishers)}}
        
        for topic, data in self.other_teams_data.items():
            team = data['team']
            if team not in all_teams:
                all_teams[team] = {'device_count': 0}
            all_teams[team]['device_count'] += 1
        
        self.get_logger().info(
            f'📊 Multi-Team Status: {len(all_teams)} teams, '
            f'{sum(t["device_count"] for t in all_teams.values())} total devices'
        )
    
    @staticmethod
    def is_datum_device(loc):
        """Eve, whose position defines local (0, 0)."""
        return (str(loc.get('type') or '').lower() == 'drone'
                or 'eve' in str(loc.get('origin_id') or '').lower())

    def find_datum(self, locations):
        """Eve if the table knows her, otherwise the laptop as a stand-in."""
        return (next((loc for loc in locations if self.is_datum_device(loc)), None)
                or next((loc for loc in locations if loc.get('type') == 'laptop'), None))

    def local_position(self, loc):
        """Metres east/north of the datum. The datum itself is always (0, 0)."""
        datum = self.datum_position
        if not datum or loc.get('origin_id') == datum.get('origin_id'):
            return 0.0, 0.0
        return self.gps_to_local(
            loc['latitude'], loc['longitude'],
            datum['latitude'], datum['longitude'],
        )

    def gps_to_local(self, lat, lon, origin_lat, origin_lon):
        """Convert GPS to local coordinates"""
        R = 6371000
        lat_factor = math.radians(1) * R
        lon_factor = math.radians(1) * R * math.cos(math.radians(origin_lat))
        
        dlat = lat - origin_lat
        dlon = lon - origin_lon
        
        y = dlat * lat_factor
        x = dlon * lon_factor
        
        return x, y


def main(args=None):
    rclpy.init(args=args)
    
    team_id = os.getenv('TEAM_ID', 'team_garching')
    print(f"\n🌐 Starting Multi-Team Bridge for {team_id}")
    print(f"Domain ID: {os.getenv('ROS_DOMAIN_ID', 'default')}")
    print("Discovering other teams...\n")
    
    node = MultiTeamBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()