#!/usr/bin/env python3
"""
Team Aggregator Node - Collects and displays data from ALL teams
Can be run by any team or a central coordinator
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import json
from datetime import datetime
from collections import defaultdict

class TeamAggregatorNode(Node):
    def __init__(self):
        super().__init__('team_aggregator')
        
        self.get_logger().info('🌐 Team Aggregator Starting...')
        self.get_logger().info('Discovering all teams on the network...')
        
        # Store data from all teams
        self.teams = defaultdict(lambda: {
            'devices': {},
            'last_update': None
        })
        
        # Timer to discover new topics
        self.create_timer(5.0, self.discover_teams)
        
        # Timer to print status
        self.create_timer(10.0, self.print_status)
        
        # Publisher for aggregated data
        self.aggregated_pub = self.create_publisher(
            String,
            '/all_teams/aggregated',
            10
        )
        
        self.get_logger().info('✅ Aggregator initialized')
    
    def discover_teams(self):
        """Discover all teams and their topics"""
        topics = self.get_topic_names_and_types()
        
        for topic, msg_types in topics:
            # Look for team device poses
            if '/devices/' in topic and '/pose' in topic:
                parts = topic.split('/')
                if len(parts) >= 4:
                    team_id = parts[1]
                    device_id = parts[3]
                    
                    topic_key = f"{team_id}_{device_id}_pose"
                    
                    # Subscribe if not already subscribed
                    if device_id not in self.teams[team_id]['devices']:
                        self.get_logger().info(f'📡 Discovered {team_id}/{device_id}')
                        
                        self.teams[team_id]['devices'][device_id] = {
                            'pose': None,
                            'battery': None
                        }
                        
                        # Subscribe to pose
                        self.create_subscription(
                            PoseStamped,
                            topic,
                            lambda msg, tid=team_id, did=device_id: self.pose_callback(msg, tid, did),
                            10
                        )
                        
                        # Subscribe to battery
                        battery_topic = f'/{team_id}/devices/{device_id}/battery'
                        self.create_subscription(
                            BatteryState,
                            battery_topic,
                            lambda msg, tid=team_id, did=device_id: self.battery_callback(msg, tid, did),
                            10
                        )
            
            # Subscribe to fleet status
            elif '/fleet_status' in topic and 'all_teams' not in topic:
                parts = topic.split('/')
                if len(parts) >= 2:
                    team_id = parts[1]
                    
                    if team_id not in self.teams:
                        self.get_logger().info(f'📊 Discovered team: {team_id}')
    
    def pose_callback(self, msg, team_id, device_id):
        """Receive pose from any team"""
        self.teams[team_id]['devices'][device_id]['pose'] = msg
        self.teams[team_id]['last_update'] = datetime.now()
    
    def battery_callback(self, msg, team_id, device_id):
        """Receive battery from any team"""
        self.teams[team_id]['devices'][device_id]['battery'] = msg
    
    def print_status(self):
        """Print status of all teams"""
        if not self.teams:
            self.get_logger().info('⏳ No teams discovered yet...')
            return
        
        print("\n" + "="*80)
        print("🌐 MULTI-TEAM STATUS DASHBOARD")
        print("="*80)
        
        total_devices = 0
        
        for team_id in sorted(self.teams.keys()):
            team_data = self.teams[team_id]
            device_count = len(team_data['devices'])
            total_devices += device_count
            
            print(f"\n📍 {team_id.upper()}: {device_count} devices")
            
            # Count active devices
            active = 0
            for device_id, device_data in team_data['devices'].items():
                if device_data['pose'] is not None:
                    pose = device_data['pose']
                    battery = device_data['battery']
                    
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = pose.pose.position.z
                    
                    battery_str = "N/A"
                    if battery:
                        battery_str = f"{battery.percentage*100:.0f}%"
                        active += 1
                    
                    print(f"   • {device_id:15} | Pos: ({x:6.1f}, {y:6.1f}, {z:5.1f})m | Battery: {battery_str}")
            
            # Team summary
            last_update = team_data['last_update']
            if last_update:
                age = (datetime.now() - last_update).total_seconds()
                print(f"   ⏱  Last update: {age:.1f}s ago | Active: {active}/{device_count}")
        
        print(f"\n📊 TOTAL: {len(self.teams)} teams, {total_devices} devices")
        print("="*80 + "\n")
        
        # Publish aggregated data
        self.publish_aggregated()
    
    def publish_aggregated(self):
        """Publish aggregated data for other systems"""
        aggregated = {
            'timestamp': datetime.now().isoformat(),
            'team_count': len(self.teams),
            'teams': {}
        }
        
        for team_id, team_data in self.teams.items():
            aggregated['teams'][team_id] = {
                'device_count': len(team_data['devices']),
                'devices': list(team_data['devices'].keys())
            }
        
        msg = String()
        msg.data = json.dumps(aggregated)
        self.aggregated_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    print("\n🌐 Team Aggregator - Monitoring All Teams")
    print("This node discovers and monitors all robot teams on the network\n")
    
    node = TeamAggregatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Aggregator stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()