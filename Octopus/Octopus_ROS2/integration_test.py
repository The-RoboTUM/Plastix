#!/usr/bin/env python3
"""
Test script to verify ROS 2 integration is working
Run this after starting your simulation and bridge node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
import sys
import time

class IntegrationTester(Node):
    def __init__(self):
        super().__init__('integration_tester')
        
        self.devices_found = set()
        self.poses_received = {}
        self.batteries_received = {}
        
        # Subscribe to all device topics using wildcard
        self.get_logger().info('🔍 Searching for Octopus devices...')
        
        # Give the system time to discover topics
        time.sleep(2)
        
        # Get list of available topics
        topics = self.get_topic_names_and_types()
        
        # Find all device pose topics
        for topic, _ in topics:
            if '/octopus/devices/' in topic and '/pose' in topic:
                device_id = topic.split('/')[3]
                self.devices_found.add(device_id)
                
                # Subscribe to pose
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, dev=device_id: self.pose_callback(msg, dev),
                    10
                )
                
                # Subscribe to battery
                battery_topic = f'/octopus/devices/{device_id}/battery'
                self.create_subscription(
                    BatteryState,
                    battery_topic,
                    lambda msg, dev=device_id: self.battery_callback(msg, dev),
                    10
                )
        
        if not self.devices_found:
            self.get_logger().error('❌ No devices found! Is the bridge node running?')
            sys.exit(1)
        
        self.get_logger().info(f'✅ Found {len(self.devices_found)} devices: {list(self.devices_found)}')
        
        # Timer to check results
        self.create_timer(5.0, self.print_results)
    
    def pose_callback(self, msg, device_id):
        """Received pose data"""
        self.poses_received[device_id] = msg
    
    def battery_callback(self, msg, device_id):
        """Received battery data"""
        self.batteries_received[device_id] = msg
    
    def print_results(self):
        """Print test results"""
        print("\n" + "="*60)
        print("ROS 2 INTEGRATION TEST RESULTS")
        print("="*60)
        
        print(f"\n📡 Devices Found: {len(self.devices_found)}")
        for device in sorted(self.devices_found):
            print(f"   - {device}")
        
        print(f"\n📍 Pose Messages Received: {len(self.poses_received)}")
        for device, pose in sorted(self.poses_received.items()):
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            print(f"   - {device:12}: ({x:7.2f}m, {y:7.2f}m, {z:7.2f}m)")
        
        print(f"\n🔋 Battery Messages Received: {len(self.batteries_received)}")
        for device, battery in sorted(self.batteries_received.items()):
            percent = battery.percentage * 100
            status_map = {
                0: 'Unknown',
                1: 'Charging',
                2: 'Discharging',
                3: 'Not Charging',
                4: 'Full'
            }
            status = status_map.get(battery.power_supply_status, 'Unknown')
            print(f"   - {device:12}: {percent:5.1f}% ({status})")
        
        # Check for issues
        print("\n🔍 Diagnostics:")
        
        if len(self.poses_received) == 0:
            print("   ⚠️  No pose messages received - check bridge node")
        elif len(self.poses_received) < len(self.devices_found):
            missing = self.devices_found - set(self.poses_received.keys())
            print(f"   ⚠️  Missing poses for: {missing}")
        else:
            print("   ✅ All devices publishing poses")
        
        if len(self.batteries_received) == 0:
            print("   ⚠️  No battery messages received")
        elif len(self.batteries_received) < len(self.devices_found):
            missing = self.devices_found - set(self.batteries_received.keys())
            print(f"   ⚠️  Missing battery for: {missing}")
        else:
            print("   ✅ All devices publishing battery status")
        
        # Check TF frames
        print("\n🗺️  To view in RViz2, run:")
        print("   rviz2")
        print("   Then set Fixed Frame to 'map'")
        print("   Add -> By Topic -> /octopus/devices/*/pose")
        
        print("\n" + "="*60)
        print("Test complete! Press Ctrl+C to exit")
        print("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    
    print("\n🧪 ROS 2 Integration Test Starting...")
    print("Make sure these are running:")
    print("  1. simulate_movement.py (your simulation)")
    print("  2. uvicorn api:app (your API server)")
    print("  3. ros2 run octopus_fleet bridge_node (ROS bridge)")
    print("\nListening for device data...\n")
    
    node = IntegrationTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Test stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()