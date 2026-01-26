# Multi-Team ROS 2 Communication Setup

## 🌐 How ROS 2 Multi-Robot Communication Works

ROS 2 uses **DDS (Data Distribution Service)** which automatically discovers other ROS 2 nodes on the same network. This means:
- ✅ Automatic discovery - no manual IP configuration
- ✅ Publish-subscribe model - efficient data sharing
- ✅ Multiple teams can communicate simultaneously
- ✅ Each team can see other teams' robots in real-time

## 🔧 Network Configuration

### Option 1: Same Network (Easiest)

If all teams are on the **same WiFi/LAN network**:

**Everyone runs this:**
```bash
# Set the same ROS_DOMAIN_ID for all teams
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
```

That's it! ROS 2 will automatically find all nodes.

### Option 2: Different Domain IDs per Team

If you want to organize teams but still allow cross-team communication:

**Team 1:**
```bash
export ROS_DOMAIN_ID=1
```

**Team 2:**
```bash
export ROS_DOMAIN_ID=2
```

**Bridge between teams (one laptop):**
```bash
# This laptop subscribes to both domains and forwards messages
export ROS_DOMAIN_ID=1  # or use domain bridge tool
```

### Option 3: Internet Communication (Remote Teams)

For teams in different locations (different WiFi networks):

**Setup VPN or use ROS 2 Cloud Bridge:**
- Use Tailscale/ZeroTier for VPN mesh network
- Or use Husarnet (built for ROS 2)
- Or use ROS 2 cloud bridges

## 🤝 Implementation Steps

### Step 1: Configure Network Settings

**On each team's laptop, create a config file:**

```bash
nano ~/octopus-dashboard/ros2_config.sh
```

```bash
#!/bin/bash
# ROS 2 Multi-Team Configuration

# Set domain ID (0-101, use same for all teams)
export ROS_DOMAIN_ID=0

# Your team ID (change this per team)
export TEAM_ID="team_garching"

# Network interface (find yours with: ip addr)
export ROS_LOCALHOST_ONLY=0  # Allow network communication

# For better discovery across networks
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# DDS configuration (optional - improves reliability)
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/octopus-dashboard/fastdds_profile.xml

echo "✅ ROS 2 configured for multi-team communication"
echo "   Domain ID: $ROS_DOMAIN_ID"
echo "   Team ID: $TEAM_ID"
```

**Make it executable and use it:**
```bash
chmod +x ~/octopus-dashboard/ros2_config.sh
source ~/octopus-dashboard/ros2_config.sh
source /opt/ros/jazzy/setup.bash
```

### Step 2: Test Cross-Team Discovery

**Each team runs this to see all nodes:**

```bash
source ~/octopus-dashboard/ros2_config.sh
source /opt/ros/jazzy/setup.bash

# List all nodes across all teams
ros2 node list

# List all topics across all teams
ros2 topic list

# See which team each node belongs to
ros2 node info /octopus_bridge
```

### Step 3: Subscribe to Other Teams' Data

**Each team can now see other teams' robots!**

```bash
# See Team 2's drone position
ros2 topic echo /team2/octopus/devices/drone_1/pose

# See all teams' fleet status
ros2 topic echo /team1/octopus/fleet_status
ros2 topic echo /team2/octopus/fleet_status
```

## 🛠️ Enhanced Bridge Node for Multi-Team

Update your bridge node to include team namespace:

```python
# Add team prefix to all topics
TEAM_ID = os.getenv('TEAM_ID', 'team_unknown')

# Instead of '/octopus/devices/...'
# Use '/{TEAM_ID}/octopus/devices/...'

self.device_publishers[device_id] = self.create_publisher(
    PoseStamped,
    f'/{TEAM_ID}/octopus/devices/{device_id}/pose',
    10
)
```

## 📊 Visualization: All Teams on One Map

Create a multi-team dashboard that shows all teams' robots.

## 🔒 Security Considerations

**For competition/sensitive scenarios:**

1. **Use different domain IDs**
2. **Enable DDS Security** (optional, advanced)
3. **Use encrypted VPN** for internet communication

## ⚡ Quick Start Checklist

- [ ] All laptops on same network or VPN
- [ ] Same `ROS_DOMAIN_ID` on all laptops
- [ ] Each team has unique `TEAM_ID`
- [ ] Firewall allows UDP ports 7400-7600
- [ ] Run `ros2 node list` to verify discovery

## 🎯 What Happens Now

When Team 1 and Team 2 both start their bridge nodes:

**Team 1 sees:**
```
/team1/octopus/devices/drone_1/pose
/team1/octopus/devices/robot_1/pose
/team2/octopus/devices/drone_1/pose  ← Team 2's robots!
/team2/octopus/devices/robot_1/pose  ← Team 2's robots!
```

**Result:** Everyone can see everyone's robots in real-time! 🚀