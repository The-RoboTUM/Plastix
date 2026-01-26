#!/bin/bash
# Setup script for Octopus Fleet ROS 2 workspace

echo "🤖 Setting up Octopus Fleet ROS 2 Workspace..."

# Create ROS 2 workspace
mkdir -p ~/octopus_ws/src
cd ~/octopus_ws/src

# Create package structure
mkdir -p octopus_fleet/octopus_fleet
mkdir -p octopus_fleet/msg
mkdir -p octopus_fleet/launch
mkdir -p octopus_fleet/config

# Create package.xml (copy the package.xml artifact content here)
echo "Creating package.xml..."

# Create CMakeLists.txt
cat > octopus_fleet/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(octopus_fleet)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DevicePosition.msg"
  "msg/DeviceStatus.msg"
  "msg/FleetStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

# Install Python scripts
install(PROGRAMS
  scripts/bridge_node.py
  scripts/device_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()