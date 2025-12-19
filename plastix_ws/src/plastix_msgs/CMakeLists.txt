cmake_minimum_required(VERSION 3.8)
project(plastix_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Garbage Detection Messages
  "msg/GarbageItem.msg"
  "msg/GarbageArray.msg"
  "msg/DetectionZone.msg"
  
  # Robot Status Messages
  "msg/RobotStatus.msg"
  "msg/DroneStatus.msg"
  "msg/GripperStatus.msg"
  "msg/ShipStatus.msg"
  
  # Commands
  "msg/MissionCommand.msg"
  "msg/NavigationGoal.msg"
  
  # Brain Coordination
  "msg/TeamHeartbeat.msg"
  "msg/SystemStatus.msg"
  
  # Services
  "srv/RegisterRobot.srv"
  "srv/AssignMission.srv"
  "srv/GetZoneStatus.srv"
  "srv/RequestPickup.srv"
  
  # Actions
  "action/CollectGarbage.action"
  "action/ScanArea.action"
  
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()

