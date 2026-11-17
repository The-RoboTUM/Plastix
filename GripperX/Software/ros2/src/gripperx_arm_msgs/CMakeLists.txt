cmake_minimum_required(VERSION 3.8)
project(gripperx_arm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlastic.action"
  "msg/ArmState.msg"
  "msg/LeaderFollowDebug.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# The TF-freshness bound shared by the goal gateway and the arm (SR-16 E3).
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
