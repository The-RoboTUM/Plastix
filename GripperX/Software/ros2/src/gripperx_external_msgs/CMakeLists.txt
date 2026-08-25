cmake_minimum_required(VERSION 3.8)
project(gripperx_external_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GeodeticDatum.msg"
  "msg/ExternalGoal.msg"
  "msg/ExternalGoalStatus.msg"
  "msg/ExternalLinkStatus.msg"
  "msg/ExternalTarget.msg"
  "msg/ExternalTargetList.msg"
  "msg/RobotTelemetry.msg"
  "msg/ArmingState.msg"
  "srv/SetArming.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
