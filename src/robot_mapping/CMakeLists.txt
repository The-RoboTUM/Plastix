cmake_minimum_required(VERSION 3.8)
project(robot_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rtabmap_ros REQUIRED)

add_executable(visual_slam src/visual_slam.cpp)

ament_target_dependencies(visual_slam
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  rtabmap_ros
)

install(TARGETS
  visual_slam
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
