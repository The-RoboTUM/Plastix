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
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_executable(visual_slam src/visual_slam.cpp)

ament_target_dependencies(visual_slam
  rclcpp
  sensor_msgs
  message_filters
  cv_bridge
  rtabmap_ros
  OpenCV
  Eigen3
  PCL
  sensor_msgs
)

target_link_libraries(visual_slam
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  message_filters::message_filters
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(TARGETS
  visual_slam
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
