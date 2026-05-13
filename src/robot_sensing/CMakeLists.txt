cmake_minimum_required(VERSION 3.8)
project(robot_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(image_transport REQUIRED)

add_executable(realsense_node src/realsense_node.cpp)
add_executable(realsense_slam_provider src/realsense_slam_provider.cpp)

target_include_directories(realsense_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(realsense_slam_provider PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(realsense_node
  ${OpenCV_LIBS}
  realsense2::realsense2
)
target_link_libraries(realsense_slam_provider
  ${OpenCV_LIBS}
  realsense2::realsense2
)

ament_target_dependencies(realsense_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)
ament_target_dependencies(realsense_slam_provider
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS realsense_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS realsense_slam_provider
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
