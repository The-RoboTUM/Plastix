cmake_minimum_required(VERSION 3.5)
project(ldlidar_stl_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # Skipped as in the other GripperX packages: the code is open source and the new files
  # (scan_stamp.*, test_scan_stamp.cpp) carry no copyright line by user decision 2026-11-05.
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Bin and Install
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/core/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/dataprocess/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/filter/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/logger/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/networkcom/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/serialcom/
)

# GripperX: /scan stamp computation, separate so it is unit-testable without ROS.
add_library(scan_stamp STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/scan_stamp.cpp)

file(GLOB  LDLIDAR_DRI_CORE ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/core/*.cpp)
file(GLOB  LDLIDAR_DRI_DATARPC ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/dataprocess/*.cpp)
file(GLOB  LDLIDAR_DRI_FILTER ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/filter/*.cpp)
file(GLOB  LDLIDAR_DRI_LOGGER ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/logger/*.cpp)
file(GLOB  LDLIDAR_DRI_NETWORK ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/networkcom/*.cpp)
file(GLOB  LDLIDAR_DRI_SERIAL ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/serialcom/*.cpp)

add_executable(${PROJECT_NAME}_node  
  ${CMAKE_CURRENT_SOURCE_DIR}/src/demo.cpp 
  ${LDLIDAR_DRI_CORE} 
  ${LDLIDAR_DRI_DATARPC} 
  ${LDLIDAR_DRI_FILTER} 
  ${LDLIDAR_DRI_LOGGER} 
  ${LDLIDAR_DRI_NETWORK} 
  ${LDLIDAR_DRI_SERIAL}
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs)
target_link_libraries(${PROJECT_NAME}_node pthread scan_stamp)

# Install
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz2
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_stamp test/test_scan_stamp.cpp)
  target_link_libraries(test_scan_stamp scan_stamp)
  # Clock check: per-point stamps (system_clock) vs node->now() (rclcpp ROS time).
  ament_target_dependencies(test_scan_stamp rclcpp)
endif()

ament_package()
