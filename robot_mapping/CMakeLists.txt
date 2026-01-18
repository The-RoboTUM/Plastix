cmake_minimum_required(VERSION 3.10)
project(robot_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include)

add_executable(mapping_with_know_poses 
  src/mapping_with_know_poses.cpp)

ament_target_dependencies(mapping_with_know_poses rclcpp sensor_msgs tf2_ros nav_msgs  geometry_msgs tf2 )


install(
  DIRECTORY include 
  DESTINATION include 
)

install(
  DIRECTORY config launch rviz maps
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  mapping_with_know_poses
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
