cmake_minimum_required(VERSION 3.8)
project(gripperx_swerve_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gripperx_control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

# Kinematics + steering limits as a plain library: no rclcpp anywhere in it, so
# the whole FR-5/SR-6 feasibility path can be unit-checked without ROS.
add_library(swerve_kinematics SHARED
  src/swerve_kinematics.cpp
  src/steering_limits.cpp
  # HWR-30a's state machine is rclcpp-free and lives in this library for the
  # same reason the kinematics do: so it can be unit-checked without a stack.
  src/stall_detector.cpp
  # And the velocity regulator's control law, for the same reason again.
  src/wheel_regulator.cpp
  # And the steering alignment gate, which replaces the teleop-side
  # TransitionGuard for every command source rather than only the keyboard.
  src/alignment_gate.cpp)
target_include_directories(swerve_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(swerve_kinematics PUBLIC cxx_std_17)

add_library(${PROJECT_NAME} SHARED src/swerve_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} swerve_kinematics)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  geometry_msgs
  gripperx_control_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs)

pluginlib_export_plugin_description_file(controller_interface gripperx_swerve_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} swerve_kinematics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_steering_limits test/test_steering_limits.cpp)
  target_link_libraries(test_steering_limits swerve_kinematics)

  ament_add_gtest(test_swerve_kinematics test/test_swerve_kinematics.cpp)
  target_link_libraries(test_swerve_kinematics swerve_kinematics)

  ament_add_gtest(test_stall_detector test/test_stall_detector.cpp)
  target_link_libraries(test_stall_detector swerve_kinematics)

  ament_add_gtest(test_wheel_regulator test/test_wheel_regulator.cpp)
  target_link_libraries(test_wheel_regulator swerve_kinematics)

  ament_add_gtest(test_alignment_gate test/test_alignment_gate.cpp)
  target_link_libraries(test_alignment_gate swerve_kinematics)

  # Functional-equivalence harness against the live Python chain — see
  # test/dump_reference.cpp / test/dump_reference.py.
  add_executable(dump_reference test/dump_reference.cpp)
  target_link_libraries(dump_reference swerve_kinematics)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME} swerve_kinematics)
ament_export_dependencies(
  controller_interface
  geometry_msgs
  gripperx_control_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs)
ament_package()
