cmake_minimum_required(VERSION 3.8)
project(sharx_communication)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(
  dummy_octopus
  src/dummy_octopus.cpp
)

ament_target_dependencies(
  dummy_octopus
  rclcpp
  std_msgs
)

target_link_libraries(
  dummy_octopus
  nlohmann_json::nlohmann_json
)

add_executable(
  sharx_receiver
  src/sharx_receiver.cpp
)

ament_target_dependencies(
  sharx_receiver
  rclcpp
  std_msgs
)

target_link_libraries(
  sharx_receiver
  nlohmann_json::nlohmann_json
)

add_executable(
  sharx_teleop
  src/sharx_teleop.cpp
)

ament_target_dependencies(
  sharx_teleop
  rclcpp
  geometry_msgs
)

add_executable(
  thruster_mixer
  src/thruster_mixer.cpp
)

ament_target_dependencies(
  thruster_mixer
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(
  movement_status
  src/movement_status.cpp
)

ament_target_dependencies(
  movement_status
  rclcpp
  std_msgs
)

target_link_libraries(
  movement_status
  nlohmann_json::nlohmann_json
)

add_executable(
  esp32_serial_bridge
  src/esp32_serial_bridge.cpp
)

ament_target_dependencies(
  esp32_serial_bridge
  rclcpp
  std_msgs
)

install(
  TARGETS
    dummy_octopus
    sharx_receiver
    sharx_teleop
    thruster_mixer
    movement_status
    esp32_serial_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
