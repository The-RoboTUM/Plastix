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

install(
  TARGETS
    dummy_octopus
    sharx_receiver
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
