cmake_minimum_required(VERSION 3.8)
project(bot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/sim_with_logging
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
