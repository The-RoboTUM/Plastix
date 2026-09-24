cmake_minimum_required(VERSION 3.8)
project(gripperx_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# geometry_msgs/Pose2D is deprecated in Jazzy, but the nav2 1.3.12 collision
# checker API (CostmapTopicCollisionChecker::isCollisionFree) still takes it, so
# the deprecation cannot be avoided here without wrapping upstream types.
add_compile_options(-Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# --- Digital-twin build shim ------------------------------------------------
# On the laptop the Nav2 core packages come from debs extracted into
# Software/ros2/.rosdeps_local (see scripts/sim_env_nav2.sh), i.e. NOT at the
# /opt/ros/jazzy prefix they were built for. The flattened link interface that
# comes back from those packages therefore contains at least one ABSOLUTE path
# from that build prefix which does not exist here
# (/opt/ros/jazzy/lib/libvoxel_grid.so, via nav2_costmap_2d), and gmake refuses
# the whole target with "No rule to make target".
#
# Applied to THIS target's own link list rather than to the imported targets:
# the dangling entry arrives flattened through ament_target_dependencies, not
# only through the imported target properties, so patching the properties alone
# does not remove it (verified).
#
# Dropping is safe because a path that does not exist cannot be providing any
# symbol at link time, and this is a plugin .so that is dlopen'ed into
# behavior_server, where the real libraries are already resolved. On a proper
# Nav2 install (the real robot) every path exists and this is a no-op.
function(gripperx_drop_dangling_link_paths)
  foreach(_target IN LISTS ARGN)
    if(NOT TARGET ${_target})
      continue()
    endif()
    foreach(_prop LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
      get_target_property(_libs ${_target} ${_prop})
      if(NOT _libs)
        continue()
      endif()
      set(_kept "")
      foreach(_lib IN LISTS _libs)
        if(IS_ABSOLUTE "${_lib}" AND NOT EXISTS "${_lib}")
          message(STATUS "gripperx_behaviors: dropping dangling ${_lib} from ${_target} ${_prop}")
        else()
          list(APPEND _kept "${_lib}")
        endif()
      endforeach()
      set_target_properties(${_target} PROPERTIES ${_prop} "${_kept}")
    endforeach()
  endforeach()
endfunction()

# Imported targets first (transitive interface), the plugin target itself after
# ament_target_dependencies (flattened direct entries) - both carry the path.
gripperx_drop_dangling_link_paths(
  nav2_costmap_2d::layers
  nav2_costmap_2d::filters
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_costmap_2d::nav2_costmap_2d_client)

set(dependencies
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros)

add_library(gripperx_crab_walk_behavior SHARED src/crab_walk.cpp)
target_include_directories(gripperx_crab_walk_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(gripperx_crab_walk_behavior ${dependencies})
gripperx_drop_dangling_link_paths(gripperx_crab_walk_behavior)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(
  FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS gripperx_crab_walk_behavior
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(gripperx_crab_walk_behavior)
ament_export_dependencies(${dependencies})
ament_package()
