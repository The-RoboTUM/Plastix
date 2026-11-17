cmake_minimum_required(VERSION 3.8)
project(gripperx_arm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS gripperx_arm/arm_action_server.py
  RENAME arm_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS gripperx_arm/arm_pose_teach.py
  RENAME arm_pose_teach
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS gripperx_arm/arm_joint_state_relay.py
  RENAME arm_joint_state_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS gripperx_arm/eeprom_limits.py
  RENAME arm_eeprom_limits
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS gripperx_arm/wrap_offset.py
  RENAME arm_wrap_offset
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS gripperx_arm/poses.py
  RENAME arm_poses_to_rad
  DESTINATION lib/${PROJECT_NAME}
)

# Operator test tool for the aimed P (FR-16 stage 1) - moves the arm, SR-1 per test.
install(PROGRAMS gripperx_arm/aimed_pick_cli.py
  RENAME arm_aimed_pick
  DESTINATION lib/${PROJECT_NAME}
)

# Offline: the allowed grasp region and the standoff proposal from the active config.
install(PROGRAMS gripperx_arm/aimed_region_cli.py
  RENAME arm_aimed_region
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 37-63 s unloaded, timed out at 120 s with the machine under load (2026-10-08)
  ament_add_pytest_test(test_kinematics test/test_kinematics.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300)
  ament_add_pytest_test(test_safety_filter test/test_safety_filter.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300)
  ament_add_pytest_test(test_eeprom_limits test/test_eeprom_limits.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
  ament_add_pytest_test(test_stop_protected test/test_stop_protected.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 900)
  ament_add_pytest_test(test_poses test/test_poses.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60)
  ament_add_pytest_test(test_servo_bus test/test_servo_bus.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60)
  ament_add_pytest_test(test_wrap_offset test/test_wrap_offset.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
  ament_add_pytest_test(test_planner test/test_planner.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600)
  # Not registered until audit r8 (2026-10-12): colcon test skipped both files.
  ament_add_pytest_test(test_limit_recovery test/test_limit_recovery.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600)
  ament_add_pytest_test(test_self_contact test/test_self_contact.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600)
  ament_add_pytest_test(test_lidar_pair test/test_lidar_pair.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300)
  # Body check, hull stage (robot test 2026-10-20): incident, switches, lidar, adversarial.
  ament_add_pytest_test(test_body_hull test/test_body_hull.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 900)
  # chassis_link as four CAD-derived boxes (2026-10-13): mesh containment, filter reads all.
  ament_add_pytest_test(test_chassis_collision test/test_chassis_collision.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300)
  # Ground-margin recovery for the operator's I (user decision 2026-10-12).
  ament_add_pytest_test(test_ground_recovery test/test_ground_recovery.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600)
  ament_add_pytest_test(test_arm_ground_recovery test/test_arm_ground_recovery.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=229 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 600)
  # Measured: 66 node tests ~530 s under load (2026-10-08; 300 s timed out); 116 tests
  # 420 s at load ~1.8 (2026-10-12). Audit runs at load 10-14 took 2-3x as long, so
  # 900 s had no headroom left: 1800 s. Slow as a whole, not per test - mostly the
  # grace periods and planned home moves each node test waits through.
  ament_add_pytest_test(test_arm_node test/test_arm_node.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    # Discovery OFF: process-local, so parallel runs cannot cross-talk (the test file
    # forces OFF itself as well); a fixed domain is then safe to share.
    ENV ROS_DOMAIN_ID=231 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 1800)
  # Space ordering / abort tests, split from test_arm_node (2026-10-12): 17 tests, 57 s
  # at load ~1.8.
  ament_add_pytest_test(test_arm_space test/test_arm_space.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=232 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 600)
  # A hold keeps the last written goal (user decision 2026-10-13).
  ament_add_pytest_test(test_arm_hold test/test_arm_hold.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=220 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 600)
  # A hung bus transaction: Space bounded, joint states one worker, exit bounded
  # (robot event 2026-10-13).
  ament_add_pytest_test(test_arm_shutdown test/test_arm_shutdown.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=225 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 600)
  # Space aborts P in every phase (user decision 2026-10-12).
  ament_add_pytest_test(test_arm_pick_space test/test_arm_pick_space.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    # 233+ is no valid domain (FastDDS ports overflow: the process exits silently).
    ENV ROS_DOMAIN_ID=230 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 900)
  # Leader setpoints keep a margin inside the joint limits (user decision 2026-10-15).
  ament_add_pytest_test(test_leader_margin test/test_leader_margin.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=226 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 600)
  # The gripper in leader mode (FR-17 amendment 2026-11-13/16): band, follow, closing block,
  # the leader gripper torque-limit hook and its restore on every exit.
  ament_add_pytest_test(test_leader_gripper test/test_leader_gripper.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=222 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 900)
  # Robot run 2026-11-02: no resume after a rejection, one read miss unexplained.
  ament_add_pytest_test(test_leader_hold_resume test/test_leader_hold_resume.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=228 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 600)
  # Soft start, the leader's ground recovery, the hold status line (user decisions
  # 2026-10-20): filter/planner level, then the node.
  ament_add_pytest_test(test_soft_start test/test_soft_start.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600)
  ament_add_pytest_test(test_arm_ground_start test/test_arm_ground_start.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=227 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 900)
  # Resumed searches under simulated load, planner.max_search_s (audit 2026-11-02 M2/M6).
  ament_add_pytest_test(test_search_cap test/test_search_cap.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=221 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 600)
  ament_add_pytest_test(test_aimed_grasp test/test_aimed_grasp.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 600)
  # The aimed P's /tf intake (robot 2026-11-09: STALE_TF from a ~1 s /tf backlog).
  ament_add_pytest_test(test_tf_intake test/test_tf_intake.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 120)
  ament_add_pytest_test(test_arm_aimed test/test_arm_aimed.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=OFF
    TIMEOUT 1200)
endif()

ament_package()
