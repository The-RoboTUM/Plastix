#!/bin/bash
# Isolated sim environment for the GripperX digital twin (SR-8).
# Reason: ~/.bashrc sources an unrelated ~/ros2_ws with its own ROS_DOMAIN_ID
# (AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH point there). This script cuts that off
# and rebuilds the environment exclusively from /opt/ros/jazzy +
# gripperx_ws/install, on the twin domain.
# Usage: source scripts/sim_env.sh
#
# PlastiX domain convention (2026-08-13, project-wide): gripper robots take ids in
# the 20s, their digital twin the same id +200. Real GripperX-1 = 20, twin = 220.
# Was domain 7 until 2026-08-13.
#
# Why +200 and not +100: ROS 2 derives the DDS base port as 7400 + 250*domain_id.
# With Linux's default ephemeral range 32768-60999, ids 102-214 land inside it and
# can lose discovery to whatever grabbed the port first. Safe bands are 0-101 and
# 215-232, so 20 and 220 are both clean and 20-29 maps onto 220-229 unbroken.
# Verify locally with: sysctl net.ipv4.ip_local_port_range

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash"

export ROS_DOMAIN_ID=220
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Dead NIC / no real multicast interface on this laptop for the sim session:
# GZ_IP=127.0.0.1 prevents gz-transport multicast exceptions.
export GZ_IP=127.0.0.1

echo "[sim_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION GZ_IP=$GZ_IP"
