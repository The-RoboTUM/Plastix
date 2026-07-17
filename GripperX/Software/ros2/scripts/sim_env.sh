#!/bin/bash
# Isolated sim environment for the GripperX digital twin (SR-8).
# Reason: ~/.bashrc sources an unrelated ~/ros2_ws with ROS_DOMAIN_ID=0
# (AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH point there). This script cuts that off
# and rebuilds the environment exclusively from /opt/ros/jazzy +
# gripperx_ws/install, domain 7.
# Usage: source scripts/sim_env.sh

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash"

export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Dead NIC / no real multicast interface on this laptop for the sim session:
# GZ_IP=127.0.0.1 prevents gz-transport multicast exceptions.
export GZ_IP=127.0.0.1

echo "[sim_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION GZ_IP=$GZ_IP"
