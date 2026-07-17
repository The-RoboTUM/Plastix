#!/bin/bash
# Isolated sim environment for DT-4 (M2: LiDAR sim + SLAM -> map).
# Analogous to scripts/sim_env.sh (M0/M1), but:
#  - ROS_DOMAIN_ID=8 instead of 7: while DT-4 was running, an M1 dev agent was
#    running in parallel on domain 7 (SR-8 sim domain isolation) ->
#    collision avoidance, see REQUIREMENTS.md SR-8. Once domain 7 is verified
#    free (`ROS_DOMAIN_ID=7 ros2 node list` returns nothing), "export
#    ROS_DOMAIN_ID=7" can be set instead.
#  - additionally appends .rosdeps_local (if present) to AMENT_PREFIX_PATH/
#    LD_LIBRARY_PATH: on this laptop, ros-jazzy-slam-toolbox and
#    ros-jazzy-nav2-map-server plus their dependencies were missing (no
#    root/sudo password available) -> pulled in locally via apt-get download +
#    dpkg-deb -x (no root needed), see scripts/fetch_missing_ros_debs.sh. If
#    .rosdeps_local isn't present (e.g. because the packages have since been
#    installed "properly" via `sudo apt install ros-jazzy-slam-toolbox
#    ros-jazzy-nav2-map-server`), this block has no effect.
#
# Usage: source scripts/sim_env_dt4.sh

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/../install/setup.bash"

export ROS_DOMAIN_ID=8
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Dead NIC / no real multicast interface on this laptop for the sim session:
# GZ_IP=127.0.0.1 prevents gz-transport multicast exceptions.
export GZ_IP=127.0.0.1

ROSDEPS_LOCAL="$(dirname "${BASH_SOURCE[0]}")/../.rosdeps_local"
if [ -d "$ROSDEPS_LOCAL/opt/ros/jazzy" ]; then
  export AMENT_PREFIX_PATH="$(cd "$ROSDEPS_LOCAL/opt/ros/jazzy" && pwd):$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$(cd "$ROSDEPS_LOCAL/opt/ros/jazzy/lib" && pwd):$(cd "$ROSDEPS_LOCAL/usr/lib/x86_64-linux-gnu" && pwd):$(cd "$ROSDEPS_LOCAL/usr/lib" && pwd):$LD_LIBRARY_PATH"
  echo "[sim_env_dt4] .rosdeps_local active (slam_toolbox/nav2_map_server fallback, see script comment)"
fi

echo "[sim_env_dt4] ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION GZ_IP=$GZ_IP"
