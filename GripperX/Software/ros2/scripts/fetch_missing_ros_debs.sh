#!/bin/bash
# DT-4: This laptop was missing several ROS Jazzy system packages needed for
# SLAM/Nav2 in the digital twin: slam_toolbox, nav2_map_server (+ their
# dependencies). "sudo apt install ros-jazzy-slam-toolbox
# ros-jazzy-nav2-map-server" is the CLEAN, permanent fix -- but needs a root
# password the agent doesn't have.
#
# This script is the non-root fallback: it downloads exactly the missing
# .deb packages (apt-get download needs NO root) and extracts them (dpkg-deb
# -x, also without root) into ~/gripperx_ws/.rosdeps_local, which mirrors the
# FHS structure (opt/ros/jazzy/..., usr/lib/...). .rosdeps_local is NOT
# committed (.gitignore) -- binary files, machine-specific.
#
# Once someone with root privileges installs the real packages
# (sudo apt install ...), .rosdeps_local can be deleted and this script
# skipped -- scripts/sim_env_dt4.sh then automatically falls back to the
# regular /opt/ros/jazzy paths (no harmful duplicate path).
#
# Usage: bash scripts/fetch_missing_ros_debs.sh

set -euo pipefail

LOCAL_PREFIX="$(dirname "${BASH_SOURCE[0]}")/../.rosdeps_local"
mkdir -p "$LOCAL_PREFIX"

WORKDIR="$(mktemp -d)"
trap 'rm -rf "$WORKDIR"' EXIT
cd "$WORKDIR"

# Order = order in which the missing depends were discovered
# (each checked via `apt-cache depends <pkg>` + `dpkg -s <pkg>`).
PACKAGES=(
  # slam_toolbox (M2) + its runtime dependencies missing on this laptop
  ros-jazzy-slam-toolbox
  ros-jazzy-bond
  ros-jazzy-bondcpp
  ros-jazzy-smclib
  libceres4t64
  libgoogle-glog0v6t64
  libcholmod5
  libspqr4
  libamd3
  libcamd3
  libccolamd3
  libcolamd3
  libsuitesparseconfig7
  # nav2_map_server (map_saver_cli, DT-4 acceptance criterion "map can be saved")
  ros-jazzy-nav2-map-server
  ros-jazzy-nav2-msgs
  ros-jazzy-nav2-util
  ros-jazzy-nav2-common
  ros-jazzy-geographic-msgs
  libgraphicsmagick-q16-3t64
  "libgraphicsmagick++-q16-12t64"
)

echo "[fetch_missing_ros_debs] downloading ${#PACKAGES[@]} packages without root ..."
apt-get download "${PACKAGES[@]}"

for deb in *.deb; do
  echo "[fetch_missing_ros_debs] extracting $deb to $LOCAL_PREFIX"
  dpkg-deb -x "$deb" "$LOCAL_PREFIX"
done

echo "[fetch_missing_ros_debs] done. Env setup: source scripts/sim_env_dt4.sh"
