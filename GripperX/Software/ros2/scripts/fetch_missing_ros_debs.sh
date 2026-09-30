#!/bin/bash
# DT-4: This laptop was missing several ROS Jazzy system packages needed for
# SLAM/Nav2 in the digital twin: slam_toolbox, nav2_map_server (+ their
# dependencies). "sudo apt install ros-jazzy-slam-toolbox
# ros-jazzy-nav2-map-server" is the CLEAN, permanent fix -- but needs a root
# password the agent doesn't have.
#
# This script is the non-root fallback: it downloads exactly the missing
# .deb packages (apt-get download needs NO root) and extracts them (dpkg-deb
# -x, also without root) into Software/ros2/.rosdeps_local, which mirrors the
# FHS (Filesystem Hierarchy Standard) layout (opt/ros/jazzy/..., usr/lib/...).
# .rosdeps_local is NOT committed (.gitignore) -- binary files, machine-specific.
#
# apt-get download fetches ONLY the named packages, never their dependencies,
# so PACKAGES has to be the full closure of what is missing. It was derived on
# this laptop with
#   apt-get install --print-uris -qq <top-level packages>
# which lists everything apt would still have to install. On a machine with a
# different set of system packages that closure can differ: re-derive it there.
#
# Once someone with root privileges installs the real packages
# (sudo apt install ...), .rosdeps_local can be deleted and this script
# skipped -- scripts/sim_env_nav2.sh / sim_env_dt4.sh then fall back to the
# regular /opt/ros/jazzy paths (no harmful duplicate path).
#
# Usage: bash scripts/fetch_missing_ros_debs.sh
#   ROSDEPS_LOCAL_PREFIX=<dir> extracts somewhere else instead, e.g. a scratch
#   directory to test a changed PACKAGES list without touching the real one.

set -euo pipefail

LOCAL_PREFIX="${ROSDEPS_LOCAL_PREFIX:-$(dirname "${BASH_SOURCE[0]}")/../.rosdeps_local}"
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
  # Nav2 core for sim_navigation.launch.py / gripperx_planning navigation.launch.py
  # (gripperx_desk.sh twin). Top level: the servers that launch starts
  # (planner, controller, behavior, bt_navigator, velocity_smoother,
  # lifecycle_manager, amcl) plus the plugin packages gripperx_planning's
  # nav2.yaml loads (navfn planner, rotation-shim controller wrapping DWB).
  # The rest is their closure, derived as in the header on 2026-09-29.
  ros-jazzy-nav2-planner
  ros-jazzy-nav2-controller
  ros-jazzy-nav2-behaviors
  ros-jazzy-nav2-bt-navigator
  ros-jazzy-nav2-velocity-smoother
  ros-jazzy-nav2-lifecycle-manager
  ros-jazzy-nav2-amcl
  ros-jazzy-nav2-navfn-planner
  ros-jazzy-nav2-rotation-shim-controller
  ros-jazzy-nav2-dwb-controller
  ros-jazzy-dwb-core
  ros-jazzy-dwb-critics
  ros-jazzy-dwb-msgs
  ros-jazzy-dwb-plugins
  ros-jazzy-costmap-queue
  ros-jazzy-nav2-core
  ros-jazzy-nav2-costmap-2d
  ros-jazzy-nav2-voxel-grid
  ros-jazzy-nav2-behavior-tree
  ros-jazzy-behaviortree-cpp
  ros-jazzy-nav-2d-msgs
  ros-jazzy-nav-2d-utils
  # Installed system-wide, so the closure above does not list it. Fetched anyway
  # because sim_env_nav2.sh is built around a .rosdeps_local copy: the Nav2 debs
  # of the June 2026 set needed a newer diagnostic_updater ABI than the system
  # one, and GRIPPERX_ROSDEPS_LIB hands the Nav2 processes this copy. The
  # 2026-09-29 set loaded with the system copy as well (lifecycle_manager
  # checked without it), so for current debs this is belt and braces.
  ros-jazzy-diagnostic-updater
)

echo "[fetch_missing_ros_debs] downloading ${#PACKAGES[@]} packages without root ..."
apt-get download "${PACKAGES[@]}"

for deb in *.deb; do
  echo "[fetch_missing_ros_debs] extracting $deb to $LOCAL_PREFIX"
  dpkg-deb -x "$deb" "$LOCAL_PREFIX"
done

echo "[fetch_missing_ros_debs] done. Env setup: source scripts/sim_env_nav2.sh (Nav2) or scripts/sim_env_dt4.sh (SLAM only)"
