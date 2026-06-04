#!/usr/bin/env bash
# ROS 2 Jazzy dependencies for simulation, localization, and laser odometry.
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "Source ROS first: source /opt/ros/jazzy/setup.bash"
  exit 1
fi

if [[ "${ROS_DISTRO}" != "jazzy" ]]; then
  echo "This script targets Jazzy (ROS_DISTRO=${ROS_DISTRO})."
fi

PACKAGES=(
  ros-jazzy-ros2-control
  ros-jazzy-gz-ros2-control
  ros-jazzy-controller-manager
  ros-jazzy-joint-state-broadcaster
  ros-jazzy-position-controllers
  ros-jazzy-velocity-controllers
  ros-jazzy-ros-gz-sim
  ros-jazzy-ros-gz-bridge
  ros-jazzy-xacro
  ros-jazzy-robot-state-publisher
  ros-jazzy-tf2-geometry-msgs
  ros-jazzy-tf2-ros
  ros-jazzy-robot-localization
  ros-jazzy-slam-toolbox
  ros-jazzy-nav2-bringup
)

echo "Installing simulation dependencies..."
sudo apt update
sudo apt install -y "${PACKAGES[@]}"

echo ""
echo "Verify:"
echo "  ros2 pkg prefix controller_manager"
echo "  ros2 pkg prefix gz_ros2_control"
if dpkg -S libgz_ros2_control-system.so >/dev/null 2>&1; then
  echo "  Plugin: $(dpkg -S libgz_ros2_control-system.so | awk '{print $2}')"
else
  echo "  ERROR: libgz_ros2_control-system.so not found — install ros-jazzy-gz-ros2-control"
  exit 1
fi
