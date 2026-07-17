#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# UDP-only FastDDS transport (SHM off) — must be consistent across all gripperx
# services, otherwise localhost data paths break. Rationale: gripperx-bringup.sh.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
export HOME=/home/ubuntu

ros2 launch gripperx_bringup mapping.launch.py &
# Fix 8 (#12): $! was missing here ("LAUNCH_PID=" instead of "LAUNCH_PID=$!").
# As a result LAUNCH_PID was always empty, and "wait $LAUNCH_PID" below, after
# word splitting, effectively became a plain "wait" (waits for all background
# jobs) - this happened to behave correctly in this particular case, but was
# fragile (e.g. if a second background job were added here in the future,
# "wait" would wait on the wrong one or return too early).
LAUNCH_PID=$!

# Wait until slam_toolbox is visible in the DDS graph
echo "[mapping] Waiting for slam_toolbox..."
until ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q unconfigured; do
  sleep 2
done

echo "[mapping] Configuring slam_toolbox..."
until ros2 lifecycle set /slam_toolbox configure 2>&1 | grep -q successful; do
  sleep 1
done

echo "[mapping] Activating slam_toolbox..."
until ros2 lifecycle set /slam_toolbox activate 2>&1 | grep -q successful; do
  sleep 1
done

echo "[mapping] slam_toolbox active — building map."
wait "$LAUNCH_PID"
