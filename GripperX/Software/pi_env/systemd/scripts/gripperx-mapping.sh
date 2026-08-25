#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/Software/ros2/install/setup.bash
# PlastiX domain convention (2026-08-13): gripper robots use 20-series ids, their
# digital twin the same id +200. Real GripperX-1 = 20, its twin = 220. Both land in
# the ROS 2 domain bands that are safe on Linux (0-101 and 215-232) — ids 102-214
# map onto the default ephemeral port range 32768-60999 and risk port collisions.
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# UDP-only FastDDS transport (SHM off) — must be consistent across all gripperx
# services, otherwise localhost data paths break. Rationale: gripperx-bringup.sh.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
export HOME=/home/ubuntu

# REPOINTED 2026-08-24 — from gripperx_bringup/mapping.launch.py to
# gripperx_localization/localization.launch.py. The old path started
# rf2o_laser_odometry (publish_tf: true, odom -> base_footprint) plus slam_toolbox on
# gripperx_bringup/config/slam_toolbox.yaml, and NO EKF. Three consequences, and the
# first one made this service unable to run Nav2 at all:
#
#   1. NOTHING published /odometry/filtered on this path, and gripperx_planning's
#      nav2.yaml points BOTH bt_navigator and controller_server at that topic. Nav2
#      therefore came up green with permanently zero velocity feedback — not down,
#      which is the harder failure to see.
#   2. rf2o and the EKF both claim odom -> base_footprint. Only the absence of the EKF
#      kept that from being a TF fight, i.e. the defect was hiding a second defect.
#   3. gripperx_bringup/config/slam_toolbox.yaml does NOT carry the 2026-08-21
#      real-robot scan-matcher tuning (angle_variance_penalty 2.0,
#      distance_variance_penalty 0.8, 4f788e1) or the fine gating
#      (minimum_travel_distance/heading 0.05 vs 0.3) that the slam_toolbox block of
#      gripperx_localization/config/localization.yaml carries.
#
# localization.launch.py supplies the EKF, the tuned slam_toolbox block, /laser/odom
# and odom_divergence_monitor, and drives the slam_toolbox lifecycle itself. The
# arguments below are deliberately the SAME SET that the Nav2 deploy/test list
# (internal NAV2_DEPLOY_TEST_2026-08-25, §6) uses by hand, so the autostart and
# the tested path are the same stack.
#
# enable_saved_map_localization:=false — live SLAM, no map_server/AMCL (user decision
# 2026-08-24). enable_laser_odometry:=true runs the scan matcher and publishes
# /laser/odom WITHOUT letting it into the EKF (fuse_laser_odometry stays false), which
# is what gives odom_divergence_monitor a second opinion at zero risk to the estimate.
#
# ROLLBACK, if this path misbehaves on the robot — the pre-2026-08-24 behaviour was:
#     ros2 launch gripperx_bringup mapping.launch.py &
# followed by unconditional configure/activate loops on /slam_toolbox. Recover the
# exact previous script with: git show <this commit>^:Software/pi_env/systemd/scripts/gripperx-mapping.sh
ros2 launch gripperx_localization localization.launch.py \
  use_sim_time:=false \
  use_rviz:=false \
  enable_slam:=true \
  enable_saved_map_localization:=false \
  enable_laser_odometry:=true &
# Fix 8 (#12): $! was missing here ("LAUNCH_PID=" instead of "LAUNCH_PID=$!").
# As a result LAUNCH_PID was always empty, and "wait $LAUNCH_PID" below, after
# word splitting, effectively became a plain "wait" (waits for all background
# jobs) - this happened to behave correctly in this particular case, but was
# fragile (e.g. if a second background job were added here in the future,
# "wait" would wait on the wrong one or return too early).
LAUNCH_PID=$!

# localization.launch.py drives slam_toolbox's configure/activate itself
# (LifecycleTransition + an OnStateTransition handler). The block below does NOT repeat
# that work — it is a SAFETY NET that only acts if the node is still not active after
# the grace period. The previous script drove the transitions unconditionally, and that
# robustness is deliberately not traded away for an assumption about launch-file timing
# on a Pi that has never cold-booted this path.
#
# THE PATTERN IS ANCHORED ON PURPOSE: "ros2 lifecycle get" prints "inactive [2]" for the
# inactive state, so an unanchored `grep -q active` would report an INACTIVE node as
# active and the safety net would never fire. Anchor stays, see the test:
#   Software/ros2/src/gripperx_localization/test/check_autonomy_launch_wiring.py
slam_state_is_active() {
  ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q '^active'
}

echo "[mapping] Waiting for slam_toolbox to reach active..."
SLAM_ACTIVE=0
for _ in $(seq 1 20); do
  if slam_state_is_active; then
    SLAM_ACTIVE=1
    break
  fi
  sleep 1
done

if [ "$SLAM_ACTIVE" -eq 1 ]; then
  echo "[mapping] slam_toolbox active — building map."
else
  echo "[mapping] slam_toolbox not active after 20 s — driving the lifecycle by hand."
  ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1 || true
  ros2 lifecycle set /slam_toolbox activate  >/dev/null 2>&1 || true
  if slam_state_is_active; then
    echo "[mapping] slam_toolbox active after the manual transition."
  else
    echo "[mapping] WARNING: slam_toolbox did NOT reach active. There will be no map," >&2
    echo "[mapping]          so Nav2's global costmap stays empty and every goal fails." >&2
  fi
fi

wait "$LAUNCH_PID"
