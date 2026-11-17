#!/usr/bin/env bash
# Offline IMU A/B replay (sub-issue 2.5). Desk only; see gripperx_localization/imu_replay.py.
#   Software/ros2/scripts/gripperx_imu_replay.sh matrix --stage drive_core --stage drive_grid
# The bags contain /cmd_vel: the domain is set HERE and again in imu_replay.py, never
# inherited, and domain 20 (the robot) is refused.
set -eu
if [ "${ROS_DOMAIN_ID:-}" = "20" ]; then
  echo "ROS_DOMAIN_ID=20 (the robot) in this shell: refused. Unset it and run again." >&2
  exit 3
fi
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
set -u
exec python3 -m gripperx_localization.imu_replay "$@"
