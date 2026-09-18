#!/usr/bin/env bash
# Run rosbridge in the foreground with the arguments the GripperX link needs.
#
# This is the single place those arguments live: the debug stack script, the
# systemd unit and a manual run all go through here, so the link can never be
# started two subtly different ways.
#
# Why the arguments look like this on Humble (rosbridge 2.0.7):
#
#   --topics_glob "['/octopus/*']"
#       A STRING parameter that rosbridge parses itself. Passing it through
#       `ros2 launch ... topics_glob:="[/octopus/*]"` makes ROS 2 coerce it to
#       STRING_ARRAY and the node dies with InvalidParameterTypeException.
#       Matching is fnmatch, where * spans '/', so this one entry also covers
#       /octopus/devices/gripperx/status - verified, not assumed.
#
#   --services_glob "[]"
#       No service is callable. rosbridge appends /rosapi/* to a non-null
#       services glob, but we never start the rosapi node, so there is nothing
#       behind it: a call returns "Service /rosapi/topics does not exist".
#
#   --actions_glob "[]"
#       NOT in the GripperX document, and the reason this script exists. On
#       2.0.7 an unset actions glob means UNRESTRICTED: a client could send
#       goals to any action server on the graph. "[]" closes that surface.
#
#   There is no params_glob on 2.0.7 - the parameter was removed. Passing it is
#   silently ignored, so do not rely on it for anything.

# No `set -u`: the ROS setup files read variables they have not set yet
# (AMENT_TRACE_SETUP_FILES and friends), so nounset makes sourcing them fail.
set -eo pipefail

PORT="${OCTOPUS_ROSBRIDGE_PORT:-9090}"
ADDRESS="${OCTOPUS_ROSBRIDGE_ADDRESS:-0.0.0.0}"
TOPICS_GLOB="${OCTOPUS_ROSBRIDGE_TOPICS_GLOB:-['/octopus/*']}"

BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# Prefer the apt package if it is installed; fall back to the local overlay
# build (see scripts/build_rosbridge.sh).
if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
  if [ -f "$BASE/ros2_ws_rosbridge/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$BASE/ros2_ws_rosbridge/install/setup.bash"
  else
    echo "rosbridge_server not found." >&2
    echo "Install it (sudo apt install -y ros-humble-rosbridge-suite)" >&2
    echo "or build the overlay: $BASE/scripts/build_rosbridge.sh" >&2
    exit 1
  fi
fi

# The robot's nodes and rosbridge must share one graph. A domain mismatch is the
# failure that looks like a network problem: the client connects, then silence.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

echo "rosbridge on ${ADDRESS}:${PORT}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, topics ${TOPICS_GLOB}"

exec ros2 run rosbridge_server rosbridge_websocket \
  --port "$PORT" \
  --address "$ADDRESS" \
  --topics_glob "$TOPICS_GLOB" \
  --services_glob "[]" \
  --actions_glob "[]"
