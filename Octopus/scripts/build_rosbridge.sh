#!/usr/bin/env bash
# Build the rosbridge workspace that carries the GripperX link.
#
# The GripperX team's setup document asks for the stock Debian package:
#
#     sudo apt install -y ros-humble-rosbridge-suite
#
# If you have root on this machine, prefer that - it is one command and it pulls
# python3-bson, python3-tornado and cbor2 with it. This script exists because
# the demo laptop has no passwordless sudo, so rosbridge is built into its own
# overlay workspace instead. Both give the same 2.0.7 that Humble ships.
#
# Once the apt package is installed, delete ros2_ws_rosbridge and drop the
# `source .../ros2_ws_rosbridge/install/setup.bash` line from the start script -
# nothing else changes, the launch arguments are identical.

# No `set -u`: sourcing the ROS setup files trips nounset.
set -eo pipefail

BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$BASE/ros2_ws_rosbridge"

# The tag that matches Humble's ros-humble-rosbridge-suite. The ros2 branch head
# is 4.x and targets Jazzy/Rolling; it does not belong on this machine.
ROSBRIDGE_TAG="2.0.7"

echo "rosbridge workspace: $WS"
mkdir -p "$WS/src"

if [ ! -d "$WS/src/rosbridge_suite/.git" ]; then
  echo "Cloning rosbridge_suite $ROSBRIDGE_TAG ..."
  git clone --depth 1 --branch ros2 \
    https://github.com/RobotWebTools/rosbridge_suite.git "$WS/src/rosbridge_suite"
  git -C "$WS/src/rosbridge_suite" fetch --depth 1 origin tag "$ROSBRIDGE_TAG"
fi
git -C "$WS/src/rosbridge_suite" checkout -q "$ROSBRIDGE_TAG"
echo "rosbridge_suite at $(git -C "$WS/src/rosbridge_suite" describe --tags)"

# Python dependencies the apt package would have pulled in as system packages.
# rosbridge imports all three at module level, so a missing one is an immediate
# traceback on startup rather than a degraded feature.
echo "Installing Python dependencies (user site) ..."
pip3 install --user --quiet tornado pymongo cbor2
python3 - <<'PY'
import bson, cbor2, tornado
assert hasattr(bson, "BSON"), "wrong bson: rosbridge needs the MongoDB implementation (pymongo)"
print(f"  tornado {tornado.version}, bson ok, cbor2 ok")
PY

echo "Building ..."
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --cmake-args -DBUILD_TESTING=OFF

echo
echo "Built. The debug stack picks it up automatically:"
echo "  $BASE/scripts/start_octopus_debug_stack.sh"
echo
echo "Or start it alone:"
echo "  source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"
echo "  ros2 run rosbridge_server rosbridge_websocket --port 9090 --address 0.0.0.0 \\"
echo "    --topics_glob \"['/octopus/*']\" --services_glob \"[]\" --actions_glob \"[]\""
