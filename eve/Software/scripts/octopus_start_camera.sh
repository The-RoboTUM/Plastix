#!/usr/bin/env bash
# Startet den camera_node im Hintergrund, ohne sudo und ohne offenes Terminal.
# Antwort geht an POST /api/eve/start_camera, das auf camera_started schaut.
set -e

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$SCRIPT_DIR/octopus_camera_env.sh"

cd "$EVE_WS"

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "Using workspace: $EVE_WS"
echo "Stopping old camera process if needed..."
"$SCRIPT_DIR/octopus_stop_camera.sh" || true

echo "Searching for USB camera..."

CAMERA_DEVICE=""

# Prefer stable USB camera path - /dev/videoX kann sich zwischen zwei Boots
# verschieben, /dev/v4l/by-id nicht.
for p in /dev/v4l/by-id/*video-index0; do
    if [ -e "$p" ]; then
        CAMERA_DEVICE="$(readlink -f "$p")"
        break
    fi
done

# Fallback: find first /dev/videoX device that supports MJPG.
if [ -z "$CAMERA_DEVICE" ]; then
    for dev in /dev/video*; do
        if v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | grep -q "MJPG"; then
            CAMERA_DEVICE="$dev"
            break
        fi
    done
fi

if [ -z "$CAMERA_DEVICE" ]; then
    echo "camera_failed: no usable USB camera found"
    echo "--- /dev/video* ---"
    ls -l /dev/video* 2>/dev/null || true
    echo "--- v4l2 devices ---"
    v4l2-ctl --list-devices || true
    exit 1
fi

DEVICE_BASENAME="$(basename "$CAMERA_DEVICE")"
DEVICE_INDEX="${DEVICE_BASENAME#video}"

echo "Using camera device: $CAMERA_DEVICE"
echo "Using device_index: $DEVICE_INDEX"

# 640x480 haengt an den Kamera-Intrinsics auf der Octopus-Seite
# (OCTOPUS_HBVCAM_640X480 in octopus-dashboard/live_data.js). Aendern nur,
# wenn die Kamera dabei denselben Bildwinkel behaelt, sonst stimmen Footprint,
# Grid und Projektion nicht mehr.
nohup ros2 run camera_pkg camera_node --ros-args \
  -p device_index:="$DEVICE_INDEX" \
  -p frame_width:=640 \
  -p frame_height:=480 \
  -p frame_rate:=30.0 \
  -p verbose:=true \
  > "$LOG_FILE" 2>&1 < /dev/null &

sleep 2

PIDS=$(pgrep -f "$MATCH" || true)

if [ -n "$PIDS" ]; then
    echo "$PIDS" > "$PID_FILE"
    echo "camera_started pids=$PIDS device=$CAMERA_DEVICE index=$DEVICE_INDEX"
else
    echo "camera_failed"
    echo "--- camera log ---"
    tail -80 "$LOG_FILE" || true
    exit 1
fi
