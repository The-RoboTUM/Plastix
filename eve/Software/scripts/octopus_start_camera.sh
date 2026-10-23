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

# Aufloesung: 640x480. Die ganze Octopus-Seite rechnet mit genau diesem
# Kameramodell: flight_camera_transform_node (Projektion der Muellziele),
# line_calibration_node, local_camera_grid_node (Footprint 4.46 x 3.34 m) und
# OCTOPUS_HBVCAM_640X480 in octopus-dashboard/live_data.js. Die Werte stammen
# aus einer echten Schachbrett-Kalibrierung
# (Octopus/config/camera_calibration/hbvcam_5m2134_640x480.yaml).
#
# Vom 2026-10-07 bis 2026-10-19 lief die Kamera auf 1280x720. Das Octopus-
# Modell passte dann nicht mehr: gleicher horizontaler Bildwinkel, aber
# vertikal beschnitten, also wurde jede Strecke quer zur Bildbreite um ~1/3 zu
# lang projiziert (bis ~40 cm daneben am Bildrand bei 2.5 m). Zurueck auf
# 640x480, weil das ohne Codeaenderung alles wieder stimmig macht, die
# Kalibrierung echt statt abgeleitet ist, die Kamera 4.45 x 3.34 m statt
# 4.45 x 2.51 m Boden sieht und 15 statt 8 Hz liefert. Das Dashboard warnt
# jetzt, wenn Bildformat und Kameramodell auseinanderlaufen.
#
# Wer 1280x720 will, muss das Octopus-Kameramodell mitziehen. Referenz dafuer:
# an einem statischen Bildpaar gemessen ist der horizontale Bildwinkel gleich
# (Stangenabstand/Bildbreite 0.6291 vs 0.6294), 16:9 entsteht durch zentrischen
# vertikalen Beschnitt von 120 Zeilen (gemessen 118.8). Daraus abgeleitet,
# nicht kalibriert: fx=718.658 fy=718.458 cx=625.641 cy=355.895.
#
# Override fuer Tests: CAM_W=1280 CAM_H=720 ./octopus_start_camera.sh
CAM_W="${CAM_W:-640}"
CAM_H="${CAM_H:-480}"

echo "Using resolution: ${CAM_W}x${CAM_H}"

nohup ros2 run camera_pkg camera_node --ros-args \
  -p device_index:="$DEVICE_INDEX" \
  -p frame_width:=$CAM_W \
  -p frame_height:=$CAM_H \
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
