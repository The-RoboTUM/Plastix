#!/usr/bin/env bash
# Gemeinsame Werte der drei octopus_*detector*-Skripte. Wird von allen dreien
# gesourct und ist selbst nicht zum Ausfuehren gedacht.

# BASE ist der Octopus-Ordner, abgeleitet aus dem Ort dieses Skripts -- kein
# $HOME, keine Annahme ueber den Checkout-Pfad, damit ein Worktree ohne
# Aenderung funktioniert. Dasselbe Muster wie start_octopus_debug_stack.sh.
BASE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
DETECT_DIR="$BASE/detect-and-localize"
ROS_WS="$BASE/ros2_ws"

# pgrep-Muster. Der Detektor laeuft als "python .../detector_node.py", nicht als
# ros2-run-Executable, deshalb wird auf den Dateinamen gematcht. Die Klammer
# verhindert, dass das Muster sich selbst trifft.
MATCH='[d]etector_node.py'

DETECTOR_MODEL="${DETECTOR_MODEL:-data/models/best_model_10_08_26.pt}"
DETECTOR_INPUT_TOPIC="${DETECTOR_INPUT_TOPIC:-/camera/image_raw/compressed}"
DETECTOR_THRESH="${DETECTOR_THRESH:-0.60}"
DETECTOR_CONFIRM_FRAMES="${DETECTOR_CONFIRM_FRAMES:-3}"
DETECTOR_MAX_LOST="${DETECTOR_MAX_LOST:-5}"
DETECTOR_JPEG_QUALITY="${DETECTOR_JPEG_QUALITY:-80}"

LOG_FILE=/tmp/octopus_detector.log
PID_FILE=/tmp/octopus_detector.pid
