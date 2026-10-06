#!/usr/bin/env bash
# Startet den YOLO-Detektor im Hintergrund, ohne offenes Terminal. Antwort geht
# an POST /api/detector/start, das auf detector_started schaut.
#
# Ersetzt Terminal 3 aus docs/SETUP.md. Die Parameter sind dieselben; abweichen
# kann man ueber die DETECTOR_*-Variablen in octopus_detector_env.sh.
set -e

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$SCRIPT_DIR/octopus_detector_env.sh"

if [ ! -f "$DETECT_DIR/.venv/bin/activate" ]; then
    echo "detector_failed: kein venv unter $DETECT_DIR/.venv"
    echo "Anlegen: cd $DETECT_DIR && python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt"
    exit 1
fi

if [ ! -f "$DETECT_DIR/$DETECTOR_MODEL" ]; then
    echo "detector_failed: Modell fehlt: $DETECT_DIR/$DETECTOR_MODEL"
    echo "Die Modelle sind per .gitignore ausgeschlossen und muessen separat kopiert werden."
    echo "Vorhanden:"
    ls -1 "$DETECT_DIR/data/models/" 2>/dev/null || echo "  (kein data/models/)"
    exit 1
fi

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
    echo "detector_failed: Workspace nicht gebaut ($ROS_WS/install fehlt)"
    echo "Bauen: cd $ROS_WS && source /opt/ros/humble/setup.bash && colcon build --symlink-install"
    exit 1
fi

echo "Stopping old detector if needed..."
"$SCRIPT_DIR/octopus_stop_detector.sh" || true

echo "Using model: $DETECTOR_MODEL"
echo "Input topic: $DETECTOR_INPUT_TOPIC"

# Reihenfolge der beiden Quellen ist wichtig und in SETUP.md begruendet: rclpy
# kommt aus /opt/ros, alles andere aus dem venv. Erst venv, dann ROS -- ROS
# haengt sich per PYTHONPATH dazu, statt das venv zu ersetzen.
nohup bash -lc "
    cd '$DETECT_DIR'
    source .venv/bin/activate
    source /opt/ros/humble/setup.bash
    source '$ROS_WS/install/setup.bash'
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
    exec python '$ROS_WS/src/detection_pkg/detection_pkg/detector_node.py' --ros-args \
      -p detect_localize_path:='$DETECT_DIR' \
      -p model:='$DETECTOR_MODEL' \
      -p input_topic:='$DETECTOR_INPUT_TOPIC' \
      -p output_frame:=camera \
      -p show_ui:=false \
      -p thresh:=$DETECTOR_THRESH \
      -p confirm_frames:=$DETECTOR_CONFIRM_FRAMES \
      -p max_lost:=$DETECTOR_MAX_LOST \
      -p yolo_frameskip:=0 \
      -p dist_thresh:=0.10 \
      -p move_thresh:=0.10 \
      -p confirmed_republish_period_sec:=1.0 \
      -p debug_image_jpeg_quality:=$DETECTOR_JPEG_QUALITY
" > "$LOG_FILE" 2>&1 < /dev/null &

# Nur pruefen, dass der Prozess steht -- nicht, dass YOLO fertig geladen hat.
# Das dauert je nach Rechner deutlich laenger, und ein Start-Endpunkt, der 30 s
# blockiert, laeuft im Dashboard in den Timeout. Den Ladezustand meldet
# stattdessen octopus_detector_status.sh als model=loading/loaded.
sleep 3

PIDS=$(pgrep -f "$MATCH" || true)

if [ -z "$PIDS" ]; then
    echo "detector_failed"
    echo "--- $LOG_FILE ---"
    tail -40 "$LOG_FILE" || true
    exit 1
fi

echo "$PIDS" > "$PID_FILE"
echo "detector_started pids=$(echo "$PIDS" | tr '\n' ' ')"
