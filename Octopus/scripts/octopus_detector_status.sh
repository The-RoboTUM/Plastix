#!/usr/bin/env bash
# Laeuft der Detektor? Antwort geht an GET /api/detector/status, das allein auf
# die Woerter detector_running / detector_not_running schaut.

source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/octopus_detector_env.sh"

PIDS=$(pgrep -f "$MATCH" || true)

if [ -z "$PIDS" ]; then
    echo "detector_not_running"
    exit 0
fi

# YOLO laedt beim Start ein paar Sekunden. In der Zeit laeuft der Prozess
# schon, publiziert aber noch nichts -- das Dashboard soll das auseinander
# halten koennen, sonst sieht ein normaler Start wie ein Fehler aus.
if grep -q "Detector ready" "$LOG_FILE" 2>/dev/null; then
    echo "detector_running pids=$(echo "$PIDS" | tr '\n' ' ') model=loaded"
else
    echo "detector_running pids=$(echo "$PIDS" | tr '\n' ' ') model=loading"
fi
