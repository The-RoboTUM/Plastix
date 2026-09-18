#!/usr/bin/env bash
# Stoppt den Detektor. Antwort geht an POST /api/detector/stop, das auf
# detector_stopped bzw. detector_not_running schaut.

source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/octopus_detector_env.sh"

PIDS=$(pgrep -f "$MATCH" || true)

if [ -n "$PIDS" ]; then
    echo "Stopping detector pids=$(echo "$PIDS" | tr '\n' ' ')"
    echo "$PIDS" | xargs -r kill || true
    # Mehr Geduld als bei den anderen Nodes: der Detektor haengt in einer
    # laufenden YOLO-Inferenz und raeumt danach torch ab, das dauert.
    for _ in 1 2 3 4 5; do
        sleep 1
        [ -z "$(pgrep -f "$MATCH" || true)" ] && break
    done

    STILL_RUNNING=$(pgrep -f "$MATCH" || true)
    if [ -n "$STILL_RUNNING" ]; then
        echo "Force stopping detector pids=$(echo "$STILL_RUNNING" | tr '\n' ' ')"
        echo "$STILL_RUNNING" | xargs -r kill -9 || true
    fi

    rm -f "$PID_FILE"
    echo "detector_stopped"
else
    rm -f "$PID_FILE"
    echo "detector_not_running"
fi
