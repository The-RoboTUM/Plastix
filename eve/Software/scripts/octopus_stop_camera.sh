#!/usr/bin/env bash
# Stoppt den camera_node. Antwort geht an POST /api/eve/stop_camera, das auf
# camera_stopped bzw. camera_not_running schaut.

source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/octopus_camera_env.sh"

PIDS=$(pgrep -f "$MATCH" || true)

if [ -n "$PIDS" ]; then
    echo "Stopping camera_node pids=$PIDS"
    echo "$PIDS" | xargs -r kill || true
    sleep 1

    STILL_RUNNING=$(pgrep -f "$MATCH" || true)
    if [ -n "$STILL_RUNNING" ]; then
        echo "Force stopping camera_node pids=$STILL_RUNNING"
        echo "$STILL_RUNNING" | xargs -r kill -9 || true
    fi

    rm -f "$PID_FILE"
    echo "camera_stopped"
else
    rm -f "$PID_FILE"
    echo "camera_not_running"
fi
