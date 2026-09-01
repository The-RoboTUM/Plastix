#!/usr/bin/env bash
# Laeuft der camera_node? Antwort geht an GET /api/eve/status im Dashboard,
# das allein auf die Woerter camera_running / camera_not_running schaut.

source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/octopus_camera_env.sh"

PIDS=$(pgrep -f "$MATCH" || true)

if [ -n "$PIDS" ]; then
    echo "camera_running pids=$PIDS"
else
    echo "camera_not_running"
fi
