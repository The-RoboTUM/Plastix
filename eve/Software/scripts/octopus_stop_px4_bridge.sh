#!/usr/bin/env bash
# Stoppt die PX4-Bruecke. Antwort geht an POST /api/eve/px4_bridge/stop, das auf
# px4_bridge_stopped bzw. px4_bridge_not_running schaut.

source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/octopus_px4_bridge_env.sh"

PIDS=$(pgrep -f "$MATCH" || true)

if [ -n "$PIDS" ]; then
    echo "Stopping MicroXRCEAgent pids=$(echo "$PIDS" | tr '\n' ' ')"
    echo "$PIDS" | xargs -r kill || true
    sleep 1

    STILL_RUNNING=$(pgrep -f "$MATCH" || true)
    if [ -n "$STILL_RUNNING" ]; then
        echo "Force stopping MicroXRCEAgent pids=$(echo "$STILL_RUNNING" | tr '\n' ' ')"
        echo "$STILL_RUNNING" | xargs -r kill -9 || true
        sleep 1
    fi

    rm -f "$PID_FILE"
    echo "px4_bridge_stopped"
else
    rm -f "$PID_FILE"
    echo "px4_bridge_not_running"
fi
