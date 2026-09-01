#!/usr/bin/env bash
# Laeuft die PX4-Bruecke? Antwort geht an GET /api/eve/px4_bridge/status, das
# allein auf die Woerter px4_bridge_running / px4_bridge_not_running schaut.

source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/octopus_px4_bridge_env.sh"

PIDS=$(pgrep -f "$MATCH" || true)

if [ -n "$PIDS" ]; then
    echo "px4_bridge_running pids=$(echo "$PIDS" | tr '\n' ' ')"
else
    echo "px4_bridge_not_running"
fi
