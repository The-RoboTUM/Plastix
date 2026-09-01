#!/usr/bin/env bash
# Gemeinsame Pfade der drei octopus_*camera*-Skripte. Wird von allen dreien
# gesourct und ist selbst nicht zum Ausfuehren gedacht.

# Workspace: EVE_WS schlaegt alles. Sonst der ros2_ws neben diesem Skript im
# Repo (auch wenn der Aufrufer ein Symlink in ~ ist - readlink -f loest ihn
# auf). Letzter Ausweg ist der Standardpfad auf der Pi, falls die Skripte nach
# ~ kopiert statt verlinkt wurden.
if [ -z "${EVE_WS:-}" ]; then
    _octopus_env_self="$(readlink -f "${BASH_SOURCE[0]}")"
    _octopus_ws_candidate="$(dirname "$_octopus_env_self")/../ros2_ws"

    if [ -d "$_octopus_ws_candidate/install" ]; then
        EVE_WS="$(cd "$_octopus_ws_candidate" && pwd -P)"
    else
        EVE_WS="$HOME/PlastiX/eve/Software/ros2_ws"
    fi

    unset _octopus_env_self _octopus_ws_candidate
fi

# pgrep -f vergleicht gegen die Kommandozeile des laufenden Nodes, das ist
# genau dieser absolute Pfad. Stimmt er nicht, meldet der Start camera_failed,
# obwohl der Node laeuft.
MATCH="$EVE_WS/install/camera_pkg/lib/camera_pkg/camera_node"

# Das Dashboard liest genau diese Datei (GET /api/eve/camera_log).
LOG_FILE=/tmp/octopus_camera_node.log
PID_FILE=/tmp/octopus_camera_node.pid
