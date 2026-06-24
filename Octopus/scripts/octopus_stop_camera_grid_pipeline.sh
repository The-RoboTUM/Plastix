#!/usr/bin/env bash

stop_node() {
    NAME="$1"
    PATTERN="$2"

    PIDS=$(pgrep -f "$PATTERN" || true)

    if [ -n "$PIDS" ]; then
        echo "Stopping $NAME pids=$PIDS"
        echo "$PIDS" | xargs -r kill || true
        sleep 1

        STILL=$(pgrep -f "$PATTERN" || true)
        if [ -n "$STILL" ]; then
            echo "Force stopping $NAME pids=$STILL"
            echo "$STILL" | xargs -r kill -9 || true
        fi

        echo "$NAME stopped"
    else
        echo "$NAME not_running"
    fi
}

stop_node "camera_marker_transform" "camera_marker_transform_node"
stop_node "map_patch_backend_bridge" "map_patch_backend_bridge_node"
stop_node "grid_map_builder" "grid_map_builder_node"

echo "camera_grid_pipeline_stopped"
