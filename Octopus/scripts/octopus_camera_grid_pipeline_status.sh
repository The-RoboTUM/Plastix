#!/usr/bin/env bash

status_node() {
    NAME="$1"
    PATTERN="$2"

    PIDS=$(pgrep -f "$PATTERN" || true)

    if [ -n "$PIDS" ]; then
        echo "$NAME=running pids=$(echo "$PIDS" | tr '\n' ' ')"
    else
        echo "$NAME=not_running"
    fi
}

status_node "grid_map_builder" "grid_map_builder_node"
status_node "map_patch_backend_bridge" "map_patch_backend_bridge_node"
status_node "camera_marker_transform" "camera_marker_transform_node"
status_node "camera_transform_status_backend_bridge" "camera_transform_status_backend_bridge_node"
