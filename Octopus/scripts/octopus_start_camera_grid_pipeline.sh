#!/usr/bin/env bash
set -e

WS="$HOME/projects/PlastiX/Octopus/ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

start_node() {
    NAME="$1"
    PATTERN="$2"
    COMMAND="$3"
    LOG_FILE="$4"

    if pgrep -f "$PATTERN" >/dev/null 2>&1; then
        echo "$NAME already_running pids=$(pgrep -f "$PATTERN" | tr '\n' ' ')"
        return
    fi

    echo "Starting $NAME..."
    nohup bash -lc "
        source /opt/ros/humble/setup.bash
        source '$WS/install/setup.bash'
        export ROS_DOMAIN_ID=0
        export ROS_LOCALHOST_ONLY=0
        $COMMAND
    " > "$LOG_FILE" 2>&1 < /dev/null &

    sleep 1

    if pgrep -f "$PATTERN" >/dev/null 2>&1; then
        echo "$NAME started pids=$(pgrep -f "$PATTERN" | tr '\n' ' ')"
    else
        echo "$NAME failed"
        echo "--- $LOG_FILE ---"
        tail -40 "$LOG_FILE" || true
        exit 1
    fi
}

start_node \
    "grid_map_builder" \
    "grid_map_builder_node" \
    "ros2 run octopus_mapping grid_map_builder_node" \
    "/tmp/octopus_grid_map_builder.log"

start_node \
    "map_patch_backend_bridge" \
    "map_patch_backend_bridge_node" \
    "ros2 run octopus_backend_bridge map_patch_backend_bridge_node" \
    "/tmp/octopus_map_patch_backend_bridge.log"

start_node \
    "camera_marker_transform" \
    "camera_marker_transform_node" \
    "ros2 run octopus_camera_transform camera_marker_transform_node" \
    "/tmp/octopus_camera_marker_transform.log"

echo "camera_grid_pipeline_started"
