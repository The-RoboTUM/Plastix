#!/usr/bin/env bash

echo "Stopping Octopus debug stack..."
pkill -f "flight_camera_transform_node" || true
pkill -f "world_posearray_to_json_bridge_node" || true
pkill -f "grid_map_builder_node" || true
pkill -f "map_patch_backend_bridge_node" || true
pkill -f "camera_debug_backend_bridge_node" || true
pkill -f "camera_transform_status_backend_bridge_node" || true
pkill -f "trash_gps_goal_node" || true
pkill -f "eve_fake_gps_bridge_node" || true
pkill -f "device_status_backend_bridge_node" || true
pkill -f "rosbridge_websocket" || true
pkill -f "uvicorn api:app" || true
echo "Stopped."

pkill -f "local_camera_grid_node" || true

pkill -f "local_camera_grid_backend_bridge_node" || true
