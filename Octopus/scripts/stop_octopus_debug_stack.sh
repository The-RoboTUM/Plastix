#!/usr/bin/env bash

echo "Stopping Octopus debug stack..."
pkill -f "flight_camera_transform_node" || true
pkill -f "world_posearray_to_json_bridge_node" || true
pkill -f "grid_map_builder_node" || true
pkill -f "map_patch_backend_bridge_node" || true
pkill -f "camera_debug_backend_bridge_node" || true
pkill -f "uvicorn api:app" || true
echo "Stopped."
