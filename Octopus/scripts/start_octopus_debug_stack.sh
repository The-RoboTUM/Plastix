#!/usr/bin/env bash
set -e

BASE="$HOME/projects/PlastiX"
ROS_WS="$BASE/Octopus/ros2_ws"
DASH="$BASE/Octopus/octopus-dashboard"
LOG_DIR="/tmp/octopus_logs"

mkdir -p "$LOG_DIR"

echo "Stopping old Octopus debug stack processes..."
pkill -f "flight_camera_transform_node" || true
pkill -f "world_posearray_to_json_bridge_node" || true
pkill -f "grid_map_builder_node" || true
pkill -f "map_patch_backend_bridge_node" || true
pkill -f "camera_debug_backend_bridge_node" || true
pkill -f "uvicorn api:app" || true

sleep 1

echo "Starting FastAPI dashboard backend..."
cd "$DASH"
python3 -m uvicorn api:app --host 0.0.0.0 --port 8000 > "$LOG_DIR/backend.log" 2>&1 &
echo $! > "$LOG_DIR/backend.pid"

sleep 2

echo "Starting Octopus ROS nodes..."
cd "$ROS_WS"

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run octopus_camera_transform flight_camera_transform_node --ros-args \
  -p projection_enabled:=true \
  -p require_local_xy_valid:=false \
  -p require_local_z_valid:=false \
  -p use_dist_bottom_if_valid:=false \
  -p ground_z_ned:=3.5 \
  -p pose_stale_sec:=10.0 \
  -p output_topic:=/octopus/detections_world_pose \
  > "$LOG_DIR/flight_camera_transform.log" 2>&1 &
echo $! > "$LOG_DIR/flight_camera_transform.pid"

ros2 run octopus_camera_transform world_posearray_to_json_bridge_node --ros-args \
  -p relative_mode:=true \
  -p relative_origin_x:=2.5 \
  -p relative_origin_y:=1.5 \
  > "$LOG_DIR/world_posearray_to_json_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/world_posearray_to_json_bridge.pid"

ros2 run octopus_mapping grid_map_builder_node --ros-args \
  -p width_m:=4.46 \
  -p height_m:=3.34 \
  -p resolution:=0.10 \
  -p origin_x:=0.0 \
  -p origin_y:=0.0 \
  > "$LOG_DIR/grid_map_builder.log" 2>&1 &
echo $! > "$LOG_DIR/grid_map_builder.pid"

ros2 run octopus_backend_bridge map_patch_backend_bridge_node \
  > "$LOG_DIR/map_patch_backend_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/map_patch_backend_bridge.pid"

ros2 run octopus_backend_bridge camera_debug_backend_bridge_node \
  > "$LOG_DIR/camera_debug_backend_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/camera_debug_backend_bridge.pid"

echo ""
echo "Started Octopus debug stack."
echo "Logs: $LOG_DIR"
echo ""
echo "Open dashboard:"
echo "http://127.0.0.1:8000/dashboard.html"
echo ""
echo "Run health check:"
echo "python3 $BASE/Octopus/scripts/octopus_pipeline_health.py"
