#!/usr/bin/env bash
set -e

# Same pattern as run_rosbridge.sh / build_rosbridge.sh: BASE is the Octopus
# directory, derived from where THIS script lives. No $HOME, no assumption about
# where the repo is checked out - so a git worktree works without edits.
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$BASE/ros2_ws"
DASH="$BASE/octopus-dashboard"
LOG_DIR="/tmp/octopus_logs"

OCTOPUS_MAPPING_MODE="${OCTOPUS_MAPPING_MODE:-flight_global_mission}"
if [ "$OCTOPUS_MAPPING_MODE" = "indoor_static_mission" ]; then
  OCTOPUS_JSON_RELATIVE_MODE=false
else
  OCTOPUS_JSON_RELATIVE_MODE=true
fi

echo "Octopus mapping mode: $OCTOPUS_MAPPING_MODE"
echo "Octopus JSON relative mode: $OCTOPUS_JSON_RELATIVE_MODE"


mkdir -p "$LOG_DIR"

echo "Stopping old Octopus debug stack processes..."
pkill -f "flight_camera_transform_node" || true
pkill -f "world_posearray_to_json_bridge_node" || true
pkill -f "grid_map_builder_node" || true
pkill -f "map_patch_backend_bridge_node" || true
pkill -f "camera_debug_backend_bridge_node" || true
pkill -f "local_camera_grid_backend_bridge_node" || true
pkill -f "local_camera_grid_node" || true
pkill -f "camera_transform_status_backend_bridge_node" || true
pkill -f "trash_gps_goal_node" || true
pkill -f "device_status_backend_bridge_node" || true
pkill -f "rosbridge_websocket" || true
pkill -f "eve_fake_gps_bridge_node" || true
pkill -f "uvicorn api:app" || true

sleep 1

echo "Starting FastAPI dashboard backend..."
cd "$DASH"
setsid python3 -m uvicorn api:app --host 0.0.0.0 --port 8000 > "$LOG_DIR/backend.log" 2>&1 < /dev/null &
echo $! > "$LOG_DIR/backend.pid"

sleep 2

echo "Starting Octopus ROS nodes..."
cd "$ROS_WS"

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

setsid ros2 run octopus_camera_transform flight_camera_transform_node --ros-args \
  -p projection_enabled:=true \
  -p require_local_xy_valid:=false \
  -p require_local_z_valid:=false \
  -p use_dist_bottom_if_valid:=false \
  -p use_manual_height_above_ground:=true \
  -p manual_height_above_ground_m:=2.5 \
  -p transform_mode:=${OCTOPUS_MAPPING_MODE} \
  -p indoor_static_origin_x:=0.0 \
  -p indoor_static_origin_y:=0.0 \
  -p indoor_static_align_yaw_on_start:=true \
  -p indoor_static_map_yaw_offset_rad:=1.57079632679 \
  -p ground_z_ned:=0.0 \
  -p pose_stale_sec:=10.0 \
  -p output_topic:=/octopus/detections_world_pose \
  > "$LOG_DIR/flight_camera_transform.log" 2>&1 &
echo $! > "$LOG_DIR/flight_camera_transform.pid"

setsid ros2 run octopus_camera_transform world_posearray_to_json_bridge_node --ros-args \
  -p relative_mode:=${OCTOPUS_JSON_RELATIVE_MODE} \
  -p relative_origin_x:=2.5 \
  -p relative_origin_y:=1.5 \
  > "$LOG_DIR/world_posearray_to_json_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/world_posearray_to_json_bridge.pid"

echo "Starting local camera grid node..."
setsid ros2 run octopus_camera_transform local_camera_grid_node --ros-args \
  -p footprint_width_m:=4.46 \
  -p footprint_height_m:=3.34 \
  -p resolution_m:=0.10 \
  > "$LOG_DIR/local_camera_grid.log" 2>&1 &

# origin_x/y ist die Ecke des Grids, nicht sein Mittelpunkt. Die Drohne sitzt jetzt
# auf map (0, 0), also muss die Ecke auf minus die halbe Kantenlänge -- sonst fällt
# jede negative Koordinate aus dem Grid. Abgedeckt wird dieselbe Fläche wie vorher.
setsid ros2 run octopus_mapping grid_map_builder_node --ros-args \
  -p width_m:=4.46 \
  -p height_m:=3.34 \
  -p resolution:=0.10 \
  -p origin_x:=-2.23 \
  -p origin_y:=-1.67 \
  > "$LOG_DIR/grid_map_builder.log" 2>&1 &
echo $! > "$LOG_DIR/grid_map_builder.pid"

setsid ros2 run octopus_backend_bridge map_patch_backend_bridge_node \
  > "$LOG_DIR/map_patch_backend_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/map_patch_backend_bridge.pid"

echo "Starting local camera grid backend bridge node..."
setsid ros2 run octopus_backend_bridge local_camera_grid_backend_bridge_node \
  > "$LOG_DIR/local_camera_grid_backend_bridge.log" 2>&1 &

setsid ros2 run octopus_backend_bridge camera_debug_backend_bridge_node \
  > "$LOG_DIR/camera_debug_backend_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/camera_debug_backend_bridge.pid"

echo "Starting Eve fake GPS bridge node..."
setsid ros2 run octopus_backend_bridge eve_fake_gps_bridge_node \
  > "$LOG_DIR/eve_fake_gps_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/eve_fake_gps_bridge.pid"

echo "Starting trash GPS goal node..."
# target_ttl_sec 2 s, nicht 1 s: der Detector bestätigt mit 1 Hz (gemessener Abstand
# 1.01 s), ein knapperer Wert lässt sichtbare Ziele im Takt verfallen und registriert
# sie jede Runde unter neuer id. Dieselbe 2x-Reserve, die GripperX für
# max_target_list_age_sec gewählt hat.
setsid ros2 run octopus_camera_transform trash_gps_goal_node --ros-args \
  -p max_radius_m:=1.25 \
  -p target_ttl_sec:=2.0 \
  > "$LOG_DIR/trash_gps_goal.log" 2>&1 &
echo $! > "$LOG_DIR/trash_gps_goal.pid"

echo "Starting device status backend bridge node..."
setsid ros2 run octopus_backend_bridge device_status_backend_bridge_node \
  > "$LOG_DIR/device_status_backend_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/device_status_backend_bridge.pid"

# rosbridge carries the GripperX link: the contract topics in and out over
# ws://<host>:9090. The arguments live in run_rosbridge.sh so this script, the
# systemd unit and a manual run cannot drift apart.
echo "Starting rosbridge for the GripperX link..."
setsid "$BASE/scripts/run_rosbridge.sh" > "$LOG_DIR/rosbridge.log" 2>&1 &
echo $! > "$LOG_DIR/rosbridge.pid"

echo "Starting camera transform status backend bridge node..."
setsid ros2 run octopus_backend_bridge camera_transform_status_backend_bridge_node \
  > "$LOG_DIR/camera_transform_status_backend_bridge.log" 2>&1 &
echo $! > "$LOG_DIR/camera_transform_status_backend_bridge.pid"

echo ""
echo "Started Octopus debug stack."
echo "Logs: $LOG_DIR"
echo ""
echo "Open dashboard:"
echo "http://127.0.0.1:8000/dashboard.html"
echo ""
echo "GripperX link (rosbridge):"
echo "ws://$(hostname -I 2>/dev/null | awk '{print $1}'):9090"
echo ""
echo "Check the link:"
echo "python3 $BASE/scripts/check_rosbridge.py"
echo ""
echo "Run health check:"
echo "python3 $BASE/scripts/octopus_pipeline_health.py"
