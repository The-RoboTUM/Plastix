# Octopus Indoor Demo Checklist

This checklist starts the indoor static Eve → Octopus demo.

Demo goal:

```text
camera detection
→ confirmed ROS2 detection
→ Pixhawk attitude + manual height projection
→ Indoor Static Mission Map
→ Global Mission Grid
→ dashboard/backend output for the other robot
```

For the indoor demo, use:

```text
Mapping mode: Indoor Static Mission Map
Grid source: Global Mission Grid
Robot output: Global Mission Grid only
```

Do **not** use the Local Camera Grid as robot output. It is only a camera/debug view.

---

## 0. Safety and setup

Before starting:

```text
- Remove propellers.
- Secure the drone so it cannot move.
- Power Pixhawk and Raspberry Pi.
- Put a clearly visible trash/object item inside the camera view.
- Use good lighting.
- Do not run fake detections.
```

The correct Octopus start mode is:

```bash
OCTOPUS_MAPPING_MODE=indoor_static_mission
```

---

## Terminal 0 — optional laptop cleanup

Run this once before starting the demo if old processes may still be running:

```bash
cd ~/projects/PlastiX

./Octopus/scripts/stop_octopus_debug_stack.sh || true

pkill -f detector_node.py || true
pkill -f pub_fake_confirmed_transient || true
pkill -f "uvicorn api:app" || true
pkill -f "local_camera_grid_node" || true
pkill -f "flight_camera_transform_node" || true
pkill -f "grid_map_builder_node" || true
pkill -f "backend_bridge_node" || true

ros2 daemon stop || true
pkill -f "_ros2_daemon" || true
```

---

## Terminal 1 — MicroXRCEAgent

SSH into the Pi:

```bash
ssh eve-pi
```

On the Pi, start the PX4 ↔ ROS2 bridge:

```bash
sudo pkill -f MicroXRCEAgent || true

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

sudo --preserve-env=ROS_DOMAIN_ID,ROS_LOCALHOST_ONLY \
  MicroXRCEAgent serial --dev /dev/serial0 -b 921600 -v 6
```

Leave this terminal running.

Expected later:

```text
/fmu/out/vehicle_odometry publishes around 70–100 Hz
```

---

## Terminal 2 — Camera node

SSH into the Pi:

```bash
ssh eve-pi
```

On the Pi, start the camera node:

```bash
pkill -f camera_node || true

cd ~/PlastiX/eve/Software/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run camera_pkg camera_node --ros-args \
  -p publish_raw:=false \
  -p device_index:=0
```

Leave this terminal running.

Expected later:

```text
/camera/image_raw/compressed publishes around 7–15 Hz
```

---

## Terminal 3 — Detector

On the laptop:

```bash
cd ~/projects/PlastiX/eve/Software/detect-and-localize

source .venv/bin/activate
source /opt/ros/humble/setup.bash
source ~/projects/PlastiX/eve/Software/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

python ~/projects/PlastiX/eve/Software/ros2_ws/src/detection_pkg/detection_pkg/detector_node.py --ros-args \
  -p detect_localize_path:=/home/dominik/projects/PlastiX/eve/Software/detect-and-localize \
  -p model:=data/models/indoor_v8s.pt \
  -p input_topic:=/camera/image_raw/compressed \
  -p output_frame:=camera \
  -p show_ui:=false \
  -p thresh:=0.20 \
  -p confirm_frames:=1 \
  -p yolo_frameskip:=0 \
  -p dist_thresh:=0.10 \
  -p move_thresh:=0.10 \
  -p confirmed_republish_period_sec:=1.0
```

Leave this terminal running.

Expected later:

```text
/detector_node/confirmed publishes around 1 Hz
```

---

## Terminal 4 — Octopus indoor-static stack

On the laptop:

```bash
cd ~/projects/PlastiX

OCTOPUS_MAPPING_MODE=indoor_static_mission ./Octopus/scripts/start_octopus_debug_stack.sh
```

Open the dashboard:

```bash
xdg-open http://127.0.0.1:8000/dashboard.html
```

In the dashboard, select:

```text
Mapping Settings:
Mission mapping mode → Indoor Static Mission Map

Local Grid Map:
Grid source → Global Mission Grid
```

Leave this terminal running.

---

## Terminal 5 — Health check

On the laptop:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws

source /opt/ros/humble/setup.bash
source ~/projects/PlastiX/Octopus/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "---- camera ----"
timeout 8 ros2 topic hz /camera/image_raw/compressed

echo "---- px4 odometry ----"
timeout 8 ros2 topic hz /fmu/out/vehicle_odometry

echo "---- confirmed detections ----"
timeout 8 ros2 topic hz /detector_node/confirmed

echo "---- transform status ----"
ros2 topic echo --once /octopus/flight_camera_transform/status --field data | head -n 1 | python3 -m json.tool

echo "---- local camera grid ----"
curl -s http://127.0.0.1:8000/api/local_camera_grid/latest | python3 -m json.tool

echo "---- global map patch ----"
curl -s http://127.0.0.1:8000/api/map_patch/latest | python3 -m json.tool
```

Good result:

```text
/camera/image_raw/compressed: live
/fmu/out/vehicle_odometry: live
/detector_node/confirmed: live

"transform_mode": "indoor_static_mission"
"state": "ready"
"last_transformed_detection_count": 1
"last_projection_error": null

/api/local_camera_grid/latest: status ok
/api/map_patch/latest: status ok
```

---

## Check that the global map is live, not stale

Run:

```bash
cd ~/projects/PlastiX

echo "---- patch 1 ----"
curl -s http://127.0.0.1:8000/api/map_patch/latest | python3 -m json.tool | grep -E '"timestamp"|"received_at"|"x"|"y"'

sleep 5

echo "---- patch 2 ----"
curl -s http://127.0.0.1:8000/api/map_patch/latest | python3 -m json.tool | grep -E '"timestamp"|"received_at"|"x"|"y"'
```

Good result:

```text
timestamp changes after 5 seconds
received_at changes after 5 seconds
```

The x/y position may stay similar if the object does not move. That is fine.

---

## Demo explanation

Use this explanation:

```text
The detector outputs normalized image coordinates u/v. Octopus uses the calibrated camera model, manual camera height, and Pixhawk orientation to project the detection onto the ground plane.

Because this is an indoor static demo, the drone x/y origin is frozen so PX4 drift does not move the map. The result is written into the Global Mission Grid, which is the map that another robot can use.
```

---

## Robot-relevant outputs

The other robot should use the global outputs:

```text
/octopus/detections_world
/octopus/global_map
/octopus/coverage_grid
/octopus/trash_grid
```

Backend debug endpoint:

```text
/api/map_patch/latest
```

Do not use this for robot navigation:

```text
/api/local_camera_grid/latest
Local Camera Grid
camera_footprint frame
```

That is only for camera/debug visualization.

---

## Common problems

### `/fmu/out/vehicle_odometry` not publishing

Restart Terminal 1:

```bash
sudo pkill -f MicroXRCEAgent || true

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

sudo --preserve-env=ROS_DOMAIN_ID,ROS_LOCALHOST_ONLY \
  MicroXRCEAgent serial --dev /dev/serial0 -b 921600 -v 6
```

### `/detector_node/confirmed` not publishing

The detector is running, but no object is becoming confirmed.

Use:

```bash
-p thresh:=0.20
-p confirm_frames:=1
```

Make sure the object is visible and well lit.

### `/api/local_camera_grid/latest` empty

Usually means:

```text
/detector_node/confirmed is not publishing
```

### `/api/map_patch/latest` empty

Usually means one of these is missing:

```text
/detector_node/confirmed
/fmu/out/vehicle_odometry
flight_camera_transform_node ready state
```

Check:

```bash
ros2 topic echo --once /octopus/flight_camera_transform/status --field data | head -n 1 | python3 -m json.tool
```

### Status command shows `Extra data`

Use this safer version:

```bash
ros2 topic echo --once /octopus/flight_camera_transform/status --field data | head -n 1 | python3 -m json.tool
```

---

## Successful demo checklist

A successful indoor demo has:

```text
[ ] Camera stream live
[ ] PX4 odometry live
[ ] Detector confirmed topic live
[ ] transform_mode = indoor_static_mission
[ ] Transform state = ready
[ ] Local camera grid endpoint = status ok
[ ] Global map patch endpoint = status ok
[ ] Global map timestamp updates live
[ ] Dashboard shows Global Mission Grid
[ ] No fake detections running
```
