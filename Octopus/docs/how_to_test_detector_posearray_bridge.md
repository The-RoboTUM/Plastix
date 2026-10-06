# How to Test Detector PoseArray Bridge

> **Paths.** Every path in this document is relative to the repository root — `cd` there first, in *every* terminal these steps open.

This test checks the detector-to-Octopus bridge.

## Pipeline

```text
fake detector PoseArray
-> /detector_node/confirmed
-> detector_posearray_bridge_node
-> /octopus/detections_world
-> grid_map_builder_node
-> /octopus/map_patch
-> map_patch_backend_bridge_node
-> FastAPI /api/map_patch
-> dashboard Latest Map Patch panel
```

## 0. Build packages

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  octopus_detector_bridge \
  octopus_mapping \
  octopus_backend_bridge

source install/setup.bash

ros2 pkg executables octopus_detector_bridge
```

Expected:

```text
octopus_detector_bridge detector_posearray_bridge_node
```

## 1. Terminal 1: Start backend

```bash
cd Octopus/octopus-dashboard
python3 -m uvicorn api:app --reload
```

Open:

```text
http://127.0.0.1:8000/dashboard.html
```

## 2. Terminal 2: Start map builder

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run octopus_mapping grid_map_builder_node
```

## 3. Terminal 3: Start backend bridge

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run octopus_backend_bridge map_patch_backend_bridge_node
```

## 4. Terminal 4: Start detector PoseArray bridge

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run octopus_detector_bridge detector_posearray_bridge_node
```

Expected:

```text
Detector PoseArray bridge started
Input topic: /detector_node/confirmed
Output topic: /octopus/detections_world
Input coordinate mode: map
```

## 5. Terminal 5: Publish fake detector output

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /detector_node/confirmed geometry_msgs/msg/PoseArray \
"{header: {frame_id: 'map'}, poses: [{position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}]}"
```

Expected result:

```text
detector bridge:
  Published 1 detection(s) to /octopus/detections_world

map builder:
  Published map patch with 1 updated cell(s)

backend bridge:
  Posted map patch to backend with 1 updated cell(s)

dashboard:
  Latest Map Patch shows cell 10,20
```

Check backend directly:

```bash
curl http://127.0.0.1:8000/api/map_patch/latest
```

Expected values:

```text
source_id: detector_node
row: 10
col: 20
trash_probability: 1.0
confidence: 1.0
```

## 6. Safety test: normalized image mode

Start the detector bridge in normalized image mode:

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run octopus_detector_bridge detector_posearray_bridge_node --ros-args \
  -p input_coordinate_mode:=normalized_image
```

Publish normalized image coordinates:

```bash
ros2 topic pub --once /detector_node/confirmed geometry_msgs/msg/PoseArray \
"{header: {frame_id: 'camera'}, poses: [{position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}]}"
```

Expected:

```text
Received PoseArray in normalized_image mode.
Skipping /octopus/detections_world publish to avoid wrong map updates.
```

This is correct. Normalized image coordinates must not be treated as map meters.
