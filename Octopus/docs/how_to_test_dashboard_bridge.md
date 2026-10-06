# How to Test Dashboard Bridge

> **Paths.** Every path in this document is relative to the repository root — `cd` there first, in *every* terminal these steps open.

This test checks the full ROS2-to-dashboard pipeline.

Pipeline:

```text
fake detection
-> /octopus/detections_world
-> grid_map_builder_node
-> /octopus/map_patch
-> map_patch_backend_bridge_node
-> FastAPI /api/map_patch
-> dashboard Latest Map Patch panel
```

## 0. Build ROS2 packages

Run once before testing:

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select octopus_mapping octopus_backend_bridge

source install/setup.bash

ros2 pkg executables octopus_mapping
ros2 pkg executables octopus_backend_bridge
```

Expected output:

```text
octopus_mapping grid_map_builder_node
octopus_backend_bridge map_patch_backend_bridge_node
```

---

## 1. Terminal 1: Start dashboard backend

```bash
cd Octopus/octopus-dashboard

python3 -m uvicorn api:app --reload
```

Open in browser:

```text
http://127.0.0.1:8000/dashboard.html
```

The dashboard should show a card called:

```text
Latest Map Patch
```

---

## 2. Terminal 2: Start Octopus map builder

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run octopus_mapping grid_map_builder_node
```

Expected output:

```text
octopus_mapping started: 50x30 cells, resolution=0.1 m/cell
```

Leave this terminal running.

---

## 3. Terminal 3: Start ROS2 backend bridge

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run octopus_backend_bridge map_patch_backend_bridge_node
```

Expected output:

```text
Map patch backend bridge started
Posting map patches to: http://127.0.0.1:8000/api/map_patch
```

Leave this terminal running.

---

## 4. Terminal 4: Publish fake detection

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /octopus/detections_world std_msgs/msg/String \
"{data: '{\"source_id\":\"test_manual\",\"frame_id\":\"map\",\"detections\":[{\"class_name\":\"trash\",\"x\":2.0,\"y\":1.0,\"confidence\":0.9}]}' }"
```

Expected output in Terminal 2:

```text
Published map patch with 1 updated cell(s)
```

Expected output in Terminal 3:

```text
Posted map patch to backend with 1 updated cell(s)
```

Expected dashboard result:

```text
Latest Map Patch
Cell: 10, 20
x,y: 2.05, 1.05
Trash: 0.90
Conf.: 0.90
```

---

## 5. Check backend directly

```bash
curl http://127.0.0.1:8000/api/map_patch/latest
```

Expected response contains:

```json
{
  "status": "ok",
  "patch": {
    "frame_id": "map",
    "updated_cells": [
      {
        "row": 10,
        "col": 20,
        "trash_probability": 0.9,
        "confidence": 0.9
      }
    ]
  }
}
```

---

## 6. Check ROS2 topics

```bash
ros2 topic list
```

Expected relevant topics:

```text
/octopus/coverage_grid
/octopus/detections_world
/octopus/global_map
/octopus/map_patch
/octopus/trash_grid
```

---

## 7. Stop test

Stop running processes with:

```text
Ctrl + C
```

Then check Git:

```bash
git status
```

Expected:

```text
nothing to commit, working tree clean
```

---

## What this test proves

This test proves:

```text
manual/fake detection
-> ROS2 mapping node
-> ROS2 map patch
-> backend bridge
-> FastAPI endpoint
-> dashboard panel
```

This is the first end-to-end Octopus mapping-to-dashboard prototype.
