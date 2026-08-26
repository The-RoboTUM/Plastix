# How to Test Coverage Polygon Support

> **Paths.** Every path in this document is relative to the repository root — `cd` there first, in *every* terminal these steps open.

This test checks whether Octopus can update map coverage for a scanned area, not only for a single trash detection.

## Pipeline

```text
coverage_polygon JSON
-> /octopus/detections_world
-> grid_map_builder_node
-> /octopus/map_patch
-> map_patch_backend_bridge_node
-> FastAPI /api/map_patch
-> dashboard Local Grid Map
```

## 0. Build package

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select octopus_mapping octopus_backend_bridge

source install/setup.bash
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

## 4. Terminal 4: Publish coverage polygon

```bash
cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /octopus/detections_world std_msgs/msg/String \
"{data: '{\"source_id\":\"coverage_test\",\"frame_id\":\"map\",\"coverage_confidence\":0.7,\"coverage_polygon\":[[1.0,0.5],[3.0,0.5],[3.0,2.0],[1.0,2.0]],\"detections\":[]}' }"
```

Expected in map builder terminal:

```text
Published map patch with many updated cell(s)
```

The polygon covers:

```text
x = 1.0 m to 3.0 m
y = 0.5 m to 2.0 m
```

At 0.10 m resolution, this is roughly:

```text
2.0 m x 1.5 m = around 300 cells
```

## 5. Check dashboard

In the dashboard, select:

```text
Local Grid Map -> Coverage
```

Expected:

```text
A rectangular scanned area appears.
```

Then select:

```text
Local Grid Map -> Confidence
```

Expected:

```text
The same area appears with confidence around 0.7.
```

## 6. Test coverage + trash together

Publish one message with scanned area and one trash detection:

```bash
ros2 topic pub --once /octopus/detections_world std_msgs/msg/String \
"{data: '{\"source_id\":\"coverage_trash_test\",\"frame_id\":\"map\",\"coverage_confidence\":0.7,\"coverage_polygon\":[[1.0,0.5],[3.0,0.5],[3.0,2.0],[1.0,2.0]],\"detections\":[{\"class_name\":\"trash\",\"x\":2.0,\"y\":1.0,\"confidence\":0.95}]}' }"
```

Expected:

```text
Coverage layer:
  rectangle appears

Trash probability layer:
  one strong trash cell appears at row 10, col 20

Confidence layer:
  scanned area has confidence 0.7
  trash cell has confidence 0.95
```

## 7. Check backend

```bash
curl http://127.0.0.1:8000/api/global_map/latest
```

Expected:

```text
Many cells inside "cells": {...}
```

## Important concept

`coverage_polygon` means:

```text
This ground area was visible to the camera.
```

Trash detections are separate:

```text
This specific point likely contains trash.
```

So one camera frame can update both:

```text
coverage area
trash target
```
