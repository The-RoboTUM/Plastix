# How to Test `octopus_mapping`

> **Paths.** Every path in this document is relative to the repository root — `cd` there first, in *every* terminal these steps open.

This file explains how to test the first Octopus ROS2 mapping prototype with fake detections.

The test checks this pipeline:

fake detection
-> /octopus/detections_world
-> grid_map_builder_node
-> /octopus/map_patch
-> /octopus/global_map
-> /octopus/coverage_grid
-> /octopus/trash_grid

0. Build the package

Run this once before testing:

cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select octopus_mapping

source install/setup.bash

ros2 pkg executables octopus_mapping

Expected output:

octopus_mapping grid_map_builder_node
1. Terminal 1: Start the map builder

Open terminal 1 and run:

cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run octopus_mapping grid_map_builder_node

Expected output:

octopus_mapping started: 50x30 cells, resolution=0.1 m/cell

Leave this terminal running.

2. Terminal 2: Listen to map patches

Open terminal 2 and run:

cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /octopus/map_patch

Leave this terminal running.

Important: start this before publishing the fake detection, otherwise the one-time map patch can be missed.

3. Terminal 3: Publish a fake detection

Open terminal 3 and run:

cd Octopus/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /octopus/detections_world std_msgs/msg/String \
"{data: '{\"source_id\":\"test_manual\",\"frame_id\":\"map\",\"detections\":[{\"class_name\":\"trash\",\"x\":2.0,\"y\":1.0,\"confidence\":0.9}]}' }"

Expected output in terminal 1:

Published map patch with 1 updated cell(s)

Expected output in terminal 2 should contain a JSON string with approximately:

{
  "frame_id": "map",
  "updated_cells": [
    {
      "row": 10,
      "col": 20,
      "x": 2.05,
      "y": 1.05,
      "coverage": 1.0,
      "trash_probability": 0.9,
      "confidence": 0.9,
      "source_id": "test_manual"
    }
  ]
}

Why row 10 and column 20?

x = 2.0 m, resolution = 0.10 m/cell -> col = 20
y = 1.0 m, resolution = 0.10 m/cell -> row = 10
