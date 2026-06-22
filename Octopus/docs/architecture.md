# Octopus Mapping Architecture

Goal: build a shared map that drone and ground robots can use.

First prototype pipeline:

1. camera sees test field
2. markers define local map frame
3. object/detection gets x,y map coordinate
4. map builder updates grid layers
5. backend/dashboard visualizes map updates

Do not start with full outdoor flight, GPS, stereo, and robot navigation at once.

Build order:

1. marker-based field transform
2. grid-map-style map layers
3. backend/dashboard map update
4. drone detection integration
5. PX4 pose integration
6. ground robot map consumption
7. outdoor scan test

Detector adapter layer

The real detector currently does not publish Octopus JSON directly.

Actual detector output:

/detector_node/detections
/detector_node/confirmed

Type:

geometry_msgs/PoseArray

Therefore the architecture needs an adapter:

detection_pkg
-> PoseArray
-> octopus_detector_bridge
-> /octopus/detections_world JSON
-> octopus_mapping

This keeps the detector independent from Octopus map/backend formats.
