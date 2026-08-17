# Eve Current Software Status

Checked branch:

- eve_ros_development

Relevant folder:

- eve/Software

Observed components:

- camera_pkg publishes camera/image_raw and camera/image_raw/compressed
- raw image publishing exists but is disabled by default
- object detection package exists
- coordinate transformation code exists
- px4_msgs exists in the drone ROS2 workspace

## Status: erledigt

Historische Momentaufnahme. Die offene Frage dieses Dokuments — welches Topic, welcher
Nachrichtentyp, welcher Frame — ist entschieden:

- Topic: `/detector_node/confirmed` (zusätzlich `/detector_node/detections` pro Frame)
- Typ: `geometry_msgs/PoseArray`
- Frame: normalisierte Bildkoordinaten, danach von `flight_camera_transform_node` in
  Map-Koordinaten projiziert

Die verworfenen Ideen `/Camera_Coordinates_Octopus` als `Float32MultiArray` oder
`geometry_msgs/Polygon` wurden nie umgesetzt.

Aktueller Vertrag: [`drone_to_octopus_interface.md`](drone_to_octopus_interface.md) und
[`coordinate_frames.md`](coordinate_frames.md).
