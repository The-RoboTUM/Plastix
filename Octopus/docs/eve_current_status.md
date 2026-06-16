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

Open issue:

The output interface is not standardized yet.

Current conflicting ideas:

- /Camera_Coordinates_Octopus as Float32MultiArray
- /Camera_Coordinates_Octopus as geometry_msgs/Polygon

Decision needed:

Agree on one detection topic, one message type, and one coordinate frame before Octopus depends on Eve output.
