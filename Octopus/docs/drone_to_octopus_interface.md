# Drone to Octopus Interface

Goal: Eve sends object detections to Octopus in map coordinates.

First prototype topic:

- /octopus/detections_world

First prototype message type:

- std_msgs/String
- payload is JSON

Example payload:

{
  "source_id": "eve_drone_01",
  "frame_id": "map",
  "timestamp": 1710000000.0,
  "detections": [
    {
      "class_name": "trash",
      "x": 2.1,
      "y": 0.9,
      "confidence": 0.87
    }
  ]
}

Rules:

- x,y are meters
- frame_id must say which coordinate frame is used
- first prototype uses local test-field map frame
- later replace JSON with custom ROS2 messages

Real detector interface update

The current detector does not publish this JSON format directly.

Current detector output:

/detector_node/detections     geometry_msgs/PoseArray
/detector_node/confirmed      geometry_msgs/PoseArray

The detector subscribes to:

/camera/image_raw/compressed  sensor_msgs/CompressedImage

Each pose contains only:

x
y
z = 0

The PoseArray header contains the image timestamp.

Missing fields:

class_name
confidence
source_id

Therefore, the Octopus JSON format is the target interface after a bridge/transform node, not the raw detector output.

Required bridge:

PoseArray detections
-> octopus_detector_bridge
-> /octopus/detections_world JSON

If AprilTags are configured, x/y can already be map/world coordinates.

If no AprilTag CSV/config is provided, x/y are normalized image coordinates in [0,1] and must not be treated as map meters.
