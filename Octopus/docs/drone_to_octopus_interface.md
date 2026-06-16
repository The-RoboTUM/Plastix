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
