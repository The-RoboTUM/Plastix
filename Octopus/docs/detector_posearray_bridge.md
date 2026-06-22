# Detector PoseArray Bridge

This document defines how the current detector output is converted into the Octopus mapping input format.

## Current detector interface

The current detection_pkg does not publish the Octopus JSON format directly.

It subscribes to:

/camera/image_raw/compressed

Type:

sensor_msgs/CompressedImage

It publishes detections to:

/detector_node/detections
/detector_node/confirmed

Type:

geometry_msgs/PoseArray

## PoseArray content

Each pose contains:

pose.position.x
pose.position.y
pose.position.z = 0

The image timestamp is stored in the PoseArray header.

Current limitations:

no class_name
no confidence
no source_id

Therefore the bridge must add default values.

## Coordinate modes

The meaning of pose.position.x/y depends on detector configuration.

### Mode 1: map/world coordinates

If AprilTags are used and a valid AprilTag CSV/config is provided, the detector can output map/world coordinates.

Then:

pose.position.x = x in map/world frame, meters
pose.position.y = y in map/world frame, meters

This can be converted directly into /octopus/detections_world.

### Mode 2: normalized image coordinates

If no AprilTag CSV/config is provided, the detector outputs normalized image coordinates.

Then:

pose.position.x = normalized image u in [0,1]
pose.position.y = normalized image v in [0,1]

These values must not be treated as map meters.

They need an additional transform before they can update the grid map:

normalized image u,v
-> camera/marker/Pixhawk transform
-> map/world x,y

## Bridge responsibility

The bridge node converts:

/detector_node/detections or /detector_node/confirmed
geometry_msgs/PoseArray

into:

/octopus/detections_world
std_msgs/String JSON

Target JSON example:

{
  "source_id": "detector_node",
  "frame_id": "map",
  "timestamp": 1781710172.45,
  "detections": [
    {
      "class_name": "trash",
      "x": 2.05,
      "y": 1.05,
      "confidence": 1.0
    }
  ]
}

## Required bridge parameters

Recommended parameters:

input_topic              default: /detector_node/confirmed
output_topic             default: /octopus/detections_world
input_coordinate_mode    default: map
source_id                default: detector_node
default_class_name       default: trash
default_confidence       default: 1.0

Allowed coordinate modes:

map
normalized_image

## Behavior in map mode

If input_coordinate_mode = map:

PoseArray x,y
-> Octopus JSON x,y
-> publish /octopus/detections_world

## Behavior in normalized_image mode

If input_coordinate_mode = normalized_image:

PoseArray x,y are u,v image coordinates

The bridge should not send these directly to /octopus/detections_world.

Instead, later versions must transform them:

u,v
-> camera ray or homography
-> map x,y
-> /octopus/detections_world

For the first version, the bridge should warn and skip publishing map detections in this mode.

## Future flight-ready transform

For flight without AprilTags, the transform should use:

normalized camera detection u,v
camera intrinsics
camera-to-drone extrinsics
Pixhawk attitude
Pixhawk position
altitude estimate
ground plane assumption

Pipeline:

u,v
-> camera ray
-> drone/body frame
-> map/world frame
-> intersect ray with ground plane
-> x,y map coordinate
-> Octopus JSON
