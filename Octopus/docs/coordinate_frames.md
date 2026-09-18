# Coordinate Frames

## Own position

Whatever anchors the local frame reports itself as `x = 0, y = 0`. In the indoor fake-GPS demo
that anchor is Eve's start point (the datum), so Eve's own local coordinate is always `0, 0`
and only her `lat`/`lon` change — see
[`octopus_to_robot_interface.md`](octopus_to_robot_interface.md#das-datum). In the marker-based
test field below it is `marker_0`. Only GPS ever expresses the anchor as a non-zero number.

## Test field

First prototype uses a local flat test-field frame.

Example field:

- marker_0: (0.0, 0.0)
- marker_1: (5.0, 0.0)
- marker_2: (5.0, 3.0)
- marker_3: (0.0, 3.0)

Axis convention:

- x: marker_0 -> marker_1
- y: marker_0 -> marker_3
- z: upward

Image convention:

- u: pixel column, positive right
- v: pixel row, positive down

First transform:

- image pixel (u,v) -> map coordinate (x,y)
- use homography for the flat-field prototype

Later we need:

- camera -> drone body
- drone body -> map
- PX4 local frame -> Octopus map

Detector PoseArray coordinate warning

The current detector publishes geometry_msgs/PoseArray.

The meaning of pose.position.x/y depends on detector configuration:

with AprilTags:
  x,y = map/world coordinates

without AprilTags:
  x,y = normalized image coordinates in [0,1]

Therefore, any bridge to /octopus/detections_world must know the coordinate mode.

Normalized image coordinates must first be transformed:

u,v
-> homography or camera/Pixhawk transform
-> map x,y

Only map/world x,y should be sent to the map builder.
