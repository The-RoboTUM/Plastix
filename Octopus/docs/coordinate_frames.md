# Coordinate Frames

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
