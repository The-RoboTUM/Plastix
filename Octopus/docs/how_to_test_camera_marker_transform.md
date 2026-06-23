# How to Test Camera Marker Transform

This test checks whether Octopus can transform normalized camera detections into map/grid coordinates using AprilTags.

## Pipeline

```text
Pi camera image
-> /camera/image_raw/compressed
-> camera_marker_transform_node
-> AprilTag homography image-to-map
-> fake detector PoseArray /detector_node/confirmed
-> /octopus/detections_world
-> grid_map_builder_node
-> /octopus/map_patch
-> map_patch_backend_bridge_node
-> FastAPI /api/map_patch
-> dashboard Local Grid Map
```

## Goal

The goal is to verify this transform:

```text
normalized image detection:
u = 0.5
v = 0.5

becomes map coordinates:
x ≈ 0.0 - 5.0 m
y ≈ 0.0 - 3.0 m
```

The detector input should still be normalized camera coordinates.

The transformed output should no longer be `0.5, 0.5`.

## Marker setup

Use AprilTag family:

```text
tag36h11
```

Required marker IDs:

```text
61 = origin corner       -> map coordinate (0.0, 0.0)
65 = x corner            -> map coordinate (5.0, 0.0)
57 = xy corner           -> map coordinate (5.0, 3.0)
11 = y corner            -> map coordinate (0.0, 3.0)
```

Optional marker:

```text
9 = currently ignored
```

The four required markers define the local map field.

For the current prototype, the default map size is:

```text
width  = 5.0 m
height = 3.0 m
resolution = 0.10 m/cell
```

This gives:

```text
cols = 50
rows = 30
```

## Important assumptions

The current prototype assumes:

```text
- The camera can see the AprilTags.
- The field is approximately planar.
- The detector outputs normalized image coordinates in [0, 1].
- PoseArray pose.position.x is normalized u.
- PoseArray pose.position.y is normalized v.
- The camera marker node has already computed a valid homography.
```

This is not yet the final flight-ready drone transform.

The final outdoor version will use:

```text
camera intrinsics
+ camera-to-drone extrinsic transform
+ Pixhawk attitude
+ drone position
+ altitude
+ ground-plane intersection
```

## 0. Build required packages

On the laptop:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --packages-select \
  octopus_camera_transform \
  octopus_mapping \
  octopus_backend_bridge

source install/setup.bash
```

## 1. Terminal 0: Start camera on the Pi

On the laptop, SSH into the Pi:

```bash
ssh eve@172.19.216.36
```

On the Pi:

```bash
cd ~/PlastiX/eve/Software/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run camera_pkg camera_node --ros-args \
  -p device_index:=1 \
  -p frame_width:=640 \
  -p frame_height:=480 \
  -p frame_rate:=30.0 \
  -p verbose:=true
```

Leave this running.

Note:

The USB camera device index can change. If the camera is not found, check:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

The correct camera is the first video device listed under:

```text
USB camera: USB camera
```

Example:

```text
USB camera: USB camera
    /dev/video1
    /dev/video2
```

Then use:

```text
device_index:=1
```

## 2. Terminal 1: Check camera topic on laptop

On the laptop:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 topic hz /camera/image_raw/compressed
```

Expected output:

```text
average rate: ...
```

A working value around 10-30 Hz is fine.

Stop with:

```text
Ctrl + C
```

Optional image check:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/camera/image_raw/compressed
```

## 3. Terminal 2: Start backend

On the laptop:

```bash
cd ~/projects/PlastiX/Octopus/octopus-dashboard

python3 -m uvicorn api:app --reload
```

Open:

```text
http://127.0.0.1:8000/dashboard.html
```

## 4. Terminal 3: Start grid map builder

On the laptop:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run octopus_mapping grid_map_builder_node
```

Expected topics:

```text
/octopus/map_patch
/octopus/global_map
/octopus/coverage_grid
/octopus/trash_grid
```

## 5. Terminal 4: Start backend bridge

On the laptop:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run octopus_backend_bridge map_patch_backend_bridge_node
```

This bridge posts map patches from ROS2 to the FastAPI backend.

## 6. Terminal 5: Start camera marker transform node

On the laptop:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run octopus_camera_transform camera_marker_transform_node
```

Expected startup output:

```text
Using marker dictionary: DICT_APRILTAG_36h11
Camera marker transform node started
Image topic: /camera/image_raw/compressed
Detector topic: /detector_node/confirmed
Output topic: /octopus/detections_world
Marker dictionary: APRILTAG_36h11
Expected marker IDs: 61=origin, 65=x corner, 57=xy corner, 11=y corner
```

Expected marker debug output:

```text
Detected marker IDs: [...]
```

When all required markers are visible, expected output:

```text
Updated image-to-map homography from field markers.
```

The node may temporarily lose markers if the tags are too small or too far away. That is okay for the first test as long as it computes a homography at least once.

## 7. Terminal 6: Watch transformed detections

On the laptop:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 topic echo /octopus/detections_world --field data
```

Leave this running.

## 8. Terminal 7: Publish fake normalized detector point

On the laptop, publish a fake detector point at the image center:

```bash
cd ~/projects/PlastiX/Octopus/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 topic pub --once /detector_node/confirmed geometry_msgs/msg/PoseArray \
"{header: {frame_id: 'camera'}, poses: [{position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}]}"
```

This input is intentionally:

```text
x = 0.5
y = 0.5
```

At this stage, `x/y` mean normalized image coordinates:

```text
x = u
y = v
```

The transform result appears on:

```text
/octopus/detections_world
```

## 9. Expected transformed output

In the terminal listening to `/octopus/detections_world`, you should see JSON similar to:

```json
{
  "source_id": "camera_marker_transform",
  "frame_id": "map",
  "timestamp": 1782222420.6834445,
  "detections": [
    {
      "class_name": "trash",
      "x": 2.15,
      "y": 1.25,
      "confidence": 1.0
    }
  ]
}
```

Important:

The transformed output should not be:

```text
x = 0.5
y = 0.5
```

It should be map coordinates in meters.

Example valid values:

```text
x = 2.15
y = 1.25
```

## 10. Verify backend/global map

Save the current global map response:

```bash
curl -s http://127.0.0.1:8000/api/global_map/latest > /tmp/global_map.json

echo "First 300 chars:"
head -c 300 /tmp/global_map.json
echo
```

Expected:

```text
{"status":"ok","map":...
```

Count trash cells:

```bash
python3 - << 'EOF'
import json

path = "/tmp/global_map.json"

with open(path) as f:
    data = json.load(f)

cells = data.get("map", {}).get("cells", {})

trash = []
for key, cell in cells.items():
    if cell.get("trash_probability", 0.0) > 0.0:
        trash.append((key, cell))

print("trash cells:", len(trash))

for key, cell in trash[-20:]:
    print(
        key,
        "x=", round(cell.get("x", -1), 2),
        "y=", round(cell.get("y", -1), 2),
        "trash=", cell.get("trash_probability"),
        "source=", cell.get("source_id"),
    )
EOF
```

Expected:

```text
trash cells: > 0
source= camera_marker_transform
```

Example real successful output:

```text
trash cells: 2
12,21 x= 2.15 y= 1.25 trash= 1.0 source= camera_marker_transform
18,17 x= 1.75 y= 1.85 trash= 1.0 source= camera_marker_transform
```

## 11. Verify dashboard

Open:

```text
http://127.0.0.1:8000/dashboard.html
```

Check the Local Grid Map.

Expected:

```text
Coverage layer:
- scanned area appears from the camera footprint

Trash probability layer:
- one or more orange trash cells appear inside the marker-defined field
```

If you do not see the new trash point clearly, check the backend/global map output first. The dashboard may already contain previous test points.

## 12. Test more image positions

Publish lower-left-ish image point:

```bash
ros2 topic pub --once /detector_node/confirmed geometry_msgs/msg/PoseArray \
"{header: {frame_id: 'camera'}, poses: [{position: {x: 0.25, y: 0.25, z: 0.0}, orientation: {w: 1.0}}]}"
```

Publish upper-right-ish image point:

```bash
ros2 topic pub --once /detector_node/confirmed geometry_msgs/msg/PoseArray \
"{header: {frame_id: 'camera'}, poses: [{position: {x: 0.75, y: 0.75, z: 0.0}, orientation: {w: 1.0}}]}"
```

Expected:

```text
The transformed map coordinates and grid cells should change.
```

## Troubleshooting

### No detected marker IDs

If the node prints:

```text
Detected marker IDs: []
```

Possible causes:

```text
- AprilTags are too small in the image
- camera is too far away
- glare on printed tags
- motion blur
- wrong tag family
- tags are cropped
- bad contrast
```

Move the camera closer or print larger tags.

### Only some markers are detected

Example:

```text
Detected marker IDs: [11, 57, 61]
```

This means one required marker is missing.

Required IDs:

```text
11, 57, 61, 65
```

All four are needed to compute a stable full-field homography.

### QoS warning on camera topic

If you see:

```text
offering incompatible QoS
Last incompatible policy: RELIABILITY
```

then the image subscriber is using the wrong QoS.

The camera marker transform node must subscribe to `/camera/image_raw/compressed` with sensor-data QoS / best-effort.

### Camera topic not visible

Check that the camera node is running on the Pi.

On the laptop:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw/compressed
```

If there is no camera topic, restart the Pi camera node.

### Camera device changed

Check on the Pi:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

Use the first `/dev/videoX` listed under the USB camera.

Example:

```text
USB camera: USB camera
    /dev/video1
    /dev/video2
```

Use:

```text
device_index:=1
```

### Backend verification command fails with JSONDecodeError

Do not pipe `curl` directly into a Python heredoc.

Use this instead:

```bash
curl -s http://127.0.0.1:8000/api/global_map/latest > /tmp/global_map.json
python3 - << 'EOF'
import json
with open("/tmp/global_map.json") as f:
    data = json.load(f)
print(data.keys())
EOF
```

## Current prototype status

This test proves:

```text
Pi camera image works
AprilTag detection works
homography image-to-map works
normalized detector input can be transformed to map coordinates
Octopus grid map receives transformed trash detections
backend receives map patches
dashboard visualizes the grid state
```

This is the first working camera-to-grid prototype.

It does not yet prove the final flight-ready drone transform.
