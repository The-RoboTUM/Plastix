# detection_pkg

ROS 2 node that detects trash in camera frames, localizes it in 2D, and publishes confirmed positions.

It wraps the `detect-and-localize` pipeline (YOLO detection → AprilTag or normalized-image localization → stationary-track confirmation). The core logic lives in `Octopus/detect-and-localize/src/pipeline.py` and is shared with the standalone `main.py` CLI, so there is one source of truth for detection behavior.

## How it works

1. A compressed camera frame arrives on the input topic.
2. YOLO runs every N frames (configurable) and detects rubbish objects.
3. Each detection is mapped to a 2D position — either world coordinates (if AprilTag positions are supplied) or normalized image coordinates.
4. A tracker links detections across frames. Once a detection has been seen for enough consecutive frames without moving, it is **confirmed** as real rubbish.
5. Positions are published on two topics (see below).

## Topics

Both outputs are `geometry_msgs/PoseArray` with `z = 0`.

| Topic | QoS | Published when | Contents |
|-------|-----|----------------|----------|
| `~/detections` | reliable, depth 10 | every processed frame | all detections in this frame |
| `~/confirmed` | transient-local (latched) | whenever a new track is confirmed | full set of confirmed positions so far |

**Coordinate system:**
- With a `tags` CSV: real-world coordinates (meters, origin at the tag layout centroid).
- Without `tags`: normalized image coordinates — `(0, 0)` = bottom-left, `(1, 1)` = top-right.

**Timestamps:** `header.stamp` carries the **camera capture time** from the incoming frame, not the processing time. This keeps detections aligned with TF and sensor fusion.

## Running

The node needs both the Python venv (for `ultralytics`) and ROS sourced. Source ROS first so it bridges `rclpy` into the venv:

```bash
source /opt/ros/humble/setup.bash
source <repo>/Octopus/detect-and-localize/.venv/bin/activate
source <repo>/Octopus/ros2_ws/install/setup.bash

# Normalized image coordinates (no tags):
python <repo>/Octopus/ros2_ws/src/detection_pkg/detection_pkg/detector_node.py

# World coordinates (with AprilTag CSV):
python <repo>/Octopus/ros2_ws/src/detection_pkg/detection_pkg/detector_node.py \
  --ros-args -p tags:=data/tags/<file>.csv

# Enable the live UI windows:
python <repo>/Octopus/ros2_ws/src/detection_pkg/detection_pkg/detector_node.py \
  --ros-args -p show_ui:=true
```

Feed it frames with a real camera or a video replay. `camera_pkg` runs **on the Pi**
and is not part of the Octopus folder - its source lives in `eve/` on the
`eve_ros_development` branch:

```bash
ros2 run camera_pkg camera_node
# or
ros2 run camera_pkg video_publisher --ros-args -p video_path:=<file>.mp4
```

Watch the output:

```bash
ros2 topic echo /detector_node/detections
ros2 topic echo /detector_node/confirmed --qos-durability transient_local
```

## Parameters

### Infrastructure

| Parameter | Default | Description |
|-----------|---------|-------------|
| `detect_localize_path` | `~/PlastiX/Octopus/detect-and-localize` | Path to `Octopus/detect-and-localize`. The node imports `pipeline.py`, `visualization.py`, and `fps_meter.py` from there at runtime. |
| `input_topic` | `camera/image_raw/compressed` | ROS topic to subscribe to for camera frames. |
| `output_frame` | `map` | `frame_id` stamped on published `PoseArray` messages. |
| `show_ui` | `false` | Open live OpenCV windows ("Frame" and "World Map") for visual debugging. |

### Detection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model` | `data/models/indoor_11s.pt` | Path to the YOLO `.pt` weights file. Relative paths are resolved from the repo root. |
| `thresh` | `0.6` | YOLO confidence threshold (0–1). Detections below this score are discarded. |
| `yolo_frameskip` | `4` | Run YOLO only every N+1 frames. E.g. `4` means YOLO fires on frames 0, 5, 10, … Frames in between reuse the last detections, reducing GPU load while keeping the tracker fed. Set to `0` to run YOLO on every frame. |

### Localization

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tags` | `` (empty) | Path to a CSV file with AprilTag world positions. When set, detections are projected into real-world coordinates via homography. When empty, normalized image coordinates are used instead. |

### Tracker

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_lost` | `100` | Frames a track can go without a matching detection before it is deleted. Raise this to keep tracks alive through occlusions; lower it to clean up faster. |
| `confirm_frames` | `10` | Consecutive frames a track must be seen before it counts as confirmed rubbish. Higher = more conservative, fewer false positives. |
| `dist_thresh` | `0.05` | Maximum distance (in coordinate units) to associate a new detection with an existing track. If a detection is further away than this from all tracks, a new track is created. |
| `move_thresh` | `0.04` | If a confirmed track moves more than this distance between frames, it is treated as still moving and will not be re-confirmed until it settles again. |

## Notes

- Use the **venv's Python**, not system Python — system Python has no `ultralytics`, the venv has no `rclpy`; sourcing ROS bridges the gap.
- Frames are decoded with `cv2.imdecode` instead of `cv_bridge` to avoid a NumPy 1.x / 2.x ABI crash between the ROS install and the venv.
- Replaying a **looping** video will re-confirm the same objects under new track IDs each loop (tracks are pruned while off-screen, then recreated). Use `-p loop:=false` on the video publisher, or raise `max_lost` high enough to survive the full loop.
