# detection_pkg

Trash detection + localization for ROS 2. Subscribes to camera frames, runs the
`detect-and-localize` pipeline (YOLO + AprilTag/normalized localization +
stationary-track confirmation), and publishes 2D positions.

The per-frame logic lives in `detect-and-localize/src/pipeline.py`
(`DetectAndLocalize`) and is shared with the CLI `main.py`, so there is one
source of truth.

## Topics

Both outputs are `geometry_msgs/PoseArray` (x, y, z = 0):

| Topic | When | Contents |
|-------|------|----------|
| `~/detections` | every processed frame (reliable QoS) | all current detections |
| `~/confirmed`  | only when trash settles (latched / transient-local) | full set of confirmed positions |

**Coordinates:** world coords if a `tags` CSV is given, else normalized image
coords in `[0, 1]` (origin bottom-left).

**Timestamps:** each message's `header.stamp` is the **camera capture time**,
propagated from the incoming frame (not processing time), so detections align
with TF / sensor fusion. It is one stamp per message (the frame time); a
`PoseArray` cannot carry per-item timestamps.

## Running

The node needs the venv (for `ultralytics`) and ROS on the path. Sourcing ROS
adds `rclpy` to the venv, so source ROS, then activate the venv, then the
workspace:

```bash
source /opt/ros/jazzy/setup.bash
source <repo>/detect-and-localize/plastix_venv/bin/activate
source <repo>/ros2_ws/install/setup.bash
python <repo>/ros2_ws/src/detection_pkg/detection_pkg/detector_node.py
# world coords: --ros-args -p tags:=data/tags/<file>.csv
```

Feed it frames with the real camera (`ros2 run camera_pkg camera_node`) or
replay footage (`ros2 run camera_pkg video_publisher -p video_path:=<.mp4>`).

Watch the output:

```bash
ros2 topic echo /detector_node/detections
ros2 topic echo /detector_node/confirmed --qos-durability transient_local
```

## Parameters

`detect_localize_path`, `model`, `thresh`, `tags`, `yolo_frameskip`,
`max_lost`, `confirm_frames`, `dist_thresh`, `move_thresh`, `input_topic`,
`output_frame`.

## Notes

- Run with the **venv's Python** — system Python has no `ultralytics`, the venv
  has no `rclpy`; sourcing ROS bridges the gap.
- JPEG is decoded with `cv2.imdecode` (not `cv_bridge`) to avoid the NumPy
  1.x/2.x ABI clash between ROS and the venv.
- Replaying a **looping** video re-confirms objects under new IDs (tracks are
  pruned while off-screen, then re-created on the next loop). Use
  `-p loop:=false` or raise `-p max_lost:=` for a clean single pass.
