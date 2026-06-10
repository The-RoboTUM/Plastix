# camera_pkg

ROS 2 package that captures frames from a V4L2 camera and publishes them as both raw and JPEG-compressed image topics.

## How it works

`camera_node` opens a camera device via OpenCV/V4L2 and publishes at a configurable frame rate. Each captured frame is published on two topics simultaneously:

- **`camera/image_raw`** — uncompressed BGR8 image (`sensor_msgs/Image`)
- **`camera/image_raw/compressed`** — JPEG-encoded image (`sensor_msgs/CompressedImage`)

Every 10th frame is also saved to `/tmp/camera_images/` as a JPEG file for debugging.

## Dependencies

```bash
sudo apt install ros-humble-cv-bridge python3-opencv python3-numpy
```

## Building

```bash
cd ~/PlastiX/eve/Software/ros2_ws
colcon build --packages-select camera_pkg
source install/setup.bash
```

## Running

```bash
ros2 run camera_pkg camera_node
```

With custom parameters:
```bash
ros2 run camera_pkg camera_node --ros-args \
  -p device_index:=0 \
  -p frame_width:=1280 \
  -p frame_height:=720 \
  -p frame_rate:=30.0 \
  -p jpeg_quality:=80
```

| Parameter | Default | Description |
|---|---|---|
| `device_index` | `0` | Camera device (`/dev/video0`, `/dev/video1`, ...) |
| `frame_width` | `640` | Capture width in pixels |
| `frame_height` | `480` | Capture height in pixels |
| `frame_rate` | `30.0` | Frames per second |
| `jpeg_quality` | `80` | JPEG compression quality (1–100) |

## Viewing the stream remotely over WiFi

### Prerequisites on the remote machine

```bash
sudo apt install ros-humble-rqt-image-view ros-humble-image-transport-plugins
```

### Network setup

1. Ensure both machines are on the same network.
2. On the **camera device**, make sure `ROS_LOCALHOST_ONLY` is not set to `1` in `~/.bashrc`.
3. Set the same `ROS_DOMAIN_ID` on both machines (default is `0`).

### View the stream

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera/image_raw/compressed` from the dropdown for best performance over WiFi.

### WSL note

If running on WSL2, the daemon may not discover remote topics. Add this to `~/.bashrc` on the WSL machine:

```bash
alias ros2='ros2 --no-daemon'
```

For full network access from WSL2 (Windows 11 22H2+), enable mirrored networking in `C:\Users\<user>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```
