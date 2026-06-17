# How to Connect Laptop to Eve Pi Camera

This explains how to start the camera node on the Raspberry Pi and view the camera feed on the laptop over WiFi.

## Terminal 1: SSH into the Pi and start camera node

On the laptop:

```bash
ssh eve@10.242.71.36
```

Inside the SSH terminal on the Pi:

```bash
cd ~/PlastiX/eve/Software/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 run camera_pkg camera_node
```

Leave this terminal running.

By default, the camera node publishes compressed images.

## Terminal 2: Open camera feed on laptop

On the laptop, in a second terminal:

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 topic list
```

Expected topics:

```bash
/camera/image_raw
/camera/image_raw/compressed
```

Open the image viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

In the dropdown, select:

```bash
/camera/image_raw/compressed
```

This should show the camera feed over WiFi.

## Optional: publish raw images too

The Eve camera node has this parameter:

```bash
publish_raw
```

Default:

```bash
False
```

That means compressed images are published by default, but raw images may not actually send data.

To publish raw images as well, start the camera node like this on the Pi:

```bash
ros2 run camera_pkg camera_node --ros-args -p publish_raw:=true
```

Then the raw image topic can be used:

```bash
/camera/image_raw
```

Use this later for OpenCV, ArUco, or object detection nodes if needed.

## Notes

The existing camera node is located in the Eve branch at:

```bash
eve/Software/ros2_ws/src/camera_pkg/camera_pkg/camera_node.py
```

Current default published stream:

```bash
/camera/image_raw/compressed
```

Useful for laptop viewing:

```bash
/camera/image_raw/compressed
```

Useful for later computer vision processing:

```bash
/camera/image_raw
```
