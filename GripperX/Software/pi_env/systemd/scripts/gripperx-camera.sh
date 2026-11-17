#!/bin/bash
# /usr/local/bin/gripperx-camera.sh -- started by gripperx-camera.service.
#
# What: the front camera driver jpeg_camera (gripperx_sensors), standalone, NOT part of
# the bringup. For whom: the deployer. Status: user decision 2026-11-17 (CAMVIEW): the
# camera node starts at boot.
#
# Why its own unit and not the bringup: the camera has no part in the drive, steering or
# arm path. Starting, stopping or restarting this unit makes nothing commandable and moves
# nothing, so it needs no SR-1 (motor-enable process rule) approval, unlike a bringup
# restart. Mode R (stack restart button) does not touch it; it keeps running.
#
# Cost while nobody watches: the node opens the camera only while /camera/front/image_raw/
# compressed or camera_info has a subscriber (lazy capture). With none, /dev/video0 is
# closed, no USB stream runs and no data leaves the Pi. Measured 2026-11-17 on GripperX-1:
# 0.5 % of one core idle, 1.6 % while streaming the "view" profile (320x240 @ 15 fps).
#
# systemd does not inherit the ROS environment: everything is sourced here.

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/Software/ros2/install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export HOME=/home/ubuntu
export PYTHONUNBUFFERED=1

# exec: ros2 launch becomes the unit's main process and forwards systemd's SIGINT to
# the node, which releases /dev/video0 on SIGINT/SIGTERM.
exec ros2 launch gripperx_sensors camera.launch.py
