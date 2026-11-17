# gripperx_sensors

Executables: `sensor_mocks` (bench stand-ins), **`scan_range_filter`** (the real
scan chain), **`bno085_imu`** (the real IMU driver) and `imu_axis_check` (reads the
IMU in base_link axes for the mounting check).

## The scan chain — read this first

```
LiDAR driver (or ros_gz_bridge)  ->  /scan_raw  ->  scan_range_filter  ->  /scan
```

**The LiDAR driver does not publish `/scan`.** `scan_range_filter` republishes it
with the robot's own returns removed (`min_range` 0.10 m). The same filter runs on
both platforms, and it is launched from `gripperx_bringup/real_robot.launch.py`
and `gripperx_gazebo/simulate_robot.launch.py` — **not** from this package's
`sensors.launch.py`, which starts only the mocks.

If the filter is bypassed or misconfigured, the robot maps itself: self-returns
appear as an obstacle ring at the chassis radius and SLAM/AMCL degrade.

## Topics

Consumed by `gripperx_localization` and Nav2:

| Topic | Type | Used by |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | laser odom, SLAM, AMCL, Nav2 |
| `/imu/bno085/data` | `sensor_msgs/Imu` | `localization_input_node`, which republishes it on `/imu/data/filtered` for the EKF (its yaw rate is fused as `imu0`, see below) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | optional GPS fusion (`enable_gps`) |

## Launch

```bash
# Mock sensors (bench)
ros2 launch gripperx_sensors sensors.launch.py

# Real hardware — launch your drivers, then:
ros2 launch gripperx_sensors sensors.launch.py use_mock_sensors:=false
```

Replace mocks with your driver nodes (RPLidar, ublox, microstrain, etc.) publishing the same topic names.

## BNO085 IMU — `bno085_imu` (commissioned 2026-10-20)

Started by `gripperx_bringup/real_robot.launch.py` (`use_imu`, default true). To run
it standalone, start the bringup with `use_imu:=false` — two instances cannot share
the sensor; the second waits on the lock.

Python driver, no dependency beyond rclpy: SHTP/SH-2 (the sensor hub's transport
protocol and command set) over `/dev/i2c-1` with plain i2c-dev reads and writes.
Parameters: `config/bno085.yaml`. Wiring: `documentation/schematics/WIRING_PLAN.md`
§2. Mounting pose: `imu_mount_*` in `gripperx_geometry/config/geometry.yaml`.

**It publishes on `/imu/bno085/data`.** `localization_input_node`
(`gripperx_localization`) reads that topic, removes the standstill gyro bias and
republishes on `/imu/data/filtered`, the EKF's (extended Kalman filter's) `imu0`
input. Since step 2.4 `imu0` fuses the yaw rate only (index 11), on the robot and the
twin alike; absolute IMU yaw stays off. Rationale: `gripperx_localization/config/localization.yaml`,
above `imu0:`.

```bash
# standalone (moves nothing, starts no controller)
ros2 launch gripperx_sensors bno085.launch.py
# at rest: rate, gravity in base_link, gyro bias, orientation drift over 60 s
ros2 run gripperx_sensors imu_axis_check --stats 60
# live, for turning/lifting the robot by hand
ros2 run gripperx_sensors imu_axis_check
```

`imu_axis_check` takes the mounting rotation from TF (coordinate-transform tree) (`base_link <- imu_link`). While
the running robot_description still has the old placeholder `imu_joint`, pass it
explicitly with `--rpy 3.141593 0 0` (the `imu_mount_rpy` value).

`bno085_query` (stop the driver first) reads, without resetting the sensor, the
dynamic-calibration enable flags, the orientation FRS (flash record system)
records and the stored dynamic calibration data; `--compare-raw 20` also averages
the raw against the calibrated accelerometer. It writes nothing persistent. The
driver soft-resets the sensor on recovery only (`reset_on_start: false`, user
decision 2026-10-20), because at a non-power-up reset the sensor saves its RAM
dynamic calibration data (DCD) to flash (BNO08X datasheet 1000-3927 v1.17
§3.4). Instead, every start switches off the reports the package may have left
enabled that this run does not use (a previous run's orientation report, the
query's raw accelerometer), and the orientation is taken only from the
configured report. Recovery resets are rate-limited (back-off doubling to
300 s, at most 6 per rolling hour, then reopening without a reset); the values
and their reasoning are in `config/bno085.yaml`.

**Periodic DCD saving is off (user decision 2026-10-20).** SH-2 command 0x09
(Configure Periodic DCD Save, SH-2 Reference Manual 1000-3625 rev 1.9 §6.4.7) is
not used. Per datasheet §3.4 the DCD therefore reaches flash only at a
non-power-up reset, which this driver issues on recovery only; calibration
learned since the last such reset is lost at power-off. Acceptable because the
EKF (extended Kalman filter, the localization) fuses only yaw and yaw rate, which
come from the gyroscope. Revisit if roll/pitch are ever used.

Only one process may talk to the sensor: two readers each receive half of every
packet. The driver takes a lock (`/run/lock/gripperx-i2c-<bus>-<addr>.lock`) and a
second instance waits instead of corrupting the first. A lock file left by
another user (e.g. `sudo bno085_query`) is opened read-only; if even that is
refused the driver logs the `sudo rm` needed, every 30 s. If the driver's reader
thread dies, the process exits with status 1.

## Front camera (jpeg_camera_node)

`ros2 launch gripperx_sensors camera.launch.py [video_device:=... params_file:=...]`
starts `jpeg_camera` (Arducam 1080P Low Light UC684, USB 0c45:0261). It is not part
of the motion bringup: on the robot the `gripperx-camera` systemd service
(`Software/pi_env/systemd/units/gripperx-camera.service`, script `gripperx-camera.sh`) starts
this launch at boot, outside the motion stack (no `Requires`/`PartOf` on the bringup), so a
bringup restart or a Mode R (shutdown-button short press: restart of the ROS stack) restart leaves it running. The node asks the camera for MJPG through OpenCV with
`CONVERT_RGB=0`, so the camera's JPEG frames are published undecoded (HWR-22: never
uncompressed YUYV over USB). Parameters: `config/camera_front.yaml`. If the first
frame after an open is not JPEG the node logs an ERROR and exits non-zero; after a
read failure it reopens the device after 1 s.

**Lazy capture.** About once per second the node checks the subscription count of
`image_raw/compressed` and `camera_info`. With none, the capture is released (USB
stream stops, no Pi CPU for it) and the node idles; it reopens when a subscriber
appears. Both transitions are logged once at INFO. A subscriber is therefore needed
to get any frame; `ros2 topic hz` on the Pi counts as one.

**Decoupled publish.** The capture thread only writes the newest frame into a
one-slot buffer; a second thread publishes it. Frames the publisher could not take
in time are dropped (newest wins), so a slow link costs frames, never capture rate.

**Profiles.** The old `width`/`height`/`fps` parameters are replaced by a table
`profiles.<name>.{width,height,fps}` plus `profile` (active name, default `view`).
`view` = 320x240 @ 15 fps, `detail` = 640x480 @ 5 fps (user decision 2026-11-17,
variant c; fps is the camera's own rate, no software throttle).
`ros2 param set /camera/front/camera_front profile detail` (or the camera tile of the
web teleop UI) reopens the camera in that mode; unknown names are rejected; the table
itself, `video_device`, `frame_id` and `camera_info_url` can only be set at start. A parameter batch is validated as a whole before anything is applied. `camera_info` width/height follow
the active mode. A calibration file whose size differs from the active mode is not
used (size-only CameraInfo, warning). After each open the node compares the capture's
reported fourcc and size with the request; if the camera ignored the mode it logs an
ERROR and publishes CameraInfo with the ACTUAL size (frames are not refused: they are
still valid JPEG; consumers must check CameraInfo). Size 0 means the capture could not say.

Measured 2026-11-17 (robot, read-only, WLAN Pi to laptop is the bottleneck):
1280x720 = about 288 kB/frame, only 5 fps arrive, 250 ms latency, WLAN saturated;
640x480 = about 92 kB/frame; 320x240 = about 30 kB/frame, full 15 fps, 25 ms median
latency. A RELIABLE subscriber over WLAN is worse than BEST_EFFORT (3.3 fps, p95
1 s). With a remote subscriber the former node's own capture rate fell from 14.7 to
6.4 fps because `publish()` blocked capture (the reason for the decoupling). The
camera has no JPEG quality control; MJPG sizes 1920x1080, 1280x720, 800x600, 640x480,
320x320, 320x240, 128x128, 96x96, each at 30/25/20/15/10/5 fps. Not measured after
this change in the measurement above (TO-VERIFY on the robot): behaviour of the camera's first
frames after a reopen. Later robot measurements (2026-11-17): idle
Pi CPU about 0.5 % with the device closed, `view` mode at 15 fps with a median latency of 6 ms, a
profile switch in 0.22 s, release about 1 s after the last unsubscribe (about 20 s, the DDS
lease, if a laptop subscriber process dies). The 25 ms figure above (320x240, 15 fps, laptop
subscriber over WLAN) and the 6 ms figure come from different runs; for the 6 ms figure the
subscriber location (on the Pi or over WLAN) and the QoS are not recorded, so which setup it
describes is TO-VERIFY.

Topics under `/camera/front/`: `image_raw/compressed` (sensor_msgs/CompressedImage,
format `jpeg`) and `camera_info`. There is deliberately no `image_raw`; viewers
and detectors subscribe to `image_raw/compressed` (QoS reliable, depth 2; a remote viewer should use BEST_EFFORT, depth 1). The
stamp is the clock time after `read()` returns and includes USB and exposure
latency (unmeasured, TO-VERIFY).

Measured 2026-11-06 on the Pi (old 1280x720 @ 15 mode, continuous capture): 15.5 Hz
published, about 14 % of one core, valid JPEG; 8.8 msg/s received on the laptop over
WiFi; the device is free after SIGINT and a restart works.

Why not `usb_cam` (ros-jazzy-usb-cam 0.8.1-1noble, tested 2026-11-06): it cannot
open the `/dev/v4l/by-id` symlink (resolves it to `/dev/../../video0`);
`pixel_format: mjpeg2rgb` segfaults (SIGSEGV, also on shutdown, leaving the device
busy); `raw_mjpeg` publishes JPEG bytes on `image_raw` labelled `yuv422`, so
image_transport re-encodes them as pixels (colour noise).

TO-VERIFY: `frame_id` is `camera_link` (defined in the URDF camera xacro behind
`use_camera:=false`); the mount pose will live in
`gripperx_geometry/config/geometry.yaml` later. `camera_info_url` is empty, so
CameraInfo carries only width/height (uncalibrated); a `file://` ROS calibration
YAML is loaded when set (vision step 2).

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_sensors
colcon test --packages-select gripperx_sensors && colcon test-result --verbose
```

Desk tests with fake devices (no camera, no I2C bus); none moves hardware. `check_scan_chain_live.py` reads a running scan chain and is for a bench with the stack up.
