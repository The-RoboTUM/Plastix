# Eve → Octopus Indoor Drone Detection and Occupancy Grid Demo

> **Paths.** Except for the clone in 2.2 — which necessarily runs before the repository exists — every path in this document is relative to the repository root. `cd` there first, in *every* terminal these steps open.

This README explains how to set up the `eve-octopus` branch on a new laptop, connect to the drone/Pixhawk/Raspberry Pi, start the detector, start the Octopus dashboard, and run the indoor static occupancy-grid demo.

> **Nur starten?** Der Startablauf steht in **[docs/SETUP.md](docs/SETUP.md)**. Diese README
> deckt Konzepte, Erstinstallation, Presentation Flow und weiteres Troubleshooting ab.
> Dashboard-Test ohne Drohne: [octopus-dashboard/SETUP_NO_DRONE.md](octopus-dashboard/SETUP_NO_DRONE.md).
> Übersicht über alle weiteren Dokumente: [docs/README.md](docs/README.md).

The current indoor demo pipeline is:

```text
Drone camera
→ detector
→ confirmed ROS2 detections
→ Pixhawk orientation + manual height projection
→ yaw-aligned Indoor Static Mission Map
→ Global Mission Grid / occupancy-style map
→ dashboard + robot-relevant ROS outputs
```

For the indoor demo, use:

```text
Mapping mode: Indoor Static Mission Map
Grid source: Global Mission Grid
Robot output: Global Mission Grid only
```

Do **not** use the Local Camera Grid as the robot map. The Local Camera Grid is only a camera/debug view.

---

## 1. Important concepts

### Local Camera Grid

The Local Camera Grid is a debug grid.

```text
Input:
u/v image coordinates + assumed camera footprint

Frame:
camera_footprint

Purpose:
debugging the detector and camera view
```

It answers:

```text
Where is the detected object inside the current camera image/footprint?
```

It is **not** meant for robot navigation.

---

### Global Mission Grid

The Global Mission Grid is the robot-relevant map.

```text
Input:
u/v image coordinates
+ camera intrinsics
+ camera-to-drone transform
+ Pixhawk orientation
+ manual height
+ fixed indoor origin
+ startup yaw alignment

Frame:
map

Purpose:
shared mission map / occupancy-style grid / trash grid for other robots
```

It answers:

```text
Where is the detected object in the shared indoor map frame?
```

This is what the other robot should use.

---

### Indoor Static Mission Map

For indoor demos, the drone is fixed or hanging. PX4 local x/y can drift indoors, so Octopus ignores PX4 x/y and uses a fixed map origin.

The indoor static mode uses:

```text
PX4 roll/pitch/yaw: yes
manual height: yes
PX4 x/y position: ignored/frozen
startup yaw alignment: yes
```

The startup yaw alignment means:

```text
map +y = drone/camera front direction at Octopus startup
map +x = drone/camera right direction at Octopus startup
```

So before starting Octopus, physically align the drone:

```text
Drone fixed in the middle of the demo area.
Drone/camera front points toward the front of the room.
Robot/home station is behind the drone.
Robot faces the same direction as the drone.
```

---

## 2. Repository setup on a new laptop

### 2.1 Install base tools

This project was tested on Ubuntu 22.04 with ROS2 Humble.

Install ROS2 Humble first:

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-pip \
  python3-venv \
  python3-colcon-common-extensions \
  ros-humble-desktop \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport
```

Source ROS2:

```bash
source /opt/ros/humble/setup.bash
```

Optional but useful:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

### 2.2 Clone the repository and switch branch

```bash
mkdir -p ~/projects
cd ~/projects

git clone https://gitex.itq.de/cirqmind/PlastiX.git
cd PlastiX

git fetch origin
git checkout eve-octopus
git pull origin eve-octopus
```

Check:

```bash
git status
git log --oneline -8
```

You should see recent commits such as:

```text
Add indoor static yaw alignment
Fix indoor demo freshness display
Add indoor demo checklist and status page
Add indoor static mission map mode
```

---

## 3. Required downloads / files

### 3.1 PX4 ROS2 message package

The Octopus transform node uses PX4 ROS2 messages:

```text
px4_msgs/msg/VehicleOdometry
px4_msgs/msg/VehicleLocalPosition
```

Check whether `px4_msgs` already exists:

```bash
ls Octopus/ros2_ws/src/px4_msgs
```

If it does not exist, clone it into the Octopus ROS2 workspace:

```bash
cd Octopus/ros2_ws/src

git clone https://github.com/PX4/px4_msgs.git
```

Important: the `px4_msgs` version should match the PX4 firmware used on the Pixhawk. If message fields mismatch, the bridge may build but topics may not decode correctly.

---

### 3.2 Detector model

The detector expects this model path:

```text
eve/Software/detect-and-localize/data/models/indoor_v8s.pt
```

Create the model folder:

```bash
mkdir -p eve/Software/detect-and-localize/data/models
```

Copy or download the model from the team/shared storage into:

```bash
eve/Software/detect-and-localize/data/models/indoor_v8s.pt
```

Example if the model file is in your Downloads folder:

```bash
cp ~/Downloads/indoor_v8s.pt \
  eve/Software/detect-and-localize/data/models/indoor_v8s.pt
```

Check:

```bash
ls -lh eve/Software/detect-and-localize/data/models/indoor_v8s.pt
```

---

### 3.3 Detector Python environment

Set up the detector virtual environment:

```bash
cd eve/Software/detect-and-localize

python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip
```

If a requirements file exists, install it:

```bash
if [ -f requirements.txt ]; then
  pip install -r requirements.txt
fi
```

If the detector complains about missing Python packages later, install the missing package inside this `.venv`.

---

## 4. Build ROS2 workspaces

### 4.1 Build Eve ROS2 workspace

```bash
cd eve/Software/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

### 4.2 Build Octopus ROS2 workspace

```bash
cd Octopus/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

If you only changed the camera transform package:

```bash
cd Octopus/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --packages-select octopus_camera_transform --symlink-install

source install/setup.bash
```

---

## 5. Connect to the drone for the first time

The drone setup uses:

```text
Raspberry Pi + Pixhawk
Pi hostname/SSH alias: eve-pi
ROS_DOMAIN_ID: 0
ROS_LOCALHOST_ONLY: 0
```

Make sure your laptop is connected to the same network/hotspot as the drone/Pi.

Test connection:

```bash
ping eve-pi
```

SSH into the Pi:

```bash
ssh eve-pi
```

If `eve-pi` does not resolve, use the Pi IP address:

```bash
ssh <user>@<pi-ip-address>
```

If this is the first time connecting, accept the SSH fingerprint.

Check Pi ROS2 environment after SSH:

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

---

## 6. Safety before demo

Before starting the indoor demo:

```text
- Remove propellers.
- Secure the drone so it cannot move.
- Power the Pixhawk and Raspberry Pi.
- Put a clearly visible trash/object item inside the camera view.
- Use good lighting.
- Do not run fake detections.
- Start Octopus only after the drone is physically aligned.
```

Physical layout:

```text
Drone/camera in the middle of the demo area.
Drone/camera front points toward the front of the room.
Robot home station is directly behind the drone.
Robot faces the same direction as the drone.
```

Indoor map convention:

```text
origin (0,0) = back-left of demo area
+x = right
+y = drone/camera front direction at Octopus startup
```

Current indoor static parameters:

```text
drone/camera fixed map position:
x = 2.23 m
y = 1.67 m

example home station:
x = 2.23 m
y = 0.67 m
```

---

# 7. Start the complete indoor demo

Der vollständige Startablauf steht in [docs/SETUP.md](docs/SETUP.md) — Sicherheitsschritte,
die vier Terminals (PX4-Brücke und Kamera auf der Pi, Detektor und Octopus-Stack auf dem
Laptop), Health-Check, Stoppen und Troubleshooting.

Kurzfassung, wenn alles eingerichtet ist:

```bash
OCTOPUS_MAPPING_MODE=indoor_static_mission ./Octopus/scripts/start_octopus_debug_stack.sh
```

Das Skript startet das Dashboard-Backend und alle sieben ROS-Nodes. Danach:

```text
http://127.0.0.1:8000/dashboard.html
```

---

## 8. Presentation flow

Show this first:

```text
http://127.0.0.1:8000/indoor_demo.html
```

This page is the presentation/status page. It should show:

```text
Camera: live
Detector confirmed: live
PX4 odometry: fresh
Transform: ready
Mapping mode: indoor_static_mission
Local camera grid: live
Global mission map: live
Yaw alignment: map +y = drone front
```

Then show:

```text
http://127.0.0.1:8000/dashboard.html
```

Use the dashboard to show:

```text
camera debug image
detections
Global Mission Grid
map patch updates
```

Demo explanation:

```text
The detector outputs normalized camera coordinates u/v. Octopus uses the calibrated camera model, manual height, and Pixhawk orientation to project the detection onto the ground plane.

Because this is an indoor static demo, the drone x/y origin is frozen and the current drone yaw at startup is locked as the map-forward direction. The result is written into the Global Mission Grid, which is the map the other robot can use.
```

---

## 9. Robot-relevant outputs

The other robot should use the global outputs:

```text
/octopus/detections_world
/octopus/global_map
/octopus/coverage_grid
/octopus/trash_grid
```

Backend debug endpoint:

```text
/api/map_patch/latest
```

Do not use this for robot navigation:

```text
/api/local_camera_grid/latest
Local Camera Grid
camera_footprint frame
```

That is only for camera/debug visualization.

---

## 10. Troubleshooting

### PX4 odometry missing

Symptom:

```text
/fmu/out/vehicle_odometry does not appear to be published
```

Fix: restart Terminal 1 on the Pi:

```bash
sudo pkill -f MicroXRCEAgent || true

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

sudo --preserve-env=ROS_DOMAIN_ID,ROS_LOCALHOST_ONLY \
  MicroXRCEAgent serial --dev /dev/serial0 -b 921600 -v 6
```

Then check again:

```bash
timeout 8 ros2 topic hz /fmu/out/vehicle_odometry
```

---

### Detector confirmed not publishing

Symptom:

```text
/detector_node/confirmed has no rate
```

Fix: make sure the detector terminal is running and the object is visible.

Use the easier indoor demo detector settings:

```text
-p thresh:=0.20
-p confirm_frames:=1
```

Check debug detections:

```bash
timeout 8 ros2 topic hz /detector_node/detections_debug
```

---

### Local camera grid empty

Usually means:

```text
/detector_node/confirmed is not publishing
```

Check:

```bash
timeout 8 ros2 topic hz /detector_node/confirmed
```

---

### Global map patch empty

Usually means one of these is missing:

```text
/fmu/out/vehicle_odometry
/detector_node/confirmed
flight_camera_transform_node ready state
```

Check transform status:

```bash
ros2 topic echo --once /octopus/flight_camera_transform/status --field data | head -n 1 | python3 -m json.tool
```

Important fields:

```text
"state": "ready"
"indoor_static_yaw_zero_rad": <number>
"last_projection_error": null
"last_transformed_detection_count": > 0
```

---

### `indoor_static_yaw_zero_rad` is null

This means yaw alignment has not locked yet.

Causes:

```text
- no fresh PX4 odometry
- no successful projection yet
- transform node started before PX4 odometry was alive
```

Fix:

```bash

./Octopus/scripts/stop_octopus_debug_stack.sh

OCTOPUS_MAPPING_MODE=indoor_static_mission ./Octopus/scripts/start_octopus_debug_stack.sh
```

Only start Octopus after PX4 odometry is already publishing.

---

### Status command shows `Extra data`

Use this safer command:

```bash
ros2 topic echo --once /octopus/flight_camera_transform/status --field data | head -n 1 | python3 -m json.tool
```

---

### `failed to shutdown: rcl_shutdown already called`

This can appear after `ros2 topic hz`.

If the topic printed rates before the warning, ignore it.

---

## 11. Successful demo checklist

A successful indoor demo has:

```text
[ ] Propellers removed
[ ] Drone fixed and physically aligned
[ ] Pi reachable over SSH
[ ] MicroXRCEAgent running
[ ] Camera node running
[ ] Detector running
[ ] /camera/image_raw/compressed live
[ ] /fmu/out/vehicle_odometry live
[ ] /detector_node/confirmed live
[ ] Octopus started with OCTOPUS_MAPPING_MODE=indoor_static_mission
[ ] transform_mode = indoor_static_mission
[ ] indoor_static_yaw_zero_rad = number
[ ] Transform state = ready
[ ] Local camera grid endpoint = status ok
[ ] Global map patch endpoint = status ok
[ ] Global map timestamp updates live
[ ] Dashboard shows Global Mission Grid
[ ] No fake detections running
```
