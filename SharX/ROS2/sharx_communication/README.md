# SharX ROS 2 Communication

This C++ ROS 2 package implements the initial communication, teleoperation, and movement-status pipeline between the Octopus computer, the SharX Raspberry Pi, and the future ESP32 controller.

## Current system architecture

```text
Octopus computer
       |
       | /octopus/commands
       v
SharX Raspberry Pi
       |
       | /sharx/status
       v
Octopus computer
```

The teleoperation pipeline is:

```text
Keyboard teleop
       |
       | /sharx/cmd_vel
       | geometry_msgs/msg/Twist
       v
Thruster mixer
       |
       | /sharx/thruster_command
       | std_msgs/msg/Float32MultiArray
       v
Movement-status node
       |
       | /sharx/status
       | std_msgs/msg/String containing JSON
       v
Octopus
```

The ESP32 and physical thrusters are not connected to this pipeline yet.

---

## Current functionality

The package currently supports:

1. Publishing a simulated plastic-collection command from Octopus.
2. Receiving the command on the SharX Raspberry Pi.
3. Checking whether the command is addressed to SharX.
4. Validating the target latitude and longitude.
5. Saving the latest target locally.
6. Publishing an acknowledgement to Octopus.
7. Controlling SharX using keyboard teleoperation.
8. Converting velocity commands into normalized left and right thruster commands.
9. Automatically stopping thruster commands after a communication timeout.
10. Publishing `moving` and `stopped` status messages.

---

## Tested environment

The package has been tested with:

* Ubuntu 24.04
* ROS 2 Jazzy
* C++17
* Laptop and Raspberry Pi connected to the same local network
* Laptop architecture: `x86_64`
* Raspberry Pi architecture: `aarch64`

---

## Package contents

```text
sharx_communication/
├── CMakeLists.txt
├── LICENSE
├── package.xml
├── config/
│   └── sharx.yaml
├── launch/
│   ├── dummy_octopus.launch.py
│   ├── local_pipeline.launch.py
│   ├── sharx_receiver.launch.py
│   └── teleop_pipeline.launch.py
└── src/
    ├── dummy_octopus.cpp
    ├── movement_status.cpp
    ├── sharx_receiver.cpp
    ├── sharx_teleop.cpp
    └── thruster_mixer.cpp
```

---

# ROS 2 nodes

## `dummy_octopus`

Simulates the Octopus computer.

It publishes a plastic-collection command and subscribes to SharX status responses.

Published topic:

```text
/octopus/commands
```

Subscribed topic:

```text
/sharx/status
```

---

## `sharx_receiver`

Runs on the SharX Raspberry Pi.

It:

* receives Octopus commands
* parses JSON data
* checks the target device ID
* validates the command type
* validates target coordinates
* stores the latest target
* publishes an acknowledgement

Subscribed topic:

```text
/octopus/commands
```

Published topic:

```text
/sharx/status
```

---

## `sharx_teleop`

Provides keyboard control for SharX.

Published topic:

```text
/sharx/cmd_vel
```

Message type:

```text
geometry_msgs/msg/Twist
```

Keyboard controls:

```text
W       Forward
S       Reverse
A       Turn left
D       Turn right
Space   Stop
Q       Stop and exit
```

Current default values:

```text
Forward/reverse speed: 0.35
Turning speed:         0.35
```

The teleop node publishes one command whenever a key is pressed.

---

## `thruster_mixer`

Converts the velocity command into normalized left and right thruster commands.

Subscribed topic:

```text
/sharx/cmd_vel
```

Message type:

```text
geometry_msgs/msg/Twist
```

Published topic:

```text
/sharx/thruster_command
```

Message type:

```text
std_msgs/msg/Float32MultiArray
```

Output format:

```text
data[0] = left thruster command
data[1] = right thruster command
```

The mixer uses differential-thrust control:

```text
left  = forward - turn
right = forward + turn
```

Current normalized output limit:

```text
-0.35 to +0.35
```

Expected software outputs:

```text
W       [ 0.35,  0.35]
S       [-0.35, -0.35]
A       [-0.35,  0.35]
D       [ 0.35, -0.35]
Space   [ 0.00,  0.00]
```

These values are normalized software commands. They are not ESC PWM values.

The actual ESC configuration still requires confirmation of:

* thruster model
* ESC model
* neutral PWM pulse
* minimum PWM pulse
* maximum PWM pulse
* reverse support
* ESC arming procedure
* left and right motor orientation

### Watchdog safety feature

If no velocity command is received for approximately `0.5` seconds, the mixer publishes:

```text
[0.0, 0.0]
```

This prevents the previous movement command from continuing when communication is interrupted.

---

## `movement_status`

Monitors normalized thruster commands and publishes movement status.

Subscribed topic:

```text
/sharx/thruster_command
```

Published topic:

```text
/sharx/status
```

The node publishes:

```text
moving
```

when either thruster command is non-zero.

It publishes:

```text
stopped
```

when both thruster commands are zero.

Example moving status:

```json
{
  "device_id": "sharx_1",
  "left_thruster": 0.35,
  "mode": "teleop",
  "right_thruster": 0.35,
  "status": "moving"
}
```

Example stopped status:

```json
{
  "device_id": "sharx_1",
  "left_thruster": 0.0,
  "mode": "teleop",
  "right_thruster": 0.0,
  "status": "stopped"
}
```

The status is only published when it changes. This avoids repeatedly publishing identical status messages.

---

# ROS 2 topics

## Octopus to SharX

Topic:

```text
/octopus/commands
```

Message type:

```text
std_msgs/msg/String
```

The string currently contains JSON.

Example command:

```json
{
  "command_id": "task_001",
  "type": "collect_plastic",
  "device_id": "sharx_1",
  "target": {
    "latitude": 48.2621,
    "longitude": 11.6683
  },
  "plastic_type": "bottle",
  "confidence": 0.91,
  "source_robot": "drone_1"
}
```

---

## SharX to Octopus

Topic:

```text
/sharx/status
```

Message type:

```text
std_msgs/msg/String
```

Example task acknowledgement:

```json
{
  "accepted": true,
  "command_id": "task_001",
  "device_id": "sharx_1",
  "latitude": 48.2621,
  "longitude": 11.6683,
  "status": "task_received"
}
```

Example movement status:

```json
{
  "device_id": "sharx_1",
  "left_thruster": 0.35,
  "mode": "teleop",
  "right_thruster": 0.35,
  "status": "moving"
}
```

---

## Teleoperation command

Topic:

```text
/sharx/cmd_vel
```

Message type:

```text
geometry_msgs/msg/Twist
```

Fields currently used:

```text
linear.x
angular.z
```

---

## Thruster command

Topic:

```text
/sharx/thruster_command
```

Message type:

```text
std_msgs/msg/Float32MultiArray
```

Format:

```text
data[0] = left thruster
data[1] = right thruster
```

---

# Configuration

The SharX receiver configuration is stored in:

```text
config/sharx.yaml
```

Default parameters:

```yaml
sharx_receiver:
  ros__parameters:
    device_id: "sharx_1"
    command_topic: "/octopus/commands"
    status_topic: "/sharx/status"
    target_file: "/tmp/sharx_latest_target.json"
```

These values can be changed without modifying the C++ source code.

---

# Dependencies

The package requires:

* ROS 2 Jazzy
* `rclcpp`
* `std_msgs`
* `geometry_msgs`
* `nlohmann-json3-dev`
* `colcon`
* `rosdep`

Install the required tools:

```bash
sudo apt update

sudo apt install -y \
  nlohmann-json3-dev \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-geometry-msgs \
  ros-jazzy-std-msgs
```

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

---

# Clone the repository

```bash
cd ~

git clone https://gitex.itq.de/cirqmind/PlastiX.git

cd ~/PlastiX
```

Switch to the SharX ROS 2 branch:

```bash
git switch SharX_ros2
```

If the branch is not available locally:

```bash
git fetch origin
git switch --track origin/SharX_ros2
```

---

# Build instructions

Create a ROS 2 workspace:

```bash
mkdir -p ~/plastix_sharx_ws/src
```

Copy the package from the repository:

```bash
cp -r \
  ~/PlastiX/SharX/ROS2/sharx_communication \
  ~/plastix_sharx_ws/src/
```

Go to the workspace:

```bash
cd ~/plastix_sharx_ws
```

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Install dependencies:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -y
```

Build the package:

```bash
colcon build \
  --packages-select sharx_communication \
  --symlink-install
```

Source the built workspace:

```bash
source ~/plastix_sharx_ws/install/setup.bash
```

Check available executables:

```bash
ros2 pkg executables sharx_communication
```

Expected output:

```text
sharx_communication dummy_octopus
sharx_communication movement_status
sharx_communication sharx_receiver
sharx_communication sharx_teleop
sharx_communication thruster_mixer
```

---

# Local Octopus-to-SharX test

Run the simulated Octopus and SharX receiver together:

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash

ros2 launch \
  sharx_communication \
  local_pipeline.launch.py
```

Expected sequence:

```text
Dummy Octopus node started
SharX receiver started
Published command
Received command
Target saved
Plastic target accepted
Published status
Received SharX status
```

Verify the saved target:

```bash
cat /tmp/sharx_latest_target.json
```

---

# Network test with laptop and Raspberry Pi

Both systems must:

* run ROS 2 Jazzy
* be connected to the same local network
* use the same ROS domain
* enable subnet discovery

## Raspberry Pi

Run the SharX receiver:

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash

unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch \
  sharx_communication \
  sharx_receiver.launch.py
```

Leave this terminal running.

## Laptop

Run the Octopus simulator:

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash

unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

ros2 launch \
  sharx_communication \
  dummy_octopus.launch.py
```

Expected Raspberry Pi output:

```text
Received command
Target saved
Plastic target accepted
Published status
```

Expected laptop output:

```text
Published command
Received SharX status
```

---

# Teleoperation test

The interactive keyboard node must run in its own terminal. Keyboard input is not reliably forwarded when the node is started through `ros2 launch`.

## Terminal 1: mixer and movement status

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash

ros2 launch \
  sharx_communication \
  teleop_pipeline.launch.py
```

This launch file starts:

```text
thruster_mixer
movement_status
```

## Terminal 2: keyboard teleop

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash

ros2 run \
  sharx_communication \
  sharx_teleop
```

Use:

```text
W       Forward
S       Reverse
A       Turn left
D       Turn right
Space   Stop
Q       Stop and exit
```

## Terminal 3: inspect movement status

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash

ros2 topic echo /sharx/status
```

## Terminal 4: inspect thruster commands

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash

ros2 topic echo /sharx/thruster_command
```

Expected forward output:

```yaml
layout:
  dim: []
  data_offset: 0
data:
- 0.3499999940395355
- 0.3499999940395355
```

The long decimal value is normal floating-point representation and is equivalent to approximately `0.35`.

After approximately `0.5` seconds without another teleop command, the watchdog should publish:

```yaml
data:
- 0.0
- 0.0
```

---

# Verify ROS 2 communication

List nodes:

```bash
ros2 node list
```

List topics:

```bash
ros2 topic list
```

Expected topics include:

```text
/octopus/commands
/sharx/cmd_vel
/sharx/status
/sharx/thruster_command
```

Inspect topic connections:

```bash
ros2 topic info /octopus/commands --verbose
ros2 topic info /sharx/status --verbose
ros2 topic info /sharx/cmd_vel --verbose
ros2 topic info /sharx/thruster_command --verbose
```

---

# Verify receiver parameters

While `sharx_receiver` is running:

```bash
ros2 param get /sharx_receiver device_id
ros2 param get /sharx_receiver command_topic
ros2 param get /sharx_receiver status_topic
ros2 param get /sharx_receiver target_file
```

Expected values:

```text
sharx_1
/octopus/commands
/sharx/status
/tmp/sharx_latest_target.json
```

---

# Troubleshooting

## Package not found

Source ROS 2 and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash
```

---

## Launch file not found

Rebuild and source the workspace:

```bash
cd ~/plastix_sharx_ws

source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select sharx_communication \
  --symlink-install

source install/setup.bash
```

---

## Keyboard keys do not work through the launch file

Run `sharx_teleop` separately:

```bash
ros2 run sharx_communication sharx_teleop
```

The mixer and movement-status nodes may still be started using:

```bash
ros2 launch sharx_communication teleop_pipeline.launch.py
```

---

## Laptop and Pi cannot discover each other

Check settings on both systems:

```bash
echo $ROS_DOMAIN_ID
echo $ROS_AUTOMATIC_DISCOVERY_RANGE
```

Expected:

```text
10
SUBNET
```

Check IP addresses:

```bash
hostname -I
```

Test basic connectivity:

```bash
ping <other-device-ip>
```

---

## JSON dependency not found

Install:

```bash
sudo apt install nlohmann-json3-dev
```

---

## Geometry messages not found

Install:

```bash
sudo apt install ros-jazzy-geometry-msgs
```

---

# Current limitations

* JSON is transported using `std_msgs/msg/String`.
* The real Octopus plastic-detection interface is not connected yet.
* Image transfer is not implemented.
* YOLO detection is not integrated into ROS 2 yet.
* ESP32 and micro-ROS communication are not integrated.
* Normalized thruster commands are not yet converted to ESC PWM.
* Actual thruster and ESC specifications still need confirmation.
* GPS, IMU, ultrasonic, battery, and net-capacity sensor nodes are not integrated.
* Autonomous GPS navigation is not implemented.
* Camera-based final approach is not implemented.
* Waste-collection confirmation is not implemented.
* Physical emergency-stop behaviour must be tested independently of ROS 2.

---

# Next milestones

## 1. Raspberry Pi to ESP32 communication

The next communication stage is:

```text
Octopus
    |
    | ROS 2
    v
Raspberry Pi
    |
    | micro-ROS
    v
ESP32
```

The first Pi-to-ESP32 test should use:

```text
/sharx/esp32/command : "ping"
/sharx/esp32/status  : "alive"
```

---

## 2. Forward thruster commands to ESP32

After the micro-ROS connection works:

```text
/sharx/thruster_command
        |
        v
ESP32 micro-ROS subscriber
        |
        v
ESC PWM conversion
        |
        v
Left and right thrusters
```

The ESP32 must convert normalized values into the confirmed ESC PWM range.

---

## 3. YOLO waste detection

Planned pipeline:

```text
Onboard camera
      |
      v
YOLO inference node
      |
      | /sharx/waste_detection
      v
Navigation controller
```

The model should provide:

* waste class
* confidence
* bounding-box coordinates
* bounding-box centre
* image dimensions

---

## 4. Autonomous navigation

Planned navigation stages:

```text
Octopus target GPS
        |
        v
GPS waypoint navigation
        |
        v
Reach target area
        |
        v
Camera detects waste
        |
        v
Visual approach controller
        |
        v
Waste collection
```

---

## 5. Octopus status reporting

Planned status values include:

```text
task_received
moving
stopped
waste_detected
approaching_waste
target_reached
collection_started
waste_collected
collection_failed
emergency_stop
```

