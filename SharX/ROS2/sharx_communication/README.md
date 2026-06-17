# SharX ROS 2 Communication

This C++ ROS 2 package implements the initial communication pipeline between the Octopus computer and the SharX Raspberry Pi.

## Current communication pipeline

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

The current implementation:

1. Publishes a simulated plastic-collection command from Octopus.
2. Receives the command on the SharX Raspberry Pi.
3. Checks whether the command is addressed to SharX.
4. Validates the target latitude and longitude.
5. Saves the latest target locally.
6. Publishes an acknowledgement to Octopus.

## Tested environment

* Ubuntu 24.04
* ROS 2 Jazzy
* Laptop and Raspberry Pi connected to the same local network

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
│   └── sharx_receiver.launch.py
└── src/
    ├── dummy_octopus.cpp
    └── sharx_receiver.cpp
```

## ROS 2 topics

### Octopus to SharX

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

### SharX to Octopus

Topic:

```text
/sharx/status
```

Message type:

```text
std_msgs/msg/String
```

Example acknowledgement:

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

## Configuration

The receiver configuration is stored in:

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

## Dependencies

The package requires:

* ROS 2 Jazzy
* `rclcpp`
* `std_msgs`
* `nlohmann-json3-dev`
* `colcon`
* `rosdep`

Install the required development tools:

```bash
sudo apt update
sudo apt install -y \
  nlohmann-json3-dev \
  python3-colcon-common-extensions \
  python3-rosdep
```

## Build instructions

Clone the repository:

```bash
cd ~
git clone https://gitex.itq.de/cirqmind/PlastiX.git
cd ~/PlastiX
```

Switch to the ROS 2 branch:

```bash
git switch SharX_ros2
```

Create a ROS 2 workspace:

```bash
mkdir -p ~/plastix_sharx_ws/src
```

Copy the package:

```bash
cp -r \
  ~/PlastiX/SharX/ROS2/sharx_communication \
  ~/plastix_sharx_ws/src/
```

Install dependencies:

```bash
cd ~/plastix_sharx_ws
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -y
```

Build:

```bash
colcon build \
  --packages-select sharx_communication \
  --symlink-install
```

Source the workspace:

```bash
source ~/plastix_sharx_ws/install/setup.bash
```

## Local test on one computer

Run both nodes together:

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash

ros2 launch \
  sharx_communication \
  local_pipeline.launch.py
```

Expected result:

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

## Network test with laptop and Raspberry Pi

Both systems must:

* run ROS 2 Jazzy
* be connected to the same local network
* use the same ROS domain
* enable subnet discovery

### Raspberry Pi

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

### Laptop

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

## Expected network-test result

The Raspberry Pi should show:

```text
Received command
Target saved
Plastic target accepted
Published status
```

The laptop should show:

```text
Published command
Received SharX status
```

## Verify communication

On either machine:

```bash
ros2 node list
ros2 topic list
```

Expected topics:

```text
/octopus/commands
/sharx/status
```

Inspect the topic connections:

```bash
ros2 topic info /octopus/commands --verbose
ros2 topic info /sharx/status --verbose
```

## Verify receiver parameters

While the receiver is running:

```bash
ros2 param get /sharx_receiver device_id
ros2 param get /sharx_receiver command_topic
ros2 param get /sharx_receiver status_topic
ros2 param get /sharx_receiver target_file
```

## Verify the saved target

On the Raspberry Pi:

```bash
cat /tmp/sharx_latest_target.json
```

## Current limitations

* JSON is currently transported using `std_msgs/msg/String`.
* The real Octopus plastic-detection message is not connected yet.
* Image transfer is not implemented.
* ESP32 and micro-ROS communication are not integrated.
* Sensors, autonomous navigation, and thruster control are not included.

## Next milestone

The next stage is:

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

The first Raspberry Pi-to-ESP32 test should use:

```text
/sharx/esp32/command : "ping"
/sharx/esp32/status  : "alive"
```

