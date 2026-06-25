# SharX Raspberry Pi Setup Guide

This guide explains how to prepare the SharX Raspberry Pi for:

* ROS 2 Jazzy
* Octopus-to-SharX communication
* keyboard teleoperation
* thruster-command generation
* YOLO floating-waste detection
* autonomous waste-following commands

The current setup has been tested with:

* Raspberry Pi 5
* Ubuntu 24.04
* ROS 2 Jazzy
* Python 3.12
* USB Logitech C270 webcam
* `aarch64` architecture

---

# 1. System requirements

Required hardware:

* Raspberry Pi 5
* Ubuntu 24.04
* internet connection
* USB webcam
* keyboard and monitor, or SSH/VNC access
* microSD card or SSD with sufficient free space

Recommended free storage:

```text
At least 10 GB
```

Check the system:

```bash
uname -m
python3 --version
lsb_release -a
```

Expected architecture:

```text
aarch64
```

Expected Ubuntu version:

```text
Ubuntu 24.04
```

---

# 2. Update the Raspberry Pi

```bash
sudo apt update
sudo apt upgrade -y
```

Install common development tools:

```bash
sudo apt install -y \
  git \
  curl \
  wget \
  build-essential \
  cmake \
  nano \
  rsync \
  python3-pip \
  python3.12-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  nlohmann-json3-dev \
  v4l-utils \
  ffmpeg
```

---

# 3. Install ROS 2 Jazzy

## 3.1 Configure locale

```bash
sudo apt install -y locales

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8
```

Verify:

```bash
locale
```

---

## 3.2 Enable Ubuntu Universe repository

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

---

## 3.3 Add the ROS 2 repository

Install required tools:

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
```

Download the ROS signing key:

```bash
sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository:

```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update package information:

```bash
sudo apt update
```

---

## 3.4 Install ROS 2 Jazzy

For a desktop installation:

```bash
sudo apt install -y ros-jazzy-desktop
```

For a lighter installation:

```bash
sudo apt install -y ros-jazzy-ros-base
```

Install required ROS message packages:

```bash
sudo apt install -y \
  ros-jazzy-rclcpp \
  ros-jazzy-rclpy \
  ros-jazzy-std-msgs \
  ros-jazzy-geometry-msgs
```

---

## 3.5 Source ROS 2 automatically

Add ROS 2 to `.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

Apply it:

```bash
source ~/.bashrc
```

Verify:

```bash
ros2 --help
```

---

# 4. Initialize rosdep

Run:

```bash
sudo rosdep init
```

If it says rosdep has already been initialized, continue.

Update rosdep:

```bash
rosdep update
```

---

# 5. Clone the PlastiX repository

Go to the home directory:

```bash
cd ~
```

Clone:

```bash
git clone https://gitex.itq.de/cirqmind/PlastiX.git
```

Enter the GitEx username and password when requested.

Open the repository:

```bash
cd ~/PlastiX
```

Fetch branches:

```bash
git fetch origin
```

Switch to the SharX ROS 2 branch:

```bash
git switch --track origin/SharX_ros2
```

If the local branch already exists:

```bash
git switch SharX_ros2
git pull --rebase origin SharX_ros2
```

Verify:

```bash
git branch --show-current
```

Expected:

```text
SharX_ros2
```

Check ROS 2 packages:

```bash
ls ~/PlastiX/SharX/ROS2
```

Expected folders:

```text
sharx_communication
sharx_vision
```

---

# 6. Create the SharX ROS 2 workspace

Create the workspace:

```bash
mkdir -p ~/plastix_sharx_ws/src
```

Copy the ROS packages:

```bash
rsync -av --delete \
  ~/PlastiX/SharX/ROS2/sharx_communication/ \
  ~/plastix_sharx_ws/src/sharx_communication/
```

```bash
rsync -av --delete \
  ~/PlastiX/SharX/ROS2/sharx_vision/ \
  ~/plastix_sharx_ws/src/sharx_vision/
```

Verify:

```bash
ls ~/plastix_sharx_ws/src
```

Expected:

```text
sharx_communication
sharx_vision
```

---

# 7. Install ROS package dependencies

```bash
cd ~/plastix_sharx_ws

source /opt/ros/jazzy/setup.bash
```

Run:

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -y
```

---

# 8. Create the YOLO Python environment

Create the YOLO project directory:

```bash
mkdir -p ~/sharx_yolo_pi
cd ~/sharx_yolo_pi
```

Create a Python virtual environment:

```bash
python3 -m venv .venv
```

Activate it:

```bash
source .venv/bin/activate
```

Upgrade Python packaging tools:

```bash
pip install --upgrade pip setuptools wheel
```

Install YOLO and OpenCV:

```bash
pip install ultralytics opencv-python
```

Verify:

```bash
python -c "from ultralytics import YOLO; import cv2; print('YOLO and OpenCV ready')"
```

Expected:

```text
YOLO and OpenCV ready
```

---

# 9. Copy the trained model to the Pi

The trained model is not stored in Git.

The model file should be copied to:

```text
/home/vineeth/sharx_yolo_pi/best.pt
```

From another computer on the same network:

```bash
scp \
  /path/to/best.pt \
  vineeth@<PI_IP_ADDRESS>:~/sharx_yolo_pi/best.pt
```

Example:

```bash
scp \
  ~/sharx_yolo_project/runs/detect/runs/floating_waste_v1/weights/best.pt \
  vineeth@192.168.0.105:~/sharx_yolo_pi/best.pt
```

Verify on the Pi:

```bash
ls -lh ~/sharx_yolo_pi/best.pt
```

---

# 10. Verify the USB webcam

List camera devices:

```bash
v4l2-ctl --list-devices
```

A Logitech C270 webcam should appear similar to:

```text
C270 HD WEBCAM:
    /dev/video0
    /dev/video1
```

Check supported formats:

```bash
v4l2-ctl \
  --device=/dev/video0 \
  --list-formats-ext
```

Test the webcam:

```bash
ffplay \
  -f v4l2 \
  -video_size 640x480 \
  -i /dev/video0
```

Press `Q` to close the window.

Do not use Raspberry Pi ISP devices such as:

```text
/dev/video20
/dev/video21
...
```

The USB webcam stream is normally:

```text
/dev/video0
```

---

# 11. Test YOLO directly on the Pi

Activate the YOLO environment:

```bash
cd ~/sharx_yolo_pi
source .venv/bin/activate
```

Run:

```bash
yolo detect predict \
  model=best.pt \
  source=/dev/video0 \
  imgsz=320 \
  conf=0.25 \
  device=cpu \
  show=True
```

Press `Q` to stop.

The Raspberry Pi uses CPU inference, so:

```text
device=cpu
```

is required.

For better detection accuracy, use:

```text
imgsz=512
```

For better speed, use:

```text
imgsz=320
```

---

# 12. Build the ROS 2 workspace

Activate ROS 2 and the YOLO environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/sharx_yolo_pi/.venv/bin/activate
```

Build:

```bash
cd ~/plastix_sharx_ws

colcon build \
  --packages-select sharx_communication sharx_vision \
  --symlink-install
```

Source the workspace:

```bash
source ~/plastix_sharx_ws/install/setup.bash
```

Verify executables:

```bash
ros2 pkg executables sharx_communication
```

Expected:

```text
sharx_communication dummy_octopus
sharx_communication movement_status
sharx_communication sharx_receiver
sharx_communication sharx_teleop
sharx_communication thruster_mixer
```

Check vision nodes:

```bash
ros2 pkg executables sharx_vision
```

Expected:

```text
sharx_vision waste_detector
sharx_vision waste_follower
```

---

# 13. Allow ROS 2 Python nodes to access YOLO packages

The generated ROS 2 Python executable currently uses:

```text
/usr/bin/python3
```

Ultralytics is installed inside:

```text
~/sharx_yolo_pi/.venv
```

Export the virtual-environment package path:

```bash
export PYTHONPATH="$HOME/sharx_yolo_pi/.venv/lib/python3.12/site-packages:$PYTHONPATH"
```

Verify:

```bash
/usr/bin/python3 -c "import ultralytics, cv2, torch; print('ROS Python can access YOLO')"
```

Expected:

```text
ROS Python can access YOLO
```

This export is required before running `waste_detector`.

---

# 14. Recommended environment script

Create a setup script:

```bash
nano ~/sharx_setup.sh
```

Paste:

```bash
#!/bin/bash

source /opt/ros/jazzy/setup.bash

if [ -f "$HOME/sharx_yolo_pi/.venv/bin/activate" ]; then
  source "$HOME/sharx_yolo_pi/.venv/bin/activate"
fi

if [ -f "$HOME/plastix_sharx_ws/install/setup.bash" ]; then
  source "$HOME/plastix_sharx_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

export PYTHONPATH="$HOME/sharx_yolo_pi/.venv/lib/python3.12/site-packages:$PYTHONPATH"

echo "SharX environment loaded"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
```

Save and make it executable:

```bash
chmod +x ~/sharx_setup.sh
```

Use it in every terminal:

```bash
source ~/sharx_setup.sh
```

Optional automatic setup:

```bash
echo "source ~/sharx_setup.sh" >> ~/.bashrc
```

---

# 15. Test Octopus-to-SharX communication

## Local test on the Pi

```bash
source ~/sharx_setup.sh
```

Run:

```bash
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

# 16. Network communication setup

All computers must use the same ROS domain:

```bash
export ROS_DOMAIN_ID=10
```

Enable subnet discovery:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Disable localhost-only communication:

```bash
unset ROS_LOCALHOST_ONLY
```

Check the Pi IP:

```bash
hostname -I
```

Test communication from another computer:

```bash
ping <PI_IP_ADDRESS>
```

---

# 17. Run the ROS 2 YOLO detector

Load the environment:

```bash
source ~/sharx_setup.sh
```

Run:

```bash
ros2 run sharx_vision waste_detector \
  --ros-args \
  -p model_path:=/home/vineeth/sharx_yolo_pi/best.pt \
  -p camera_index:=0 \
  -p image_size:=320 \
  -p device:=cpu \
  -p show_image:=true
```

Detection topic:

```text
/sharx/waste_detection
```

Check it in another terminal:

```bash
source ~/sharx_setup.sh

ros2 topic echo /sharx/waste_detection
```

Example:

```json
{
  "detected": true,
  "class_name": "floating_waste",
  "confidence": 0.88,
  "center_x": 430,
  "center_y": 280,
  "box_width": 120,
  "box_height": 240,
  "image_width": 640,
  "image_height": 480
}
```

---

# 18. Run the waste follower

```bash
source ~/sharx_setup.sh
```

Run:

```bash
ros2 run sharx_vision waste_follower \
  --ros-args \
  -p stop_area_ratio:=0.45
```

Output topic:

```text
/sharx/cmd_vel_auto
```

Inspect:

```bash
ros2 topic echo /sharx/cmd_vel_auto
```

Expected behaviour:

```text
Object left       -> turn left
Object right      -> turn right
Object centered   -> move forward
Object very close -> stop
No detection      -> stop
```

---

# 19. Run the thruster mixer

```bash
source ~/sharx_setup.sh
```

Run:

```bash
ros2 run sharx_communication thruster_mixer \
  --ros-args \
  -r /sharx/cmd_vel:=/sharx/cmd_vel_auto
```

Output topic:

```text
/sharx/thruster_command
```

Inspect:

```bash
ros2 topic echo /sharx/thruster_command
```

Expected values:

```text
Target left       -> approximately [-0.22, 0.22]
Target right      -> approximately [0.22, -0.22]
Target centered   -> approximately [0.18, 0.18]
Target close      -> [0.0, 0.0]
No target         -> [0.0, 0.0]
```

These values are normalized software commands, not ESC PWM values.

---

# 20. Run movement status

```bash
source ~/sharx_setup.sh
```

For autonomous mode:

```bash
ros2 run sharx_communication movement_status \
  --ros-args \
  -p mode:=autonomous
```

Inspect status:

```bash
ros2 topic echo /sharx/status
```

Expected:

```json
{
  "device_id": "sharx_1",
  "left_thruster": 0.18,
  "mode": "autonomous",
  "right_thruster": 0.18,
  "status": "moving"
}
```

When stopped:

```json
{
  "device_id": "sharx_1",
  "left_thruster": 0.0,
  "mode": "autonomous",
  "right_thruster": 0.0,
  "status": "stopped"
}
```

Verify the mode parameter:

```bash
ros2 param get /movement_status mode
```

Expected:

```text
String value is: autonomous
```

---

# 21. Full autonomous software test

Use separate terminals.

## Terminal 1: detector

```bash
source ~/sharx_setup.sh

ros2 run sharx_vision waste_detector \
  --ros-args \
  -p model_path:=/home/vineeth/sharx_yolo_pi/best.pt \
  -p camera_index:=0 \
  -p image_size:=320 \
  -p device:=cpu \
  -p show_image:=true
```

## Terminal 2: follower

```bash
source ~/sharx_setup.sh

ros2 run sharx_vision waste_follower \
  --ros-args \
  -p stop_area_ratio:=0.45
```

## Terminal 3: mixer

```bash
source ~/sharx_setup.sh

ros2 run sharx_communication thruster_mixer \
  --ros-args \
  -r /sharx/cmd_vel:=/sharx/cmd_vel_auto
```

## Terminal 4: movement status

```bash
source ~/sharx_setup.sh

ros2 run sharx_communication movement_status \
  --ros-args \
  -p mode:=autonomous
```

## Terminal 5: monitor output

```bash
source ~/sharx_setup.sh

ros2 topic echo /sharx/thruster_command
```

or:

```bash
ros2 topic echo /sharx/status
```

Full pipeline:

```text
USB webcam
    |
    v
YOLO waste detector
    |
    | /sharx/waste_detection
    v
Waste follower
    |
    | /sharx/cmd_vel_auto
    v
Thruster mixer
    |
    | /sharx/thruster_command
    v
Movement status
    |
    | /sharx/status
    v
Octopus
```

---

# 22. Update the Pi from Git

Before pulling updates, check for local modifications:

```bash
cd ~/PlastiX
git status
```

Pull the latest branch:

```bash
git switch SharX_ros2
git pull --rebase origin SharX_ros2
```

Copy updated packages:

```bash
rsync -av --delete \
  ~/PlastiX/SharX/ROS2/sharx_communication/ \
  ~/plastix_sharx_ws/src/sharx_communication/
```

```bash
rsync -av --delete \
  ~/PlastiX/SharX/ROS2/sharx_vision/ \
  ~/plastix_sharx_ws/src/sharx_vision/
```

Rebuild:

```bash
source ~/sharx_setup.sh

cd ~/plastix_sharx_ws

colcon build \
  --packages-select sharx_communication sharx_vision \
  --symlink-install
```

Reload:

```bash
source ~/plastix_sharx_ws/install/setup.bash
```

---

# 23. Save Pi changes to Git

Changes should normally be made in:

```text
~/PlastiX/SharX/ROS2/
```

If a file was edited in the workspace first, copy it back to the repository.

Example:

```bash
cp \
  ~/plastix_sharx_ws/src/sharx_communication/src/movement_status.cpp \
  ~/PlastiX/SharX/ROS2/sharx_communication/src/movement_status.cpp
```

Review:

```bash
cd ~/PlastiX
git status --short
git diff
```

Commit:

```bash
git add SharX/ROS2
git commit -m "Describe the change"
```

Sync:

```bash
git fetch origin SharX_ros2
git rebase origin/SharX_ros2
```

Push:

```bash
git push origin SharX_ros2
```

Do not use force push.

---

# 24. Troubleshooting

## ROS package not found

```bash
source /opt/ros/jazzy/setup.bash
source ~/plastix_sharx_ws/install/setup.bash
```

---

## Ultralytics module not found

Error:

```text
ModuleNotFoundError: No module named 'ultralytics'
```

Fix:

```bash
export PYTHONPATH="$HOME/sharx_yolo_pi/.venv/lib/python3.12/site-packages:$PYTHONPATH"
```

Verify:

```bash
/usr/bin/python3 -c "import ultralytics"
```

---

## Webcam does not open

Check:

```bash
v4l2-ctl --list-devices
```

Confirm `/dev/video0` belongs to the USB webcam.

Check permissions:

```bash
groups
```

Add the user to the video group:

```bash
sudo usermod -aG video "$USER"
```

Then reboot:

```bash
sudo reboot
```

---

## No detection topic

Check nodes:

```bash
ros2 node list
```

Check topics:

```bash
ros2 topic list
```

Check the detector output:

```bash
ros2 topic echo /sharx/waste_detection
```

---

## Thruster output is always zero

Check detection:

```bash
ros2 topic echo /sharx/waste_detection
```

Check follower output:

```bash
ros2 topic echo /sharx/cmd_vel_auto
```

The follower intentionally publishes zero when:

* no object is detected
* the object is too close
* image dimensions are invalid
* the detection stream stops

Increase the indoor test stop threshold:

```bash
ros2 run sharx_vision waste_follower \
  --ros-args \
  -p stop_area_ratio:=0.45
```

---

## Status shows teleop during autonomous operation

Run:

```bash
ros2 run sharx_communication movement_status \
  --ros-args \
  -p mode:=autonomous
```

Verify:

```bash
ros2 param get /movement_status mode
```

---

## Laptop and Pi cannot discover each other

On both systems:

```bash
export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
```

Check IP addresses:

```bash
hostname -I
```

Test connection:

```bash
ping <OTHER_DEVICE_IP>
```

---

# 25. Current limitations

* ESC and physical thrusters are not yet connected.
* `/sharx/thruster_command` contains normalized values only.
* ESP32 micro-ROS communication is not yet integrated.
* GPS and IMU navigation are not yet implemented.
* The current stop distance is estimated from bounding-box area.
* The trained YOLO model is stored outside Git.
* The Python environment currently requires a `PYTHONPATH` export.
* Physical emergency-stop behaviour must be implemented and tested before water operation.

---

# 26. Safety

Do not connect or activate physical thrusters until the following are confirmed:

* ESC model
* thruster model
* ESP32 output GPIO pins
* neutral PWM value
* minimum PWM value
* maximum PWM value
* ESC arming sequence
* reverse support
* motor direction
* emergency-stop mechanism
* propeller safety area

Always test software output before attaching propellers.

---

# 27. Next milestone

The next integration stage is:

```text
Raspberry Pi
     |
     | /sharx/esp32/command
     v
ESP32 micro-ROS
     |
     | /sharx/esp32/status
     v
Raspberry Pi
```

Initial test:

```text
Pi publishes:  "ping"
ESP32 replies: "alive"
```

After the ping/alive test works:

```text
/sharx/thruster_command
        |
        v
ESP32
        |
        v
ESC PWM
        |
        v
Left and right thrusters
```
