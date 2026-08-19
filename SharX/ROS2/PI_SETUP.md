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
* Ubuntu 24.04.4 LTS
* ROS 2 Jazzy
* Python 3.12
* Raspberry Pi Camera Module 3 (`IMX708`)
* Raspberry Pi 5 `PiSP` camera pipeline
* `rpicam-apps` + Raspberry Pi `libcamera`
* `aarch64` architecture

---

# 1. System requirements

Required hardware:

* Raspberry Pi 5
* Ubuntu 24.04
* internet connection
* Raspberry Pi Camera Module 3 (`IMX708`)
* Raspberry Pi 5-compatible camera ribbon cable
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

The camera setup in this guide is for the Raspberry Pi CSI camera stack. It does **not** use a Raspberry Pi Camera Module 3 or `cv2.VideoCapture(0)` for the final SharX detector.

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

First check whether the ROS 2 repository is already configured:

```bash
sudo apt update
```

If the output already contains something similar to:

```text
http://packages.ros.org/ros2/ubuntu noble InRelease
```

do **not** add a second ROS source entry.

If the ROS repository is not configured, use the ROS apt-source package method rather than creating multiple manual source files.

A duplicate ROS source can cause an error similar to:

```text
Conflicting values set for option Signed-By
```

If that happens and `/etc/apt/sources.list.d/ros2.list` was manually created while another ROS source already exists, remove only the duplicate manual file:

```bash
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
```

Do not remove an existing working ROS source configuration unless you know it is the duplicate.

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

Clone over HTTPS:

```bash
git clone https://gitex.itq.de/cirqmind/PlastiX.git
```

If GitEx rejects the normal account password with:

```text
HTTP Basic: Access denied
```

use a GitEx/GitLab Personal Access Token as the HTTPS password. The token needs at least `read_repository`; add `write_repository` if the Pi must push changes.

SSH is also possible if the network permits outbound port 22:

```bash
git clone git@gitex.itq.de:cirqmind/PlastiX.git
```

On the ITQ guest network, SSH port 22 may time out. In that case, use HTTPS + token.

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

This can take a long time on the Pi because PyTorch and related ARM64 wheels are large.

If pip reports:

```text
Temporary failure in name resolution
Failed to resolve 'pypi.org'
```

fix the Pi's network/DNS connection before retrying. This is not an Ultralytics error.

Verify:

```bash
python -c "from ultralytics import YOLO; import cv2; print('YOLO and OpenCV ready')"
```

Expected:

```text
YOLO and OpenCV ready
```

Deactivate the environment before building ROS packages:

```bash
deactivate
```

# 9. Copy the trained model to the Pi

The trained model is not stored in Git.

The model file on the current Pi is:

```text
/home/itq/sharx_yolo_pi/best.pt
```

From another computer on the same network:

```bash
scp \
  /path/to/best.pt \
  itq@<PI_IP_ADDRESS>:~/sharx_yolo_pi/best.pt
```

Example:

```bash
scp \
  ~/sharx_yolo_project/runs/detect/runs/floating_waste_v1/weights/best.pt \
  itq@192.168.0.105:~/sharx_yolo_pi/best.pt
```

Verify on the Pi:

```bash
ls -lh ~/sharx_yolo_pi/best.pt
```

Verify that Ultralytics can load it:

```bash
cd ~/sharx_yolo_pi
source .venv/bin/activate

python -c "from ultralytics import YOLO; YOLO('best.pt'); print('Model loaded successfully')"

deactivate
```

# 10. Set up and verify Raspberry Pi Camera Module 3

The final SharX setup uses a Raspberry Pi Camera Module 3 (`IMX708`), not a Raspberry Pi Camera Module 3.

## 10.1 Check kernel detection

Check video/media devices:

```bash
v4l2-ctl --list-devices
```

Check for the IMX708 sensor:

```bash
sudo dmesg | grep -i imx708
```

A successful detection contains lines similar to:

```text
imx708 ... camera module ID 0x0301
rp1-cfe ... Using sensor imx708 for capture
```

The Pi 5 also exposes many PiSP video devices. Their presence alone does not prove that the camera sensor is detected.

## 10.2 Camera boot configuration

Check:

```bash
grep -Ei "camera|imx|overlay" /boot/firmware/config.txt
ls /boot/firmware/overlays/ | grep imx708
```

The system should contain:

```text
camera_auto_detect=1
imx708.dtbo
```

On this Ubuntu 24.04 Pi 5 setup, automatic detection initially did not bind the sensor correctly. The working configuration used an explicit IMX708 overlay.

Edit:

```bash
sudo nano /boot/firmware/config.txt
```

For the camera connector in use, add the appropriate explicit overlay. The tested setup used:

```text
camera_auto_detect=0
dtoverlay=imx708
```

If the camera is on the connector that requires the `cam0` selector, use:

```text
dtoverlay=imx708,cam0
```

Power off before changing the physical ribbon cable:

```bash
sudo poweroff
```

After reboot, verify again:

```bash
sudo dmesg | grep -i imx708
v4l2-ctl --list-devices
```

The working camera interface should include an `rp1-cfe` device and `/dev/video0` through related capture nodes.

## 10.3 Build Raspberry Pi libcamera with PiSP support on Ubuntu 24.04

Ubuntu's stock `cam -l` may show no cameras and an IPA warning. The tested solution is to build the Raspberry Pi libcamera fork with the Pi 5 `rpi/pisp` pipeline.

Install build dependencies:

```bash
sudo apt update

sudo apt install -y \
  git meson ninja-build pkg-config g++ \
  libyaml-dev python3-yaml python3-ply python3-jinja2 \
  libgnutls28-dev openssl \
  libboost-dev \
  libtiff-dev pybind11-dev \
  libevent-dev libdrm-dev libjpeg-dev
```

Clone and configure:

```bash
cd ~
git clone https://github.com/raspberrypi/libcamera.git
cd ~/libcamera
```

```bash
meson setup build \
  --prefix=/usr/local \
  --buildtype=release \
  -Dpipelines=rpi/pisp \
  -Dipas=rpi/pisp \
  -Dv4l2=true \
  -Dtest=false \
  -Dlc-compliance=disabled
```

Build and install:

```bash
ninja -C build -j$(nproc)
sudo ninja -C build install
sudo ldconfig
```

The successful installation should include the PiSP IPA and IMX708 tuning files under `/usr/local`, including:

```text
/usr/local/lib/aarch64-linux-gnu/libcamera/ipa/ipa_rpi_pisp.so
/usr/local/share/libcamera/ipa/rpi/pisp/imx708.json
```

## 10.4 Build rpicam-apps

Clone:

```bash
cd ~
git clone https://github.com/raspberrypi/rpicam-apps.git
cd ~/rpicam-apps
```

Install dependencies:

```bash
sudo apt install -y \
  meson ninja-build \
  libboost-program-options-dev \
  libdrm-dev \
  libexif-dev \
  libjpeg-dev \
  libtiff-dev \
  libpng-dev
```

Make sure the build sees the `/usr/local` libcamera:

```bash
export PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
```

Verify:

```bash
pkg-config --modversion libcamera
```

The tested build reported:

```text
0.7.2
```

Configure and build `rpicam-apps`:

```bash
meson setup build --buildtype=release
meson compile -C build -j$(nproc)
sudo meson install -C build
sudo ldconfig
```

Verify:

```bash
which rpicam-hello
rpicam-hello --list-cameras
```

Expected camera:

```text
0 : imx708 [4608x2592 10-bit RGGB]
```

## 10.5 Fix DMA heap permissions

If `rpicam-hello` reports:

```text
Could not open any dmaHeap device
dmaHeap allocation failure
```

check:

```bash
ls -l /dev/dma_heap/*
```

On the tested Pi the devices are owned by `root:video`.

Add the current user to the `video` group:

```bash
sudo usermod -aG video "$USER"
sudo reboot
```

After reboot, camera capture should work without `sudo`.

## 10.6 Capture a test image

```bash
rpicam-still -o ~/camera_test.jpg
```

A successful run ends with:

```text
Still capture image received
```

Verify:

```bash
ls -lh ~/camera_test.jpg
xdg-open ~/camera_test.jpg
```

`Failed to create drm preview` or `Preview window unavailable` can appear even when capture itself is working.

# 11. Test YOLO directly on the Pi

Activate the YOLO environment:

```bash
cd ~/sharx_yolo_pi
source .venv/bin/activate
```

Verify the model:

```bash
python -c "from ultralytics import YOLO; YOLO('best.pt'); print('Model loaded successfully')"
```

The Camera Module 3 should not be treated as a normal Raspberry Pi Camera Module 3 with:

```text
cv2.VideoCapture(0)
```

The working camera path is:

```text
IMX708
  -> rpicam-vid / libcamera
  -> MJPEG frames
  -> OpenCV
  -> Ultralytics YOLO
```

A standalone `camera_yolo.py` test can launch `rpicam-vid` and read MJPEG frames through stdout before running `YOLO('best.pt')`.

For the production ROS pipeline, this same camera path is implemented directly inside `sharx_vision/waste_detector`, so the standalone script is only a diagnostic test.

Deactivate before building ROS:

```bash
deactivate
```

# 12. Build the ROS 2 workspace

**Important:** do not build the ROS workspace while the YOLO virtual environment is active.

The ROS CMake build must use:

```text
/usr/bin/python3
```

not:

```text
~/sharx_yolo_pi/.venv/bin/python3
```

Prepare a clean build shell:

```bash
deactivate 2>/dev/null || true
unset PYTHONPATH

source /opt/ros/jazzy/setup.bash
which python3
```

Expected:

```text
/usr/bin/python3
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

If the build fails with:

```text
ModuleNotFoundError: No module named 'catkin_pkg'
```

and the error points to `~/sharx_yolo_pi/.venv/bin/python3`, the virtual environment is still influencing the build. Deactivate it, clear the failed package build directories, and rebuild with `/usr/bin/python3`.

# 13. Allow ROS 2 Python nodes to access YOLO packages at runtime

The generated ROS 2 Python executable uses:

```text
/usr/bin/python3
```

Ultralytics, OpenCV and PyTorch are installed inside:

```text
~/sharx_yolo_pi/.venv
```

Do **not** activate the virtual environment for `colcon build`.

At runtime, expose only its site-packages directory:

```bash
export PYTHONPATH="$HOME/sharx_yolo_pi/.venv/lib/python3.12/site-packages:$PYTHONPATH"
```

Verify:

```bash
/usr/bin/python3 -c "import ultralytics, cv2, torch, numpy; print('ROS + YOLO ready')"
```

Expected:

```text
ROS + YOLO ready
```

This export is required before running `waste_detector` with the current package layout.

# 14. Recommended runtime environment script

Create:

```bash
nano ~/sharx_setup.sh
```

Paste:

```bash
#!/bin/bash

# ROS 2
source /opt/ros/jazzy/setup.bash

# SharX workspace
if [ -f "$HOME/plastix_sharx_ws/install/setup.bash" ]; then
    source "$HOME/plastix_sharx_ws/install/setup.bash"
fi

# ROS network configuration
export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY

# Allow ROS system Python to import YOLO dependencies
export PYTHONPATH="$HOME/sharx_yolo_pi/.venv/lib/python3.12/site-packages:$PYTHONPATH"

echo "SharX environment loaded"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
```

Save and make it executable:

```bash
chmod +x ~/sharx_setup.sh
```

Use it in each **runtime** terminal:

```bash
source ~/sharx_setup.sh
```

Do **not** automatically activate the YOLO `.venv` inside this script. Doing so can cause `colcon` to use the wrong Python interpreter.

For builds, use a clean shell:

```bash
deactivate 2>/dev/null || true
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
```

It is better not to add `sharx_setup.sh` to `.bashrc` while the system is still under active development.

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

The detector now uses the Raspberry Pi Camera Module 3 through `rpicam-vid`. It no longer uses `camera_index` or `cv2.VideoCapture(0)`.

Load the environment:

```bash
source ~/sharx_setup.sh
```

Verify Python access:

```bash
/usr/bin/python3 -c "import ultralytics, cv2, torch, numpy; print('ROS + YOLO ready')"
```

Run the tested detector configuration:

```bash
ros2 run sharx_vision waste_detector \
  --ros-args \
  -p model_path:=/home/itq/sharx_yolo_pi/best.pt \
  -p image_size:=320 \
  -p device:=cpu \
  -p show_image:=false \
  -p camera_width:=640 \
  -p camera_height:=480 \
  -p camera_fps:=30
```

`show_image:=false` is recommended for the reliable headless/runtime test. `show_image:=true` can be used on the desktop, but OpenCV/Qt may print font or Wayland warnings.

Detection topic:

```text
/sharx/waste_detection
```

Check it in another terminal:

```bash
source ~/sharx_setup.sh
ros2 topic echo /sharx/waste_detection
```

Example detection:

```json
{
  "detected": true,
  "class_name": "floating_waste",
  "confidence": 0.293,
  "center_x": 326,
  "center_y": 290,
  "box_width": 29,
  "box_height": 0,
  "image_width": 640,
  "image_height": 480
}
```

The exact box values depend on the observed target. A stream alternating between `detected: true` and `detected: false` is normal when the target is near the confidence threshold.

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

Load:

```bash
source ~/sharx_setup.sh
```

For autonomous mode:

```bash
ros2 run sharx_communication movement_status \
  --ros-args \
  -p mode:=autonomous
```

Inspect:

```bash
ros2 topic echo /sharx/status
```

Expected moving status:

```json
{
  "device_id": "sharx_1",
  "left_thruster": -0.22,
  "mode": "autonomous",
  "right_thruster": 0.22,
  "status": "moving"
}
```

Forward motion can appear as approximately:

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

Verify:

```bash
ros2 param get /movement_status mode
```

Expected:

```text
String value is: autonomous
```

## Required `movement_status.cpp` fix

The original source hardcoded:

```cpp
{"mode", "teleop"},
```

so the ROS parameter had no effect.

The node must declare and store the parameter:

```cpp
mode_ = declare_parameter<std::string>(
    "mode",
    "teleop"
);
```

Use it in the JSON:

```cpp
{"mode", mode_},
```

and declare a class member:

```cpp
std::string mode_;
```

After changing the source, rebuild cleanly:

```bash
cd ~/plastix_sharx_ws

deactivate 2>/dev/null || true
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select sharx_communication \
  --symlink-install
```

Then reload:

```bash
source ~/sharx_setup.sh
```

# 21. Full autonomous software test

Use separate terminals. The Pi-only software pipeline has been validated with this sequence.

## Terminal 1: detector

```bash
source ~/sharx_setup.sh

ros2 run sharx_vision waste_detector \
  --ros-args \
  -p model_path:=/home/itq/sharx_yolo_pi/best.pt \
  -p image_size:=320 \
  -p device:=cpu \
  -p show_image:=false \
  -p camera_width:=640 \
  -p camera_height:=480 \
  -p camera_fps:=30
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
```

Check each stage as needed:

```bash
ros2 topic echo /sharx/waste_detection
```

```bash
ros2 topic echo /sharx/cmd_vel_auto
```

```bash
ros2 topic echo /sharx/thruster_command
```

```bash
ros2 topic echo /sharx/status
```

Validated behaviour:

```text
Target centered
  -> /sharx/cmd_vel_auto linear.x ~= 0.18
  -> thrusters ~= [0.18, 0.18]

Target off-center
  -> angular.z ~= +/-0.22
  -> thrusters ~= [-0.22, 0.22] or [0.22, -0.22]

No valid detection / stop condition
  -> thrusters [0.0, 0.0]

movement_status
  -> mode = autonomous
  -> status = moving/stopped
```

Full pipeline:

```text
Raspberry Pi Camera Module 3 (IMX708)
    |
    v
rpicam-vid / libcamera / PiSP
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
Octopus / downstream control
```

At this stage, the complete **Pi-only autonomous software pipeline is validated**. Network integration with the other computer/Octopus should be tested next.

# 22. Update the Pi from Git

Before pulling updates:

```bash
cd ~/PlastiX
git status
```

Pull:

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

Rebuild in a **clean build environment**:

```bash
deactivate 2>/dev/null || true
unset PYTHONPATH

source /opt/ros/jazzy/setup.bash
cd ~/plastix_sharx_ws

colcon build \
  --packages-select sharx_communication sharx_vision \
  --symlink-install
```

Reload for runtime:

```bash
source ~/sharx_setup.sh
```

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

## `catkin_pkg` missing during `colcon build`

Error:

```text
ModuleNotFoundError: No module named 'catkin_pkg'
```

If CMake reports that it is using:

```text
/home/itq/sharx_yolo_pi/.venv/bin/python3
```

the YOLO environment is incorrectly active during the ROS build.

Fix:

```bash
deactivate 2>/dev/null || true
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash

which python3
```

Expected:

```text
/usr/bin/python3
```

Then rebuild.

---

## Ultralytics module not found at ROS runtime

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
/usr/bin/python3 -c "import ultralytics, cv2, torch, numpy"
```

---

## Camera Module 3 is not detected

Check:

```bash
sudo dmesg | grep -i imx708
v4l2-ctl --list-devices
```

If there is no IMX708 entry, check the ribbon cable with the Pi powered off and verify `/boot/firmware/config.txt`.

The working kernel log contains:

```text
camera module ID 0x0301
Using sensor imx708 for capture
```

---

## `rpicam-hello` command not found

On this Ubuntu 24.04 setup, Raspberry Pi `libcamera` and `rpicam-apps` were built from source. Follow Section 10.

---

## DMA heap allocation failure

Error:

```text
Could not open any dmaHeap device
dmaHeap allocation failure
```

Check:

```bash
ls -l /dev/dma_heap/*
```

If the devices are owned by `root:video`, add the user:

```bash
sudo usermod -aG video "$USER"
sudo reboot
```

---

## `Failed to create drm preview`

This can occur even when the camera is streaming successfully.

Test capture directly:

```bash
rpicam-still -o ~/camera_test.jpg
```

If the command ends with:

```text
Still capture image received
```

the camera capture path is working.

---

## No detection topic

First make sure `waste_detector` is still running.

Check:

```bash
ros2 node list
```

Expected:

```text
/waste_detector
```

Then:

```bash
ros2 topic list -t | grep sharx
```

and:

```bash
ros2 topic echo /sharx/waste_detection
```

If OpenCV/Qt display problems occur, run the detector with:

```text
-p show_image:=false
```

---

## `/sharx/cmd_vel_auto` does not exist

The topic is published by `waste_follower`. Start the follower before echoing the topic:

```bash
ros2 run sharx_vision waste_follower \
  --ros-args \
  -p stop_area_ratio:=0.45
```

---

## Thruster output is always zero

Check the pipeline in order:

```bash
ros2 topic echo /sharx/waste_detection
```

```bash
ros2 topic echo /sharx/cmd_vel_auto
```

```bash
ros2 topic echo /sharx/thruster_command
```

The follower intentionally stops when:

* no object is detected
* the object is too close
* image dimensions are invalid
* the detection stream stops

---

## Status still shows `teleop` in autonomous mode

If this command:

```bash
ros2 run sharx_communication movement_status \
  --ros-args \
  -p mode:=autonomous
```

still publishes:

```text
"mode":"teleop"
```

inspect:

```bash
grep -Rni "teleop\|declare_parameter\|mode" \
  ~/plastix_sharx_ws/src/sharx_communication
```

The source must not hardcode:

```cpp
{"mode", "teleop"},
```

It must publish the `mode_` parameter as described in Section 20.

---

## Laptop and Pi cannot discover each other

On both systems:

```bash
export ROS_DOMAIN_ID=10
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
```

Check IPs:

```bash
hostname -I
```

Test:

```bash
ping <OTHER_DEVICE_IP>
```

Both machines must be on a network that allows peer-to-peer traffic.

# 25. Current limitations

* ESC and physical thrusters are not yet connected.
* `/sharx/thruster_command` contains normalized values only.
* ESP32 micro-ROS communication is not yet integrated.
* GPS and IMU navigation are not yet implemented.
* The current stop distance is estimated from bounding-box area.
* The trained YOLO model is stored outside Git.
* The Python environment currently requires a runtime `PYTHONPATH` export.
* The Pi Camera Module 3 userspace stack is installed from `/usr/local` and should be documented when updating Ubuntu/libcamera.
* Physical emergency-stop behaviour must be implemented and tested before water operation.

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

The Pi-only perception and autonomous command pipeline is now validated.

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

````