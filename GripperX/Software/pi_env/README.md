# pi_env — Raspberry Pi environment configuration (snapshot)

This folder is **not a colcon workspace**, but a pure reference collection
of the Pi environment outside of the ROS workspace (the source packages
live under `../ros2/src/`). Snapshot as of: 2026-07-06. Goal: after a fresh
Pi setup, these files can be used to check whether everything is running
as before.

## Structure

- `systemd/units/` — content of all `gripperx-*` systemd units (via
  `systemctl cat`, so any drop-ins would also be captured).
  Start order via `After=`/`Requires=`:
  `gripperx-bringup` → (20s delay) `gripperx-mapping` → (15s delay)
  `gripperx-navigation`. `gripperx-agent` is independent (Docker).
  `gripperx-wifi.timer` triggers `gripperx-wifi.service` every 15s.
- `systemd/scripts/` — the scripts referenced by the units, from
  `/usr/local/bin/gripperx-*.sh` (root-owned, not in the ROS workspace, hence
  backed up separately here instead of in the src mirror).
- `udev/` — content of `/etc/udev/rules.d/99-gripperx.rules` (ESP32,
  steering servo adapter, arm servo board — fixed symlinks via USB
  vendor/product/serial) and `99-lidar.rules` (LiDAR on `ttyAMA0`).
- `dpkg/` — package lists: `ros-jazzy-packages.txt` (409 lines, `dpkg -l
  'ros-jazzy-*'`), `other-relevant-packages.txt` (Gazebo/Nav2/slam_toolbox/
  micro-ros/platformio hits from `dpkg -l`), `pip-packages.txt` (`pip3
  list --format=freeze`, 235 packages), `apt-ros2-source.txt` (content of
  `/etc/apt/sources.list.d/ros2.list`).
- `bashrc/bashrc_ros_relevant.txt` — ROS-relevant lines from `~/.bashrc`:
  `source /opt/ros/jazzy/setup.bash`, `ROS_DOMAIN_ID=42`,
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `source
  ~/microros_ws/install/setup.bash`.
  **Important:** this only applies to interactive shells. All
  `gripperx-*.service` scripts explicitly override this with
  `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (see
  `systemd/scripts/`). This appears to be intentional (runtime domain
  0/FastRTPS, interactive shell defaults 42/CycloneDDS) — during a
  fresh setup, don't be confused by the bashrc values, the services
  are authoritative.
- `network/network_overview.txt` — `ip -br addr`, hostname (mDNS via
  `avahi-daemon`, `GripperX-1.local`), note on `/etc/netplan/` (only
  `50-cloud-init.yaml` present, content **deliberately not backed up** —
  could contain WiFi credentials, see section "Not backed up").
- `os/os_hardware_info.txt` — `/etc/os-release` (Ubuntu 24.04.4 LTS "Noble"),
  `uname -a` (kernel 6.8.0-1057-raspi, aarch64), board model
  ("Raspberry Pi 5 Model B Rev 1.1"), CPU info excerpt.

## Not backed up (deliberately, or not accessible via file)

- **WiFi credentials** (`/etc/netplan/50-cloud-init.yaml`,
  `wpa_supplicant` configuration): reading was classified as a credential
  risk and was not done. During a fresh setup, WiFi/hotspot pairing
  (iPhone hotspot `172.20.10.2`, LAN fallback `10.42.0.70`) must be
  manually reconfigured, including static ethernet IP setup (`nmcli`/netplan)
  for a direct Pi-laptop cable link if needed.
- **`~/microros_ws`** (separate, non-colcon Pi workspace, ~151 MB
  source code, of which 15 MB is `src/` with the official
  `micro_ros_setup` repo + `Micro-XRCE-DDS-Agent`/`micro-ROS-Agent` submodules):
  **not mirrored**. Reason: it is standard upstream tooling
  (github.com/micro-ROS/micro_ros_setup), reproducible via the official
  micro-ROS docs, and appears unused at runtime — the
  actually running agent (`gripperx-agent.service`) directly starts the
  public Docker image `microros/micro-ros-agent:jazzy` and does not reference
  `~/microros_ws`. `.bashrc` still sources it interactively; whether it is
  still needed for something (e.g. rebuilding a custom agent) is unclear —
  verify before a Pi fresh setup if the Docker variant turns out not to
  suffice.
- **Docker image** `microros/micro-ros-agent:jazzy` (755 MB, Docker Hub,
  public) — will be pulled again via `docker pull` on a fresh setup, no
  reason for a local backup. Docker version at snapshot time:
  29.1.3 (`docker.io` package, not docker-ce).
  colcon: `python3-colcon-common-extensions` 0.3.0-100 (incl. `colcon-core`
  0.20.1), `rosdep` 0.26.0-1 — both installed as apt packages, not included in
  `dpkg/other-relevant-packages.txt`, see if needed again
  `dpkg -l 'python3-colcon-*' 'python3-rosdep*'`.
- **USB device serial numbers** are indirectly contained in `udev/99-gripperx.rules`
  (idVendor/idProduct/serial) — if a servo adapter/ESP32 is
  swapped, the serial number changes and the rule must be adjusted
  (determine new serial number via `udevadm info`).
- **SSH host keys, user password/sudo configuration**: not backed up
  (security-relevant, regenerated/reassigned anyway on a fresh setup).

## Known pitfalls on fresh setup

- `gripperx_arm` is **not** symlink-installed — after every change to
  `arm_action_server.py`, two install paths must be updated.
- Strictly observe the service start order (bringup → mapping →
  navigation), otherwise the `ExecStartPre=sleep` wait times run for nothing.
- `gripperx-navigation.sh` sets the teleop mode to `autonomous` at startup via
  `ros2 topic pub` — this is movement-relevant: the robot can start moving
  autonomously as soon as this service comes up.
- DDS zombies: `docker stop mros_agent` is needed when `gripperx-agent.service`
  is stopped (the container survives `systemctl stop`).
