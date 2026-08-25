# pi_env — Raspberry Pi environment configuration (snapshot)

This folder is **not a colcon workspace**, but a pure reference collection
of the Pi environment outside of the ROS workspace (the source packages
live under `../ros2/src/`). Goal: after a fresh Pi setup, these files can be
used to check whether everything is running as before.

**The snapshot is not uniform, and the date below is per-file, not folder-wide.**
The original collection was taken 2026-07-06, but several parts have been re-taken
since and carry their own dates in their own contents — `udev/99-gripperx.rules`
(entries dated 2026-08-13 and 2026-08-17), `bashrc/bashrc_ros_relevant.txt`
(comment dating the RMW change to 2026-08-13) and `systemd/` (the
`gripperx-mapping` unit and script carry 2026-08-24 headers). The `dpkg/`, `os/`
and `network/` files have **not** been re-taken and are TO-VERIFY against the
machine. Read each file's own dates rather than trusting a folder-wide "snapshot
as of". Corrected 2026-08-24.

## Structure

- `systemd/units/` — content of all `gripperx-*` systemd units (via
  `systemctl cat`, so any drop-ins would also be captured).
  Start order via `After=`/`Requires=`:
  `gripperx-bringup` → (20s delay) `gripperx-mapping` → (15s delay)
  `gripperx-navigation`. `gripperx-agent` is independent (Docker).
  `gripperx-wifi.timer` triggers `gripperx-wifi.service` every **30 s**
  (`OnBootSec=45`, `OnUnitInactiveSec=30`). Corrected 2026-08-24 against
  `systemd/units/gripperx-wifi.timer` — the 15 s figure was the pre-Fix-8 value
  that Fix 8 (#12) raised precisely because 15 s made the timer flap.
- `systemd/scripts/` — the scripts referenced by the units. **These are not a
  backup, they are the source. See "A `git pull` does not update what systemd
  runs" below before deploying any change to them.**
- `udev/` — content of `/etc/udev/rules.d/99-gripperx.rules` (ESP32,
  steering servo adapter, arm servo board — fixed symlinks via USB
  vendor/product/serial) and `99-lidar.rules` (LiDAR on `ttyAMA0`).
- `dpkg/` — package lists: `ros-jazzy-packages.txt` (409 lines, `dpkg -l
  'ros-jazzy-*'`), `other-relevant-packages.txt` (Gazebo/Nav2/slam_toolbox/
  micro-ros/platformio hits from `dpkg -l`), `pip-packages.txt` (`pip3
  list --format=freeze`, 235 packages), `apt-ros2-source.txt` (content of
  `/etc/apt/sources.list.d/ros2.list`).
- `bashrc/bashrc_ros_relevant.txt` — ROS-relevant lines from `~/.bashrc`:
  `source /opt/ros/jazzy/setup.bash`, `ROS_DOMAIN_ID=20`,
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
  `FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml`, `source
  ~/microros_ws/install/setup.bash`. This file is a **snapshot of what is on the
  Pi**, not a target state — it is deliberately left at the recorded values.

  **CLOSED (corrected 2026-08-24).** This section used to carry an "ACTION
  REQUIRED" notice saying the shell recorded `ROS_DOMAIN_ID=42` and CycloneDDS
  while the services ran 20/FastRTPS, and that the snapshot still had to be
  re-taken. **The snapshot in this folder has since been re-taken and shows
  20 / FastRTPS / the UDP-only profile**, and its own inline comment dates the
  mixed-RMW state as "the state until 2026-08-13". The prose here had simply not
  been updated with the file it describes. Verified 2026-08-24 by reading
  `bashrc/bashrc_ros_relevant.txt` itself, not this description.

  Note what this does and does not prove: the snapshot is the record of the Pi at
  the time it was taken, so the domain/RMW mismatch is closed **as of that
  snapshot**. It is not a live reading. No Pi contact was made by this correction.

  Why +200 and not +100: ROS 2 derives the DDS base port as `7400 + 250*domain_id`.
  With Linux's default ephemeral range `32768-60999`, ids 102-214 land inside it and
  can lose discovery to whatever grabbed the port first — intermittent and hard to
  reproduce. Safe bands are 0-101 and 215-232, so 20 and 220 are both clean and the
  gripper range 20-29 maps onto 220-229 without leaving them. Check the local range
  with `sysctl net.ipv4.ip_local_port_range`.

  The RMW split (interactive CycloneDDS vs. service FastRTPS) is **closed as of the
  current snapshot** — the recorded shell now exports `rmw_fastrtps_cpp` and the
  UDP-only profile, i.e. interactive diagnosis runs on the same transport as the
  services. The reason it mattered is worth keeping: `fastdds_udp_only.xml` is
  FastRTPS-specific and is silently ignored by a CycloneDDS shell, so under the old
  split the SHM hardening did not apply to interactive sessions at all (OP-8).
- `network/network_overview.txt` — `ip -br addr`, hostname (mDNS via
  `avahi-daemon`, `GripperX-1.local`), note on `/etc/netplan/` (only
  `50-cloud-init.yaml` present, content **deliberately not backed up** —
  could contain WiFi credentials, see section "Not backed up").
- `os/os_hardware_info.txt` — `/etc/os-release` (Ubuntu 24.04.4 LTS "Noble"),
  `uname -a` (kernel 6.8.0-1057-raspi, aarch64), board model
  ("Raspberry Pi 5 Model B Rev 1.1"), CPU info excerpt.

## A `git pull` does NOT update what systemd runs

**Every unit runs `ExecStart=/usr/local/bin/gripperx-*.sh`. The repository keeps
those scripts in `systemd/scripts/`. Nothing links the two.** Pulling the
repository on the Pi updates `~/ws/Software/pi_env/systemd/scripts/`, which no
unit ever reads. The running script is whatever was last copied into
`/usr/local/bin/`, and it changes only when somebody copies it there.

This is not a corner case. Verified 2026-08-24 by reading all five service units
in `systemd/units/` (the sixth file is `gripperx-wifi.timer`, which has no
`ExecStart`): every `ExecStart` points into `/usr/local/bin/`, and no unit, script
or document in this repository contains a copy step.

**Why it bites right now.** `gripperx-mapping.sh` was repointed on 2026-08-24
from `gripperx_bringup/mapping.launch.py` to
`gripperx_localization/localization.launch.py` — the change that makes the EKF
start and gives Nav2 a publisher on `/odometry/filtered`. A `git pull` on the Pi
brings that change into the working tree and **not** into the running service.
The failure is silent in the worst way: the service still starts, still reports
active, and Nav2 still comes up green with no velocity feedback.

**The completion check in `documentation/DEPLOYMENT.md` does not catch this.** That
document defines a finished deploy as "`git status` on the Pi should print
nothing". A Pi whose working tree is spotless and whose `/usr/local/bin` is weeks
stale satisfies that check exactly. Clean tree, wrong robot.

**After changing anything under `systemd/`, on the Pi:**

```bash
# scripts — the units read only these copies
sudo install -m 0755 -o root -g root \
  ~/ws/Software/pi_env/systemd/scripts/gripperx-*.sh /usr/local/bin/

# units — only if a .service/.timer changed
sudo install -m 0644 -o root -g root \
  ~/ws/Software/pi_env/systemd/units/gripperx-* /etc/systemd/system/
sudo systemctl daemon-reload
```

Then confirm the copy actually landed, rather than assuming it did:

```bash
diff -u /usr/local/bin/gripperx-mapping.sh \
        ~/ws/Software/pi_env/systemd/scripts/gripperx-mapping.sh   # must be empty
```

Restarting the affected services is a separate step and is governed by
`documentation/DEPLOYMENT.md` §1 (clean teardown) and by SR-1 — a bringup restart is
a movement event and needs user approval.

**TO-VERIFY:** the `install` invocations above were written from the unit files
and the README's own statement that the live scripts are root-owned; **they have
not been executed or checked against the Pi** (no Pi contact was made, 2026-08-24).
Confirm ownership and mode against `ls -l /usr/local/bin/gripperx-*.sh` before
relying on the flags.

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

- **Build on the Pi with plain `colcon build`, never `--symlink-install`** — see
  the rule at the top of `documentation/DEPLOYMENT.md`; on the Pi the attempt fails
  *after* removing package metadata and leaves the drive stack in a restart loop.
  The note that used to stand here — "`gripperx_arm` is not symlink-installed, so
  after every change to `arm_action_server.py` two install paths must be updated" —
  read as if the other packages were symlink-installed. On the Pi none of them
  should be. Clarified 2026-08-24.
- **Changing a `gripperx-*.sh` is not deployed by `git pull`** — see the section
  above. This is the pitfall most likely to cost a session.
- Strictly observe the service start order (bringup → mapping →
  navigation), otherwise the `ExecStartPre=sleep` wait times run for nothing.
- **RETRACTED 2026-08-24 — this entry was wrong and said the opposite of the
  truth.** It used to read: "`gripperx-navigation.sh` sets the teleop mode to
  `autonomous` at startup via `ros2 topic pub` — the robot can start moving
  autonomously as soon as this service comes up." That block was **removed
  outright** by Fix 8 (#12); `systemd/scripts/gripperx-navigation.sh` contains
  no `ros2 topic pub /teleop/set_mode` today and its header documents the
  removal and why. What actually happens: `teleop_mux` starts at
  `initial_mode=keyboard` (`gripperx_teleop/config/teleop_mux.yaml`) and stays
  there across a boot or a restart of this service. Reaching `autonomous`
  requires a deliberate operator action (`G` in `keyboard_teleop_node`, or a
  manual publish). Kept as a retraction rather than deleted: a reader who
  remembers this warning needs to see that it was withdrawn, not just find it
  missing.
- DDS zombies: `docker stop mros_agent` is needed when `gripperx-agent.service`
  is stopped (the container survives `systemctl stop`).
