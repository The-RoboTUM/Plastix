# VERIFY — checks after provisioning, BEFORE the first stack start

**What:** the checklist that decides whether a freshly provisioned card may run the stack.
**For:** whoever swaps the card. **Status:** written 2026-09-30; expected values are the reference
card's, read over read-only SSH that day. **Runbook position:** step E of [README.md](README.md) —
after `provision.sh` and **one reboot**, before `provision.sh --enable-units`.

None of these commands moves anything or starts a gripperx unit. The stack is **not running**
during these checks (its units are not enabled yet), so the `ros2` CLI calls below — which read the
package index only — cannot disturb a bringup. Run them on the Pi unless marked *laptop*.
Every row must match; anything else is a finding, not a formality.

## 1. Identity, access, time

| # | Command | Expected |
| --- | --- | --- |
| 1.1 | *laptop:* `ssh -o BatchMode=yes gripperx 'hostname; uname -r'` | `GripperX-1`, no host-key warning; kernel `6.8.0-1065-raspi` or newer (the reference card ran 1065) |
| 1.2 | `for t in ecdsa ed25519 rsa; do ssh-keygen -l -f /etc/ssh/ssh_host_${t}_key.pub; done` | the three fingerprints in `CARRY_OVER.md` A1 |
| 1.3 | `awk '{print $1, $NF}' ~/.ssh/authorized_keys` | 3 keys: 2 × `dev@GripperX`, 1 × `blaesse@itqlm104` |
| 1.4 | *laptop:* `date; ssh gripperx date` | within ~1 s |
| 1.5 | `timedatectl show -p NTPSynchronized -p Timezone; timedatectl show-timesync -p ServerName` | `yes`, `Europe/Berlin`, the laptop (LOCAL_ENV.md §2) |
| 1.6 | `sudo -n true && echo ok` | `ok` (the button daemon calls `sudo -n`) |
| 1.7 | `ip -br addr show wlan0` | the DHCP address and the rescue address from LOCAL_ENV.md §2 |
| 1.8 | `sudo grep -c 'PRIVATE KEY' /boot/firmware/user-data` | `0` (stripped by `provision.sh` step 0 — CARRY_OVER.md C2) |

## 2. Boot configuration and hardware access

| # | Command | Expected |
| --- | --- | --- |
| 2.1 | `diff /boot/firmware/config.txt ~/ws/Software/pi_env/provision/reference/boot/config.txt && echo same` | `same` |
| 2.2 | `diff /boot/firmware/cmdline.txt ~/ws/Software/pi_env/provision/reference/boot/cmdline.txt && echo same` | `same` |
| 2.3 | `grep -c 'console=serial0' /proc/cmdline` | `0` |
| 2.4 | `systemctl is-enabled serial-getty@ttyAMA0.service` | `masked` |
| 2.5 | `id -nG` | contains `dialout docker i2c spi gpio video plugdev` |
| 2.6 | `ls -l /dev/esp32 /dev/steering_servo /dev/arm_servo /dev/lidar` | four symlinks: `esp32`, `steering_servo`, `arm_servo` → `ttyACM*` (numbering may differ from the reference, which had 1/0/2), `lidar` → `ttyAMA0`. A missing `ttyACM` link = adapter unplugged or unpowered, or a replaced adapter with a new serial (then fix `udev/99-gripperx.rules` in git) |
| 2.7 | `ls -l /dev/ttyAMA0 /dev/gpiochip4 /dev/i2c-1 /dev/spidev0.0` | `root dialout 0660`, `root dialout 0660`, `root i2c 0660`, `root dialout 0660` |
| 2.8 | `python3 -c 'import gpiod; c=gpiod.Chip("/dev/gpiochip4", gpiod.Chip.OPEN_BY_PATH); print(c.label(), c.num_lines())'` | `pinctrl-rp1 54` (button GPIO17 and LiDAR power line live here) |
| 2.9 | `for f in 99-gripperx.rules 99-lidar.rules; do cmp ~/ws/Software/pi_env/udev/$f /etc/udev/rules.d/$f && echo "$f same"; done` | both `same` |

## 3. Software

| # | Command | Expected |
| --- | --- | --- |
| 3.1 | `cat /etc/apt/sources.list.d/*.list` | only the `snapshots.ros.org/jazzy/2026-06-18` line for ROS |
| 3.2 | `dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 'ros-jazzy-*' \| awk '$1=="ii"{print $2, $3}' \| diff - ~/ws/Software/pi_env/provision/reference/ros-jazzy-expected.txt && echo same` | `same` (363 packages = the reference's 405 minus the 42 MoveIt-family packages) |
| 3.3 | `ls /opt/ros/jazzy/share/ament_index/resource_index/packages \| wc -l` | `350` (reference 390, minus the 40 MoveIt-family entries it had) |
| 3.4 | `python3 -c 'import numpy, scservo_sdk; print(numpy.__version__, numpy.__file__); print(scservo_sdk.__file__)'` | `1.26.4 /usr/lib/python3/dist-packages/...` and `/home/ubuntu/.local/lib/python3.12/site-packages/scservo_sdk/...` |
| 3.5 | `ls ~/.local/lib/python3.12/site-packages/` | only `scservo_sdk` and `feetech_servo_sdk-1.0.0.dist-info` — above all no `numpy` |
| 3.6 | `docker image inspect microros/micro-ros-agent:jazzy --format '{{.Id}} {{json .RepoDigests}}'` (no sudo — proves the docker group) | contains `sha256:1d13bfebcd8ab6f8bbdd95ac64acf42a54eb97f2421ec559d871e7aa2c036f48` |
| 3.7 | `docker ps -a --format '{{.Names}}'` | no `mros_agent` — the agent has not run |
| 3.8 | `cmp ~/fastdds_udp_only.xml ~/ws/Software/pi_env/fastdds_udp_only.xml && echo same` | `same` |
| 3.9 | `bash -ic 'echo $ROS_DOMAIN_ID $RMW_IMPLEMENTATION $FASTRTPS_DEFAULT_PROFILES_FILE'` | `20 rmw_fastrtps_cpp /home/ubuntu/fastdds_udp_only.xml` |
| 3.10 | `systemctl is-enabled apt-daily.timer apt-daily-upgrade.timer; cat /etc/apt/apt.conf.d/99gripperx-hold-auto-upgrades` | `disabled` × 2 and the hold file (run made with `--hold-auto-upgrades`). Revert after the demo: `bash ~/ws/Software/pi_env/provision/provision.sh --release-auto-upgrades` |
| 3.11 | `~/.local/bin/pio --version` | `PlatformIO Core, version 6.1.19` |
| 3.12 | `~/.local/bin/pio pkg list -d ~/ws/Software/microros/firmware -e esp32-s3` | platform `espressif32 @ 7.0.1` with its toolchains, and library `micro_ros_platformio`. **List only — no `pio run`, upload or monitor** (firmware change = separate user approval) |
| 3.13 | `python3 -c 'import click; print(click.__file__)'` | a path under `/usr/lib/python3/dist-packages` — PlatformIO's `click` stays inside its venv |

## 4. Workspace and deploy state

| # | Command | Expected |
| --- | --- | --- |
| 4.1 | `git -C ~/ws status --porcelain; git -C ~/ws log --oneline -1; git -C ~/ws rev-parse --abbrev-ref @{u}` | empty; the commit that was pushed; `origin/Theo` (the deploy branch) |
| 4.2 | `git -C ~/repos/gripperx_ws.git branch` | `Theo`, `main`, `pi-history-2026-08-25` |
| 4.3 | `cd ~/ws/Software/ros2 && source /opt/ros/jazzy/setup.bash && colcon list --names-only \| wc -l` | `22` |
| 4.4 | `source ~/ws/Software/ros2/install/setup.bash && ros2 pkg list \| grep -c '^gripperx_'` | `18` — the same 18 as the reference card |
| 4.5 | `ros2 pkg list \| wc -l` (same shell) | `372` = 350 + 22 (reference: 390 + 22 = 412) |
| 4.6 | `ls -d ~/ws/Software/ros2/install/*/lib/python3.12/site-packages/*.egg-info \| wc -l` | `12` — as on the reference (arm, arm_msgs, control, control_msgs, debug, external, external_msgs, geometry, localization, planning, sensors, teleop). A missing `gripperx_control` egg-info is the `--symlink-install` failure of DEPLOYMENT.md |
| 4.7 | `tail -n 3 ~/provision_logs/colcon_build_*.log` | `Summary: 22 packages finished`, no `failed`/`aborted` |
| 4.8 | *laptop:* `tools/deploy_check.sh --ref Theo` | `in step: every installed copy matches the repository.` (18 files) |

## 5. Units — installed, not enabled

| # | Command | Expected |
| --- | --- | --- |
| 5.1 | `systemctl is-enabled gripperx-agent gripperx-bringup gripperx-mapping gripperx-navigation gripperx-external gripperx-button gripperx-wifi.timer gripperx.target` | `disabled` × 6, `enabled` (wifi timer), `disabled` (target — also disabled on the reference) |
| 5.2 | `systemctl list-units --all 'gripperx*' --no-legend` | every unit `inactive dead` except `gripperx-wifi.timer` (`active waiting`); none `failed` |
| 5.3 | `systemctl --failed --no-legend` | empty |
| 5.4 | `systemctl is-active docker avahi-daemon systemd-timesyncd` | `active` × 3 |

## 6. After the checks — the first stack start

Only when every row above matches:

1. `bash ~/ws/Software/pi_env/provision/provision.sh --enable-units` — enables the stack units, starts
   nothing.
2. The next boot **starts the stack**. That is a motion trigger under SR-1 (motor-enable process
   rule): it needs the user's explicit approval for that test, with the expected behaviour announced
   beforehand. Robot on blocks.
3. What the reference card logged on a clean start, and what to look for in
   `journalctl -u gripperx-bringup -b` (journal first — no `ros2` CLI during bringup, DEPLOYMENT.md
   §2.1): `swerve_controller activated. No command written on activation (SR-14 item 1)`,
   `Startup pose adopted from the servos, nothing commanded` and `Startup homing DISABLED
   (home_on_startup=false) — arm stays where it is`. Their absence is a stop signal.
