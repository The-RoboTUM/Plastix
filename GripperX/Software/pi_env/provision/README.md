# provision — rebuild the robot Pi on a fresh SD card

**What:** a script and three checklists that turn a freshly flashed Ubuntu 24.04.4 server (raspi,
arm64) card into a GripperX robot Pi that runs the stack exactly like the old card did.
**For:** whoever replaces the Pi's SD card (first use: swapping out the card that silently corrupts
files). **Status:** written 2026-09-30, revised the same day after review (cloud-init carry-over,
early ROS plan check); partially smoke-tested (steps 2, 3, 9, 12, 13 in an
Ubuntu 24.04 container; nothing run on a Pi yet). **Reference:** the old GripperX-1 card,
inventoried read-only over SSH on 2026-09-30.

| File | Purpose |
| --- | --- |
| `provision.sh` | the provisioning itself — idempotent, numbered steps, re-runnable after a failure (`--list`, `--from N`) |
| `CARRY_OVER.md` | the few files that must come from the old card (SSH host keys, authorized keys), and what deliberately does not |
| `VERIFY.md` | checks after provisioning, before the first stack start |
| `reference/boot/` | the old card's `config.txt` and `cmdline.txt` |
| `reference/ros-jazzy-expected.txt` | every `ros-jazzy-*` package with the exact version the robot must have (363) |
| `keys/ros-snapshots-archive-keyring.gpg` | public key of snapshots.ros.org (fingerprint `4B63 CF8F DE49 746E 98FA 01DD AD19 BAB3 CBF1 25EA`) |
| `requirements-pi-user.txt` | the only pip package (Feetech servo SDK), hash-pinned |

**Safety (SR-1 — motor-enable process rule):** the script never starts a gripperx unit and **does
not enable the stack units either**. Provisioning needs one reboot, and with the units enabled that
reboot would start the bringup — a motion trigger. Enabling is a separate, explicit call after
`VERIFY.md` (`provision.sh --enable-units`), and the boot after it needs the user's approval for that
test. The script also refuses to run while any gripperx unit is active.

---

## Runbook

`<ssh-alias>` and the laptop's hostname/address come from `LOCAL_ENV.md` §2 on the laptop. The
deploy branch is `Theo`; the user merges `Theo-provision` into `Theo` **before** step D, so the
pushed branch contains this directory. Run laptop commands from the workspace root.

**A. Stage from the old Pi while it still runs** — `CARRY_OVER.md` A: host keys and authorized keys
over SSH into a private laptop directory, integrity-checked; optionally A5 (`~/ws/runs`, agent
image, SDK). Then shut the robot down the usual way.

**B. Prepare the unbooted new card** — `CARRY_OVER.md` B: inject the host keys and the three
authorized keys into its cloud-init `user-data`.

**C. First boot** — card into the Pi, power on, wait for cloud-init (a few minutes). No gripperx
unit exists yet, so nothing starts. Then `CARRY_OVER.md` C1:

```bash
ssh -o BatchMode=yes <ssh-alias> 'hostname; date'        # no host-key warning = host keys taken over
```

**D. Deploy repo, clock, provisioning.** `.local` names do not resolve yet (avahi comes in step 6),
so push through the SSH alias:

```bash
ssh <ssh-alias> 'mkdir -p ~/repos && git init --bare ~/repos/gripperx_ws.git'
git push <ssh-alias>:repos/gripperx_ws.git Theo pi-history-2026-08-25 refs/remotes/pi/main:refs/heads/main
ssh <ssh-alias> 'git clone -b Theo ~/repos/gripperx_ws.git ~/ws'

# 1) clock first, interactively (short). Step 0 also strips the private host keys from
#    /boot/firmware/user-data (CARRY_OVER.md C2).
ssh <ssh-alias> 'bash ~/ws/Software/pi_env/provision/provision.sh --branch Theo \
    --ntp-servers "<laptop-hostname>.local <laptop-address>" --only 1'
```

2) the long run, detached so a WiFi drop cannot kill it. tmux may not be on the fresh image, so
use `setsid`/`nohup` and follow the output file:

```bash
ssh <ssh-alias> 'mkdir -p ~/provision_logs && nohup setsid bash ~/ws/Software/pi_env/provision/provision.sh \
    --branch Theo --ntp-servers "<laptop-hostname>.local <laptop-address>" --from 2 --hold-auto-upgrades \
    > ~/provision_logs/run.out 2>&1 < /dev/null &'
ssh <ssh-alias> 'tail -f ~/provision_logs/run.out'          # Ctrl-C ends only the tail; reconnect any time
```

(If `command -v tmux` succeeds on the Pi, `ssh -t <ssh-alias> "tmux new -s provision '...'"` works
equally.) The run ends with the summary block or with `##### FAILED` naming step and line; fix the
cause and start the same command with `--from <step>`.

`--hold-auto-upgrades` (user decision 2026-09-30: automatic upgrades OFF) stops and disables
`apt-daily.timer` and `apt-daily-upgrade.timer` and writes
`/etc/apt/apt.conf.d/99gripperx-hold-auto-upgrades`, so nothing changes packages on the robot
until it is released. **Revert** (e.g. after the demo):

```bash
ssh <ssh-alias> 'bash ~/ws/Software/pi_env/provision/provision.sh --release-auto-upgrades'
```

Without the flag the script only **waits** for running apt background jobs before each apt step.

**E. Reboot once and verify** — `sudo reboot`, then all of `VERIFY.md`, then `CARRY_OVER.md` D
(shred the staged keys). Nothing starts on this boot except the WiFi reconnect timer.

**F. Enable the stack (separate decision)** — `bash ~/ws/Software/pi_env/provision/provision.sh
--enable-units`, then the first stack start as an SR-1 test (`VERIFY.md` §6).

### Steps and runtimes

Runtimes are **estimates** (no step has run on a Pi yet), for the NAT'd WiFi link through the laptop.

| # | Step | Estimate |
| --- | --- | --- |
| 0 | preflight: user, sudo, arch, OS, board, hostname, no active gripperx unit, disk, DNS; strips private host keys from `/boot/firmware/user-data` | < 10 s |
| 1 | time: timesyncd sources, timezone `Europe/Berlin`, locale `en_US.UTF-8`, wait for NTP sync | 10 s – 2 min |
| 2 | deploy repo: bare repo, checkout `~/ws` on the deploy branch, clean + fast-forward check | < 30 s (the laptop push before it: 1–3 min, 87 MB) |
| 3 | boot: `config.txt` must equal the reference (fails otherwise, never overwrites); `cmdline.txt` must equal the reference apart from `console=serial0` (removed if present); `serial-getty@ttyAMA0` masked. On this card a no-op pass: both files already match | < 5 s |
| 4 | optional auto-upgrade hold; wait for apt background jobs; gnupg if missing; snapshot key (fingerprint checked), pinned ROS source, `apt-get update --error-on=any`; **dry run of the ROS install compared with the reference** — fails here, not after 20 min | 1–3 min (+ waiting for first-boot apt jobs) |
| 5 | `apt-get full-upgrade` (skip with `--skip-upgrade`) | 5–20 min |
| 6 | system packages (docker.io, avahi, colcon, libgpiod, …) | 3–8 min |
| 7 | ROS packages (reference packages outside the dependency closure added explicitly at their pinned version), then all 363 checked version by version | 10–25 min |
| 8 | Feetech SDK via pip `--user` (hash-pinned, no deps); numpy must resolve to apt | < 1 min |
| 9 | groups `i2c spi gpio` + membership `dialout docker i2c spi gpio video plugdev` | < 5 s |
| 10 | udev rules, reload, trigger | < 15 s |
| 11 | docker, agent image pulled by digest (or `--agent-image-tar`) | 2–5 min |
| 12 | `~/fastdds_udp_only.xml`, `.bashrc` block, git config | < 5 s |
| 13 | `/etc/gripperx/` | < 5 s |
| 14 | plain `colcon build --parallel-workers 2` of `~/ws/Software/ros2` (22 packages) + egg-info check | 15–30 min |
| 15 | install scripts to `/usr/local/bin`, units to `/etc/systemd/system`, `daemon-reload`, enable the WiFi timer only | < 10 s |
| 16 | PlatformIO 6.1.19 in its own venv `~/.platformio/penv` (links in `~/.local/bin`), prefetch of platform `espressif32@7.0.1` and the firmware project's packages (skip: `--skip-platformio`; prefetch failures only warn). **Never builds, uploads or monitors** | 5–15 min |
| 17 | summary and next steps | — |
| | **total** (without the laptop push and the reboot) | **~45–110 min** |

---

## Decisions taken, and why

**ROS 2 is pinned to the 2026-06-18 snapshot, not `packages.ros.org`.** The reference card's 405
`ros-jazzy-*` versions are all in `snapshots.ros.org/jazzy/2026-06-18` (checked: 0 missing), while
the live repository has moved on (e.g. `controller_manager` 4.45.2 → 4.48.0, `nav2` 1.3.12 → 1.3.13,
`rclpy` 7.1.11 → 7.1.12). For a swap that must behave like the old card on the day of a demo, the
exact versions win. Consequence: `apt upgrade` no longer moves ROS; bumping it later is a
deliberate change of the snapshot date here. `Software/pi_env/dpkg/apt-ros2-source.txt` records
the old source and is not used by the script.

**ROS package set = the reference card's, minus MoveIt.** The top-level packages are the ones the
reference card had installed manually; the only omission is `ros-jazzy-moveit`, which nothing in
`Software/ros2` uses and which alone accounts for 42 of the 405 packages. `gz-ros2-control` stays
(no extra cost — its Gazebo dependencies come with `nav2-bringup` anyway). Step 7 fails if any of
the remaining 363 differs in version.

**numpy: apt's 1.26.4 only — the ~/.local numpy 2.2.6 is not reinstalled.** Measured on the
reference card 2026-09-30 with the user site switched off (`python3 -s`, i.e. apt numpy 1.26.4):
`sensor_msgs`/`nav_msgs` messages including numpy-typed arrays (`Imu.orientation_covariance` is a
`numpy.ndarray`), and all three workspace message packages, import and work. No runtime code in
`Software/ros2` imports numpy directly — only tests and tools do (`check_*.py`, `plot_debug_csv.py`,
`decimate_meshes.py`). numpy 2.2.6 arrived with the LeRobot/torch stack, which is gone; the apt ROS
binaries were built against numpy 1.26, and a fresh build compiles the workspace's message
extensions against the same numpy, so the build and the runtime agree. Every running Python node on
the reference card did load numpy 2.2.6 (through the message modules), so the change is real, but
it is covered by the `-s` test above.

**Time: systemd-timesyncd, as on the reference card — not chrony.** The reference card has no
chrony installed; it syncs with timesyncd against the laptop (`NTP=` in `timesyncd.conf`). Step 1
recreates that as a drop-in. `Software/pi_env/chrony/10-gripperx-slew.conf` is therefore **not in
effect on the robot today** — see open question 3.

**Boot configuration.** The reference `config.txt` has the image's build date as mtime and enables
I2C, SPI and the UART (`dtparam=i2c_arm=on`, `dtparam=spi=on`, `enable_uart=1`). Step 3 only
checks that the new card's file is identical (verified by hand for this card) and fails otherwise. The one known change is in `cmdline.txt`:
`console=serial0,115200` was removed on 2026-07-02 (a backup with the stock line exists on the old
card) and `serial-getty@ttyAMA0` masked the same day, so the kernel console does not share the
LiDAR UART (`/dev/lidar` → `ttyAMA0`). Step 3 edits only that token and refuses any other
difference.

**PlatformIO (user decision 2026-09-30: install it).** The reference card had PlatformIO 6.1.19
as a `pip --user` install on the system python, which put `click` 8.4.2 into `~/.local` against
PlatformIO's own `click<8.4` pin. Step 16 installs the same version into its own venv at
`~/.platformio/penv` — the location `Software/microros/firmware/README.md` names — with a
compatible `click`, and links `pio`/`platformio` into `~/.local/bin`. The platform is pinned to
the reference card's `espressif32@7.0.1`; `platformio.ini` itself leaves it unpinned, and an
already-installed matching platform satisfies it. `micro_ros_platformio` is a git `lib_deps` entry
without a pin (reference card: `cfee17f`); its `libmicroros.a` is built on the first `pio run`,
which needs the network and has not been done here (open point 4). No PlatformIO udev rules —
the reference card had none either; flashing goes through `/dev/esp32` with the `dialout` group.

**Not provisioned:** `~/microros_ws`, MoveIt, the
leftover pip packages of the LeRobot stack, lint/dev packages of the reference card (`clang-tidy` →
`llvm-18-dev`, `bison`, `flex`, `libncurses-dev`, `libserial-dev`, `can-utils` — none is used by a
package in `Software/ros2`). `CARRY_OVER.md` §4 lists the files.

**Kept as on the reference card:** `gripperx.target`
installed but not enabled, no `gripperx-button` sudoers drop-in (the button daemon's `sudo -n`
works through cloud-init's passwordless sudo, checked by step 0).

---

## Open points (TO-VERIFY)

1. **Nothing has run on a Pi.** Steps 0, 1, 4–8, 10, 11, 14 and 15 were reviewed but only
   smoke-tested where a container allows; the first real run is on the robot.
2. **Kernel after step 5.** The reference card ran `6.8.0-1065-raspi`. A full-upgrade brings today's
   noble kernel; whether that is 1065 or newer is not known until it runs. `VERIFY.md` 2.8 checks
   the one thing a kernel change is known to move on a Pi 5 (the RP1 GPIO chip staying `gpiochip4`).
3. **The chrony slew configuration is not deployed on the reference card.** The file in
   `Software/pi_env/chrony/` argues that the Pi must never step its clock after the first sync
   (a backwards step is a `CLOCK_JUMPED_BACK` auto-disarm, SR-15), but the robot runs timesyncd,
   which steps. This card reproduces the reference; switching to chrony is a separate decision.
4. **First firmware build is not offline.** Step 16 prefetches the platform, toolchains and the
   `micro_ros_platformio` library, but `libmicroros.a` is only built by the first `pio run`
   (downloads micro-ROS sources; the reference card's build directory is 564 MB). The library
   revision follows the upstream default branch, not the reference card's `cfee17f`. Building and
   flashing remain a separate, user-approved action.
5. **Snapshot key expiry.** The snapshots.ros.org key expires 2027-06-01; after that, `apt-get
   update` against the snapshot fails until a refreshed key is committed here.
