# micro-ROS — troubleshooting (Pi + ESP32)

## Correct startup order

Use **two terminals** (plus an optional third for teleop).

**Terminal 1 — agent (keep running):**

```bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source ~/uros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 115200 -v6
```

`/dev/esp32` is the udev symlink from `pi_env/udev/99-gripperx.rules`. Use it rather
than the raw device: the ESP32-S3 board and both Feetech servo adapters share the same
CH343 vendor/product id, so `/dev/ttyACM0` can be any of the three depending on plug
order. On a laptop without the udev rule, substitute the raw `/dev/ttyACM0`.

On the Pi, the agent normally runs as the Docker-based
`gripperx-agent.service` (see `pi_env/systemd`) instead of this manual command;
use the manual invocation above for ad-hoc debugging. Stop the service first —
a second agent on the same device is the classic DDS-zombie cause, see
`documentation/DEPLOYMENT.md` §1:

```bash
sudo systemctl stop gripperx-agent.service && docker stop mros_agent
```

Wait for logs such as `create_publisher`, `create_subscription`, or `session established`.

**Terminal 2 — check topics:**

```bash
export ROS_DOMAIN_ID=20
source /opt/ros/jazzy/setup.bash
ros2 node list          # → /gripperx_firmware
ros2 topic echo /hw/joint_states
```

If topics are missing, press **RESET** on the ESP32 while the agent is running.

---

## Checklist

| Check | Command / action |
|-------|------------------|
| Port exists | `ls -l /dev/esp32 /dev/ttyACM*` |
| Symlink points at the S3 | `udevadm info -n /dev/esp32 \| grep DEVNAME` — must not be a servo adapter |
| Permissions | `sudo chmod 666 /dev/ttyACM0` (the udev rule already sets `MODE="0666"`) |
| Same domain ID | `echo $ROS_DOMAIN_ID` → `20` in all terminals |
| Agent installed | `source ~/uros_ws/install/setup.bash` |
| Node visible | `ros2 node list` → `/gripperx_firmware` |
| Message type | `ros2 topic info /hw/joint_states -v` → `Float64MultiArray` |
| Encoder firmware active | `ros2 topic echo /hw/joint_states` → **16** values. 12 = encoder feedback but no provenance block (pre-FR-11 firmware); 8 = pre-encoder firmware. Corrected 2026-08-24 against `firmware/src/main.cpp` (`NUM_STATE_JOINTS 16`) and `gripperx_hardware_interface/INTERFACE.md`. |
| No serial monitor | Close the PlatformIO Monitor while the agent is running |
| Firmware distro | `board_microros_distro = jazzy` in `platformio.ini` |
| Firmware env | `pio run -e esp32-s3` — there is no `esp32dev` env any more |

---

## Agent log

Once the ESP32 connects you should see:

```
create_publisher ... hw/joint_states
create_subscription ... hw/joint_commands
```

If there is only ping/heartbeat but **no create_publisher**, the ESP32 did not finish setup. Reflash and reset the board.

---

## Common errors

1. **Reading topics in the same terminal as the agent** — the agent blocks that shell; open another terminal.
2. **Different `ROS_DOMAIN_ID`** — the agent sees the topics but `ros2 topic list` does not. Use `export ROS_DOMAIN_ID=20` in all of them.
3. **Agent started before flashing jazzy firmware** — reflash, then reset with the agent running.
4. **Float32 vs Float64** — must be `Float64MultiArray` on both sides.
5. **Keyboard not responding** — `teleop_twist_keyboard` needs an interactive terminal (TTY). Use a separate terminal (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`).
