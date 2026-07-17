# micro-ROS — troubleshooting (Pi + ESP32)

## Correct startup order

Use **two terminals** (plus an optional third for teleop).

**Terminal 1 — agent (keep running):**

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source ~/uros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6
```

On the Pi, the agent normally runs as the Docker-based
`gripperx-agent.service` (see `pi_env/systemd`) instead of this manual command;
use the manual invocation above for ad-hoc debugging.

Wait for logs such as `create_publisher`, `create_subscription`, or `session established`.

**Terminal 2 — check topics:**

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
ros2 node list          # → /gripperx_firmware
ros2 topic echo /hw/joint_states
```

If topics are missing, press **RESET** on the ESP32 while the agent is running.

---

## Checklist

| Check | Command / action |
|-------|------------------|
| Port exists | `ls -l /dev/ttyUSB*` |
| Permissions | `sudo chmod 666 /dev/ttyUSB0` |
| Same domain ID | `echo $ROS_DOMAIN_ID` → `0` in all terminals |
| Agent installed | `source ~/uros_ws/install/setup.bash` |
| Node visible | `ros2 node list` → `/gripperx_firmware` |
| Message type | `ros2 topic info /hw/joint_states -v` → `Float64MultiArray` |
| No serial monitor | Close the PlatformIO Monitor while the agent is running |
| Firmware distro | `board_microros_distro = jazzy` in `platformio.ini` |

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
2. **Different `ROS_DOMAIN_ID`** — the agent sees the topics but `ros2 topic list` does not. Use `export ROS_DOMAIN_ID=0` in all of them.
3. **Agent started before flashing jazzy firmware** — reflash, then reset with the agent running.
4. **Float32 vs Float64** — must be `Float64MultiArray` on both sides.
5. **Keyboard not responding** — `teleop_twist_keyboard` needs an interactive terminal (TTY). Use a separate terminal (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`).
