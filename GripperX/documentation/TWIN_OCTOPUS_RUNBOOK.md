# Twin run book — Octopus link on the 2.5 x 2.5 m demo area

Created 2026-08-21. Brings up the digital twin so external Octopus goals can be
fed in by hand, previewed, armed and executed. **Twin only, ROS_DOMAIN_ID=220,
no hardware, no Pi.**

State this assumes (all done 2026-08-21). **Re-verified against the tree
2026-08-24 — all four still hold:**
- `octopus_link_twin.yaml` geofence = ±1.25 m, `allow_arm: true`
  (`geofence.min_x_m`/`max_x_m`/`min_y_m`/`max_y_m`, `allow_arm: true`)
- `demo_area_2m5.world.sdf` exists, is committed, and is installed by
  `gripperx_gazebo/CMakeLists.txt`
- `keyboard_teleop_node` has the `U` / `L` arming keys
- `gripperx_external`, `gripperx_gazebo`, `gripperx_teleop` all rebuilt
  — rebuild state is per-machine and stays TO-VERIFY

---

## 0. Every terminal, first

The laptop's `.bashrc` sources `~/ros2_ws` and sets `ROS_DOMAIN_ID=0`. Both are
wrong here: the package will not be found, and the gateway exits **FATAL** on a
domain that is not 220 (SR-8).

```bash
source /opt/ros/jazzy/setup.bash
source ~/gripperx_ws/Software/ros2/install/setup.bash
export ROS_DOMAIN_ID=220
```

## 1. Gazebo + Nav2 + SLAM

Start this FIRST. `env:=twin` implies `use_sim_time:=true`, and the gateway
refuses to arm until it has seen `/clock` advance (SAFETY.md F-24).

```bash
ros2 launch gripperx_gazebo sim_navigation.launch.py \
  world_file:=$(ros2 pkg prefix gripperx_gazebo)/share/gripperx_gazebo/worlds/demo_area_2m5.world.sdf \
  localization:=slam spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
```

`spawn_yaw:=0.0` makes map, odom and world axes coincide: a target at map (x, y)
is at world (x, y), no rotation to reason about.

**Check before going on:**

```bash
ros2 run tf2_ros tf2_echo map base_footprint     # expect ~ (0, 0)
ros2 topic hz /clock                             # must be publishing
```

If `map -> base_footprint` is not near (0,0), the geofence rectangle is not where
you think it is. Stop and re-check rather than placing targets.

## 2. Fake Octopus (mock rosbridge + their two producer nodes)

```bash
cd ~/gripperx_ws/Software/ros2/src/gripperx_external
python3 test/fake_octopus.py --port 9090 --targets "1.0,0.6;-0.8,0.9;0.4,-1.0"
```

Interactive on stdin: `add <x> <y>`, `remove <id>`, `unreachable <id>`,
`move-datum-m <dx> <dy>`, `silence <gps|goal|datum>`, `resume`, `disconnect`,
`outage <sec>`, `restart`, `status`, `help`.

The geofence bounds the **robot pose**, not the object, so a target slightly
outside ±1.25 can still be served. Beyond ~1.61 m from centre it is always refused.

## 3. Link + gateway

```bash
ros2 launch gripperx_external octopus_link.launch.py \
  env:=twin goal_ingress:=true dry_run:=false
```

Both flags are startup-only (`ros2 param set` on them is refused) — they belong
on this line, not at run time.

## 4. Teleop — mode and arming

```bash
ros2 run gripperx_teleop keyboard_teleop_node
```

Press **`G`** for autonomous mode. This is layer 2 of the authority gate: without
it Nav2 output never reaches `/cmd_vel`, and a stale mux (`max_teleop_mode_age_sec`
2.0) cancels a goal in flight. Equivalent without teleop:

```bash
ros2 topic pub --once /teleop/set_mode std_msgs/String '{data: "autonomous"}'
```

Then **`U`** to arm (120 s window), **`L`** to disarm. Equivalent by service:

```bash
ros2 service call /gripperx/external/set_arming \
  gripperx_external_msgs/srv/SetArming \
  '{arm: true, duration_sec: 120.0, requested_by: "theo"}'
```

## 5. Optional — the pick success path

The twin has no arm in the URDF (DT-8 / OP-11), so `auto_pick` exercises the
decision logic and nothing moves. For a run that reaches `trash_goal_done`, add a
mock pick server with its nav server switched OFF so it does not compete with the
real Nav2:

```bash
cd ~/gripperx_ws/Software/ros2/src/gripperx_external
python3 test/mock_motion_servers.py --ros-args -p nav_available:=false
```

It is a test script, not a console script — `ros2 run gripperx_external
mock_motion_servers` does NOT exist. It refuses to start on domain 20.

Without it the stub reports unavailable, nothing is acknowledged, the target is
blacklisted — the deliberately honest failure path, not a defect.

## 6. Watching it

```bash
ros2 topic echo /gripperx/external/arming_state      # seconds_remaining is the field to trust
ros2 topic echo /diagnostics --once
rviz2                                                # preview markers
```

On a successful arm the gateway logs:

```
ARMED for 120s by 'keyboard_teleop'. Remaining blocks: ...
```

**That "Remaining blocks" list is the most useful line of the session** — it names
what still blocks dispatch. With the launch above it should be empty.

---

## Shutdown

Ctrl+C in reverse order (teleop, gateway, fake, Gazebo). A `SIGKILL`ed FastDDS
leaks `/dev/shm`; if a later run behaves strangely, `Software/ros2/scripts/shm_clean.sh` is the cure.

## What this does NOT test

- **No obstacle in the play area.** An unobstructed run is not evidence that
  avoidance works — that is `testworld_simple`.
- **No real Octopus timing.** The fake publishes at our own rate, so
  `max_target_list_age_sec` is still unmeasured against their machine
  (FR-12 §10.1 item 5).
- **Nothing here touches the real robot.** §10.1's fourteen items stand.
