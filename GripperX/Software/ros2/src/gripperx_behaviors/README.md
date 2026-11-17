# gripperx_behaviors

One Nav2 recovery behaviour: **`gripperx_behaviors::CrabWalk`**, a bounded,
pure-lateral escape that uses the swerve chassis's ability to move sideways
without rotating.

For: anyone tuning Nav2 recoveries. Status: 2026-11-17 (the code changed only in comments since
the previous README; this revision adds the alignment-grace parameter and the build notes).

Short forms: BT (behaviour tree), Nav2 (the ROS 2 navigation stack), swerve (four-wheel steering
and drive).

Built as the pluginlib library `gripperx_crab_walk_behavior`, exported through
`behavior_plugin.xml` against `nav2_core::Behavior`.

## How it works

It derives from `nav2_behaviors::TimedBehavior<nav2_msgs::action::DriveOnHeading>`.
The `DriveOnHeading` action type is **deliberately reused** so the stock
`DriveOnHeading` BT node can drive it by `server_name` — no custom BT node is
needed.

Each cycle it publishes a twist with `linear.x = 0`, `linear.y = ±speed`,
`angular.z = 0` and holds heading constant. The collision probe is the upstream
one with a single change: it walks the **lateral** unit vector `(−sin θ, +cos θ)`
instead of the heading, because the body does not rotate during a crab.

Before moving it probes the whole manoeuvre. If the preferred side is blocked and
`allow_mirrored_fallback` is true it re-probes the mirrored side and flips
direction; if both are blocked it returns `COLLISION_AHEAD`.

**It rejects, it never clamps.** A non-zero `target.z`, both `target.x` and
`target.y` non-zero, a zero distance, a sign mismatch between distance and speed,
or a distance beyond `max_distance` all return `INVALID_INPUT`. An over-long goal
is refused rather than silently shortened.

## Parameters

| Parameter | Default | Note |
|---|---|---|
| `<behavior_name>.max_distance` | `0.20` (code default) | `gripperx_planning/config/nav2.yaml` overrides it; the reasoning is in the comments there |
| `<behavior_name>.allow_mirrored_fallback` | `true` | |
| `<behavior_name>.alignment_grace_sec` | `0.0` (code default) | Extra time for the stand-still phase while `swerve_controller`'s steering alignment gate withholds drive during the swing into the crab pose. Spent only once the robot is measurably moving, so raising it does not loosen the distance bound. `nav2.yaml` sets it; with the gate off it has no effect. |
| `simulate_ahead_time` | `2.0` | node-level, shared with the other behaviours on the server |

Everything else (cycle frequency, frames, transform tolerance, the cmd_vel
publisher) is inherited from `TimedBehavior`.

## It is already wired

`gripperx_planning/config/nav2.yaml` lists `crab_walk` in `behavior_plugins` and
configures it, and both behaviour trees (`config/navigate_*_w_replanning_and_recovery.xml`)
invoke it through the stock `DriveOnHeading` node with `server_name="crab_walk"`. Keep the BT
port `dist_to_travel` at or below `max_distance`; an over-long goal is rejected, which disables the
recovery instead of shortening it.

## Known risk

This behaviour has had the least field exposure of anything in the Nav2 stack. If
the autonomy stack misbehaves during recovery, treat `crab_walk` as the first
suspect and confirm by checking whether the failing goal reached a recovery branch
at all.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_behaviors
```

The package has no tests of its own (`CMakeLists.txt` only runs the lint hooks). Behaviour is
checked on the twin (`gripperx_gazebo`), with a crab-walk recovery goal.
