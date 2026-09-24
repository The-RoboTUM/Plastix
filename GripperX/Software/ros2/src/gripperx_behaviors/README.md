# gripperx_behaviors

One Nav2 recovery behaviour: **`gripperx_behaviors::CrabWalk`**, a bounded,
pure-lateral escape that uses the swerve chassis's ability to move sideways
without rotating.

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
| `<behavior_name>.max_distance` | `0.20` | `nav2.yaml` sets `0.170` |
| `<behavior_name>.allow_mirrored_fallback` | `true` | |
| `simulate_ahead_time` | `2.0` | node-level, shared with the other behaviours on the server |

Everything else (cycle frequency, frames, transform tolerance, the cmd_vel
publisher) is inherited from `TimedBehavior`.

## It is already wired

`gripperx_planning/config/nav2.yaml` lists `crab_walk` in `behavior_plugins` and
configures it, and both behaviour trees invoke it as
`<DriveOnHeading server_name="crab_walk" dist_to_travel="0.17" speed="0.10"
time_allowance="2.8"/>`. Keep `dist_to_travel` at or below `max_distance`.

## Known risk

This behaviour has had the least field exposure of anything in the Nav2 stack. If
the autonomy stack misbehaves during recovery, treat `crab_walk` as the first
suspect and confirm by checking whether the failing goal reached a recovery branch
at all.
