# gripperx_planning

Nav2 for the robot: global planner, DWB-based holonomic controller (holonomic path following),
recovery behaviours, behaviour-tree navigator and a velocity smoother, plus the project's
goal-checker configuration.

For: anyone who tunes or debugs autonomous driving. Status: 2026-11-17. Nav2 has been verified in
the twin (`gripperx_gazebo`) far more than on the robot: treat twin success as necessary, not
sufficient.

Short forms: DWB (Nav2's dynamic window controller), BT (behaviour tree), SR-9 (safety
requirement 9 — `teleop_mux` is the single publisher on `/cmd_vel`), GAP-1 (the design rule behind
SR-9, named in the config comments).

**Does not** include EKF, SLAM or AMCL: launch `gripperx_localization` first (contract:
`INTERFACE.md`).

## Launch

```bash
ros2 launch gripperx_planning navigation.launch.py [use_sim_time:=true]
```

On the robot it is started by the `gripperx-navigation` systemd service, or by
`gripperx_bringup/launch/real_autonomy.launch.py enable_navigation:=true`.

| Launch argument | Default | Meaning |
|---|---|---|
| `use_sim_time` | `false` | `true` only for the twin. |
| `params_file` | `config/nav2.yaml` | Nav2 parameter file. |
| `default_nav_to_pose_bt_xml` | `config/navigate_to_pose_w_replanning_and_recovery.xml` | BT for `NavigateToPose`. |
| `default_nav_through_poses_bt_xml` | `config/navigate_through_poses_w_replanning_and_recovery.xml` | BT for `NavigateThroughPoses`. |
| `enable_terrain_cost`, `terrain_cost_yaml` | `false` | Optional terrain cost map on `/terrain_costmap` (needs the terrain layer in `nav2.yaml`). |

Started nodes (lifecycle, brought up by `lifecycle_manager_navigation`): `planner_server`,
`controller_server`, `behavior_server`, `bt_navigator`, `velocity_smoother`; plus an optional
`terrain_cost_map_server`.

## The command chain (do not short-circuit it)

```
controller_server / behavior_server --> /nav/cmd_vel_raw --> velocity_smoother
    --> /teleop/autonomous/cmd_vel --> teleop_mux --> /cmd_vel --> swerve_controller
```

`teleop_mux` (package `gripperx_teleop`) forwards the autonomous branch **only while it is in
`autonomous` mode**; the default is `keyboard`, so Nav2 goals do nothing until the mode is set
(`/teleop/set_mode`). `teleop_mux` stays the single publisher on `/cmd_vel`. The smoother's input
and output topics must stay different names. There is no `swerve_cmd_node` in the path any more.

## Actions and topics

Provided by `bt_navigator`: `/navigate_to_pose`, `/navigate_through_poses`. Also served:
`/follow_path` (controller) and the behaviour actions. Consumed (Nav2 defaults): `/odometry/filtered`,
the costmaps built from `/scan` and `/map`, TF (coordinate-transform tree). The external-goal gateway
(`gripperx_external`) is the main goal client on the robot; RViz "2D Goal Pose" is the manual one.

## Configuration

`config/nav2.yaml` (all Nav2 servers, with the decisions and measurements in comments). Points that
matter:

- Goal tolerances: the goal checker, the FollowPath controller's rotate-to-goal window and the
  global planner's `tolerance` are coupled; the comments in `nav2.yaml` state the invariant and
  that the planner tolerance now equals, not undercuts, the goal checker's `xy_goal_tolerance`
  (marked TO-VERIFY there). `test/test_goal_checker_invariants.py` holds other invariants: every
  `FollowPath` names a loaded goal checker, the oscillation-reset angle stays below every
  checker's yaw tolerance, and the re-aim tree stays a pure rotation with a tighter checker.
- `reaim_goal_checker` and `config/reaim_in_place.xml`: never a default. The external-goal gateway
  sends that BT per goal (`NavigateToPose.behavior_tree`) for the in-place re-aim at arrival.
- Recovery behaviours: `drive_on_heading`, `backup`, `crab_walk`, `wait`. `crab_walk` is this
  project's own plugin (`gripperx_behaviors`, see its README), a bounded pure-lateral escape, and
  the least field-exercised part of the stack: suspect it first when autonomy misbehaves in
  recovery.
- DWB stays forward-only (no lateral velocity); only `crab_walk` may command lateral motion.

## Known defects

- **The drivetrain rotation shortfall consumes the yaw tolerance** on turns beyond roughly 90
  degrees: goal checkers tuned as if rotation were delivered in full time out on large heading
  changes. See `gripperx_swerve_controller/README.md`.
- The twin's own load defect: `gripperx_gazebo/README.md`.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_planning
colcon test --packages-select gripperx_planning && colcon test-result --verbose
```

Desk tests only (`test_goal_checker_invariants.py`, `test_bt_ack_window.py`,
`test_annulus_exclusion.py`); none moves hardware. Starting Nav2 on the robot with the mux in
`autonomous` makes the robot commandable: that needs an approval under SR-1 (safety requirement 1
— no motion without explicit approval per test).
