# gripperx_geometry

Single source of truth for GripperX robot geometry.

For: anyone who touches a geometric value anywhere in the workspace. Status: 2026-11-17. The
package declares no nodes and ships no launch files; its checked-in constants module is imported
by a few packages (see "Who reads it").

Short forms: OP-29 (open point 29 — the odometry/kinematics left-right and contact-patch
question), GQ-n (geometry question n, recorded in `config/geometry.yaml` under
`open_questions`), CAD (computer-aided design model), URDF (robot description file).

## The one rule

**Never edit a geometric value in a config, node, header, xacro or test.** Edit
`config/geometry.yaml`, regenerate, and run the tests:

```bash
python3 -m gripperx_geometry.generate --sync-constants   # rewrite the checked-in constants module
colcon test --packages-select gripperx_geometry           # prove every consumer agrees
python3 -m gripperx_geometry.generate --check             # human-readable drift report
python3 -m gripperx_geometry.generate --out DIR           # write the derived artefacts into DIR
```

Each quantity in `geometry.yaml` carries its `provenance` (CAD, measured, decided), `date`,
`status`, `rationale`, and a `consumers` list naming every file that must agree. The test reads
those consumer sites back off disk and fails on drift, and also on a consumer that no longer
resolves. Files that are generated from the declaration (for example
`gripperx_teleop/config/keyboard_teleop.yaml` and
`gripperx_geometry/src/gripperx_geometry/constants.py`) carry a "GENERATED ... DO NOT EDIT"
banner.

## Layout

| Path | What it is |
|---|---|
| `config/geometry.yaml` | **The declaration.** Edit values here and nowhere else. |
| `src/gripperx_geometry/constants.py` | Generated Python constants. Import these instead of writing a number into a test. |
| `src/gripperx_geometry/generate.py` | Emits the derived artefacts (`gripperx_geometry_generate` entry point). |
| `src/gripperx_geometry/inventory.py` | Reads the consumers back off disk. |
| `test/test_geometry_consistency.py` | Proves the consumer sites agree. `KNOWN_DIVERGENCES` pins any site allowed to disagree, with the value it holds; it is empty now. |
| `scripts/phase1_migrate.py`, `PHASE1_MANIFEST.md` | The one-time migration of the control configs and tests onto this source. A spent artefact, kept as the record. |

## What is declared

Wheel radius (geometric and effective), king-pin and contact-patch wheelbase and track, the
king-pin to contact lateral offset (in joint order) and the contact positions (in model order),
steer mount and base heights, the arm mount pose, the arm joint limits and mechanical stops, the
LiDAR mount and collision envelope, the four chassis collision boxes with their padding, and the
IMU mount pose. The exact list and every value are in `config/geometry.yaml`: this page does not
restate numbers.

## Quantities are distinct and named

Two numbers that are equal today are not thereby the same quantity, and two that differ are not
thereby in conflict. Collapsing a pair into one variable would build in the next class of error.

- **Geometric versus effective wheel radius.** Currently the same number by coincidence of
  measurement, not identity: a loaded tyre generally rolls smaller than its geometric radius.
  The effective value is a carpet/hard-floor figure; an outdoor value for the lugged tyres is a
  known, accepted gap (GQ-3, decided by the user).
- **King-pin versus contact point.** The half-track and half-wheelbase at the king pins and at
  the tyre contact patches differ, and both are correct: the gap is the lateral lever arm from
  each king pin out to its tyre. Steering geometry needs the king-pin figures; odometry needs the
  contact-patch figures. Confusing them is OP-29. For the steering angles the two answers are
  identical by derivation (GQ-2); the contact patch matters for wheel speeds and odometry.
- **Joint order versus model order.** Two wheel orders are live and both are correct. Joint
  order is FL, FR, BL, BR (`steer_servo.yaml`, `/hw/joint_commands`, `/teleop/direct_steer`).
  Model order is FL, BL, BR, FR (the kinematic model, the xacro, `localization.yaml` wheel
  positions). The controller converts explicitly; a value moved between lists without
  reordering swaps two wheels silently. The 2026-08-24 left/right odometry sign fault was such a
  model-order confusion.

## Who reads it

The generated constants module is imported at runtime by `gripperx_control`
(`steering_limits.py`), `gripperx_teleop` (`keyboard_teleop_node.py`), `gripperx_leader` (`follower_config.py`, `leader_arm_node.py`), `gripperx_arm`
(`eeprom_limits.py`, `stop_protected.py`) and the IMU axis check in `gripperx_sensors`. The URDF
in `gripperx_description` and the config files of the control, localization and teleop packages
carry the values as generated or checked consumers.

## Status of the open questions

All four geometry questions in `geometry.yaml` are resolved: GQ-1 (the control configs held an
independent tape measurement of the king-pin spacing; the user decided for the CAD figures as one
quantity, 2026-08-21), GQ-4 (the robot is mainly 3D-printed, so parts carry CAD dimensions),
GQ-2 (answered by derivation, 2026-08-25) and GQ-3 (accepted gap, 2026-08-25). The migration of
the consumers onto the declaration (Phase 1) has been applied; correcting the tape pair to CAD
moved the commanded in-place spin steering angle slightly, so the deploy to the robot follows a
drive test (see `PHASE1_MANIFEST.md`; whether that drive test has been done is not recorded here:
TO-VERIFY).

## Scope

Not modelled yet: joint dynamics and the four per-corner `*_wheel_offset_xyz` triples in
`gripperx_v1.core.xacro` (see the note in `generate.py`). Steering limits live in
`gripperx_control/config/steer_servo.yaml`.

## Build and test

```bash
cd Software/ros2 && colcon build --packages-select gripperx_geometry
colcon test --packages-select gripperx_geometry && colcon test-result --verbose
```
