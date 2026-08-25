# Phase 1 — APPLIED

> **Status corrected 2026-08-25.** This file said *"prepared, not applied"*, which
> described the branch and stopped being true at the merge (`9f68ed4`). The
> migration has run: `swerve_cmd.yaml` and `ros2_controllers.yaml` carry
> `a: 0.1809 / b: 0.1087` with the pointer comment, the test files import from
> `gripperx_geometry.constants`, and `KNOWN_DIVERGENCES` is empty. Re-running
> `phase1_migrate.py` now reports every target as already migrated — that is the
> script working, not a fault. **It is a spent artefact, kept as the record of
> what was changed.**
>
> One target was removed from it on 2026-08-25 (user decision): the deletion
> round retired `swerve_cmd_node.py`, so `NODE_DEFAULTS` went from four entries
> to three. The three `geometry.yaml` consumer sites that named that file were
> removed with it — `test_every_consumer_site_still_exists` caught them, which is
> the test doing exactly its job. 13 of 13 consistency tests pass after the
> change; two failed before it.

Everything here is **dry-run verified** against a scratch copy of `Theo`. Nothing
in this branch touches a file outside `gripperx_geometry`, so it collides with
no other session until it is merged.

Run it with:

```bash
python3 scripts/phase1_migrate.py --src-root <ws>/Software/ros2/src            # show the diff
python3 scripts/phase1_migrate.py --src-root <ws>/Software/ros2/src --apply    # do it
```

The script refuses to write **anything** if any single edit does not match, so a
partial migration is not a state it can produce.

## Verified end state

| Check | Result |
|---|---|
| Files changed | 17 patched, 1 created *(16 after the 2026-08-25 target removal)* |
| `colcon build --packages-up-to gripperx_teleop gripperx_localization gripperx_arm_msgs` | 8 packages, clean |
| `gripperx_geometry` consistency tests | 13 passed |
| `KNOWN_DIVERGENCES` | **empty** |
| `check_steering_limits.py` / `check_manoeuvres.py` / `check_teleop_manoeuvre_path.py` | all run |

## The change set

**Node defaults → mandatory** (4 files; **3 since 2026-08-25** — `swerve_cmd_node.py` was deleted). `declare_parameter("a", 0.203)` becomes
`declare_parameter("a", Parameter.Type.DOUBLE)`. A node that cannot find its
parameter file then fails at startup instead of running on a stale literal.

**The GQ-1 correction** (3 files). `a: 0.180 → 0.1809`, `b: 0.110 → 0.1087` in
`swerve_cmd.yaml`, `teleop_joint_commands.yaml`, `ros2_controllers.yaml`.

**Test literals → imports** (4 files). `A = 0.203` becomes
`from gripperx_geometry.constants import HALF_WHEELBASE_KINGPIN as A`.

**A parameter file keyboard_teleop_node never had** (created, plus 3 files).
See the finding below.

**Dependencies and the registry** (4 files). `<test_depend>` where a test now
imports, and `KNOWN_DIVERGENCES` emptied.

## Two findings the dry run produced

### keyboard_teleop_node has been running on obsolete geometry

It receives `a`/`b`/`wheel_radius` from **no launch path** —
`laptop_teleop.launch.py` passes an inline dict that does not include them — so
it has always fallen back on its in-code defaults, the obsolete
`a=0.203 / b=0.16556` pair.

It uses them. `keyboard_teleop_node.py:237` builds a `FourWIS4WIDKinematicModel`
and the comment at the call site claims the result is *"exactly the pose
swerve_cmd_node will command for this twist"*. Computed with the real model
code, it is not:

| | in-place spin pose |
|---|---|
| `keyboard_teleop_node` (today) | ±50.80° / ±129.20° |
| `swerve_cmd_node` (today) | ±58.57° / ±121.43° |
| both, after Phase 1 | **±59.00° / ±121.00°** |

**7.77° apart, on every wheel.** The arming guard compares that prediction
against measured steering to decide `drive_allowed`, so it has been judging
against a pose the robot is never commanded to reach. Phase 1 closes it as a
side effect. This is a live defect, not a migration obstacle, and it is
independent of OP-29.

### Two edits to one file did not compose

The first version of the script read each file from disk per edit, so the second
edit to `check_teleop_manoeuvre_path.py` silently discarded the first. The dry
run surfaced it as a `NameError` on a constant the discarded edit would have
imported. Fixed by chaining staged text. Worth recording because it is the exact
failure a hand-run migration produces without noticing.

## Still to do at intervention time

1. **Prose.** 12 comment lines in the three configs still name a superseded
   number. The script reports them and deliberately does not rewrite them —
   those blocks carry the measurement history and the OP-29 reasoning, which no
   regex should touch. List: `--src-root ...` without `--apply`.
2. **The internal requirements document** still carries `a`/`b` as `TO-VERIFY`. GQ-4 retired that
   doubt on 2026-08-21; the document is now behind `geometry.yaml`.
3. **`swerve_controller.sim.yaml`** was not examined for geometry keys. It is
   not a declared consumer and may need to become one.

## Sequencing — Phase 1 may land, the Pi must wait

The `a`/`b` correction moves the commanded in-place-spin steering angle by
0.43°, module speed by +0.045 % and the odometry rotation scale by +0.091 %. A
repo commit moves nothing; a deploy does.

```
OP-29 chirality test  ->  regulator deploy  ->  Phase 1 deploy + drive test
```

The OP-29 test must run **first and open-loop** — an enabled regulator removes
the chiral shortfall from the velocity signal and destroys the observation.
