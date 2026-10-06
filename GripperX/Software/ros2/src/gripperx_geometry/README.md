# gripperx_geometry

Single source of truth for GripperX robot geometry.

This package has **no runtime role**. It declares no nodes, ships no launch
files, and nothing on the robot imports it. It exists so that a geometric
quantity is written down once and every other copy of it can be proven to agree.

## Why

Before this package, `wheel_radius` was written out at **13 code sites** plus
documentation, and `a`/`b` at **8**. They drifted. As of 2026-08-21 three node
defaults and two test files still carried the obsolete `a=0.203 / b=0.16556`
CAD pair — a robot with a 0.406 m wheelbase and 0.331 m track that has not
existed since 2026-08-19 — while the configs had moved on to `0.180 / 0.110`.
Any node started without its parameter file used the obsolete robot silently.

## Layout

| Path | What it is |
|---|---|
| `config/geometry.yaml` | **The declaration.** Edit values here and nowhere else. |
| `src/gripperx_geometry/inventory.py` | Reads consumers back off disk. |
| `src/gripperx_geometry/generate.py` | Emits the derived artefacts. |
| `test/test_geometry_consistency.py` | Proves the 42 consumer sites agree. |

## The rule

**Quantities are distinct and named.** Two numbers that are equal today are not
thereby the same quantity, and two that differ are not thereby in conflict.

* `wheel_radius_geometric` and `wheel_radius_effective` are both `0.070` today —
  by coincidence of measurement, not identity. A loaded tyre generally rolls
  smaller than its geometric radius.
* `half_track_kingpin` (`0.1087`) and `half_track_contact` (`0.164222`) differ by
  51 % and **both are correct**. The gap is the lateral lever arm from each king
  pin out to its tyre. Steering geometry needs the first; odometry needs the
  second. Confusing them is OP-29.

Collapsing either pair into one variable would build in the next class of error.

## Using it

```bash
colcon test --packages-select gripperx_geometry     # prove the tree agrees
colcon test-result --test-result-base build/gripperx_geometry

python3 -m gripperx_geometry.generate --check       # human-readable drift report
python3 -m gripperx_geometry.generate --out DIR     # write derived artefacts
```

## Changing a value

1. Edit it in `config/geometry.yaml`, with `provenance`, `date` and `rationale`.
2. Run the test. It will name every site that now disagrees.
3. Update those sites — or, from Phase 1, regenerate them.

Never edit a consumer by hand. That is the failure mode this package exists to
remove.

## Known divergences, and why the test is green anyway

Twenty sites disagree with the declaration right now. Each is registered in
`KNOWN_DIVERGENCES` in the test, **with the value it holds**, so that:

* the test is green on the known state and red on any *new* drift, instead of
  being red from birth and therefore ignored;
* a registered site that moves to some third value still fails;
* a registered site that gets *fixed* also fails, with an instruction to delete
  its entry — the registry cannot decay into a permanent amnesty.

Phase 1 empties the registry.

**The two halves are no longer the same kind of thing.** Ten sites hold the
obsolete `0.203 / 0.16556` pair; ten hold the tape pair `0.180 / 0.110` that
GQ-1 decided against. Both are known-wrong with a scheduled fix — neither is an
open question any more.

### Phase 1 lands in the repo; the Pi waits

Correcting the tape pair to CAD is **not** value-neutral. It moves the commanded
in-place-spin steering angle from **58.570°** to **58.999°**, module speed by
+0.045 % and the odometry rotation scale by +0.091 %. Small, non-zero, and it
changes how the robot moves.

So Phase 1 may **land**, but must not be **deployed** before a drive test — and
that test sits behind the OP-29 chirality test in the ordering constraint:

```
OP-29 chirality test  ->  regulator deploy  ->  Phase 1 deploy + drive test
```

For scale: GQ-1 is a 0.045 % question. OP-29, unresolved in the same kinematics,
is a ±26 % one.

The registry has already earned its keep once. Between it being written and the
branch being rebased, `Theo` advanced 44 commits and `check_teleop_manoeuvre_path.py`
moved from the obsolete pair to the config pair (fbd2276). The test caught it
unprompted, named both sites, and reported what had changed — real drift, not
injected.

## Open questions

Recorded in `config/geometry.yaml` under `open_questions`, deliberately not
resolved by this package:

* **GQ-1 — DECIDED 2026-08-21: CAD wins, as one quantity.** The premise turned
  out to be wrong: `0.180 / 0.110` were never roundings (`0.1087` rounds to
  `0.109`) but an independent **tape measurement** of the same king-pin spacing,
  taken on the rebuilt robot 2026-08-13 (commit `275246c`). Two methods, two
  answers, 0.9 mm and 1.3 mm apart. The user decided for the CAD figures, and
  every consumer converges on them in Phase 1.
* **GQ-2** — should the inverse kinematics use the contact-patch pair? This is
  OP-29 as a geometry question. Not this package's call.
* **GQ-3** — does `wheel_radius_effective` need a second, outdoor value? The
  measured `0.070` is a carpet/hard-floor figure and the tyres are lugged.
* **GQ-4** — is the CAD king-pin spacing right for the **as-built** robot? GQ-1
  settled what the workspace declares, not what the machine measures. CAD
  describes the design; the chassis was hand-rebuilt. Settled by caliper on a
  defined reference face plus the CAD offset — static, no movement, no approval
  needed. Blocks nothing.

## Scope

Phase 0 covers the quantities duplicated across two or more sites, plus the
couplings that have broken before (ride height against wheel radius). Steering
limits, joint dynamics, mesh offsets and the arm are **not** modelled yet.
Neither are the four per-corner `*_wheel_offset_xyz` triples — see the note in
`generate.py`.
