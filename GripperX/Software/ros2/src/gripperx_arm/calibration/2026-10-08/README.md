# Follower arm calibration evidence, 2026-10-08

**What:** the raw evidence behind the follower calibration in `../../config/arm_calibration.yaml`
and the arm joint stops/limits in `gripperx_geometry/config/geometry.yaml`.
**For:** whoever re-checks or repeats the calibration. **Status:** recorded 2026-10-08;
`calibration_verified` is still `false` (user acceptance on the running system pending).

Set-up: SO101 follower arm (LeRobot SO-101 design) on the robot, torque OFF on servos 1-5 throughout,
arm moved by hand only. Robot standing on its wheels, level (user statement; no spirit level used).
Raw ticks of servo IDs 1..6 (limb1..limb5, gripper; 4096 ticks per turn) were logged at 10 Hz.
The full logs are not versioned; the files here are the extract that the results depend on.

## Files

| File | Content |
| --- | --- |
| `stop_extremes.csv` | Per joint the outermost raw tick reading of the day (UTC time of the sample) and its angle under the new calibration. limb1 is taken from the sweep after its mounting screws were removed; all other joints from the whole session log. |
| `touches.csv` | Median raw ticks of each 2 s capture (all captures were still: spread 0-1 tick), with the reference the TCP (tool centre point, the gripper fingertip point of the model) touched. |

## Results

**Signs** `[1, -1, -1, -1, 1, 1]`: the user moved each joint by hand and compared the direction with
the URDF model (robot description) in RViz, joint by joint.

**Zeros** `[2108, 978, 3074, 1035, 2148, 1527]`:
- limb1, limb4, limb5, gripper: the hand-taught home of 2026-08-25, unchanged.
- limb3 3074 (was 3091), user decision "option a": the stop sweep read tick 3082 as the lower stop,
  so 3091 lay 9 ticks behind the stop. Now the stop is at -0.70 deg and home 0.7 deg inside it.
- limb2 978 (was 1003): **a fit result, not a direct measurement.** Least-squares fit of one limb2
  zero offset to the model TCP heights of the 7 floor touches: -2.19 +- 0.38 deg (= -24.9 ticks,
  sign -1). Leaving one point out kept the result between -1.8 and -2.4 deg. A forward mount tilt
  of about 2 deg explains the floor points almost equally well; the 3 block touches at a second
  height were taken to tell the two apart and agree with the limb2 correction.

TCP height under the new calibration (repository kinematics, `base_footprint`), reference in brackets:

| Capture | Height | Capture | Height |
| --- | --- | --- | --- |
| boden1 (0) | -1.4 mm | boden6 (0) | 0.0 mm |
| boden2 (0) | +1.2 mm | boden7 (0) | -2.5 mm |
| boden3 (0) | -1.5 mm | block1 (~100) | 97.6 mm |
| boden4 (0) | +3.9 mm | block2 (~100) | 101.1 mm |
| boden5 (0) | -2.7 mm | block3 (~100) | 98.2 mm |

Floor residuals -2.7 .. +3.9 mm; block -2.4 .. +1.1 mm against 100 mm. The block was "just under
100 mm" (user), not measured more precisely. Accepted height tolerance: +-5 mm (user; nothing is
commanded below 10 mm). `pose1`..`pose3` were a rough tape check by eye (model now 15.8 / 2.1 /
28.8 mm high against ~10 / 5 / 25 mm); horizontal TCP position is not verified better than about
+-15 mm.

**Stops and limits:** see `stop_extremes.csv` and the `arm_joint_*` entries of `geometry.yaml`,
which carry the per-joint decisions. **Tick wrap:** no joint's travel crosses 0/4095 (no jump between
consecutive samples in any log), so no servo Homing_Offset is needed.

## Re-computing the heights

```python
from gripperx_arm.kinematics import ArmKinematics, TickCalibration
from gripperx_arm.generate_acm import load_urdf
kin = ArmKinematics(load_urdf(), base_link='base_footprint')
# cal = TickCalibration(urdf_joints, zero_ticks, signs, 4096, range_center_rad) from arm_calibration.yaml
z_mm = 1000 * kin.tcp(cal.to_radians(ticks))[2, 3]
```
