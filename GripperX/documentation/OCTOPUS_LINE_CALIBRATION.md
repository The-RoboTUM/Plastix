# Octopus ↔ GripperX — shared reference line (line calibration)

| | |
|---|---|
| **What** | The interface specification for a shared coordinate frame between the Octopus overhead system (the EVE drone — the camera drone hanging from the aluminium frame — and its ground software) and the GripperX robot, defined by two posts of the drone's frame. |
| **For** | The EVE/Octopus team implementing their side; the GripperX operator. |
| **Status** | **Draft. Implemented on the GripperX side (`gripperx_external`), not yet tested against the real Octopus.** Values marked `TO-VERIFY` are unmeasured and must not be replaced by estimates. |
| **Date** | 2026-09-24 |

Other interface documents are referenced by name and not repeated here:
`OCTOPUS_INTERFACE_PROPOSAL.md` (topics, datum, telemetry payload),
`OCTOPUS_ROSBRIDGE_SETUP.md` (transport).

---

## 1. Why

Both sides express positions as flat-earth "fake GPS" (WGS84 — World Geodetic System 1984,
latitude/longitude) around a shared datum. That arithmetic is agreed and unchanged. What was
missing is that the robot's own map is anchored wherever the robot happened to start, at an
arbitrary heading, so the Octopus's metres and the robot's metres were never in the same place.
The line fixes this: **both sides measure the same two physical posts** and express every
coordinate they exchange in the frame those posts define.

## 2. Physical setup

- **Anchors:** two vertical posts of the aluminium frame the drone hangs from. They must stand in
  the scan plane of GripperX's LD06 LiDAR (a 2D laser scanner — light detection and ranging), so
  each post shows up as a small cluster of scan points on the robot's side, and they must be visible
  in the drone's camera image.
- **Post A and post B:** the operator marks one post **A** and the other **B** physically (e.g. with
  tape labelled "A" and "B"). **Both sides must use the same physical post as A.** Swapping A and B
  does not produce an error — it silently turns the whole frame by 180° (every x and y changes sign).
- **What is marked on each post — its visible edge, no correction (user decision 2026-09-24).**
  Neither side can see the centre of a post: GripperX's LiDAR sees only the face turned towards the
  robot, the drone sees the post from above. So each side marks the post **where it sees it**:
  - GripperX clicks **on the LiDAR cluster itself** — the scan points of the post — with no offset
    towards the centre.
  - The drone side marks the **visible edge** of the post in its image, likewise with no offset.
  - Residual error: the two marks for the same post can lie **at most the profile's cross-section
    diagonal** apart (e.g. the drone marks the far edge while the LiDAR sees the near face). This is
    negligible against the drone detector's own position uncertainty (a sigma of 0.5 m, hardcoded
    on the Octopus side — see `gripperx_external/geodesy.py`), so no correction is applied and the
    profile size is not needed for this interface.

## 3. The frame `octopus_line`

```
                      +y  (left of A→B)
                       ^
                       |
      post A  o--------+--------o  post B      ---> +x  (from A towards B)
            x = -L/2   origin   x = +L/2
                     (midpoint)

   seen from ABOVE. z points up, out of the page, towards the viewer.
```

| Item | Definition |
|---|---|
| Origin | Midpoint of the marked points of post A and post B (§2). z = 0. |
| +x | From A towards B. |
| +y | +x turned 90° **counter-clockwise seen from above** = to the **left** of someone standing at A looking at B. |
| +z | Up. The frame is **right-handed** (REP-103 — the ROS convention for coordinate frames). |
| Units | Metres. Angles in radians unless stated; degrees only where labelled `_deg`. |
| Yaw / heading | 0 = pointing from A to B; positive = counter-clockwise seen from above. |
| Scale | 1 unit = 1 metre. Post A is at (−L/2, 0), post B at (+L/2, 0). |
| L | The distance between the two marked post points **as measured by GripperX's LiDAR** (see §5). |

Frame name on the GripperX side: `octopus_line` (ROS transform `map → octopus_line`).

### Pitfall: image coordinates are usually left-handed

Most image conventions put pixel column `u` to the right and row `v` **downwards**. Seen from above
(camera looking down, image not mirrored), that pair is **left-handed**, so a formula copied
straight from pixel space mirrors +y. With `pA`, `pB` the pixel positions of the marked post edges (§2),
`pM = (pA + pB) / 2`, `d = pB − pA`, `ex = d / |d|` (a unit vector in pixels) and
`s = L / |d|` (metres per pixel), a pixel `p` with `dp = p − pM` maps to:

```
x = s · ( dp_u · ex_u + dp_v · ex_v )
y = s · ( dp_u · ex_v − dp_v · ex_u )      # for u right / v DOWN, image seen from above
```

If the image is mirrored or the camera is mounted so that the displayed image is not the view from
above, the sign of `y` flips. **Do not trust the formula — verify the sign** with the zero-motion
check in §7 step 7 at every start.

## 4. What the Octopus sends (goals)

Unchanged topics and payloads (`OCTOPUS_INTERFACE_PROPOSAL.md`). What changes is **which frame the
metres are in**:

- The datum published on `/octopus/fake_eve_gps_start` is, by definition, **the line midpoint**.
  Its lat/lon value may be any fixed value both sides use — it is fake GPS — but it must not be the
  bootstrap fallback, which GripperX refuses.
- Goal and target positions are line-frame metres `(x, y)` expanded with the existing flat-earth
  arithmetic, unchanged:

  ```
  lat = datum_lat + y / 111320
  lon = datum_lon + x / (111320 · cos(datum_lat))
  ```

  Their "x" (the longitude direction of that formula) **is** line +x; their "y" (latitude
  direction) **is** line +y. North/east play no role.
- GripperX inverts exactly that (`y = (lat − datum_lat) · 111320`,
  `x = (lon − datum_lon) · 111320 · cos(datum_lat)`), then applies its own calibrated
  `octopus_line → map` transform.

## 5. Scale: L

GripperX does **not** scale: the LiDAR is metric, so its measured L is taken as the reference
length. After calibration GripperX shows **`L = x.xxx m`** (millimetre precision) as a text marker in
RViz (the ROS 3D viewer) and logs it. **The drone operator enters exactly that value** on the drone
side, where it sets their metres-per-pixel scale (`s` in §3). A new GripperX calibration produces a
new L, which has to be entered again.

GripperX also **sends L and the calibration id in its telemetry** (§6), so the Octopus can compare
the value its operator typed in with ours and notice a new calibration. The manual entry stays: it
is the cross-check, not a formality.

## 5a. Geofence — goals outside it are refused

GripperX only drives to standing poses inside a **square of side L, centred on the line midpoint
and aligned with the line**: x ∈ [−L/2, +L/2], y ∈ [−L/2, +L/2] in `octopus_line` (user decision
2026-09-24). It is derived from the live calibration, so it moves with every recalibration, and
without a calibration there is none and every goal is refused.

What it bounds is the robot's **standing pose** next to the object, not the object itself: an
object just outside the square may still be served when a standing pose inside exists, and one
just inside may be refused when none does. It bounds the goal, not the path.

**Consequence for the Octopus:** a goal with no standing pose inside the square is refused at
validation — it never becomes a mission and is never acknowledged. Because the Octopus protocol
only advances on `trash_goal_done` and has no failure channel, such a goal **blocks the Octopus
mission** until it disappears from the target list. Keep detections inside the square, or bound
the detector to it.

## 6. What GripperX sends back (telemetry)

On `/octopus/devices/gripperx/status` (payload: `OCTOPUS_INTERFACE_PROPOSAL.md`):

- `pose.x`, `pose.y` — the robot's position **in the line frame**, metres.
- `pose.yaw_deg` — heading in the line frame, degrees, 0 = along A→B, counter-clockwise positive.
- `pose.lat`, `pose.lon` — the same position through the same flat-earth formula (§4).
- `line_calibration` — `{"status", "reason", "id", "length_m"}`: the live calibration's id and its
  length L, for comparison with the value the drone operator entered (§5).
- Without a valid line calibration the pose is reported `"status": "unavailable"`,
  `"reason": "NO_LINE_CALIBRATION"`, values `null`, and `line_calibration.status` is
  `"unavailable"` — never robot-map coordinates.
- Reason fields (e.g. `nav_state_reason`) carry a machine-readable code only, never coordinates.

## 7. Procedure — at EVERY start of either side

Nothing is stored across restarts. The calibration is repeated each time.

**GripperX side**

1. Start the robot stack (bringup → mapping → navigation) and the external link
   (`octopus_link.launch.py`). The calibration node is started by the **mapping** service
   (`gripperx-mapping` → `gripperx_localization/launch/localization.launch.py`), next to
   slam_toolbox, so it lives and dies with the SLAM map it is clicked in.
2. Open RViz with `gripperx_external/rviz/octopus_goals.rviz` (fixed frame `map`). The LiDAR scan
   is shown; each post is a small cluster.
3. Select **Publish Point**. Click **on the scan cluster of post A**, then **of post B** (§2).
4. Read **L** from the text marker or the log line `LINE CALIBRATED ... ENTER L = x.xxx m`.
   **GripperX refuses any pair whose L lies outside 2.20–2.80 m** (the user-stated range for the post
   spacing — a plausibility bound, not a measurement; changed 2026-09-24 during the first real-robot
   test from ~~2.5–3.0 m~~): no calibration, and the same
   line in red in RViz and in the log:
   `REJECTED (LENGTH_OUT_OF_RANGE): L = x.xxx m, expected 2.200-2.800 m - click A and B again`.
   That catches a wrong post or a stray click; the operator then clicks A and B again.
5. Tell the drone operator L. To redo: click A and B again (the old calibration is dropped at the
   first click), or call `/gripperx/external/line_calibration_node/reset`.

**Octopus side**

6. Mark the visible edges of post A and post B in the image (§2), enter L, build the frame of §3, and publish
   the datum and goals as in §4.

**Both**

7. **Zero-motion cross-check** before anything is armed: compare GripperX's reported `pose.x/y`
   (§6) with where the drone sees the robot. Both signs must agree (robot left of A→B ⇒ y > 0 on
   both sides); a sign mismatch means A/B are swapped or +y is mirrored (§3). Accepted tolerance:
   `TO-VERIFY`.

## 8. Restarts and recalibration — fail closed

| Event | GripperX behaviour |
|---|---|
| No calibration yet | Every goal refused (`NO_LINE_CALIBRATION`), visible in logs, `/diagnostics` and goal status. |
| Clicked pair with L outside 2.20–2.80 m | Refused; no calibration; red `REJECTED` text in RViz; re-click. |
| GripperX operator starts a new pair (first click) or resets | Calibration dropped immediately; a goal in flight is **cancelled**; nothing dispatched until the pair is complete. |
| GripperX SLAM restart (its `map` frame is re-created) | The calibration node is restarted with it (same service). Independently, the gateway sees a **different** `/map` publisher: calibration dropped, goal in flight **cancelled**, re-click required. |
| `/map` publisher **not visible** (SLAM stopped, or a discovery gap) | No new goal is dispatched at once. Once the absence outlasts a grace time it counts as a loss (dropped, goal in flight **cancelled**). The grace time is `TO-VERIFY`; while it is, any absence counts at once for a goal in flight, and the calibration node drops the calibration at once. In practice this drops the calibration on the GripperX side as well, so a **re-click is required**. |
| GripperX calibration node stops or crashes while SLAM keeps running | The gateway **keeps the last calibration and keeps accepting goals** — the geometry is unchanged. A re-click or reset has **no effect** until the node is running again. User decision 2026-09-24; stricter handling is an open point on the GripperX side (OP-40). |
| Octopus frame re-lock reported on `/octopus/flight_camera_transform/status` | Goal in flight **cancelled** (unchanged existing behaviour). |
| Octopus datum moves | Goal in flight **cancelled** (unchanged existing behaviour). |
| **Octopus-side recalibration (new A/B marks, new L)** | **Not observable by GripperX today.** Open point for both teams: signal it, e.g. by a re-lock on the transform status or a datum republish. |

None of these events arms or disarms anything; arming stays an explicit operator act on the
GripperX side.

## 9. Not yet verified

- Clicking accuracy on the LiDAR cluster, and the resulting error in L and in the line's angle —
  `TO-VERIFY`.
- The accepted tolerance for the zero-motion cross-check (§7 step 7) — `TO-VERIFY`.
- The post spacing itself is not measured. GripperX accepts L only within **2.20–2.80 m** (~~2.5–3.0 m~~ until the first real-robot test), the
  user-stated range (2026-09-24), and refuses outside it; the range is a plausibility bound, not a
  measured value.
- Whether the drone image is mirrored (sign of `y` in §3) — to be established on the real system.
- A slam_toolbox map **reset without a process restart** is not detected by GripperX (§8 relies on
  the `/map` publisher changing); until that is closed, do not reset the map of a running SLAM
  session while a calibration is in use.
- The grace time for an invisible `/map` publisher (§8) — `TO-VERIFY`.
- End-to-end run against the real Octopus: not done.
