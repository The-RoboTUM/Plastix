#!/usr/bin/env python3
"""Replays the real Octopus contract over the mock rosbridge. Pure python.

    python3 src/gripperx_external/test/fake_octopus.py --port 9090

One process = the mock rosbridge server (``mock_rosbridge_server.py``) plus the two
Octopus producer nodes behind it. Rollout stages 0-3 therefore need neither their
machine nor a network. Once their stack is reachable, a recorded rosbag of the real
topics replaces this file and the same tests re-run.

WHAT IT REPRODUCES, AND WHY EACH ONE IS HERE
============================================
These are behaviours of the counterpart verified against branch tip ``a7ab8e6278``
and re-verified on 2026-08-18. A fake that smooths any of them over would let us
pass stage 0 and fail stage 4.

1. **Goal publishing stops entirely when nothing is open.** ``/octopus/trash_goal``
   simply goes silent - there is no "done" message. Only ``open_count`` in
   ``/octopus/trash_gps`` distinguishes *mission complete* from *the source died*.
   Anything that treats silence as an error, or as completion, breaks here.
2. **A datum move republishes every target immediately.** Their change threshold is
   ``1e-9`` degrees, i.e. any drag of the Eve marker on the dashboard map at all, so
   the burst is the normal case rather than an edge case. All targets are stored in
   map metres and converted at publish time, which is why the whole list moves.
3. **Ids restart at 1 on a restart**, and every ``collected`` flag is lost with them.
   A restart can hand us an id we have already collected. This is proposal item 3;
   until they adopt it, our side has to survive it.
4. **``confidence`` may be ``null``** - a real value in their JSON, not a bug.
5. **The goal only advances on ``trash_goal_done``.** There is no failure channel
   (proposal item 2), so an unreachable target stalls the mission permanently. That
   is exactly what ``unreachable`` simulates.

WIRE FORMAT
===========
Emitted through ``octopus_protocol.build_navsatfix`` / ``build_trash_gps`` so that
one module owns the contract. The NavSatFix values are the verified ones:
``header.stamp`` always set, ``status.status = STATUS_FIX``, ``frame_id = "map"``,
covariance ``[0.25,0,0, 0,0.25,0, 0,0,1.0]``, ``position_covariance_type = 1``. JSON
keys are ``lat``/``lon`` - never ``latitude``/``longitude`` - and ``last_seen`` is an
epoch float.

Coordinates use ``geodesy.map_to_latlon``, i.e. our inverse of their flat-earth
expansion with ``cos(datum_lat)`` computed once per datum. Their ``update_datum()``
does the same, so the fake and the code under test agree by construction, and a
round-trip error here would be a real defect rather than a projection artefact.

CONTROL
=======
Interactive commands on stdin (``help`` lists them) and ``--script`` for unattended
runs. The set covers what the stage 0-3 tests need: move the datum, add and remove
targets, make a target unreachable, force a disconnect, hold the port shut, and
restart the id sequence.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))
sys.path.insert(0, _HERE)

from gripperx_external import octopus_protocol as proto  # noqa: E402
from gripperx_external.geodesy import Datum, map_to_latlon  # noqa: E402
from mock_rosbridge_server import MockRosbridgeServer  # noqa: E402

_LOG = logging.getLogger("gripperx_external.fake_octopus")

#: Their bootstrap fallback (Garching). Selectable so the "refuse to dispatch on
#: the fallback datum" path can be exercised.
BOOTSTRAP_LAT = 48.2513611
BOOTSTRAP_LON = 11.6359722

#: A datum that is deliberately NOT the bootstrap fallback, for the normal case.
DEFAULT_DATUM_LAT = 48.2650000
DEFAULT_DATUM_LON = 11.6710000

#: Their working patch is ~4.46 x 3.34 m at 0.10 m resolution. Default targets sit
#: inside it, in map metres relative to the datum.
# THE THREE TARGETS ACTUALLY OBSERVED on their running system 2026-08-21, in
# map metres relative to the datum. The previous defaults - (1.20, 0.80),
# (-0.90, 1.40), (0.30, -1.10) - were invented, and two of the three sit OUTSIDE
# the 1.25 m radius filter their demo stack now applies, so they would be
# dropped at ingest and never appear. Keeping them would have made the fake
# quietly disagree with the counterpart on the very first tick.
DEFAULT_TARGETS_M: Tuple[Tuple[float, float], ...] = (
    (-0.283, 0.032),
    (-0.328, 0.564),
    (0.857, -0.391),
)

# ---------------------------------------------------------------------------
# CAPTURED FROM THE RUNNING OCTOPUS, 2026-08-21, branch `item-a-map-origin`.
# These are observations, not guesses: a probe subscribed to their rosbridge at
# 10.42.0.158:9090 and the frames are archived with the session. Every value
# below replaced a value this fake had invented.
# ---------------------------------------------------------------------------

#: Their detector reports 0.8, not the 0.85 this fake used to make up.
REAL_CONFIDENCE = 0.8
#: Their targets carry a class. We do not read it; it is emitted so we can show
#: that we tolerate it.
REAL_CLASS_NAME = "trash"
#: Their trash_gps envelope names its own producer and frame.
REAL_SOURCE_ID = "trash_gps_goal_node"
REAL_FRAME_ID = "map"

#: `indoor_static_map_yaw_offset_rad` - +pi/2, fixed in their configuration.
TRANSFORM_MAP_YAW_OFFSET_RAD = 1.57079632679
#: `indoor_static_yaw_zero_rad` AS OBSERVED. The drone's PX4 yaw locked at their
#: startup - about -175.9 deg, nearly due south. It is NOT a constant: their own
#: message of the same day quoted -3.06995218682264 and a restart had re-locked
#: it to this value by the time we measured, a 2.27 deg move. That is precisely
#: the event `relock` reproduces.
OBSERVED_YAW_ZERO_RAD = -3.1094807565268194

#: Their `max_radius_m`, set to 1.25 in the indoor demo stack (our number, their
#: parameter). Detections further than this from the datum are dropped at ingest
#: and never offered as targets, with one WARN at most every 10 s.
DEFAULT_MAX_RADIUS_M = 1.25

#: The five keys of their transform payload this fake does not MODEL -
#: per-frame detector internals. Frozen VERBATIM from the capture of
#: 2026-08-21 rather than invented, so the frame is schema-identical to
#: theirs and we can show we tolerate the whole thing. The VALUES are a
#: snapshot and do not track anything; nothing may read them as live.
CAPTURED_TRANSFORM_EXTRAS = json.loads("""{"camera_model": {"normalized_v_origin": "bottom_left", "image_width": 640.0, "image_height": 480.0, "fx": 359.3292231592479, "fy": 359.2290038414162, "cx": 312.8204647201454, "cy": 237.947360594595, "distortion_coefficients": {"k1": -0.057128411511179616, "k2": 0.0028040539388385884, "p1": 0.00015933624483912515, "p2": -0.001408459710522939, "k3": 0.0}, "camera_to_body_rpy_rad": [0.0, 0.0, 1.5707963267948966], "camera_translation_body_m": [0.113, 0.0, 0.022], "ground_z_ned": 0.0, "use_dist_bottom_if_valid": false, "use_manual_height_above_ground": true, "manual_height_above_ground_m": 2.5}, "odometry": {"topic": "/fmu/out/vehicle_odometry", "fresh": true, "age_sec": 0.004132270812988281, "pose_frame": 1, "position": [0.09763315320014954, -0.00518490606918931, 36.89802932739258], "q": [-0.022815745323896408, 0.04058292135596275, 0.039497919380664825, 0.9981344938278198], "position_valid": true, "quaternion_valid": true}, "local_position": {"topic": "/fmu/out/vehicle_local_position", "fresh": false, "age_sec": null, "xy_valid": false, "z_valid": false, "dist_bottom_valid": false, "position_ned": null, "heading": null, "dist_bottom": null}, "output_note": "Flight projection node. Keep projection_enabled=false until camera intrinsics/extrinsics are verified.", "last_output_points_ned": [[-0.3047194514833486, 0.16640160132449092, 39.42062898424406], [-0.31748312607126605, 0.5742278293327645, 39.42062898424406]]}""")

#: How far away ``unreachable`` puts a target. Their JSON has no reachability
#: field, so "unreachable" can only be expressed as a position our geofence and
#: costmap will refuse - which is exactly how it would present in reality.
DEFAULT_UNREACHABLE_OFFSET_M = 500.0

#: Their datum-change threshold, in degrees. Below it, nothing happens; at or above
#: it, every target is republished at once.
DATUM_CHANGE_THRESHOLD_DEG = 1e-9


@dataclass
class FakeTarget:
    id: int
    x: float
    y: float
    confidence: Optional[float] = 0.85
    collected: bool = False
    last_seen: float = field(default_factory=time.time)


class FakeOctopus:
    """The two Octopus producer nodes, plus their goal-advance state machine."""

    def __init__(
        self,
        server: MockRosbridgeServer,
        *,
        datum_lat: float = DEFAULT_DATUM_LAT,
        datum_lon: float = DEFAULT_DATUM_LON,
        rate_hz: float = 1.0,
        targets_m: Tuple[Tuple[float, float], ...] = DEFAULT_TARGETS_M,
        unreachable_offset_m: float = DEFAULT_UNREACHABLE_OFFSET_M,
        null_confidence_ids: Tuple[int, ...] = (),
        yaw_zero_rad: Optional[float] = OBSERVED_YAW_ZERO_RAD,
        max_radius_m: float = DEFAULT_MAX_RADIUS_M,
        transform_state: str = "ready",
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.server = server
        self.rate_hz = max(0.05, float(rate_hz))
        self.unreachable_offset_m = float(unreachable_offset_m)
        self._log = logger or _LOG

        #: Their startup yaw lock. ``None`` models the window before it exists,
        #: where their own payload carries JSON ``null`` - the case that must
        #: never be read as 0.0.
        self._yaw_zero_rad = yaw_zero_rad
        #: "ready" | "not_ready". Their `state` is "ready" only once the
        #: transform actually is; a consumer must check it before trusting a
        #: heading.
        self._transform_state = transform_state
        self.max_radius_m = float(max_radius_m)
        self.relocks = 0
        self.radius_drops = 0

        self._datum = Datum(latitude_deg=datum_lat, longitude_deg=datum_lon, from_topic=True)
        self._next_id = 1
        self._targets: List[FakeTarget] = []
        self._initial_targets_m = tuple(targets_m)
        self._null_confidence_ids = set(null_confidence_ids)
        for x, y in targets_m:
            self.add_target(x, y)

        # observable counters, asserted by the stage-0 checks
        self.acks_received = 0
        self.unknown_acks = 0
        self.goal_publishes = 0
        self.goal_suppressed_ticks = 0
        self.datum_bursts = 0
        self.restarts = 0
        #: SAFETY.md F-28's verification path: their DETECTOR dies while their
        #: datum publisher lives. `trash_goal` and the datum keep flowing, so
        #: the consumer's link watchdog - which measures the last frame of ANY
        #: topic - still reports a healthy link, while the target list the
        #: correlation depends on quietly stops being refreshed. Nothing in this
        #: fake could produce that case before; `silence gps` does.
        self.silenced_topics: set = set()
        self.silenced_ticks = 0

        self.server.add_local_subscriber(proto.TOPIC_TRASH_GOAL_DONE, self._on_goal_done)

    # -- target bookkeeping ------------------------------------------------
    def add_target(
        self, x: float, y: float, confidence: Optional[float] = REAL_CONFIDENCE
    ) -> Optional[FakeTarget]:
        """Ingest one detection. ``None`` when their radius filter drops it.

        THE FILTER IS AT INGEST, where theirs is - beside `min_confidence`, so
        it reads as "within N metres of the datum". A dropped detection never
        becomes a target and never reaches `trash_gps`, which is why it cannot
        deadlock us. `unreachable` deliberately BYPASSES this: it moves a target
        that already exists, modelling a different failure (a target we cannot
        reach) that predates the filter and is still worth exercising.
        """
        radius = math.hypot(float(x), float(y))
        if self.max_radius_m > 0.0 and radius > self.max_radius_m:
            self.radius_drops += 1
            self._log.warning(
                "Detection at map (%.2f, %.2f) is %.2f m from the datum, outside "
                "max_radius_m=%.2f. Not offered as a target. %d dropped so far.",
                x, y, radius, self.max_radius_m, self.radius_drops,
            )
            return None
        target = FakeTarget(id=self._next_id, x=float(x), y=float(y), confidence=confidence)
        if target.id in self._null_confidence_ids:
            target.confidence = None
        self._next_id += 1
        self._targets.append(target)
        self._log.info("target %d added at map (%.2f, %.2f)", target.id, target.x, target.y)
        return target

    def remove_target(self, target_id: int) -> bool:
        before = len(self._targets)
        self._targets = [t for t in self._targets if t.id != target_id]
        removed = len(self._targets) != before
        if removed:
            self._log.info("target %d removed", target_id)
        return removed

    def make_unreachable(self, target_id: int, distance_m: Optional[float] = None) -> bool:
        """Move a target far outside the working patch.

        Their protocol cannot express "this one is impossible", so this is what an
        unreachable target actually looks like on the wire: a perfectly well-formed
        goal at a position nothing can drive to. The mission then stalls, because
        the goal only advances on an acknowledgement we must not send.
        """
        distance = self.unreachable_offset_m if distance_m is None else float(distance_m)
        for target in self._targets:
            if target.id == target_id:
                target.x, target.y = distance, distance
                self._log.warning(
                    "target %d moved to (%.1f, %.1f) m - unreachable by construction",
                    target_id,
                    target.x,
                    target.y,
                )
                return True
        return False

    def restart_ids(self) -> None:
        """Reproduce a restart of ``trash_gps_goal_node``.

        Ids go back to 1 and every ``collected`` flag is lost, so previously
        collected trash reappears under a familiar id. Proposal item 3.
        """
        self._next_id = 1
        old = self._targets
        self._targets = []
        for target in old:
            self.add_target(target.x, target.y, target.confidence)
        self.restarts += 1
        self._log.warning("id sequence restarted at 1; %d collected flag(s) lost",
                          sum(1 for t in old if t.collected))

    # -- datum -------------------------------------------------------------
    @property
    def datum(self) -> Datum:
        return self._datum

    def move_datum(self, lat: float, lon: float) -> bool:
        """Set a new datum. Returns True when it counted as a change.

        Threshold ``1e-9`` deg, matching theirs - so any drag at all triggers the
        immediate full republish of every target.
        """
        changed = (
            abs(lat - self._datum.latitude_deg) >= DATUM_CHANGE_THRESHOLD_DEG
            or abs(lon - self._datum.longitude_deg) >= DATUM_CHANGE_THRESHOLD_DEG
        )
        self._datum = Datum(latitude_deg=lat, longitude_deg=lon, from_topic=True)
        if changed:
            self.datum_bursts += 1
            self._log.warning(
                "datum moved to (%.7f, %.7f) - republishing all %d target(s) at once",
                lat,
                lon,
                len(self._targets),
            )
            self.publish_all()
        return changed

    def move_datum_m(self, dx: float, dy: float) -> bool:
        """Drag the Eve marker by ``dx``/``dy`` metres in the current expansion."""
        lat, lon = map_to_latlon(self._datum, dx, dy)
        return self.move_datum(lat, lon)

    # -- goal selection ----------------------------------------------------
    def open_targets(self) -> List[FakeTarget]:
        return [t for t in self._targets if not t.collected]

    def current_goal(self) -> Optional[FakeTarget]:
        """``goal_selection: nearest`` - nearest to the *datum*, because they do
        not know where the robot is. Making that better is proposal item 1."""
        open_targets = self.open_targets()
        if not open_targets:
            return None
        return min(open_targets, key=lambda t: math.hypot(t.x, t.y))

    def _on_goal_done(self, topic: str, msg: Any) -> None:
        """Advance on acknowledgement. Accepts a bare id and ``{"id": n}``.

        Their own parser accepts both, so ours must too - the alternative is a
        silent no-op and a mission that never advances.
        """
        try:
            ack_id = proto.parse_goal_done(msg)
        except proto.ProtocolError as exc:
            self.unknown_acks += 1
            self._log.error("unparseable trash_goal_done %r: %s", msg, exc)
            return
        self.acks_received += 1
        for target in self._targets:
            if str(target.id) == ack_id and not target.collected:
                target.collected = True
                self._log.info(
                    "target %s acknowledged as collected; %d open",
                    ack_id,
                    len(self.open_targets()),
                )
                # Immediate, like theirs: the next goal appears on the next tick
                # rather than after a timeout.
                self.publish_all()
                return
        self.unknown_acks += 1
        self._log.warning("trash_goal_done for unknown or already-collected id %s", ack_id)

    # -- publishing --------------------------------------------------------
    def _navsatfix(self, lat: float, lon: float) -> Dict[str, Any]:
        return proto.build_navsatfix(
            lat,
            lon,
            altitude_m=0.0,
            status=proto.STATUS_FIX,
            frame_id=proto.OCTOPUS_FRAME_ID,
            # Always set by both their nodes - never left at zero.
            stamp_sec=time.time(),
            position_covariance=proto.OCTOPUS_POSITION_COVARIANCE,
            position_covariance_type=proto.OCTOPUS_POSITION_COVARIANCE_TYPE,
        )

    def publish_datum(self) -> None:
        """``/octopus/fake_eve_gps_start``: TRANSIENT_LOCAL depth 1 on their side.

        Republished every tick rather than relying on a latch: a rosbridge client
        subscribes VOLATILE and misses the latched replay, and since this topic
        runs at 1 Hz anyway the latch is worth at most one second at startup.
        """
        if "datum" in self.silenced_topics:
            self.silenced_ticks += 1
            return
        self.server.inject_publish(
            proto.TOPIC_DATUM,
            self._navsatfix(self._datum.latitude_deg, self._datum.longitude_deg),
            latch=True,
        )

    def publish_goal(self) -> bool:
        """``/octopus/trash_goal`` - or nothing at all, which is the hard part.

        When no target is open this publishes NOTHING. Not a sentinel, not a
        NO_FIX, not an empty message: the topic simply goes quiet, and
        ``open_count`` on ``trash_gps`` is the only thing that says why.
        """
        if "goal" in self.silenced_topics:
            self.silenced_ticks += 1
            return False
        goal = self.current_goal()
        if goal is None:
            self.goal_suppressed_ticks += 1
            return False
        lat, lon = map_to_latlon(self._datum, goal.x, goal.y)
        self.server.inject_publish(proto.TOPIC_TRASH_GOAL, self._navsatfix(lat, lon), latch=True)
        self.goal_publishes += 1
        return True

    def publish_trash_gps(self) -> None:
        """``/octopus/trash_gps`` - the full picture, including ``open_count``."""
        if "gps" in self.silenced_topics:
            self.silenced_ticks += 1
            return
        goal = self.current_goal()
        datum_info = proto.TrashDatumInfo(
            latitude_deg=self._datum.latitude_deg,
            longitude_deg=self._datum.longitude_deg,
            from_topic=True,
        )
        targets = []
        for target in self._targets:
            lat, lon = map_to_latlon(self._datum, target.x, target.y)
            targets.append(
                proto.TrashTarget(
                    id=str(target.id),
                    latitude_deg=lat,
                    longitude_deg=lon,
                    x=target.x,
                    y=target.y,
                    confidence=target.confidence,
                    collected=target.collected,
                    is_goal=goal is not None and target.id == goal.id,
                    last_seen=target.last_seen,
                )
            )
        payload = proto.build_trash_gps(
            datum=datum_info,
            goal_id=None if goal is None else str(goal.id),
            open_count=len(self.open_targets()),
            targets=targets,
            # The real envelope, captured 2026-08-21. `numeric_ids` matters
            # most: theirs are JSON numbers, ours used to be strings.
            source_id=REAL_SOURCE_ID,
            frame_id=REAL_FRAME_ID,
            timestamp=time.time(),
            datum_xy=(0.0, 0.0),
            class_name=REAL_CLASS_NAME,
            numeric_ids=True,
        )
        self.server.inject_publish(proto.TOPIC_TRASH_GPS, {"data": payload})

    def publish_transform_status(self) -> None:
        """``/octopus/flight_camera_transform/status`` - their startup yaw lock.

        THE FIELD SET IS THEIRS, not a minimal one. We read four keys; the rest
        are emitted because the real publisher emits them, and a fake that sends
        only what the consumer reads cannot show that the consumer survives the
        rest. Their per-frame detector internals (`last_output_points_ned`,
        `camera_model`, `odometry`, `local_position`) are deliberately OMITTED -
        this fake has no model of them and inventing them would be worse than
        leaving them out.
        """
        if "transform" in self.silenced_topics:
            self.silenced_ticks += 1
            return
        locked = self._yaw_zero_rad is not None
        ready = locked and self._transform_state == "ready"
        payload = {
            "mode": "flight_pose_ground_plane",
            "transform_mode": "indoor_static_mission",
            "indoor_static_origin_x": 0.0,
            "indoor_static_origin_y": 0.0,
            "indoor_static_align_yaw_on_start": True,
            "indoor_static_map_yaw_offset_rad": TRANSFORM_MAP_YAW_OFFSET_RAD,
            # JSON null until their yaw is locked. NEVER 0.0 - that would read as
            # "the drone faces east" instead of "we do not know yet".
            "indoor_static_yaw_zero_rad": self._yaw_zero_rad,
            "state": "ready" if ready else "not_ready",
            "transform_ready": ready,
            "pose_ready": ready,
            "local_valid_enough": ready,
            "reason": (
                "projection enabled and fresh pose/local validity available"
                if ready
                else "waiting for a valid local pose to lock the startup yaw"
            ),
            "projection_enabled": True,
            "pose_stale_sec": 10.0,
            "detector_topic": "/detector_node/confirmed",
            "output_topic": "/octopus/detections_world_pose",
            "last_input_detection_count": len(self._targets),
            "last_transformed_detection_count": len(self._targets),
            "last_projection_error": None,
            "backend_received_at": None,
            # Verbatim capture, not a model - see CAPTURED_TRANSFORM_EXTRAS.
            **CAPTURED_TRANSFORM_EXTRAS,
        }
        self.server.inject_publish(
            proto.TOPIC_TRANSFORM_STATUS, {"data": json.dumps(payload)}
        )

    def align_angle_rad(self) -> Optional[float]:
        """What a consumer derives from the two angles. ``None`` while unlocked."""
        if self._yaw_zero_rad is None:
            return None
        return TRANSFORM_MAP_YAW_OFFSET_RAD - self._yaw_zero_rad

    def relock(self, yaw_zero_rad: Optional[float] = None) -> float:
        """Re-lock their startup yaw, as a restart of their transform node does.

        This is the event the whole transform-status subscription exists for: an
        alignment measured before this moment is DEAD, and nothing else in the
        protocol says so. The default moves it by the 2.27 deg actually observed
        between their written message and our own measurement on 2026-08-21.
        """
        previous = self._yaw_zero_rad
        if yaw_zero_rad is None:
            base = previous if previous is not None else OBSERVED_YAW_ZERO_RAD
            yaw_zero_rad = base + math.radians(2.27)
        self._yaw_zero_rad = float(yaw_zero_rad)
        self._transform_state = "ready"
        self.relocks += 1
        self._log.warning(
            "TRANSFORM RE-LOCK #%d: indoor_static_yaw_zero_rad %s -> %.6f rad "
            "(align_angle %s -> %.2f deg). Any their-map -> our-map alignment "
            "measured before now is no longer true.",
            self.relocks,
            "null" if previous is None else f"{previous:.6f}",
            self._yaw_zero_rad,
            "n/a" if previous is None
            else f"{math.degrees(TRANSFORM_MAP_YAW_OFFSET_RAD - previous):.2f}",
            math.degrees(self.align_angle_rad()),
        )
        return self._yaw_zero_rad

    def set_transform_state(self, state: str) -> bool:
        """``ready`` | ``not_ready`` | ``nolock`` (yaw back to JSON null)."""
        if state == "nolock":
            self._yaw_zero_rad = None
            self._transform_state = "not_ready"
            self._log.warning("transform lock CLEARED: yaw_zero is now null")
            return True
        if state in ("ready", "not_ready"):
            self._transform_state = state
            self._log.info("transform state -> %s", state)
            return True
        return False

    def publish_all(self) -> None:
        self.publish_datum()
        self.publish_transform_status()
        self.publish_trash_gps()
        self.publish_goal()

    async def run(self) -> None:
        period = 1.0 / self.rate_hz
        while True:
            self.publish_all()
            await asyncio.sleep(period)

    # -- introspection -----------------------------------------------------
    def status_text(self) -> str:
        goal = self.current_goal()
        lines = [
            f"datum      : {self._datum.latitude_deg:.7f}, {self._datum.longitude_deg:.7f}",
            f"targets    : {len(self._targets)} total, {len(self.open_targets())} open",
            f"goal       : {'none (topic silent)' if goal is None else goal.id}",
            f"acks       : {self.acks_received} ({self.unknown_acks} unknown)",
            f"goal pubs  : {self.goal_publishes} (suppressed ticks {self.goal_suppressed_ticks})",
            f"datum burst: {self.datum_bursts}   id restarts: {self.restarts}",
            f"ws clients : {self.server.client_count} (listening={self.server.listening})",
        ]
        for target in self._targets:
            lines.append(
                f"  #{target.id:<3} map=({target.x:7.2f},{target.y:7.2f}) "
                f"conf={target.confidence} collected={target.collected}"
            )
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# control surface
# ---------------------------------------------------------------------------
HELP_TEXT = """\
commands:
  status                     print datum, targets, goal and counters
  add <x> <y> [conf|null]    add a target at map metres (x, y)
  remove <id>                remove a target
  unreachable <id> [m]       move a target far out of reach (default 500 m)
  move-datum <lat> <lon>     set the datum absolutely -> full republish burst
  move-datum-m <dx> <dy>     drag the datum by metres  -> full republish burst
  bootstrap                  jump to the Garching bootstrap fallback datum
  restart                    restart the id sequence at 1, lose collected flags
  relock [yaw_rad]           re-lock their startup yaw (default: +2.27 deg, as observed)
  transform <ready|not_ready|nolock>
                             transform state; nolock puts yaw_zero back to null
  radius <m>                 their max_radius_m ingest filter (0 = off)
  silence <gps|goal|datum|transform>
                             stop publishing ONE topic; the others keep going
  resume [topic]             publish everything again (no arg = all topics)
  disconnect                 force-close every connected rosbridge client
  outage <sec>               disconnect and refuse connections for <sec>
  help                       this text
  quit                       stop the fake
"""


async def _handle_command(fake: FakeOctopus, line: str) -> bool:
    """Returns False when the fake should stop."""
    parts = line.split()
    if not parts:
        return True
    cmd, args = parts[0].lower(), parts[1:]
    try:
        if cmd in ("quit", "exit"):
            return False
        if cmd == "help":
            print(HELP_TEXT, end="", flush=True)
        elif cmd == "status":
            print(fake.status_text(), flush=True)
        elif cmd == "add":
            conf: Optional[float] = REAL_CONFIDENCE
            if len(args) >= 3:
                conf = None if args[2].lower() in ("null", "none") else float(args[2])
            added = fake.add_target(float(args[0]), float(args[1]), conf)
            if added is None:
                print(
                    f"DROPPED at ingest: outside max_radius_m={fake.max_radius_m:.2f}. "
                    "Their filter did this, not us - it never becomes a target.",
                    flush=True,
                )
            fake.publish_all()
        elif cmd == "remove":
            print("removed" if fake.remove_target(int(args[0])) else "no such id", flush=True)
            fake.publish_all()
        elif cmd == "unreachable":
            distance = float(args[1]) if len(args) > 1 else None
            print("moved" if fake.make_unreachable(int(args[0]), distance) else "no such id",
                  flush=True)
            fake.publish_all()
        elif cmd == "move-datum":
            fake.move_datum(float(args[0]), float(args[1]))
        elif cmd == "move-datum-m":
            fake.move_datum_m(float(args[0]), float(args[1]))
        elif cmd == "bootstrap":
            fake.move_datum(BOOTSTRAP_LAT, BOOTSTRAP_LON)
        elif cmd == "restart":
            fake.restart_ids()
            fake.publish_all()
        elif cmd == "relock":
            yaw = float(args[0]) if args else None
            new_yaw = fake.relock(yaw)
            print(
                f"yaw_zero -> {new_yaw:.9f} rad, align_angle "
                f"{math.degrees(fake.align_angle_rad()):.2f} deg (re-lock #{fake.relocks})",
                flush=True,
            )
            fake.publish_all()
        elif cmd == "transform":
            state = args[0].lower() if args else "ready"
            if not fake.set_transform_state(state):
                print("try: transform ready | not_ready | nolock", flush=True)
            fake.publish_all()
        elif cmd == "radius":
            fake.max_radius_m = float(args[0]) if args else DEFAULT_MAX_RADIUS_M
            print(
                f"max_radius_m = {fake.max_radius_m:.2f} "
                "(applies to NEW detections only, as theirs does)",
                flush=True,
            )
        elif cmd == "silence":
            topic = (args[0].lower() if args else "gps")
            known = ("gps", "goal", "datum", "transform")
            if topic not in known:
                print(f"cannot silence {topic!r}; try {', '.join(known)}", flush=True)
            else:
                fake.silenced_topics.add(topic)
                print(f"silenced {topic}; still publishing "
                      + ", ".join(sorted(set(known) - fake.silenced_topics))
                      + " (the link stays healthy - that is the point)", flush=True)
        elif cmd == "resume":
            if args:
                fake.silenced_topics.discard(args[0].lower())
            else:
                fake.silenced_topics.clear()
            print(f"publishing again; silenced now: "
                  + (", ".join(sorted(fake.silenced_topics)) or "nothing"), flush=True)
            fake.publish_all()
        elif cmd == "disconnect":
            n = await fake.server.drop_all_clients(reason="fake_octopus disconnect command")
            print(f"dropped {n} client(s)", flush=True)
        elif cmd == "outage":
            seconds = float(args[0]) if args else 5.0
            await fake.server.close_listener()
            await fake.server.drop_all_clients(reason="fake_octopus outage")
            print(f"outage for {seconds:.1f} s", flush=True)
            await asyncio.sleep(seconds)
            await fake.server.open_listener()
            print("outage over, listening again", flush=True)
        else:
            print(f"unknown command {cmd!r}; try 'help'", flush=True)
    except (IndexError, ValueError) as exc:
        print(f"bad arguments for {cmd!r}: {exc}", flush=True)
    return True


async def _stdin_loop(fake: FakeOctopus) -> None:
    loop = asyncio.get_running_loop()
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line:  # EOF - run headless from here on
            await asyncio.sleep(3600)
            continue
        if not await _handle_command(fake, line.strip()):
            return


async def _script_loop(fake: FakeOctopus, script: str) -> None:
    """``--script "5:disconnect;12:move-datum-m 1.0 0.0;20:quit"``.

    Offsets are seconds from start, so an unattended test reads as a timeline.
    """
    entries: List[Tuple[float, str]] = []
    for raw in script.split(";"):
        raw = raw.strip()
        if not raw:
            continue
        when, _, command = raw.partition(":")
        entries.append((float(when), command.strip()))
    entries.sort(key=lambda item: item[0])

    start = time.monotonic()
    for when, command in entries:
        delay = start + when - time.monotonic()
        if delay > 0:
            await asyncio.sleep(delay)
        _LOG.info("script t=%.1f: %s", when, command)
        if not await _handle_command(fake, command):
            return
    await asyncio.sleep(3600)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fake Octopus over a mock rosbridge.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument(
        "--datum",
        default=f"{DEFAULT_DATUM_LAT},{DEFAULT_DATUM_LON}",
        help="initial datum as 'lat,lon'",
    )
    parser.add_argument(
        "--bootstrap-datum",
        action="store_true",
        help="start on the Garching bootstrap fallback, so the refusal path can be tested",
    )
    parser.add_argument(
        "--targets",
        default=";".join(f"{x},{y}" for x, y in DEFAULT_TARGETS_M),
        help="initial targets in map metres, 'x,y;x,y;...' ('' for none)",
    )
    parser.add_argument(
        "--null-confidence-ids",
        default="",
        help="comma-separated ids whose confidence is published as null",
    )
    parser.add_argument("--unreachable-offset-m", type=float, default=DEFAULT_UNREACHABLE_OFFSET_M)
    parser.add_argument(
        "--yaw-zero",
        default=str(OBSERVED_YAW_ZERO_RAD),
        help="indoor_static_yaw_zero_rad; 'null' models the pre-lock window",
    )
    parser.add_argument(
        "--max-radius-m",
        type=float,
        default=DEFAULT_MAX_RADIUS_M,
        help="their ingest filter; 0 disables it (default: their demo value 1.25)",
    )
    parser.add_argument(
        "--transform-state",
        default="ready",
        choices=["ready", "not_ready"],
        help="their `state` field; not_ready means a heading must not be trusted",
    )
    parser.add_argument("--script", default="", help="'sec:command;sec:command;...'")
    parser.add_argument("--no-stdin", action="store_true")
    parser.add_argument("--latch-replay", action="store_true", help="see mock_rosbridge_server")
    parser.add_argument("--log-level", default="INFO")
    return parser


def _parse_targets(text: str) -> Tuple[Tuple[float, float], ...]:
    out: List[Tuple[float, float]] = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        x_text, _, y_text = chunk.partition(",")
        out.append((float(x_text), float(y_text)))
    return tuple(out)


async def _main(args: argparse.Namespace) -> int:
    if args.bootstrap_datum:
        datum_lat, datum_lon = BOOTSTRAP_LAT, BOOTSTRAP_LON
    else:
        lat_text, _, lon_text = args.datum.partition(",")
        datum_lat, datum_lon = float(lat_text), float(lon_text)

    null_ids = tuple(
        int(v) for v in args.null_confidence_ids.split(",") if v.strip()
    )

    server = MockRosbridgeServer(args.host, args.port, latch_replay=args.latch_replay)
    await server.start()
    fake = FakeOctopus(
        server,
        datum_lat=datum_lat,
        datum_lon=datum_lon,
        rate_hz=args.rate_hz,
        targets_m=_parse_targets(args.targets),
        unreachable_offset_m=args.unreachable_offset_m,
        null_confidence_ids=null_ids,
        yaw_zero_rad=(None if str(args.yaw_zero).lower() in ('null', 'none')
                      else float(args.yaw_zero)),
        max_radius_m=args.max_radius_m,
        transform_state=args.transform_state,
    )

    print(
        f"fake Octopus on ws://{args.host}:{args.port} - "
        f"{len(fake.open_targets())} open target(s), datum "
        f"{datum_lat:.7f},{datum_lon:.7f}"
        + ("  [BOOTSTRAP FALLBACK]" if args.bootstrap_datum else ""),
        flush=True,
    )
    if not args.no_stdin and not args.script:
        print(HELP_TEXT, end="", flush=True)

    publisher = asyncio.ensure_future(fake.run())
    if args.script:
        controller = asyncio.ensure_future(_script_loop(fake, args.script))
    elif args.no_stdin:
        controller = asyncio.ensure_future(asyncio.sleep(10**9))
    else:
        controller = asyncio.ensure_future(_stdin_loop(fake))

    try:
        await controller
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        publisher.cancel()
        controller.cancel()
        await asyncio.gather(publisher, controller, return_exceptions=True)
        await server.stop()
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )
    try:
        return asyncio.run(_main(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
