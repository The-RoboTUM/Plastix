"""The ordered validation pipeline for incoming external goals.

A pure ordered function. FIRST FAILURE WINS, nothing is partially dispatched,
and every verdict is a machine-readable reason code - rejections are logged,
counted on ``/diagnostics`` (SR-13) and surfaced in telemetry, and a
free-text-only rejection is useless for all three.

Order (:func:`validate_goal`), exactly as designed:

 1. ``NavSatFix`` well-formed and ``status >= 0``
 2. datum present and NOT the Octopus bootstrap fallback
 3. lat/lon finite and in range
 4. staleness of ``header.stamp``
 5. duplicate of the current goal id (idempotent, not re-dispatched)
 6. convert to map metres
 7. grasp offset configured
 8. approach candidate found
 9. chosen pose inside the geofence
10. TF ``map -> base_footprint`` fresh
11. chosen pose inside the global costmap
12. cell cost below ``max_goal_cost``

Two notes on that order, both deliberate:

* Step 8 needs the robot pose to seed the approach bearing, i.e. it depends on
  the TF freshness that step 10 formally checks. Rather than silently reordering
  the designed pipeline, step 8 carries the dependency as its own precondition
  and returns the *same* ``TF_UNAVAILABLE`` reason step 10 would - so the
  verdict is identical either way and only the position in the list differs.
* Steps 9/11/12 are evaluated on the CHOSEN ROBOT POSE, not on the object. The
  acceptance predicate handed to the approach ring in step 8 is built from the
  very same four checks (:func:`check_pose`), so a ring failure and a
  post-condition failure can never disagree - and step 8 reports their specific
  reason whenever every heading failed for the same one, keeping
  ``NO_APPROACH_CANDIDATE`` for the genuinely mixed case. The post-conditions
  remain as an assertion that the chosen pose really is dispatchable, which is
  also the check :func:`validate_dispatch` repeats later against a moved world.

Unknown costmap cells (``-1``) are ACCEPTED WITH A WARNING. This matches
``GridBased.allow_unknown: true`` in ``nav2.yaml`` and the fact that the default
twin mode is ``localization:=slam``, where the costmap only exists where the
robot has already driven. Rejecting unknown space would make the twin's normal
mode unusable.

Pure module: no rclpy, no TF, no costmap. Everything world-facing arrives as an
injected callable or a plain value.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from .geodesy import DatumTracker, GeodesyError, latlon_to_map
from .grasp import (
    ApproachResult,
    Candidate,
    CandidateVerdict,
    GraspOffset,
    GraspOffsetNotConfigured,
    choose_approach,
)

# --- verdicts --------------------------------------------------------------
VERDICT_ACCEPTED = "ACCEPTED"
VERDICT_REJECTED = "REJECTED"
#: Same id as the goal already in flight. Not an error and not a rejection: the
#: Octopus republishes its current goal at 1 Hz, so this is the normal case and
#: must be idempotent.
VERDICT_DUPLICATE = "DUPLICATE"
#: Valid but deliberately not dispatched - disarmed or dry_run. A separate
#: verdict because it must not be counted or logged as a rejection: "disarmed"
#: is the correct, designed state, and burying it among errors would train the
#: operator to ignore the rejection counter.
VERDICT_PREVIEW = "PREVIEW"

# --- severities ------------------------------------------------------------
#: Caused by the peer or the world: log WARN.
SEVERITY_CLIENT = "CLIENT"
#: Caused by our own stack: log ERROR *plus* an ERROR-level DiagnosticStatus.
#: SR-13 requires an active signal, not an absence.
SEVERITY_LOCAL = "LOCAL"

# --- reason codes ----------------------------------------------------------
MALFORMED_FIX = "MALFORMED_FIX"
BAD_FIX_STATUS = "BAD_FIX_STATUS"
NO_DATUM = "NO_DATUM"
BOOTSTRAP_FALLBACK_DATUM = "BOOTSTRAP_FALLBACK_DATUM"
LATLON_NOT_FINITE = "LATLON_NOT_FINITE"
LATLON_OUT_OF_RANGE = "LATLON_OUT_OF_RANGE"
STALE_STAMP = "STALE_STAMP"
NO_STAMP = "NO_STAMP"
CONVERSION_FAILED = "CONVERSION_FAILED"
GRASP_OFFSET_NOT_CONFIGURED = "GRASP_OFFSET_NOT_CONFIGURED"
#: The geofence rectangle is still TO-VERIFY. Raised by the caller BEFORE the
#: pipeline runs: with no measured area there is nothing to validate against,
#: and defaulting to some invented rectangle would make every verdict a guess
#: (FR-12 item 6). Counted as a LOCAL failure because it is our configuration
#: that is missing, not the peer's payload that is wrong.
GEOFENCE_NOT_CONFIGURED = "GEOFENCE_NOT_CONFIGURED"
NO_APPROACH_CANDIDATE = "NO_APPROACH_CANDIDATE"
OUTSIDE_GEOFENCE = "OUTSIDE_GEOFENCE"
TF_UNAVAILABLE = "TF_UNAVAILABLE"
OUTSIDE_COSTMAP = "OUTSIDE_COSTMAP"
COST_TOO_HIGH = "COST_TOO_HIGH"
PATH_NOT_FOUND = "PATH_NOT_FOUND"
BLACKLISTED = "BLACKLISTED"
# --- goal/target correlation (SAFETY.md F-6, C-7) ---------------------------
#: `/octopus/trash_goal` is a bare NavSatFix with no id, so the id it refers to
#: is recovered by position from `/octopus/trash_gps`. These three are the ways
#: that recovery can fail, and each of them BLOCKS the dispatch - not only the
#: acknowledgement. Driving to a goal we could not name would mean arriving with
#: no id to acknowledge and no way to report which object we are standing at.
#: See `correlation.py` for why ambiguity is a refusal rather than a choice.
GOAL_NOT_CORRELATED = "GOAL_NOT_CORRELATED"
GOAL_AMBIGUOUS = "GOAL_AMBIGUOUS"
GOAL_ID_MISMATCH = "GOAL_ID_MISMATCH"
#: The target list has not arrived yet; nothing is wrong, we just cannot name
#: the goal yet.
NO_TARGET_LIST = "NO_TARGET_LIST"
#: A target list exists but has stopped being refreshed, so it is no longer
#: evidence about where the objects are (SAFETY.md F-28). LOCAL severity, with
#: LINK_LOST: an input channel that died is our stack's problem to notice, and
#: it must not be reported as the peer sending us something wrong.
TARGET_LIST_STALE = "TARGET_LIST_STALE"
# dispatch-time only
NOT_ARMED = "NOT_ARMED"
DRY_RUN = "DRY_RUN"
LINK_LOST = "LINK_LOST"
MODE_NOT_AUTONOMOUS = "MODE_NOT_AUTONOMOUS"
MODE_STALE = "MODE_STALE"
NAV2_UNAVAILABLE = "NAV2_UNAVAILABLE"
DATUM_CHANGED = "DATUM_CHANGED"
INTERNAL_ERROR = "INTERNAL_ERROR"

#: Our-side failures. Everything else is the peer's or the world's.
_LOCAL_REASONS = frozenset(
    {
        NAV2_UNAVAILABLE,
        TF_UNAVAILABLE,
        LINK_LOST,
        TARGET_LIST_STALE,
        CONVERSION_FAILED,
        INTERNAL_ERROR,
        GEOFENCE_NOT_CONFIGURED,
    }
)

#: Warning emitted when the chosen pose sits on an unknown costmap cell.
WARN_UNKNOWN_COSTMAP_CELL = "UNKNOWN_COSTMAP_CELL"

#: Costmap sentinel for "unknown".
COST_UNKNOWN = -1


def severity_of(reason: str) -> str:
    return SEVERITY_LOCAL if reason in _LOCAL_REASONS else SEVERITY_CLIENT


# ---------------------------------------------------------------------------
# inputs
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class IncomingGoal:
    """One goal as it arrived, before any interpretation.

    ``well_formed=False`` is how a payload that failed to parse enters the
    pipeline: the parse error is a verdict like any other, so it goes through
    the same reporting path instead of being swallowed by an exception handler.
    """

    target_id: str
    latitude_deg: float
    longitude_deg: float
    status: int = 0
    stamp_sec: Optional[float] = None
    confidence: Optional[float] = None
    well_formed: bool = True
    malformed_detail: str = ""


#: ``(x, y) -> bool``
GeofenceFn = Callable[[float, float], bool]
#: ``(x, y) -> Optional[int]``; ``None`` = outside the costmap,
#: :data:`COST_UNKNOWN` = unknown cell, else 0..254.
CostmapFn = Callable[[float, float], Optional[int]]
#: ``(x, y, yaw) -> bool``; only called when ``verify_path`` is on.
PathCheckFn = Callable[[float, float, float], bool]


@dataclass
class ValidationContext:
    """Everything the pipeline is allowed to know."""

    goal: IncomingGoal
    now_sec: float
    datum_tracker: DatumTracker
    grasp_offset: GraspOffset

    #: ``(x, y, yaw)`` of ``base_footprint`` in ``map``, or ``None`` when TF is
    #: unavailable.
    robot_pose: Optional[Tuple[float, float, float]] = None
    #: Age of that TF lookup in seconds; ``None`` counts as unavailable.
    robot_pose_age_sec: Optional[float] = None
    max_tf_age_sec: float = 1.0

    max_stamp_age_sec: float = 5.0
    #: Set ``False`` for the Octopus's hand-built messages if their stamp turns
    #: out to be unset; a missing stamp then warns instead of rejecting.
    require_stamp: bool = True

    #: Id of the goal currently in flight, for the duplicate check.
    current_goal_id: Optional[str] = None
    #: Ids that exhausted ``max_attempts_per_target``. Deliberately NOT
    #: acknowledged to the Octopus - see build_goal_done().
    blacklisted_ids: Sequence[str] = ()

    geofence: Optional[GeofenceFn] = None
    costmap_cost: Optional[CostmapFn] = None
    max_goal_cost: int = 200
    path_check: Optional[PathCheckFn] = None

    approach_candidates: int = 12
    #: Operator override. When set, the ring is reduced to this single heading -
    #: an override means "approach from here", not "prefer here".
    approach_theta_override: Optional[float] = None


@dataclass
class ValidationResult:
    verdict: str
    reason: str = ""
    detail: str = ""
    severity: str = SEVERITY_CLIENT
    warnings: List[str] = field(default_factory=list)
    object_xy: Optional[Tuple[float, float]] = None
    robot_pose: Optional[Tuple[float, float, float]] = None
    approach: Optional[ApproachResult] = None

    @property
    def accepted(self) -> bool:
        return self.verdict == VERDICT_ACCEPTED


def _reject(reason: str, detail: str = "", **extra) -> ValidationResult:
    return ValidationResult(
        verdict=VERDICT_REJECTED,
        reason=reason,
        detail=detail,
        severity=severity_of(reason),
        **extra,
    )


def _preview(
    reason: str, detail: str, pose: Optional[Tuple[float, float, float]] = None
) -> ValidationResult:
    return ValidationResult(
        verdict=VERDICT_PREVIEW, reason=reason, detail=detail, robot_pose=pose
    )


# ---------------------------------------------------------------------------
# the pose acceptance predicate, shared by the ring and steps 9/11/12
# ---------------------------------------------------------------------------
def check_pose(ctx: ValidationContext, x: float, y: float, yaw: float) -> CandidateVerdict:
    """Geofence, costmap presence, cell cost and optional path reachability.

    Single definition on purpose: the approach ring in step 8 and the
    post-conditions in steps 9/11/12 must apply identical criteria, or a goal
    could pass the ring and then be rejected by its own post-condition.
    """
    warnings: List[str] = []

    if ctx.geofence is not None and not ctx.geofence(x, y):
        return CandidateVerdict(False, OUTSIDE_GEOFENCE, f"pose ({x:.3f}, {y:.3f})")

    if ctx.costmap_cost is not None:
        cost = ctx.costmap_cost(x, y)
        if cost is None:
            return CandidateVerdict(False, OUTSIDE_COSTMAP, f"pose ({x:.3f}, {y:.3f})")
        if cost == COST_UNKNOWN:
            # Accepted with a warning: nav2.yaml has GridBased.allow_unknown
            # true, and in the default twin mode (slam) the costmap only exists
            # where the robot has already driven.
            warnings.append(WARN_UNKNOWN_COSTMAP_CELL)
        elif cost > ctx.max_goal_cost:
            return CandidateVerdict(
                False, COST_TOO_HIGH, f"cost {cost} > max_goal_cost {ctx.max_goal_cost}"
            )

    if ctx.path_check is not None and not ctx.path_check(x, y, yaw):
        return CandidateVerdict(False, PATH_NOT_FOUND, f"pose ({x:.3f}, {y:.3f}, {yaw:.3f})")

    return CandidateVerdict(True, warnings=tuple(warnings))


def make_pose_acceptor(ctx: ValidationContext) -> Callable[[Candidate], CandidateVerdict]:
    def accept(candidate: Candidate) -> CandidateVerdict:
        return check_pose(ctx, candidate.robot_x, candidate.robot_y, candidate.theta)

    return accept


# ---------------------------------------------------------------------------
# the pipeline
# ---------------------------------------------------------------------------
def validate_goal(ctx: ValidationContext) -> ValidationResult:
    """Run the ordered pipeline. First failure wins."""
    goal = ctx.goal

    # 1 -- well-formed and status >= 0
    if not goal.well_formed:
        return _reject(MALFORMED_FIX, goal.malformed_detail or "payload did not parse")
    if goal.status < 0:
        return _reject(BAD_FIX_STATUS, f"NavSatFix.status = {goal.status} (NO_FIX)")

    # 2 -- datum present and not the bootstrap fallback
    blocker = ctx.datum_tracker.dispatch_blocker()
    if blocker == "NO_DATUM":
        return _reject(NO_DATUM, "no datum received and no fallback configured")
    if blocker == "BOOTSTRAP_FALLBACK_DATUM":
        return _reject(
            BOOTSTRAP_FALLBACK_DATUM,
            "datum is the Octopus Garching bootstrap value; the dashboard has "
            "not supplied a real Eve position, so the coordinates mean nothing",
        )
    if blocker:
        return _reject(blocker, "datum unusable")
    datum = ctx.datum_tracker.datum
    assert datum is not None

    # 3 -- lat/lon finite and in range (delegated to the geodesy module, which
    #      owns the definition of a usable coordinate)
    try:
        object_x, object_y = latlon_to_map(datum, goal.latitude_deg, goal.longitude_deg)
    except GeodesyError as exc:
        reason = exc.reason if exc.reason in (LATLON_NOT_FINITE, LATLON_OUT_OF_RANGE) else CONVERSION_FAILED
        return _reject(reason, exc.detail)

    # 4 -- staleness of header.stamp
    if goal.stamp_sec is None:
        if ctx.require_stamp:
            return _reject(NO_STAMP, "NavSatFix header.stamp is unset")
    else:
        age = ctx.now_sec - goal.stamp_sec
        if age > ctx.max_stamp_age_sec:
            return _reject(
                STALE_STAMP, f"stamp age {age:.2f}s > max {ctx.max_stamp_age_sec:.2f}s"
            )

    # 5 -- duplicate of the goal in flight. Their node republishes the current
    #      goal at 1 Hz, so this is the common case, not an anomaly.
    if ctx.current_goal_id is not None and goal.target_id == ctx.current_goal_id:
        return ValidationResult(
            verdict=VERDICT_DUPLICATE,
            reason=VERDICT_DUPLICATE,
            detail=f"target {goal.target_id} is already in flight",
            object_xy=(object_x, object_y),
        )
    if goal.target_id in tuple(ctx.blacklisted_ids):
        return _reject(
            BLACKLISTED,
            f"target {goal.target_id} exhausted its attempts; not acknowledged, "
            "so the Octopus mission is stalled by design until this is resolved",
            object_xy=(object_x, object_y),
        )

    # 6 -- conversion already done above (step 3 and step 6 are the same
    #      arithmetic; splitting them would mean converting twice).
    warnings: List[str] = []

    # 7 -- grasp offset configured. No placeholder exists, so this fires until
    #      the bench measurement is done. `configured` covers offset_x/y only:
    #      grasp.tolerance_m gates the post-arrival reached check and nothing
    #      here (user decision 2026-08-19).
    if not ctx.grasp_offset.configured:
        return _reject(
            GRASP_OFFSET_NOT_CONFIGURED,
            "grasp.offset_x_m / offset_y_m are TO-VERIFY (the tolerance gates "
            "only the reached check, not resolution - user decision 2026-08-19)",
            object_xy=(object_x, object_y),
        )

    # 8 -- approach candidate found. Precondition: a fresh robot pose, because
    #      the seed heading is the bearing from the robot to the object. Returns
    #      the same reason step 10 would, so the verdict does not depend on
    #      where in the list the TF check sits.
    tf_failure = _check_tf(ctx)
    if tf_failure is not None:
        return _reject(tf_failure[0], tf_failure[1], object_xy=(object_x, object_y))
    assert ctx.robot_pose is not None

    seed = ctx.approach_theta_override
    # An override is an instruction, not a preference: no ring walk around it.
    candidate_count = 1 if seed is not None else max(1, ctx.approach_candidates)
    try:
        approach = choose_approach(
            object_xy=(object_x, object_y),
            robot_xy=(ctx.robot_pose[0], ctx.robot_pose[1]),
            offset=ctx.grasp_offset,
            accept=make_pose_acceptor(ctx),
            candidate_count=candidate_count,
            seed_theta=seed,
        )
    except GraspOffsetNotConfigured as exc:
        return _reject(GRASP_OFFSET_NOT_CONFIGURED, str(exc), object_xy=(object_x, object_y))

    warnings.extend(approach.warnings)
    if not approach.ok:
        reasons = sorted(set(approach.failure_reasons()))
        # When every heading failed for the SAME reason there is no ambiguity
        # about what is wrong, so report that reason (OUTSIDE_GEOFENCE,
        # OUTSIDE_COSTMAP, COST_TOO_HIGH, PATH_NOT_FOUND) rather than the
        # generic one - "no approach found" tells an operator nothing. This is
        # also the path an operator heading override takes, since an override
        # reduces the ring to a single candidate.
        # NO_APPROACH_CANDIDATE is reserved for the genuinely mixed case: each
        # heading was blocked by something different, i.e. the object is boxed
        # in rather than one constraint being violated.
        reason = reasons[0] if len(reasons) == 1 else NO_APPROACH_CANDIDATE
        return _reject(
            reason,
            f"{len(approach.evaluated)} heading(s) examined, all refused: {', '.join(reasons)}",
            object_xy=(object_x, object_y),
            approach=approach,
        )
    chosen = approach.chosen
    assert chosen is not None
    pose = (chosen.robot_x, chosen.robot_y, chosen.theta)

    # 9 / 11 / 12 -- post-conditions on the chosen pose, in the designed order.
    #                Same predicate as the ring; they produce the specific
    #                reason when an override reduced the ring to one heading.
    verdict = check_pose(ctx, pose[0], pose[1], pose[2])
    if not verdict.accepted:
        return _reject(
            verdict.reason,
            verdict.detail,
            object_xy=(object_x, object_y),
            robot_pose=pose,
            approach=approach,
        )

    # 10 -- TF freshness, formally. Re-checked here because the pose search and
    #       the path check above can take time.
    tf_failure = _check_tf(ctx)
    if tf_failure is not None:
        return _reject(
            tf_failure[0], tf_failure[1], object_xy=(object_x, object_y), robot_pose=pose
        )

    for warning in verdict.warnings:
        if warning not in warnings:
            warnings.append(warning)

    return ValidationResult(
        verdict=VERDICT_ACCEPTED,
        warnings=warnings,
        object_xy=(object_x, object_y),
        robot_pose=pose,
        approach=approach,
    )


def _check_tf(ctx: ValidationContext) -> Optional[Tuple[str, str]]:
    if ctx.robot_pose is None:
        return TF_UNAVAILABLE, "no map -> base_footprint transform"
    if ctx.robot_pose_age_sec is None:
        return TF_UNAVAILABLE, "transform age unknown"
    if ctx.robot_pose_age_sec > ctx.max_tf_age_sec:
        return (
            TF_UNAVAILABLE,
            f"map -> base_footprint is {ctx.robot_pose_age_sec:.2f}s old "
            f"> max {ctx.max_tf_age_sec:.2f}s",
        )
    return None


# ---------------------------------------------------------------------------
# dispatch-time re-validation
# ---------------------------------------------------------------------------
@dataclass
class DispatchContext:
    """State re-checked immediately before handing a goal to Nav2.

    A validated goal can sit while the world moves, so acceptance is not a
    licence to dispatch later. Note what is deliberately absent: nothing here
    can be satisfied by the external system - arming and the teleop mode are
    both operator-owned (SR-1, two-layer gate).
    """

    armed: bool
    dry_run: bool
    link_alive: bool
    teleop_mode: str
    teleop_mode_age_sec: Optional[float]
    nav2_available: bool
    #: Datum in force now vs. when the goal was validated.
    datum_unchanged: bool
    #: ``(x, y, yaw)`` of the resolved goal, re-checked against the costmap.
    pose: Tuple[float, float, float]
    max_teleop_mode_age_sec: float = 2.0
    required_teleop_mode: str = "autonomous"


def validate_dispatch(
    dctx: DispatchContext, ctx: ValidationContext
) -> ValidationResult:
    """Second gate. First failure wins, same reason vocabulary."""
    if not dctx.armed:
        return _preview(NOT_ARMED, "arming gate is closed (default state)", dctx.pose)
    if dctx.dry_run:
        return _preview(DRY_RUN, "dry_run is on; goal validated and previewed only", dctx.pose)
    if not dctx.link_alive:
        return _reject(LINK_LOST, "external link is down")
    if dctx.teleop_mode != dctx.required_teleop_mode:
        return _reject(
            MODE_NOT_AUTONOMOUS,
            f"teleop mode is '{dctx.teleop_mode}', need '{dctx.required_teleop_mode}'",
        )
    if dctx.teleop_mode_age_sec is None or dctx.teleop_mode_age_sec > dctx.max_teleop_mode_age_sec:
        return _reject(
            MODE_STALE,
            f"/teleop/active_mode age {dctx.teleop_mode_age_sec} > "
            f"{dctx.max_teleop_mode_age_sec:.2f}s",
        )
    if not dctx.nav2_available:
        return _reject(NAV2_UNAVAILABLE, "navigate_to_pose action server not available")
    if not dctx.datum_unchanged:
        return _reject(
            DATUM_CHANGED,
            "the datum moved since validation; the pose no longer means what it did",
        )

    verdict = check_pose(ctx, dctx.pose[0], dctx.pose[1], dctx.pose[2])
    if not verdict.accepted:
        return _reject(verdict.reason, verdict.detail, robot_pose=dctx.pose)

    return ValidationResult(
        verdict=VERDICT_ACCEPTED,
        warnings=list(verdict.warnings),
        robot_pose=dctx.pose,
    )
