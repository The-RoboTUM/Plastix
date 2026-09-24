"""Correlating the dispatched goal fix back to a target id. Pure module.

WHY THIS EXISTS - THE PROTOCOL GIVES THE GOAL NO ID
===================================================
``/octopus/trash_goal`` is a bare ``sensor_msgs/NavSatFix``. It carries a
position and nothing else. The ids live only on ``/octopus/trash_gps``, and
``/octopus/trash_goal_done`` - the one thing that advances their mission - takes
an **id**. So between the goal we drive to and the acknowledgement we send there
is a gap that only a position match can close, and closing it wrongly is
irreversible: their protocol has no un-acknowledge and no failure channel, so a
mis-correlated ack marks an object collected that is still lying there and
advances the mission past it for ever (SAFETY.md F-6, FR-12 item 7).

Asking them for an id on the goal topic is item 2a of the proposal. Until then
this module is the correlation, and its guiding rule is:

    **AMBIGUITY IS A REFUSAL, NOT A CHOICE.**

More than one candidate inside ``tolerance_m`` means we do not know which object
the fix denotes. Picking the nearest would be a guess with a permanent,
unobservable consequence, so :func:`correlate` returns ``AMBIGUOUS`` and the
caller neither dispatches nor acknowledges. This is not a theoretical corner:
their detector merges detections at ``merge_radius_m: 0.25``, i.e. two distinct
targets 0.3 m apart are exactly what their pipeline is built to keep separate,
so near pairs are the expected case rather than the exotic one.

A REFUSAL IS NOT A MOMENT, IT IS A CONDITION THAT MUST HOLD THROUGHOUT
=====================================================================
This function is pure and knows nothing about time, so the rule above is only
worth what its caller's re-evaluation is worth. It was once evaluated exactly
once per mission, at dispatch, and a fix that became ambiguous half a second
later still drove to arrival, still actuated the arm and still acknowledged an
id this module had explicitly refused to supply (SAFETY.md F-13). The gateway
therefore re-runs it against the state as it is at THREE points, on the goal's
own fix: on every dispatch tick while the goal is in flight (cancels), at
arrival before the pick is sent (refuses to actuate), and at the
acknowledgement (refuses to publish). Nothing is inherited from the dispatch
decision. If this module is ever given a new caller, that is the contract it
joins.

A LIST WE STOPPED RECEIVING IS NOT A LIST OF WHAT IS THERE
=========================================================
This function answers "which target does this fix denote, in the list I was
handed". SAFETY.md F-28 is about the *handing*: the gateway kept re-asking the
last list it had received, so a detector that went silent left all three gates
returning the same confident ``UNIQUE`` for minutes, on evidence that had
stopped being evidence. Their link watchdog cannot see it either - it measures
the last frame of ANY topic, so ``trash_goal`` and the datum at 1 Hz keep the
link looking healthy while ``trash_gps`` is dead.

``list_age_sec``/``max_list_age_sec`` are therefore part of the correlation and
not of its caller: an age beyond the maximum is :data:`TARGETS_STALE`, checked
before anything else, because no amount of arithmetic on a frozen list can
recover the fact that it is frozen. Passing ``None`` for either keeps the old
behaviour and is what the preview-only callers do.

COLLECTED TARGETS ARE INCLUDED IN THE AMBIGUITY TEST
====================================================
A target already flagged ``collected`` still participates. It cannot itself be
the goal, but if it sits within ``tolerance_m`` of one that can, then the fix
does not identify either of them - and their ids restart at 1 on a restart of
their node, which is precisely when a stale ``collected`` flag is least
trustworthy. Excluding them would turn a genuinely undecidable pair into a
confident wrong answer.

THE REPORTED goal_id IS A CROSS-CHECK, NEVER A SUBSTITUTE
=========================================================
``trash_gps`` reports which id their side currently considers the goal. It is
used to *contradict* a position match, never to supply one: the two streams are
independent 1 Hz publications with no consistency guarantee at any instant, so
agreement is evidence and disagreement is a refusal (``ID_MISMATCH``). Taking
the reported id when the position match found nothing would reintroduce exactly
the gap this module closes.

Pure module: no rclpy, no ROS message types. Positions arrive already converted
into map metres by the caller, with the datum the caller is about to dispatch on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

#: Exactly one target inside the tolerance, and no reported id contradicts it.
UNIQUE = "UNIQUE"
#: No target inside the tolerance. The fix denotes something we do not know
#: about, so there is no id we could ever acknowledge.
NO_MATCH = "NO_MATCH"
#: More than one target inside the tolerance. Refusal, never a nearest-wins pick.
AMBIGUOUS = "AMBIGUOUS"
#: A unique position match that the reported ``goal_id`` contradicts.
ID_MISMATCH = "ID_MISMATCH"
#: The target list has not arrived yet. Distinct from NO_MATCH: nothing is
#: wrong, we simply cannot correlate yet.
NO_TARGETS = "NO_TARGETS"
#: A target list exists, but it is too old to be evidence about the world now.
#: Deliberately NOT folded into NO_MATCH: "there is nothing there" and "we
#: stopped being told what is there" are different facts and only one of them
#: is about the objects (SAFETY.md F-28).
TARGETS_STALE = "TARGETS_STALE"


@dataclass(frozen=True)
class TargetPosition:
    """One entry of the target list, already in map metres."""

    id: str
    x: float
    y: float
    collected: bool = False


@dataclass(frozen=True)
class CorrelationCandidate:
    id: str
    distance_m: float
    collected: bool


@dataclass(frozen=True)
class CorrelationResult:
    status: str
    #: Only set when ``status == UNIQUE``. Empty in every other case, including
    #: ``ID_MISMATCH`` - a contradicted match is not a usable id.
    target_id: str = ""
    distance_m: float = float("nan")
    candidates: Tuple[CorrelationCandidate, ...] = ()
    detail: str = ""

    @property
    def unique(self) -> bool:
        return self.status == UNIQUE


def correlate(
    goal_xy: Tuple[float, float],
    targets: Sequence[TargetPosition],
    tolerance_m: float,
    reported_goal_id: str = "",
    list_age_sec: Optional[float] = None,
    max_list_age_sec: Optional[float] = None,
) -> CorrelationResult:
    """Match a dispatched goal position to exactly one target id, or refuse.

    ``tolerance_m`` is ``goal_match_tolerance_m``. It is a *decision* radius,
    not a search radius: everything inside it is a candidate, and two candidates
    are a refusal.

    ``list_age_sec`` is how long ago the list in ``targets`` was received, and
    ``max_list_age_sec`` how old it may be. Both ``None`` disables the check;
    an age over the maximum is :data:`TARGETS_STALE` and is decided FIRST,
    before the geometry, because the geometry of a frozen list is exactly as
    self-consistent as the geometry of a live one (SAFETY.md F-28).
    """
    if (
        list_age_sec is not None
        and max_list_age_sec is not None
        and max_list_age_sec > 0.0
        and (not math.isfinite(list_age_sec) or list_age_sec > max_list_age_sec)
    ):
        return CorrelationResult(
            TARGETS_STALE,
            detail=(
                f"the target list is {list_age_sec:.1f} s old, over "
                f"max_target_list_age_sec={max_list_age_sec:.1f} s. It cannot be "
                "correlated against: it says what was there when it arrived, not "
                "what is there now, and the link watchdog cannot see one topic "
                "going silent (SAFETY.md F-28)"
            ),
        )
    if not targets:
        return CorrelationResult(
            NO_TARGETS,
            detail="no target list received yet; the goal fix carries no id of its own",
        )
    if not (tolerance_m > 0.0) or not math.isfinite(tolerance_m):
        # A non-positive tolerance cannot match anything, and silently
        # substituting a default would invent the very number this refuses.
        return CorrelationResult(
            NO_MATCH,
            detail=f"goal_match_tolerance_m={tolerance_m!r} is not a usable radius",
        )

    gx, gy = goal_xy
    if not (math.isfinite(gx) and math.isfinite(gy)):
        return CorrelationResult(NO_MATCH, detail="goal position is not finite")

    scored = []
    for target in targets:
        if not (math.isfinite(target.x) and math.isfinite(target.y)):
            continue
        distance = math.hypot(target.x - gx, target.y - gy)
        if distance <= tolerance_m:
            scored.append(CorrelationCandidate(target.id, distance, target.collected))
    scored.sort(key=lambda c: c.distance_m)
    candidates = tuple(scored)

    if not candidates:
        return CorrelationResult(
            NO_MATCH,
            candidates=(),
            detail=(
                f"no target within goal_match_tolerance_m={tolerance_m:.3f} of the "
                f"dispatched fix at ({gx:.3f}, {gy:.3f}); "
                f"{len(targets)} target(s) known"
            ),
        )
    if len(candidates) > 1:
        listing = ", ".join(f"{c.id}@{c.distance_m:.3f}m" for c in candidates)
        return CorrelationResult(
            AMBIGUOUS,
            candidates=candidates,
            detail=(
                f"{len(candidates)} targets within {tolerance_m:.3f} m of the "
                f"dispatched fix ({listing}). Refusing rather than guessing: an "
                "acknowledgement cannot be taken back, and their merge_radius_m "
                "of 0.25 makes near pairs expected"
            ),
        )

    only = candidates[0]
    if reported_goal_id and reported_goal_id != only.id:
        return CorrelationResult(
            ID_MISMATCH,
            candidates=candidates,
            distance_m=only.distance_m,
            detail=(
                f"position matches target {only.id} at {only.distance_m:.3f} m, but "
                f"the target list reports goal_id={reported_goal_id!r}. The two "
                "streams disagree about which object this is"
            ),
        )
    if only.collected:
        return CorrelationResult(
            NO_MATCH,
            candidates=candidates,
            detail=(
                f"the only match, target {only.id}, is already flagged collected; "
                "acknowledging it again would tell the source nothing and driving "
                "to it would repeat work it believes is done"
            ),
        )
    return CorrelationResult(
        UNIQUE,
        target_id=only.id,
        distance_m=only.distance_m,
        candidates=candidates,
        detail=f"target {only.id} at {only.distance_m:.3f} m",
    )


def nearest_distance(
    goal_xy: Tuple[float, float], targets: Sequence[TargetPosition]
) -> Optional[float]:
    """Distance to the closest target, for reporting a NO_MATCH usefully."""
    gx, gy = goal_xy
    distances = [
        math.hypot(t.x - gx, t.y - gy)
        for t in targets
        if math.isfinite(t.x) and math.isfinite(t.y)
    ]
    if not distances or not (math.isfinite(gx) and math.isfinite(gy)):
        return None
    return min(distances)
