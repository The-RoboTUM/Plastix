"""Manoeuvre modes and the mode-transition guard for the keyboard teleop.

Why this module exists
----------------------
The keyboard teleop can put the four steering modules into three fundamentally
different poses:

    cornering   (delta, delta, -delta, -delta)   -- A/D, legacy direct-steer path
    crab        all four at -+90 deg             -- arrow left/right, IK path
    spin        (-58.57, +58.57, +58.57, -58.57) -- arrow up/down,   IK path

Switching between them swings wheels through up to 140 deg. At roughly
0.3-0.5 s per 90 deg that is NOT instantaneous, and applying traction while the
modules are still slewing moves the robot in a direction nobody asked for and
scrubs the tyres. `TransitionGuard` below is what stops that: it withholds
drive until the modules have actually reached the requested pose, judged by the
real servo feedback on `/hw/steer_states` where that is available and by a
worst-case timeout where it is not.

The pose targets are NOT re-derived here. They come out of the same
`inverse_kinematics()` + `resolve_wheel_targets()` that `swerve_cmd_node` will
use on the Pi, so the pose the guard waits for is by construction the pose the
servos are being driven to. A second, hand-written formula for the same angles
would be a second source of truth for a safety-relevant comparison -- exactly
the failure mode the per-wheel steering-limit rework removed elsewhere.

Pure python: imports nothing from rclpy, so the guard and the pose maths can be
checked without a ROS graph (`test/check_manoeuvres.py`).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from gripperx_control.steering_limits import (
    MODEL_TO_JOINT_INDEX,
    SteeringLimits,
    normalize_angle,
    resolve_wheel_targets,
)
from gripperx_control.swerve_kinematic_model import (
    BodyTwist,
    FourWIS4WIDKinematicModel,
)

# ── Manoeuvre identities ────────────────────────────────────────────────────

CORNERING = "cornering"
CRAB_LEFT = "crab_left"
CRAB_RIGHT = "crab_right"
SPIN_CW = "spin_cw"
SPIN_CCW = "spin_ccw"

MANOEUVRES = (CORNERING, CRAB_LEFT, CRAB_RIGHT, SPIN_CW, SPIN_CCW)

# Which synthetic key name requests which manoeuvre. The arrow keys are the
# only manoeuvre keys; everything else (W/S/A/D) means "cornering".
#
# Mapping as specified by the user: LEFT/RIGHT crab, UP = clockwise,
# DOWN = counter-clockwise. Note that UP/DOWN is therefore NOT the usual
# "up = forward" reading -- see the note in the node's banner.
KEY_TO_MANOEUVRE = {
    "left": CRAB_LEFT,
    "right": CRAB_RIGHT,
    "0": SPIN_CW,
    "9": SPIN_CCW,
}

# REBOUND 2026-08-24 (user). The arrows used to be all four manoeuvres; UP/DOWN
# are now the CRAB STEERING keys (see CRAB_STEER_KEYS below) and the two spins
# moved to `0` and `9`. The old UP/DOWN binding was already the odd one out --
# UP meant "spin clockwise", not "forward", and the node's banner had to carry a
# note saying so.
CRAB_STEER_KEYS = ("up", "down")

# Precedence if two manoeuvre keys somehow look held at the same instant (the
# node invalidates the others on every press, so this is only a tie-breaker for
# identical timestamps). Earlier entries win.
MANOEUVRE_KEY_PRECEDENCE = ("left", "right", "0", "9")

HUMAN_LABEL = {
    CORNERING: "CORNERING (W/S drive, A/D steer)",
    CRAB_LEFT: "CRAB LEFT  (sideways left, arrow up/down steers)",
    CRAB_RIGHT: "CRAB RIGHT (sideways right, arrow up/down steers)",
    SPIN_CW: "SPIN CW    (in place, -omega, key 0)",
    SPIN_CCW: "SPIN CCW   (in place, +omega, key 9)",
}


# ── Steerable crab: which translation directions the chassis can actually reach ──
#
# A PURE TRANSLATION PUTS ALL FOUR MODULES ON THE SAME ANGLE. That is not a
# simplification, it is the only solution: every point of the body moves in the
# same direction, so every wheel must stand perpendicular to nothing and along
# that direction. The steering angle a translation at heading psi needs is
# therefore delta = psi on every wheel, modulo the +-180 deg module fold.
#
# AND THE WINDOWS ARE ASYMMETRIC, so most of the circle is not available. With
# the calibrated 100 deg outward / 35 deg inward and the measured outward sign
# (-1, +1, +1, -1), the reachable set is FOUR SEPARATE ARCS with four 45 deg
# DEAD BANDS between them:
#
#     reachable   [-180,-145]  [-100,-80]  [-35,+35]  [+80,+100]  [+145,+180]
#     dead band          (-145,-100)   (-80,-35)   (+35,+80)   (+100,+145)
#
# So "steer the crab" cannot mean "sweep psi continuously". Between pure crab
# (+-90 deg, and note it has only +-10 deg of room) and the forward cone (+-35
# deg) there is a 45 deg gap in which NO pose exists — swerve_controller's
# limit_twist_to_steering_range would return kRejected, zero the drive and hold
# the steering, for the whole width of it.
#
# nav2.yaml's crab_walk block reached the same conclusion from the other end
# ("the required 90 deg sits 10 deg inside the outward stop ... any mix moves the
# required angle off 90 deg, and the +-10 deg that exist are the whole budget"),
# which is why this is computed here rather than argued: the arcs come out of the
# SAME SteeringLimits the pose resolution uses, so they cannot drift apart from
# it if the calibration changes.
#
# THE DEAD BANDS COME FROM THE INWARD LIMIT, NOT THE OUTWARD ONE. Continuous psi
# would need outward >= 145 deg (so that the folded solution psi-180 stays inside
# the window all the way down to +35). Raising the outward limit past the
# calibrated 100 deg is explicitly out of scope.


def reachable_translation_arcs(
    limits: SteeringLimits,
    resolution_deg: float = 0.5,
) -> List[Tuple[float, float]]:
    """Closed intervals of psi (rad, in (-pi, pi]) a pure translation can reach.

    Swept rather than solved: the window algebra has to account for both module
    folds on four wheels with two different window shapes, and a closed form for
    that is a page of case analysis whose bugs would be silent. At half a degree
    the arcs are exact to well inside the servo's own resolution, and this runs
    once at construction.
    """

    steps = int(round(360.0 / resolution_deg))
    arcs: List[Tuple[float, float]] = []
    start: Optional[float] = None
    previous = -math.pi
    for index in range(steps + 1):
        psi = -math.pi + (index * 2.0 * math.pi / steps)
        ok = all(
            any(
                limits.lower[wheel] - 1e-9
                <= normalize_angle(psi + fold)
                <= limits.upper[wheel] + 1e-9
                for fold in (0.0, math.pi, -math.pi)
            )
            for wheel in range(len(limits.lower))
        )
        if ok and start is None:
            start = psi
        elif not ok and start is not None:
            arcs.append((start, previous))
            start = None
        previous = psi
    if start is not None:
        arcs.append((start, math.pi))
    return arcs


def snap_psi_into_reach(
    psi: float,
    arcs: Sequence[Tuple[float, float]],
    moving_towards_zero: bool,
) -> float:
    """Move psi to the nearest reachable arc edge IN THE DIRECTION OF TRAVEL.

    Called only when psi has walked into a dead band. `moving_towards_zero` says
    which way the operator is steering, and it decides which side of the gap to
    come out on — snapping backwards would make the key do the opposite of what
    it says.

    Returns psi unchanged if it is already reachable, or if there is no arc on
    the far side (the operator is steering into the outside of the last arc, so
    the correct behaviour is to stay put, which the caller enforces by clamping).
    """

    for low, high in arcs:
        if low - 1e-9 <= psi <= high + 1e-9:
            return psi

    candidates = []
    for low, high in arcs:
        for edge in (low, high):
            forward = abs(edge) < abs(psi) if moving_towards_zero else abs(edge) > abs(psi)
            # Stay on the operator's side of the robot: a crab-left that snapped
            # through zero into a crab-right would be a different manoeuvre.
            same_side = (edge >= -1e-9) == (psi >= -1e-9) or abs(edge) < 1e-9
            if forward and same_side:
                candidates.append(edge)
    if not candidates:
        return psi
    return min(candidates, key=lambda edge: abs(edge - psi))


def crab_twist(crab_speed: float, psi: float) -> Tuple[float, float, float]:
    """Pure translation at heading psi. omega is 0 BY CONSTRUCTION, not by choice.

    psi is measured in the robot frame: 0 = straight ahead, +pi/2 = the robot's
    left. So the plain crab manoeuvres are psi = +-pi/2 and the steered ones move
    away from there.

    THE TWO CARDINAL HEADINGS ARE RETURNED LITERALLY, and that is not a
    micro-optimisation. `math.cos(math.pi / 2)` is 6.1e-17, not 0.0, so the
    general formula would give an unsteered crab a phantom forward component
    where the pre-2026-08-24 code had an exact zero. Physically it is 15
    attometres per second and irrelevant; as a VALUE it is the difference between
    `vx == 0.0` and `vx != 0.0`, and this chain tests exact zeros in several
    places on purpose (swerve_controller's OP-24/S1 zero-twist test, its
    manoeuvre classification). Introducing an epsilon to clean it up would add
    the very kind of unmeasured threshold those exact tests exist to avoid, so
    the comparison here is EXACT EQUALITY against the value the caller starts psi
    at — no tolerance, and nothing to tune.
    """

    speed = abs(crab_speed)
    if psi == math.pi / 2.0:
        return (0.0, speed, 0.0)
    if psi == -math.pi / 2.0:
        return (0.0, -speed, 0.0)
    return (speed * math.cos(psi), speed * math.sin(psi), 0.0)


def manoeuvre_twist(
    manoeuvre: str,
    crab_speed: float,
    spin_speed: float,
    crab_psi: Optional[float] = None,
) -> Tuple[float, float, float]:
    """Body twist (vx, vy, omega) a manoeuvre asks for at full traction.

    Sign convention is the robot frame used everywhere else in this chain:
    +y is the robot's left, +omega is counter-clockwise (right-hand rule, z up).

    `crab_psi` steers a crab away from pure sideways. Omitting it reproduces the
    pre-2026-08-24 behaviour EXACTLY — psi = +-pi/2 gives (0, +-crab_speed, 0) to
    the last bit — which is what keeps the accepted crab recovery unchanged.
    """

    if manoeuvre == CRAB_LEFT:
        return crab_twist(crab_speed, math.pi / 2.0 if crab_psi is None else crab_psi)
    if manoeuvre == CRAB_RIGHT:
        return crab_twist(crab_speed, -math.pi / 2.0 if crab_psi is None else crab_psi)
    if manoeuvre == SPIN_CW:
        return (0.0, 0.0, -abs(spin_speed))
    if manoeuvre == SPIN_CCW:
        return (0.0, 0.0, abs(spin_speed))
    return (0.0, 0.0, 0.0)


def model_to_joint_order(model_values: Sequence[float]) -> List[float]:
    """Lee-2015 wheel order (FL, BL, BR, FR) -> joint order (FL, FR, BL, BR)."""

    joint = [0.0] * len(model_values)
    for model_index, joint_index in enumerate(MODEL_TO_JOINT_INDEX):
        joint[joint_index] = float(model_values[model_index])
    return joint


def joint_to_model_order(joint_values: Sequence[float]) -> List[float]:
    """Joint order (FL, FR, BL, BR) -> Lee-2015 wheel order (FL, BL, BR, FR)."""

    return [float(joint_values[joint_index]) for joint_index in MODEL_TO_JOINT_INDEX]


def pose_for_twist(
    twist: Tuple[float, float, float],
    model: FourWIS4WIDKinematicModel,
    model_limits: SteeringLimits,
    current_joint_angles: Optional[Sequence[float]] = None,
) -> Optional[List[float]]:
    """Steering pose (joint order, rad) the controller will command for a twist.

    Split out of manoeuvre_pose() when the crab gained a steering angle: the
    guard now has to wait for the pose of an ARBITRARY psi, not only for one of
    the five named manoeuvres. Same code path either way, so a steered crab is
    guarded exactly as a plain one is.

    Returns None if no wheel has a reachable module solution -- swerve_controller
    would then reject the twist, hold the steering and command zero drive, so
    there is no pose to wait for and the manoeuvre must not be armed.
    """

    vx, vy, omega = twist
    current_model = (
        joint_to_model_order(current_joint_angles)
        if current_joint_angles is not None
        else [0.0] * 4
    )
    targets = resolve_wheel_targets(
        model.inverse_kinematics(BodyTwist(vx=vx, vy=vy, omega=omega)),
        current_model,
        model_limits,
    )
    if targets is None:
        return None
    return model_to_joint_order([target.angle for target in targets])


def manoeuvre_pose(
    manoeuvre: str,
    model: FourWIS4WIDKinematicModel,
    model_limits: SteeringLimits,
    crab_speed: float,
    spin_speed: float,
    current_joint_angles: Optional[Sequence[float]] = None,
    crab_psi: Optional[float] = None,
) -> Optional[List[float]]:
    """Steering pose (joint order, rad) `swerve_controller` will command."""

    if manoeuvre == CORNERING:
        return [0.0] * 4

    return pose_for_twist(
        manoeuvre_twist(manoeuvre, crab_speed, spin_speed, crab_psi),
        model,
        model_limits,
        current_joint_angles,
    )


# ── Transition guard ────────────────────────────────────────────────────────


class GuardState:
    """Where a manoeuvre change currently is."""

    # Waiting for steer_servo_node's /teleop/direct_steer override to lapse.
    # While the override is alive it wins over /hw/joint_commands, so the IK
    # path cannot move the modules at all and there is nothing to align yet.
    RELEASING = "releasing"
    # Pose commanded, modules slewing, traction withheld.
    ALIGNING = "aligning"
    # Modules are where they were asked to be -- traction released.
    ARMED = "armed"


class TransitionGuard:
    """Withhold drive until the steering modules have reached the new pose.

    Deliberately dumb about *what* the pose is: the caller passes the target and
    the measurement, both in joint order. That keeps the same guard usable for
    the IK poses (crab/spin) and for the legacy direct-steer pose (cornering),
    which are produced by completely different code paths.

    Start state is ARMED with manoeuvre=cornering: as long as no arrow key is
    ever pressed, `request()` is never called with a different manoeuvre and the
    guard never intervenes. Plain W/S/A/D driving is therefore byte-identical to
    before this feature existed -- including on a laptop that never sees
    /hw/steer_states.
    """

    def __init__(
        self,
        release_sec: float,
        align_timeout_sec: float,
        align_tolerance_rad: float,
    ) -> None:
        self.release_sec = float(release_sec)
        self.align_timeout_sec = float(align_timeout_sec)
        self.align_tolerance_rad = float(align_tolerance_rad)

        self.manoeuvre: str = CORNERING
        self.state: str = GuardState.ARMED
        self.state_since: float = 0.0
        # True if ARMED was reached on the timeout instead of on measured
        # angles -- the caller warns about it, because "armed" then means
        # "probably arrived", not "known arrived".
        self.armed_without_feedback: bool = False
        # Diagnostics only. Deliberately NOT part of describe(): it changes on
        # every tick while the modules slew, and the status line is written to a
        # raw terminal on change — a live value there would scroll the operator's
        # screen at the publish rate.
        self.last_error_rad: Optional[float] = None
        self.pose_reachable: bool = True

    # -- requests ---------------------------------------------------------

    def request(self, manoeuvre: str, now: float) -> bool:
        """Ask for a manoeuvre. Returns True if this started a transition."""

        if manoeuvre == self.manoeuvre:
            return False

        previous = self.manoeuvre
        self.manoeuvre = manoeuvre
        self.armed_without_feedback = False
        self.last_error_rad = None
        self.pose_reachable = True
        self.state_since = now
        # Leaving cornering is the only transition that has to wait for the
        # direct-steer override to expire; every other transition is already on
        # the IK path and can start slewing immediately.
        self.state = (
            GuardState.RELEASING
            if (manoeuvre != CORNERING and previous == CORNERING)
            else GuardState.ALIGNING
        )
        return True

    def force_cornering(self, now: float, *, rearm: bool) -> None:
        """Emergency stop / shutdown: back to cornering unconditionally.

        `rearm=True` restores the pre-feature behaviour (traction immediately
        available again after the stop) and is used when no manoeuvre was
        active, so the accepted SR-2 behaviour of the space bar is unchanged.
        `rearm=False` keeps traction withheld until the modules are measurably
        straight again, which is what a stop out of a 90 deg crab pose needs.
        """

        self.manoeuvre = CORNERING
        self.armed_without_feedback = False
        self.last_error_rad = None
        self.pose_reachable = True
        self.state_since = now
        self.state = GuardState.ARMED if rearm else GuardState.ALIGNING

    # -- progress ---------------------------------------------------------

    def update(
        self,
        now: float,
        target_joint_angles: Optional[Sequence[float]],
        measured_joint_angles: Optional[Sequence[float]],
    ) -> None:
        if self.state == GuardState.ARMED:
            return

        if self.state == GuardState.RELEASING:
            if (now - self.state_since) >= self.release_sec:
                self.state = GuardState.ALIGNING
                self.state_since = now
            return

        if target_joint_angles is None:
            # No reachable pose -> swerve_cmd_node will reject and hold. Never
            # arm; the operator sees "will not arm" and swerve_cmd_node logs
            # the rejection.
            self.pose_reachable = False
            self.last_error_rad = None
            return
        self.pose_reachable = True

        if measured_joint_angles is not None:
            self.last_error_rad = max(
                abs(normalize_angle(target - measured))
                for target, measured in zip(target_joint_angles, measured_joint_angles)
            )
            if self.last_error_rad <= self.align_tolerance_rad:
                self.state = GuardState.ARMED
                self.armed_without_feedback = False
                self.state_since = now
                return

        if (now - self.state_since) >= self.align_timeout_sec:
            self.state = GuardState.ARMED
            self.armed_without_feedback = True
            self.state_since = now

    # -- verdict ----------------------------------------------------------

    @property
    def drive_allowed(self) -> bool:
        return self.state == GuardState.ARMED

    @property
    def pose_commanded(self) -> bool:
        """Should the manoeuvre pose be commanded (at reduced traction)?

        Not during RELEASING: the override still owns the servos there, so the
        only thing a pose command would achieve is turning the wheels a little
        while they are still in the old pose.
        """

        return self.state in (GuardState.ALIGNING, GuardState.ARMED)

    def describe(self) -> str:
        """One line for the operator. Stable per state — see last_error_rad."""

        if self.state == GuardState.RELEASING:
            note = " - handing the steering servos over to the swerve IK"
        elif self.state == GuardState.ALIGNING and not self.pose_reachable:
            note = " - POSE UNREACHABLE, will not arm"
        elif self.state == GuardState.ALIGNING:
            note = " - wheels moving into pose, drive withheld"
        elif self.armed_without_feedback:
            note = " - NO STEER FEEDBACK, armed on timeout, pose NOT confirmed"
        else:
            note = " - wheels in pose"
        return f"{HUMAN_LABEL[self.manoeuvre]} | {self.state}{note}"
