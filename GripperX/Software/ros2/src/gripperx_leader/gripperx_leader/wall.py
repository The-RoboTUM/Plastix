"""Virtual wall on the leader (user decisions 2026-10-15/16, progressive wall and engage
deadband 2026-10-20).

The follower keeps leader setpoints leader.limit_command_margin_ticks inside its joint
limits (gripperx_arm.leader_band). Leading a joint further changes nothing on the
follower, so the operator gets no cue. While leader mode is active (armed), a leader joint
beyond an EDGE gets torque on ONLY its own servo and feels a SPRING: the deeper beyond,
the stronger. Pure python: the bus is injected, the caller ticks it (leader_arm_node),
the tests run with a register-level mock.

WHY A SPRING (robot test 2026-10-20). The 2026-10-16 design engaged at the present tick,
ramped the goal inward and released on an inward overrun. Under gravity limb2's lower
wall engaged 75 times in ~15 s: the servo overshot the goal inward, the overrun release
cut torque, gravity pulled the joint out again, re-engage - felt as chatter. Waiting for
"still" before re-engaging let limb4 fall 0.074 rad beyond. User decision 2026-10-20: a
progressive wall, full force at wall_full_force_depth_rad (0.1) beyond, at most 50 %.

ENGAGE DEADBAND, release at the edge (robot test and user decision 2026-10-20): the
spring design kept torque enabled with Torque_Limit 0 in a strip inside the wall; the
STS3215 brakes the motor in that state, so the leader stuck at the wall. Torque now goes
off as soon as the joint is back inside, and the engage waits for a small depth.

EDGES come from two SOURCES, one owner per servo, the band wins:
  band      the follower's command band (static, per limb joint: lower and upper edge).
  boundary  EXTENSION POINT, not fed yet: when the follower rejects a leader setpoint for
            ground/collision it holds the last approved pose; set_boundary() takes, per
            joint, an Edge at that pose with a tolerance of free travel beyond it.
            Freshness: (session, seq) must advance, and the boundary is dropped after
            boundary_timeout_s without an update; a dropped or cleared boundary releases
            only the joints it owns.
An edge's SPRING ZERO is its rad moved outward by its tolerance (band: the edge itself).
DEPTH d = how far the present reading is beyond the spring zero (0 when inside; within
DEPTH_EPS_RAD of the zero counts as 0 - the shipped band edges lie on leader ticks, and a
reading at the wall tick converts to +-1e-13 rad there).

Per joint with a band (all six since 2026-11-13, the gripper like limb1..limb5):
  FREE  torque off. d > engage_depth_rad -> ENGAGE (HELD); a joint beyond at arming
        likewise, from the first step on. A joint beyond by 0 < d <= engage_depth_rad
        stays FREE (the ENGAGE DEADBAND, user decision 2026-10-20).
  HELD  torque on, Goal_Position = the HOLD TICK (below), written ONCE at engage (and
        again only if the owning edge changes). Each tick:
          d > 0   Torque_Limit = spring(d): wall_min_torque_limit at the first tick
                  beyond, rising linearly to wall_torque_limit at d >=
                  wall_full_force_depth_rad, never above the servo's own Torque_Limit,
                  never below 1. The deadband only delays the engage: once HELD, the
                  spring is measured from the zero, so a joint engaged at d just above
                  engage_depth_rad gets spring(d) there, not the floor at d = 0.
          d <= 0  (back inside, by any amount) -> torque off (confirmed), Torque_Limit
                  restored -> FREE.
        TORQUE IS NEVER LEFT ENABLED WITH Torque_Limit 0 (robot 2026-10-20: the
        STS3215 brakes the motor then - the leader got stuck in the wall and moved out
        only stiffly). Engage writes a Torque_Limit >= 1 before torque on, the spring
        is >= 1 for d > 0, release turns torque off before the restore, and
        _write_limit refuses a 0 while held (a fault, all limp).
        WHY THE DEADBAND: release at d <= 0 and engage at d > engage_depth_rad are two
        readings more than engage_depth_rad apart; with engage_depth_rad >= 2 ticks a
        reading jittering by +-1 tick can never do both, wherever it sits - no
        Torque_Enable toggling from reading noise at the edge. Toggling still costs bus
        time: an engage is 6 reads + 3 writes, a release 1 write + 1 read-back + 1
        restore write (all request/reply), see README "Bus load".
        Registers are written only when the value changes: a joint resting at one depth
        costs no write per tick.
        THE HOLD TICK (robot risk 2026-10-20): the first tick beyond the zero by more
        than one tick - 2 ticks beyond for an edge on a tick (the shipped edges), 1..2
        otherwise - and never outward of the present reading at engage. With the goal
        AT the zero, a servo stiff enough to hold within half a tick would bring the
        joint to rest reading d <= 0 -> released -> gravity pulls it out -> engaged
        again: chatter. At the hold tick a joint resting on its goal reads d > 0 and
        stays held, and a reading jittering +-1 tick around it still reads d > 0: no
        release from noise at rest either. With engage_depth_rad >= 2 ticks the present
        reading at engage (beyond by more than 2 ticks) is always outward of the hold
        tick, so the clamp to the present only acts for a smaller deadband or a boundary
        retarget. The servo's position loop gives a force growing from 0 at the hold
        tick, capped by Torque_Limit = spring(d) measured from the ZERO; a goal further
        inward would push the joint inward of the wall when the hand lets go (the
        2026-10-20 overshoot), a goal at the present tick gives no force.
        WHAT IS TRUE about outward pull: as of a reading, torque is on only while the
        joint is beyond the zero. Between the zero and the hold tick (at most 2 ticks)
        the servo pulls OUTWARD toward the hold tick with at most spring(d) there (about
        the floor) - a 2-tick detent the hand leaves through; one more tick inward
        reads d <= 0 and releases. Between two readings a joint that has crossed inward
        is pulled back toward the hold tick by at most the previous tick's
        Torque_Limit, for at most one tick. A goal is never written outward of the
        present reading.
        OVERLOAD (thermal protection, user decision 2026-10-16): |Present_Load| >=
        overload_load and no inward progress of overload_progress_rad for overload_time_s
        -> torque off (confirmed) -> LIMP ('overload release'). Present_Load is most
        likely the PWM duty, which Torque_Limit caps (TO-VERIFY): so the release can
        only fire where spring(d) >= overload_load - deep beyond, where the servo works
        hard; a joint resting shallow under gravity is not timed out unless gravity
        alone needs more than overload_load.
        Every HELD_LOG_PERIOD_S per joint an event reports depth, Torque_Limit and load,
        to calibrate these values on the robot.
  LIMP  torque off beyond an edge: after an overload release or a refused engage. Back
        inside that edge (d <= 0, the release criterion) -> FREE, writing nothing; it
        engages again only beyond by more than engage_depth_rad, so the deadband is
        also the latch's re-arm hysteresis - no separate re-arm distance. An overload
        latch is kept when a boundary-latched joint crosses the band edge (it moves to
        the band, the band owns it, still limp) - no fresh push.

ENGAGE pre-checks on that servo: Torque_Enable 0, Operating_Mode 0, Present_Voltage in
the band, no status bits, the present tick and the hold tick inside Min/Max_Position_Limit,
the servo's own Torque_Limit above 0 - else LIMP ('not engaged'). Then Torque_Limit :=
spring(d) (the cap: min(wall_torque_limit, the servo's own value), never raised), goal :=
hold tick, torque on. The servo's own value is the register read at the first engage, or
the original still pending after a failed restore (kept and retried at the next release).
WRAP: the leader's limb2 wraps 0/4095 109 ticks inward of its lower wall tick (109). The
goal is the hold tick (111), inside the position limits, and the joint is torqued (as of
a reading) only beyond the zero - all on the wall's side of the wrap. A joint beyond that
wall sits between the wall and its mechanical stop (ticks 110..~127), on the same side.
A yank inward across the wrap within one tick reads as far inside -> released. The goal
is computed once, from a reading beyond the zero, never from a wrapped one. Radians wrap
into the calibration's range_center_rad +- pi, so a reading beyond a wall aliases to the
other wall only half a revolution from that centre: about 890 ticks beyond limb2's lower
wall; the tightest is limb5's lower wall at 274 ticks, while its recorded travel
(travel_min_rad) ends about 130 ticks beyond it (computed 2026-10-16 from the shipped
calibration and the band).
While HELD: Present_Load of every held servo each tick, and one held servo per tick
monitored (voltage, Torque_Enable 1, Torque_Limit as last written, status bits).

FAULT (any read or write error, failed monitor, unconfirmed single torque-off) and every
release_all (Space, leader mode off, J, read error in the caller, node shutdown): torque
off on EVERY leader servo, confirmed by read-back; unconfirmed ids stay PENDING and the
caller retries them every tick (retry_release), as the aligner does.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from gripperx_leader.leader_bus import (ADDR_MAX_POS_LIMIT, ADDR_MIN_POS_LIMIT,
                                        ADDR_OPERATING_MODE, ADDR_PRESENT_LOAD,
                                        ADDR_PRESENT_VOLTAGE, ADDR_TORQUE,
                                        ADDR_TORQUE_LIMIT, LOAD_MAGNITUDE_MASK,
                                        LeaderReadError, LeaderWriteError)

FREE, HELD, LIMP = 'free', 'held', 'limp'
BAND, BOUNDARY = 'band', 'boundary'
OVERLOAD = 'overload release'
# Depth / Torque_Limit / load report per held joint, for calibration on the robot.
HELD_LOG_PERIOD_S = 2.0
# A depth within this of the zero is 0: floating-point noise of the tick -> rad
# conversion (an edge on a tick reads +-1e-13 rad at its tick), not a tuning value.
DEPTH_EPS_RAD = 1e-9
# The hold goal lies beyond the zero by more than 1 + this many ticks (floating-point
# margin, well above DEPTH_EPS_RAD in ticks), so the tick inward of it still reads d > 0.
HOLD_EPS_TICKS = 1e-6


class WallFault(RuntimeError):
    """A wall error; torque off on every leader servo has already been attempted."""


@dataclass(frozen=True)
class Edge:
    """The joint must stay on the inner side of `rad`: side 'lower' = at or above it,
    'upper' = at or below it. `tolerance`: free travel beyond `rad` before the spring
    starts (the spring zero lies `tolerance` outward of `rad`)."""
    side: str
    rad: float
    tolerance: float = 0.0

    def inward(self, rad: float) -> float:
        """Distance inside the edge (negative = beyond it)."""
        return rad - self.rad if self.side == 'lower' else self.rad - rad

    @property
    def zero(self) -> float:
        """The spring zero (rad): where the spring (Torque_Limit) is measured from."""
        return self.rad - self.tolerance if self.side == 'lower' else self.rad + self.tolerance

    def inside_zero(self, rad: float) -> float:
        """Distance inside the spring zero (negative = beyond it: minus the depth)."""
        return self.inward(rad) + self.tolerance


@dataclass
class WallConfig:
    # Torque_Limit register 48 (SRAM), 0..1000, at and beyond full_force_depth_rad; never
    # raised above the servo's own value. TO-VERIFY (user decision 2026-10-20: 50 % = 500).
    torque_limit: Optional[int] = 500
    # Torque_Limit at the first tick beyond (the spring's floor). TO-VERIFY
    # (implementation start value 2026-10-20, not a user value).
    min_torque_limit: int = 100
    # Depth beyond the spring zero at which torque_limit is reached. TO-VERIFY (user
    # decision 2026-10-20: 0.1).
    full_force_depth_rad: float = 0.1
    # Engage deadband: a FREE joint engages only beyond the zero by MORE than this; back
    # inside (depth <= 0) releases. >= 2 ticks keeps +-1 tick reading noise from toggling
    # torque. TO-VERIFY (user decision 2026-10-20: start at 2 ticks, 2 * 2 pi / 4096 =
    # 0.003068 rad, rounded up so a reading exactly 2 ticks beyond stays in the deadband).
    engage_depth_rad: float = 0.00307
    # Overload release (user decision 2026-10-16), all TO-VERIFY; time is the user's
    # value, load and progress are implementation start values.
    overload_time_s: float = 2.0
    # Present_Load magnitude (reg 60, bits 0-9, 0..1000, likely per mille of max PWM duty).
    overload_load: int = 300
    overload_progress_rad: float = 0.01
    # Boundary source freshness (extension point, not fed yet). TO-VERIFY.
    boundary_timeout_s: float = 0.5
    # The alignment's mandatory Present_Voltage band (align_voltage_min/max_v).
    voltage_min_v: float = 6.5
    voltage_max_v: float = 8.4
    voltage_lsb_v: float = 0.1               # Present_Voltage unit, TO-VERIFY

    def problem(self) -> Optional[str]:
        if self.torque_limit is None or isinstance(self.torque_limit, bool) or \
                not 0 < int(self.torque_limit) <= 1000:
            return f'wall_torque_limit {self.torque_limit} not set or outside 1..1000'
        if isinstance(self.min_torque_limit, bool) or \
                not 1 <= int(self.min_torque_limit) <= int(self.torque_limit):
            return (f'wall_min_torque_limit {self.min_torque_limit} outside 1..'
                    f'wall_torque_limit {self.torque_limit} (0 = torque on without force, '
                    'which brakes the STS3215)')
        if not self.full_force_depth_rad > 0.0:
            return f'wall_full_force_depth_rad {self.full_force_depth_rad} not > 0'
        if not 0.0 <= self.engage_depth_rad < self.full_force_depth_rad:
            return (f'wall_engage_depth_rad {self.engage_depth_rad} outside 0..'
                    f'wall_full_force_depth_rad {self.full_force_depth_rad}')
        if not 0.0 < self.voltage_min_v < self.voltage_max_v:
            return (f'voltage band {self.voltage_min_v}..{self.voltage_max_v} V not set '
                    '(align_voltage_min_v/max_v)')
        if not self.overload_time_s > 0.0:
            return f'wall_overload_time_s {self.overload_time_s} not > 0'
        if isinstance(self.overload_load, bool) or \
                not 0 < int(self.overload_load) <= int(self.torque_limit):
            return (f'wall_overload_load {self.overload_load} outside 1..wall_torque_limit '
                    f'{self.torque_limit} (Present_Load is capped by Torque_Limit: the '
                    'overload release could never fire)')
        if not self.overload_progress_rad > 0.0:
            return f'wall_overload_progress_rad {self.overload_progress_rad} not > 0'
        if not self.boundary_timeout_s > 0.0:
            return f'boundary_timeout_s {self.boundary_timeout_s} not > 0'
        return None


@dataclass
class _Joint:
    state: str = FREE
    owner: Optional[str] = None       # BAND | BOUNDARY while HELD or LIMP
    edge: Optional[Edge] = None
    goal: int = 0                     # the hold tick, written at engage
    cap: int = 0                      # min(wall torque limit, the servo's own value)
    tl: int = 0                       # Torque_Limit last written
    timer_t0: Optional[float] = None  # overload timer start, None = not loaded
    ref_rad: float = 0.0              # position the progress is measured from
    reason: str = ''                  # why LIMP
    load: int = 0                     # last Present_Load magnitude
    log_t: Optional[float] = None     # last depth/load report


class LeaderWall:
    def __init__(self, bus, cal, cfg: WallConfig, names: Sequence[str],
                 bands: Sequence[Optional[Tuple[float, float]]]):
        """`bands`: per bus id (bus.ids order) the follower's command band in rad, or
        None for a joint without a wall (none in leader_arm_node since 2026-11-13)."""
        why = cfg.problem()
        if why:
            raise ValueError(why)
        if len(bands) != len(bus.ids):
            raise ValueError('one band (or None) per configured servo required')
        ticks = cfg.engage_depth_rad * cal.ticks_per_rev / (2.0 * math.pi)
        if ticks < 2.0:
            raise ValueError(f'wall_engage_depth_rad {cfg.engage_depth_rad} is {ticks:.2f} ticks, '
                             'below 2: +-1 tick reading noise could toggle torque')
        self._bus, self._cal, self._cfg = bus, cal, cfg
        self._names = list(names)
        self._bands = list(bands)
        self._k = cal.ticks_per_rev / (2.0 * math.pi)
        self.armed = False
        self._j = [_Joint() for _ in bands]
        self._orig_limit: Dict[int, int] = {}     # sid -> Torque_Limit to restore
        self._set_limit: Dict[int, int] = {}
        self.unconfirmed: List[int] = []
        self._mon = 0
        self._events: List[str] = []
        self._boundary: Optional[List[Optional[Edge]]] = None
        self._boundary_key: Optional[Tuple[object, int]] = None
        self._boundary_rx = 0.0

    # ------------------------------------------------------------------ public
    @property
    def state(self) -> List[str]:
        return [j.state for j in self._j]

    @property
    def wall_tick(self) -> Dict[int, int]:
        """index -> goal (hold) tick, while HELD."""
        return {i: j.goal for i, j in enumerate(self._j) if j.state == HELD}

    def owner(self, i: int) -> Optional[str]:
        return self._j[i].owner

    def goal(self, i: int) -> Optional[int]:
        j = self._j[i]
        return j.goal if j.state == HELD else None

    def torque_limit(self, i: int) -> Optional[int]:
        """Torque_Limit last written while HELD (always >= 1)."""
        j = self._j[i]
        return j.tl if j.state == HELD else None

    def spring(self, cap: int, depth: float) -> int:
        """Torque_Limit at `depth` (rad) beyond the zero, for a servo capped at `cap`
        (>= 1). 0 inside - never written: inside is released."""
        cfg = self._cfg
        if depth <= 0.0:
            return 0
        floor = min(int(cfg.min_torque_limit), cap)
        frac = min(depth / cfg.full_force_depth_rad, 1.0)
        return max(1, int(round(floor + (cap - floor) * frac)))

    def held(self) -> List[str]:
        # Stable while held (published on change, transient_local): no depth/load here.
        return [f'{self._names[i]} {j.edge.side} {self._label(j.owner)} (spring, tick '
                f'{j.goal})' for i, j in enumerate(self._j) if j.state == HELD]

    def outside(self) -> List[str]:
        return [f'{self._names[i]} {j.edge.side} {self._label(j.owner)} ({j.reason})'
                for i, j in enumerate(self._j) if j.state == LIMP]

    def arm(self, ticks: Sequence[int]) -> List[str]:
        """Start watching. Writes nothing. Returns notes on joints already beyond the band
        by more than the engage deadband (they get the spring from the next step on)."""
        rad = self._cal.to_radians(ticks)
        self._j = [_Joint() for _ in self._bands]
        self._boundary, self._boundary_key = None, None     # none from before arming
        notes = []
        for i, band in enumerate(self._bands):
            if self._band_violation(i, rad[i]) is not None:
                notes.append(f'{self._names[i]} at {rad[i]:.3f} rad beyond the wall '
                             f'[{band[0]:.3f}, {band[1]:.3f}] at arming: the spring holds it '
                             'from the next tick')
        self.armed = True
        return notes

    def set_boundary(self, edges: Sequence[Optional[Edge]], now: float,
                     session: object, seq: int) -> bool:
        """EXTENSION POINT for the follower's held pose on a rejected setpoint (not fed
        yet). `edges`: per bus id an Edge or None. Applied only if (session, seq) is new:
        a seq not above the last one of the same session is ignored (returns False); a
        new session replaces the boundary whatever its seq."""
        if len(edges) != len(self._bands):
            raise ValueError('one boundary edge (or None) per configured servo required')
        if self._boundary_key is not None and self._boundary_key[0] == session \
                and int(seq) <= self._boundary_key[1]:
            return False
        self._boundary = list(edges)
        self._boundary_key = (session, int(seq))
        self._boundary_rx = now
        return True

    def clear_boundary(self) -> None:
        """The follower accepts setpoints again: the next step releases boundary joints."""
        self._boundary = None

    def step(self, ticks: Sequence[int], now: float) -> List[str]:
        """One tick with a fresh reading at monotonic time `now`. Returns event texts.
        Raises WallFault."""
        if not self.armed:
            return []
        self._monitor()
        rad = self._cal.to_radians(ticks)
        events = self._events = []
        if self._boundary is not None and now - self._boundary_rx > self._cfg.boundary_timeout_s:
            self._boundary = None
            events.append(f'boundary stale (no update for > {self._cfg.boundary_timeout_s} s): '
                          'dropped')
        for i in range(len(self._j)):
            ev = self._step_joint(i, int(ticks[i]), rad[i], now)
            if ev:
                events.append(ev)
        return events

    def release_all(self, reason: str) -> str:
        """Torque off on every leader servo, confirmed. Returns the log text."""
        self.armed = False
        self._j = [_Joint() for _ in self._bands]
        self._boundary, self._boundary_key = None, None
        bad = self._bus.torque_off_verified()
        self.unconfirmed = bad
        if bad:
            return (f'{reason}; torque-off NOT confirmed on servo(s) {bad} - retrying every '
                    'tick, check the leader')
        failed = self._restore_limits()
        return f'{reason}; torque off confirmed' + (
            f'; Torque_Limit restore failed on servo(s) {failed}, retried at the next '
            'release' if failed else '')

    def restore_pending(self) -> List[int]:
        """Disarmed with torque confirmed off: retry the Torque_Limit restores that failed
        earlier. Returns the servo ids still pending (all of them while armed or while a
        torque-off is unconfirmed - nothing is written then)."""
        if self.armed or self.unconfirmed:
            return sorted(self._orig_limit)
        return self._restore_limits()

    def retry_release(self) -> List[int]:
        if self.unconfirmed:
            self.unconfirmed = self._bus.torque_off_verified()
            if not self.unconfirmed:
                self._restore_limits()
        return self.unconfirmed

    # ------------------------------------------------------------------ per joint
    @staticmethod
    def _label(owner: Optional[str]) -> str:
        return 'wall' if owner == BAND else 'boundary'

    @staticmethod
    def _depth(edge: Edge, rad: float) -> float:
        """Depth beyond `edge`'s spring zero; 0 inside or within DEPTH_EPS_RAD of it."""
        d = -edge.inside_zero(rad)
        return d if d > DEPTH_EPS_RAD else 0.0

    def _band_violation(self, i: int, rad: float) -> Optional[Edge]:
        """The band edge the joint is beyond by more than the engage deadband."""
        band = self._bands[i]
        if band is None:
            return None
        for edge in (Edge('lower', band[0]), Edge('upper', band[1])):
            if self._depth(edge, rad) > self._cfg.engage_depth_rad:
                return edge
        return None

    def _boundary_violation(self, i: int, rad: float) -> Optional[Edge]:
        edge = None if self._boundary is None else self._boundary[i]
        return edge if edge is not None and \
            self._depth(edge, rad) > self._cfg.engage_depth_rad else None

    def _step_joint(self, i: int, tick: int, rad: float, now: float) -> Optional[str]:
        j, name = self._j[i], self._names[i]
        band_edge = self._band_violation(i, rad)
        if j.state != FREE and j.owner == BOUNDARY:
            new = None if self._boundary is None else self._boundary[i]
            if band_edge is not None:                      # one owner: the band wins
                if j.state == HELD:
                    return self._retarget(i, BAND, band_edge, tick, rad)
                if j.reason == OVERLOAD:                   # the latch moves, no new push
                    self._j[i] = _Joint(state=LIMP, owner=BAND, edge=band_edge,
                                        reason=OVERLOAD)
                    return (f'{name}: overload latch moved to the {band_edge.side} wall, '
                            'limp until back inside')
                j = self._j[i] = _Joint()
            elif new is None:                              # cleared or stale
                was = j.state
                if was == HELD:
                    self._release_one(i)
                self._j[i] = _Joint()
                return (f'{name}: boundary gone, '
                        + ('released' if was == HELD else 'no longer limp-latched'))
            elif new != j.edge:                            # the held pose moved
                if j.state == LIMP:
                    j.edge = new
                elif self._depth(new, rad) > 0.0:
                    return self._retarget(i, BOUNDARY, new, tick, rad)
                else:                  # inside the moved boundary: its goal would pull out
                    self._release_one(i)
                    self._j[i] = _Joint()
                    return f'{name}: boundary moved past the joint, released'
        if j.state == FREE:
            edge, owner = band_edge, BAND
            if edge is None:
                edge, owner = self._boundary_violation(i, rad), BOUNDARY
            if edge is None:
                return None
            return self._engage(i, owner, edge, tick, rad, now)
        if j.state == HELD and j.owner == BAND and band_edge is not None \
                and band_edge != j.edge:                   # beyond the other band edge
            return self._retarget(i, BAND, band_edge, tick, rad)
        depth = self._depth(j.edge, rad)
        if depth <= 0.0:                                   # back inside: no torque, at once
            if j.state == LIMP:
                self._j[i] = _Joint()
                return f'{name}: back inside, {self._label(j.owner)} active again'
            self._release_one(i)
            self._j[i] = _Joint()
            return f'{name}: moved back inside, {j.edge.side} {self._label(j.owner)} released'
        if j.state == LIMP:
            return None
        return self._hold(i, rad, depth, now)

    def _goal_tick(self, i: int, tick: int, rad: float, edge: Edge) -> int:
        """The HOLD tick: the first tick beyond `edge`'s spring zero by MORE than one tick
        (depth in (1, 2] ticks; 2 ticks for an edge on a tick), from a present reading on
        its side - but never outward of that present reading."""
        out = self._cal.sign[i] * (-1 if edge.side == 'lower' else 1)   # outward, in ticks
        zero = tick + self._cal.sign[i] * (edge.zero - rad) * self._k  # the zero, in ticks
        if out > 0:
            goal = math.floor(zero + 1.0 + HOLD_EPS_TICKS) + 1
            return min(goal, tick)
        goal = math.ceil(zero - 1.0 - HOLD_EPS_TICKS) - 1
        return max(goal, tick)

    def _write_limit(self, i: int, tl: int):
        """Torque is on: a Torque_Limit below 1 is refused (it brakes the STS3215)."""
        j, sid = self._j[i], self._bus.ids[i]
        if tl < 1:
            self._fault(f'{self._names[i]}: Torque_Limit {tl} with torque on refused')
        if tl == j.tl:
            return
        try:
            self._bus.write_torque_limit(sid, tl)
        except LeaderWriteError as exc:
            self._fault(f'{self._names[i]}: Torque_Limit write failed: {exc}')
        j.tl = self._set_limit[sid] = tl

    def _hold(self, i: int, rad: float, depth: float, now: float) -> Optional[str]:
        cfg, j, sid = self._cfg, self._j[i], self._bus.ids[i]
        try:
            j.load = self._bus.read_word(sid, ADDR_PRESENT_LOAD) & LOAD_MAGNITUDE_MASK
        except LeaderReadError as exc:
            self._fault(f'{self._names[i]}: Present_Load read failed: {exc}')
        self._write_limit(i, self.spring(j.cap, depth))
        if j.log_t is None or now - j.log_t >= HELD_LOG_PERIOD_S:
            j.log_t = now
            self._events.append(
                f'{self._names[i]} held at the {j.edge.side} {self._label(j.owner)}: depth '
                f'{depth:.3f} rad ({depth * self._k:.0f} ticks beyond tick {j.goal}), '
                f'Torque_Limit {j.tl}, load {j.load}')
        if j.load < cfg.overload_load:
            j.timer_t0 = None
            return None
        if j.timer_t0 is None or \
                j.edge.inward(rad) - j.edge.inward(j.ref_rad) >= cfg.overload_progress_rad:
            j.timer_t0, j.ref_rad = now, rad
            return None
        if now - j.timer_t0 < cfg.overload_time_s:
            return None
        edge, owner, load, tl = j.edge, j.owner, j.load, j.tl
        self._release_one(i)
        self._j[i] = _Joint(state=LIMP, owner=owner, edge=edge, reason=OVERLOAD)
        return (f'{self._names[i]}: {OVERLOAD} at the {edge.side} {self._label(owner)} - '
                f'loaded {cfg.overload_time_s} s without inward progress ({depth:.3f} rad '
                f'beyond, Torque_Limit {tl}, load {load}): limp until back inside')

    @staticmethod
    def _limits_problem(tick: int, goal: int, lo: int, hi: int) -> Optional[str]:
        """Present and goal inside the servo's position limits. No extra guard: torque is
        on (as of a reading) only beyond the zero, between the goal and the present."""
        if not lo < hi or not lo <= tick <= hi:
            return f'present tick {tick} outside the servo position limits {lo}..{hi}'
        if not lo <= goal <= hi:
            return f'goal tick {goal} outside the servo position limits {lo}..{hi}'
        return None

    def _retarget(self, i: int, owner: str, edge: Edge, tick: int, rad: float) -> str:
        """A HELD joint beyond a different edge (the band took over, or the boundary
        moved): new goal - its hold tick, never outward of the joint - then the
        spring of the new depth."""
        j, name, sid = self._j[i], self._names[i], self._bus.ids[i]
        goal = self._goal_tick(i, tick, rad, edge)
        try:
            lo, hi = (self._bus.read_word(sid, ADDR_MIN_POS_LIMIT),
                      self._bus.read_word(sid, ADDR_MAX_POS_LIMIT))
        except LeaderReadError as exc:
            self._fault(f'{name}: register read before retargeting failed: {exc}')
        why = self._limits_problem(tick, goal, lo, hi)
        if why:
            self._release_one(i)
            self._j[i] = _Joint(state=LIMP, owner=owner, edge=edge, reason='not engaged')
            return f'{name}: {edge.side} {self._label(owner)} NOT engaged ({why}): released'
        if goal != j.goal:
            try:
                self._bus.write_goal(sid, goal)
            except LeaderWriteError as exc:
                self._fault(f'{name}: goal write failed: {exc}')
        j.owner, j.edge, j.goal, j.timer_t0 = owner, edge, goal, None
        self._write_limit(i, self.spring(j.cap, self._depth(edge, rad)))
        return f'{name}: now held by the {edge.side} {self._label(owner)}, hold tick {goal}'

    def _engage(self, i: int, owner: str, edge: Edge, tick: int, rad: float,
                now: float) -> str:
        bus, name = self._bus, self._names[i]
        sid = bus.ids[i]
        goal = self._goal_tick(i, tick, rad, edge)
        depth = self._depth(edge, rad)
        label = f'{edge.side} {self._label(owner)}'
        bus.status_bits.pop(sid, None)
        try:
            torque = bus.read_byte(sid, ADDR_TORQUE)
            mode = bus.read_byte(sid, ADDR_OPERATING_MODE)
            volts = bus.read_byte(sid, ADDR_PRESENT_VOLTAGE)
            lo, hi = bus.read_word(sid, ADDR_MIN_POS_LIMIT), bus.read_word(sid, ADDR_MAX_POS_LIMIT)
            limit = bus.read_word(sid, ADDR_TORQUE_LIMIT)
        except LeaderReadError as exc:
            self._fault(f'{name}: register read before engaging failed: {exc}')
        why = None
        if torque != 0:
            why = f'servo {sid}: torque already ON'
        elif mode != 0:
            why = f'servo {sid}: Operating_Mode {mode}, not 0 (position)'
        elif bus.status_bits.get(sid):
            why = f'servo {sid}: status error bits 0x{bus.status_bits[sid]:02x}'
        else:
            why = self._limits_problem(tick, goal, lo, hi) or self._volts(sid, volts)
        # A restore that failed earlier left the wall's own value (often 0) in the
        # register: the remembered original stays the servo's value, never that.
        orig = self._orig_limit.get(sid, int(limit))
        if not why and orig <= 0:
            why = f'servo {sid}: Torque_Limit {orig}, no force possible'
        if why:
            self._j[i] = _Joint(state=LIMP, owner=owner, edge=edge, reason='not engaged')
            return f'{name}: {label} NOT engaged ({why}): stays limp'
        cap = min(int(self._cfg.torque_limit), orig)
        tl = self.spring(cap, depth)
        self._orig_limit[sid] = orig
        self._set_limit[sid] = tl
        try:
            bus.write_torque_limit(sid, tl)
            bus.write_goal(sid, goal)
            bus.torque_on(sid)
        except LeaderWriteError as exc:
            self._fault(f'{name}: engaging failed: {exc}')
        self._j[i] = _Joint(state=HELD, owner=owner, edge=edge, goal=goal, cap=cap, tl=tl)
        return (f'{name}: {label} engaged at {depth:.3f} rad beyond, hold tick {goal}, '
                f'Torque_Limit {tl} (full {cap} at {self._cfg.full_force_depth_rad} rad)')

    # ------------------------------------------------------------------ internals
    def _fault(self, reason: str):
        raise WallFault(self.release_all(f'wall fault: {reason}'))

    def _volts(self, sid: int, raw: int) -> Optional[str]:
        cfg = self._cfg
        v = raw * cfg.voltage_lsb_v
        if not cfg.voltage_min_v <= v <= cfg.voltage_max_v:
            return (f'servo {sid}: {v:.1f} V outside the band '
                    f'{cfg.voltage_min_v}..{cfg.voltage_max_v} V')
        return None

    def _release_one(self, i: int):
        sid = self._bus.ids[i]
        bad = self._bus.torque_off_verified([sid])
        if bad:
            self._fault(f'{self._names[i]}: torque-off not confirmed on servo {sid}')
        if not self._restore_one(sid):
            self._events.append(f'{self._names[i]}: Torque_Limit restore failed on servo '
                                f'{sid} (torque is off): kept, retried at the next release')

    def _monitor(self):
        """One HELD servo per tick: voltage, Torque_Enable, Torque_Limit, status bits."""
        held = [i for i, j in enumerate(self._j) if j.state == HELD]
        if not held:
            return
        i = held[self._mon % len(held)]
        self._mon += 1
        sid = self._bus.ids[i]
        try:
            volts = self._bus.read_byte(sid, ADDR_PRESENT_VOLTAGE)
            torque = self._bus.read_byte(sid, ADDR_TORQUE)
            limit = self._bus.read_word(sid, ADDR_TORQUE_LIMIT)
        except LeaderReadError as exc:
            self._fault(f'monitor read failed: {exc}')
        why = self._volts(sid, volts)
        if why:
            self._fault(why)
        if torque != 1:
            self._fault(f'servo {sid}: Torque_Enable {torque} while holding (servo reset?)')
        if limit != self._set_limit[sid]:
            self._fault(f'servo {sid}: Torque_Limit {limit}, set {self._set_limit[sid]} '
                        '(servo reset?)')
        if self._bus.status_bits.get(sid):
            self._fault(f'servo {sid}: status error bits 0x{self._bus.status_bits[sid]:02x}')

    def _restore_one(self, sid: int) -> bool:
        """Torque is off. On a failed write the original is KEPT: the register holds the
        wall's own value (the spring's, as low as the floor), which the next engage must
        not take for the servo's own limit (a silently weak wall) - it is retried at the
        next release."""
        self._set_limit.pop(sid, None)
        val = self._orig_limit.get(sid)
        if val is None:
            return True
        try:
            self._bus.write_torque_limit(sid, int(val))
        except (LeaderWriteError, LeaderReadError):
            return False
        self._orig_limit.pop(sid, None)
        return True

    def _restore_limits(self) -> List[int]:
        """Returns the servo ids whose restore failed (kept for the next attempt)."""
        return [sid for sid in list(self._orig_limit) if not self._restore_one(sid)]
