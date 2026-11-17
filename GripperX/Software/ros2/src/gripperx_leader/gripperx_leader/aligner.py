"""Leader self-alignment to the follower's current pose (user decision 2026-10-13, FR-17).

On an align request the leader drives ITSELF to the follower's measured URDF joint
angles, goes limp, and only then is leader mode requested. The follower does not
move for this. Pure python: the bus is injected and `now` is passed in, so the tests
run with a mock bus and a fake clock.

Phases:
  begin()   NOTHING is written. Homing_Offset match first (it guards the enable
            itself). Then DIRECT (user decision 2026-10-15): every leader joint already
            within the follower's leader-mode activation tolerance of the follower
            angle -> nothing moves, the caller requests leader mode at once; the
            motion pre-checks (voltage, torque, travel, status bits) do not apply to
            that path. Otherwise pre-checks (registers read-only, voltage band,
            follower limits, travel band) and the plan. Remembers the present ticks.
  SETTLING  first step() at least still_check_sec later: the leader must not have
            moved meanwhile (someone is handling it -> abort), then per servo
            Torque_Limit (never raised), goal := the FRESH reading, torque on, and a
            read-back: a servo that jumped on torque-on aborts (audit LA1 M2).
  RAMP      linear ramp in the leader's own calibrated radians, all joints arriving
            together, written each tick. Each tick one servo (round robin) is
            monitored: Present_Voltage in band, Torque_Enable 1, Torque_Limit as set
            (a reset servo comes back at its EEPROM value), no status error bits.
  arrival   (only after the final setpoint was written) torque off on every servo,
            confirmed by read-back -> the caller publishes the enable request.
Any abort is torque off on every servo and no enable request. A torque-off that is
not confirmed stays PENDING and is retried by the caller every tick (retry_release).

PATH. The ramp runs from the present reading to the target inside the leader's radian
window [c-pi, c+pi) (calibration.py) and is commanded in continuous ticks. A position
servo with single-turn limits cannot be told to cross its 0/4095 wrap: a goal on the
other side would send it the long way round. A path whose end lies outside the
servo's Min/Max_Position_Limit (normally 0..4095) is therefore REFUSED, never wrapped.

TRAVEL BAND (user decision 2026-10-15, "like the arm"). Per joint the allowed band is
[travel_min + margin, travel_max - margin]. A follower angle outside it is CLAMPED to the
band edge if the leader, once within the arrival tolerance of that edge, is still within
the activation tolerance of the follower (|edge - follower| + arrival <= activation), so
the follower can accept leader mode after arrival; farther outside it is refused. A
joint already within the arrival tolerance of the follower angle (decided first, before
any clamp - audit LA4 M1) or of its clamped target does not move: its
goal is its fresh reading, it gets no band or window check (its present angle may lie
outside the band), only the servo position-limit check, because torque comes on for
every joint during the alignment and a goal outside those limits would be clamped by
the servo firmware - a motion.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from gripperx_leader.leader_bus import (ADDR_PRESENT_VOLTAGE, ADDR_TORQUE, ADDR_TORQUE_LIMIT,
                                        LeaderReadError, LeaderWriteError)

TWO_PI = 2.0 * math.pi
IDLE, SETTLING, RAMP = 'idle', 'settling', 'ramp'


class AlignRefused(RuntimeError):
    """Nothing is moving: the request was turned down before torque came on."""


class AlignAbort(RuntimeError):
    """A running alignment was stopped; torque off has already been attempted."""


@dataclass
class AlignConfig:
    # Design parameters, all TO-VERIFY on the leader except where noted.
    max_joint_velocity_rad_s: float = 0.3     # given in the task 2026-10-13
    # Torque_Limit register (48, SRAM), 0..1000; LeRobot so_follower writes 500 as
    # "50% of max torque". 300 for the first test (audit LA1 L4). Never raised above
    # the servo's present value. TO-VERIFY.
    torque_limit: Optional[int] = 300
    deviation_tolerance_rad: float = 0.1      # present vs commanded setpoint, TO-VERIFY
    arrival_tolerance_rad: float = 0.035      # TO-VERIFY; must stay below
    # activation_tolerance_rad (checked in begin)
    settle_timeout_sec: float = 3.0           # after the ramp's end, TO-VERIFY
    follower_pose_timeout_sec: float = 1.5    # TO-VERIFY; 3 periods of the 2 Hz relay
    follower_motion_tolerance_rad: float = 0.05   # TO-VERIFY
    still_check_sec: float = 0.1              # leader must be still this long, TO-VERIFY
    still_tolerance_rad: float = 0.02         # TO-VERIFY
    torque_on_tolerance_rad: float = 0.02     # present vs goal right after torque-on, TO-VERIFY
    travel_margin_rad: float = 0.05           # keep-out from the recorded stops, TO-VERIFY
    # The follower's leader-mode activation match (arm_action_server
    # leader.activation_match_tolerance_rad, gripperx_arm/config/arm_safety_filter.yaml).
    # Never a number of its own: the node reads it from that file (follower_config.py).
    # None = unknown -> every align request refused.
    activation_tolerance_rad: Optional[float] = None
    # Mandatory band on Present_Voltage (unset -> every alignment refused).
    voltage_min_v: float = 6.5                # TO-VERIFY (7.4 V servos)
    voltage_max_v: float = 8.4                # TO-VERIFY (2S full charge)
    voltage_lsb_v: float = 0.1                # Present_Voltage unit, TO-VERIFY


def _fmt(vals) -> str:
    return '[' + ', '.join(f'{v:.3f}' for v in vals) + ']'


def _bits(bits: dict) -> str:
    return ', '.join(f'servo {s}: 0x{b:02x}' for s, b in sorted(bits.items()))


class Aligner:
    def __init__(self, bus, cal, cfg: AlignConfig, names: Sequence[str],
                 follower_limits: Optional[Sequence[Tuple[float, float]]] = None):
        self._bus = bus
        self._cal = cal
        self._cfg = cfg
        self._names = list(names)
        self._follower_limits = follower_limits
        self._k = cal.ticks_per_rev / TWO_PI
        self.phase = IDLE
        self.direct = False
        self.unconfirmed: List[int] = []      # torque-off not yet confirmed (retried)
        self._orig_limit = {}
        self._set_limit = {}
        self._mon = 0
        self.clamped: List[str] = []          # per-joint clamp notes of the last plan

    @property
    def active(self) -> bool:
        return self.phase != IDLE

    # ------------------------------------------------------------------ checks
    def _volts_ok(self, sid, raw) -> Optional[str]:
        cfg = self._cfg
        volts = raw * cfg.voltage_lsb_v
        if not cfg.voltage_min_v <= volts <= cfg.voltage_max_v:
            return (f'servo {sid}: {volts:.1f} V outside the configured band '
                    f'{cfg.voltage_min_v}..{cfg.voltage_max_v} V')
        return None

    def check_offsets(self, diag: dict) -> None:
        """The calibration's ticks are only valid with the Homing_Offsets they were
        measured with: guards the enable itself, so also the direct path."""
        for i, sid in enumerate(self._bus.ids):
            d = diag.get(sid) or {}
            rec = None if self._cal.homing_offset_raw is None else self._cal.homing_offset_raw[i]
            if rec is None:
                raise AlignRefused(f'servo {sid}: calibration has no homing_offset_raw - the '
                                   'offset its ticks were measured with is unknown')
            if d.get('offset') != rec:
                raise AlignRefused(
                    f'servo {sid}: Homing_Offset {d.get("offset")} != calibration '
                    f'homing_offset_raw {rec}: the calibration does not match the EEPROM '
                    '(README "Leader EEPROM step": update zero_ticks + homing_offset_raw '
                    'together, or roll back with leader_eeprom --restore)')

    def activation_tolerance(self) -> float:
        cfg = self._cfg
        act = cfg.activation_tolerance_rad
        if act is None or not act > 0.0:
            raise AlignRefused('leader-mode activation tolerance unknown (arm_safety_filter.yaml '
                               'leader.activation_match_tolerance_rad not read)')
        if not cfg.arrival_tolerance_rad < act:
            raise AlignRefused(f'align_arrival_tolerance_rad {cfg.arrival_tolerance_rad} not below '
                               f'the activation tolerance {act}')
        return float(act)

    def within_activation(self, present_ticks: Sequence[int],
                          follower_q: Sequence[float]) -> bool:
        """Every leader joint within the activation tolerance of the follower angle.
        Plain differences, as the follower's own match (arm_action_server._match)."""
        act = self.activation_tolerance()
        rad = self._cal.to_radians(present_ticks)
        return all(abs(r - float(q)) <= act for r, q in zip(rad, follower_q))

    def check_registers(self, diag: dict) -> None:
        """Refuse on anything the read-only registers say is unsafe or unknown."""
        cfg = self._cfg
        if not 0.0 < cfg.voltage_min_v < cfg.voltage_max_v:
            raise AlignRefused(f'align_voltage_min_v/max_v not set '
                               f'({cfg.voltage_min_v}/{cfg.voltage_max_v}): mandatory')
        if cfg.torque_limit is None or not 0 < int(cfg.torque_limit) <= 1000:
            raise AlignRefused(f'align_torque_limit {cfg.torque_limit} not set or '
                               'outside 1..1000')
        self.check_offsets(diag)
        for sid in self._bus.ids:
            d = diag.get(sid) or {}
            missing = [k for k in ('voltage', 'min_voltage', 'max_voltage', 'mode', 'min_pos',
                                   'max_pos', 'torque_limit', 'torque') if d.get(k) is None]
            if missing:
                raise AlignRefused(f'servo {sid}: registers unreadable: {missing}')
            if d['mode'] != 0:
                raise AlignRefused(f'servo {sid}: Operating_Mode {d["mode"]}, not 0 (position)')
            if d['torque'] != 0:
                raise AlignRefused(f'servo {sid}: torque already ON - the leader must be limp')
            if not d['min_voltage'] <= d['voltage'] <= d['max_voltage']:
                raise AlignRefused(
                    f'servo {sid}: Present_Voltage {d["voltage"]} outside the servo\'s own '
                    f'Min/Max_Voltage_Limit {d["min_voltage"]}..{d["max_voltage"]} (raw)')
            why = self._volts_ok(sid, d['voltage'])
            if why:
                raise AlignRefused(why)
            if d['min_pos'] >= d['max_pos']:
                raise AlignRefused(
                    f'servo {sid}: position limits {d["min_pos"]}..{d["max_pos"]} disabled or '
                    'invalid (multi-turn?) - refusing to command it')
        bits = {s: b for s, b in self._bus.status_bits.items() if b}
        if bits:
            raise AlignRefused(f'servo status error bits set: {_bits(bits)}')

    def plan(self, present_ticks: Sequence[int], follower_q: Sequence[float],
             diag: dict) -> List[float]:
        """Per joint the continuous end tick of the ramp (= the present tick for a joint
        that does not move). Raises AlignRefused. Notes clamped joints in self.clamped."""
        cal, cfg = self._cal, self._cfg
        act = self.activation_tolerance()
        present_rad = cal.to_radians(present_ticks)
        ends, clamped = [], []
        for i, sid in enumerate(self._bus.ids):
            name = self._names[i]
            target = float(follower_q[i])
            if self._follower_limits is None:
                raise AlignRefused('follower joint limits unknown')
            f_lo, f_hi = self._follower_limits[i]
            if not f_lo <= target <= f_hi:
                raise AlignRefused(f'{name}: follower angle {target:.3f} rad outside the '
                                   f'follower joint limits [{f_lo:.3f}, {f_hi:.3f}]')
            lo_r = None if cal.travel_min_rad is None else cal.travel_min_rad[i]
            hi_r = None if cal.travel_max_rad is None else cal.travel_max_rad[i]
            if lo_r is None or hi_r is None:
                raise AlignRefused(f'{name}: no recorded leader travel (travel_min_rad/'
                                   'travel_max_rad) in the calibration file')
            m = cfg.travel_margin_rad
            b_lo, b_hi = lo_r + m, hi_r - m
            if b_lo > b_hi:
                raise AlignRefused(f'{name}: recorded leader travel [{lo_r:.3f}, {hi_r:.3f}] '
                                   f'narrower than twice the {m} rad margin')
            lo, hi = diag[sid]['min_pos'], diag[sid]['max_pos']
            if not lo <= present_ticks[i] <= hi:
                raise AlignRefused(f'{name}: present tick {present_ticks[i]} outside the '
                                   f'servo position limits {lo}..{hi}')
            if abs(target - present_rad[i]) <= cfg.arrival_tolerance_rad:
                ends.append(float(present_ticks[i]))        # at the follower: holds, no clamp
                continue
            goal = min(max(target, b_lo), b_hi)
            if goal != target:
                off = abs(goal - target)
                if off + cfg.arrival_tolerance_rad > act:
                    raise AlignRefused(
                        f'{name}: follower angle {target:.3f} rad is {off:.3f} rad outside the '
                        f'leader band [{b_lo:.3f}, {b_hi:.3f}] (recorded travel [{lo_r:.3f}, '
                        f'{hi_r:.3f}] minus the {m} rad margin): too far to clamp, the band '
                        f'edge plus the {cfg.arrival_tolerance_rad} rad arrival tolerance must '
                        f'stay within the {act:.3f} rad activation tolerance')
            if abs(goal - present_rad[i]) <= cfg.arrival_tolerance_rad:
                ends.append(float(present_ticks[i]))        # at the band edge: holds
                continue
            if goal != target:
                clamped.append(f'{name} {target:.3f} -> {goal:.3f}')
            c = cal.centers()[i]
            if not c - math.pi <= goal < c + math.pi:
                raise AlignRefused(
                    f'{name}: target {goal:.3f} rad outside the leader window '
                    f'[{c - math.pi:.3f}, {c + math.pi:.3f})')
            delta = goal - present_rad[i]
            end = present_ticks[i] + cal.sign[i] * delta * self._k
            if not lo <= round(end) <= hi:
                raise AlignRefused(
                    f'{name}: the path to the follower pose ends at tick {end:.0f}, outside '
                    f'the servo position limits {lo}..{hi} (it would cross the 0/4095 wrap '
                    'or a limit; the servo would take the long way)')
            ends.append(end)
        self.clamped = clamped
        return ends

    # ------------------------------------------------------------------ run
    def begin(self, now: float, present_ticks: Sequence[int], follower_q: Sequence[float],
              diag: dict) -> bool:
        """Pre-check and plan; writes NOTHING. False = nothing to move (self.direct True:
        within the activation tolerance on every joint; else within the arrival
        tolerance after clamping) - the caller requests leader mode at once; True =
        SETTLING started, torque comes on in a later step(). Raises AlignRefused."""
        self.direct = False
        self.clamped = []
        if self.unconfirmed:
            raise AlignRefused(f'torque-off of an earlier run still unconfirmed on servo(s) '
                               f'{self.unconfirmed}')
        self.check_offsets(diag)
        if self.within_activation(present_ticks, follower_q):
            self.direct = True
            return False
        self.check_registers(diag)
        ends = self.plan(present_ticks, follower_q, diag)
        dist = [abs(e - p) / self._k for e, p in zip(ends, present_ticks)]
        if max(dist) <= self._cfg.arrival_tolerance_rad:
            return False
        self._diag = diag
        self._orig_limit = {}
        self._still_ref = [float(p) for p in present_ticks]
        self._follower_ref = [float(q) for q in follower_q]
        self._t_begin = now
        self.phase = SETTLING
        return True

    @property
    def duration(self) -> float:
        return self._duration

    def step(self, now: float, present_ticks: Sequence[int], follower_q: Sequence[float],
             follower_rx: float) -> bool:
        """One tick. True = arrived and torque confirmed off (caller requests leader
        mode); False = still running. Raises AlignAbort (torque already off)."""
        cfg = self._cfg
        if now - follower_rx > cfg.follower_pose_timeout_sec:
            self._abort(f'follower pose stale (> {cfg.follower_pose_timeout_sec} s)')
        moved = [abs(a - b) for a, b in zip(follower_q, self._follower_ref)]
        if max(moved) > cfg.follower_motion_tolerance_rad:
            self._abort(f'follower moved during the alignment ({_fmt(moved)} rad, tolerance '
                        f'{cfg.follower_motion_tolerance_rad})')
        if self.phase == SETTLING:
            return self._settle(now, present_ticks, follower_q)
        self._monitor()
        dev = [abs(p - s) / self._k for p, s in zip(present_ticks, self._setpoint)]
        if max(dev) > cfg.deviation_tolerance_rad:
            self._abort(f'leader deviates from the commanded ramp ({_fmt(dev)} rad, tolerance '
                        f'{cfg.deviation_tolerance_rad}): held, pushed or obstructed')
        elapsed = now - self._t0
        if self._final_sent:
            off = [abs(p - e) / self._k for p, e in zip(present_ticks, self._ends)]
            if max(off) <= cfg.arrival_tolerance_rad:
                bad = self._release()
                if bad:
                    raise AlignAbort(f'arrived, but torque-off NOT confirmed on servo(s) {bad} '
                                     '(retrying every tick): leader mode not requested')
                return True
            if elapsed > self._duration + cfg.settle_timeout_sec:
                self._abort(f'timeout: not within {cfg.arrival_tolerance_rad} rad '
                            f'{cfg.settle_timeout_sec} s after the ramp ({_fmt(off)} rad)')
            return False
        frac = min(1.0, elapsed / self._duration)
        self._setpoint = [s + (e - s) * frac for s, e in zip(self._start_ticks, self._ends)]
        try:
            for sid, sp in zip(self._bus.ids, self._setpoint):
                self._bus.write_goal(sid, int(round(sp)))
        except LeaderWriteError as exc:
            self._abort(f'goal write failed: {exc}')
        self._final_sent = frac >= 1.0
        return False

    def abort(self, reason: str) -> str:
        """External abort (Space, J, O/I/P, read error, shutdown). Returns the log text."""
        return self._stop(reason)

    def retry_release(self) -> List[int]:
        """Retry an unconfirmed torque-off (audit LA1 M1). Returns the ids still
        unconfirmed; empty = confirmed now."""
        if self.unconfirmed:
            self.unconfirmed = self._bus.torque_off_verified()
            if not self.unconfirmed:
                self._restore_limits()
        return self.unconfirmed

    # ------------------------------------------------------------------ internals
    def _settle(self, now, present_ticks, follower_q) -> bool:
        cfg = self._cfg
        if now - self._t_begin < cfg.still_check_sec:
            return False
        moved = [abs(p - r) / self._k for p, r in zip(present_ticks, self._still_ref)]
        if max(moved) > cfg.still_tolerance_rad:
            self._abort(f'leader is being handled: moved {_fmt(moved)} rad within '
                        f'{cfg.still_check_sec} s before torque-on (tolerance '
                        f'{cfg.still_tolerance_rad})')
        try:
            ends = self.plan(present_ticks, follower_q, self._diag)
        except AlignRefused as exc:
            self._abort(str(exc))
        self._start_ticks = [float(p) for p in present_ticks]
        self._ends = ends
        self._setpoint = list(self._start_ticks)
        self._final_sent = False
        dist = [abs(e - p) / self._k for e, p in zip(ends, present_ticks)]
        self._duration = max(max(dist) / float(cfg.max_joint_velocity_rad_s), 1e-6)
        self._orig_limit = {sid: self._diag[sid]['torque_limit'] for sid in self._bus.ids}
        self._set_limit = {sid: min(int(cfg.torque_limit), int(v))
                           for sid, v in self._orig_limit.items()}
        self._bus.status_bits.clear()
        self.phase = RAMP
        try:
            for i, sid in enumerate(self._bus.ids):
                self._bus.write_torque_limit(sid, self._set_limit[sid])
                self._bus.write_goal(sid, int(present_ticks[i]))      # goal := fresh present
                self._bus.torque_on(sid)
            after = self._bus.read_positions()
        except (LeaderWriteError, LeaderReadError) as exc:
            self._abort(f'torque-on sequence failed: {exc}')
        jump = [abs(a - p) / self._k for a, p in zip(after, present_ticks)]
        if max(jump) > cfg.torque_on_tolerance_rad:
            self._abort(f'leader moved on torque-on ({_fmt(jump)} rad, tolerance '
                        f'{cfg.torque_on_tolerance_rad})')
        self._t0 = now
        return False

    def _monitor(self):
        """One servo per tick: 3 single reads (voltage, torque enable, torque limit)."""
        ids = self._bus.ids
        sid = ids[self._mon % len(ids)]
        self._mon += 1
        try:
            volts = self._bus.read_byte(sid, ADDR_PRESENT_VOLTAGE)
            torque = self._bus.read_byte(sid, ADDR_TORQUE)
            limit = self._bus.read_word(sid, ADDR_TORQUE_LIMIT)
        except LeaderReadError as exc:
            self._abort(f'monitor read failed: {exc}')
        why = self._volts_ok(sid, volts)
        if why:
            self._abort(why)
        if torque != 1:
            self._abort(f'servo {sid}: Torque_Enable {torque} during the ramp (servo reset?)')
        if limit != self._set_limit[sid]:
            self._abort(f'servo {sid}: Torque_Limit {limit}, set {self._set_limit[sid]} '
                        '(servo reset?)')
        bits = {s: b for s, b in self._bus.status_bits.items() if b}
        if bits:
            self._abort(f'servo status error bits: {_bits(bits)}')

    def _abort(self, reason: str):
        raise AlignAbort(self._stop(reason))

    def _restore_limits(self):
        for sid, val in self._orig_limit.items():      # best effort, torque is off
            try:
                self._bus.write_torque_limit(sid, int(val))
            except (LeaderWriteError, LeaderReadError):
                pass

    def _release(self) -> List[int]:
        self.phase = IDLE
        bad = self._bus.torque_off_verified()
        self.unconfirmed = bad
        if not bad:
            self._restore_limits()
        return bad

    def _stop(self, reason: str) -> str:
        bad = self._release()
        if bad:
            return (f'{reason}; torque-off NOT confirmed on servo(s) {bad} - retrying every '
                    'tick, check the leader')
        return f'{reason}; torque off confirmed'
