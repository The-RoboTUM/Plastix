#!/usr/bin/env python3
"""ARM action server — the single writer to the arm bus (SR-17 rule 9).

Sources of arm setpoints:
  * /pick_plastic        fixed pick trajectory (FR-3), raw Feetech tick poses; with
                         `aimed` the grasp pose is computed for a target (x, y) at P's
                         grip height, gripper vertical (FR-16 stage 1, _aim_* below)
  * /arm/command         go_home, open_gripper
  * leader mode (FR-17)  /arm/leader/joint_states from the laptop's leader arm, all six
                         joints (the gripper like limb1..limb5, user decision 2026-11-13)
                         per-tick diagnostics out on /arm/leader/follow_debug
                         (leader.follow_debug_enabled; read-only, best effort)

Every ARM setpoint passes the SR-17 safety filter (gripperx_arm.safety_filter):
the arm parts of the pick sequence, go_home and the return home after an abort are
planned through gripperx_arm.planner (direct, joint-ordered detours, RRT-Connect;
no path = no motion), leader setpoints are checked one by one. Without the filter,
its configuration (arm_safety_filter.yaml) or the model, P and I are REFUSED
(SR-17 rule 12). SR-17 rule 11 (user decision 2026-10-08, option a): while
planner.automatic_paths is direct_only (the default), AUTOMATICALLY triggered arm
motions - the return home after an abort, the re-plan from the measured pose inside a
running pick, a pick triggered by the external-goal gateway, startup homing - take
the direct path or none; detours stay for P and I commanded by the operator. Every
planned path is converted to servo ticks and validated
(tick wrap included) before its first setpoint is written; the planner validates the
setpoints the executor writes (whole ticks, gripperx_arm.exec_steps), not the ideal
joint-linear line. Gripper-only moves
(open_gripper, the open/close steps of the pick) are evaluated and logged but not
blocked by the filter (user decision 2026-10-07); the L2 joint limit still applies
to their targets. The audit log marks them gripper_unchecked.

Stops (user decisions 2026-10-07):
  * /arm/stop (Space): arm MOVING -> torque OFF on all arm servos (limp);
    arm stationary -> no arm action. A later torque-on first sets every goal to the
    present position, so re-enabling never jumps. Space also wins over every O/I/P
    and J RECEIVED before it that has not claimed the arm yet (DDS receipt order,
    _rx_time): refused, nothing written. P (every sender: teleop and gateway) is
    ordered causally (user decision 2026-10-12, "Space must always abort P"): the
    goal carries the stop epoch its sender last saw on /arm/stop_epoch; one that is
    not current is refused, at acceptance and atomically with the claim, and every
    _prepare / torque-on of a claimed motion re-checks the epoch.
  * every other abort (action cancel, gateway disarm, leader watchdog, leader
    disabled via J, ...) -> the arm returns home on a planned, checked path, after a
    short grace in which a Space stop or an O/I/P command takes precedence; no path
    -> the arm holds and reports (SR-17 rule 13).
  * a refusal after a planned motion has started (e.g. the return path from the
    measured grip pose is blocked) -> the arm holds at its measured position and
    the refusal is reported; it is never a silent stop.
  * a filter rejection of ONE leader setpoint is not an abort: that setpoint is
    not written (the arm stays at the last approved one), leader mode continues.
  * leader setpoints are kept leader.limit_command_margin_ticks inside the joint
    limits (user decision 2026-10-15, gripperx_arm.leader_band): an arm commanded to a
    limit sags past it and ends leader mode by the measured-pose check (SR-17 rule 8).
  * near a band edge the leader step brakes (leader.edge_slowdown_*, user decision
    2026-10-16): a servo arriving there at full speed overshot past the limit.

Recovery from outside the joint limits (user decision 2026-10-09, option a): an arm
measured outside its L2 limits (gravity after a restart) is brought back by the go-home
motions only - I and the automatic return home (still direct path only, SR-17 rule 11)
- on a path along which every joint outside moves monotonically back toward its limit
and then stays inside (planner recovery=True). P and leader activation are REFUSED
while the arm is outside, with the advice to press I first.
"""

import collections
import faulthandler
import math
import os
import secrets
import sys
import threading
import time
import traceback
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray, Int64, String

from gripperx_arm_msgs.action import PickPlastic
from gripperx_arm_msgs.msg import LeaderFollowDebug
from gripperx_arm_msgs.msg import ArmState
from gripperx_arm import aimed_grasp as aimed_mod
from gripperx_arm import poses as poses_mod
from gripperx_arm.exec_steps import (
    check_soft_start, exec_poses, exec_poses_at, monotone_cover, ramp_count,
    soft_start_fractions, tick_ramp, ticks_at, unwrapped_ticks, velocity_count)
from gripperx_arm.planner import PlanResult, Planner
from gripperx_arm.kinematics import ArmKinematics, TickCalibration, wrap
from gripperx_arm.leader_band import (
    braked_fractions, check_edge_slowdown, command_band, step_scale)
from gripperx_arm.servo_bus import (
    ADDR_MAX_TORQUE_LIMIT, ADDR_TORQUE_LIMIT, TORQUE_LIMIT_MAX)
from gripperx_arm.safety_filter import (
    FilterConfig, SafetyFilter, Verdict, describe_outside, load_acm, load_link_hulls,
    mechanical_steer_ranges, unwrap_steer_reading)

SERVO_PORT  = '/dev/arm_servo'
SPIN_ESCAPE_LIMIT = 20         # spin_guarded: escapes tolerated within the window
SPIN_ESCAPE_WINDOW_S = 5.0
SPIN_RESUME_BACKOFF_S = 0.05
BAUDRATE    = 1_000_000
IDS         = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = ['limb1', 'limb2', 'limb3', 'limb4', 'limb5', 'gripper']
GRIPPER_ID  = 6
LIMB_IDS    = [1, 2, 3, 4, 5]
# Leader mode (FR-17) drives all six joints, the gripper exactly like limb1..limb5 (user
# decision 2026-11-13): band, activation match, checks, lag gate, re-plans.
LEADER_IDS  = IDS
TICK_RANGE  = 4096

POS_TOLERANCE   = 80   # ticks — target position is considered reached within this value

# 2 Hz instead of 20 Hz: each tick reads 6 servos blocking over serial (the SDK
# busy-polls without sleep); at 20 Hz this saturates a full CPU core.
JOINT_STATE_RATE_HZ = 2.0

# These Feetech servos ignore MOVE_TIME/GOAL_SPEED in practice and always move
# at a fixed internal speed. Speed is therefore enforced via a software ramp:
# many small intermediate setpoints instead of a single jump to the target
# position.
#
# CONSEQUENCE, worth knowing before shortening these times further: step_delay is
# 1/MOVE_STEP_HZ regardless of duration, so a shorter move_time means FEWER setpoints
# over the same travel, not faster setpoints. The servo still runs at its own fixed
# internal speed. Once the setpoints advance faster than the servo can follow, the
# ramp stops setting the speed and _move() simply returns while the arm is still
# catching up — _verify_positions would then warn (it only warns, it does not fail
# the action). The wait_extra_ms settle times are the margin against that.
MOVE_STEP_HZ = 30.0

# No fallback poses in code (removed 2026-10-07; they predated the current assembly):
# home = arm_calibration.yaml zero_ticks, grip / gripper.open / gripper.close from
# config/arm_poses.yaml, as ticks or as angles relative to home (gripperx_arm.poses).

# L2 joint limits (SR-17 rule 3): ONLY gripperx_geometry/config/geometry.yaml, through
# the URDF. The tick-valued joint_limits.min/max of arm_poses.yaml were removed
# (a hand-maintained second copy is forbidden by rule 3).

HOME_MOVE_MS = 4267

LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# The enable REQUEST is read VOLATILE on purpose: a True latched by a teleop before
# this node started must never activate leader mode - only a request published
# while this node runs counts (FR-17 item 3).
ENABLE_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
# Per-tick leader diagnostics (/arm/leader/follow_debug): a lost sample is fine, a
# blocked publish in the tick is not.
DEBUG_QOS = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)

LEADER_OFF, LEADER_ACTIVE = 'off', 'active'
# LeaderFollowDebug phase fields (<name>_s), in tick order; other_s is the remainder.
DEBUG_PHASES = ('refusal', 'read', 'envelope', 'replan', 'target', 'check', 'write')


def tick_deviation(a: int, b: int) -> int:
    """a - b in servo register ticks, the short way round the 0/4095 wrap."""
    return (int(a) - int(b) + TICK_RANGE // 2) % TICK_RANGE - TICK_RANGE // 2


def lag_bound_ok(tick: int, measured: int, ref: int, floor: Optional[int], noise: int,
                 strict: bool = False) -> bool:
    """The lag-growth gate's bound on one limb goal (user decision D3 (2026-10-23)): for
    EVERY arm position m within `noise` ticks of the reading `measured`, |tick - m| is at
    most |ref - m| (`strict`: below it) - with a `floor` (a joint the gate does not
    hold), at most max(|ref - m|, floor). Judged over the band, not at the reading
    alone: where a goal crosses the arm, one tick of reading noise moves the comparison
    by two ticks and could pass a goal that moves away from the arm.
    The goal also may not CROSS the arm (user decisions 2026-10-29, and 2026-10-30 for
    every joint while the gate is closed: the servo's force reverses on any joint): at no
    m in the band on the other side of it from `ref` - a goal landing exactly on the
    reading excepted. With a floor, a `ref` itself within `noise` of the reading has no
    known side (nor a force to reverse): such a joint keeps its floor, or a joint settled
    on its goal could not move at all while another joint holds the gate closed."""
    sided = floor is None or abs(tick_deviation(ref, measured)) > noise
    for m in range(measured - noise, measured + noise + 1):
        dt, dr = tick_deviation(tick, m), tick_deviation(ref, m)
        d, r = abs(dt), abs(dr)
        if sided and dt * dr < 0 and tick != measured:
            return False
        if floor is not None:
            r = max(r, floor)
        if d > r or (strict and d == r):
            return False
    return True


def _rx_time(info=None) -> Tuple[float, bool]:
    """Receipt time of a message, on time.monotonic(). Space must win over every
    operator command RECEIVED before it, and the order in which the callbacks start
    is not that order (audit r8 W2): a command can wait in the DDS queue, or for a
    free executor thread, while a later Space is handled. The DDS reception stamp
    (MessageInfo received_timestamp, system clock) is taken when the sample arrives,
    independent of the executor; it is moved onto the monotonic clock over the short
    queue delay only, so a system clock step elsewhere does not bias it. Without a
    stamp (rmw without one, a direct call) the callback start is used - that is the
    old, weaker order. Returns (time, stamped)."""
    now = time.monotonic()
    rx = (info or {}).get('received_timestamp') or 0
    if rx <= 0:
        return now, False
    return now - max(0.0, (time.time_ns() - rx) * 1e-9), True


def _share(pkg: str, *rel: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(pkg), *rel)


def _resolve_mesh(uri: str) -> str:
    if uri.startswith('package://'):
        pkg, rel = uri[len('package://'):].split('/', 1)
        return _share(pkg, rel)
    if uri.startswith('file://'):
        return uri[len('file://'):]
    return uri


class MotionRefused(Exception):
    pass


class HeldPoseInvalid(MotionRefused):
    """A held arm has no safe start: the goals it holds fail the envelope even as a static
    pose, and starting at the measured pose would re-anchor it (_held_start)."""


class MotionStopped(MotionRefused):
    """Space: torque went off, nothing else follows."""


class MotionAborted(MotionRefused):
    """Cancel/abort: the caller schedules the return home."""


class BusWriteFailed(MotionRefused):
    """A goal write some servo did not acknowledge (FeetechBus returns False on a
    communication failure; only OS errors raise). `failed`: the servo ids."""

    def __init__(self, text: str, failed):
        super().__init__(text)
        self.failed = list(failed)


class GroundSagHold(MotionRefused):
    """The ground-recovery lift measured a rising link below its start clearance (or
    could not measure it): no further write, torque stays on, the last written goal
    stays - re-writing the measured pose would lower the arm again. No _hold: nothing
    at all is written after the stop."""


class BusLost(OSError):
    """The arm bus is disconnected (_BusGuard): every bus call fails at once."""


class _BusGuard:
    """The servo bus as the node uses it (2026-10-13 18:09: the arm's USB serial
    adapter re-enumerated, every call failed with an I/O error and the node never
    reopened the port). The first OSError of any call (pyserial's SerialException is
    one) marks the bus LOST and is re-raised as before; from then on every call raises
    BusLost at once - nothing waits on a dead device - until _reconnect_tick reopens
    it. Attributes are looked up on the real bus at every call, so a test may replace
    a method of the bus it holds.

    `switch_s` (bus.python_switch_interval_s): the interpreter's switch interval for the
    length of each bus call, the normal one back after it. Only then: process-wide it
    let the leader tick take the search's share too (search 5x slower, 2026-11-02)."""

    def __init__(self, raw, on_lost):
        self.raw = raw
        self.lost: Optional[str] = None
        self._on_lost = on_lost
        self.switch_s: Optional[float] = None
        self._normal_switch = sys.getswitchinterval()

    def __getattr__(self, name):
        attr = getattr(self.raw, name)
        if not callable(attr) or name in ('open', 'close', 'reopen'):
            return attr

        def call(*args, **kwargs):
            # A torque-OFF is always attempted on the real bus: one servo's I/O error
            # must not spare the others (_torque_off_locked tries every servo).
            off = name == 'torque' and len(args) > 1 and not args[1]
            if self.lost is not None and not off:
                raise BusLost(f'arm bus disconnected ({self.lost})')
            if self.switch_s is not None:
                sys.setswitchinterval(self.switch_s)
            try:
                return attr(*args, **kwargs)
            except OSError as exc:
                if self.lost is None:
                    self.lost = repr(exc)
                    self._on_lost(exc)
                raise
            finally:
                # the constant, not the value found: two overlapping calls (Space during
                # a batch) must not restore each other's short interval for good
                sys.setswitchinterval(self._normal_switch)
        return call


class ReturnBlocked(MotionRefused):
    """The return home of a running motion has no executable path: hold and report
    (SR-17 rule 13), no further attempt."""


def _path_chain(planner: Planner, start, path, arm, env):
    """(setpoints after the start, rising entries per step, refusal or None) of a
    planned leader path re-anchored at `start`: the first segment with its soft start,
    every segment while a link still rises re-checked with check_setpoint's trace (the
    per-step rising entries execution uses), the rest as planned. Pure: run by the
    search thread (job.start, job.env) or by the tick."""
    pts = [start] + [{**start, **{n: p[n] for n in arm}} for p in path[1:]]
    seq, rising_at = [], []
    rising: tuple = ()
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        poses = planner.executed(a, b, False, first=i == 0)
        if i == 0 or rising:
            trace: list = []
            why = planner.check_segment(a, b, env, first=i == 0, rising=rising, trace=trace)
            if why is not None:
                where = 'first segment' if i == 0 else f'segment {i + 1}'
                return seq, rising_at, f'{where} from the held pose: {why}'
            if len(trace) != len(poses):
                return seq, rising_at, ('the rising check did not cover every setpoint '
                                        f'({len(trace)} of {len(poses)})')
            rising_at += trace[:-1]
            rising = trace[-1]
        else:
            rising_at += [()] * (len(poses) - 1)
        seq += poses[1:]
    return seq, rising_at, None


def _resumed_search(planner: Planner, start, target, env, max_s: float, on_slice=None):
    """(PlanResult, stalled): planner.plan() in slices of planner.budget, a slice cut by
    the time cap RESUMED (the planner replays the verdicts already computed, so the
    checks and the path are those of one uncut search - only the wall time grows). Ends
    with a path, any bound other than 'time' (checks, cancelled, ...), when a slice got
    no further at all (stalled), or once the slices together have used `max_s`
    (planner.max_search_s; bound 'search_time', no path - fails closed). The total is
    the sum of the slices' own cost_ms, i.e. measured on the planner's clock; the
    overrun is at most one slice. `on_slice(n)` after each slice."""
    spent, slices = 0.0, 0
    while True:
        res = planner.plan(start, target, env)
        slices += 1
        spent += res.cost_ms / 1e3
        if on_slice is not None:
            on_slice(slices)
        if res.ok or res.bound != 'time':
            return res, False
        if planner.fresh_checks == 0 and planner.fresh_poses == 0:
            return res, True
        if spent >= max_s:
            res.bound = 'search_time'
            res.reason = (f'search cap planner.max_search_s = {max_s:.1f} s used up after '
                          f'{slices} slice(s) of {planner.budget:.2f} s, resumed: '
                          f'{res.reason}')
            return res, False

class AimRefused(MotionRefused):
    """An aimed P refused, with its reason code (gripperx_arm.aimed_grasp)."""

    def __init__(self, reason: str, text: str):
        super().__init__(f'{reason}: {text}')
        self.reason = reason


class BaseMovedAbort(MotionAborted):
    """SR-16 E5: the base moved (or its rest could no longer be shown) DURING an aimed
    P. Aborted like a cancel - the caller schedules the return home (SR-17 rule 13) -
    but with its own reason."""

    def __init__(self, text: str):
        super().__init__(f'{aimed_mod.BASE_MOVED_DURING_PICK}: {text}')
        self.reason = aimed_mod.BASE_MOVED_DURING_PICK


class _AimContext:
    """One aimed P: the target in base_link, the candidates, and the base-still
    reference taken at the start (_aim_start), re-checked by _aim_still."""

    def __init__(self, target_base, frame, candidates, still_ref, steer_ref):
        self.target_base = target_base       # (x, y, z) in the base frame
        self.frame = frame                   # the frame the goal named
        self.candidates = candidates         # [aimed_grasp.Candidate], static-checked
        self.still_ref = still_ref           # (stamp_sec, x, y, yaw) of base in still_frame
        self.steer_ref = steer_ref           # {wheel: reading or None}


class _AimPlan:
    """The planned motions of one aimed P (_aim_approach): the candidate, pre-grasp
    goal ticks, the approach to it (PlanResult) and its end ticks, the grasp goal
    ticks and the grasp end ticks after the descent."""

    def __init__(self, cand, pre, approach, at_pre, grip, at_grip):
        self.cand, self.pre, self.approach, self.at_pre = cand, pre, approach, at_pre
        self.grip, self.at_grip = grip, at_grip


class _ReplanJob:
    """One search for a path from the held arm to the leader, run in its own thread so
    the leader tick keeps its rate (watchdog, refusals, measured-pose watch). The
    thread only computes: it never touches the bus or node state; the tick adopts the
    result after re-checking it from the then measured pose.

    The search runs in slices of the planner's time budget; a slice cut by the time cap
    is RESUMED (Planner remembers the verdicts), not restarted. It ends with a path,
    with planner.max_checks used up (the bound on the whole search), when a slice
    got no further at all (not one pose of a check), with `max_s` of search time used
    (planner.max_search_s, audit 2026-11-02 M2: a failed search, the arm holds), or on
    cancel() (_resumed_search). A slice may end
    inside a check: one leader path check takes seconds under load, and a check longer
    than a slice counted as a failed search for a valid path (replayed 2026-11-02)."""

    def __init__(self, planner: Planner, start, target, env, cal, max_s: float = math.inf):
        self.planner, self.start, self.target, self.env = planner, start, target, env
        self.cal = cal                       # the calibration the planner was built with
        self.max_s = max_s
        self.result: Optional[PlanResult] = None
        self.slices = 0                      # written by the thread, read by the tick
        self.stalled = False
        self.elapsed_s = 0.0
        self.reported_slices = 0             # tick side: progress already published
        self.measured_ticks: Optional[Dict[int, int]] = None   # tick side: at the start
        self.why_failed = 'no path to the leader'               # tick side: for the hold line
        # (start, envelope, _path_chain result) for a found path, from this thread
        self.chain = None
        self.t0 = time.monotonic()
        self.thread = threading.Thread(target=self._run, name='leader_replan', daemon=True)

    def _run(self):
        try:
            res, self.stalled = _resumed_search(self.planner, self.start, self.target,
                                                self.env, self.max_s,
                                                lambda n: setattr(self, 'slices', n))
        except Exception as exc:             # never leave the tick waiting forever
            res = PlanResult(False, reason=f'planner error: {exc!r}')
        if res.ok:
            try:                             # the tick re-checks itself without it
                self.chain = (dict(self.start), self.env,
                              _path_chain(self.planner, self.start, res.path,
                                          self.planner.joints, self.env))
            except Exception:                # noqa: BLE001
                self.chain = None
        self.elapsed_s = time.monotonic() - self.t0
        self.result = res                    # last: the tick reads chain after result

    def cancel(self):
        self.planner.cancel()


class ArmActionServer(Node):
    def __init__(self, bus=None, robot_description: Optional[str] = None,
                 mesh_resolver=None, **kwargs):
        super().__init__('arm_action_server', **kwargs)
        self._cb_group = ReentrantCallbackGroup()
        self._leader_group = MutuallyExclusiveCallbackGroup()
        self._serial_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._motion_owner: Optional[str] = None   # 'pick'|'command'|'leader'|'abort_home'
        self._leader_state = LEADER_OFF
        self._stop = threading.Event()             # set by /arm/stop while moving
        self._op_planner: Optional[Planner] = None  # the operator's I search: Space cancels
        self._stop_count = 0                       # every /arm/stop, moving or not
        self._stop_rx = -math.inf                  # _rx_time of the newest Space handled
        self._cmd_waiting = False                  # an operator command waits for leader off
        self._rx_warned = False                    # missing DDS receipt stamp reported once
        self._goal_rx: Dict[int, tuple] = {}       # id(goal) -> (goal, _rx_time at _goal_cb)
        # Stop epoch (user decision 2026-10-12, "Space must always abort P"): base +
        # handled Spaces, published latched on /arm/stop_epoch. A P goal carries the
        # epoch its sender last saw; any other value is refused. The random base makes a
        # value from before a respawn invalid too (0 = not filled, never valid).
        self._epoch_base = 1 + secrets.randbelow(2 ** 62)
        self._claim_epoch: Optional[int] = None    # epoch when the current owner claimed
        self._leaving = 0                          # _leave calls between OFF and their end
        self._dead = False                         # spin_guarded gave up: no further writes
        self._last_motion = -math.inf              # monotonic time of the last motion write
        self._pending_home: Optional[tuple] = None  # (deadline, reason)
        self._torque_on = False
        # The last goal each servo acknowledged (register ticks); None = unknown (a
        # write not acknowledged or interrupted, or torque off since). _write keeps it,
        # under the serial lock; _write_seq counts the writes.
        self._goal: Dict[int, Optional[int]] = {sid: None for sid in IDS}
        self._write_seq = 0
        self._held_memo = None                     # _held_start's last kept-goal verdict
        # Leader gripper torque limit (FR-17 amendment 2026-11-16): the Torque_Limit servo 6
        # held before leader mode wrote its own value - kept while the leader value may
        # still be in the servo, None once the previous one is back (_restore_grip_limit).
        self._grip_limit_prev: Optional[int] = None
        self._grip_driven = False                  # this leader session drives the gripper
        self._grip_restore_t = -math.inf           # last retry from the leader tick
        self._grip_restore_why: Optional[str] = None   # last restore failure reported
        # servos whose last torque-off was not acknowledged (or raised): they may still
        # hold torque and push; every Space retries them (audit HG2 R1)
        self._torque_stuck: Dict[int, str] = {}
        # A Space whose torque-off could not take the serial lock within
        # stop.lock_timeout_sec (a bus transaction hangs): its reason. Applied by the
        # next bus access (_write, _ensure_torque), by _torque_off_pending_tick and by
        # every later Space.
        self._torque_off_pending: Optional[str] = None
        self._last_seen: Optional[Dict[int, int]] = None   # last complete position read
        self._read_misses: Dict[int, int] = {sid: 0 for sid in IDS}  # failed position reads
        self._reads_done = 0                       # all position reads
        self._read_why = threading.local()         # .text: why this thread's _read_all failed
        self._models: Dict[int, Optional[int]] = {}
        self._mesh_resolver = mesh_resolver or _resolve_mesh
        # Every bus write, in order: (kind, {sid: ticks}). Goal writes: 'approve'
        # (filter-approved arm setpoint: planned path or leader step),
        # 'gripper_unchecked' (gripper-only move), 'hold' (the servos a hold re-anchors
        # at their measured position - possibly none: the others keep their goal,
        # user decision 2026-10-13), 'torque_on_sync'
        # (goal = present before torque-on). Torque writes: 'torque_on'/'torque_off'
        # with {sid: 1|0}.
        self.audit = collections.deque(maxlen=20000)
        self.status_log = collections.deque(maxlen=200)

        self._declare_and_read_parameters()
        self._declare_aimed_parameters()

        if bus is None:
            from gripperx_arm.servo_bus import FeetechBus
            bus = FeetechBus(SERVO_PORT, BAUDRATE)
            bus.open()
        self._bus = _BusGuard(bus, self._on_bus_lost)
        self._bus.switch_s = self._bus_switch
        self._bus_lost_reported = -math.inf
        self._bus_lost_why: Optional[str] = None     # last reported reconnect failure

        self._status_pub = self.create_publisher(String, '/arm/safety_filter/status', LATCHED)
        self._epoch_pub = self.create_publisher(Int64, '/arm/stop_epoch', LATCHED)
        self._publish_epoch()
        self._leader_state_pub = self.create_publisher(String, '/arm/leader/state', LATCHED)

        self._log_eeprom_limits()
        self._grip_limit_at_start()
        try:
            self._ensure_torque()
            self.get_logger().info(f'Arm connected on {SERVO_PORT}, Torque ON (goals synced)')
        except MotionRefused as exc:
            self.get_logger().error(f'Torque NOT enabled at start: {exc}')

        # ---- safety filter
        self._filter: Optional[SafetyFilter] = None
        self._cal: Optional[TickCalibration] = None
        self._cal_verified = False
        self._filter_error = 'robot description not received yet'
        self._steer_names = list(self._p('safety_filter.steer_joint_names'))
        self._steer: Dict[str, Optional[float]] = {}
        self._steer_rx = -math.inf
        self._steer_env_key = None
        self._steer_env = None
        if robot_description is not None:
            self._build_filter(robot_description)
        else:
            self.create_subscription(String, self._p('safety_filter.robot_description_topic'),
                                     self._on_robot_description, LATCHED,
                                     callback_group=self._cb_group)
        if self._wheel_mode == 'measured':
            self.create_subscription(Float64MultiArray, self._p('safety_filter.steer_states_topic'),
                                     self._on_steer_states, 10, callback_group=self._cb_group)

        # Startup homing is a real movement the operator has not asked for, and it runs
        # on every respawn. It must be switchable off while the home pose is being
        # re-taught, otherwise each bringup drives the arm onto stale tick values.
        if self._home_on_startup and self._filter is None:
            # The filter is built when /robot_description arrives, i.e. after this
            # constructor: a planned home move is impossible here. Not deferred on
            # purpose - startup homing is unattended motion (arm_poses.yaml).
            self.get_logger().error(
                'Startup homing requested (home_on_startup=true) but NOT performed: the '
                f'safety filter is not built yet ({self._filter_error}); P and I are planned '
                'through it. Send "go_home" on /arm/command once it is READY.')
        elif self._home_on_startup:
            self.get_logger().info('Startup: moving to home position...')
            try:
                self._run('command', lambda: self._arm_home('startup_home'), 'startup_home')
            except MotionRefused as exc:
                self.get_logger().error(f'Startup homing refused: {exc}')
        else:
            self.get_logger().warn(
                'Startup homing DISABLED (home_on_startup=false) — arm stays where it is. '
                'Send "go_home" on /arm/command when the home pose is confirmed.'
            )

        self._joint_pub = self.create_publisher(JointState, '/arm/joint_states', 10)
        # Its own group (2026-10-13 robot event): a bus read that hangs can then occupy
        # at most ONE executor worker, not all four (the 2 Hz timer re-fired into a
        # reentrant group while the read hung, and Space found no thread).
        self._state_pub = self.create_publisher(ArmState, '/arm/state', LATCHED)
        self.create_timer(1.0 / JOINT_STATE_RATE_HZ, self._publish_joint_states,
                          callback_group=MutuallyExclusiveCallbackGroup())

        # Two-argument callbacks: rclpy then passes the MessageInfo (_rx_time).
        self.create_subscription(String, '/arm/command',
                                 lambda msg, info: self._on_command(msg, info),
                                 10, callback_group=self._cb_group)
        self.create_timer(0.05, self._pending_home_tick, callback_group=self._cb_group)
        self.create_timer(0.05, self._torque_off_pending_tick,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(self._reconnect_period, self._reconnect_tick,
                          callback_group=MutuallyExclusiveCallbackGroup())

        self._setup_leader()
        self._setup_aimed()

        self._action_server = ActionServer(
            self, PickPlastic, 'pick_plastic',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            'arm_action_server ready — Action: /pick_plastic | Topic: /arm/command, /arm/stop '
            '| leader mode: /arm/leader/* | safety filter: '
            + ('READY' if self._filter else f'NOT READY ({self._filter_error})')
        )
        # Last: its thread runs _on_stop, which reads state set up above (AP2 (3)).
        self._setup_stop_listener()

    # ================================================================ parameters
    def _p(self, name):
        return self.get_parameter(name).value

    def _declare_and_read_parameters(self):
        """Poses come from config/arm_poses.yaml, the filter and leader settings from
        config/arm_safety_filter.yaml, which explains every TO-VERIFY value. The code
        defaults below mirror that file so the node can start without it - except
        safety_filter.body_hull_stage (default off) and body_hull_clearance_m (default
        infinite, so the hull stage can never clear a box overlap): without the file
        they fail closed to the box verdict."""
        # Either variant may be absent: declared typed, without a value.
        self.declare_parameter('poses.grip', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('poses.grip_rad', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('poses.home', Parameter.Type.INTEGER_ARRAY)   # refused if set
        for key in ('open', 'close'):
            self.declare_parameter(f'gripper.{key}', Parameter.Type.INTEGER)
            self.declare_parameter(f'gripper.{key}_rad', Parameter.Type.DOUBLE)
        self.declare_parameter('home_on_startup', False)

        sf = {
            'config_loaded': False,
            'robot_description_topic': '/robot_description',
            'calibration_file': '',
            'acm_file': '',
            'stop_protected_file': '',
            'steer_config_file': '',
            'steer_joint_names': ['f_left_steer', 'f_right_steer', 'b_leftsteer', 'b_rightsteer'],
            'ground_frame': 'base_footprint',
            'ground_clearance_tip_m': 0.01,
            'ground_clearance_links_m': 0.01,
            'tip_links': ['arm_limb5_link', 'arm_gripper_link'],
            'body_clearance_m': 0.0,
            'self_clearance_m': 0.0,
            'max_joint_step_rad': 0.0349,
            'mesh_hull_stage': True,
            'body_hull_stage': False,                 # unset: box verdict final
            'body_hull_clearance_m': math.inf,       # unset: the hull stage never clears
            'wheel_envelope': 'measured',
            'steer_states_topic': '/hw/steer_states',
            'steer_state_timeout_sec': 0.5,
            'steer_motion_margin_rad': 0.0873,
            'allow_unverified_calibration': False,
            'self_contact_recovery': 'operator_go_home',
            'self_contact_dip_m': 0.0,
            'ground_recovery': 'operator',
            'ground_recovery_sag_tolerance_m': 0.00075,
            'ground_recovery_start_drift_ticks': 2,
        }
        for k, v in sf.items():
            self.declare_parameter(f'safety_filter.{k}', v)
        ld = {
            'rate_hz': 30.0,
            'watchdog_timeout_sec': 0.3,
            'activation_match_tolerance_rad': 0.174533,
            'replan_repeat_tolerance_rad': 0.10,
            'max_joint_velocity_rad_s': 0.3,
            'command_tolerance_rad': 0.35,
            'calibration_verified_topic': '/arm/leader/calibration_verified',
            'arming_state_topic': '/gripperx/external/arming_state',
            'command_wait_sec': 0.3,
            'limit_command_margin_ticks': 10,
            'edge_slowdown_zone_rad': 0.15,
            'edge_slowdown_velocity_rad_s': 0.2,
            'follow_debug_enabled': True,
            'hold_status_period_sec': 5.0,
            'replan_discard_move_ticks': 2,
            'replan_max_goal_lag_growth_ticks': 4,
            'replan_lag_gate_noise_ticks': 1,
        }
        for k, v in ld.items():
            self.declare_parameter(f'leader.{k}', v)
        self.declare_parameter('abort.home_grace_sec', 0.5)
        self.declare_parameter('hold.contact_threshold_ticks', 100)
        self.declare_parameter('stop.lock_timeout_sec', 0.1)
        self.declare_parameter('bus.reconnect_period_sec', 1.0)
        self.declare_parameter('bus.python_switch_interval_s', 0.0005)
        pl = {
            'time_budget_s': 5.0,
            'max_search_s': 30.0,
            'start_drift_ticks': 2,
            'max_checks': 600,
            'rrt_step_rad': 0.25,
            'seed': 0,
            'max_joint_velocity_rad_s': 0.5,
            'automatic_paths': 'direct_only',
            'soft_start_step_fractions': [0.05, 0.15, 0.3, 0.5, 0.75],
        }
        for k, v in pl.items():
            self.declare_parameter(f'planner.{k}', v)
        self.declare_parameter('leader.replan_period_sec', 0.5)
        # Declared typed WITHOUT a value (FR-17 amendment, user decisions 2026-11-16): unset =
        # the gripper is driven at the servo's own Torque_Limit, like limb1..limb5 (no limit
        # is written for any servo); a measured, lower value (TO-VERIFY) is the hook.
        self.declare_parameter('leader.gripper_torque_limit', Parameter.Type.INTEGER)
        self.declare_parameter('eeprom_check', 'log')
        self.declare_parameter('eeprom_margin_min_ticks', [0] * len(IDS))
        self.declare_parameter('eeprom_margin_max_ticks', [0] * len(IDS))
        # /arm/state at_home: limb1..limb5 only (false, USER DECISION 2026-10-13, audit
        # AP3 F1 variant a: the jaws may hold the litter) or with the gripper (true).
        # Startup-only.
        self.declare_parameter('arm_state.at_home_includes_gripper', False)
        self._home_with_gripper = bool(self._p('arm_state.at_home_includes_gripper'))

        self._resolve_fixed_poses()
        self._home_on_startup = bool(self._p('home_on_startup'))
        self._wheel_mode = str(self._p('safety_filter.wheel_envelope'))
        if self._wheel_mode not in ('swept', 'measured'):
            raise RuntimeError(f'safety_filter.wheel_envelope must be swept|measured, '
                               f'not {self._wheel_mode!r}')
        self._allow_unverified = bool(self._p('safety_filter.allow_unverified_calibration'))
        self._grace = float(self._p('abort.home_grace_sec'))
        self._stop_lock_timeout = float(self._p('stop.lock_timeout_sec'))
        self._reconnect_period = float(self._p('bus.reconnect_period_sec'))
        switch = float(self._p('bus.python_switch_interval_s'))
        if switch <= 0.0:
            raise RuntimeError(f'bus.python_switch_interval_s must be > 0, not {switch}')
        self._bus_switch = switch            # _BusGuard.switch_s
        self._contact_ticks = int(self._p('hold.contact_threshold_ticks'))
        if not 0 < self._contact_ticks < TICK_RANGE // 2:
            raise RuntimeError('hold.contact_threshold_ticks must lie in (0, '
                               f'{TICK_RANGE // 2}), not {self._contact_ticks}')
        self._automatic_paths = str(self._p('planner.automatic_paths'))
        self._self_contact_mode = str(self._p('safety_filter.self_contact_recovery'))
        if self._self_contact_mode not in ('operator_go_home', 'off'):
            raise RuntimeError('safety_filter.self_contact_recovery must be '
                               f'operator_go_home|off, not {self._self_contact_mode!r}')
        # operator: the operator's I and leader re-plans (user decision 2026-10-20);
        # operator_go_home: the operator's I only (the 2026-10-12 scope); off: neither.
        self._ground_mode = str(self._p('safety_filter.ground_recovery'))
        if self._ground_mode not in ('operator', 'operator_go_home', 'off'):
            raise RuntimeError('safety_filter.ground_recovery must be '
                               f'operator|operator_go_home|off, not {self._ground_mode!r}')
        try:
            self._soft_start = check_soft_start(self._p('planner.soft_start_step_fractions'))
        except ValueError as exc:
            raise RuntimeError(f'planner.soft_start_step_fractions: {exc}') from exc
        self._discard_move_ticks = int(self._p('leader.replan_discard_move_ticks'))
        self._max_lag_growth = int(self._p('leader.replan_max_goal_lag_growth_ticks'))
        if self._max_lag_growth < 0:
            raise RuntimeError('leader.replan_max_goal_lag_growth_ticks must be >= 0')
        noise = self._p('leader.replan_lag_gate_noise_ticks')
        if isinstance(noise, bool) or not isinstance(noise, int) \
                or not 0 < noise < self._max_lag_growth:
            # 0 would switch the crossing protection off, a huge value stall the tick
            # (lag_bound_ok walks the band), a float be truncated
            raise RuntimeError('leader.replan_lag_gate_noise_ticks must be an integer with '
                               '0 < it < leader.replan_max_goal_lag_growth_ticks '
                               f'({self._max_lag_growth}), not {noise!r}')
        self._lag_noise = noise
        if self._discard_move_ticks < 0:
            raise RuntimeError('leader.replan_discard_move_ticks must be >= 0')
        self._grip_limit = self._opt('leader.gripper_torque_limit')
        if self._grip_limit is not None and (
                isinstance(self._grip_limit, bool) or not isinstance(self._grip_limit, int)
                or not 0 < self._grip_limit <= TORQUE_LIMIT_MAX):
            # 0 would make the servo brake (gripperx_leader, robot 2026-10-20)
            raise RuntimeError('leader.gripper_torque_limit must be unset or an integer in '
                               f'1..{TORQUE_LIMIT_MAX}, not {self._grip_limit!r}')
        self._hold_status_period = float(self._p('leader.hold_status_period_sec'))
        if not self._hold_status_period > 0.0:
            raise RuntimeError('leader.hold_status_period_sec must be > 0')
        if self._automatic_paths not in ('direct_only', 'detour'):
            raise RuntimeError('planner.automatic_paths must be direct_only|detour, not '
                               f'{self._automatic_paths!r}')
        text = (f'SR-17 rule 11: planner.automatic_paths = {self._automatic_paths} - '
                'automatically triggered arm motions (return home after an abort, re-plan '
                'from the measured pose, gateway-triggered P, startup homing) ')
        if self._automatic_paths == 'direct_only':
            text += 'take the DIRECT path only; detours only for operator P and I'
            self.get_logger().info(text)
        else:
            text += ('MAY TAKE DETOURS (joint-ordered, RRT-Connect) - not the user decision '
                     'of 2026-10-08 (option a)')
            self.get_logger().warn(text)
        self.status_log.append(text)       # the status topic does not exist yet
        self._eeprom_mode = str(self._p('eeprom_check'))
        if self._eeprom_mode not in ('log', 'enforce'):
            raise RuntimeError('eeprom_check must be log|enforce')
        self._eeprom_ok = self._eeprom_mode == 'log'
        self._config_loaded = bool(self._p('safety_filter.config_loaded'))
        if not self._config_loaded:
            self.get_logger().error(
                'arm_safety_filter.yaml NOT LOADED (safety_filter.config_loaded is false): '
                'running on code defaults. Leader mode, P (pick) and I (go_home) are '
                'refused; only gripper moves (O) run. Fix the launch file.')

        # Sequence: (step_name, positions_dict, move_time_ms, wait_extra_ms)
        #
        # The six move_time values (5067 / 4267 / 4000 / 2400 / 2400 / 1333 ms) were
        # scaled AS A SET from a slower baseline; if the speed is changed again, change
        # them together, or the sequence loses its shape.
        #
        # grip_close is deliberately slower than the others (2400 ms): a short move_time
        # forces the servo to reach the target quickly, which means a higher current
        # spike while gripping, and slower movement reduces the current draw. Current
        # draw at 2400 ms HAS NOT BEEN MEASURED. If gripping starts browning out the rail
        # or tripping the servo, this is the first number to put back.
        #
        # wait_extra_ms is NOT scaled: it is settle time, not motion. It is the margin that
        # lets the servos catch up before _verify_positions reads them, and shorter moves
        # need that margin more, not less. (_execute_cb runs the steps; this table is the
        # record of the timing set.)
        if self._poses_error:
            self._pick_sequence = None
        else:
            self._pick_sequence = [
                ('approach',    {**self._grip_pos, GRIPPER_ID: self._grip_open}, 4000, 800),
                ('grip_close',  {GRIPPER_ID: self._grip_close},                  2400, 700),
                ('return_home', dict(self._home_pos),                            HOME_MOVE_MS, 900),
            ]

    def _opt(self, name):
        try:
            return self.get_parameter(name).value
        except Exception:        # noqa: BLE001 - uninitialized typed parameter = absent
            return None

    def _resolve_fixed_poses(self):
        """home from the calibration (one source), grip/open/close as ticks or angles."""
        self._home_pos = self._grip_pos = self._grip_open = self._grip_close = None
        self._poses_error = ''
        try:
            with open(self._file('safety_filter.calibration_file', 'gripperx_arm', 'config',
                                 'arm_calibration.yaml')) as f:
                cal = poses_mod.calibration_from_doc(yaml.safe_load(f))
            doc = {'poses': {k: self._opt(f'poses.{k}') for k in ('grip', 'grip_rad')
                             if self._opt(f'poses.{k}') is not None},
                   'gripper': {k: self._opt(f'gripper.{k}')
                               for k in ('open', 'open_rad', 'close', 'close_rad')
                               if self._opt(f'gripper.{k}') is not None}}
            if self._opt('poses.home') is not None:
                doc['poses']['home'] = self._opt('poses.home')
            r = poses_mod.resolve(doc, cal)
        except Exception as exc:     # noqa: BLE001 - reported, fixed poses then refused
            self._poses_error = f'fixed poses unavailable: {exc}'
            self.get_logger().error(self._poses_error)
            return
        self._home_pos = dict(zip(IDS, r['home']))
        self._grip_pos = dict(zip(IDS, r['grip']))
        self._grip_open, self._grip_close = r['open'], r['close']
        self.get_logger().info(f'fixed poses: home = calibration zero_ticks {r["home"]}; '
                               f'sources {r["sources"]}')

    # ================================================================ filter setup
    def _on_robot_description(self, msg: String):
        if self._filter is None:
            self._build_filter(msg.data)

    def _file(self, param: str, pkg: str, *rel: str) -> str:
        path = str(self._p(param))
        return path if path else _share(pkg, *rel)

    def _build_filter(self, urdf_xml: str):
        """Everything the filter needs, or a reason why not. Missing data = refusal."""
        try:
            kin = ArmKinematics(urdf_xml)
            with open(self._file('safety_filter.calibration_file', 'gripperx_arm', 'config',
                                 'arm_calibration.yaml')) as f:
                c = yaml.safe_load(f)['arm_joint_state_relay']['ros__parameters']
            if list(c['servo_names']) != JOINT_NAMES:
                raise ValueError(f'calibration servo_names {c["servo_names"]} != {JOINT_NAMES}')
            if list(c['urdf_joints']) != kin.joint_names:
                raise ValueError(f'calibration joints {c["urdf_joints"]} != URDF {kin.joint_names}')
            cal = TickCalibration(list(c['urdf_joints']), [float(v) for v in c['zero_ticks']],
                                  [int(v) for v in c['signs']], float(c['ticks_per_rev']),
                                  [float(v) for v in c.get('range_center_rad',
                                                           [0.0] * len(c['urdf_joints']))])
            for n, ctr in zip(cal.joints, cal.range_center):
                lo, hi = kin.limits[n]
                if lo < ctr - math.pi - 1e-6 or hi > ctr + math.pi + 1e-6:
                    raise ValueError(f'{n}: joint limits [{lo:.3f}, {hi:.3f}] do not fit the '
                                     f'angle window {ctr:.3f} +- pi (range_center_rad)')
            stop_doc = None
            try:
                with open(self._file('safety_filter.stop_protected_file', 'gripperx_arm',
                                     'config', 'arm_stop_protected.yaml')) as f:
                    stop_doc = yaml.safe_load(f)
            except OSError as exc:
                self.get_logger().warn(f'no stop-protected pairs ({exc}): all checked')
            with open(self._file('safety_filter.acm_file', 'gripperx_arm', 'config',
                                 'arm_allowed_collisions.yaml')) as f:
                acm = load_acm(yaml.safe_load(f), stop_doc)
            with open(self._file('safety_filter.steer_config_file', 'gripperx_control', 'config',
                                 'steer_servo.yaml')) as f:
                st = yaml.safe_load(f)['steer_servo_node']['ros__parameters']
            sign = [int(v) for v in st['steering_outward_sign']]
            out_deg = [float(v) for v in st['steering_mechanical_outward_deg']]
            in_deg = [float(v) for v in st['steering_mechanical_inward_deg']]
            ranges = mechanical_steer_ranges(kin, self._steer_names, sign, out_deg, in_deg)
            cfg = FilterConfig.from_params(lambda n: self._p(f'safety_filter.{n}'))
            hulls = (load_link_hulls(urdf_xml, kin, self._mesh_resolver)
                     if cfg.mesh_hull_stage or cfg.body_hull_stage else None)
            flt = SafetyFilter(kin, acm, cfg, ranges, str(self._p('safety_filter.ground_frame')),
                               hulls=hulls)
        except Exception as exc:     # noqa: BLE001 - any failure is a refusal with a reason
            self._filter_error = f'safety filter unavailable: {exc}'
            self.get_logger().error(self._filter_error)
            self._report(self._filter_error)
            return
        self._steer_sign = sign
        self._steer_out = [math.radians(v) for v in out_deg]
        self._steer_in = [math.radians(v) for v in in_deg]
        self._cal, self._cal_verified = cal, bool(c.get('calibration_verified', False))
        self._filter = flt
        from gripperx_arm.eeprom_limits import recorded_offsets
        try:
            self._recorded_offsets, self._offsets_error = recorded_offsets(c), ''
        except SystemExit as exc:          # malformed entry: enforce refuses, log warns
            self._recorded_offsets, self._offsets_error = None, str(exc)
            self.get_logger().warn(self._offsets_error)
        self._verify_eeprom(kin, cal)
        self._filter_error = ''
        msg = (f'safety filter READY (wheel envelope: {self._wheel_mode}; mesh hull stage: '
               f'{len(flt.hulls)} links; P/I/return home: planned and checked; gripper '
               f'moves: evaluated, not blocked; calibration ')
        if self._cal_verified:
            self.get_logger().info(msg + 'verified)')
        elif self._allow_unverified:
            msg += 'UNVERIFIED, allow_unverified_calibration=true: leader mode allowed)'
            self.get_logger().warn(msg)
        else:
            msg += 'UNVERIFIED: leader mode refused)'
            self.get_logger().warn(msg)
        self._report(msg)

    def _report(self, text: str):
        # Called from error handlers: must not raise (a publish fails at shutdown).
        self.status_log.append(text)       # also for tests
        try:
            self._status_pub.publish(String(data=text))
        except Exception as exc:     # noqa: BLE001
            self.get_logger().error(f'status publish failed ({exc!r}): {text}')

    # ================================================================ conversions
    def _q_of(self, ticks: Dict[int, int]) -> Dict[str, float]:
        return self._cal.to_radians([ticks[sid] for sid in IDS])

    def _unwrapped_ticks(self, q: Dict[str, float], cal=None) -> Dict[int, int]:
        return dict(zip(IDS, unwrapped_ticks(cal or self._cal, q)))

    def _wrap_reason(self, qa: Dict[str, float], qb: Dict[str, float],
                     sids=IDS, cal=None) -> Optional[str]:
        """None if the joint-linear motion qa -> qb is what the servos do: a ramp is
        LINEAR IN REGISTER TICKS, which equals the joint-linear motion exactly when
        both ends lie in the same turn of the unwrapped tick axis. Otherwise the
        motion crosses the 0/4095 register wrap and the servo would turn the other
        way round through poses nobody checked. Moving the wrap out of the joint's
        travel is arm_wrap_offset's job."""
        cal = cal or self._cal
        ua, ub = self._unwrapped_ticks(qa, cal), self._unwrapped_ticks(qb, cal)
        for sid in sids:
            if ua[sid] // TICK_RANGE != ub[sid] // TICK_RANGE:
                n = cal.joints[IDS.index(sid)]
                return (f'{n}: {ua[sid] % TICK_RANGE} -> {ub[sid] % TICK_RANGE} crosses the '
                        'servo tick wrap (the servo would turn the long way)')
        return None

    def _ticks_of(self, q: Dict[str, float],
                  ref_q: Optional[Dict[str, float]] = None) -> Dict[int, int]:
        """Servo goals for a pose (single-turn register: modulo one turn). With `ref_q`
        (the pose the motion starts from): refused unless the motion between them is
        executable as checked (_wrap_reason) - never written."""
        if ref_q is not None:
            why = self._wrap_reason(ref_q, q)
            if why:
                raise MotionRefused(why)
        return {sid: t % TICK_RANGE for sid, t in self._unwrapped_ticks(q).items()}

    def _read_all(self) -> Optional[Dict[int, int]]:
        out, misses = {}, []
        self._read_why.text = ''
        with self._serial_lock:
            for sid in IDS:
                if self._pending_first():
                    self._read_why.text = 'broken off: a pending Space torque-off comes first'
                    return None
                out[sid] = self._read_pos(sid, misses)
        if any(v is None for v in out.values()):
            self._read_why.text = '; '.join(misses)
            return None
        self._last_seen = dict(out)
        return out

    def _read_failure(self) -> str:
        """Why the calling thread's last _read_all returned None. Per thread and per call:
        the tick, a command and the joint-state timer read concurrently, and none may
        report another's miss."""
        return getattr(self._read_why, 'text', '')

    def _read_pos(self, sid: int, misses: Optional[list] = None) -> Optional[int]:
        """One servo's present position, a miss counted and named. Caller holds the
        serial lock. Which servo failed, and why, was nowhere (robot run 2026-11-02:
        'no measured arm pose' and nothing else). `misses`: the miss text is added here."""
        t = time.monotonic()
        v = self._bus.read_pos(sid)
        self._reads_done += 1
        if v is None:
            self._read_misses[sid] += 1
            why = getattr(self._bus, 'last_error', None) or 'no answer'
            text = f'servo {sid}: {why} after {(time.monotonic() - t) * 1e3:.0f} ms'
            if misses is not None:
                misses.append(text)
            self.get_logger().warn(
                f'arm bus read miss, {text}; misses per servo {self._read_misses} in '
                f'{self._reads_done} reads', throttle_duration_sec=1.0)
        return v

    def _pending_first(self) -> bool:
        """Caller holds the serial lock, between two servo transactions of a batch
        (audit HG3 H1: on a degraded bus one transaction may take servo_bus.TXN_MAX_S,
        and a Space waiting for the lock met a whole 6-servo batch first). A pending
        Space torque-off is applied right here; True = the batch must end now."""
        if self._torque_off_pending is None:
            return False
        if self._bus.lost is None:
            self._apply_torque_off_pending()
        return True

    def _envelope(self):
        """None = the filter's default, wheels swept over the MECHANICAL stops."""
        if self._wheel_mode != 'measured':
            return None
        timeout = float(self._p('safety_filter.steer_state_timeout_sec'))
        with self._state_lock:
            fresh = time.monotonic() - self._steer_rx <= timeout
            steer = dict(self._steer) if fresh else {}
        known = {n: steer.get(n) for n in self._steer_names}
        if all(v is None for v in known.values()):
            return None
        key = tuple(None if v is None else round(v, 2) for v in known.values())
        with self._state_lock:            # pick, command and leader threads share the cache
            if key != self._steer_env_key:
                self._steer_env = self._filter.envelope_measured(
                    known, float(self._p('safety_filter.steer_motion_margin_rad')))
                self._steer_env_key = key
            return self._steer_env

    def _on_steer_states(self, msg: Float64MultiArray):
        """Plausibility, as far as it is detectable here: a reading outside the
        mechanical stops (after unwrapping) is rejected, and an exact 0.0 is treated as
        UNKNOWN because steer_servo_node publishes 0.0 for a dropped channel - a wheel
        truly at centre is then swept too (conservative). NOT detectable: the node
        publishes the COMMANDED angle for a servo whose read failed; that echo looks
        like a measurement."""
        if self._filter is None or len(msg.data) != len(self._steer_names):
            return
        margin = float(self._p('safety_filter.steer_motion_margin_rad'))
        vals = {}
        for i, n in enumerate(self._steer_names):
            r = float(msg.data[i])
            vals[n] = None if r == 0.0 else unwrap_steer_reading(
                r, self._steer_sign[i], self._steer_out[i], self._steer_in[i], margin)
        with self._state_lock:
            self._steer = vals
            self._steer_rx = time.monotonic()

    # ================================================================ writing
    def _write(self, kind: str, setpoint: Dict[int, int]):
        """The ONLY place that writes a goal position. Caller holds _serial_lock.
        Every servo is written; then BusWriteFailed if any did not acknowledge (its
        joint may not follow, and the arm would leave the validated path). After
        spin_guarded gave up (_dead), nothing is written: the node no longer serves
        Space, so no motion may continue (audit r7)."""
        if self._dead:
            raise MotionStopped(f'{kind}: node shutting down (callback errors), no write')
        if self._bus.lost is not None:
            raise MotionRefused(f'{kind}: arm bus disconnected ({self._bus.lost}), no write')
        if self._torque_off_pending is not None:
            self._apply_torque_off_pending()
            raise MotionStopped(f'{kind}: a pending Space torque-off was applied, no write')
        self.audit.append((kind, dict(setpoint)))
        if kind not in ('hold', 'torque_on_sync'):
            self._last_motion = time.monotonic()
        self._write_seq += 1
        failed = []
        motion = kind not in ('hold', 'torque_on_sync')
        for sid, ticks in setpoint.items():
            if self._pending_first() or (motion and self._stop.is_set()):
                raise MotionStopped(f'{kind}: stopped (Space) - goal write broken off before '
                                    f'servo {sid}')
            self._goal[sid] = None              # unknown until acknowledged (or if it raises)
            if self._bus.write_goal(sid, ticks):
                self._goal[sid] = int(ticks)
            else:
                failed.append(sid)
        if failed:
            raise BusWriteFailed(f'{kind}: goal write not acknowledged by servo(s) {failed}',
                                 failed)

    def _ensure_torque(self, source: Optional[str] = None):
        """Torque on, never with a stale goal: goal := present position first, for
        every servo. Refuses (torque stays off) if any servo cannot be read.
        `source` (a claimed motion, _prepare): a Space is re-checked under the serial
        lock, right before the sync write - Space sets _stop before it takes that lock
        for its torque-off, so a Space after _prepare's own check is never undone
        (audit r9b T1)."""
        if self._torque_on:
            return
        if self._dead:
            raise MotionStopped('torque-on refused: node shutting down (callback errors)')
        if not self._eeprom_ok:
            raise MotionRefused('torque-on refused: SR-17 rule 2 (eeprom_check: enforce) - '
                                f'{getattr(self, "_eeprom_error", "EEPROM limits not verified yet")}')
        with self._serial_lock:
            if source is not None and self._spaced():
                raise MotionStopped(f'{source}: stopped (Space) before the torque-on, '
                                    'torque stays off')
            if self._bus.lost is not None:
                raise MotionRefused(f'torque-on refused: arm bus disconnected '
                                    f'({self._bus.lost}) - reconnecting')
            if self._torque_off_pending is not None:
                self._apply_torque_off_pending()
                raise MotionStopped('torque-on refused: a pending Space torque-off was applied')
            present = {}
            for sid in IDS:
                if self._pending_first():
                    raise MotionStopped('torque-on refused: a pending Space torque-off '
                                        'was applied')
                present[sid] = self._bus.read_pos(sid)
            missing = [sid for sid, v in present.items() if v is None]
            if missing:
                raise MotionRefused(f'torque-on refused: servo(s) {missing} unreadable')
            try:
                self._write('torque_on_sync', present)
            except BusWriteFailed as exc:
                raise MotionRefused(f'torque-on refused: {exc}') from exc
            self.audit.append(('torque_on', {sid: 1 for sid in IDS}))
            failed = [sid for sid in IDS if not self._bus.torque(sid, True)]
            if failed:
                # Nothing is switched off (audit r7): a servo may already have held the
                # arm (a respawn), and turning it off would drop the arm under gravity.
                # Every acknowledged servo holds its present position (goal := present
                # was written first); the next motion repeats the sync and torque-on.
                self._torque_stuck = {}      # torque was commanded on: none is 'off' now
                raise MotionRefused(f'torque-on not acknowledged by servo(s) {failed}: '
                                    'their torque state is unknown; the others hold their '
                                    'present position; nothing moves')
            self._torque_on = True
            self._torque_stuck = {}

    def _log_eeprom_limits(self):
        """SR-17 rule 2. eeprom_check: log (default, user decision 2026-10-07: until the
        first calibration session) - the registers are read and logged, nothing is
        refused. eeprom_check: enforce (after that session) - torque-on is refused until
        the registers equal the L1 values computed from the joint limits and the
        calibration (arm_eeprom_limits); checked once the robot description is in."""
        from gripperx_arm.servo_bus import (
            ADDR_HOMING_OFFSET, ADDR_LOCK, ADDR_MAX_ANGLE_LIMIT, ADDR_MIN_ANGLE_LIMIT,
            ADDR_MODEL_NUMBER, sm_decode)
        with self._serial_lock:
            self._eeprom_regs = {sid: (self._bus.read_word(sid, ADDR_MIN_ANGLE_LIMIT),
                                       self._bus.read_word(sid, ADDR_MAX_ANGLE_LIMIT))
                                 for sid in IDS}
            # Read-only: the journal of any ordinary boot then shows the offsets the
            # calibration was measured with (Present = Actual - Homing_Offset).
            self._eeprom_offsets = {sid: self._bus.read_word(sid, ADDR_HOMING_OFFSET)
                                    for sid in IDS}
            lock = {sid: self._bus.read_byte(sid, ADDR_LOCK) for sid in IDS}
            # identity for a reconnect (audit HG3 H4); None = not readable
            self._models = {sid: self._bus.read_word(sid, ADDR_MODEL_NUMBER) for sid in IDS}
        offsets = {sid: (None if v is None else f'{v}={sm_decode(v, 11)}')
                   for sid, v in self._eeprom_offsets.items()}
        factory = [sid for sid, r in self._eeprom_regs.items() if r == (0, TICK_RANGE - 1)]
        self.get_logger().warn(
            f'SR-17 L1 EEPROM angle limits (min, max) per servo: {self._eeprom_regs}; '
            f'factory range on {factory or "none"}; eeprom_check: {self._eeprom_mode}; '
            f'Homing_Offset raw=decoded {offsets}; Lock {lock}.')

    def _verify_eeprom(self, kin, cal):
        """eeprom_check: enforce - compare the registers with the computed L1 values."""
        if self._eeprom_mode != 'enforce':
            return
        from gripperx_arm.eeprom_limits import RUN_HINT, compute, offset_refusals
        # Same keys as arm_eeprom_limits reads (arm_safety_filter.yaml): one margin source.
        mlo = list(self._p('eeprom_margin_min_ticks'))
        mhi = list(self._p('eeprom_margin_max_ticks'))
        if len(mlo) != len(IDS) or len(mhi) != len(IDS):
            bad = [f'eeprom_margin_min_ticks / _max_ticks need {len(IDS)} values each, '
                   f'not {mlo} / {mhi}']
        else:
            rows = compute(kin.limits, list(cal.joints), list(cal.zero_ticks),
                           list(cal.signs), cal.ticks_per_rev, mlo, mhi)
            bad = [f"id {r['id']}: servo {self._eeprom_regs[r['id']]} != computed "
                   f"({r['min']}, {r['max']}){' ' + r['why'] if r['why'] else ''}"
                   for r in rows
                   if not r['ok'] or self._eeprom_regs[r['id']] != (r['min'], r['max'])]
        bad_offset = ([self._offsets_error] if getattr(self, '_offsets_error', '') else
                      offset_refusals(self._eeprom_offsets,
                                      getattr(self, '_recorded_offsets', None)))
        unread = [sid for sid, r in self._eeprom_regs.items() if None in r]
        self._eeprom_ok = not (bad or bad_offset)
        self._eeprom_error = '; '.join(bad + bad_offset)
        if unread:
            # A failed read is a bus problem: re-writing the EEPROM would not help.
            self._eeprom_error += (f' - limit registers of servo(s) {unread} UNREADABLE (bus '
                                   'read failed): check cable/power/bus, then restart; '
                                   're-writing will not help')
        elif bad:
            # A calibration, joint-limit or margin change after the write lands here:
            # the operator needs the remedy, not only the mismatch.
            self._eeprom_error += f' - to re-write the L1 limits: {RUN_HINT}'
        if not self._eeprom_ok:
            self.get_logger().error('SR-17 rule 2: EEPROM limits mismatch, torque stays off: '
                                    + self._eeprom_error)
        else:
            self.get_logger().info(
                'SR-17 rule 2: EEPROM limits match the computed L1 values '
                f'{self._eeprom_regs} (margins min {mlo} / max {mhi} ticks) - torque-on allowed')
            self._torque_on = False
            try:
                self._ensure_torque()
            except MotionRefused as exc:
                self.get_logger().error(str(exc))
            except Exception as exc:     # noqa: BLE001 - a bus error must not end spin()
                text = (f'torque-on after the EEPROM check failed: {exc!r} - any servo '
                        'already torqued holds its present position (goal := present first)')
                self.get_logger().error(text)
                self._report(text)

    # _torque_off_all and _hold run inside error handlers (Space, every leader exit, the
    # leader tick's last resort, after an aborted motion). The bus raises
    # SerialException/OSError on a lost port: they catch, report and NEVER raise - an
    # exception escaping a callback would end spin() and Space with it.

    def _shut_down_motion(self, reason: str):
        """spin_guarded gives up: this node stops serving Space, but an executor thread
        may still be inside a motion (audit r7 of 47d8d9b: 494 goal writes after the
        give-up, the arm drove home with Space ignored - the 2026-07-06 class). From
        here no goal is written (_dead, checked in _write under the serial lock) and
        every motion loop ends at its next step (_stop). A MOVING arm goes limp, as on
        Space; a stationary one keeps holding (the respawned node syncs to it)."""
        with self._state_lock:
            moving = self._moving()
            self._stop.set()
            self._pending_home = None
            self._leader_state = LEADER_OFF
        with self._serial_lock:
            self._dead = True
        if moving:
            self._torque_off_all(f'node shutting down while the arm was moving: {reason}')
        # _dead: with torque on the gripper goal cannot be re-anchored, so the leader limit
        # stays (the lower force); with torque off it is restored.
        self._restore_grip_limit(f'node shutting down: {reason}', lock_timeout=1.0)

    def _torque_off_all(self, reason: str, lock_timeout: Optional[float] = None) -> bool:
        """Torque off on every servo; one that fails does not spare the others.
        `lock_timeout` (Space only): the serial lock is waited for at most this long -
        Space must never wait behind a bus transaction that hangs (2026-10-13). Not
        taken in time: the torque-off is made PENDING (_torque_off_pending), the stop
        flag is set (a ramp ends at its next setpoint check), and False is returned."""
        if self._bus.lost is not None:
            self._torque_off_pending = reason
            self._stop.set()
            text = (f'TORQUE OFF PENDING: {reason} - the arm bus is DISCONNECTED '
                    f'({self._bus.lost}): the torque-off is applied as soon as it is '
                    'reconnected; no goal is written meanwhile')
            self.get_logger().error(text)
            self._report(text)
            return False
        if lock_timeout is None:
            self._serial_lock.acquire()
        elif not self._serial_lock.acquire(timeout=lock_timeout):
            self._torque_off_pending = reason
            self._stop.set()
            text = (f'TORQUE OFF PENDING: {reason} - the arm bus is BUSY (serial lock not '
                    f'free within {lock_timeout:.2f} s, a bus transaction hangs): the '
                    'torque-off is applied by the next bus access, retried every 50 ms and '
                    'by every Space; no goal is written meanwhile')
            self.get_logger().error(text)
            self._report(text)
            return False
        try:
            self._torque_off_pending = None
            failed = self._torque_off_locked()
        finally:
            self._serial_lock.release()
        self._log_torque_off(reason, failed)
        return True

    def _torque_off_locked(self) -> Dict[int, str]:
        """Caller holds the serial lock. {sid: why} for the servos that failed."""
        failed = {}
        self.audit.append(('torque_off', {sid: 0 for sid in IDS}))
        for sid in IDS:
            try:
                if not self._bus.torque(sid, False):
                    failed[sid] = 'not acknowledged'
            except Exception as exc:     # noqa: BLE001
                failed[sid] = repr(exc)
        self._torque_on = False
        self._torque_stuck = dict(failed)
        # limp: no goal stays valid (the next torque-on writes goal := present)
        self._goal = {sid: None for sid in IDS}
        return failed

    def _log_torque_off(self, reason: str, failed: Dict[int, str]):
        text = f'TORQUE OFF (arm limp): {reason}'
        if failed:
            text += (f' - TORQUE-OFF WRITE FAILED on servo(s) {failed}: those may still '
                     'hold torque')
        self.get_logger().error(text)
        self._report(text)

    def _on_bus_lost(self, exc: Exception):
        """_BusGuard, first I/O error (the caller usually holds the serial lock: no
        lock taken here). Torque state and every goal become UNKNOWN: _torque_on False
        (the next motion goes through the torque-on sync, goal := present first), and
        every servo counts as possibly torqued (_torque_stuck), so a Space after the
        reconnect turns the torque off. Motion is refused until the reconnect."""
        self._torque_on = False
        self._goal = {sid: None for sid in IDS}
        self._torque_stuck = {sid: 'torque state unknown since the arm bus was lost'
                              for sid in IDS}
        self._stop.set()
        text = (f'ARM BUS LOST ({exc!r}): port closed, reopening {SERVO_PORT} every '
                f'{self._reconnect_period:.1f} s; torque state and goals UNKNOWN; every motion is '
                'refused; Space turns the torque off once the bus is back')
        self.get_logger().error(text)
        self._report(text)
        self._bus_lost_reported = time.monotonic()

    # A display choice, not a safety value: how often a growing count of failed bus
    # transactions is logged at most.
    BUS_HEALTH_LOG_PERIOD_S = 30.0

    def _log_bus_health(self, guard):
        """Audit EW1 L1: transactions ended by servo_bus.TXN_MAX_S or by a short reply
        return None and leave no trace by themselves; a degrading bus shows here, at most
        once per BUS_HEALTH_LOG_PERIOD_S, only when the counts grew."""
        health = getattr(guard.raw, 'health', None)
        if health is None:
            return
        try:
            now_counts = tuple(health())
        except Exception:     # noqa: BLE001 - a report, never a failure of its own
            return
        last = getattr(self, '_bus_health_logged', ((0, 0), -math.inf))
        if now_counts == last[0] or time.monotonic() - last[1] < self.BUS_HEALTH_LOG_PERIOD_S:
            return
        self._bus_health_logged = (now_counts, time.monotonic())
        self.get_logger().warn(
            f'arm bus: {now_counts[0]} transaction(s) ended by the servo_bus.TXN_MAX_S bound and '
            f'{now_counts[1]} by a malformed reply since start (+{now_counts[0] - last[0][0]} / '
            f'+{now_counts[1] - last[0][1]} since the last report)')

    def _reconnect_tick(self):
        """Own group, every bus.reconnect_period_sec while the bus is lost: reopen the port
        by name and require every servo to answer. Nothing is written on reconnect - no
        torque, no goal (the servos may have lost power, or still hold their last
        goal); it is a fresh start for the next commanded motion. A pending Space
        torque-off is applied right after (_torque_off_pending_tick)."""
        guard = self._bus
        self._log_bus_health(guard)
        if guard.lost is None or not self._serial_lock.acquire(timeout=0.5):
            return
        why = None
        try:
            raw = guard.raw
            reopen = getattr(raw, 'reopen', None)
            if reopen is not None:
                reopen()
            else:
                try:
                    raw.close()
                except Exception:     # noqa: BLE001
                    pass
                raw.open()
            present = {}
            for sid in IDS:
                try:
                    present[sid] = raw.read_pos(sid)
                except Exception as exc:     # noqa: BLE001
                    present[sid] = None
                    why = repr(exc)
            missing = [sid for sid, v in present.items() if v is None]
            if missing:
                why = f'servo(s) {missing} unreadable after reopening' + (
                    f' ({why})' if why else '')
            else:
                why = self._reconnect_identity(raw)
            if why is None:
                guard.lost = None
        except Exception as exc:     # noqa: BLE001 - the device is still gone
            why = f'reopen failed: {exc!r}'
        finally:
            self._serial_lock.release()
        if why is None:
            seen = self._last_seen
            moved = {} if seen is None else {
                sid: tick_deviation(present[sid], seen[sid]) for sid in IDS
                if abs(tick_deviation(present[sid], seen[sid])) > POS_TOLERANCE}
            if moved:
                text = (f'ARM BUS RECONNECTED - positions MOVED while the bus was lost '
                        f'(present - last seen, ticks, beyond {POS_TOLERANCE}): {moved} - '
                        'power loss and sag, or a different device on the port')
                self.get_logger().error(text)
                self._report(text)
            text = (f'ARM BUS RECONNECTED on {SERVO_PORT}: all servos readable {present}; '
                    'nothing written (torque and goals as the servos hold them, state '
                    'unknown to this node); the next motion syncs goal := present before '
                    'its torque-on; Space turns the torque off')
            self.get_logger().warn(text)
            self._report(text)
        elif (why != self._bus_lost_why
              or time.monotonic() - self._bus_lost_reported >= 10.0):
            self._bus_lost_reported = time.monotonic()
            self._bus_lost_why = why
            text = f'ARM BUS still disconnected: {why}; every motion refused'
            self.get_logger().error(text)
            self._report(text)

    def _reconnect_identity(self, raw) -> Optional[str]:
        """Caller holds the serial lock. A reopened /dev/arm_servo must be the same
        servos (audit HG3 H4: the udev rule is the only identity otherwise): each
        servo's model number equal to the one read at startup, where both are known.
        Why not, or None."""
        from gripperx_arm.servo_bus import ADDR_MODEL_NUMBER
        bad = {}
        for sid, before in self._models.items():
            if before is None:
                continue
            try:
                now = raw.read_word(sid, ADDR_MODEL_NUMBER)
            except Exception as exc:     # noqa: BLE001
                now = repr(exc)
            if now != before:
                bad[sid] = (before, now)
        if bad:
            return (f'servo model number(s) differ from startup (before, now): {bad} - '
                    'not the same servos; reconnect refused')
        return None

    def _apply_torque_off_pending(self):
        """Caller holds the serial lock: the pending Space torque-off, now."""
        reason, self._torque_off_pending = self._torque_off_pending, None
        failed = self._torque_off_locked()
        self._log_torque_off(f'{reason} (applied once the bus was free)', failed)

    def _torque_off_pending_tick(self):
        """Own group, every 50 ms: applies a pending Space torque-off as soon as the
        serial lock is free (never waits for it)."""
        if (self._torque_off_pending is None or self._bus.lost is not None
                or not self._serial_lock.acquire(blocking=False)):
            return
        try:
            if self._torque_off_pending is not None:
                self._apply_torque_off_pending()
        except Exception as exc:     # noqa: BLE001 - a timer must not end spin()
            self.get_logger().error(f'pending torque-off: {exc!r}')
        finally:
            self._serial_lock.release()

    def _hold(self, reason: str):
        """Stop where the arm is commanded, torque stays on (user decision 2026-10-13,
        option a): every servo KEEPS its last written goal - re-writing a measured pose
        that sags under load lowered the arm hold by hold (2026-10-12). Per servo, the
        measured position is written instead (re-anchored) when
          - |present - last goal| > hold.contact_threshold_ticks: contact/obstruction,
            it must stop pushing;
          - its last goal is unknown (a write not acknowledged or interrupted).
        If the kept goals fail the filter under the CURRENT envelope, grown for the gap
        to the measured pose (_kept_goals_why), they are still kept while the kept pose
        AND the measured pose each pass as static poses (user decision 2026-11-02): only
        the growth refused them, and a measured pose written there drops the arm by the
        sag at every hold, near the chassis into the hull (twice the sag after J). A
        goal in a wheel steered into it fails statically, and so does an arm touching a
        wheel with its goal clear beyond it (a clear goal does not mean a clear way):
        then, or if they cannot be checked, every servo is held at its measured position
        (the pre-2026-10-13 hold). A servo that cannot be read keeps its goal. The 'hold' audit entry is the write (possibly empty).
        Reads and writes under the serial lock, the filter check between them outside
        it (Space must not wait for it); torque off meanwhile (Space) -> no write."""
        problems, unheld = [], set()
        with self._serial_lock:
            measured = {}
            for sid in IDS:
                if self._pending_first():
                    problems.append('a pending Space torque-off came first')
                    break
                try:
                    v = self._read_pos(sid)
                except Exception as exc:     # noqa: BLE001
                    problems.append(f'read servo {sid}: {exc!r}')
                    v = None
                if v is None:
                    unheld.add(sid)
                else:
                    measured[sid] = v
            loads = self._read_loads(IDS)
            torque, goals, seq = self._torque_on, dict(self._goal), self._write_seq
        anchor, note = {}, ''
        if measured and torque:
            anchor, note = self._hold_split(measured, goals)
        with self._serial_lock:
            if measured and self._torque_on:
                if self._write_seq != seq:
                    # unreachable while the caller owns the arm; never overwrite a newer
                    # motion's goals with this hold's decision
                    problems.append('goals rewritten during the hold decision: nothing '
                                    'written by this hold')
                else:
                    try:
                        self._write('hold', anchor)
                    except BusWriteFailed as exc:
                        problems.append(str(exc))
                        unheld |= set(exc.failed)
                    except Exception as exc:     # noqa: BLE001
                        problems.append(f'hold write: {exc!r}')
                        unheld = set(IDS)        # unknown how far the write got
        text = f'HOLD: {reason}'
        if note:
            text += f' - {note}'
        text += f'; present load (likely per mille of max PWM duty, TO-VERIFY) {loads}'
        if problems:
            text += (f' - HOLD INCOMPLETE ({"; ".join(problems)}): servo(s) '
                     f'{sorted(unheld)} may keep their previous goal')
        self.get_logger().warn(text)
        self._report(text)

    def _hold_split(self, measured: Dict[int, int], goals: Dict[int, Optional[int]]):
        """(servos to re-anchor at their measured position, report text) for _hold.
        Never raises: it runs inside error handlers."""
        try:
            kept, anchor, why = {}, {}, []
            for sid, m in measured.items():
                g = goals.get(sid)
                if g is None:
                    anchor[sid] = m
                    why.append(f'{sid} (last goal unknown)')
                    continue
                d = tick_deviation(m, g)
                if abs(d) > self._contact_ticks:
                    anchor[sid] = m
                    why.append(f'{sid} (present - goal {d:+d} ticks > '
                               f'{self._contact_ticks}: contact/obstruction, stops pushing)')
                else:
                    kept[sid] = g
            if kept:
                bad = self._kept_goals_why(measured, kept)
                if bad and (self._kept_goals_why(measured, kept, static=True)
                            or self._kept_goals_why(measured, {}, static=True)):
                    return dict(measured), (
                        f'kept goals NOT valid ({bad}): measured position written on every '
                        f'servo (goal - measured was {self._goal_lag(measured, kept)} ticks)')
                if bad:
                    why.append(f'kept as static poses, the gap grown refused them: {bad}')
            text = f'last goal kept on servo(s) {sorted(kept)}'
            lag = self._goal_lag(measured, kept)
            if lag:
                text += f' (goal - measured {lag} ticks)'
            if why:
                text += '; re-anchored at the measured position: ' + ', '.join(why)
            return anchor, text
        except Exception as exc:     # noqa: BLE001 - _hold must hold, whatever failed
            return dict(measured), (f'hold decision failed ({exc!r}): measured position '
                                    'written on every servo')

    def _read_loads(self, sids) -> Dict[int, Optional[int]]:
        """Present_Load per servo, for the log only (audit HG1 F3: the robot test is to
        measure the force a servo applies at the contact threshold). Caller holds the
        serial lock. Never raises; None = not read (or a bus without the register)."""
        read = getattr(self._bus, 'read_load', None)
        out = {}
        for sid in sids:
            try:
                out[sid] = None if read is None else read(sid)
            except Exception:     # noqa: BLE001 - logging only, inside error handlers
                out[sid] = None
        return out

    @staticmethod
    def _goal_lag(measured: Dict[int, int], kept: Dict[int, int]) -> Dict[int, int]:
        return {sid: tick_deviation(g, measured[sid]) for sid, g in kept.items()
                if g != measured[sid]}

    def _kept_goals_why(self, measured: Dict[int, int], kept: Dict[int, int],
                        static: bool = False, sids=LIMB_IDS) -> Optional[str]:
        """Why goals kept from earlier writes may not stay commanded, None if they may.
        Each was filter-approved (or the measured pose) when written; since then the
        envelope (steering) or the model may have changed, and the servo has not
        reached it. Checked as the MOTION the servos still make (audit HG1 F2): the
        measured pose and the kept pose (limbs only - the gripper is at its measured
        value, its targets are unchecked by rule 12, audit HG1 F4), each grown for the
        per-joint gap between them, as SafetyFilter.check_setpoints checks a step. So
        the measured pose itself must pass the envelope too. Two allowances, which only
        ever keep the arm where it is commanded rather than where it sagged to: a joint
        beyond its L2 limit may stay there if not further out than measured
        (recovery_limits), and a link failing the ground check is not ground-checked if
        its kept height is not below its measured one - keeping it is what stops the
        sag (2026-10-12). No filter or calibration: cannot be checked -> not kept.
        `static`: the kept pose alone, not grown for the gap - may a motion START there
        (_held_start)? The gap is then the first step's to pass, from the measured pose.
        `sids`: the servos whose kept goals count (leader mode: LEADER_IDS, its gripper
        goals were filter-checked when written)."""
        flt, cal = self._filter, self._cal
        if flt is None or cal is None:
            return f'no safety filter to re-check them: {self._filter_error}'
        limbs = {sid: g for sid, g in kept.items() if sid in sids}
        mq = self._q_of(measured)
        kq = self._q_of({**measured, **limbs})
        dq = {n: 0.0 if static else abs(kq[n] - mq[n]) for n in mq}
        hm, hk = flt.ground_heights(mq), flt.ground_heights(kq)
        low = set(flt.ground_shortfalls(kq, dq))
        if not static:
            low |= set(flt.ground_shortfalls(mq, dq))
        skip = {n for n in low if hk[n] >= hm[n]} or None
        lim, env = flt.recovery_limits(mq), self._envelope()
        poses = (('kept pose', kq),) if static else (('measured pose', mq), ('kept pose', kq))
        for what, q in poses:
            v = flt.check_pose(q, env, dq=dq, limits=lim, ground_skip=skip)
            if not v.ok:
                return f'{what}: {v}' if static else \
                    f'{what}, grown for the gap between them: {v}'
        return None

    def _held_start(self, here: Dict[int, int], source: str,
                    report: bool = True, sids=LIMB_IDS) -> Dict[int, int]:
        """The start of a motion from a held arm (user decision 2026-10-13): per limb
        servo its last written goal; its measured position only where the goal is unknown
        or the torque is off. The gripper starts measured (a limb path does not move it),
        except in leader mode (`sids` LEADER_IDS), which drives it like a limb. Starting at the measured pose would make the first setpoint re-anchor a
        servo that sags under load, the ratchet a kept goal avoids.

        Where _hold would re-anchor - a servo beyond hold.contact_threshold_ticks of its
        goal, kept goals that fail _kept_goals_why - the start is CLAMPED to the held goal
        all the same (user decision 2026-10-30; audit: goals near the chassis, a sag of 8,
        re-plans from the measured pose wrote limb2 3122, 3130, 3138 into the hull). The
        kept goals were refused for the motion they grew by the gap, so the clamped pose
        is checked again as a static pose: valid, the path starts there, and a leader path's
        first step is checked from the measured pose like any other. `I` has no such
        check: _arm_home plans from this start and _prepare checks the measured pose only
        as a static pose (audit 2026-11-02, the servo already drives toward that goal).
        Not valid: HeldPoseInvalid, no motion starts - no quiet fall-back to the measured
        pose.
        The verdict is memoised on its inputs (measured ticks, held goals, envelope,
        filter, calibration): the leader tick asks every tick while it holds. `report`
        False: no status line (that tick)."""
        with self._serial_lock:
            torque, goals = self._torque_on, dict(self._goal)
        if not torque:
            return dict(here)
        known = {sid: goals[sid] for sid in sids if goals[sid] is not None}
        kept = {sid: g for sid, g in known.items()
                if abs(tick_deviation(here[sid], g)) <= self._contact_ticks}
        lag = self._goal_lag(here, known)
        if not lag:
            return dict(here)
        env = self._envelope()
        key = (tuple(sorted(here.items())), tuple(sorted(known.items())))
        memo = self._held_memo
        if (memo is not None and memo[0] == key and memo[1] is env
                and memo[2] is self._filter and memo[3] is self._cal):
            bad, clamp_bad = memo[4]
        else:
            bad = self._kept_goals_why(here, kept, sids=sids) if kept else None
            clamp_bad = None
            if bad or len(kept) < len(known):
                clamp_bad = self._kept_goals_why(here, known, static=True, sids=sids)
            self._held_memo = (key, env, self._filter, self._cal, (bad, clamp_bad))
        if clamp_bad:
            text = (f'{source}: refused - the held goals are not a valid start ({clamp_bad}), '
                    f'and the measured pose would re-anchor the arm (goal - measured {lag} '
                    'ticks): holding')
            if report:
                self.get_logger().warn(text)
                self._report(text)
            raise HeldPoseInvalid(text)
        if report and (bad or len(kept) < len(known)):
            text = (f'{source}: starts at the held goals as a static pose, not the measured '
                    f'pose (goal - measured {lag} ticks; kept for the motion they would not '
                    f'be: {bad or "beyond hold.contact_threshold_ticks"})')
            self.get_logger().warn(text)
            self._report(text)
        elif report:
            text = (f'{source}: starts at the held goals, not the measured pose (goal - '
                    f'measured {lag} ticks)')
            self.get_logger().info(text)
            self._report(text)
        return {**here, **known}

    # ================================================================ motions
    # User decision 2026-10-07 ("im Zweifel abbrechen"): the ARM parts of the pick
    # sequence (P), go_home (I) and the return home after an abort are planned through
    # gripperx_arm.planner and every executed segment has passed the filter; no path =
    # no motion. Gripper-only moves (O, and the open/close steps of P) stay UNCHECKED
    # (user decision); the audit log marks them gripper_unchecked.

    def _ramp(self, segments, source: str, kind: str, should_abort=None, guard=None):
        """Write tick-linear ramps; segments = [(start_ticks, end_ticks, seconds)] or,
        with a soft start, [(start_ticks, end_ticks, seconds, fractions)]: the setpoints
        at those fractions of the way (exec_steps.ticks_at, repeats dropped - what the
        planner validated), one per ramp period of the plain ramp.
        `guard(setpoint)`: called after each write (the ground-recovery lift, it reads
        the present positions); it raises to stop the ramp."""
        for seg in segments:
            start, end, seconds = seg[:3]
            steps = ramp_count(seconds, MOVE_STEP_HZ)
            delay = seconds / steps
            sps = (tick_ramp(start, end, steps) if len(seg) < 4
                   else ticks_at(start, end, seg[3], dedupe=True))
            for sp in sps:
                if self._stop.is_set() or self._spaced():
                    raise MotionStopped(f'{source}: stopped (Space), torque off')
                if should_abort is not None and should_abort():
                    raise MotionAborted(f'{source}: aborted')
                with self._serial_lock:
                    if self._stop.is_set() or self._spaced():
                        raise MotionStopped(f'{source}: stopped (Space), torque off')
                    self._write(kind, sp)
                if guard is not None:
                    guard(sp)
                time.sleep(delay)

    def _prepare(self, source: str) -> Dict[int, int]:
        """Space first (a stop between two steps must not be undone by a torque-on),
        then torque, then the measured pose."""
        if self._spaced():
            raise MotionStopped(f'{source}: stopped (Space), torque off')
        if self._grip_limit_prev is not None:
            # never with the leader gripper limit (FR-17 amendment 2026-11-16)
            why = self._restore_grip_limit(f'{source}: before the motion')
            if why:
                raise MotionRefused(f'{source}: refused - {why}')
        self._ensure_torque(source)
        here = self._read_all()
        if here is None:
            raise MotionRefused(f'{source}: no measured arm pose (a servo did not answer)')
        return here

    def _gripper_target(self, ticks: int, here: Dict[int, int], source: str,
                        evaluate: bool = True):
        """SR-17 rule 12: a gripper target is NOT blocked by the filter (evaluated and
        logged only), but L2 (the URDF joint limit) applies, and the register ramp to
        it must not cross the tick wrap. Without a model there is no L2 to apply (L1,
        the servo EEPROM, still does): logged."""
        if self._filter is None or self._cal is None:
            self.get_logger().warn(f'{source}: gripper target {ticks} NOT evaluated and L2 '
                                   f'not applied - no model ({self._filter_error})')
            return
        g = self._cal.joints[IDS.index(GRIPPER_ID)]
        q_here = self._q_of(here)
        q = {**q_here, g: self._q_of({**here, GRIPPER_ID: int(ticks)})[g]}
        lo, hi = self._filter.kin.limits[g]
        if not lo <= q[g] <= hi:
            raise MotionRefused(f'{source}: gripper target {ticks} = {q[g]:+.4f} rad outside '
                                f'the L2 joint limit [{lo:+.4f}, {hi:+.4f}] (SR-17 rule 3)')
        why = self._wrap_reason(q_here, q, (GRIPPER_ID,))
        if why:
            raise MotionRefused(f'{source}: {why}')
        if evaluate:
            v = self._filter.check_pose(q, self._envelope())
            text = f'{source}: gripper target {ticks} evaluated (not blocked, rule 12): {v}'
            if v.ok:
                self.get_logger().info(text)
            else:
                self.get_logger().warn(text)
            self._report(text)

    def _move_gripper(self, ticks: int, move_ms: int, source: str, should_abort=None):
        """Gripper only, not blocked by the filter (user decision 2026-10-07)."""
        here = self._prepare(source)
        self._gripper_target(int(ticks), here, source)
        self._ramp([({GRIPPER_ID: here[GRIPPER_ID]}, {GRIPPER_ID: int(ticks)},
                     move_ms / 1000.0)], source, 'gripper_unchecked', should_abort)

    def _outside(self, q: Dict[str, float], source: str, what: str) -> Optional[str]:
        """The refusal text for a motion other than the operator's go-home from a pose
        outside the L2 joint limits or with arm links in self-contact (user decisions
        2026-10-09: recovery, option C), None if neither."""
        if self._filter is None:
            return None
        out = self._filter.outside_limits(q)
        touch = self._self_contact(q)
        why = [f'start outside joint limits ({describe_outside(out)})'] if out else []
        why += [f'start pose in self-contact ({touch})'] if touch else []
        if not why:
            return None
        return (f'{source}: {"; ".join(why)}: {what} refused - press I (go_home) first, '
                + ('it plans a recovery path back inside' if out else '')
                + (' and ' if out and touch else '')
                + ('it separates the pair' if touch else ''))

    def _self_contact(self, q: Dict[str, float]) -> str:
        """The arm self pairs in contact at q (static self check, limits relaxed to
        q itself: only the self check decides), described; '' if none."""
        _v, hits = self._filter.self_contacts(q, self._envelope(),
                                              limits=self._filter.recovery_limits(q))
        return self._filter.describe_contacts(q, hits) if hits else ''

    def _plan_arm(self, start_ticks: Dict[int, int], goal_ticks: Dict[int, int],
                  source: str, envelope=None, automatic: bool = True,
                  direct_ms: Optional[int] = None, recovery: bool = False,
                  separate_self: bool = False, rise_ground: bool = False,
                  soft_start: bool = False, resume: bool = False):
        """Planned arm path (gripper held at its start value): PlanResult or refusal.
        `direct_ms`: the move time _path_segments will give a direct path (the
        planner validates the ramp steps that time produces).
        SR-17 rule 12: no filter, configuration or model -> refused. SR-17 rule 11:
        `automatic` (the default: only an operator command may pass False) -> direct
        path only while planner.automatic_paths is direct_only. `recovery` (the go-home
        motions only): a start outside the joint limits is planned back inside
        (Planner recovery); any other motion from there is refused. `separate_self`
        (the operator's I only, safety_filter.self_contact_recovery): arm links in
        self-contact at the start are separated (Planner separate_self); a go-home
        without it refuses such a start (user decision 2026-10-09, option C).
        `rise_ground` (the operator's I only, safety_filter.ground_recovery): a start
        inside the ground margin - since 2026-10-15 also at or below the ground - is
        planned out of it, the offending links rising from the first setpoint on
        (Planner rise_ground, user decisions 2026-10-12 and 2026-10-15); every
        other motion from there is refused by the start check as before.
        `soft_start` (every planned ramp motion since 2026-10-20: the automatic return
        home, the operator's I, P's approach and return - user decisions 2026-10-20):
        the segment leaving the start begins with planner.soft_start_step_fractions of
        a full ramp step (_planner) - validated as written by _ramp; not a segment that
        lifts out of the ground margin (res.soft_first False).
        `resume` (the operator's I only, audit 2026-11-02 M6): a plan cut by
        planner.time_budget_s is resumed in further slices, as a leader re-plan is
        (_resumed_search), up to planner.max_search_s - the same checks and path, more
        wall time under load; Space cancels it at once (MotionStopped, nothing written).
        Without it one slice decides, as before."""
        if not self._config_loaded:
            raise MotionRefused(f'{source}: arm_safety_filter.yaml not loaded '
                                '(safety_filter.config_loaded false) - arm motions are '
                                'planned through the safety filter and refused without '
                                'its configuration (SR-17 rule 12)')
        if self._filter is None:
            raise MotionRefused(f'{source}: {self._filter_error} - arm motions are planned '
                                'through the safety filter and refused without it')
        start = self._q_of(start_ticks)
        goal = self._q_of({**start_ticks, **{s: goal_ticks[s] for s in LIMB_IDS}})
        if not recovery:
            why = self._outside(start, source, 'this motion is')
            if why:
                self.get_logger().error(why)
                self._report(why)
                raise MotionRefused(why)
        env = self._envelope() if envelope is None else envelope
        budget = float(self._p('planner.time_budget_s'))
        direct_only = automatic and self._automatic_paths == 'direct_only'
        out = self._filter.outside_limits(start) if recovery else {}
        if out:
            text = (f'{source}: start outside joint limits ({describe_outside(out)}): '
                    'recovery path back inside')
            self.get_logger().warn(text)
            self._report(text)
        touch = self._self_contact(start) if recovery else ''
        if touch and not separate_self:
            why = f'{source}: start pose in self-contact ({touch}): press I to separate'
            self.get_logger().error(why)
            self._report(why)
            raise MotionRefused(why)
        if touch:
            text = f'{source}: start pose in self-contact ({touch}): recovery separates the pair'
            self.get_logger().warn(text)
            self._report(text)
        planner = self._planner(budget, direct_only, direct_ms=direct_ms, recovery=bool(out),
                                separate_self=separate_self, rise_ground=rise_ground,
                                soft_start=soft_start)
        slices = [1]
        if resume:
            self._op_planner = planner
            try:
                if self._spaced():           # a Space before Space could see the planner
                    planner.cancel()
                res, _stalled = _resumed_search(planner, start, goal, env,
                                                float(self._p('planner.max_search_s')),
                                                lambda n: slices.__setitem__(0, n))
            finally:
                self._op_planner = None
            if res.bound == 'cancelled':
                raise MotionStopped(f'{source}: stopped (Space) while planning, nothing moved')
        else:
            res = planner.plan(start, goal, env)
        res.envelope = env                   # the envelope this plan was checked against
        # Whether the planner validated the first segment WITH its soft start: not while
        # it lifts out of the ground margin (Planner.starts_soft) - _path_segments must
        # write exactly what was validated.
        res.soft_first = bool(res.ok and len(res.path) > 1 and planner.starts_soft(
            res.path[0], res.path[1], res.method == 'direct'))
        if res.ground:
            text = (f'{source}: start pose inside the ground margin ({res.ground}): ground '
                    'recovery - the path lifts it out first, one joint at a time, rising at '
                    'every setpoint; between them certified above the ground, or - from at or '
                    'below it - never lower than at the setpoint before')
            self.get_logger().warn(text)
            self._report(text)
        mode = 'automatic: direct path only' if direct_only else 'detours allowed'
        if out:
            mode += '; recovery from outside the joint limits'
        if res.self_contact:
            mode += '; separating a self-contact'
        if res.ground:
            mode += '; lifting out of the ground margin'
        if res.soft_first or (soft_start and not res.ok):     # as validated / as tried
            mode += '; soft start'
        text = (f'{source} ({mode}): path {res.method or "NONE"} ({len(res.path)} waypoints, '
                f'{res.checks} checks, {res.cost_ms:.0f} ms of {budget * 1e3:.0f} ms'
                f'{f" in the last of {slices[0]} slices, resumed" if slices[0] > 1 else ""})'
                f'{"" if res.ok else ": " + res.reason}')
        if res.ok:
            self.get_logger().info(text)
        else:
            self.get_logger().error(text)
        self._report(text)
        if not res.ok:
            raise MotionRefused(f'NO_PATH {text}')
        return res

    def _planner(self, budget_s: float, direct_only: bool = False, executor: str = 'ramp',
                 direct_ms: Optional[int] = None, recovery: bool = False,
                 separate_self: bool = False, rise_ground: bool = False,
                 soft_start: bool = False):
        """A Planner that validates what `executor` writes: 'ramp' (_ramp: a direct
        path in direct_ms when given, detours at planner.max_joint_velocity_rad_s, at
        MOVE_STEP_HZ) or 'leader' (the leader tick following a path: equal steps of at
        most leader.max_joint_velocity_rad_s / leader.rate_hz, braked toward the
        session's command band edges as the direct steps are - leader.edge_slowdown_*).
        `soft_start` (user decision 2026-10-20): the segment leaving the plan's start
        begins with planner.soft_start_step_fractions of a full step (Planner
        exec_first) - the leader always uses it for its re-plans."""
        # Filter, calibration and speeds are bound HERE: a search running in a
        # _ReplanJob thread keeps computing against what it started with, never against
        # self._cal live - and its memoised verdicts stay valid for its whole life.
        # Detours are searched over limb1..limb5 also for leader re-plans: the gripper keeps
        # its start (held goal) along the path and then follows the leader by direct steps
        # (2026-11-13: the RRT would sample its opening at random along a detour, and in the
        # desk tests the 6-D search ran out of planner.max_checks where the 5-D one found a path).
        cal, arm = self._cal, self._cal.joints[:5]
        if executor == 'leader':
            vmax = float(self._p('leader.max_joint_velocity_rad_s'))
            rate = float(self._p('leader.rate_hz'))
            zone = float(self._p('leader.edge_slowdown_zone_rad'))
            vslow = float(self._p('leader.edge_slowdown_velocity_rad_s'))
            band = self._leader_band()        # RRT samples keep the leader margin
            # The session edges the target is clamped to (bound here, as the rest):
            # a path brakes toward them as the direct steps do (audit ES1 H1).
            edges = dict(self._leader_edges or band)

            def steps(qa, qb, direct, soft=()):
                fr = braked_fractions(qa, qb, arm, edges, vmax, rate, zone, vslow, soft)
                if fr is not None:
                    return exec_poses_at(cal, qa, qb, fr, dedupe=bool(soft))
                return exec_poses(cal, qa, qb, velocity_count(
                    max(abs(qb[n] - qa[n]) for n in arm), vmax / rate))
        else:
            band = None
            v = float(self._p('planner.max_joint_velocity_rad_s'))
            sub = self._filter.cfg.max_joint_step_rad

            def steps(qa, qb, direct, soft=()):
                n = ramp_count(self._ramp_seconds(qa, qb, direct_ms if direct else None,
                                                  v, cal), MOVE_STEP_HZ)
                if soft:
                    # every written setpoint: monotone_cover would merge the small first
                    # steps into one large one - the motion bound the soft start avoids
                    return exec_poses_at(cal, qa, qb, soft_start_fractions(n, soft),
                                         dedupe=True)
                poses = exec_poses(cal, qa, qb, n)
                # separating a self-contact or rising out of the ground margin: every
                # written setpoint, so the motion bound per step (and the clearance dip
                # it allows) stays smallest and each setpoint is held to the rise
                return poses if separate_self or rise_ground else monotone_cover(poses, arm, sub)
        soft = list(self._soft_start)

        def soft_steps(qa, qb, direct):
            return steps(qa, qb, direct, soft)
        first = soft_steps if soft_start or executor == 'leader' else None
        return Planner(self._filter, arm, budget_s,
                       float(self._p('planner.rrt_step_rad')), int(self._p('planner.seed')),
                       max_checks=int(self._p('planner.max_checks')),
                       edge_check=lambda qa, qb: self._wrap_reason(qa, qb, cal=cal),
                       direct_only=direct_only, exec_steps=steps, recovery=recovery,
                       sample_band=band, exec_first=first,
                       separate_self=separate_self, rise_ground=rise_ground)

    @staticmethod
    def _ramp_seconds(a, b, direct_ms: Optional[int], v: float, cal) -> float:
        """Move time of one ramp segment: direct_ms for a direct path, else at v."""
        if direct_ms is not None:
            return direct_ms / 1000.0
        return max(0.2, max(abs(b[n] - a[n]) for n in cal.joints[:5]) / v)

    def _path_segments(self, path, method: str, direct_ms: Optional[int],
                       here: Dict[int, int], soft: bool = False):
        """Waypoints -> tick segments, ALL converted (and wrap-validated) before the
        caller writes anything. path[0] must be the pose of `here`. A direct path keeps
        the sequence's move time; a detour runs at planner.max_joint_velocity_rad_s.
        `soft`: the first segment carries the soft-start fractions the planner
        validated it with (_planner soft_start)."""
        v = float(self._p('planner.max_joint_velocity_rad_s'))
        segs, prev = [], dict(here)
        for i, (a, b) in enumerate(zip(path, path[1:])):
            ticks = self._ticks_of(b, a)
            seconds = self._ramp_seconds(a, b, direct_ms if method == 'direct' else None,
                                         v, self._cal)
            seg = ({s: prev[s] for s in LIMB_IDS}, {s: ticks[s] for s in LIMB_IDS}, seconds)
            if soft and i == 0:
                seg += (soft_start_fractions(ramp_count(seconds, MOVE_STEP_HZ),
                                             self._soft_start),)
            segs.append(seg)
            prev = {**prev, **{s: ticks[s] for s in LIMB_IDS}}
        return segs

    def _planned_segments(self, start: Dict[int, int], goal: Dict[int, int], source: str,
                          direct_ms: Optional[int], envelope=None, automatic: bool = True,
                          recovery: bool = False, separate_self: bool = False,
                          rise_ground: bool = False, soft_start: bool = False,
                          resume: bool = False):
        """Plan + convert: (PlanResult, segments, end ticks) or MotionRefused - before
        any write."""
        res = self._plan_arm(start, goal, source, envelope, automatic=automatic,
                             direct_ms=direct_ms, recovery=recovery,
                             separate_self=separate_self, rise_ground=rise_ground,
                             soft_start=soft_start, resume=resume)
        segs = self._path_segments(res.path, res.method, direct_ms, start,
                                   **({'soft': True} if res.soft_first else {}))
        end = {**start, **(segs[-1][1] if segs else {})}
        return res, segs, end

    def _verify_positions(self, targets: Dict[int, int], step: str) -> Dict[int, int]:
        """Measured vs commanded, per servo: the deviations beyond POS_TOLERANCE (logged
        and reported). Unreadable servos are reported as missing."""
        here = {}
        with self._serial_lock:
            for sid in targets:
                here[sid] = self._bus.read_pos(sid)
        bad = {}
        for sid, target in targets.items():
            actual = here[sid]
            if actual is None:
                self.get_logger().warn(f'{step}: ID {sid} not responding - position unknown')
                continue
            delta = abs(actual - target)
            if delta > POS_TOLERANCE:
                bad[sid] = actual - target
        if bad:
            text = (f'{step}: target not reached (ticks measured - commanded, tolerance '
                    f'{POS_TOLERANCE}): {bad}')
            self.get_logger().warn(text)
            self._report(text)
        return bad

    def _revalidated_segments(self, here: Dict[int, int], planned, goal: Dict[int, int],
                              source: str, direct_ms: Optional[int], blocked=MotionRefused,
                              recovery: bool = False):
        """A pre-planned path of a running motion, just before it executes: from where
        the arm IS (contact or load may have stopped the servos short of the commanded
        pose) and against the wheel envelope as it is NOW. The path re-anchored at the
        measured pose is re-validated; if it fails, a new path to `goal` is planned. No
        executable path -> `blocked` (the return home: ReturnBlocked = hold and report,
        SR-17 rule 13). The re-plan is automatic (SR-17 rule 11): direct path only while
        planner.automatic_paths is direct_only, also inside an operator's pick.
        `recovery` (P's return home only, user decision 2026-10-09 option a): a measured
        pose outside the joint limits (e.g. a few ticks of load lag past a limit at the
        grip) is validated and re-planned with the joint-limit recovery - joints
        outside only move back toward their limit. Self-contact separation is NOT
        applied here: a return starting in self-contact holds ("press I to separate").

        TO-VERIFY (audit 2026-10-08): the envelope is re-checked HERE, before the
        approach and before the return, not during the ramp itself. Whether the steering
        can move while a pick runs is unknown offline; if it can, a wheel may enter the
        arm's path mid-ramp unchecked."""
        env = self._envelope()
        measured = self._q_of(here)
        arm = self._cal.joints[:5]
        path = [measured] + [{**measured, **{n: p[n] for n in arm}} for p in planned.path[1:]]
        why = None
        out = self._filter.outside_limits(measured) if recovery else {}
        if out:
            text = (f'P return: start outside joint limits ({describe_outside(out)}): '
                    f'recovery path back inside ({source})')
            self.get_logger().warn(text)
            self._report(text)
        # soft-started as planned (P, user decision 2026-10-20): the first segment leaves
        # the measured pose with planner.soft_start_step_fractions of a full ramp step
        check = self._planner(0.0, direct_ms=direct_ms, recovery=bool(out), soft_start=True)
        for i, (a, b) in enumerate(zip(path, path[1:])):
            why = check.check_segment(a, b, env, direct=planned.method == 'direct',
                                      first=i == 0)
            if why:
                break
        if why is None:
            text = (f'{source}: planned path re-validated from the measured pose against '
                    'the current envelope')
            self.get_logger().info(text)
            self._report(text)
            soft = len(path) > 1 and check.starts_soft(path[0], path[1],
                                                       planned.method == 'direct')
            return self._path_segments(path, planned.method, direct_ms, here,
                                       **({'soft': True} if soft else {}))
        text = f'{source}: planned path no longer valid from the measured pose ({why}); re-planning'
        self.get_logger().warn(text)
        self._report(text)
        try:
            _res, segs, _end = self._planned_segments(here, goal, f'{source} (re-plan)',
                                                      direct_ms, env, automatic=True,
                                                      recovery=recovery, soft_start=True)
        except MotionRefused as exc:
            raise blocked(f'{source}: no executable path from the measured pose: {exc}') \
                from exc
        return segs

    def _arm_home(self, source: str, should_abort=None, automatic: bool = True):
        """I and the return home after an abort: planned arm path, then the gripper
        (not blocked) to its home value. Both validated before the first write.
        `automatic` False only for the operator's I (SR-17 rule 11). From outside the
        joint limits: the recovery path back inside (user decision 2026-10-09)."""
        if self._home_pos is None:
            raise MotionRefused(f'{source}: {self._poses_error}')
        here = self._prepare(source)
        # Self-contact separation for the operator's I only (option C, 2026-10-09):
        # the automatic return home and startup homing hold and report instead.
        separate = not automatic and self._self_contact_mode == 'operator_go_home'
        # Ground-margin recovery likewise for the operator's I only (2026-10-12).
        rise = not automatic and self._ground_mode in ('operator', 'operator_go_home')
        start, sagged = self._home_start(here, source, automatic, rise)
        # Soft start (user decisions 2026-10-20): its first full ramp step's motion bound
        # alone refused the automatic return home from up to ~21 mm above the ground, and
        # the operator's I from a held pose 3.1 mm off the chassis (padded boxes; robot
        # test 2026-10-20). The automatic one still takes the direct path only, without
        # ground recovery (rule 11/13); a start that lifts out of the ground margin
        # takes full steps (Planner.starts_soft).
        with self._serial_lock:
            goals = dict(self._goal)          # what the servos hold while I searches
        res, segs, end = self._planned_segments(start, self._home_pos, source, HOME_MOVE_MS,
                                                automatic=automatic, recovery=True,
                                                separate_self=separate, rise_ground=rise,
                                                soft_start=True, resume=not automatic)
        watch = {**sagged, **res.ground_start}
        if not automatic:
            self._recheck_before_write(res, start, goals, here, source, separate, rise,
                                       watch)
        self._gripper_target(self._home_pos[GRIPPER_ID], end, f'{source}/gripper',
                             evaluate=False)
        guard = self._lift_guard(watch, here, source) if watch else None
        self._ramp(segs, source, 'approve', should_abort, guard)
        self._move_gripper(self._home_pos[GRIPPER_ID], 1333, f'{source}/gripper',
                           should_abort)

    def _recheck_before_write(self, res: PlanResult, start: Dict[int, int],
                              goals: Dict[int, Optional[int]], here: Dict[int, int],
                              source: str, separate: bool, rise: bool, watch):
        """The operator's I, between its (resumed, up to planner.max_search_s) search and
        the first write (audit 2026-11-02 S1, re-audit of 4b36ae2). The present positions
        and held goals are read again; per limb servo, by where its path starts:
        - at the goal it holds (`goals`, read when the search began; _held_start, or the
          torque-on sync after a Space): that goal must be unchanged. The servo drives
          toward it all along, so the first setpoint continues from what it commands,
          however the measured pose creeps or sags meanwhile; the new measured pose must
          pass the planner's own start check, as a static pose against the current
          envelope (joint limits relaxed to it, self contacts allowed while separating,
          ground margin only for links the lift guard watches) - else refused;
        - at its measured pose (where _home_start starts measured: its goal differs):
          moved more than planner.start_drift_ticks since the planning read `here` ->
          refused, the path would start where the arm no longer is (not re-planned: a
          re-plan would chase a still sagging arm, as for the lift guard).
        If the wheel envelope differs from the one the search used (res.envelope),
        every segment is checked again against the current one with the flags the
        planner checked it with (rising entries carried from segment to segment). A
        refusal writes nothing and is reported. Space ends the re-check at its next pose.
        The automatic return home keeps its one-slice gap, as before."""
        def refuse(text):
            self.get_logger().error(text)
            self._report(text)
            raise MotionRefused(text)

        now = self._read_all()
        with self._serial_lock:
            goals_now = dict(self._goal)
        if now is None:
            refuse(f'{source}: a servo did not answer before the first write; nothing written')
        names = {sid: self._cal.joints[IDS.index(sid)] for sid in LIMB_IDS}
        held = {sid for sid in LIMB_IDS if goals[sid] is not None and start[sid] == goals[sid]}
        changed = {names[sid]: (goals[sid], goals_now[sid]) for sid in held
                   if goals_now[sid] != goals[sid]}
        if changed:
            refuse(f'{source}: a held goal changed while the path was planned ((before, '
                   f'now): {changed}); nothing written - press I again')
        drift_max = int(self._p('planner.start_drift_ticks'))
        drift = {names[sid]: now[sid] - here[sid] for sid in LIMB_IDS
                 if sid not in held and abs(now[sid] - here[sid]) > drift_max}
        if drift:
            refuse(f'{source}: the arm moved while the path was planned (ticks since the '
                   f'planning read: {drift}, more than planner.start_drift_ticks = '
                   f'{drift_max}): the path no longer starts where the arm is; nothing '
                   'written - press I again')
        cur = self._envelope()
        flt, qn = self._filter, self._q_of(now)
        lim = flt.recovery_limits(qn)
        skip = {n for n in flt.ground_shortfalls(qn) if n in watch} or None if rise else None
        if separate:
            v, hits = flt.self_contacts(qn, cur, limits=lim, ground_skip=skip)
        else:
            v, hits = flt.check_pose(qn, cur, limits=lim, ground_skip=skip), []
        if not hits and not v.ok:
            refuse(f'{source}: the measured pose is no longer a valid start ({v}) - it moved '
                   'while the path was planned; nothing written')
        if cur is res.envelope:
            return
        out = self._filter.outside_limits(self._q_of(start))
        # RRT and smoothing segments were checked without separate_self (Planner._rrt)
        check = self._planner(0.0, direct_ms=HOME_MOVE_MS, recovery=bool(out),
                              separate_self=separate and res.method != 'rrt',
                              rise_ground=rise, soft_start=True)
        self._op_planner = check
        try:
            if self._spaced():
                check.cancel()
            rising: tuple = ()
            for i, (a, b) in enumerate(zip(res.path, res.path[1:])):
                trace: list = []
                why = check.check_segment(a, b, cur, direct=res.method == 'direct',
                                          first=i == 0, rising=rising, trace=trace)
                if self._spaced():
                    raise MotionStopped(f'{source}: stopped (Space) while re-checking the '
                                        'path, nothing moved')
                if why is not None:
                    text = (f'{source}: the wheel envelope changed while the path was '
                            f'planned and segment {i + 1} is no longer valid against the '
                            f'current one ({why}); nothing written - press I again')
                    self.get_logger().error(text)
                    self._report(text)
                    raise MotionRefused(text)
                rising = tuple(trace[-1]) if trace else ()
        finally:
            self._op_planner = None
        text = f'{source}: path re-checked against the changed wheel envelope before writing'
        self.get_logger().info(text)
        self._report(text)

    def _home_start(self, here: Dict[int, int], source: str, automatic: bool, rise: bool):
        """(start ticks, links to watch) for _arm_home. The operator's I from a held arm
        starts at the held goals (_held_start, user decision 2026-10-13), so its first
        setpoint does not re-anchor a sagging servo below them. The planner's start
        checks then judge the COMMANDED pose; the measured one, which lags it, is held
        to what the ground recovery demands of a start: every link statically inside its
        margin is watched by the lift guard (measured, as for a recovery start). A
        measured pose AT OR BELOW the ground (user decision 2026-10-15) starts at the
        measured pose instead (audit GR2 M1): the floor carries the arm, so there is no
        sag to avoid re-anchoring, and the lift guard cannot see an arm pressed into the
        floor (load, not a falling clearance) - only planning from the real pose puts it
        under the rise rule. With safety_filter.ground_recovery
        off, a measured pose inside the margin starts there, as before (the planner
        refuses it). The automatic return home and startup homing start at the measured
        pose: their refusals (inside the ground margin, self-contact) were decided on
        it (SR-17 rule 13)."""
        if automatic:
            return here, {}
        start = self._held_start(here, source)
        if start == here:
            return here, {}
        low = self._filter.ground_shortfalls(self._q_of(here))
        if low and not rise:
            text = (f'{source}: measured pose inside the ground margin and the ground '
                    'recovery is off: starts at the measured pose')
            self.get_logger().warn(text)
            self._report(text)
            return here, {}
        touch = {n: h for n, h in low.items() if h <= 0.0}
        if touch and rise:
            text = (f'{source}: measured pose at or below the ground '
                    f'({self._filter.describe_ground(touch)}): starts at the measured pose, '
                    'not at the held goals')
            self.get_logger().warn(text)
            self._report(text)
            return here, {}
        return start, low

    def _lift_guard(self, start: Dict[str, float], here: Dict[int, int], source: str):
        """Audit r11 F3: the recovery plan rises for the COMMANDED poses; a servo that
        sags under load (steady-state error, 2026-10-12: limb5 commanded >= 10 mm,
        measured 8.5 -> 1.2 mm with torque on) is not covered. Returns the guard for
        _ramp, or None if no entry is statically below its margin.

        Right before the first write the present positions are read again (r11b B2):
        planning took seconds, and an arm that moved more than
        safety_filter.ground_recovery_start_drift_ticks on any limb servo since `here`
        is refused - nothing written (a re-plan would chase a still sagging arm). The
        guard's baseline is that read.

        The guard watches only the entries statically below their margin at that read
        (r11b B1: a link above it, in the trigger only through the motion bound, is
        held by the normal check later). After every written setpoint it reads the
        present positions; per entry: at or above its margin -> no longer watched;
        else below its baseline minus safety_filter.ground_recovery_sag_tolerance_m at
        TWO consecutive reads -> GroundSagHold (no further write, torque on, the last
        goal kept). One-tick reading noise alone does not trip it. Space -> MotionStopped
        (r11b B3). The read holds the serial lock: Space waits at most one read cycle
        (6 servos) before its torque-off (r11b B4). Logs present vs goal of
        limb2..limb4 and the measured clearances - the robot run's evidence."""
        flt, cal = self._filter, self._cal
        drift_max = int(self._p('safety_filter.ground_recovery_start_drift_ticks'))
        now = self._read_all()
        if now is None:
            raise MotionRefused(f'{source}: ground recovery - a servo did not answer before '
                                'the lift; nothing written')
        drift = {cal.joints[IDS.index(sid)]: now[sid] - here[sid] for sid in LIMB_IDS
                 if abs(now[sid] - here[sid]) > drift_max}
        if drift:
            raise MotionRefused(f'{source}: ground recovery - the arm moved while the path '
                                f'was planned (ticks since the planning read: {drift}, more '
                                f'than {drift_max}): it sags under load; nothing written - '
                                'press I again once it rests, or take the torque off')
        h0 = flt.ground_heights(self._q_of(now))
        base = {n: h0[n] for n in start if h0[n] < flt.ground_need(n)}
        if not base:
            return None
        tol = float(self._p('safety_filter.ground_recovery_sag_tolerance_m'))
        state = {'n': 0, 'strikes': {n: 0 for n in base}}

        def stopped():
            if self._stop.is_set() or self._spaced():
                raise MotionStopped(f'{source}: stopped (Space), torque off')

        def guard(sp: Dict[int, int]):
            watch = state['strikes']
            if not watch:
                return
            stopped()
            state['n'] += 1
            present = self._read_all()
            stopped()
            if present is None:
                raise GroundSagHold(f'{source}: ground recovery - a servo did not answer '
                                    'during the lift, the clearance cannot be verified: '
                                    'stopped, last goal kept')
            h = flt.ground_heights(self._q_of(present))
            lag = {cal.joints[IDS.index(s)]: present[s] - sp[s] for s in (2, 3, 4) if s in sp}
            with self._serial_lock:
                loads = self._read_loads((2, 3, 4))
            self.get_logger().info(
                f'{source}: lift setpoint {state["n"]}: present - goal (ticks) {lag}; '
                f'present load (likely per mille of max PWM duty, TO-VERIFY) {loads}; '
                'measured clearance ' + ', '.join(f'{n} {h[n] * 1e3:.1f} mm (start '
                                                  f'{base[n] * 1e3:.1f})' for n in watch))
            for n in list(watch):
                if h[n] >= flt.ground_need(n):
                    del watch[n]
                elif h[n] < base[n] - tol:
                    watch[n] += 1
                    if watch[n] >= 2:
                        raise GroundSagHold(
                            f'{source}: ground recovery - measured clearance below the start '
                            f'(tolerance {tol * 1e3:.2f} mm, two reads in a row): {n} '
                            f'{h[n] * 1e3:.1f} mm < {base[n] * 1e3:.1f} mm - the arm sags '
                            'under load; stopped, torque on, last goal kept')
                else:
                    watch[n] = 0
            if not watch:
                done = f'{source}: lifted out of the ground margin (measured)'
                self.get_logger().info(done)
                self._report(done)
        return guard

    def _claim(self, owner: str) -> bool:
        with self._state_lock:
            if self._motion_owner is not None or self._leader_state != LEADER_OFF:
                return False
            self._motion_owner = owner
            self._claim_epoch = self._epoch_now()
            self._pending_home = None          # an explicit command supersedes it
            self._stop.clear()
            return True

    def _release(self, owner: str):
        with self._state_lock:
            if self._motion_owner == owner:
                self._motion_owner = None

    def _claim_after_leader(self, owner: str, source: str, wait: bool = True,
                            rx: Optional[float] = None, epoch: Optional[int] = None):
        """Claim the arm for an operator command (O/I, operator P). Refused at once if
        another motion holds it. If ONLY leader mode does, wait up to
        leader.command_wait_sec for it to end (user decision 2026-10-09): the keyboard
        teleop sends the leader disable and the command on separate topics, whose
        arrival order is not guaranteed - the command must not lose to its own disable.
        The wait also covers the leader's _leave (hold, then the return home it
        schedules); the claim then cancels that return home. Space during the wait
        refuses. Runs in a reentrant-group thread; the enable callback has its own
        group, so polling here never starves it. `wait` False: refused at once.

        Space wins over every command received before it (audit r8 W2): `rx` is the
        command's receipt time (_rx_time), and a Space received at or after it that
        has been handled refuses the claim - also when this callback only STARTS after
        that Space, i.e. the command was still queued. A Space received after the
        claim finds the arm claimed (moving) and stops it. Only ONE command waits at a
        time (audit r8 W1): each waiter holds an executor thread for up to the wait,
        and a pile of them delayed Space and the leader tick; a second command while
        one waits is refused at once (it would lose the claim to the first anyway)."""
        if rx is None:
            rx = time.monotonic()
        with self._state_lock:
            stops = self._stop_count
        limit = float(self._p('leader.command_wait_sec')) if wait else 0.0
        deadline = time.monotonic() + limit
        waiting = False
        try:
            while True:
                with self._state_lock:
                    if self._stop_count != stops:
                        raise MotionRefused(f'{source}: Space while waiting for leader mode '
                                            'to switch off - refused, nothing moves')
                    if epoch is not None and epoch != self._epoch_now():
                        raise MotionRefused(self._epoch_text(source, epoch))
                    if self._stop_rx >= rx:
                        raise MotionRefused(f'{source}: Space received after this command '
                                            '- refused, nothing moves')
                    if self._motion_owner not in (None, 'leader'):
                        raise MotionRefused(f'{source}: arm motion "{self._motion_owner}" '
                                            'is active')
                    if self._leader_state == LEADER_OFF and not self._leaving:
                        if self._claim(owner):
                            return
                        raise MotionRefused(f'{source}: another arm motion is active')
                    if limit > 0.0 and not waiting:
                        if self._cmd_waiting:
                            raise MotionRefused(
                                f'{source}: another command is already waiting for leader '
                                'mode to switch off - refused')
                        self._cmd_waiting = waiting = True
                if time.monotonic() >= deadline:
                    raise MotionRefused(
                        f'{source}: leader mode is active' + (
                            f' (still on after waiting {limit:.2f} s for it to switch off)'
                            if wait else '') + ' - switch it off (J) first')
                time.sleep(0.01)
        finally:
            if waiting:
                with self._state_lock:
                    self._cmd_waiting = False

    def _run(self, owner: str, fn, source: str, rx: Optional[float] = None):
        self._claim_after_leader(owner, source, rx=rx)
        try:
            return fn()
        finally:
            self._release(owner)

    # ================================================================ stops / aborts
    def _epoch_now(self) -> int:
        return self._epoch_base + self._stop_count

    def _epoch_text(self, source: str, epoch: int) -> str:
        return (f'{source}: stop epoch {epoch or "missing"} is not the current '
                f'{self._epoch_now()} - a Space the sender had not seen yet (or an old/'
                'unfilled goal): refused, nothing moves; send P again')

    def _spaced(self) -> bool:
        """A Space handled since the current owner claimed the arm. Authoritative for
        _prepare and the torque-on: Space raises the count under the state lock BEFORE
        it takes the serial lock for its torque-off, so a check under the serial lock
        sees every Space whose torque-off could come after the check."""
        return self._epoch_now() != self._claim_epoch

    def _publish_epoch(self):
        try:
            self._epoch_pub.publish(Int64(data=self._epoch_now()))
        except Exception as exc:     # noqa: BLE001 - never let a publish end Space
            self.get_logger().error(f'stop epoch publish failed: {exc!r}')

    def _moving(self) -> bool:
        with self._state_lock:
            # A running leader exit (_leave: hold, then the return home it schedules)
            # counts as moving: Space then goes limp, and _leave sees the Space.
            return (self._motion_owner is not None or self._leader_state != LEADER_OFF
                    or self._leaving > 0
                    or time.monotonic() - self._last_motion < self._grace)

    def _rx(self, info, topic: str) -> float:
        """_rx_time; a delivery without a DDS receipt stamp is reported once: the order
        against Space then falls back to the callback start (weaker, see _rx_time).
        info None = a direct call (tests), not a delivery."""
        t, stamped = _rx_time(info)
        if info is not None and not stamped and not self._rx_warned:
            self._rx_warned = True
            text = (f'{topic}: message without a DDS receipt stamp (rmw '
                    f'{rclpy.get_rmw_implementation_identifier()}): Space-vs-command order '
                    'falls back to callback start (weaker) - reported once')
            self.get_logger().warn(text)
            self._report(text)
        return t

    def _setup_stop_listener(self):
        """/arm/stop on its OWN node, executor and thread (audit AP1 M2): no goal, plan
        or preview occupying the four executor threads can delay Space. The handler
        takes only the state and serial locks, as before."""
        from rclpy.executors import SingleThreadedExecutor
        self._stop_node = Node(f'{self.get_name()}_stop', context=self.context,
                               enable_rosout=False, start_parameter_services=False,
                               parameter_overrides=[Parameter('use_sim_time', value=False)])
        self._stop_node.create_subscription(Empty, '/arm/stop',
                                            lambda msg, info: self._on_stop(msg, info), 10)
        self._stop_exec = SingleThreadedExecutor(context=self.context)
        self._stop_exec.add_node(self._stop_node)
        self._stop_listening = True

        def spin():
            while self._stop_listening:
                try:
                    self._stop_exec.spin_once(timeout_sec=0.1)
                except ExternalShutdownException:
                    return
                except Exception as exc:     # noqa: BLE001 - Space must keep being served
                    if not self._stop_listening or not self.context.ok():
                        return
                    self.get_logger().error(f'/arm/stop listener: {exc!r} - continuing',
                                            throttle_duration_sec=10.0)
                    time.sleep(SPIN_RESUME_BACKOFF_S)
        self._stop_thread = threading.Thread(target=spin, name='arm_stop', daemon=True)
        self._stop_thread.start()

    def _stop_listener_shutdown(self):
        if not getattr(self, '_stop_listening', False):
            return
        self._stop_listening = False
        try:
            self._stop_thread.join(1.0)
            self._stop_exec.shutdown(timeout_sec=1.0)
            self._stop_node.destroy_node()
        except Exception as exc:     # noqa: BLE001 - shutdown must finish
            self.get_logger().warn(f'/arm/stop listener shutdown: {exc!r}')

    def _on_stop(self, _msg: Empty, info=None):
        """Space. Moving -> torque off on every arm servo; stationary with a servo
        pushing (_pushing, user decision 2026-10-13) -> the same; otherwise nothing."""
        rx = self._rx(info, '/arm/stop')
        with self._state_lock:
            moving = self._moving()
            self._stop_count += 1              # a leader activation in progress is refused
            self._stop_rx = max(self._stop_rx, rx)   # and every command received before
            self._activation_requested = None  # and one requested but not yet started
            self._pending_home = None
            leader = self._leader_state != LEADER_OFF
            if moving:
                self._stop.set()
            self._leader_state = LEADER_OFF
            if self._motion_owner == 'leader':
                self._motion_owner = None
        self._drop_replan()
        op = self._op_planner
        if op is not None:
            op.cancel()
        wait = self._stop_lock_timeout
        pushing, stuck = ({}, {}) if moving else self._pushing(wait)
        busy = 'bus' in pushing and 'busy' in str(pushing['bus'])
        if moving:                             # torque first: a publish may raise
            self._torque_off_all('Space while the arm was moving', wait)
        elif pushing:
            self._torque_off_all('Space on a stationary arm with servo(s) pushing (present - '
                                 f'goal beyond {POS_TOLERANCE} ticks, or unknown): {pushing}',
                                 0.0 if busy else wait)
        elif stuck:
            self._torque_off_all(f'Space: retrying the torque-off of servo(s) {stuck}, whose '
                                 'last torque-off was not acknowledged - they may still hold '
                                 'torque', wait)
        elif self._torque_off_pending is not None:
            stuck = {'pending': self._torque_off_pending}
            self._torque_off_all(f'Space: retrying the pending torque-off '
                                 f'({self._torque_off_pending})', wait)
        pushing = pushing or stuck
        self._publish_epoch()
        if leader:
            self._publish_leader_state('off (Space)')
        # After the torque-off; bounded like it (not free in time: the leader tick retries)
        self._restore_grip_limit('Space', lock_timeout=wait)
        if not moving and not pushing:
            self.get_logger().info('Space: arm stationary - no arm action')
            self._report('Space: arm stationary - no arm action')

    def _pushing(self, lock_timeout: float):
        """(pushing, stuck) for Space on a stationary arm. User decision 2026-10-13
        (F1 option a, "Bei Leertaste für Servos a"): the arm still counts for Space while
        torque is on and a servo is more than POS_TOLERANCE from its goal - since a hold
        keeps the last goal, such a servo may push against an obstruction indefinitely.
        pushing = {sid: present - goal} for those, 'goal unknown' / 'unreadable' where it
        cannot be told (counted as pushing); a bus error, or the serial lock not free
        within `lock_timeout` (a transaction hangs), counts as pushing too ('bus').
        stuck = the servos whose last torque-off failed (_torque_stuck; with _torque_on
        False they are not seen as pushing). Never raises."""
        out: Dict[int, object] = {}
        if self._bus.lost is not None:
            return {'bus': f'disconnected ({self._bus.lost})'}, {}
        if not self._serial_lock.acquire(timeout=lock_timeout):
            return {'bus': f'busy (serial lock not free within {lock_timeout:.2f} s)'}, {}
        try:
            stuck = dict(self._torque_stuck)
            if not self._torque_on:
                return {}, stuck
            goals = dict(self._goal)
            for sid in IDS:
                v = self._bus.read_pos(sid)
                if v is None:
                    out[sid] = 'unreadable'
                elif goals[sid] is None:
                    out[sid] = 'goal unknown'
                else:
                    d = tick_deviation(v, goals[sid])
                    if abs(d) > POS_TOLERANCE:
                        out[sid] = d
        except Exception as exc:     # noqa: BLE001 - Space must never raise
            out['bus'] = repr(exc)
            stuck = {}
        finally:
            self._serial_lock.release()
        return out, stuck

    def _schedule_home(self, reason: str, stops: Optional[int] = None):
        """`stops`: _stop_count when the aborted motion still held the arm. A Space
        since then (between the abort and this call, audit r9 L1) wins: nothing is
        scheduled - otherwise the return home would re-power an arm Space left limp."""
        with self._state_lock:
            spaced = stops is not None and self._stop_count != stops
            # The aborted motion has released the arm; anyone holding it now claimed it
            # in between - a newer command, which supersedes the return home (_claim).
            busy = self._motion_owner or (self._leader_state != LEADER_OFF and 'leader')
            if not busy and not spaced:
                self._pending_home = (time.monotonic() + self._grace, reason)
        if spaced:
            text = (f'return home after abort ({reason}) NOT scheduled: Space during the '
                    'abort - the arm stays as Space left it')
            self.get_logger().warn(text)
            self._report(text)
            return
        if busy:
            text = (f'return home after abort ({reason}) NOT scheduled: superseded by '
                    f'"{busy}", which claimed the arm after the abort')
            self.get_logger().error(text)
            self._report(text)
            return
        self.get_logger().warn(f'abort ({reason}): returning home in {self._grace:.2f} s '
                               'unless Space or an O/I/P command arrives first')

    def _pending_home_tick(self):
        # Taking the pending home and claiming the arm happen under ONE lock: a Space
        # either comes first (and cancels the pending home) or finds the arm claimed,
        # i.e. moving, and turns the torque off - never a gap in which it is lost.
        with self._state_lock:
            pending = self._pending_home
            if pending is None or time.monotonic() < pending[0]:
                return
            self._pending_home = None
            busy = self._motion_owner or (self._leader_state != LEADER_OFF and 'leader')
            claimed = self._claim('abort_home')
        if not claimed:
            # Unreachable today, kept as a guard: _claim and leader activation clear
            # _pending_home, and _schedule_home sets none while the arm is held, so a
            # due return home never meets a holder (the test injects one by hand).
            # Not retried: whoever holds the arm claimed it AFTER the abort (the aborted
            # motion released it before _schedule_home), i.e. a newer command, and a
            # newer command supersedes the return home (_claim). Never silent, though.
            text = (f'return home after abort ({pending[1]}) NOT performed: superseded by '
                    f'"{busy}", which claimed the arm after the abort')
            self.get_logger().error(text)
            self._report(text)
            return
        try:
            self._arm_home(f'abort_home ({pending[1]})')
        except MotionRefused as exc:
            self._hold(f'return home after abort NOT possible, arm holds: {exc}')
        except Exception as exc:     # noqa: BLE001 - never leave a moved arm unattended
            self.get_logger().error(f'abort_home: unexpected error: {traceback.format_exc()}')
            self._hold(f'return home after abort NOT possible (unexpected error), arm '
                       f'holds: {exc!r}')
        finally:
            self._release('abort_home')

    # ================================================================ /arm/command
    def _on_command(self, msg: String, info=None):
        cmd = msg.data.strip().lower()
        rx = self._rx(info, '/arm/command')
        t0 = time.monotonic()
        try:
            if cmd == 'open_gripper':
                self.get_logger().info('arm/command: opening gripper (unchecked)')
                if self._grip_open is None:
                    raise MotionRefused(self._poses_error)
                self._run('command', lambda: self._move_gripper(self._grip_open, 2400, cmd),
                          cmd, rx)
            elif cmd == 'go_home':
                self.get_logger().info('arm/command: home position (planned)')
                self._run('command', lambda: self._arm_home(cmd, automatic=False), cmd, rx)
            else:
                self.get_logger().warn(f'arm/command: unknown "{cmd}"')
        except MotionStopped as exc:
            self.get_logger().error(f'arm/command {cmd}: {exc}')
            self._report(f'STOPPED: {exc}')
        except GroundSagHold as exc:
            # no hold write: the last goal stays (writing the sagged measured pose would
            # lower the arm again)
            self.get_logger().error(f'arm/command {cmd} STOPPED: {exc}')
            self._report(f'HOLD (last goal kept): {exc}')
        except MotionRefused as exc:
            self.get_logger().error(f'arm/command {cmd} REFUSED: {exc}')
            self._report(f'REFUSED: {exc}')
            if self._last_motion >= t0:          # it had started moving: never silent
                self._hold(f'arm/command {cmd} refused after motion started: {exc}')
        except Exception as exc:     # noqa: BLE001 - never leave a moved arm unattended
            self.get_logger().error(f'arm/command {cmd}: unexpected error: '
                                    f'{traceback.format_exc()}')
            self._report(f'REFUSED: arm/command {cmd}: unexpected error: {exc!r}')
            if self._last_motion >= t0:
                self._hold(f'arm/command {cmd}: unexpected error after motion started: '
                           f'{exc!r}')

    # ================================================================ /pick_plastic
    def _goal_cb(self, goal):
        # An operator P while ONLY leader mode is active is accepted: _execute_cb waits
        # for the leader disable sent with it (_claim_after_leader). A gateway P is not:
        # it is rejected as before (fail-closed - it is no operator's leader exit).
        # The receipt time for the Space rule (_claim_after_leader) is taken here: the
        # execute callback can start much later. A goal request carries no DDS stamp
        # in rclpy, so this is the callback start - weaker than a topic's _rx_time.
        rx = time.monotonic()
        if bool(getattr(goal, 'preview', False)):
            # Audit AP1 M2: a preview holds no claim, so at most ONE runs at a time.
            with self._state_lock:
                busy = self._preview_busy or self._motion_owner is not None
                if not busy:
                    self._preview_busy = True
            if busy:
                self.get_logger().warn('pick_plastic preview REJECTED: another preview or an '
                                       'arm motion is running')
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT
        with self._state_lock:
            other = self._motion_owner not in (None, 'leader')
            leader = self._leader_state != LEADER_OFF
        busy = other or (leader and self._pick_trigger(goal)[0])
        if busy:
            self.get_logger().warn('pick_plastic REJECTED: leader mode or another arm motion '
                                   'is active (FR-17 item 8)')
            return GoalResponse.REJECT
        epoch = int(getattr(goal, 'stop_epoch', 0))
        if epoch != self._epoch_now():
            text = f'REJECTED: {self._epoch_text("pick_plastic", epoch)}'
            self.get_logger().warn(text)
            self._report(text)
            return GoalResponse.REJECT
        with self._state_lock:
            for k in [k for k, (_g, t) in self._goal_rx.items() if rx - t > 60.0]:
                del self._goal_rx[k]           # accepted, but never executed
            self._goal_rx[id(goal)] = (goal, rx)
        return GoalResponse.ACCEPT

    def _pick_trigger(self, goal):
        """(automatic, why) for a /pick_plastic goal - SR-17 rule 11 needs the sender,
        and the action carries NO designed sender field. Two observable facts are used,
        fail-closed: the external-goal gateway sends PickPlastic only while ARMED
        (SR-16) and sets execute: true; the keyboard/web teleop sends a default goal
        (execute: false). A goal counts as the OPERATOR's only if execute is false AND
        the gateway is known to be DISARMED; anything else is treated as
        gateway-triggered. A misclassified operator P only loses its detours; it never
        gains motion. A designed sender field would replace this."""
        execute = bool(getattr(goal, 'execute', True))
        armed = self._gateway_armed
        if not execute and armed is False:
            return False, 'operator (execute false, gateway disarmed)'
        return True, ('gateway/automatic (' + ', '.join(
            w for w, c in (('execute true', execute), ('gateway ARMED', armed is True),
                           ('gateway arming state UNKNOWN', armed is None)) if c) + ')')

    def _execute_cb(self, goal_handle):
        if bool(getattr(goal_handle.request, 'preview', False)):
            return self._aim_preview(goal_handle)
        result = PickPlastic.Result()
        with self._state_lock:
            got = self._goal_rx.pop(id(goal_handle.request), None)
        rx = got[1] if got is not None and got[0] is goal_handle.request else None
        if self._pick_sequence is None:
            goal_handle.abort()
            result.success, result.message = False, f'Refused: {self._poses_error}'
            result.reason = 'REFUSED'
            return result
        try:
            self._claim_after_leader('pick', 'pick_plastic',
                                     wait=not self._pick_trigger(goal_handle.request)[0],
                                     rx=rx, epoch=int(goal_handle.request.stop_epoch))
        except MotionRefused as exc:
            self.get_logger().error(f'pick_plastic REFUSED: {exc}')
            self._report(f'REFUSED: {exc}')
            goal_handle.abort()
            result.success, result.message = False, f'Refused: {exc}'
            result.reason = 'REFUSED'
            return result
        home_after = None
        with self._state_lock:
            stops = self._stop_count        # a Space after the abort cancels its return
        t0 = time.monotonic()
        cancel = lambda: goal_handle.is_cancel_requested  # noqa: E731
        watch = lambda: False  # noqa: E731 - fixed P: no base-motion watch
        abort = cancel
        try:
            # EVERYTHING is planned, converted to ticks and validated before the first
            # write: both arm paths (approach, return home) and the three gripper
            # targets. Nothing that can be refused up front is refused after motion.
            automatic, trigger = self._pick_trigger(goal_handle.request)
            aim = None
            if bool(getattr(goal_handle.request, 'aimed', False)):
                # Before _prepare: a refusal here writes nothing to the bus at all.
                aim = self._aim_start(goal_handle.request, result)
            text = (f'pick_plastic: start - trigger {trigger}; '
                    + ('AIMED at ({:+.3f}, {:+.3f}) in base_link; '.format(*aim.target_base[:2])
                       if aim is not None else 'fixed grip pose; ')
                    + 'planning both arm paths')
            self.get_logger().info(text)
            self._report(text)
            if aim is None:
                here = self._prepare('pick_plastic')
            else:
                # Aimed P (audit AP1 M4): both arm paths are planned from the READ pose,
                # before any write - the torque-on sync (_prepare) comes with the first
                # step, so every refusal of an aimed P leaves the bus untouched.
                if self._spaced():
                    raise MotionStopped('pick_plastic: stopped (Space) before planning')
                here = self._read_all()
                if here is None:
                    raise MotionRefused('pick_plastic: no measured arm pose '
                                        f'({self._read_failure()})')
            why = self._outside(self._q_of(here), 'pick_plastic', 'P is')
            if why:
                self._report(why)
                raise MotionRefused(why)
            self._gripper_target(self._grip_open, here, 'pick_plastic/open', evaluate=False)
            g_open = {**here, GRIPPER_ID: self._grip_open}
            plan = lift = None
            if aim is None:
                grip_pos = self._grip_pos
                approach, _segs, at_grip = self._planned_segments(
                    g_open, grip_pos, 'pick_plastic/approach', 4000, automatic=automatic,
                    soft_start=True)
            else:
                # Steep approach (user decision 2026-11-16): the approach ends at the
                # pre-grasp pose, a vertical descent reaches the grasp.
                plan = self._aim_approach(aim, g_open, automatic)
                grip_pos, approach, at_grip = plan.pre, plan.approach, plan.at_grip
            self._gripper_target(self._grip_close, at_grip, 'pick_plastic/grip_close',
                                 evaluate=False)
            at_grip = {**at_grip, GRIPPER_ID: self._grip_close}
            if plan is None:
                back, _segs, at_home = self._planned_segments(
                    at_grip, self._home_pos, 'pick_plastic/return_home', HOME_MOVE_MS,
                    automatic=automatic, soft_start=True)
                back_from = at_grip
            else:
                lift, back, back_from, at_home = self._aim_return(plan, automatic,
                                                                  'pick_plastic')
            self._gripper_target(self._home_pos[GRIPPER_ID], at_home,
                                 'pick_plastic/gripper_home', evaluate=False)

            def approach_grip():
                if aim is not None:
                    self._aim_still(aim, 'before the approach')
                now = self._prepare('pick_plastic/approach')
                segs = self._revalidated_segments(now, approach, grip_pos,
                                                  'pick_plastic/approach', 4000)
                self._ramp(segs, 'pick/approach', 'approve', abort)

            def descend():
                # the second motion segment of the aimed approach: the same base-still
                # check, Space/torque order and abort closure as the first
                self._aim_still(aim, 'before the descent')
                now = self._prepare('pick_plastic/descent')
                segs = self._aim_vertical(now, plan.cand.descent[1:], 'pick_plastic/descent')
                self._ramp(segs, 'pick/descent', 'approve', abort)

            def lift_up():
                now = self._prepare('pick_plastic/lift')
                try:
                    segs = self._aim_vertical(now, lift, 'pick_plastic/lift')
                except MotionRefused as exc:
                    # as before the steep approach: the return re-anchors at the
                    # measured pose (re-plans if it must), so nothing is lost
                    text = (f'pick_plastic/lift skipped, no vertical lift from the measured '
                            f'pose ({exc}) - the return home starts at the grasp')
                    self.get_logger().warn(text)
                    self._report(text)
                    return
                self._ramp(segs, 'pick/lift', 'approve', abort)

            def return_home():
                now = self._prepare('pick_plastic/return_home')
                self._verify_positions({s: back_from[s] for s in LIMB_IDS},
                                       'pick_plastic/grip (before the return)')
                segs = self._revalidated_segments(now, back, self._home_pos,
                                                  'pick_plastic/return_home', HOME_MOVE_MS,
                                                  ReturnBlocked, recovery=True)
                self._ramp(segs, 'pick/return_home', 'approve', abort)

            feedback = PickPlastic.Feedback()
            # the aimed approach is two motions: to the pre-grasp pose (a short settle
            # before the descent's base-still check), then the vertical descent
            steps = [
                ('open', lambda: self._move_gripper(self._grip_open, 1000, 'pick/open', abort),
                 0),
                ('approach', approach_grip, 800 if plan is None else 300),
                *([('descent', descend, 800)] if plan is not None else []),
                ('grip_close', lambda: self._move_gripper(self._grip_close, 2400,
                                                          'pick/grip_close', abort), 700),
                *([('lift', lift_up, 0)] if lift is not None else []),
                ('return_home', return_home, 0),
                ('gripper_home', lambda: self._move_gripper(self._home_pos[GRIPPER_ID], 1333,
                                                            'pick/gripper_home', abort), 900),
            ]
            if aim is not None:
                # SR-16 E5 (base at rest: BASE_MOVING before, abort during), E3 (the one
                # transform is valid only for the pose it was taken in), audit F3.
                self._aim_still(aim, 'before the first motion')
                # From here to the end of the P - every ramp setpoint, every pause
                # between steps - the base is watched (_aim_watch). Abort = the cancel
                # path: MotionAborted, return home (SR-17 rule 13), own reason.
                watch = self._aim_watch(aim)
                abort = lambda: cancel() or watch()  # noqa: E731
            for idx, (step, run, extra_ms) in enumerate(steps):
                if self._stop.is_set() or self._spaced():
                    raise MotionStopped('pick_plastic stopped (Space)')
                if goal_handle.is_cancel_requested:
                    raise MotionAborted('pick_plastic canceled')
                watch()
                self.get_logger().info(f'pick_plastic: {idx+1}/{len(steps)} — {step}')
                if step in ('grip_close', 'gripper_home'):
                    self._prepare(f'pick_plastic/{step}')     # Space before torque-on
                run()
                feedback.progress = (idx + 1) / len(steps)
                feedback.current_step = step
                goal_handle.publish_feedback(feedback)
                # Watched pauses; the one after the last step is not part of the motion
                # (the arm is home), so a base move then does not fail a done P.
                pause_end = time.monotonic() + extra_ms / 1000.0
                while (left := pause_end - time.monotonic()) > 0:
                    time.sleep(min(left, 1.0 / MOVE_STEP_HZ))
                    if idx + 1 < len(steps):
                        watch()
            self._verify_positions(self._home_pos, 'pick_plastic')
            goal_handle.succeed()
            result.success, result.message = True, 'Grip sequence completed'
            result.reason = ''
            self.get_logger().info('pick_plastic: done')
            return result
        except MotionStopped as exc:
            result.success, result.message = False, f'Stopped (Space) - torque off: {exc}'
            result.reason = 'STOPPED'
        except MotionAborted as exc:
            # BaseMovedAbort (SR-16 E5) keeps its own reason; the return is the same.
            result.success, result.message = False, f'Canceled - returning home: {exc}'
            result.reason = getattr(exc, 'reason', None) or 'CANCELED'
            home_after = ('pick_plastic aborted: base moved' if isinstance(exc, BaseMovedAbort)
                          else 'pick_plastic canceled')
        except ReturnBlocked as exc:
            # SR-17 rule 13: no path home -> the arm holds and reports.
            self._hold(f'pick_plastic: {exc}')
            result.success, result.message = False, f'Holding - no path home: {exc}'
            result.reason = 'HOLDING'
        except MotionRefused as exc:
            result.reason = getattr(exc, 'reason', None) or (
                aimed_mod.NO_PATH if 'NO_PATH' in str(exc) else 'REFUSED')
            if self._last_motion >= t0:
                # Audit AP1 M5: whatever the cause, a refusal after motion started is
                # its own reason - never one the gateway may answer with a re-approach.
                result.reason = aimed_mod.REFUSED_AFTER_MOTION
                # Refused after motion started: hold where the arm is (rule 6), then
                # the planned return home (rule 13; it holds and reports if no path).
                self._hold(f'pick_plastic refused after motion started: {exc}')
                result.success = False
                result.message = f'Refused after motion started - holding, returning home: {exc}'
                home_after = 'pick_plastic refused after motion started'
            else:
                result.success, result.message = False, f'Refused: {exc}'
        except Exception as exc:     # noqa: BLE001 - never leave a moved arm unattended
            # Not a refusal the code foresaw: after the first write the arm HOLDS and
            # the error is reported; no automatic return home (SR-17 rule 13, user decision 2026-10-09).
            self.get_logger().error(f'pick_plastic: unexpected error: {traceback.format_exc()}')
            if self._last_motion >= t0:
                self._hold(f'pick_plastic: unexpected error after motion started, arm '
                           f'holds, no automatic return home: {exc!r}')
                result.message = f'Error after motion started - holding: {exc!r}'
            else:
                self._report(f'REFUSED: pick_plastic: unexpected error: {exc!r}')
                result.message = f'Refused (error): {exc!r}'
            result.success = False
            result.reason = 'ERROR'
        finally:
            self._release('pick')
        if self._epoch_now() != int(goal_handle.request.stop_epoch) \
                and not result.message.startswith('Stopped (Space)'):
            # Whatever ended it (cancel, refusal, error), a Space came: it is the stop.
            result.message = f'Stopped (Space) - torque off; ended as: {result.message}'
            result.reason = 'STOPPED'
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        if home_after:
            self._schedule_home(home_after, stops)
        return result

    # ================================================================ aimed P (FR-16 stage 1)
    # User decisions 2026-10-13: P grasps at P's grip height but at the litter's (x, y),
    # gripper vertical, wrist roll free (searched, logged); a target outside the allowed
    # region is OUT_OF_REACH (the gateway re-approaches); the litter position's own age
    # is not limited. Everything else of P - claim, stop epoch, Space, planning, direct
    # path when automatic, ground recovery, hold and return rules - is unchanged: only
    # the grasp pose differs. Kept in these functions so the pick sequence stays one.
    # User decision 2026-11-16 (steep approach from above): the planned approach ends
    # at a pre-grasp pose aimed.pregrasp_height_m above the grasp; a vertical descent
    # (_aim_vertical, same roll) reaches the grasp and, after the close, lifts back up
    # the same line before the planned return home. Each is its own step, under the
    # same Space / cancel / base-watch abort as the approach.

    def _declare_aimed_parameters(self):
        """config/arm_safety_filter.yaml, section aimed.*, explains each value."""
        am = {
            'enabled': True,
            'tilt_deg': 0.0,
            'roll_offsets_deg': [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 120.0, -120.0,
                                 150.0, -150.0, 180.0],
            'max_candidates': 4,
            'base_frame': 'base_link',
            'max_tf_future_sec': 0.05,
            'target_frames': ['map', 'odom', 'base_link'],
            'still_frame': 'odom',
            'tf_queue_depth': 10,
            'tf_poll_period_sec': 0.02,
            'still_max_translation_m': 0.005,
            'still_max_rotation_rad': 0.0087,
            'still_min_stamp_advance_sec': 0.05,
            'region_at_start': False,
            'region_step_m': 0.02,
            'region_topic': '/arm/aimed/region',
        }
        for k, v in am.items():
            self.declare_parameter(f'aimed.{k}', v)
        # Startup-only (SR-15 rule 14 applied to the aimed P's inputs): read once here,
        # a later `ros2 param set` changes nothing; the region is fixed for the node.
        self._aim_cfg = {k: self._p(f'aimed.{k}') for k in am}
        # SR-16 E3 / audit F4 (a): the TF age bound is ONE bound with the goal gateway,
        # top-level max_tf_age_sec from gripperx_arm_msgs/config/tf_freshness.yaml. No
        # code default: missing -> every aimed P is refused (_aim_lookup).
        self.declare_parameter('max_tf_age_sec', Parameter.Type.DOUBLE)
        try:
            tf_age = float(self._p('max_tf_age_sec'))
        except Exception:      # noqa: BLE001 - uninitialised: the shared file is missing
            tf_age = None
        self._aim_cfg['max_tf_age_sec'] = (
            tf_age if tf_age is not None and math.isfinite(tf_age) and tf_age > 0.0 else None)
        if self._aim_cfg['max_tf_age_sec'] is None:
            self.get_logger().error(
                f'max_tf_age_sec missing or invalid ({tf_age}): the shared tf_freshness.yaml '
                '(gripperx_arm_msgs) is not loaded - every aimed P is refused '
                '(AIMED_UNAVAILABLE); fixed P unchanged')
        # SR-16 E4 / audit 2026-11-13 N7: the inward region margin (horizontal TCP
        # calibration error). Startup-only and NO code default - E12 (d) needs a
        # measured value before mode automatic; missing or invalid -> no margin for
        # the interim mode and the operator's P, automatic refused (_aim_margin).
        self.declare_parameter('aimed.region_margin_m', Parameter.Type.DOUBLE)
        try:
            margin = float(self._p('aimed.region_margin_m'))
        except Exception:      # noqa: BLE001 - uninitialised: not set in the config
            margin = None
        self._aim_cfg['region_margin_m'] = (
            margin if margin is not None and math.isfinite(margin) and margin >= 0.0
            else None)
        if self._aim_cfg['region_margin_m'] is None:
            self.get_logger().error(
                f'aimed.region_margin_m missing or invalid ({margin}): no inward margin '
                'of the allowed grasp region (SR-16 E4, TO-VERIFY until measured, E12 (d)) '
                '- confirm_by_rearm previews/picks and the operator\'s aimed P run without '
                'a margin (E4), any other aimed P (mode automatic) is refused '
                '(AIMED_UNAVAILABLE); fixed P unchanged')
        # User decision 2026-11-16: the steep approach from above - pre-grasp pose
        # pregrasp_height_m above the grasp, then a vertical descent (aimed_grasp).
        # Startup-only and NO code default: missing or invalid -> every aimed P is
        # refused (AIMED_UNAVAILABLE, _aim_solver); the old direct approach cannot
        # come back silently. Fixed P unaffected.
        bad = []
        for k in ('pregrasp_height_m', 'descent_step_m', 'descent_max_lateral_m'):
            self.declare_parameter(f'aimed.{k}', Parameter.Type.DOUBLE)
            try:
                v = float(self._p(f'aimed.{k}'))
            except Exception:  # noqa: BLE001 - uninitialised: not set in the config
                v = None
            ok = v is not None and math.isfinite(v) and v > 0.0
            self._aim_cfg[k] = v if ok else None
            if not ok:
                bad.append(f'aimed.{k} ({v})')
        if not bad and self._aim_cfg['descent_step_m'] > self._aim_cfg['pregrasp_height_m']:
            bad.append('aimed.descent_step_m > aimed.pregrasp_height_m')
        self._aim_descent_error = (
            f'{", ".join(bad)} missing or invalid: the vertical descent from the pre-grasp '
            'pose is not configured' if bad else '')
        if bad:
            self.get_logger().error(f'{self._aim_descent_error} - every aimed P is refused '
                                    '(AIMED_UNAVAILABLE); fixed P unchanged')
        self._aim_enabled = bool(self._aim_cfg['enabled'])

    def _setup_aimed(self):
        """TF on its own node and thread (the arm's executor threads serve Space; /tf
        traffic must not compete for them), and - only with aimed.region_at_start - the
        allowed-region diagnostic, derived in the background once the filter is built."""
        self._aim_stop = threading.Event()
        self._aim_cache = None                 # (filter, cal) -> solver
        self._aim_region = None
        self._aim_region_thread = None
        self._aim_region_busy = False
        self._preview_busy = False
        self._aim_tf_age = {}
        self._tf_node = self._tf_thread = None
        if not self._aim_enabled:
            self.get_logger().warn('aimed P DISABLED (aimed.enabled false): aimed goals are '
                                   'refused (AIMED_UNAVAILABLE); fixed P unchanged')
            return
        from tf2_ros.buffer import Buffer
        from tf2_msgs.msg import TFMessage
        from rclpy.serialization import deserialize_message
        from gripperx_arm.tf_intake import TfIntake
        self._tf_buffer = Buffer()
        node = Node(f'{self.get_name()}_tf', context=self.context, enable_rosout=False,
                    start_parameter_services=False,
                    parameter_overrides=[Parameter('use_sim_time', value=False)])
        static = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # tf_intake.py (robot 2026-11-09: a full depth-100 /tf queue read ~1 s old): raw
        # /tf, deserialised only when it can feed a looked-up frame, a short queue, and
        # no executor - one thread drains both queues every aimed.tf_poll_period_sec.
        self._tf_intake = TfIntake(
            self._tf_buffer, [self._aim_cfg['base_frame'], self._aim_cfg['still_frame'],
                              *self._aim_cfg['target_frames']],
            lambda raw: deserialize_message(raw, TFMessage))
        tf_sub = node.create_subscription(TFMessage, '/tf', lambda _m: None,
                                          int(self._aim_cfg['tf_queue_depth']), raw=True)
        static_sub = node.create_subscription(TFMessage, '/tf_static', lambda _m: None,
                                              static)
        self._tf_node = node
        period = float(self._aim_cfg['tf_poll_period_sec'])

        def poll():
            while not self._aim_stop.wait(period):
                try:
                    self._tf_intake.poll(tf_sub, static_sub)
                except Exception as exc:   # noqa: BLE001 - shutdown, or a broken message
                    if self._aim_stop.is_set() or not self.context.ok():
                        return
                    self.get_logger().error(f'aimed P: TF intake: {exc!r}',
                                            throttle_duration_sec=10.0)
        self._tf_thread = threading.Thread(target=poll, name='arm_tf', daemon=True)
        self._tf_thread.start()
        if not bool(self._aim_cfg['region_at_start']):
            # USER DECISION 2026-11-05 (FR-16 item 5): the boot-time derivation is CPU-
            # bound and competes with the servo bus reads for the GIL; the region is
            # derived offline (arm_aimed_region). The per-target check is unaffected.
            return
        self._region_pub = self.create_publisher(String, str(self._aim_cfg['region_topic']),
                                                 LATCHED)
        self._aim_region_timer = self.create_timer(1.0, self._aim_region_tick,
                                                   callback_group=self._cb_group)

    def _aim_shutdown(self):
        if not hasattr(self, '_aim_stop'):
            return
        self._aim_stop.set()
        try:
            if self._tf_thread is not None:
                self._tf_thread.join(1.0)
            if self._tf_node is not None:
                self._tf_node.destroy_node()
        except Exception as exc:       # noqa: BLE001 - shutdown must finish
            self.get_logger().warn(f'aimed P: TF listener shutdown: {exc!r}')
        try:
            if self._aim_region_thread is not None:
                self._aim_region_thread.join(2.0)
        except RuntimeError:           # never let shutdown raise (destroy_node follows)
            pass

    def _aim_solver(self) -> aimed_mod.AimedGraspSolver:
        """The solver for the active filter and calibration, built once per pair.
        Grasp height and opening come from P's taught pose (arm_poses.yaml) through
        the active calibration and model - never typed in."""
        if not self._aim_enabled:
            raise AimRefused(aimed_mod.AIMED_UNAVAILABLE, 'aimed.enabled is false')
        if self._filter is None or self._cal is None or not self._config_loaded:
            raise AimRefused(aimed_mod.AIMED_UNAVAILABLE,
                             self._filter_error or 'arm_safety_filter.yaml not loaded')
        if self._grip_pos is None or self._grip_open is None:
            raise AimRefused(aimed_mod.AIMED_UNAVAILABLE, self._poses_error)
        if self._aim_descent_error:
            raise AimRefused(aimed_mod.AIMED_UNAVAILABLE, self._aim_descent_error)
        key = (id(self._filter), id(self._cal))
        cache = self._aim_cache
        if cache is not None and cache[0] == key:
            return cache[1]
        grip_q = self._q_of(self._grip_pos)
        g = self._cal.joints[IDS.index(GRIPPER_ID)]
        gopen = self._q_of({**self._grip_pos, GRIPPER_ID: self._grip_open})[g]
        ref = self._filter.envelope_measured(
            {n: 0.0 for n in self._steer_names},
            float(self._p('safety_filter.steer_motion_margin_rad')))
        solver = aimed_mod.AimedGraspSolver(
            self._filter, grip_q, gopen,
            [math.radians(float(v)) for v in self._aim_cfg['roll_offsets_deg']],
            math.radians(float(self._aim_cfg['tilt_deg'])), ref,
            pregrasp_height=self._aim_cfg['pregrasp_height_m'],
            descent_step=self._aim_cfg['descent_step_m'],
            descent_max_lateral=self._aim_cfg['descent_max_lateral_m'])
        self._aim_cache = (key, solver)
        self.get_logger().info(
            f'aimed P: grasp height {solver.z:+.4f} m in base_link (TCP of P\'s grip '
            f'pose), approach tilt {float(self._aim_cfg["tilt_deg"]):.1f} deg, gripper '
            f'open {gopen:+.3f} rad, P\'s wrist roll {math.degrees(solver.roll0):+.1f} deg, '
            f'{len(solver.roll_offsets)} roll candidates; vertical descent from '
            f'{solver.pregrasp_height * 1e3:.0f} mm above, {len(solver.descent_heights) - 1} '
            'segments')
        return solver

    def _aim_region_tick(self):
        """Start the region derivation once the filter is ready (one-shot)."""
        with self._state_lock:             # the timer's group is reentrant
            if self._aim_region_busy or self._filter is None or self._aim_stop.is_set():
                return
            self._aim_region_busy = True
        try:
            solver = self._aim_solver()
        except AimRefused as exc:
            self._aim_region_busy = False
            self.get_logger().warn(f'aimed P: allowed region not derived: {exc}',
                                   throttle_duration_sec=30.0)
            return
        except Exception as exc:       # noqa: BLE001 - AP1 L6: never stuck busy
            self._aim_region_busy = False
            self.get_logger().error(f'aimed P: region not derived: {exc!r}',
                                    throttle_duration_sec=30.0)
            return
        self._aim_region_timer.cancel()
        step = float(self._aim_cfg['region_step_m'])

        def run():
            t0 = time.monotonic()
            try:
                region = solver.derive_region(
                    step, should_stop=self._aim_stop.is_set,
                    pause=lambda: time.sleep(0.0005),
                    fingerprint_extra=(repr(self._cal.zero_ticks),))
            except Exception as exc:   # noqa: BLE001 - a diagnostic, never fatal
                if not self._aim_stop.is_set():
                    self.get_logger().error(f'aimed P: region derivation failed: {exc!r}')
                return
            if region is None or self._aim_stop.is_set():
                return
            self._aim_region = region
            text = region.text()
            self.get_logger().info(f'aimed P: allowed grasp region (diagnostic, derived in '
                                   f'{time.monotonic() - t0:.1f} s): {text}')
            try:
                self._region_pub.publish(String(data=text))
            except Exception:          # noqa: BLE001 - shutdown
                pass
        thread = threading.Thread(target=run, name='arm_aim_region', daemon=True)
        thread.start()
        self._aim_region_thread = thread       # only a started thread is ever joined

    def _aim_lookup(self, target_frame: str, source_frame: str, what: str):
        """(stamp_sec, 4x4 transform target<-source) of the latest TF, or STALE_TF.
        A static transform (stamp 0) counts as fresh."""
        if self._tf_node is None:
            raise AimRefused(aimed_mod.AIMED_UNAVAILABLE, 'no TF listener')
        if self._aim_cfg['max_tf_age_sec'] is None:
            raise AimRefused(aimed_mod.AIMED_UNAVAILABLE,
                             'no TF age bound: max_tf_age_sec (shared tf_freshness.yaml) '
                             'is not loaded (SR-16 E3)')
        from rclpy.time import Time
        try:
            t = self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except Exception as exc:       # noqa: BLE001 - every lookup failure is a refusal
            raise AimRefused(aimed_mod.STALE_TF,
                             f'{what}: no transform {source_frame} -> {target_frame} '
                             f'({type(exc).__name__}: {exc})') from exc
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        limit = float(self._aim_cfg['max_tf_age_sec'])
        age = 0.0 if stamp == 0.0 else now - stamp
        if age < -float(self._aim_cfg['max_tf_future_sec']):
            raise AimRefused(aimed_mod.STALE_TF,
                             f'{what}: transform {source_frame} -> {target_frame} is dated '
                             f'{-age:.2f} s in the future')
        if age > limit:
            raise AimRefused(aimed_mod.STALE_TF,
                             f'{what}: transform {source_frame} -> {target_frame} is '
                             f'{age:.2f} s old (max_tf_age_sec {limit:.2f})')
        self._aim_tf_age[what] = age               # logged (robot check of the intake)
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        m = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
             [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
             [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        tr = t.transform.translation
        return stamp, m, (tr.x, tr.y, tr.z)

    def _aim_base_pose(self, what: str):
        """(stamp, x, y, yaw) of the base frame in aimed.still_frame."""
        base, still = str(self._aim_cfg['base_frame']), str(self._aim_cfg['still_frame'])
        stamp, m, tr = self._aim_lookup(still, base, what)
        return stamp, tr[0], tr[1], math.atan2(m[1][0], m[0][0])

    def _aim_steer(self) -> Dict[str, Optional[float]]:
        timeout = float(self._p('safety_filter.steer_state_timeout_sec'))
        with self._state_lock:
            fresh = time.monotonic() - self._steer_rx <= timeout
            return dict(self._steer) if fresh else {}

    def _aim_start(self, request, result) -> '_AimContext':
        """The target in base_link (TF looked up ONCE, here), the base-still reference
        and the static-checked grasp candidates. Every refusal comes before any bus
        write. Fills result.target_in_base."""
        solver = self._aim_solver()
        margin = self._aim_margin(request)
        self._aim_tf_age = {}
        tgt = request.target
        frame = str(tgt.header.frame_id).strip()
        x, y, z = float(tgt.point.x), float(tgt.point.y), float(tgt.point.z)
        if not frame or not all(math.isfinite(v) for v in (x, y)):
            raise AimRefused(aimed_mod.INVALID_TARGET,
                             f'target needs a frame and finite x, y (frame "{frame}", '
                             f'x {x}, y {y})')
        base = str(self._aim_cfg['base_frame'])
        if frame not in list(self._aim_cfg['target_frames']):
            raise AimRefused(aimed_mod.INVALID_TARGET,
                             f'target frame "{frame}" is not one of '
                             f'{list(self._aim_cfg["target_frames"])} (aimed.target_frames)')
        tstamp = tgt.header.stamp.sec + tgt.header.stamp.nanosec * 1e-9
        if frame == base:
            pb = (x, y, z)
        else:
            _st, m, tr = self._aim_lookup(base, frame, 'target')
            pb = tuple(m[i][0] * x + m[i][1] * y + m[i][2] * z + tr[i] for i in range(3))
        result.target_in_base.header.frame_id = base
        result.target_in_base.header.stamp = self.get_clock().now().to_msg()
        result.target_in_base.point.x, result.target_in_base.point.y = pb[0], pb[1]
        result.target_in_base.point.z = pb[2]
        if bool(getattr(request, 'check_target_shift', False)):
            self._aim_shift(request, pb, base)
        still = self._aim_base_pose('base-still reference')
        steer = self._aim_steer()
        missing = [n for n in self._steer_names if steer.get(n) is None]
        if missing:
            text = (f'pick_plastic AIMED: no steering reading for {missing} at the start - '
                    'a wheel turning during the pick cannot be seen')
            if self._wheel_mode == 'measured':
                self.get_logger().error(text + ' - REFUSED (wheel_envelope: measured)')
                self._report(text)
                raise AimRefused(aimed_mod.STEERING_UNKNOWN, text)
            self.get_logger().warn(text)
            self._report(text)
        t0 = time.monotonic()
        res = solver.solve(pb[0], pb[1], self._envelope(),
                           limit=int(self._aim_cfg['max_candidates']), margin=margin,
                           margin_step=float(self._aim_cfg['region_step_m']))
        ms = (time.monotonic() - t0) * 1e3
        ages = ', '.join(f'{k} {v * 1e3:.0f} ms' for k, v in self._aim_tf_age.items())
        text = (f'pick_plastic AIMED: target ({x:+.3f}, {y:+.3f}) in {frame} (stamp '
                f'{tstamp:.3f}, age not limited) -> ({pb[0]:+.3f}, {pb[1]:+.3f}) in {base}; '
                f'TF age {ages}; grasp height {solver.z:+.4f} m; ')
        if not res.ok:
            text += f'REFUSED {res.reason} ({ms:.0f} ms): {res.detail}'
            self.get_logger().error(text)
            self._report(text)
            raise AimRefused(res.reason, res.detail)
        text += (f'{len(res.candidates)} candidate(s) pass the static check ({ms:.0f} ms): '
                 + ', '.join(f'{c.branch} roll {math.degrees(c.roll):+.0f} deg'
                             for c in res.candidates))
        self.get_logger().info(text)
        self._report(text)
        return _AimContext(pb, frame, res.candidates, still, steer)

    def _aim_margin(self, request) -> float:
        """The inward region margin for this aimed goal (SR-16 E4, audit N7). Set: it
        applies to every aimed goal. Unset (TO-VERIFY until measured, E12 (d)): E4's
        "none while only confirm_by_rearm runs" - 0.0 for that mode's preview and its
        confirmed pick (check_target_shift, sent only by it) and for the operator's P
        (_pick_trigger); any other aimed goal is mode automatic's and is refused
        before anything else. The arm cannot see the gateway's mode; these are the
        observable facts a goal carries."""
        margin = self._aim_cfg['region_margin_m']
        if margin is not None:
            return float(margin)
        interim = (bool(getattr(request, 'preview', False))
                   or bool(getattr(request, 'check_target_shift', False)))
        automatic, why = self._pick_trigger(request)
        if interim or not automatic:
            return 0.0
        raise AimRefused(aimed_mod.AIMED_UNAVAILABLE,
                         f'no inward region margin: aimed.region_margin_m is not set (SR-16 '
                         f'E4; TO-VERIFY until measured, E12 (d)) - only the confirm_by_rearm '
                         f'preview/confirmed pick or the operator\'s P run without it; this '
                         f'goal is neither ({why})')

    def _aim_shift(self, request, pb, base: str):
        """SR-16 E11 item 4 (one shift bound, in the quantity the arm acts on), E3 (pb IS
        the one pick-start transform), audit F5: the gateway's own check sees only its
        map -> base_footprint pose; a map -> odom correction between the preview and
        here shows only in this transform. Before any bus write; fails closed."""
        prev = request.preview_target_in_base
        bound = float(request.max_target_shift_m)
        px, py = float(prev.x), float(prev.y)
        if not (math.isfinite(bound) and bound > 0.0 and math.isfinite(px)
                and math.isfinite(py)):
            raise AimRefused(aimed_mod.INVALID_TARGET,
                             f'target shift check without a valid bound or preview point '
                             f'(max_target_shift_m {bound}, preview ({px}, {py}))')
        shift = math.hypot(pb[0] - px, pb[1] - py)
        text = (f'pick_plastic AIMED: target shift from the preview ({px:+.3f}, {py:+.3f}) '
                f'to ({pb[0]:+.3f}, {pb[1]:+.3f}) in {base}: {shift * 1e3:.0f} mm, bound '
                f'{bound * 1e3:.0f} mm')
        if shift > bound:
            self.get_logger().error(text + ' - REFUSED')
            self._report(text + ' - REFUSED')
            raise AimRefused(aimed_mod.TARGET_SHIFTED, text)
        self.get_logger().info(text + ' - within')
        self._report(text + ' - within')

    def _aim_approach(self, aim: '_AimContext', g_open: Dict[int, int], automatic: bool):
        """The approach to the PRE-GRASP pose of the first candidate with a path (direct
        only when automatic, SR-17 rule 11) and its vertical descent to the grasp
        (user decision 2026-11-16), both checked: an _AimPlan or NO_PATH."""
        tried = []
        for k, c in enumerate(aim.candidates):
            tag = f'{c.branch} roll {math.degrees(c.roll):+.0f} deg'
            pre = {**self._ticks_of(c.pregrasp), GRIPPER_ID: self._grip_pos[GRIPPER_ID]}
            grip = {**self._ticks_of(c.q), GRIPPER_ID: self._grip_pos[GRIPPER_ID]}
            try:
                approach, _segs, at_pre = self._planned_segments(
                    g_open, pre, f'pick_plastic/approach (aimed, candidate {k + 1}: {tag}, '
                    'to the pre-grasp pose)', 4000, automatic=automatic, soft_start=True)
                _segs = self._aim_vertical(at_pre, c.descent[1:],
                                           f'pick_plastic/descent (aimed, candidate {k + 1})')
            except MotionRefused as exc:
                if 'NO_PATH' not in str(exc):
                    raise
                tried.append(tag)
                continue
            at_grip = {**at_pre, **{s: grip[s] for s in LIMB_IDS}}
            text = (f'pick_plastic AIMED: grasp pose {tag} (wrist roll '
                    f'{math.degrees(c.roll):+.1f} deg, the same at the pre-grasp pose and '
                    f'through the descent), pre-grasp ticks {[pre[s] for s in LIMB_IDS]}, '
                    f'grasp ticks {[grip[s] for s in LIMB_IDS]}, vertical descent '
                    f'{len(c.descent) - 1} segments')
            self.get_logger().info(text)
            self._report(text)
            return _AimPlan(c, pre, approach, at_pre, grip, at_grip)
        raise AimRefused(aimed_mod.NO_PATH, f'no approach path for any candidate ({tried}); '
                         + ('automatic: direct path only (SR-17 rule 11)' if automatic
                            and self._automatic_paths == 'direct_only' else 'detours allowed'))

    def _aim_vertical(self, here: Dict[int, int], waypoints, source: str, envelope=None):
        """The vertical descent (or lift) of an aimed P from the arm pose `here`
        through `waypoints` (aimed_grasp descent poses, radians; the gripper stays as
        in `here`): every segment checked as the planner checks a path segment - the
        written ramp setpoints (soft-started first segment), the current envelope, the
        tick wrap - and timed at planner.max_joint_velocity_rad_s (never faster than a
        detour of the approach). No re-plan: a re-planned path would not be vertical.
        Segments, or MotionRefused('NO_PATH ...') - before any write."""
        start = self._q_of(here)
        arm = self._cal.joints[:5]
        path = [start] + [{**start, **{n: w[n] for n in arm}} for w in waypoints]
        env = self._envelope() if envelope is None else envelope
        check = self._planner(0.0, soft_start=True)
        for i, (a, b) in enumerate(zip(path, path[1:])):
            why = check.check_segment(a, b, env, direct=False, first=i == 0)
            if why:
                text = f'{source}: segment {i + 1}/{len(path) - 1} refused: {why}'
                self.get_logger().error(text)
                self._report(text)
                raise MotionRefused(f'NO_PATH {text}')
        soft = len(path) > 1 and check.starts_soft(path[0], path[1], False)
        segs = self._path_segments(path, 'vertical', None, here,
                                   **({'soft': True} if soft else {}))
        text = (f'{source}: {len(segs)} vertical segments checked, '
                f'{sum(sg[2] for sg in segs):.1f} s')
        self.get_logger().info(text)
        self._report(text)
        return segs

    def _aim_return(self, plan: '_AimPlan', automatic: bool, source: str):
        """After the close: the vertical lift back to the pre-grasp height (the descent
        reversed, gripper closed) when it passes the checks, then the planned return
        home from there; else - logged - the return home straight from the grasp, as
        before the steep approach. (lift waypoints or None, return PlanResult, the
        return's start ticks, home end ticks)."""
        at_grip = {**plan.at_grip, GRIPPER_ID: self._grip_close}
        up = plan.cand.descent[-2::-1]
        lift, start = None, at_grip
        try:
            self._aim_vertical(at_grip, up, f'{source}/lift')
            lift = up
            start = {**at_grip, **{s: plan.pre[s] for s in LIMB_IDS}}
        except MotionRefused as exc:
            text = (f'{source}: no vertical lift after the close ({exc}) - the return home '
                    'starts at the grasp pose')
            self.get_logger().warn(text)
            self._report(text)
        back, _segs, at_home = self._planned_segments(
            start, self._home_pos, f'{source}/return_home', HOME_MOVE_MS,
            automatic=automatic, soft_start=True)
        return lift, back, start, at_home

    def _aim_preview(self, goal_handle):
        """An aimed P's refusal checks without the P: nothing claimed, no torque, no bus
        write (reads only). Automatic or not as the real P would be (_pick_trigger)."""
        req, result = goal_handle.request, PickPlastic.Result()
        ok = False
        try:
            if not bool(getattr(req, 'aimed', False)):
                raise AimRefused(aimed_mod.INVALID_TARGET, 'a preview needs aimed: true')
            if self._pick_sequence is None:
                raise AimRefused(aimed_mod.AIMED_UNAVAILABLE, self._poses_error)
            with self._state_lock:
                busy = self._motion_owner or (self._leader_state != LEADER_OFF and 'leader')
            if busy:
                raise MotionRefused(f'preview: arm motion "{busy}" is active')
            automatic, trigger = self._pick_trigger(req)
            aim = self._aim_start(req, result)
            here = self._read_all()
            if here is None:
                raise MotionRefused(f'preview: no measured arm pose ({self._read_failure()})')
            why = self._outside(self._q_of(here), 'pick_plastic preview', 'P is')
            if why:
                raise MotionRefused(why)
            g_open = {**here, GRIPPER_ID: self._grip_open}
            plan = self._aim_approach(aim, g_open, automatic)
            lift, _back, _start, _home = self._aim_return(plan, automatic,
                                                          'pick_plastic preview')
            result.success, result.reason = True, 'PREVIEW_OK'
            result.message = (f'preview passed ({trigger}): target ({aim.target_base[0]:+.3f}, '
                              f'{aim.target_base[1]:+.3f}) in base_link, approach to the '
                              'pre-grasp pose, vertical descent, '
                              + ('vertical lift, ' if lift is not None else '')
                              + 'return home planned; nothing moved')
            self._report(f'pick_plastic PREVIEW: {result.message}')
            ok = True
        except MotionRefused as exc:
            result.success = False
            result.reason = getattr(exc, 'reason', None) or (
                aimed_mod.NO_PATH if 'NO_PATH' in str(exc) else 'REFUSED')
            result.message = f'Preview refused: {exc}'
            self.get_logger().warn(f'pick_plastic PREVIEW refused: {exc}')
        except Exception as exc:     # noqa: BLE001 - a preview never moves; report it
            result.success, result.reason = False, 'ERROR'
            result.message = f'Preview error: {exc!r}'
            self.get_logger().error(f'pick_plastic PREVIEW: {traceback.format_exc()}')
        finally:
            with self._state_lock:
                self._preview_busy = False
        try:                         # at shutdown the goal may already be gone (AP1 L6)
            goal_handle.succeed() if ok else goal_handle.abort()
        except Exception as exc:     # noqa: BLE001
            self.get_logger().warn(f'pick_plastic PREVIEW: result not delivered: {exc!r}')
        return result

    def _aim_still(self, aim: '_AimContext', when: str):
        """The base has not moved since _aim_start (_aim_motion). Fail-closed: a pose
        that cannot be read refuses."""
        bad, text = self._aim_motion(aim, when)
        if bad:
            self.get_logger().error(text + ' - BASE MOVING')
            self._report(text + ' - BASE MOVING')
            raise AimRefused(aimed_mod.BASE_MOVING, text)
        self.get_logger().info(text)
        self._report(text)

    def _aim_watch(self, aim: '_AimContext'):
        """SR-16 E5, "drive motion DURING an aimed P aborts it" (audit F3): a callable,
        polled on every ramp setpoint and pause of the P, that raises BaseMovedAbort.
        WHAT IS OBSERVABLE HERE: the arm node does not see the drive command (no
        cmd_vel; "commanded to zero" is the gateway's/mux's business), so "drive
        motion" is MEASURED motion - the same source and thresholds as _aim_still:
        base pose in aimed.still_frame (wheel odometry) vs the start reference, and
        the steering readings vs their start values. Fail-closed: a pose that can no
        longer be read fresh (STALE_TF) aborts too. Quiet unless it fires."""
        def watch():
            try:
                bad, text = self._aim_motion(aim, 'during the pick')
            except AimRefused as exc:
                bad, text = True, f'base rest no longer shown during the pick: {exc}'
            if bad:
                self.get_logger().error(text + ' - BASE MOVING, aborting the P')
                self._report(text + ' - BASE MOVING, aborting the P')
                raise BaseMovedAbort(text)
            return False
        return watch

    def _aim_motion(self, aim: '_AimContext', when: str):
        """(moved, text): base pose in aimed.still_frame vs the start reference within
        aimed.still_max_translation_m / still_max_rotation_rad, its TF stamp advanced
        (a frozen transform reads as still), and no known steering reading changed by
        more than safety_filter.steer_motion_margin_rad (the margin the grasp was
        checked with). A wheel read at the start that has no fresh reading now
        (/hw/steer_states stale, or that channel 0.0 = dropped) counts as MOVED -
        the safety filter's rule for an unknown wheel, swept over its stops (audit
        2026-11-13 N1: dropping it from the comparison switched the watch off).
        Raises AimRefused(STALE_TF) when the pose cannot be read."""
        ref = aim.still_ref
        now = self._aim_base_pose(f'base-still check {when}')
        adv = float(self._aim_cfg['still_min_stamp_advance_sec'])
        if not now[0] > ref[0] + adv:
            raise AimRefused(aimed_mod.STALE_TF,
                             f'base-still check {when}: the {self._aim_cfg["still_frame"]} '
                             f'transform did not advance since the start ({ref[0]:.3f} -> '
                             f'{now[0]:.3f} s): a frozen pose cannot show the base at rest')
        dxy = math.hypot(now[1] - ref[1], now[2] - ref[2])
        dyaw = abs(wrap(now[3] - ref[3]))
        steer = self._aim_steer()
        margin = float(self._p('safety_filter.steer_motion_margin_rad'))
        dsteer = {n: abs(steer[n] - v) for n, v in aim.steer_ref.items()
                  if v is not None and steer.get(n) is not None}
        lost = [n for n, v in aim.steer_ref.items()
                if v is not None and steer.get(n) is None]
        age = self._aim_tf_age.get(f'base-still check {when}', 0.0)
        text = (f'pick_plastic AIMED: base-still check {when}: TF age {age * 1e3:.0f} ms, '
                f'moved {dxy * 1e3:.1f} mm, {math.degrees(dyaw):.2f} deg; steering change '
                + (', '.join(f'{n} {math.degrees(d):.1f} deg' for n, d in dsteer.items())
                   or 'not measured')
                + (f'; steering reading LOST for {lost} (stale or 0.0) - counted as moved'
                   if lost else ''))
        bad = (dxy > float(self._aim_cfg['still_max_translation_m'])
               or dyaw > float(self._aim_cfg['still_max_rotation_rad'])
               or any(d > margin for d in dsteer.values())
               or bool(lost))
        return bad, text

    # ================================================================ joint states
    def _publish_joint_states(self):
        if not rclpy.ok():
            return
        try:
            positions, ticks = [], {}
            for sid in IDS:
                with self._serial_lock:
                    val = self._read_pos(sid)
                if val is None:
                    self.get_logger().warn(
                        f'joint_states: servo {sid} not responding',
                        throttle_duration_sec=10.0,
                    )
                    self._publish_arm_state(None, self.get_clock().now().to_msg())
                    return
                ticks[sid] = val
                positions.append(math.radians(val / 4096 * 360))
            self._last_seen = dict(ticks)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = positions
            self._joint_pub.publish(msg)
            self._publish_arm_state(ticks, msg.header.stamp)
        except Exception as exc:
            self.get_logger().warn(f'joint_states: {exc}', throttle_duration_sec=10.0)

    def _publish_arm_state(self, ticks: Optional[Dict[int, int]], stamp):
        """/arm/state (gripperx_arm_msgs/ArmState) - what the gateway's dispatch gate
        reads (ARM_NOT_HOME). `ticks` None: a servo did not answer - published as
        known false (AP3 F2), never silence."""
        with self._state_lock:
            owner = self._motion_owner or ''
            idle = (self._motion_owner is None and self._leader_state == LEADER_OFF
                    and self._pending_home is None and self._leaving == 0)
        home = self._home_pos

        def near(s):
            return min((ticks[s] - home[s]) % TICK_RANGE,
                       (home[s] - ticks[s]) % TICK_RANGE) <= POS_TOLERANCE
        known = ticks is not None and home is not None
        gripper = known and near(GRIPPER_ID)
        limbs = known and all(near(s) for s in LIMB_IDS)
        st = ArmState(known=known, idle=idle,
                      at_home=limbs and (gripper or not self._home_with_gripper),
                      gripper_at_home=gripper, torque_on=bool(self._torque_on), owner=owner)
        st.header.stamp = stamp
        try:
            self._state_pub.publish(st)
        except Exception:            # noqa: BLE001 - shutdown
            pass

    # ================================================================ leader mode (FR-17)
    def _setup_leader(self):
        self._enable_last = False            # assumed False at start: only a fresh True counts
        self._withdraw_count = 0             # every enable False
        # None, or (stop count, withdrawal count) at the J rising edge that requested it
        self._activation_requested = None
        self._leader_active_rx = -math.inf
        self._leader_rx = -math.inf
        self._leader_q: Dict[str, float] = {}
        self._leader_cal_verified: Optional[bool] = None
        self._gateway_armed: Optional[bool] = None
        self._cmd_q: Optional[Dict[str, float]] = None
        self._held = False                   # a setpoint was rejected: no writes
        self._follow_steps: list = []        # planned setpoints toward the leader (ticks)
        # Per leader-mode session: the command edge per leader joint, only moving inward
        # (_leader_target). None = not yet set.
        self._leader_edges: Optional[Dict[str, Tuple[float, float]]] = None
        self._follow_job: Optional[_ReplanJob] = None     # the search they came from
        self._follow_total = 0
        self._last_replan = -math.inf
        self._replan: Optional[_ReplanJob] = None         # search running in the background
        self._replan_failed: Optional[_ReplanJob] = None  # last search that found no path
        # (job, measured ticks, rising, gate): a found path discarded because its first
        # step is refused from the measured pose, or by the closed lag-growth gate (`gate`)
        # - not a failed search; re-checked (or, for a lift, searched again) once the arm
        # moved or the search's inputs change (_replan_step).
        self._replan_discarded: Optional[Tuple[_ReplanJob, Dict[int, int], bool, bool]] = None
        # Per planned step: the entries still rising out of the ground margin at the
        # step's start (() = a normal step, checked from the measured pose).
        self._follow_rising: list = []
        self._held_since = 0.0               # when the current hold began (monotonic)
        self._hold_reason = ''               # why it holds, for the status line
        self._hold_binding = ''              # the last REJECTED verdict, for the status line
        self._hold_status_t = 0.0            # last hold status line
        # per leader joint: (measured ticks where it last tracked, reference |servo goal -
        # measured|, servo goal travel is measured from); None until the first leader tick
        # after activation (_lag_growth)
        self._lag_base: Optional[Tuple[Dict[str, int], Dict[str, int], Dict[str, int]]] = None
        self._lag_gated = False              # the growth gate is closed
        self._lag_gate_now = None            # its last closed state (_lag_gate)
        self._lag_gate_t = -math.inf         # its last message (monotonic)
        self._hold_detail = ''               # the hold reason behind the closed gate's text
        # The adopted path came through the closed gate (_lag_shrink_why): per leader servo
        # (measured tick at adoption, bound tick, floor or None for a gated joint) - every
        # goal change of the path is bounded by it (_follow_step); None: not gated.
        self._lag_shrink: Optional[Dict[int, Tuple[int, int, Optional[int]]]] = None
        g = self._leader_group
        self.create_subscription(Bool, '/arm/leader/enable',
                                 lambda msg, info: self._on_leader_enable(msg, info),
                                 ENABLE_QOS, callback_group=g)
        self.create_subscription(Bool, '/arm/leader/active', self._on_leader_active, 10,
                                 callback_group=g)
        self.create_subscription(JointState, '/arm/leader/joint_states', self._on_leader_joints,
                                 10, callback_group=g)
        self.create_subscription(Bool, self._p('leader.calibration_verified_topic'),
                                 self._on_leader_cal, LATCHED, callback_group=g)
        try:
            from gripperx_external_msgs.msg import ArmingState
            self.create_subscription(ArmingState, self._p('leader.arming_state_topic'),
                                     self._on_arming, LATCHED, callback_group=g)
        except ImportError:
            self.get_logger().warn('gripperx_external_msgs not available: gateway arming '
                                   'state unknown - leader mode will be refused')
        self._dbg: Optional[dict] = None          # notes of the running tick (follow_debug)
        self._dbg_prev_t0: Optional[float] = None
        self._follow_debug_pub = None
        if bool(self._p('leader.follow_debug_enabled')):
            self._follow_debug_pub = self.create_publisher(
                LeaderFollowDebug, '/arm/leader/follow_debug', DEBUG_QOS)
        # The tick has its own group: a slow tick (filter checks) must not starve the
        # stream and heartbeat callbacks, or the watchdog would trip on its own load.
        self.create_timer(1.0 / float(self._p('leader.rate_hz')), self._leader_tick,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self._publish_leader_state('off (start)')

    def _publish_leader_state(self, text: str):
        # Under the state lock, and an 'active ...' text only while still active: a
        # tick finishing late must not overwrite the 'off (...)' of a concurrent leave.
        with self._state_lock:
            if text.startswith('active') and self._leader_state != LEADER_ACTIVE:
                return
            self.leader_text = text            # last state text, also for tests
            self._leader_state_pub.publish(String(data=text))

    def _on_leader_enable(self, msg: Bool, info=None):
        # Under the state lock: an activation running in the tick thread (tens of ms)
        # reads these in its final locked block and refuses if enable was withdrawn
        # meanwhile - here leader mode is still OFF, so nothing else would end it.
        rx = self._rx(info, '/arm/leader/enable')
        overtaken = False
        with self._state_lock:
            if not msg.data:
                self._enable_last = False
                self._withdraw_count += 1
                self._activation_requested = None
                leave = self._leader_state != LEADER_OFF
            else:
                if self._enable_last:
                    return                   # no rising edge
                self._enable_last = True
                # A Space received after this edge but handled before it (the enable
                # waited for the leader group) wins, as for O/I/P (_claim_after_leader).
                overtaken = self._stop_rx >= rx
                if not overtaken:
                    # Snapshot AT the edge (audit r6 of 5acf87e): a Space between this
                    # edge and the tick that takes the request must refuse it too.
                    self._activation_requested = (self._stop_count, self._withdraw_count)
        if overtaken:
            text = 'leader mode NOT activated: Space received after the enable (J)'
            self.get_logger().warn(text)
            self._report(text)
            self._publish_leader_state('off (Space after the enable)')
            return
        if msg.data:
            return
        if leave:
            self._leave('enable withdrawn (J, or O/I/P first)')

    def _on_leader_active(self, msg: Bool):
        if msg.data:
            self._leader_active_rx = time.monotonic()

    def _on_leader_joints(self, msg: JointState):
        if self._cal is None:
            return
        q = {}
        names = [JOINT_NAMES[IDS.index(sid)] for sid in LEADER_IDS]
        for name, pos in zip(msg.name, msg.position):
            if name in names:
                q[self._cal.joints[JOINT_NAMES.index(name)]] = float(pos)
        self._leader_q = q
        self._leader_rx = time.monotonic()

    def _on_leader_cal(self, msg: Bool):
        self._leader_cal_verified = bool(msg.data)

    def _on_arming(self, msg):
        self._gateway_armed = bool(msg.armed)
        if self._gateway_armed and self._leader_state != LEADER_OFF:
            self._leave('external-goal gateway ARMED (FR-17 item 8)')

    def _leader_joints(self, cal=None) -> List[str]:
        """The joint names leader mode drives (LEADER_IDS), in servo id order."""
        cal = cal or self._cal
        return [cal.joints[IDS.index(sid)] for sid in LEADER_IDS]

    def _leader_fresh(self, now: float) -> Optional[str]:
        timeout = float(self._p('leader.watchdog_timeout_sec'))
        if now - self._leader_active_rx > timeout:
            return f'no /arm/leader/active heartbeat for > {timeout:.2f} s'
        if now - self._leader_rx > timeout:
            return f'no /arm/leader/joint_states for > {timeout:.2f} s'
        need = [n for n in self._leader_joints()
                if n != self._grip_name() or self._grip_required()]
        missing = [n for n in need if n not in self._leader_q]
        if missing:
            return f'leader stream lacks {missing}'
        return None

    def _refusal(self, now: float) -> Optional[str]:
        if not self._config_loaded:
            return 'arm_safety_filter.yaml not loaded (safety_filter.config_loaded false)'
        if self._filter is None:
            return self._filter_error
        if not self._cal_verified and not self._allow_unverified:
            return 'follower calibration unverified (allow_unverified_calibration=false)'
        if self._leader_cal_verified is not True and not self._allow_unverified:
            return ('leader calibration not verified or unknown '
                    '(allow_unverified_calibration=false)')
        if self._gateway_armed is None:
            return 'external-goal gateway arming state UNKNOWN (no arming_state received)'
        if self._gateway_armed:
            return 'external-goal gateway is ARMED'
        # /arm/leader/enable is NOT counted: it has two legitimate publishers by design
        # (keyboard teleop and leader_arm_node, which withdraws the request on errors).
        for topic in ('/arm/leader/joint_states', '/arm/leader/active'):
            n = self.count_publishers(topic)
            if n != 1:
                return f'{n} publishers on {topic} (exactly 1 required)'
        try:
            self._leader_band()
        except ValueError as exc:
            return f'leader command band (leader.limit_command_margin_ticks): {exc}'
        try:
            check_edge_slowdown(float(self._p('leader.edge_slowdown_zone_rad')),
                                float(self._p('leader.edge_slowdown_velocity_rad_s')),
                                2.0 * math.pi / self._cal.ticks_per_rev
                                * float(self._p('leader.rate_hz')))
        except ValueError as exc:
            return f'leader edge slowdown: {exc}'
        return self._leader_fresh(now)

    def _leader_band(self) -> Dict[str, Tuple[float, float]]:
        """Per leader joint (LEADER_IDS, the gripper included since 2026-11-13) the band
        leader mode may command: the joint limits narrowed by leader.limit_command_margin_ticks whole ticks (user decision 2026-10-15,
        gripperx_arm.leader_band). Raises ValueError. The joint limits themselves and
        the measured-pose check (SR-17 rule 8) are unchanged."""
        cal, kin = self._cal, self._filter.kin
        m = self._p('leader.limit_command_margin_ticks')
        return {n: command_band(*kin.limits[n], cal.zero_ticks[i], cal.signs[i],
                                cal.ticks_per_rev, m)
                for i, n in ((IDS.index(sid), cal.joints[IDS.index(sid)])
                             for sid in LEADER_IDS)}

    def _leader_target(self, ref: Dict[str, float]) -> Tuple[Dict[str, float], Dict]:
        """(target, far): the leader pose clamped into _leader_band - what leader mode
        steers toward (direct steps and re-plans alike) - and the joints whose RAW leader
        value differs from `ref` (the commanded / held pose) by more than pi.

        A joint activated inside the margin (the CAD home has limb3 8 ticks above its
        lower limit) keeps its activation pose as its edge: never commanded further out,
        never pulled in by itself - activation does not move the arm. The edges are
        per leader-mode SESSION (_leader_edges, set in _enter_leader) and only ever move
        INWARD, as `ref` moves inward, up to the band (audit LW1 L1): a re-plan whose
        `ref` is a measured pose sagged outward (a goal unknown to _held_start)
        must not re-anchor the edge there - repeated, that ratcheted the
        edge tick by tick to the limit.

        `far` is decided on the raw value BEFORE the clamp: a value that crossed its
        angle window edge (2 pi off, a wrong range_center_rad) must stay a rejection and
        never become a clamped, drivable target at the band edge."""
        band, q = self._leader_band(), self._leader_q
        g = self._grip_name()
        # A gripper not driven this session (FR-17 amendment 2026-11-16) keeps `ref`, its
        # held goal, and is not judged by the leader's value.
        far = {n: round(q[n] - ref[n], 3) for n in band
               if (n != g or self._grip_driven) and abs(q[n] - ref[n]) > math.pi}
        edges = self._leader_edges or {n: (min(lo, ref[n]), max(hi, ref[n]))
                                       for n, (lo, hi) in band.items()}
        edges = {n: (max(edges[n][0], min(lo, ref[n])), min(edges[n][1], max(hi, ref[n])))
                 for n, (lo, hi) in band.items()}
        self._leader_edges = edges
        out = {n: min(max(q[n], e_lo), e_hi) for n, (e_lo, e_hi) in edges.items()
               if n != g or self._grip_driven}
        if not self._grip_driven:
            out[g] = ref[g]
        return out, far

    # -------------------------------------------------- leader gripper torque limit
    # FR-17 amendment, user decisions 2026-11-16. A block of the gripper while CLOSING
    # (jaws stopped by an object) does not end leader mode (_grip_squeezing). Its force:
    # (a) leader.gripper_torque_limit unset (the shipped config) - the servo's own
    # Torque_Limit, like limb1..limb5: this node writes no Torque_Limit for any servo.
    # (b) set (a measured value, TO-VERIFY) - that value in servo_bus.ADDR_TORQUE_LIMIT
    # (RAM; the EEPROM Max_Torque_Limit is never written) while leader mode drives the
    # gripper (_try_activate); the previous value is restored on every exit (_leave,
    # Space, the tick's error handler, _shut_down_motion, destroy_node, a refused
    # activation; the leader tick retries a failed restore) and before every claimed
    # motion (_prepare): O, P and I never run with the leader value. Not read, or not read
    # back as written: the gripper is NOT driven this session (held where it is, as
    # before 2026-11-13), the limbs follow.

    def _grip_name(self) -> str:
        return self._cal.joints[IDS.index(GRIPPER_ID)]

    def _grip_close_sign(self) -> Optional[int]:
        """+1/-1: the tick direction in which the gripper closes (arm_poses.yaml
        gripper.open/close); None if unknown."""
        o, c = self._grip_open, self._grip_close
        if o is None or c is None:
            return None
        d = tick_deviation(int(c), int(o))
        return None if d == 0 else (1 if d > 0 else -1)

    def _grip_squeezing(self, goal: int, measured: int) -> bool:
        """In a session driving the gripper: its goal (ticks) lies on the CLOSING side of
        its measured position - jaws stopped short of their goal, e.g. by an object."""
        sign = self._grip_close_sign()
        return (self._grip_driven and sign is not None
                and tick_deviation(int(goal), int(measured)) * sign > 0)

    def _grip_squeezing_q(self, cmd: float, measured: float) -> bool:
        """_grip_squeezing for joint angles (both in the joint's window: plain difference)."""
        sign = self._grip_close_sign()
        if not self._grip_driven or sign is None:
            return False
        return (cmd - measured) * sign * self._cal.signs[IDS.index(GRIPPER_ID)] > 0

    def _drive_ids(self) -> List[int]:
        """The servos the leader session writes: all six, or the limbs only while the
        gripper is held (not driven)."""
        return LEADER_IDS if self._grip_driven else LIMB_IDS

    def _grip_required(self) -> bool:
        """Does the leader stream have to carry the gripper? While active: if this session
        drives it; at activation: always (it is to be driven)."""
        if self._leader_state == LEADER_ACTIVE:
            return self._grip_driven
        return True

    def _bus_error(self) -> str:
        return str(getattr(self._bus, 'last_error', None) or 'no reply')

    def _apply_grip_limit(self) -> Optional[str]:
        """None: the gripper is driven this session - at the servo's own Torque_Limit
        (parameter unset, nothing written) or with leader.gripper_torque_limit (never above
        the servo's present value) written to servo 6 and read back; otherwise why it is
        not (it is held), any partial write undone."""
        if self._grip_close_sign() is None:
            return 'gripper.open/close unknown: the closing side cannot be told'
        cfg = self._grip_limit
        if cfg is None:
            return None
        why = None
        with self._serial_lock:
            # Not restored from an earlier session: the leader value is what the servo
            # holds now - the one to restore is the one saved then.
            prev = self._grip_limit_prev
            try:
                if prev is None:
                    prev = self._bus.read_word(GRIPPER_ID, ADDR_TORQUE_LIMIT)
                    if prev is None or not 0 < prev <= TORQUE_LIMIT_MAX:
                        return (f'Torque_Limit (register {ADDR_TORQUE_LIMIT}) of servo '
                                f'{GRIPPER_ID} not read ({prev!r}, {self._bus_error()})')
                value = min(cfg, prev)
                self._grip_limit_prev = prev        # before the write: it may land and raise
                ok = self._bus.write_word(GRIPPER_ID, ADDR_TORQUE_LIMIT, value)
                back = self._bus.read_word(GRIPPER_ID, ADDR_TORQUE_LIMIT) if ok else None
                if not ok or back != value:
                    why = (f'Torque_Limit {value} on servo {GRIPPER_ID}: '
                           + ('write not acknowledged' if not ok else f'read back {back!r}'))
            except Exception as exc:     # noqa: BLE001 - e.g. the bus lost
                why = f'Torque_Limit on servo {GRIPPER_ID}: {exc!r}'
            if why is not None:
                undo = self._restore_grip_limit_locked('leader gripper limit not applied')
                return why + (f'; {undo}' if undo else '')
        text = (f'leader gripper: Torque_Limit {value} written to servo {GRIPPER_ID} and read '
                f'back (previous {prev}, restored when leader mode ends)')
        if value != cfg:
            text += (f' - leader.gripper_torque_limit {cfg} capped at the previous value '
                     '(never raised)')
        self.get_logger().info(text)
        self._report(text)
        return None

    def _restore_grip_limit(self, reason: str, lock_timeout: Optional[float] = None,
                            idle_only: bool = False) -> Optional[str]:
        """_restore_grip_limit_locked under the serial lock. `lock_timeout` (Space): not
        free in time -> not restored now (retried). `idle_only` (the tick's retry): nothing
        while a motion owns the arm - its _prepare restores it itself."""
        if self._grip_limit_prev is None:
            return None
        if lock_timeout is None:
            self._serial_lock.acquire()
        elif not self._serial_lock.acquire(timeout=lock_timeout):
            return self._grip_restore_failed(reason, 'arm bus busy (serial lock not free)')
        try:
            if idle_only and (self._motion_owner is not None
                              or self._leader_state != LEADER_OFF):
                return None
            return self._restore_grip_limit_locked(reason)
        finally:
            self._serial_lock.release()

    def _restore_grip_limit_locked(self, reason: str) -> Optional[str]:
        """Caller holds the serial lock. Never raises. The Torque_Limit servo 6 had before
        leader mode, written and read back: None when done (or nothing to restore), else
        why not - the leader value may still be in the servo (_grip_limit_prev stays).
        Torque on: the gripper's goal := its present position FIRST - a goal left inside
        an object would squeeze with the full limit the moment it is raised; not readable
        or not writable (e.g. node giving up, _dead): the limit is NOT raised."""
        prev = self._grip_limit_prev
        if prev is None:
            return None
        why = None
        try:
            if self._torque_on:
                present = self._bus.read_pos(GRIPPER_ID)
                if present is None:
                    why = (f'gripper position not readable ({self._bus_error()}): its goal '
                           'cannot be re-anchored first')
                else:
                    self._write('hold', {GRIPPER_ID: int(present)})
            if why is None:
                ok = self._bus.write_word(GRIPPER_ID, ADDR_TORQUE_LIMIT, prev)
                back = self._bus.read_word(GRIPPER_ID, ADDR_TORQUE_LIMIT) if ok else None
                if not ok or back != prev:
                    why = 'write not acknowledged' if not ok else f'read back {back!r}'
        except Exception as exc:     # noqa: BLE001 - it runs on every exit path
            why = repr(exc)
        if why is not None:
            return self._grip_restore_failed(reason, why)
        self._grip_limit_prev = None
        self._grip_restore_why = None
        text = (f'leader gripper: Torque_Limit {prev} restored on servo {GRIPPER_ID} '
                f'({reason})')
        self.get_logger().info(text)
        self._report(text)
        return None

    def _grip_restore_failed(self, reason: str, why: str) -> str:
        text = (f'leader gripper: previous Torque_Limit {self._grip_limit_prev} NOT restored '
                f'on servo {GRIPPER_ID} ({reason}): {why} - the leader limit may still be in '
                'the servo; O, P and I are refused until it is restored (retried)')
        if getattr(self, '_grip_restore_why', None) != why:     # a retry repeats it quietly
            self._grip_restore_why = why
            self.get_logger().error(text)
            self._report(text)
        return text

    def _grip_limit_at_start(self):
        """Startup, only with leader.gripper_torque_limit set: a process that ended inside leader mode without its exit
        path (killed, crashed) leaves the leader Torque_Limit in servo 6 - the RAM value
        survives a respawn. Torque_Limit below Max_Torque_Limit (EEPROM) is taken as such
        a leftover: the EEPROM value is restored before the first claimed motion
        (_prepare), or by the leader tick. TO-VERIFY: that the servo loads Torque_Limit
        from Max_Torque_Limit at power-on."""
        if self._grip_limit is None:
            return                   # no limit is ever written: nothing can be left over
        try:
            with self._serial_lock:
                ram = self._bus.read_word(GRIPPER_ID, ADDR_TORQUE_LIMIT)
                eeprom = self._bus.read_word(GRIPPER_ID, ADDR_MAX_TORQUE_LIMIT)
        except Exception as exc:     # noqa: BLE001 - diagnostics; a lost bus reconnects
            self.get_logger().warn(f'leader gripper: Torque_Limit not read at start: {exc!r}')
            return
        text = (f'leader gripper: servo {GRIPPER_ID} Torque_Limit (RAM) {ram}, '
                f'Max_Torque_Limit (EEPROM, read only) {eeprom}')
        if (ram is not None and eeprom is not None and 0 < eeprom <= TORQUE_LIMIT_MAX
                and ram < eeprom):
            self._grip_limit_prev = eeprom
            text += (' - RAM below EEPROM, likely left by a leader session of an earlier '
                     'process: the EEPROM value is restored before the next motion')
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self._report(text)

    def _measured_q(self) -> Optional[Dict[str, float]]:
        ticks = self._read_all()
        return None if ticks is None else self._q_of(ticks)

    def _match(self, ref: Dict[str, float]) -> Optional[str]:
        tol = float(self._p('leader.activation_match_tolerance_rad'))
        # Plain differences: both values live in the joint's window, so 2 pi - eps is
        # a 2 pi - eps mismatch, not a match. The gripper only if it is to be driven.
        diffs = {n: self._leader_q[n] - ref[n] for n in self._leader_joints()
                 if n != self._grip_name() or self._grip_required()}
        bad = {n: round(d, 3) for n, d in diffs.items() if abs(d) > tol}
        if bad:
            return f'leader does not match the follower (rad, tolerance {tol}): {bad}'
        return None

    def _try_activate(self, now: float, stops: int, withdrawals: int):
        # Runs in the tick thread for tens of ms (bus reads, filter checks, torque-on).
        # J, Space or gateway arming arriving meanwhile find leader mode OFF and cannot
        # end it: the final locked block refuses instead (`stops`, `withdrawals`: the
        # counts at the J rising edge, _on_leader_enable).
        why = self._refusal(now)
        with self._state_lock:
            if why is None and self._motion_owner is not None:
                why = f'arm motion "{self._motion_owner}" is running'
        measured = None
        if why is None:
            measured = self._measured_q()
            if measured is None:
                why = f'no measured arm pose ({self._read_failure()})'
        if why is None:
            # Recovery from outside the limits is I's job, not the follower's (user
            # decision 2026-10-09): it would need the leader outside too, and every
            # tick's step re-anchored at a lagging measured pose.
            out = self._filter.outside_limits(measured)
            touch = self._self_contact(measured)
            if out:
                why = (f'measured pose outside joint limits ({describe_outside(out)}): '
                       'press I (go_home) first, it plans a recovery path back inside')
                if touch:
                    why += f'; measured pose in self-contact ({touch})'
            elif touch:
                why = (f'measured pose in self-contact ({touch}): press I (go_home) '
                       'first, it separates the pair')
        if why is None:
            v = self._filter.check_pose(measured, self._envelope())
            if not v.ok:
                why = f'measured pose violates the envelope: {v}'
        if why is None:
            why = self._match(measured)
        grip_why = None
        torqued_here = False
        if why is None:
            try:
                torqued_here = not self._torque_on
                self._ensure_torque()
            except MotionRefused as exc:
                why = str(exc)
            except Exception as exc:     # noqa: BLE001 - e.g. the serial port lost
                why = (f'torque-on failed: {exc!r} - any servo already torqued holds its '
                       'present position (goal := present is written first)')
        commanded = measured
        if why is None:
            # A held arm (torque on, user decision 2026-10-13): the first leader step
            # continues from the held goals, it does not re-anchor a sagging servo.
            try:
                commanded = self._q_of(self._held_start(self._ticks_of(measured),
                                                        'leader activation',
                                                        sids=LEADER_IDS))
            except HeldPoseInvalid as exc:
                why = str(exc)
            except Exception as exc:     # noqa: BLE001 - the measured start, as before
                self._report(f'leader activation: held-goal start failed ({exc!r}): '
                             'starts at the measured pose')
        if why is None:
            # Last before the final check: its own limit, or the gripper is held (fail
            # closed, FR-17 amendment 2026-11-16).
            grip_why = self._apply_grip_limit()
        if why is None:
            with self._state_lock:
                why = self._activation_overtaken(stops, withdrawals)
                if why is None:
                    self._enter_leader(commanded, grip_driven=grip_why is None)
        if why is not None:
            text = f'leader activation REFUSED: {why}'
            self.get_logger().warn(text)
            if torqued_here and self._torque_on and why.startswith('Space'):
                # Space found the arm stationary (limp, from an earlier Space) and did
                # nothing; this activation then turned the torque on (audit r6).
                self._torque_off_all('Space during the leader activation that turned '
                                     'the torque on')
            self._restore_grip_limit('leader activation refused')
            self._publish_leader_state(f'off ({text})')
            self._report(text)
            return
        if grip_why is None:
            self.get_logger().info(
                'leader mode ACTIVE (gripper driven, ' + (
                    "at the servo's own Torque_Limit like the limbs)" if self._grip_limit is None
                    else f'leader.gripper_torque_limit {self._grip_limit})'))
            return
        text = (f'leader mode ACTIVE - follower gripper HELD, NOT driven by the leader: '
                f'{grip_why}')
        self.get_logger().warn(text)
        self._report(text)
        self._publish_leader_state(f'active (gripper held: {grip_why})')

    def _activation_overtaken(self, stops: int, withdrawals: int) -> Optional[str]:
        """What arrived while _try_activate ran. Caller holds _state_lock."""
        if self._stop_count != stops:
            return 'Space during activation'
        if self._withdraw_count != withdrawals or not self._enable_last:
            return 'enable withdrawn during activation (J, or O/I/P first)'
        if self._gateway_armed is not False:
            return 'external-goal gateway ARMED during activation'
        if self._motion_owner is not None:
            return f'arm motion "{self._motion_owner}" claimed the arm during activation'
        return None

    def _enter_leader(self, commanded: Dict[str, float], grip_driven: bool = False):
        """Caller holds _state_lock and has checked _activation_overtaken.
        `commanded`: the pose the follower holds (_held_start). `grip_driven`: the
        leader gripper limit is in servo 6 (_apply_grip_limit)."""
        self._motion_owner = 'leader'
        self._grip_driven = grip_driven
        self._leader_state = LEADER_ACTIVE
        self._pending_home = None
        self._stop.clear()                   # from before this activation (counted above)
        self._cmd_q = dict(commanded)
        try:
            self._leader_edges = {n: (min(lo, commanded[n]), max(hi, commanded[n]))
                                  for n, (lo, hi) in self._leader_band().items()}
        except ValueError:
            self._leader_edges = None        # _refusal refused already; the tick re-checks
        self._held = False
        self._follow_steps = []
        self._follow_job = None
        self._last_replan = -math.inf
        self._drop_replan()
        self._replan_failed = None
        self._replan_discarded = None
        self._hold_binding = ''
        self._lag_base, self._lag_gated = None, False
        self._lag_gate_now = self._lag_shrink = None
        self._follow_rising = []
        self._publish_leader_state('active')

    def _leave(self, reason: str):
        """Every leader termination except Space: stop, then return home."""
        with self._state_lock:
            was = self._leader_state
            self._leader_state = LEADER_OFF
            if self._motion_owner == 'leader':
                self._motion_owner = None
            self._cmd_q = None
            if was != LEADER_OFF:
                self._leaving += 1          # a waiting command claims after this leave
            stops = self._stop_count        # a Space during the leave cancels its return
            self._publish_leader_state(f'off ({reason})')
        self._drop_replan()                 # now: the hold's bus reads compete with it
        if was != LEADER_OFF:
            try:
                self._hold(f'leader mode off: {reason}')
                self._restore_grip_limit(f'leader mode off: {reason}')
                self._schedule_home(f'leader mode off: {reason}', stops)
            finally:
                with self._state_lock:
                    self._leaving -= 1

    def _leader_tick(self):
        # An exception escaping a callback ends MultiThreadedExecutor.spin(): every
        # timer of the node dies with it, the pending return home included (audit of
        # 5762c7f: J during a tick made the tick raise on the cleared _cmd_q; the arm
        # held and never returned home). None may escape.
        t0 = time.monotonic()
        stamp = None
        if self._follow_debug_pub is not None:
            self._dbg = {}
            try:
                stamp = self.get_clock().now()
            except Exception:        # noqa: BLE001 - _follow_debug stamps it then
                pass
        try:
            self._leader_tick_once()
        except Exception as exc:     # noqa: BLE001 - the executor must survive
            self.get_logger().error(f'leader tick: unexpected error: {traceback.format_exc()}')
            with self._state_lock:
                active = self._leader_state != LEADER_OFF
                self._leader_state = LEADER_OFF
                if self._motion_owner == 'leader':
                    self._motion_owner = None
                self._cmd_q = None
            if active:
                # As after an unexpected error in P: hold and report, no automatic
                # return home (SR-17 rule 13, user decision 2026-10-09).
                self._publish_leader_state(f'off (unexpected error: {exc!r})')
                self._hold(f'leader mode off: unexpected error in the leader tick, arm '
                           f'holds, no automatic return home: {exc!r}')
            else:
                # Leader mode was off for this tick (an activation, or a tick overtaken
                # by a leave). Setpoint writes are gated on ACTIVE under the serial
                # lock, and a torque-on error in an activation is a refusal there.
                self._report(f'leader tick: unexpected error while leader mode was off '
                             f'(no leader setpoint written by this tick): {exc!r}')
            self._restore_grip_limit(f'leader tick error: {exc!r}')
        self._follow_debug(t0, stamp)

    def _note(self, key: str, value):
        """A follow_debug note of the running tick; nothing when disabled."""
        d = self._dbg
        if d is not None:
            d[key] = value

    def _phase(self, name: str, t0: float) -> float:
        """Adds the time since `t0` to the running tick's follow_debug phase `name`
        (LeaderFollowDebug <name>_s); returns now, the start of the next phase."""
        t = time.monotonic()
        d = self._dbg
        if d is not None:
            ph = d.setdefault('phases', {})
            ph[name] = ph.get(name, 0.0) + (t - t0)
        return t

    def _follow_debug(self, t0: float, stamp):
        """Publish the tick's diagnostics (/arm/leader/follow_debug) while leader mode is
        active. Runs after the tick, servo write included, and never raises: an
        exception here must not reach the tick's handler, which ends leader mode."""
        d, self._dbg = self._dbg, None
        prev, self._dbg_prev_t0 = self._dbg_prev_t0, t0
        if d is None:
            return
        try:
            cmd = self._commanded()
            if cmd is None:                      # leader mode not active
                return
            t_end = d.get('t_end') or time.monotonic()
            arm = self._leader_joints()
            msg = LeaderFollowDebug()
            msg.header.stamp = (stamp or self.get_clock().now()).to_msg()
            msg.joint_names = list(arm)
            msg.setpoint = [float(cmd[n]) for n in arm]
            for field in ('target', 'measured'):
                q = d.get(field)
                if q is not None:
                    setattr(msg, field, [float(q[n]) for n in arm])
            msg.tick_duration_s = t_end - t0
            msg.period_s = 0.0 if prev is None else t0 - prev
            msg.outcome = d.get('outcome', 'other')
            ph = d.get('phases', {})
            for name in DEBUG_PHASES:
                setattr(msg, f'{name}_s', float(ph.get(name, 0.0)))
            msg.other_s = msg.tick_duration_s - sum(ph.get(n, 0.0) for n in DEBUG_PHASES)
            msg.check_poses = int(d.get('check_poses', 0))
            self._follow_debug_pub.publish(msg)
        except Exception as exc:     # noqa: BLE001 - diagnostics never end leader mode
            try:
                self.get_logger().warn(f'follow_debug not published: {exc!r}',
                                       throttle_duration_sec=10.0)
            except Exception:        # noqa: BLE001
                pass

    def _commanded(self) -> Optional[Dict[str, float]]:
        """The commanded pose, None once leader mode is off. _leave (enable and arming
        callbacks, another thread) clears it at any point of a tick: read it once per
        use, never through self._cmd_q twice."""
        with self._state_lock:
            return self._cmd_q if self._leader_state == LEADER_ACTIVE else None

    def _set_commanded(self, q: Dict[str, float]):
        with self._state_lock:              # not after a concurrent _leave
            if self._leader_state == LEADER_ACTIVE:
                self._cmd_q = q

    def _leader_tick_once(self):
        now = time.monotonic()
        with self._state_lock:
            requested, self._activation_requested = self._activation_requested, None
        if requested is not None:
            if self._leader_state == LEADER_OFF:
                self._try_activate(now, *requested)
                self._note('outcome', 'activation')     # published only if it succeeded
            return
        if self._commanded() is None:
            self._drop_replan()
            if self._grip_limit_prev is not None and now - self._grip_restore_t >= 1.0:
                self._grip_restore_t = now
                self._restore_grip_limit('retry while leader mode is off', idle_only=True)
            return
        if not self._enable_last:            # a withdrawal no other path acted on
            self._leave('enable withdrawn (J, or O/I/P first)')
            return
        why = self._refusal(now)
        t = self._phase('refusal', now)
        if why is not None:
            self._leave(why)
            return
        measured = self._measured_q()
        t = self._phase('read', t)
        if measured is None:
            self._leave(f'no measured arm pose ({self._read_failure()})')
            return
        self._note('measured', measured)
        env = self._envelope()
        mv = self._filter.check_pose(measured, env)
        t = self._phase('envelope', t)
        if not mv.ok:
            self._leave(f'MEASURED pose violates the envelope (SR-17 rule 8): {mv}')
            return
        cmd_q = self._commanded()
        if cmd_q is None:
            return                           # left meanwhile; _leave held the arm
        tol = float(self._p('leader.command_tolerance_rad'))
        # The gripper (FR-17 amendment 2026-11-16): not judged while held (not driven); a
        # lag on its CLOSING side (jaws stopped by an object, pressing with the leader
        # limit at most) does not end leader mode - one on its opening side does.
        g = self._grip_name()
        lag = {n: abs(measured[n] - cmd_q[n]) for n in self._leader_joints()
               if n != g or (self._grip_driven
                             and not self._grip_squeezing_q(cmd_q[g], measured[g]))}
        if max(lag.values()) > tol:
            self._leave(f'measured differs from commanded by > {tol} rad: '
                        + str({n: round(v, 3) for n, v in lag.items() if v > tol}))
            return
        arm = self._leader_joints()
        here = self._ticks_of(measured)
        if self._lag_base is None:           # the lag-growth gate's baseline: activation
            self._lag_growth(here, self._held_lag(here))
        else:
            self._lag_caught_up(here)
        if self._held and not self._follow_steps:
            t = time.monotonic()
            try:
                start = self._replan_start(here, cmd_q)
            except HeldPoseInvalid as exc:   # fail closed: no search, no write
                self._hold_reason = str(exc)
                self._note('outcome', 'holding')
                self._hold_status(now, measured)
                return
            adopted = self._replan_step(now, measured, here, start, env)
            self._phase('replan', t)
            if not adopted:
                self._note('outcome', 'holding')
                self._hold_status(now, measured)
                return
        if self._follow_steps:
            self._follow_step(measured, here, env)
            return
        t = time.monotonic()
        # Both angles live in the joint's window [range_center - pi, range_center + pi),
        # so the physical travel is the PLAIN difference. A difference beyond pi means a
        # value crossed the window edge (a wrong range_center_rad, here or on the
        # leader): rejected, never driven the long way round.
        cmd_q = self._commanded()            # again: a re-plan adopted above resets it
        if cmd_q is None:
            return
        # Clamped into the command band: an arm at the limit itself sags past it and
        # trips the measured-pose check (robot 2026-10-15).
        goal, far = self._leader_target(cmd_q)
        self._note('target', goal)
        d = {n: goal[n] - cmd_q[n] for n in arm}
        if far:
            self._reject_setpoint(f'leader and follower differ by > pi on {far}: '
                                  'angle window edge crossed - check range_center_rad')
            return
        # Velocity limit by UNIFORM scaling: the step keeps the direction of the motion.
        # Near a command-band edge the joint moving toward it brakes (leader.edge_slowdown_*,
        # user decision 2026-10-16, gripperx_arm.leader_band.step_scale) - and with it the
        # whole step, so the direction is kept.
        scale = step_scale(cmd_q, goal, self._leader_edges,
                           float(self._p('leader.max_joint_velocity_rad_s')),
                           float(self._p('leader.rate_hz')),
                           float(self._p('leader.edge_slowdown_zone_rad')),
                           float(self._p('leader.edge_slowdown_velocity_rad_s')),
                           measured)
        new = dict(cmd_q)
        for n, v in d.items():
            new[n] = cmd_q[n] + v * scale
        try:
            ticks = self._ticks_of(new, measured)
        except MotionRefused as exc:
            self._reject_setpoint(str(exc))
            return
        # The pose checked is the one actually written (whole ticks), and the motion to
        # it from where the arm IS.
        written = self._q_of({**here, **{sid: ticks[sid] for sid in LEADER_IDS}})
        t = self._phase('target', t)
        v = self._filter.check_setpoints([measured, written], env)
        t = self._phase('check', t)
        self._note('check_poses', v.checked)
        if not v.ok:
            self._reject_setpoint(str(v), verdict=v)
            return
        gate = self._lag_gate_now
        if self._lag_gated and gate is not None:
            # The gate closed at the last hold and nothing has re-judged it since (it is
            # evaluated at hold ticks only): a direct step must not move a goal further
            # from the arm or across it either (audit of D3 (2026-10-23), 2026-10-29: after
            # a path admitted through the gate, one direct step put limb3 17 ticks further
            # from a stuck arm). Refused -> hold, where the gate is judged again (an arm
            # that followed reopens it).
            gated, threshold, ref = gate
            held = {sid: here[sid] if self._goal.get(sid) is None else self._goal[sid]
                    for sid in LEADER_IDS}
            over = {n: ticks[sid] for n, sid in zip(arm, LEADER_IDS)
                    if ticks[sid] != held[sid]
                    and not (sid == GRIPPER_ID and self._grip_squeezing(ticks[sid], here[sid]))
                    and not lag_bound_ok(
                        ticks[sid], here[sid], held[sid],
                        None if n in gated else ref.get(n, 0) + threshold,
                        self._lag_noise)}
            if over:
                self._reject_setpoint(f'lag-growth gate closed: the leader step would move '
                                      f'{over} further from the measured arm or across it')
                return
        # Compared with the goals the servos hold, not with the measured pose: under
        # load the two differ, and the held goal is what a repeated write would repeat.
        drive = self._drive_ids()
        if all(ticks[sid] == self._goal[sid] for sid in drive):
            self._note('outcome', 'unchanged')
            return
        t = time.monotonic()
        with self._serial_lock:
            if self._leader_state != LEADER_ACTIVE or self._stop.is_set():
                return
            self._write('approve', {sid: ticks[sid] for sid in drive})
        self._note('t_end', self._phase('write', t))
        self._note('outcome', 'written')
        self._set_commanded(written)

    def _follow_step(self, measured, here: Dict[int, int], env):
        """The next setpoint of an adopted path: exactly the step the planner validated
        (_planner 'leader'), checked again against the envelope as it is NOW - from
        where the arm IS (_step_verdict). Rejected -> hold; the search it came from
        counts as failed unless only the measured pose's lag behind the commanded one
        made the difference (the planned step itself still passes): then a new search
        from the held pose may start at once."""
        ticks = self._follow_steps[0]
        rising = self._follow_rising[0] if self._follow_rising else ()
        written = self._q_of({**here, **ticks})
        prev = self._commanded()
        if prev is None:
            return                           # left meanwhile; _leave held the arm
        shrink = self._lag_shrink
        if shrink is not None:
            # A path admitted through the closed lag-growth gate: no goal change beyond
            # the bound it was admitted with (_lag_shrink_why).
            over = {self._cal.joints[IDS.index(sid)]: ticks[sid] for sid in LEADER_IDS
                    if ticks[sid] != self._goal.get(sid)
                    and not (sid == GRIPPER_ID and self._grip_squeezing(ticks[sid], here[sid]))
                    and not lag_bound_ok(ticks[sid], *shrink[sid], self._lag_noise)}
            if over:
                k = self._follow_total - len(self._follow_steps) + 1
                self._reject_setpoint(
                    f'planned step {k} of {self._follow_total}: it would move {over} '
                    'further from the measured arm than the bound this path was admitted '
                    'with through the closed lag-growth gate', record=False,
                    note='NOT recorded as a failed search; the next re-plan is judged from '
                         'where the arm is then')
                return
        t = time.monotonic()
        v, planned_ok = self._step_verdict(measured, prev, written, rising, env)
        self._phase('check', t)
        self._note('check_poses', v.checked)
        if not v.ok:
            k = self._follow_total - len(self._follow_steps) + 1
            self._reject_setpoint(f'planned step {k} of {self._follow_total}: {v}',
                                  record=not planned_ok, verdict=v)
            return
        self._follow_steps.pop(0)
        if self._follow_rising:
            self._follow_rising.pop(0)
        if not self._follow_steps:
            self._follow_job = None
            self._lag_shrink = None
            self._publish_leader_state('active (path done, following the leader)')
        drive = self._drive_ids()
        if any(ticks[sid] != self._goal[sid] for sid in drive):
            t = time.monotonic()
            with self._serial_lock:
                if self._leader_state != LEADER_ACTIVE or self._stop.is_set():
                    return
                self._write('approve', {sid: ticks[sid] for sid in drive})
            self._note('t_end', self._phase('write', t))
        self._note('outcome', 'follow_step')
        self._set_commanded(written)

    # ---------------------------------------------------------------- re-plan (held arm)
    # User decision 2026-10-07: after a rejection the arm HOLDS (no write) until the
    # leader's pose is valid again; then a path from the held pose to it is planned and
    # followed - no jump, no walking along the envelope edge. The search runs in a
    # _ReplanJob thread (slices of planner.time_budget_s, resumed; planner.max_checks
    # bounds it as a whole): the tick never waits for it. A search that found no path
    # is not repeated until its inputs change (the planner is deterministic: the same
    # inputs fail the same way).

    def _same_inputs(self, job: _ReplanJob, held, target, env=None,
                     same_env: bool = False) -> bool:
        """Leader target and held pose within leader.replan_repeat_tolerance_rad of the
        job's (two poses that close count as the same search); with `same_env`, also
        no envelope change that could free a path (_envelope_may_free). A RUNNING
        search is not tied to its envelope: its result is re-checked against the
        current one on adoption and every step on writing."""
        tol = float(self._p('leader.replan_repeat_tolerance_rad'))
        arm = self._leader_joints()
        return ((not same_env or not self._envelope_may_free(job.env, env))
                and max(abs(target[n] - job.target[n]) for n in arm) <= tol
                and max(abs(held[n] - job.start[n]) for n in arm) <= tol)

    def _envelope_may_free(self, old, new) -> bool:
        """Could envelope `new` allow a path that a search under `old` did not find?
        Only a wheel that is known under `new` can free space: one that was unknown
        (swept over its stops) under `old`, or whose reading moved by more than
        safety_filter.steer_motion_margin_rad from the one `old` was built from. A
        wheel going stale is swept: more body, never less. Not the identity of the
        envelope object: _envelope() builds a new one whenever a reading changes at its
        0.01 rad rounding or freshness flickers, and steering jitter then restarted the
        same failed search every replan_period_sec (audit round 3, 2026-10-08). Drift
        below the margin can free a sliver of space; the search for it starts once
        the wheel has drifted past the margin or the leader moves - until then the arm
        holds, the conservative side."""
        if old is new:
            return False
        so = {} if old is None else old.steer      # None: every wheel swept
        sn = {} if new is None else new.steer
        if so is None or sn is None:               # not from envelope_measured
            return True
        margin = float(self._p('safety_filter.steer_motion_margin_rad'))
        return any(sn.get(n) is not None
                   and (so.get(n) is None or abs(sn[n] - so[n]) > margin)
                   for n in self._steer_names)

    def _drop_replan(self):
        # One read: the tick (executor thread) and destroy_node (another thread) may
        # both drop; cancelling a job twice is harmless, calling None.cancel() is not.
        job, self._replan = self._replan, None
        if job is not None:
            job.cancel()

    def _replan_start(self, here: Dict[int, int], cmd_q) -> Dict[str, float]:
        """Where a re-plan of the held follower starts (user decisions 2026-10-13,
        2026-10-30): per leader servo (gripper included) the goal it holds - the last approved setpoint - unless
        it is unknown, with the torque and static checks of _held_start (audit HG1 F5;
        memoised there, the held tick repeats it); HeldPoseInvalid if that start is not
        valid. A path from the measured pose would re-anchor a servo that
        sags under load at its first step. Every step is still checked from the
        measured pose before it is written (_follow_step)."""
        return self._q_of(self._held_start(here, 'leader re-plan', report=False,
                                           sids=LEADER_IDS))

    def _replan_step(self, now: float, measured, here: Dict[int, int], start, env) -> bool:
        """True once a path toward the leader has been adopted (_follow_steps set).
        `start`: the held pose the path starts from (_replan_start); `here`: the
        measured ticks. Every branch that keeps the arm holding says why
        (_hold_reason, the throttled status line of _hold_status)."""
        clamped, far = self._leader_target(start)
        if far:
            self._hold_reason = (f'leader and follower differ by > pi on {far} (angle '
                                 'window edge crossed - check range_center_rad)')
            return False                     # window edge crossed: the arm holds
        target = {**measured, **clamped}
        lag = self._held_lag(here)
        was_gated = self._lag_gated
        grown, gate = self._lag_gate(here, lag)
        if gate is not None:
            # The held-goal ratchet (robot test 2026-10-20): a path adopted from goals the
            # arm lags wrote its first step, its next one was refused from the measured
            # pose, and the next search started from goals one step further from an arm
            # that did not move (limb2 goal - measured 8 -> 13 ticks). Judged as GROWTH of
            # the per-joint setpoint - measured deviation since the arm last tracked
            # (_lag_growth, user decision D2 (2026-10-23)), not as the deviation itself: at
            # rest limb1-4 sit 5-9 ticks off their goals and re-plans from there recover
            # (audit 2026-10-20). Only growth the goal's own travel away from the arm
            # explains counts: a sag closes no gate (user decision 2026-10-30), so every
            # joint in `grown` has a goal that travelled away by that much while the arm
            # did not keep up, which is what the text below says. While the gate is
            # closed, searches run and finish as usual, but a path is adopted only if its
            # goal changes bring the goals toward the measured arm (user decision D3 (2026-10-23); the rule and why the ratchet
            # stays impossible: _lag_shrink_why). The goals are NOT pulled back to the
            # measured pose: re-anchoring a servo that sags under load lowers the arm
            # hold by hold (2026-10-12) - an admitted path moves them only along a planned
            # path toward the operator's leader target, never past it.
            limit = self._max_lag_growth
            reopen = max(0, limit - self._lag_noise)
            text = (
                f'the held goals moved away from an arm that did not keep up: goal - '
                f'measured {lag} ticks, growth by goal travel away from the arm {grown} '
                f'(more than leader.replan_max_goal_lag_growth_ticks = {limit} closes the gate; '
                f'it opens again at {reopen} or less): only a re-plan whose goal changes '
                'bring the goals closer to the measured arm is adopted (no limb further '
                'from it or across it, a gated one closer); otherwise waiting for the arm or '
                'the leader to move, or press I (go_home, planned from the held goals)')
            # Once per closing, and a closing within hold_status_period_sec of the last
            # message is not announced again; the hold status line carries it meanwhile.
            if not was_gated and now - self._lag_gate_t >= self._hold_status_period:
                self._lag_gate_t = now
                self.get_logger().warn(f'leader: {text}')
                self._report(f'leader: {text}')
                self._publish_leader_state(f'active (holding; {text})')
        if was_gated:
            self._hold_reason = self._hold_detail     # the gate's text is added below
        adopted = self._replan_search(now, measured, here, start, target, env, gate)
        if gate is not None and not adopted:
            self._hold_detail = self._hold_reason
            self._hold_reason = f'{text}; {self._hold_detail}'
        return adopted

    def _lag_gate(self, here: Dict[int, int], lag: Dict[str, int]):
        """(growth per gated joint, gate) of the lag-growth gate at this hold tick; gate
        None while open, else (gated joints, the threshold they grew beyond, reference
        lag per joint) for _lag_shrink_why. Sets _lag_gated. Hysteresis: a closed gate
        opens again only at a growth of the limit minus leader.replan_lag_gate_noise_ticks
        or less - at exactly the limit, one tick of reading noise would open it for a
        tick, and an ordinary adoption in that tick (no shrink rule) could move the goals
        away again."""
        limit = self._max_lag_growth
        threshold = max(0, limit - self._lag_noise) if self._lag_gated else limit
        grown = self._lag_growth(here, lag, threshold)
        self._lag_gated = bool(grown)
        self._lag_gate_now = (set(grown), threshold, dict(self._lag_base[1])) if grown \
            else None
        return grown, self._lag_gate_now

    def _replan_search(self, now: float, measured, here: Dict[int, int], start, target,
                       env, gate) -> bool:
        """_replan_step past the lag-growth gate: polls, adopts, or starts the search.
        `gate`: None while the gate is open, else what _lag_shrink_why judges by."""
        job = self._replan
        if job is not None:
            if not self._same_inputs(job, start, target):
                self._drop_replan()          # its result would be stale
            elif job.result is None:         # still searching; the arm holds
                self._hold_reason = (f'searching a path to the leader '
                                     f'({time.monotonic() - job.t0:.1f} s)')
                if job.slices != job.reported_slices:
                    job.reported_slices = job.slices
                    self._publish_leader_state(
                        f'active (holding; searching a path to the leader: '
                        f'{time.monotonic() - job.t0:.1f} s, {job.slices} slice(s) of '
                        f'{job.planner.budget:.2f} s cut by the time cap, resumed)')
                return False
            else:
                adopted = self._adopt_replan(job, start, measured, here, env, gate=gate,
                                             target=target)
                self._replan = None          # after the report: None = nothing pending
                return adopted
        tv = self._filter.check_pose(target, env)
        if not tv.ok:
            self._hold_reason = f'the leader target is not valid: {tv}'
            self._hold_binding = 'leader target ' + self._binding(tv)
            return False
        failed = self._replan_failed
        if failed is not None and self._same_inputs(failed, start, target, env, True):
            # reported once; waits for the leader to move
            self._hold_reason = (f'{failed.why_failed} - no new search until the leader, '
                                 'the held pose or the envelope change')
            return False
        disc = self._replan_discarded
        reuse = None
        if disc is not None:
            # The same search would find the same path. "The arm moved" only beyond
            # leader.replan_discard_move_ticks on a leader servo: compared exactly, one tick
            # of reading noise restarted the identical search every replan_period_sec
            # (audit 2026-10-20; the livelock of 2026-10-08).
            same = self._same_inputs(disc[0], start, target, env, True)
            if same and disc[3]:
                # Parked by the closed lag-growth gate: the leader moving back toward the
                # arm by less than leader.replan_repeat_tolerance_rad is the operator's
                # way out of the gate (D3 (2026-10-23)) and must start a new search. Leader reading
                # noise counts as no move, as for the arm.
                tick = 2.0 * math.pi / self._cal.ticks_per_rev
                same = max(abs(target[n] - disc[0].target[n])
                           for n in self._leader_joints()) <= self._discard_move_ticks * tick
            still = all(abs(here[sid] - disc[1][sid]) <= self._discard_move_ticks
                        for sid in LEADER_IDS)
            if same and still:
                return False
            self._replan_discarded = None
            # The arm moved, the inputs did not: the cached path is re-checked from the
            # new measured pose instead of searched again (no search thread competing
            # with the tick). Not for a lift: its start-drift guard measures from the
            # search's own start, so a lift is searched again.
            if same and not disc[2]:
                reuse = disc[0]
        if now - self._last_replan < float(self._p('leader.replan_period_sec')):
            if reuse is not None:
                self._replan_discarded = disc     # keep it until the period has passed
            return False
        if reuse is not None:
            self._last_replan = now
            return self._adopt_replan(reuse, start, measured, here, env, cached=True,
                                      gate=gate, target=target)
        self._last_replan = now
        self._replan_failed = None
        touch = {n: h for n, h in self._filter.ground_heights(start).items() if h <= 0.0}
        if touch:
            # The lift from AT OR BELOW the ground (user decision 2026-10-15) is the
            # operator's I only; leader mode never lifts out of the ground (unreachable
            # while rule 8 holds the measured pose clear - kept as a guard).
            self._hold_reason = (f'held pose at or below the ground '
                                 f'({self._filter.describe_ground(touch)}): press I')
            return False
        # direct_only=False, deliberately: SR-17 rule 11 limits AUTOMATICALLY triggered
        # motions to the direct path (planner.automatic_paths), and "leader re-plan is
        # not an automatic motion - user decision 2026-10-08": it follows the operator's
        # leader arm, so detours stay allowed here. The ground recovery (rise_ground)
        # likewise, user decision 2026-10-20 (SR-17 rule 13): a held start inside the
        # ground margin for its first step is lifted out one joint at a time, as the
        # operator's I does; never once the path is clear of it.
        self._replan = _ReplanJob(self._planner(float(self._p('planner.time_budget_s')),
                                                direct_only=False, executor='leader',
                                                rise_ground=self._ground_mode == 'operator'),
                                  dict(start), target, env, self._cal,
                                  float(self._p('planner.max_search_s')))
        self._replan.measured_ticks = dict(here)
        self._replan.thread.start()
        self._hold_reason = 'searching a path to the leader'
        self._publish_leader_state('active (holding; searching a path to the leader)')
        return False

    def _step_verdict(self, measured, prev, written, rising, env):
        """(verdict, planned step ok) of one step of an adopted path, to `written` from
        the commanded pose `prev` (the previous planned setpoint) - the arm is at
        `measured`.

        Normal step (`rising` empty): the motion from the MEASURED pose, as every leader
        step (SR-17 rule 5). Rising out of the ground margin (`rising`: the entries
        still rising at `prev`, from the planner's own check of the path - never a new
        recovery): the motion the servos really make, measured -> written - every other
        check as normal, and each rising entry certified above the ground anywhere inside
        that step's joint box (_rising_floor > 0); the measured pose is statically clear
        (the tick's SR-17 rule 8 check). The planned step's own rising rules (one joint,
        strictly rising, certified between the setpoints) depend on the planned poses
        only and were checked when the path was adopted (_path_chain); they are checked
        again only to classify a refusal (tick budget, audit 2026-10-20 MED-4: the full
        re-check cost ~37 ms a step on the laptop). 'planned step ok' (computed on a
        refusal only): the planned step prev -> written still passes, i.e. the refusal
        comes from the measured pose lagging behind the commanded one. Since 2026-11-13
        the gripper is commanded like every other leader joint; a planned path keeps it at
        its held goal (_planner), the direct steps move it."""
        flt = self._filter
        if not rising:
            v = flt.check_setpoints([measured, written], env)
            if v.ok:
                return v, True
            return v, flt.check_setpoints([prev, written], env).ok
        v = flt.check_setpoints([measured, written], env, ground_skip=rising)
        if v.ok:
            low = self._rising_floor(measured, written, rising)
            bad = {n: b for n, b in low.items() if b <= 0.0}
            if bad:
                n, b = min(bad.items(), key=lambda kv: kv[1])
                v = Verdict(False, 'ground', f'{n}: ground recovery - from the measured '
                            'pose the step\'s joint box may reach the ground (certified '
                            f'bound {b * 1e3:+.4f} mm)', 0, 1)
        if v.ok:
            return v, True
        return v, flt.check_setpoints([prev, written], env, rising=rising).ok

    def _rising_floor(self, measured, written, rising) -> Dict[str, float]:
        """Per rising entry, a certified lower bound of its ground clearance anywhere
        inside the joint box measured -> written: the lower of the two end poses, each
        grown by the step's motion bound (SafetyFilter.ground_heights with dq) - every
        configuration in that box lies within half the motion bound of one end pose
        (safety_filter module docstring), the argument the normal check rests on. Two
        poses instead of SafetyFilter.ground_step_bound's 2^k box vertices (k = joints
        that differ, up to five with sag): ~2.5 ms instead of ~16 ms on the laptop, at
        the price of a more conservative bound."""
        flt = self._filter
        dq = {k: abs(written[k] - measured[k]) for k in measured}
        hm, hw = flt.ground_heights(measured, dq), flt.ground_heights(written, dq)
        return {n: min(hm[n], hw[n]) for n in rising}

    def _adopt_replan(self, job: _ReplanJob, start, measured, here: Dict[int, int],
                      env, cached: bool = False, gate=None, target=None) -> bool:
        res = job.result
        text = (f'leader re-plan: path {res.method or "NONE"} ({len(res.path)} waypoints, '
                f'{res.checks} checks, {job.elapsed_s * 1e3:.0f} ms in {job.slices} '
                f'slice(s) of {job.planner.budget * 1e3:.0f} ms)')
        if cached:
            text += ' - the discarded path, re-checked after the arm moved'
        if res.ok and res.ground:
            text += f'; lifting out of the ground margin ({res.ground})'
        if job.planner.f is not self._filter or job.cal is not self._cal:
            text += ': DISCARDED, computed against a safety filter or calibration since replaced'
            self.get_logger().warn(text)
            self._report(text)
            return False
        if not res.ok:
            self._replan_failed = job
            job.why_failed = f'no path to the leader: {res.reason}'
            text += f': {res.reason}'
            if job.stalled:
                text += ' - not one pose passed within one slice'
            self.get_logger().warn(text)
            self._report(text)
            self._publish_leader_state(f'active (holding; no path to the leader: {res.reason}; '
                                       'waiting for the leader to move)')
            return False
        # The search ran from job.start; the path is re-checked from the held pose as it
        # is NOW - the steps the follower will write from there, as the planner checks
        # them (and each one again from the measured pose when written, _follow_step).
        # While a segment still rises out of the ground margin, the next one is checked
        # too: the per-step rising entries come from this check, and execution re-checks
        # each rising step with exactly them.
        # Precomputed by the search thread when its start and envelope are those of this
        # tick (MED-4: ~75 ms on the tick otherwise); else re-checked here.
        chain = job.chain
        if chain is None or chain[0] != start or chain[1] is not env:
            chain = (start, env, _path_chain(job.planner, start, res.path,
                                             job.planner.joints, env))
        poses_seq, rising_at, why = chain[2]
        if why is not None:
            text += f': DISCARDED, {why}'
            self.get_logger().warn(text)
            self._report(text)
            return False                     # start differs: the next search is a new one
        steps = [{sid: t[sid] for sid in LEADER_IDS} for t in map(self._ticks_of, poses_seq)]
        # The first step as execution will check it, from the measured pose of THIS
        # tick (user decision 2026-10-20): a path whose first step the follower would
        # refuse is not adopted - and not recorded as a failed search (the planner found
        # a valid path from the held pose; only the arm's lag refuses it).
        why = None
        if steps:
            first = self._q_of({**here, **steps[0]})
            v, _planned = self._step_verdict(measured, start, first, rising_at[0], env)
            if not v.ok:
                why = f'its first step from the measured pose: {v}'
                self._hold_binding = 're-plan first step ' + self._binding(v)
            elif rising_at[0]:
                why = self._drift_since(job, here)
        shrink = admitted = None
        if why is None and gate is not None:
            # Judged from the goals the servos HOLD (a path starts at the measured pose
            # where a goal is unknown, and then is refused below) toward the leader target
            # as it is NOW (a running search is kept while the leader stays within
            # replan_repeat_tolerance_rad of its target - the operator may have turned).
            with self._serial_lock:
                goals = dict(self._goal)
            try:
                aim = self._ticks_of(target if target is not None else job.target)
            except MotionRefused as exc:
                why = f'lag-growth gate closed: leader target not in ticks ({exc})'
            else:
                if any(goals[sid] is None for sid in LEADER_IDS):
                    why = 'lag-growth gate closed: a servo goal is unknown'
                else:
                    why, shrink, admitted = self._lag_shrink_why(goals, aim, steps, here,
                                                                 gate)
        if why is not None:
            text += f': DISCARDED (not a failed search), {why}'
            self._replan_discarded = (job, dict(here), bool(rising_at[0]),
                                      gate is not None and shrink is None and
                                      why.startswith('lag-growth gate closed'))
            self._hold_reason = (f'path found but not adopted: {why}; waiting for the arm '
                                 'or the leader to move')
            self.get_logger().warn(text)
            self._report(text)
            return False
        self.get_logger().info(text)
        self._report(text)
        if admitted is not None:
            gtext = ('leader: re-plan ADMITTED through the closed lag-growth gate - its '
                     f'first goal change brings {admitted} closer to the measured arm '
                     '(|goal - measured| ticks, held -> first change) and moves no limb '
                     'further from it; every later goal change of this path stays within '
                     'that bound')
            self.get_logger().warn(gtext)
            self._report(gtext)
        self._lag_shrink = shrink
        self._replan_discarded = None
        self._hold_binding = ''
        self._follow_steps, self._follow_total, self._follow_job = steps, len(steps), job
        self._follow_rising = rising_at
        self._held = False
        self._set_commanded(dict(start))
        self._publish_leader_state(f'active (following a {res.method} path, '
                                   f'{len(res.path) - 1} waypoints, {len(steps)} steps)')
        return True

    def _lag_shrink_why(self, start: Dict[int, int], target: Dict[int, int], steps,
                        here: Dict[int, int], gate):
        """While the lag-growth gate is closed (user decision D3 (2026-10-23)): may this
        path be adopted? (why not or None, the bounds for _follow_step, the gated joints
        brought closer: {joint: (held deviation, first change's deviation)}).
        `start`: the goals the servos hold now, `target`: the leader
        target at adoption, `steps`: the path's setpoints (all ticks), `here`: the
        measured ticks NOW (the arm may have moved since the search began, or the gate
        closed while it ran), `gate`: (gated joints, the threshold they grew beyond, the
        reference lags) from _lag_gate.

        The rule, for every arm position within leader.replan_lag_gate_noise_ticks of the
        reading (lag_bound_ok), on the FIRST step that changes a servo goal (the soft
        start's first fractions may round to the held ticks; an unchanged goal moves
        nothing):
        - a gated joint's goal is no further from the arm than its held goal and does not
          cross it (stays on its held goal's side of the arm at every position in the
          noise band, landing exactly on the reading allowed - user decision 2026-10-29;
          held 5 ticks above, first change 2 below: refused), and at least one gated
          joint's strictly closer;
        - any other joint's goal stays within the larger of its held goal's deviation
          and the gate's own allowance (reference lag + threshold): no joint is pushed
          into the gate; and does not cross the arm either (user decision 2026-10-30),
          unless its held goal is within the noise band of the reading (lag_bound_ok);
        - every goal it changes moves toward the leader target on that joint and not
          past it (a detour's first step against the leader is refused).
        Every LATER goal change of the path is bounded against the same reading: a gated
        joint no further from it than at that first change, the others as above; none
        across it.

        Why the ratchet stays impossible: it needs goals that move away from an arm that
        does not move. For an arm that does not move, each admitted path leaves every
        gated joint's goal no further from it and one strictly closer, by at least a
        whole tick at every position the noise allows, and no other joint beyond the gate
        allowance - finitely many admissions, none of them growing. That holds while the
        gate is closed; once it reopens one path or a run of direct steps can overshoot
        until the next hold (_lag_growth), which closes it again. An arm that moves
        lets a later step exceed the bound: refused as a lag refusal (_follow_step),
        and the next re-plan is judged from where the arm then is.
        Why it is no re-anchoring (2026-10-12): the goals move only along a path the
        planner found toward the operator's leader target, the first change in the
        leader's direction on every joint - never to the measured pose as such - so only
        the operator's leader can ask for a motion toward a sagging arm, and an arm that
        keeps sagging is followed at most as far as the leader asks."""
        gated, threshold, ref = gate
        noise = self._lag_noise
        first = next((s for s in steps if any(s[sid] != start[sid] for sid in LEADER_IDS)),
                     None)
        if first is None:
            return 'lag-growth gate closed: it does not change the held goals', None, None
        against = []
        for n, sid in zip(self._leader_joints(), LEADER_IDS):
            moved, wanted = (tick_deviation(first[sid], start[sid]),
                             tick_deviation(target[sid], start[sid]))
            # toward the target, not past it: same sign, no larger
            if moved != 0 and not (moved * wanted > 0 and abs(moved) <= abs(wanted)):
                against.append(n)
        if against:
            return (f'lag-growth gate closed: its first goal change moves {against} away from '
                    'the leader target', None, None)
        bounds, further, closer = {}, [], {}
        for n, sid in zip(self._leader_joints(), LEADER_IDS):
            floor = None if n in gated else ref.get(n, 0) + threshold
            if not lag_bound_ok(first[sid], here[sid], start[sid], floor, noise):
                across = (first[sid] != here[sid]
                          and (floor is None
                               or abs(tick_deviation(start[sid], here[sid])) > noise)
                          and tick_deviation(first[sid], here[sid])
                          * tick_deviation(start[sid], here[sid]) <= 0)
                further.append(f'{n} (across the arm)' if across else n)
            elif n in gated and lag_bound_ok(first[sid], here[sid], start[sid], None, noise,
                                             strict=True):
                closer[n] = (abs(tick_deviation(start[sid], here[sid])),
                             abs(tick_deviation(first[sid], here[sid])))
            bounds[sid] = (here[sid], first[sid] if floor is None else start[sid], floor)
        if further:
            return (f'lag-growth gate closed: its first goal change moves {further} further '
                    'from the measured arm', None, None)
        if not closer:
            return ('lag-growth gate closed: its first goal change brings no gated joint '
                    f'({sorted(gated)}) closer to the measured arm', None, None)
        return None, bounds, closer

    def _drift_since(self, job: _ReplanJob, here: Dict[int, int]) -> Optional[str]:
        """A path rising out of the ground margin, as the operator's I lift (_lift_guard):
        refused if the arm moved more than safety_filter.ground_recovery_start_drift_ticks
        on any leader servo since the search started - it sags under load."""
        drift_max = int(self._p('safety_filter.ground_recovery_start_drift_ticks'))
        base = getattr(job, 'measured_ticks', None) or here
        drift = {self._cal.joints[IDS.index(sid)]: here[sid] - base[sid] for sid in LEADER_IDS
                 if abs(here[sid] - base[sid]) > drift_max}
        if drift:
            return (f'ground recovery - the arm moved while the path was searched (ticks '
                    f'since the search started: {drift}, more than {drift_max})')
        return None

    def _reject_setpoint(self, why: str, record: bool = True, verdict=None,
                         note: Optional[str] = None):
        """A rejected leader setpoint: not written; the arm HOLDS where it is and leader
        mode continues; a path to the leader is planned once its pose is valid again
        (user decision 2026-10-07). `record` False: a step of a planned path refused
        only because the measured pose lags the commanded one (the planned step itself
        passes) - its search is not recorded as failed (user decision 2026-10-20).
        `verdict`: the filter's REJECTED verdict, named by the hold status line.
        `note`: replaces the explanation appended to an unrecorded refusal."""
        self._hold_binding = self._binding(verdict) if verdict is not None else why
        if not self._held:
            self._held_since = self._hold_status_t = time.monotonic()
        self._held = True
        self._note('outcome', 'rejected')
        job, self._follow_job = self._follow_job, None
        if self._follow_steps and job is not None:
            if record:
                # A planned path failed in execution: its search is not repeated while
                # its inputs stay the same (_replan_step) - the planner would find it
                # again.
                self._replan_failed = job
                job.why_failed = f'the planned path failed in execution ({why})'
                why += ('; the planned path is recorded as failed - no new search until the '
                        'leader, the held pose or the envelope change')
            elif note is not None:
                why += f'; {note}'
            else:
                why += ('; the planned step itself still passes from the commanded pose - '
                        'the arm lags it: NOT recorded as a failed search, a new one starts '
                        'from the held pose')
        self._follow_steps = []
        self._lag_shrink = None
        self._follow_rising = []
        self._hold_reason = f'setpoint rejected: {why}'
        text = f'leader setpoint REJECTED, holding the last approved one: {why}'
        self.get_logger().warn(text, throttle_duration_sec=1.0)
        self._report(text)
        self._publish_leader_state(f'active (setpoint rejected: {why})')

    def _hold_status(self, now: float, measured):
        """While leader mode holds (a rejected setpoint, a failed or running search, an
        invalid leader target): one status line every leader.hold_status_period_sec -
        since when, why, the measured pose's lowest ground clearance (user decision
        2026-10-20: a hold is never silent; one held 7.5 min without a log line), the
        binding refusal (kind and pair of the last REJECTED verdict) and the largest
        held goal - measured lag in ticks and its counted growth since the arm last
        tracked (_lag_counted, the replan_max_goal_lag_growth_ticks gate)."""
        if now - self._hold_status_t < self._hold_status_period:
            return
        self._hold_status_t = now
        try:
            h = self._filter.ground_heights(measured)
            low = min(h, key=h.get)
            clear = f'; measured lowest ground clearance {low} {h[low] * 1e3:.1f} mm'
        except Exception as exc:     # noqa: BLE001 - a status line never ends leader mode
            clear = f'; ground clearance not computed ({exc!r})'
        binding = f'; binding refusal: {self._hold_binding or "none recorded"}'
        try:
            pos = self._ticks_of(measured)
            lag = self._held_lag(pos)
            if lag:
                n = max(lag, key=lambda k: abs(lag[k]))
                lagged = f'; held goal - measured max {abs(lag[n])} ticks ({n})'
            else:
                lagged = '; held goal - measured max 0 ticks'
            base = self._lag_base
            if base is not None:
                grown = max(self._lag_counted(k, pos[s], lag.get(k, 0))
                            for k, s in zip(self._leader_joints(), LEADER_IDS))
                lagged += (f', grown {grown} by goal travel since the arm last tracked '
                           f'(limit {self._max_lag_growth})')
        except Exception as exc:     # noqa: BLE001
            lagged = f'; goal lag not computed ({exc!r})'
        text = (f'leader: HOLDING for {now - self._held_since:.1f} s - '
                f'{self._hold_reason or "reason unknown"}{clear}{binding}{lagged}')
        self.get_logger().warn(text)
        self._report(text)

    @staticmethod
    def _binding(v) -> str:
        """Kind and pair (or link) of a REJECTED verdict, without the envelope text."""
        where = f' at sub-step {v.index}' if v.index >= 0 else ''
        return f'[{v.rule}] {str(v.detail).split(" (")[0]}{where}'

    def _lag_growth(self, here: Dict[int, int], lag: Dict[str, int],
                    threshold: Optional[int] = None) -> Dict[str, int]:
        """Per joint whose |servo goal - measured| (the setpoint - measured deviation,
        user decision D2 (2026-10-23)) has grown by more than
        leader.replan_max_goal_lag_growth_ticks beyond its reference, by goal travel
        (_lag_counted), WITHOUT the arm following: that growth, ticks. `lag`: servo goal -
        measured per joint (_held_lag). Reference: the smallest deviation seen since the
        joint's baseline, and the goal it was seen at.

        Tracking: where the joint's measured position moved since its baseline by more
        than the deviation's growth plus the limit (the goals moved and the arm went most
        of the way after them), that joint alone is re-baselined at its new position - a
        joint tracking never resets another one that creeps (audit 2026-10-20). A joint
        whose deviation grows by as much as it moved is never re-baselined (audit
        2026-10-20: re-baselining on any motion let an arm sagging 5 ticks per cycle grow
        its lag 8 -> 98 ticks, the 2026-10-12 pattern): its reference stays. Goal travel
        AWAY from such an arm counts against it (_lag_counted); travel toward it does not,
        and this gate does not stop it - that a goal does not run with a sag is the start
        rule's (_held_start). A catch-up moves the travel's origin (_lag_caught_up).

        The baseline is taken at the first leader tick after activation and dropped only
        by the next activation - NOT by a direct leader step written or a path completed
        (audit of D3 (2026-10-23), 2026-10-29: a direct step 17 ticks off a stuck arm
        reset it, the next hold re-baselined at 23 ticks and re-plans ran the goals to 31
        ticks, cycle after cycle). Evaluated at hold ticks only: a transient minimum
        while the arm moves would make the reference meaningless. So the invariant is no
        growth WHILE THE GATE IS CLOSED, not a bound on the deviation: after it reopens,
        one path or a run of direct steps can overshoot until the next hold - without a
        ratchet, since the reference stays and that hold closes the gate again.
        `threshold`: the growth reported (default the limit; _lag_gate passes the lower
        reopening threshold while the gate is closed); tracking is judged by the limit."""
        base, limit = self._lag_base, self._max_lag_growth
        threshold = limit if threshold is None else threshold
        names = self._leader_joints()
        pos = {n: here[s] for n, s in zip(names, LEADER_IDS)}
        dev = {n: abs(lag.get(n, 0)) for n in names}
        if base is None:
            self._lag_base = (pos, dev, {n: pos[n] + lag.get(n, 0) for n in names})
            return {}
        at, ref, goal_at = base
        untracked = {}
        for n in names:
            # the smallest lag seen since: a lag that shrank while the arm settled is the
            # reference, not the transient one the baseline was taken at
            if dev[n] < ref[n]:
                ref[n], goal_at[n] = dev[n], pos[n] + lag.get(n, 0)
            counted = self._lag_counted(n, pos[n], lag.get(n, 0))
            if counted <= threshold:
                continue
            if abs(tick_deviation(pos[n], at[n])) - (dev[n] - ref[n]) > limit:
                # this joint tracked: its own new baseline
                at[n], ref[n], goal_at[n] = pos[n], dev[n], pos[n] + lag.get(n, 0)
            else:
                untracked[n] = counted
        return untracked

    def _lag_counted(self, name: str, pos: int, lag: int) -> int:
        """The growth of one joint's deviation that counts for the lag-growth gate (user
        decision 2026-10-30): the part its GOAL moved away from the arm, measured from
        the goal at the reference (_lag_base) - min(deviation - reference, goal travel
        away from the arm), 0 at least. `pos`: measured ticks, `lag`: servo goal - measured.

        A sag grows the deviation with the goal fixed: not counted, and it cannot lock up
        a joint (a closed gate lets a gated goal only toward the arm, near the ground
        that is refused, 2026-10-20). A goal that moved a tick or two and then sagged
        counts those ticks, not the sag; a goal that moved TOWARD an arm sagging faster
        counts nothing. The travel is a net distance from the reference goal, so travel
        back toward it uncounts itself, and tick_deviation takes the wrap at 0/4095.

        The ratchet stays blocked: it moves goals away from an arm that does not move, and
        for such an arm the growth cannot exceed the travel away (triangle inequality), so
        this is the plain growth, tick for tick. Under reading noise the two differ by up
        to the noise (the travel is exact, the growth is not), so the gate can close a
        noise margin earlier or later than the plain growth would. A sagging arm gains
        nothing: on the side away from the arm its goal stays within the limit of the
        reference goal, whatever the deviation, until the gate closes. Toward the arm
        nothing is limited here."""
        _at, ref, goal_at = self._lag_base
        if (name == self._grip_name() and lag
                and self._grip_squeezing(pos + lag, pos)):
            # FR-17 amendment 2026-11-16: a gripper goal inside an object is the squeeze
            # the leader limit bounds, not a runaway goal - it never closes the gate
            return 0
        away = tick_deviation(pos + lag, goal_at[name])
        return max(0, min(abs(lag) - ref[name], away if lag > 0 else -away))

    def _lag_caught_up(self, here: Dict[int, int]):
        """Every leader tick (user decision 2026-10-30, audit finding C): a joint whose
        deviation is back within the reopening threshold of its reference, and whose arm
        moved more than the limit since `at`, has followed its goal: `at` and the goal
        travel's origin move there, the reference stays. Without this, a goal moved T and
        followed, then a sag S with no hold between, counted as T of travel (a false lock-up
        for T up to 2S + limit). A stuck arm cannot fire it: it does not move. Remaining:
        a sag DURING the move keeps the deviation above the threshold, and still locks."""
        at, ref, goal_at = self._lag_base
        lag = self._held_lag(here)
        reopen = max(0, self._max_lag_growth - self._lag_noise)
        for n, sid in zip(self._leader_joints(), LEADER_IDS):
            d = lag.get(n, 0)
            if (abs(d) - ref[n] <= reopen
                    and abs(tick_deviation(here[sid], at[n])) > self._max_lag_growth):
                at[n], goal_at[n] = here[sid], here[sid] + d

    def _held_lag(self, here: Dict[int, int]) -> Dict[str, int]:
        """Per leader joint whose SERVO goal differs from it: servo goal - measured, ticks -
        what the servos actually hold, not the start of the next re-plan: a gate judged
        by that start went blind exactly when the goals had run away (audit of D3
        (2026-10-23), 2026-10-29). Torque off: none."""
        with self._serial_lock:
            torque, goals = self._torque_on, dict(self._goal)
        if not torque:
            return {}
        return {self._cal.joints[IDS.index(sid)]: tick_deviation(goals[sid], here[sid])
                for sid in LEADER_IDS if goals[sid] is not None and goals[sid] != here[sid]}

    def destroy_node(self):
        # The servos are left exactly as by any exit: no torque-off, no goal write here -
        # except the leader gripper limit's restore (FR-17 amendment 2026-11-16: the
        # gripper's goal := present, then its previous Torque_Limit) - whatever torque and goal they hold stays (a respawned node syncs goal :=
        # present before its torque-on). Only the port is closed, and only if the bus is
        # free within 1 s: a hanging transaction must not hold the exit (2026-10-13).
        if hasattr(self, '_replan'):
            self._drop_replan()
        self._aim_shutdown()
        self._stop_listener_shutdown()
        try:
            if self._serial_lock.acquire(timeout=1.0):
                try:
                    if getattr(self, '_grip_limit_prev', None) is not None:
                        why = self._restore_grip_limit_locked('node shutdown')
                        if why:
                            _stage(why)
                    self._bus.close()
                finally:
                    self._serial_lock.release()
            else:
                _stage('serial lock still held after 1 s (a bus transaction hangs): '
                       'the port is left to the process exit')
        finally:
            super().destroy_node()


def _stage(text: str):
    """Shutdown evidence on stderr (the logger may already be gone)."""
    print(f'arm_action_server shutdown: {text}', file=sys.stderr, flush=True)


def spin_guarded(executor, node: 'ArmActionServer'):
    """executor.spin(), resumed after an exception that escaped a callback.

    rclpy re-raises a callback's exception out of spin(): the node would stop serving
    /arm/stop, the leader watchdog and the pending return home while the servos keep
    whatever torque and goal they had - a clean exit would leave them the same way,
    with nothing left to stop them. So spin() is resumed, the error is logged and
    reported, and nothing is written: the motion paths handle their own errors (hold,
    torque off) - an exception that still escapes comes from a path that moves
    nothing, and a blanket hold here could fight a motion another thread owns.
    More than SPIN_ESCAPE_LIMIT escapes within SPIN_ESCAPE_WINDOW_S: a persistent
    fault, not a stray one - re-raised, so the launch respawn takes over; the node
    cannot serve its callbacks meanwhile anyway (audit r6)."""
    last = -math.inf
    suppressed = 0
    escapes = collections.deque()
    while True:
        try:
            executor.spin()
            return                           # shutdown
        except ExternalShutdownException:
            raise
        except Exception as exc:     # noqa: BLE001
            now = time.monotonic()
            escapes.append(now)
            while escapes and now - escapes[0] > SPIN_ESCAPE_WINDOW_S:
                escapes.popleft()
            if len(escapes) > SPIN_ESCAPE_LIMIT:
                text = (f'UNEXPECTED ERROR escaped a callback {len(escapes)} times within '
                        f'{SPIN_ESCAPE_WINDOW_S:.0f} s - persistent fault, node exits for a '
                        f'respawn: {exc!r}')
                node.get_logger().fatal(text)
                node._shut_down_motion(text)
                node._report(text)
                raise
            time.sleep(SPIN_RESUME_BACKOFF_S)
            if now - last < 1.0:             # a failing timer must not flood the topic
                suppressed += 1
                continue
            last = now
            text = (f'UNEXPECTED ERROR escaped a callback, node keeps running, nothing '
                    f'written{f" ({suppressed} more suppressed)" if suppressed else ""}: '
                    f'{exc!r}')
            suppressed = 0
            node.get_logger().error(f'{text}\n{traceback.format_exc()}')
            node._report(text)


def shutdown_bounded(executor, node, code: int):
    """Bounded process exit (2026-10-13 robot event: SIGINT and SIGTERM ignored, SIGKILL
    after 15 s). The executor's worker threads are non-daemon and Python joins them
    WITHOUT timeout at interpreter exit: one callback that hangs (a bus read) held the
    process. So: executor shutdown (2 s), destroy_node (bus lock at most 1 s), rclpy
    shutdown, then os._exit - no join. If a stage hangs anyway, faulthandler dumps
    every thread's stack to stderr after 4 s. Servo end state: as after a clean exit
    (destroy_node writes nothing)."""
    try:
        faulthandler.dump_traceback_later(4.0, exit=False)
    except Exception:     # noqa: BLE001 - evidence only
        pass
    for name, fn in (('executor shutdown (timeout 2 s)',
                      lambda: executor.shutdown(timeout_sec=2.0)),
                     ('destroy_node', node.destroy_node),
                     ('rclpy shutdown', rclpy.try_shutdown)):
        _stage(name)
        try:
            if fn() is False:
                _stage(f'{name}: callbacks still running - left to the process exit')
        except BaseException as exc:     # noqa: BLE001 - a second Ctrl-C included
            _stage(f'{name} failed: {exc!r}')
    try:
        faulthandler.cancel_dump_traceback_later()
    except Exception:     # noqa: BLE001
        pass
    _stage(f'os._exit({code})')
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


def main(bus=None):
    rclpy.init()
    node = ArmActionServer(bus=bus)
    # Multi-threaded: a running pick must not block its own cancel, /arm/stop, nor
    # the leader watchdog.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    code = 0
    try:
        spin_guarded(executor, node)
        _stage('executor spin returned')
    except KeyboardInterrupt:
        _stage('SIGINT (KeyboardInterrupt) received')
    except ExternalShutdownException:
        _stage('external shutdown (SIGINT/SIGTERM through rclpy)')
    except BaseException as exc:     # noqa: BLE001 - a persistent fault: exit non-zero
        code = 1
        _stage(f'spin ended with {exc!r}\n{traceback.format_exc()}')
    finally:
        shutdown_bounded(executor, node, code)


if __name__ == '__main__':
    main()
