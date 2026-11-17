"""Leader virtual wall (user decisions 2026-10-15/16, progressive wall 2026-10-20),
hardware-free: the real LeaderBus over the register-level scservo_sdk mock of test_align
(with torque on, present follows the goal unless the test holds the servo - the operator's
hand or gravity). The wall edge is the follower's leader command band, read from the
follower's own config; the wall parameters are read from the shipped leader_arm.yaml
(never repeated here). Time is synthetic: step(ticks, now)."""

import math
import os
import random
import shutil
import sys

os.environ.setdefault('ROS_DOMAIN_ID', '227')

import pytest  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_align as ta  # noqa: E402
from test_align import (ARM_YAML, CAL, HERE, IDS, K, NAMES, Q, bus_at,  # noqa: E402,F401
                        node_env, ticks_at, writes_of)
from gripperx_geometry.constants import ARM_JOINT_LOWER, ARM_JOINT_UPPER  # noqa: E402
from gripperx_leader import leader_bus as lb  # noqa: E402
from gripperx_leader.follower_config import command_band  # noqa: E402
from gripperx_leader.wall import (BAND as BY_BAND, BOUNDARY, FREE, HELD, LIMP,  # noqa: E402
                                  OVERLOAD, Edge, LeaderWall, WallConfig, WallFault)

# The follower's band for all six servo ids (the gripper since 2026-11-13), and the
# bands of the ids these fixtures configure (test_align.IDS).
BAND_ALL = command_band(ARM_YAML)
BAND = [BAND_ALL[i - 1] for i in IDS]
ARM_CAL = os.path.join(os.path.dirname(ARM_YAML), 'arm_calibration.yaml')
L2 = 1                                   # index of limb2 (servo id 2)
EDGE = BAND[L2][0]                       # limb2's lower wall (rad)

# The active wall parameters: the shipped node config, not constants repeated here.
LEADER_YAML = os.path.join(HERE, 'config', 'leader_arm.yaml')
P = yaml.safe_load(open(LEADER_YAML))['leader_arm_node']['ros__parameters']
TMAX, TMIN = P['wall_torque_limit'], P['wall_min_torque_limit']
FULL, ENG = P['wall_full_force_depth_rad'], P['wall_engage_depth_rad']
OT, OL, OP = P['wall_overload_time_s'], P['wall_overload_load'], P['wall_overload_progress_rad']
# Parameters of earlier wall designs; wall_release_hysteresis_rad (the Torque_Limit-0 strip
# inside the wall, removed 2026-10-20) included.
REMOVED = ('wall_pushback_velocity_rad_s', 'wall_max_lead_rad', 'wall_rest_tolerance_rad',
           'wall_follow_release_rad', 'wall_max_overshoot_rad', 'wall_release_hysteresis_rad')
INSIDE = 0.01                            # rad inside a wall: back inside, released


def cfg(**over):
    c = dict(torque_limit=TMAX, min_torque_limit=TMIN, full_force_depth_rad=FULL,
             engage_depth_rad=ENG, overload_time_s=OT, overload_load=OL,
             overload_progress_rad=OP, voltage_min_v=P['align_voltage_min_v'],
             voltage_max_v=P['align_voltage_max_v'])
    c.update(over)
    return WallConfig(**c)


def spring_expected(depth, cap=None):
    """The spec: floor at depth 0, linear to the cap at FULL, never below 1; 0 inside."""
    cap = min(TMAX, 1000 if cap is None else cap)
    if depth <= 0:
        return 0
    floor = min(TMIN, cap)
    return max(1, int(round(floor + (cap - floor) * min(depth / FULL, 1.0))))


def _margin():
    doc = yaml.safe_load(open(ARM_YAML))
    return doc['arm_action_server']['ros__parameters']['leader']['limit_command_margin_ticks']


def _follower_tpr():
    doc = yaml.safe_load(open(ARM_CAL))
    return doc['arm_joint_state_relay']['ros__parameters']['ticks_per_rev']


def _q(**over):
    q = list(Q)
    for i, v in over.items():
        q[int(i[1:])] = v
    return q


def _put(sdk, q):
    """The operator holds the leader at q (radians)."""
    for sid, t in ticks_at(q).items():
        sdk.regs[sid][lb.ADDR_PRESENT] = t
        sdk.held.add(sid)


def _let_go(sdk):
    sdk.held.clear()


DT = 1 / 30                              # the node's 30 Hz tick

# Bus transactions (request/reply) per wall action, counted against the real LeaderBus
# code: engage = 5 pre-check reads + the servo's own Torque_Limit + Torque_Limit, goal and
# torque-on writes; release = torque-off write, read-back, Torque_Limit restore; monitor =
# voltage, Torque_Enable, Torque_Limit of one held servo per tick.
ENGAGE_TX, RELEASE_TX, MONITOR_TX = 9, 3, 3

# Gravity model: ticks the joint moves per 30 Hz tick per unit of net force (Torque_Limit
# units). A test model, not a measured servo value.
MOB = 0.005


def make(present=None, **over):
    bus, sdk = bus_at(ticks_at(present or Q))
    w = LeaderWall(bus, CAL, cfg(**over), NAMES, list(BAND))
    w.arm(bus.read_positions())
    assert sdk.writes == []
    return bus, sdk, w


def step(bus, w, t=0.0):
    return w.step(bus.read_positions(), t)


def goals(sdk, sid=None):
    return [v for s, a, v in sdk.writes if a == lb.ADDR_GOAL and (sid is None or s == sid)]


def limits(sdk, sid):
    return [v for s, a, v in sdk.writes if a == lb.ADDR_TORQUE_LIMIT and s == sid]


def toggles(sdk, sid):
    return [v for s, a, v in sdk.writes if a == lb.ADDR_TORQUE and s == sid]


def target_tick(i, edge):
    """The tick of `edge` (rad) on joint i, from the calibration (Q elsewhere)."""
    return ticks_at([edge if j == i else q for j, q in enumerate(Q)])[i + 1]


def depth_of(sdk, i=L2, side=0, edge=None):
    """Depth (rad) of joint i's present reading beyond its band edge (0 inside)."""
    e = Edge('lower' if side == 0 else 'upper', BAND[i][side] if edge is None else edge)
    rad = CAL.to_radians([sdk.regs[s][lb.ADDR_PRESENT] for s in IDS])[i]
    return max(0.0, -e.inward(rad))


def inward_tick_dir(i, side):
    """+1 or -1: the tick direction pointing inward of joint i's band edge `side`."""
    e = BAND[i][side]
    s = 1 if side == 0 else -1
    return 1 if target_tick(i, e + s * 0.01) > target_tick(i, e - s * 0.01) else -1


def rad_at(i, n):
    """Joint i's radians at leader tick n (Q elsewhere), from the calibration."""
    ticks = ticks_at(Q)
    ticks[i + 1] = n
    return CAL.to_radians([ticks[s] for s in IDS])[i]


def hold_tick(i, edge):
    """The spec of the hold goal, by search rather than the wall's formula: the first tick
    outward whose depth beyond `edge`'s spring zero exceeds one tick."""
    out = -inward_tick_dir(i, 0 if edge.side == 'lower' else 1)
    n = target_tick(i, edge.zero) - 3 * out
    while -edge.inside_zero(rad_at(i, n)) * K <= 1.0 + 1e-6:
        n += out
    return n


WALL2 = target_tick(L2, EDGE)                      # limb2's lower wall tick, 109
HOLD2 = hold_tick(L2, Edge('lower', EDGE))         # its hold goal, 2 ticks beyond: 111


def engaged(over=0.005, hold=True, i=L2, side=0, **c):
    """Joint i seen `over` rad beyond its band edge at t=0: engaged. hold = the operator
    keeps it there."""
    bus, sdk, w = make(**c)
    edge = BAND[i][side]
    _put(sdk, _q(**{f'q{i}': edge - over if side == 0 else edge + over}))
    if not hold:
        _let_go(sdk)
    ev = step(bus, w, 0.0)
    assert w.state[i] == HELD and 'engaged' in ev[0], ev
    assert goals(sdk) == [hold_tick(i, Edge('lower' if side == 0 else 'upper', edge))]
    return bus, sdk, w


def run(bus, w, t0, t1, hook=None):
    """Step at 30 Hz from t0 (exclusive) to t1; returns the last t and all events."""
    t, events = t0, []
    while t + DT <= t1 + 1e-9:
        t += DT
        if hook:
            hook(t)
        events += step(bus, w, t)
    return t, events


def count_transactions(bus, sdk):
    """Counts request/reply transactions: every register write (sdk.writes logs each
    attempt) and every single-register read. The sync position read is not counted: it
    runs once per tick whatever the wall does. Returns a function giving the count."""
    reads, start = [0], len(sdk.writes)
    for name in ('read_byte', 'read_word'):
        real = getattr(bus, name)

        def counted(sid, addr, _real=real):
            reads[0] += 1
            return _real(sid, addr)
        setattr(bus, name, counted)
    return lambda: reads[0] + len(sdk.writes) - start


def watch_torque_limit(bus, sdk):
    """After every register write: no servo has torque enabled with a Torque_Limit below
    1 (the STS3215 brakes then - robot 2026-10-20). Returns the violations (stays empty)."""
    bad = []
    for name in ('write1ByteTxRx', 'write2ByteTxRx'):
        real = getattr(bus._pkt, name)

        def checked(port, sid, addr, val, _real=real):
            out = _real(port, sid, addr, val)
            r = sdk.regs.get(sid)
            if r is not None and r[lb.ADDR_TORQUE] and r[lb.ADDR_TORQUE_LIMIT] < 1:
                bad.append((len(sdk.writes), sid, addr, val))
            return out
        setattr(bus._pkt, name, checked)
    return bad


def watch_goals(bus, sdk):
    """Records every Goal_Position write with the present reading at that moment:
    [(sid, goal, present)]."""
    seen = []
    real = bus._pkt.write2ByteTxRx

    def checked(port, sid, addr, val, _real=real):
        if addr == lb.ADDR_GOAL and sid in sdk.regs:
            seen.append((sid, int(val), sdk.regs[sid][lb.ADDR_PRESENT]))
        return _real(port, sid, addr, val)
    bus._pkt.write2ByteTxRx = checked
    return seen


# ------------------------------------------------------------------ config
def test_the_parameters_are_the_users_values_and_the_node_defaults_match_the_yaml():
    """The user's 2026-10-20 values; WallConfig defaults (the node's declared defaults)
    equal the shipped yaml, so a launch without the yaml behaves the same."""
    assert (TMAX, FULL, OT) == (500, 0.1, 2.0)
    d = WallConfig()
    assert (d.torque_limit, d.min_torque_limit, d.full_force_depth_rad,
            d.engage_depth_rad, d.overload_time_s, d.overload_load,
            d.overload_progress_rad) == (TMAX, TMIN, FULL, ENG, OT, OL, OP)
    assert not any(k in P for k in REMOVED)


def test_invalid_config_is_refused():
    bus, _ = bus_at(ticks_at(Q))
    for c in ({'torque_limit': 0}, {'torque_limit': 1001}, {'voltage_min_v': 0.0},
              {'min_torque_limit': -1}, {'min_torque_limit': TMAX + 1},
              {'min_torque_limit': 0},              # torque on without force: brakes
              {'full_force_depth_rad': 0.0}, {'engage_depth_rad': -0.001},
              {'engage_depth_rad': FULL},
              {'engage_depth_rad': 0.0},            # below 2 ticks: noise could toggle
              {'engage_depth_rad': 1.99 / K},
              {'overload_time_s': 0.0}, {'overload_load': 0},
              {'overload_load': TMAX + 1},          # Present_Load cannot exceed the cap
              {'overload_progress_rad': 0.0}, {'boundary_timeout_s': 0.0}):
        with pytest.raises(ValueError):
            LeaderWall(bus, CAL, cfg(**c), NAMES, list(BAND))


# ------------------------------------------------------------------ the spring
def test_inside_the_band_no_force_and_no_writes():
    bus, sdk, w = make()
    rnd = random.Random(1)
    t = 0.0
    for _ in range(200):                                   # moved about inside the band
        q = [rnd.uniform(lo + 0.001, hi - 0.001) for lo, hi in BAND]
        _put(sdk, q)
        t += DT
        assert step(bus, w, t) == []
    assert sdk.writes == [] and set(w.state) == {FREE}


def test_arming_writes_nothing_and_a_joint_beyond_gets_the_spring_from_the_next_tick():
    bus, sdk = bus_at(ticks_at(_q(q1=EDGE - 0.05)))
    w = LeaderWall(bus, CAL, cfg(), NAMES, list(BAND))
    notes = w.arm(bus.read_positions())
    assert sdk.writes == [] and 'limb2' in notes[0] and w.state[L2] == FREE
    ev = step(bus, w, 0.0)
    assert w.state[L2] == HELD and 'engaged' in ev[0]
    tl = spring_expected(depth_of(sdk))
    assert TMIN < tl < TMAX
    # Torque_Limit first, then the goal = the wall tick, then torque on.
    assert sdk.writes == [(2, lb.ADDR_TORQUE_LIMIT, tl), (2, lb.ADDR_GOAL, HOLD2),
                          (2, lb.ADDR_TORQUE, 1)]
    assert (WALL2, HOLD2) == (109, 111) and w.wall_tick == {L2: HOLD2}
    assert w.torque_limit(L2) == tl


@pytest.mark.parametrize('servo_limit', [1000, 200, TMIN // 2])
def test_depth_ramp_is_monotonic_from_the_floor_and_capped(servo_limit):
    """Outward one tick at a time: free inside the engage deadband, then held with
    Torque_Limit = spring(depth) measured from the wall (not from the engage depth),
    rising to the cap at the full-force depth, never above wall_torque_limit nor the
    servo's own value, never below 1."""
    bus, sdk = bus_at(ticks_at(Q))
    sdk.regs[2][lb.ADDR_TORQUE_LIMIT] = servo_limit
    w = LeaderWall(bus, CAL, cfg(), NAMES, list(BAND))
    w.arm(bus.read_positions())
    cap, tls, depths = min(TMAX, servo_limit), [], []
    for k in range(1, int(1.5 * FULL * K)):                # one tick at a time, outward
        _put(sdk, _q(q1=EDGE - k / K))
        step(bus, w, k * DT)
        d = depth_of(sdk)
        if d <= ENG:
            assert w.state[L2] == FREE and sdk.writes == []
            continue
        assert w.state[L2] == HELD
        tls.append(sdk.regs[2][lb.ADDR_TORQUE_LIMIT])
        depths.append(d)
    assert ENG < depths[0] <= ENG + 1.001 / K              # the first tick past the deadband
    assert tls == sorted(tls) and max(tls) == cap and min(tls) >= 1
    assert all(tl == spring_expected(d, servo_limit) for tl, d in zip(tls, depths))
    assert all(tl == cap for tl, d in zip(tls, depths) if d >= FULL)
    if servo_limit >= TMAX:
        assert tls[0] > TMIN                               # the ramp from the wall, not the floor
        assert all(tl < cap for tl, d in zip(tls, depths) if d < FULL - 1 / K)
    assert toggles(sdk, 2) == [1] and goals(sdk, 2) == [HOLD2]   # one engage, one goal
    w.release_all('test')
    assert sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == servo_limit     # restored


@pytest.mark.parametrize('i,side', [(L2, 0), (0, 1), (3, 1), (4, 0)])
def test_force_only_while_the_goal_is_inward_of_the_present_reading(i, side):
    """A random walk around the edge (the hand). Whenever the servo has torque: its
    Torque_Limit is >= 1 and the joint was read beyond the wall. Outward of the hold goal
    the servo pulls inward; between the wall and the hold goal (at most 2 ticks, the
    detent) it pulls outward toward the goal with at most spring(2 ticks). The only goal
    is the hold tick, and no goal is ever written outward of the present reading."""
    bus, sdk, w = make()
    seen = watch_goals(bus, sdk)
    rnd = random.Random(7 + i)
    edge, d_in = BAND[i][side], inward_tick_dir(i, side)
    out = -1.0 if side == 0 else 1.0                       # rad direction pointing outward
    hold = hold_tick(i, Edge('lower' if side == 0 else 'upper', edge))
    detent = 0
    pos, t = edge - out * 0.01, 0.0
    for _ in range(600):
        pos += rnd.uniform(-0.02, 0.02)
        off = max(-0.06, min(0.15, out * (pos - edge)))    # -0.06 inside .. 0.15 beyond
        pos = edge + out * off
        _put(sdk, _q(**{f'q{i}': pos}))
        t += DT
        step(bus, w, t)
        r = sdk.regs[i + 1]
        if r[lb.ADDR_TORQUE]:
            d = depth_of(sdk, i, side)
            assert r[lb.ADDR_TORQUE_LIMIT] >= 1 and d > 0
            if d_in * (r[lb.ADDR_PRESENT] - r[lb.ADDR_GOAL]) > 0:     # inward of the goal
                detent += 1
                assert d * K <= 2.0 + 1e-6
                assert r[lb.ADDR_TORQUE_LIMIT] <= spring_expected(2.0 / K)
    assert set(goals(sdk, i + 1)) == {hold}
    assert all(d_in * (g - p) >= 0 for s, g, p in seen)  # never outward of the reading
    assert len(toggles(sdk, i + 1)) >= 2                   # the walk did cross in and out


def _gravity_run(G, kp, seconds=10.0, noise=0, seed=5):
    """limb2's lower wall, gravity pulling outward (robot 2026-10-20). Overdamped model,
    x = true position in ticks beyond the wall tick (outward +): while torque is on, the
    servo's position loop pulls toward its goal register with kp * (x - goal), capped by
    the Torque_Limit; gravity pulls outward with G; the joint moves (G - force) * MOB
    ticks per tick. The reading is round(x), plus +-`noise` ticks. Returns sdk, wall,
    transaction count, torque-0 violations and the write count per tick."""
    bus, sdk, w = make()
    tx, bad = count_transactions(bus, sdk), watch_torque_limit(bus, sdk)
    out = -inward_tick_dir(L2, 0)
    rnd = random.Random(seed)
    x, t, history = -1.0, 0.0, []
    sdk.held.add(2)                                        # the model moves it, not the mock
    for _ in range(int(seconds / DT)):
        r = sdk.regs[2]
        force = 0.0
        if r[lb.ADDR_TORQUE]:
            tl = r[lb.ADDR_TORQUE_LIMIT]
            force = max(-tl, min(tl, kp * (x - out * (r[lb.ADDR_GOAL] - WALL2))))
        x += max(-3.0, min(3.0, (G - force) * MOB))
        r[lb.ADDR_PRESENT] = WALL2 + out * (int(round(x)) + rnd.randint(-noise, noise))
        r[lb.ADDR_PRESENT_LOAD] = int(min(abs(force), 1000))
        t += DT
        step(bus, w, t)
        history.append(len(sdk.writes))
    return sdk, w, tx(), bad, history


@pytest.mark.parametrize('G,kp', [((TMIN + OL) // 2, 50), (TMIN // 3, 20)])
def test_gravity_pulling_outward_settles_beyond_without_toggling(G, kp):
    """Gravity stronger than the floor (settles where spring(d) = G), or weak against a
    soft position loop (settles where kp * x = G, more than half a tick beyond): either
    way the joint rests beyond the wall - one engage, no release, one goal write. Once
    settled at most a Torque_Limit write per tick (the model's joint may rest on a reading
    boundary and flip between two ticks), never a torque or goal write."""
    sdk, w, _, bad, history = _gravity_run(G, kp)
    assert w.state[L2] == HELD and bad == []
    assert toggles(sdk, 2) == [1] and goals(sdk, 2) == [HOLD2]
    late = sdk.writes[history[-150]:]                       # the last 5 s
    assert all(a == lb.ADDR_TORQUE_LIMIT for s, a, v in late) and len(late) <= 150
    assert len({v for s, a, v in late}) <= 2                # two neighbouring depths at most
    assert 0 < depth_of(sdk) < FULL


@pytest.mark.parametrize('noise', [0, 1])
def test_stiff_servo_resting_on_its_goal_under_weak_gravity_is_never_released(noise):
    """Regression for the rest chatter (risk found 2026-10-20). Gravity below the floor,
    a position loop stiff enough to hold within half a tick of its goal, 30 s, with and
    without +-1 tick reading noise: the joint rests on the hold tick, 2 ticks beyond the
    wall, reading d >= 1 tick even with the noise - one engage, zero releases."""
    sdk, w, _, bad, _ = _gravity_run(TMIN // 3, 200, seconds=30.0, noise=noise)
    assert bad == [] and w.state[L2] == HELD
    assert toggles(sdk, 2) == [1] and goals(sdk, 2) == [HOLD2]
    assert sdk.regs[2][lb.ADDR_GOAL] == HOLD2


def test_the_gravity_model_does_chatter_with_the_goal_at_the_wall_tick(monkeypatch):
    """The model is not blind to the failure the hold tick prevents: with the goal at the
    wall tick (the design before) the same stiff servo and weak gravity cycle - the joint
    rests reading the wall tick, is released, falls past the deadband, engages again.
    Each cycle costs ENGAGE_TX + RELEASE_TX (+ the monitor)."""
    def at_wall(self, i, tick, rad, edge):
        return int(round(tick + self._cal.sign[i] * (edge.zero - rad) * self._k))
    monkeypatch.setattr(LeaderWall, '_goal_tick', at_wall)
    sdk, w, tx, bad, _ = _gravity_run(TMIN // 3, 200, seconds=10.0)
    on = toggles(sdk, 2).count(1)
    assert bad == [] and on >= 5 and toggles(sdk, 2).count(0) >= on - 1
    assert on <= (10.0 / DT) * (TMIN // 3) * MOB / math.floor(ENG * K) + 1


def test_the_engage_event_names_the_hold_tick_not_the_wall_tick():
    """The log is what the wall is calibrated from on the robot: the tick it reports as
    the goal is the hold tick (2 ticks beyond), never labelled as the wall tick."""
    bus, sdk, w = make()
    _put(sdk, _q(q1=EDGE - 0.02))
    ev = step(bus, w, 0.0)
    assert f'hold tick {HOLD2}' in ev[0] and 'wall tick' not in ev[0], ev


def test_the_hold_goal_is_never_outward_of_the_present_reading():
    """With the shipped deadband (> 2 ticks) the reading at engage is always outward of the
    hold tick (2 ticks beyond). With a smaller deadband (0: engage at the first tick
    beyond) the goal is clamped to the present reading - never outward of it."""
    out = -inward_tick_dir(L2, 0)
    assert ENG * K > 2.0                                    # shipped: the clamp never acts
    for ticks_beyond, goal in ((1, WALL2 + out), (2, HOLD2), (3, HOLD2), (9, HOLD2)):
        bus, sdk, w = make()
        w._cfg.engage_depth_rad = 0.0      # test only: startup refuses < 2 ticks (LOW-3)
        sdk.held.add(2)
        sdk.regs[2][lb.ADDR_PRESENT] = WALL2 + out * ticks_beyond
        step(bus, w, 0.0)
        assert w.state[L2] == HELD and goals(sdk, 2) == [goal], ticks_beyond


@pytest.mark.parametrize('frac', [0.0, 0.25, 0.5, 0.99])
@pytest.mark.parametrize('center', [-2, -1, 0, 1, 2, 3, 4])
def test_one_tick_reading_noise_at_the_edge_never_toggles_torque(frac, center):
    """A joint whose reading jitters by +-1 tick (gravity drift, dither) around a reading
    `center` ticks outward of the wall tick, with the wall `frac` of a tick outward of that
    tick (the shipped edges lie on ticks: frac 0). Engage needs a reading beyond by more
    than wall_engage_depth_rad, release one at or inside the wall: with the shipped value
    (>= 2 ticks) more than 2 ticks apart, so the noise engages at most once and never
    releases - around the wall, around the engage depth and in between."""
    assert ENG * K >= 2.0                                   # the property needs >= 2 ticks
    out = -inward_tick_dir(L2, 0)
    bands = list(BAND)
    bands[L2] = (EDGE - frac / K, BAND[L2][1])              # lower wall, frac outward
    bus, sdk = bus_at(ticks_at(Q))
    w = LeaderWall(bus, CAL, cfg(), NAMES, bands)
    w.arm(bus.read_positions())
    tx = count_transactions(bus, sdk)
    rnd = random.Random(1000 * frac + center)
    sdk.held.add(2)
    t, beyond = 0.0, []
    for _ in range(900):                                    # 30 s
        c = center + rnd.randint(-1, 1)
        sdk.regs[2][lb.ADDR_PRESENT] = WALL2 + out * c
        t += DT
        step(bus, w, t)
        beyond.append((c - frac) / K)
    engages = 1 if max(beyond) > ENG else 0
    assert toggles(sdk, 2) == [1] * engages                 # never a torque-off
    assert all(x[0] == 2 for x in sdk.writes)
    if not engages:
        assert tx() == 0                                    # not a single transaction


def test_robot_case_overshoot_and_fall_back_toggles_once_per_crossing():
    """Replay of the 2026-10-20 log of the 2026-10-16 design: limb2 seen 0.003-0.018 rad
    beyond, the servo overshooting 7-17 ticks inward past the wall tick, gravity pulling it
    out again - 75 times. Now torque goes off on every inward overshoot (back inside) and
    on again only when the reading is beyond by more than the deadband: at most one engage
    and one release per crossing, each a fixed number of bus transactions."""
    d_in = inward_tick_dir(L2, 0)
    bus, sdk, w = make()
    tx, bad = count_transactions(bus, sdk), watch_torque_limit(bus, sdk)
    rnd = random.Random(3)
    t, crossings = 0.0, 0
    for _ in range(75):
        _put(sdk, _q(q1=EDGE - rnd.uniform(0.003, 0.018)))
        n0 = tx()
        t += DT
        step(bus, w, t)
        if depth_of(sdk) > ENG:
            crossings += 1
            assert w.state[L2] == HELD and tx() - n0 == ENGAGE_TX
            assert sdk.regs[2][lb.ADDR_TORQUE_LIMIT] >= min(TMIN, TMAX)
        else:                                               # within the deadband: nothing
            assert w.state[L2] == FREE and tx() == n0
        was = w.state[L2]
        sdk.regs[2][lb.ADDR_PRESENT] = WALL2 + d_in * rnd.randint(7, 17)
        n0 = tx()
        t += DT
        step(bus, w, t)
        assert w.state[L2] == FREE and sdk.regs[2][lb.ADDR_TORQUE] == 0
        assert tx() - n0 == (MONITOR_TX + RELEASE_TX if was == HELD else 0)
    assert 0 < crossings <= 75 and bad == []
    assert toggles(sdk, 2) == [1, 0] * crossings and goals(sdk, 2) == [HOLD2] * crossings


def test_no_register_writes_while_nothing_changes():
    """A held joint at one depth costs a Present_Load read and the monitor per tick, no
    write; let go, the servo brings it to the hold tick 2 ticks beyond the wall and keeps
    holding it there: one Torque_Limit write, then none."""
    bus, sdk, w = engaged(over=0.05)
    tx = count_transactions(bus, sdk)
    n = len(sdk.writes)
    t, _ = run(bus, w, 0.0, 1.0)                           # held at one depth
    assert sdk.writes[n:] == [] and tx() == 30 * (1 + MONITOR_TX)
    _let_go(sdk)                                           # the servo reaches the hold tick
    t, ev = run(bus, w, t, t + 1.0)
    assert sdk.regs[2][lb.ADDR_PRESENT] == HOLD2 and w.state[L2] == HELD
    assert sdk.writes[n:] == [(2, lb.ADDR_TORQUE_LIMIT, spring_expected(depth_of(sdk)))]
    assert 1.0 < depth_of(sdk) * K <= 2.0 + 1e-6


def test_back_inside_releases_at_once_and_only_beyond_the_deadband_engages():
    """User decision 2026-10-20: torque off as soon as the joint is back inside (depth <=
    0) - never torque on with Torque_Limit 0; engage only beyond by more than
    wall_engage_depth_rad. A held joint inside the deadband keeps its spring."""
    bus, sdk, w = engaged(over=0.02)
    t = DT
    _put(sdk, _q(q1=EDGE - ENG / 2))                       # still beyond, within the deadband
    step(bus, w, t)
    assert w.state[L2] == HELD and sdk.regs[2][lb.ADDR_TORQUE] == 1
    assert sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == spring_expected(depth_of(sdk)) >= 1
    _put(sdk, _q(q1=EDGE))                                 # at the wall tick: inside
    t += DT
    ev = step(bus, w, t)
    assert w.state[L2] == FREE and 'released' in ev[0]
    assert sdk.writes[-2:] == [(2, lb.ADDR_TORQUE, 0), (2, lb.ADDR_TORQUE_LIMIT, 1000)]
    n = len(sdk.writes)
    for q1 in (EDGE - ENG / 2, EDGE - ENG + 0.0001, EDGE + 0.003):   # deadband, inside
        _put(sdk, _q(q1=q1))
        t += DT
        step(bus, w, t)
        assert w.state[L2] == FREE
    assert sdk.writes[n:] == []
    _put(sdk, _q(q1=EDGE - ENG - 1.0 / K))                 # past the deadband: engages
    t += DT
    step(bus, w, t)
    assert w.state[L2] == HELD and toggles(sdk, 2) == [1, 0, 1]


def test_a_joint_falling_beyond_gets_force_at_once_no_still_condition():
    """Robot 2026-10-20: limb4 fell 0.074 rad beyond before the old wall re-engaged.
    Now the force is there the first tick past the deadband and grows every tick it falls."""
    bus, sdk, w = make()
    edge = BAND[3][1]
    pos, t, tls = edge - 0.01, 0.0, []
    for _ in range(30):                                    # falling 0.006 rad per tick
        pos += 0.006
        _put(sdk, _q(q3=pos))
        t += DT
        step(bus, w, t)
        if depth_of(sdk, 3, 1) > ENG:
            assert w.state[3] == HELD
            tls.append(sdk.regs[4][lb.ADDR_TORQUE_LIMIT])
    assert tls[0] >= TMIN and tls == sorted(tls) and tls[-1] == TMAX
    assert toggles(sdk, 4) == [1]


# ------------------------------------------------------------------ overload release
def _hand(sdk, sid, load):
    sdk.regs[sid][lb.ADDR_PRESENT_LOAD] = load


@pytest.mark.parametrize('load', [TMAX, TMAX | 0x400])
def test_held_deep_without_progress_is_released_after_the_overload_time(load):
    """load with or without the direction bit 10. The latch re-arms back inside (depth
    <= 0, the release criterion); the deadband then keeps it from re-engaging at once."""
    bus, sdk, w = engaged(over=0.3)
    _hand(sdk, 2, load)
    t, ev = run(bus, w, 0.0, OT)
    assert w.state[L2] == HELD and not [e for e in ev if OVERLOAD in e]
    t, ev = run(bus, w, t, OT + 0.2)
    assert w.state[L2] == LIMP and OVERLOAD in ev[-1], ev
    assert sdk.regs[2][lb.ADDR_TORQUE] == 0 and sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == 1000
    assert w.outside() == ['limb2 lower wall (overload release)']
    n = len(sdk.writes)
    for q1 in (EDGE - 0.3, EDGE - 0.01, EDGE - ENG / 2, EDGE - 0.1):   # beyond: stays limp
        _put(sdk, _q(q1=q1))
        t += DT
        step(bus, w, t)
    assert sdk.writes[n:] == [] and w.state[L2] == LIMP
    _put(sdk, _q(q1=EDGE + 0.001))                          # back inside: re-armed
    t += DT
    ev = step(bus, w, t)
    assert w.state[L2] == FREE and 'active again' in ev[0] and sdk.writes[n:] == []
    _put(sdk, _q(q1=EDGE - ENG / 2))                        # within the deadband: free
    t += DT
    step(bus, w, t)
    assert w.state[L2] == FREE and sdk.writes[n:] == []
    _put(sdk, _q(q1=EDGE - 0.01))
    step(bus, w, t + DT)
    assert w.state[L2] == HELD


def test_overload_fires_only_where_the_spring_reaches_the_load_threshold():
    """Present_Load is capped by the Torque_Limit (model): a hand or gravity holding the
    joint shallow, where spring(d) < wall_overload_load, is never timed out; deeper, it
    is. The threshold depth follows from the yaml values."""
    d_star = (OL - TMIN) / (TMAX - TMIN) * FULL
    for over, released in ((d_star - 0.01, False), (d_star + 0.01, True)):
        bus, sdk, w = engaged(over=over)

        def capped(t):
            _hand(sdk, 2, min(sdk.regs[2][lb.ADDR_TORQUE_LIMIT], 900))
        _, ev = run(bus, w, 0.0, 3 * OT, hook=capped)
        assert (w.state[L2] == LIMP) is released, (over, ev)
        assert any('depth' in e and 'Torque_Limit' in e and 'load' in e for e in ev)


def test_a_low_load_is_never_released():
    bus, sdk, w = engaged(over=0.3)
    _hand(sdk, 2, OL - 1)
    run(bus, w, 0.0, 3 * OT)
    assert w.state[L2] == HELD


def test_a_yielding_hand_is_not_overload_released():
    bus, sdk, w = engaged(over=0.3)
    _hand(sdk, 2, TMAX)
    pos = [EDGE - 0.3]
    rate = 1.2 * OP / OT                                    # just faster than the criterion

    def yield_slowly(t):
        pos[0] += rate * DT
        _put(sdk, _q(q1=pos[0]))
    run(bus, w, 0.0, 3 * OT, hook=yield_slowly)
    assert w.state[L2] == HELD


# ------------------------------------------------------------------ disarm / faults
def test_disarm_makes_every_servo_limp():
    bus, sdk, w = engaged(over=0.3)
    _put(sdk, _q(q0=BAND[0][1] + 0.2, q1=EDGE - 0.3))       # a second joint held too
    run(bus, w, 0.0, 0.3)
    assert w.state[0] == HELD and w.state[L2] == HELD
    text = w.release_all('Space (/arm/stop)')
    assert text.endswith('torque off confirmed') and not w.armed
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS)
    assert sdk.regs[1][lb.ADDR_TORQUE_LIMIT] == 1000 == sdk.regs[2][lb.ADDR_TORQUE_LIMIT]
    n = len(sdk.writes)
    assert step(bus, w, 1.0) == [] and sdk.writes[n:] == []


@pytest.mark.parametrize('reg,val,match', [
    (lb.ADDR_PRESENT_VOLTAGE, 50, 'outside the band'),
    (lb.ADDR_PRESENT_VOLTAGE, 90, 'outside the band'),
    (lb.ADDR_OPERATING_MODE, 3, 'Operating_Mode'),
    (lb.ADDR_TORQUE, 1, 'torque already ON'),
    (lb.ADDR_MIN_POS_LIMIT, 4000, 'position limits'),
])
def test_engage_preconditions_leave_it_limp(reg, val, match):
    bus, sdk, w = make()
    sdk.regs[2][reg] = val
    _put(sdk, _q(q1=EDGE - 0.005))
    ev = step(bus, w)
    assert w.state[L2] == LIMP and match in ev[0] and 'NOT engaged' in ev[0]
    assert goals(sdk) == [] and writes_of(sdk, lb.ADDR_TORQUE, 1) == []
    assert w.outside() == ['limb2 lower wall (not engaged)']


@pytest.mark.parametrize('fault', ['voltage', 'torque_reset', 'limit_reset', 'status',
                                   'read_error'])
def test_monitor_fault_releases_every_servo(fault):
    bus, sdk, w = engaged()
    sdk.regs[3][lb.ADDR_TORQUE] = 1
    if fault == 'voltage':
        sdk.regs[2][lb.ADDR_PRESENT_VOLTAGE] = 60
    elif fault == 'torque_reset':
        sdk.regs[2][lb.ADDR_TORQUE] = 0
    elif fault == 'limit_reset':
        sdk.regs[2][lb.ADDR_TORQUE_LIMIT] = 1000
    elif fault == 'status':
        sdk.status[2] = 0x20
    ticks = bus.read_positions()
    if fault == 'read_error':
        sdk.fail_read = True
    with pytest.raises(WallFault):
        w.step(ticks, DT)
    sdk.fail_read = False
    assert not w.armed
    if fault == 'read_error':                  # read-back impossible: pending, retried
        assert w.unconfirmed == list(IDS)
        assert w.retry_release() == []
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS)
    assert w.unconfirmed == []


def test_torque_limit_write_error_while_holding_releases_everything():
    bus, sdk, w = engaged(over=0.05)
    sdk.fail_write.add((2, lb.ADDR_TORQUE_LIMIT))
    _put(sdk, _q(q1=EDGE - 0.08))                          # deeper: a new Torque_Limit
    with pytest.raises(WallFault, match='Torque_Limit write failed'):
        step(bus, w, DT)
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS) and not w.armed


def test_unconfirmed_single_release_becomes_a_full_release_retried():
    bus, sdk, w = engaged()
    sdk.torque_stuck.add(2)
    _put(sdk, _q(q1=EDGE + INSIDE))
    with pytest.raises(WallFault, match=r'NOT confirmed on servo\(s\) \[2\]'):
        step(bus, w, DT)
    assert w.unconfirmed == [2] and not w.armed
    assert w.retry_release() == [2]
    sdk.torque_stuck.clear()
    assert w.retry_release() == [] and sdk.regs[2][lb.ADDR_TORQUE] == 0
    assert sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == 1000


def test_write_error_while_engaging_releases_everything():
    bus, sdk, w = make()
    sdk.fail_write.add((2, lb.ADDR_GOAL))
    _put(sdk, _q(q1=EDGE - 0.005))
    with pytest.raises(WallFault, match='engaging failed'):
        step(bus, w)
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS) and not w.armed


def test_joint_without_a_band_never_gets_a_wall():
    bus, sdk = bus_at(ticks_at(Q))
    bands = list(BAND)
    bands[L2] = None
    w = LeaderWall(bus, CAL, cfg(), NAMES, bands)
    w.arm(bus.read_positions())
    _put(sdk, _q(q1=EDGE - 0.005))
    step(bus, w)
    assert sdk.writes == []


def test_a_present_load_read_error_is_a_fault_to_all_limp():
    bus, sdk, w = engaged(over=0.3)
    real = bus.read_word

    def failing(sid, addr):
        if addr == lb.ADDR_PRESENT_LOAD:
            raise lb.LeaderReadError('servo 2: read of address 60 failed')
        return real(sid, addr)
    bus.read_word = failing
    with pytest.raises(WallFault, match='Present_Load read failed'):
        step(bus, w, DT)
    assert not w.armed and all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS)


def test_release_all_restores_the_torque_limit_only_after_torque_off():
    bus, sdk, w = engaged(over=0.3)
    n = len(sdk.writes)
    w.release_all('Space (/arm/stop)')
    after = sdk.writes[n:]
    offs = [k for k, (s, a, v) in enumerate(after) if a == lb.ADDR_TORQUE and v == 0]
    limit = [k for k, (s, a, v) in enumerate(after) if a == lb.ADDR_TORQUE_LIMIT]
    assert set(s for s, a, v in after if a == lb.ADDR_TORQUE) == set(IDS)
    assert limit == [len(after) - 1] and max(offs) < limit[0]
    assert after[-1] == (2, lb.ADDR_TORQUE_LIMIT, 1000)


def test_a_failed_torque_limit_restore_keeps_the_original_for_the_next_engage():
    """Audit 2026-10-20 (PW1): after torque-off the register still holds the wall's own
    spring value (as low as the floor). If the restore write fails, the next engage must
    not read that value as the servo's own limit (a silently weak wall) - the original is
    kept, used and restored at the next release."""
    bus, sdk, w = engaged(over=0.005)
    low = sdk.regs[2][lb.ADDR_TORQUE_LIMIT]
    assert 1 <= low < spring_expected(0.08)
    sdk.fail_write.add((2, lb.ADDR_TORQUE_LIMIT))
    _put(sdk, _q(q1=EDGE + INSIDE))
    ev = step(bus, w, DT)
    assert w.state[L2] == FREE and any('restore failed' in e for e in ev), ev
    assert sdk.regs[2][lb.ADDR_TORQUE] == 0 and sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == low
    sdk.fail_write.clear()
    _put(sdk, _q(q1=EDGE - 0.08))
    step(bus, w, 2 * DT)
    assert w.state[L2] == HELD
    assert sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == spring_expected(depth_of(sdk)) > low
    assert 'restore failed' not in w.release_all('Space (/arm/stop)')
    assert sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == 1000


def test_a_servo_with_torque_limit_zero_is_not_engaged():
    bus, sdk, w = make()
    sdk.regs[2][lb.ADDR_TORQUE_LIMIT] = 0
    _put(sdk, _q(q1=EDGE - 0.05))
    ev = step(bus, w)
    assert w.state[L2] == LIMP and 'Torque_Limit 0' in ev[0] and sdk.writes == []


@pytest.mark.parametrize('seed', range(6))
def test_no_code_path_leaves_a_servo_torqued_with_torque_limit_zero(seed):
    """Fuzz every path: the five limb joints wander across both edges (held by the hand or
    let go to their goal), random loads (overload releases), a boundary set, moved, gone
    stale or cleared, write and read faults (full releases, then re-armed), failed
    Torque_Limit restores. After every register write no servo has torque on with a
    Torque_Limit below 1, and after every step a torqued servo is one the wall holds."""
    rnd = random.Random(seed)
    bus, sdk, w = make()
    bad, seen = watch_torque_limit(bus, sdk), watch_goals(bus, sdk)
    pos, t, seq, faults, checked = list(Q), 0.0, 0, 0, 0
    for _ in range(3000):
        t += DT
        for i in range(5):
            lo, hi = BAND[i]
            pos[i] = min(hi + 0.15, max(lo - 0.15, pos[i] + rnd.uniform(-0.02, 0.02)))
        _put(sdk, pos)
        if rnd.random() < 0.3:
            _let_go(sdk)
        for s in IDS:
            sdk.regs[s][lb.ADDR_PRESENT_LOAD] = rnd.choice([0, OL - 1, OL, TMAX])
        roll = rnd.random()
        if roll < 0.05:
            seq += 1
            j = rnd.randrange(5)
            e = Edge(rnd.choice(['lower', 'upper']), pos[j] + rnd.uniform(-0.1, 0.1),
                     rnd.uniform(0.0, 0.03))
            w.set_boundary(_boundary(**{f'q{j}': e}), t, 's', seq)
        elif roll < 0.07:
            w.clear_boundary()
        sdk.fail_write.clear()
        if rnd.random() < 0.01:
            sdk.fail_write.add((rnd.choice(IDS[:5]), rnd.choice(
                [lb.ADDR_TORQUE_LIMIT, lb.ADDR_GOAL, lb.ADDR_TORQUE])))
        sdk.fail_read = rnd.random() < 0.003
        try:
            w.step(bus.read_positions(), t)
        except (WallFault, lb.LeaderReadError):
            faults += 1
            sdk.fail_read = False
            sdk.fail_write.clear()
            if w.armed:
                w.release_all('read error')
            assert w.retry_release() == []
            w.arm(bus.read_positions())
        for i, s in enumerate(IDS[:5]):
            r = sdk.regs[s]
            if r[lb.ADDR_TORQUE]:
                assert r[lb.ADDR_TORQUE_LIMIT] >= 1 and w.state[i] == HELD
        for sid, g, p in seen:                 # goals written this step, never outward
            j = w._j[sid - 1]
            if j.state == HELD and j.goal == g:
                out = CAL.sign[sid - 1] * (-1 if j.edge.side == 'lower' else 1)
                assert out * (g - p) <= 0
                checked += 1
        seen.clear()
    assert bad == [] and checked > 50
    assert faults > 0 and len(writes_of(sdk, lb.ADDR_TORQUE, 1)) > 50
    assert len(writes_of(sdk, lb.ADDR_TORQUE, 0)) > 50


def test_spring_never_returns_zero_beyond_the_wall():
    """Even with the smallest floor and cap and a depth a hair beyond, the spring is >= 1:
    a held servo is never given Torque_Limit 0."""
    bus, _ = bus_at(ticks_at(Q))
    w = LeaderWall(bus, CAL, cfg(min_torque_limit=1, torque_limit=1, overload_load=1),
                   NAMES, list(BAND))
    for cap in (1, 2, 1000):
        for d in (1e-8, 1e-6, ENG, FULL, 10 * FULL):
            assert w.spring(cap, d) >= 1
    assert w.spring(1000, 0.0) == 0 and w.spring(1000, -0.1) == 0


def test_dead_parameters_are_gone_everywhere():
    """wall_release_hysteresis_rad (the Torque_Limit-0 strip, removed 2026-10-20) and the
    earlier designs' parameters: not in the shipped yaml, not a WallConfig field, not named
    in the node or the wall source."""
    import dataclasses
    from gripperx_leader import leader_arm_node, wall
    assert 'release_hysteresis_rad' not in {f.name for f in dataclasses.fields(WallConfig)}
    assert not any(k in P for k in REMOVED)
    for mod in (leader_arm_node, wall):
        text = open(mod.__file__).read()
        assert not any(k in text for k in REMOVED) and 'release_hysteresis' not in text


# ------------------------------------------------------------------ limb2 and the wrap
def test_limb2_lower_wall_goal_never_crosses_the_wrap():
    """limb2 wraps 0/4095 109 ticks inward of its lower wall tick (109). Beyond the wall
    (ticks 110.., mechanical stop about 18 ticks beyond) the only goal is the hold tick
    111; the servo brings the joint there and holds it, on the wall's side."""
    bus, sdk, w = engaged(over=0.028, hold=False)          # about 18 ticks beyond
    run(bus, w, 0.0, 0.5)
    assert goals(sdk) == [HOLD2] and sdk.regs[2][lb.ADDR_PRESENT] == HOLD2
    assert w.state[L2] == HELD and sdk.regs[2][lb.ADDR_TORQUE] == 1


def test_limb2_yanked_across_the_wrap_is_released_without_a_goal_write():
    bus, sdk, w = engaged(over=0.02)
    run(bus, w, 0.0, 0.1)
    _put(sdk, _q(q1=EDGE + 0.25))                           # inward, across 0/4095
    assert sdk.regs[2][lb.ADDR_PRESENT] > 3900
    ev = step(bus, w, 0.2)
    assert w.state[L2] == FREE and 'released' in ev[0] and goals(sdk) == [HOLD2]


def test_wall_tick_too_close_to_the_position_limit_is_not_engaged():
    """Present and goal (hold) tick inside Min/Max_Position_Limit; no extra guard."""
    bus, sdk, w = make()
    _put(sdk, _q(q1=EDGE - 0.005))
    sdk.regs[2][lb.ADDR_MIN_POS_LIMIT] = HOLD2 + 1
    ev = step(bus, w)
    assert w.state[L2] == LIMP and \
        f'goal tick {HOLD2} outside the servo position limits' in ev[0]
    assert sdk.writes == []
    bus, sdk, w = make()
    _put(sdk, _q(q1=EDGE - 0.005))
    sdk.regs[2][lb.ADDR_MIN_POS_LIMIT] = HOLD2
    step(bus, w)
    assert w.state[L2] == HELD and goals(sdk) == [HOLD2]


def test_robot_limb2_with_its_recorded_eeprom_limits_gets_no_lower_wall():
    """Leader limb2 read Min/Max_Position_Limit 823/3199 on 2026-10-13 (README) and wraps
    0/4095 past its bottom stop: present and wall tick lie outside those limits. Limp."""
    bus, sdk, w = make()
    sdk.regs[2][lb.ADDR_MIN_POS_LIMIT], sdk.regs[2][lb.ADDR_MAX_POS_LIMIT] = 823, 3199
    _put(sdk, _q(q1=EDGE - 0.005))
    assert sdk.regs[2][lb.ADDR_PRESENT] < 823
    ev = step(bus, w)
    assert w.state[L2] == LIMP and 'position limits 823..3199' in ev[0]
    assert sdk.writes == []


def test_limb2_reading_continuously_past_4095_is_not_engaged():
    """Audit LW1 L5 re-check: limits 0..4095. If limb2 read continuously past 4095, its
    present (and wall tick, about 4205) lie outside 0..4095 -> refused, stays limp."""
    bus, sdk, w = make()
    _put(sdk, _q(q1=EDGE - 0.005))
    sdk.regs[2][lb.ADDR_PRESENT] += 4096
    ev = step(bus, w)
    assert w.state[L2] == LIMP and 'position limits 0..4095' in ev[0] and sdk.writes == []


# ------------------------------------------------------------------ second source: boundary
L3 = 2                                                    # limb3, index 2


def _boundary(**edges):
    """Per-joint Edge or None, keyed q<i>."""
    out = [None] * len(IDS)
    for k, e in edges.items():
        out[int(k[1:])] = e
    return out


def test_boundary_spring_starts_at_its_tolerance_line():
    """The tolerance is free travel: the spring zero (depth 0) is the held pose moved
    outward by it; the goal is the hold tick beyond that zero."""
    bus, sdk, w = make()
    held, tol = Q[L3], 0.02
    edge = Edge('upper', held, tol)
    assert w.set_boundary(_boundary(q2=edge), 0.0, 's1', 1)
    _put(sdk, _q(q2=held + 0.015))                       # within the tolerance
    step(bus, w, 0.0)
    assert sdk.writes == [] and w.state[L3] == FREE
    _put(sdk, _q(q2=held + 0.1))
    ev = step(bus, w, DT)
    assert w.state[L3] == HELD and w.owner(L3) == BOUNDARY and 'upper boundary' in ev[0]
    hold = hold_tick(L3, edge)
    assert goals(sdk, 3) == [hold]
    d = depth_of(sdk, L3, 1, edge=held + tol)
    assert sdk.regs[3][lb.ADDR_TORQUE_LIMIT] == spring_expected(d)
    _let_go(sdk)
    for k in range(2, 30):
        w.set_boundary(_boundary(q2=edge), k * DT, 's1', k)
        step(bus, w, k * DT)
    d = depth_of(sdk, L3, 1, edge=held + tol)
    assert sdk.regs[3][lb.ADDR_PRESENT] == hold and 1.0 < d * K <= 2.0 + 1e-6
    assert w.state[L3] == HELD and w.torque_limit(L3) == spring_expected(d)
    assert w.held() == [f'limb3 upper boundary (spring, tick {hold})']


def test_band_and_boundary_one_owner_the_band_wins():
    lo = BAND[0][0]                                      # limb1 lower wall, tick 508
    bus, sdk, w = make()
    b = _boundary(q0=Edge('lower', lo + 0.2, 0.0))
    w.set_boundary(b, 0.0, 's1', 1)
    _put(sdk, _q(q0=lo + 0.1))                           # inside the band, beyond boundary
    step(bus, w, 0.0)
    assert w.owner(0) == BOUNDARY and w.wall_tick[0] == hold_tick(0, Edge('lower', lo + 0.2))
    w.set_boundary(b, DT, 's1', 2)
    _put(sdk, _q(q0=lo - 0.01))                          # now beyond the band as well
    ev = step(bus, w, DT)
    h1 = hold_tick(0, Edge('lower', lo))
    assert w.owner(0) == BY_BAND and w.wall_tick[0] == h1 and 'now held by the lower wall' in ev[0]
    assert goals(sdk, 1)[-1] == h1
    assert sdk.regs[1][lb.ADDR_TORQUE_LIMIT] == spring_expected(depth_of(sdk, 0, 0))
    assert len(writes_of(sdk, lb.ADDR_TORQUE, 1)) == 1   # one engage, one owner
    _put(sdk, _q(q0=lo + 0.1))                           # back inside the band: band releases
    ev = step(bus, w, 2 * DT)
    assert w.state[0] == FREE and 'released' in ev[0]
    # a joint FREE and beyond both engages with the band directly
    bus, sdk, w = make()
    w.set_boundary(b, 0.0, 's1', 1)
    _put(sdk, _q(q0=lo - 0.01))
    step(bus, w, 0.0)
    assert w.owner(0) == BY_BAND and w.wall_tick[0] == hold_tick(0, Edge('lower', lo))


def test_a_boundary_moved_past_the_joint_releases_it():
    bus, sdk, w = make()
    held = Q[L3]
    w.set_boundary(_boundary(q2=Edge('upper', held, 0.0)), 0.0, 's1', 1)
    _put(sdk, _q(q2=held + 0.05))
    step(bus, w, 0.0)
    assert w.state[L3] == HELD
    w.set_boundary(_boundary(q2=Edge('upper', held + 0.1, 0.0)), DT, 's1', 2)
    ev = step(bus, w, DT)
    assert w.state[L3] == FREE and 'moved past the joint' in ev[0]
    assert goals(sdk, 3) == [hold_tick(L3, Edge('upper', held))]   # never one outward


def test_boundary_across_the_limb2_wrap_is_not_engaged():
    """A boundary target past limb2's 0/4095 wrap (inward of the lower wall by more than
    109 ticks, the joint on the other side) is refused by the same limits guard."""
    bus, sdk, w = make()
    w.set_boundary(_boundary(q1=Edge('lower', EDGE + 0.2, 0.0)), 0.0, 's1', 1)
    _put(sdk, _q(q1=EDGE + 0.1))
    ev = step(bus, w, 0.0)
    assert w.state[L2] == LIMP and 'position limits' in ev[0] and sdk.writes == []


def test_stale_boundary_releases_only_boundary_held_joints():
    bus, sdk, w = make()
    held = Q[L3]
    w.set_boundary(_boundary(q2=Edge('upper', held, 0.0)), 0.0, 's1', 1)
    _put(sdk, _q(q1=EDGE - 0.01, q2=held + 0.1))          # limb2 band, limb3 boundary
    step(bus, w, 0.0)
    assert w.owner(L2) == BY_BAND and w.owner(L3) == BOUNDARY
    assert not w.set_boundary(_boundary(q2=Edge('upper', held, 0.0)), 0.3, 's1', 1)  # old seq
    step(bus, w, 0.3)
    assert w.state[L3] == HELD                           # 0.3 s < 0.5 s timeout
    ev = step(bus, w, 0.6)
    assert any('stale' in e for e in ev) and any('limb3: boundary gone, released' in e
                                                 for e in ev)
    assert w.state[L3] == FREE and sdk.regs[3][lb.ADDR_TORQUE] == 0
    assert w.state[L2] == HELD and sdk.regs[2][lb.ADDR_TORQUE] == 1
    # a new session replaces the boundary whatever its seq
    assert w.set_boundary(_boundary(q2=Edge('upper', held, 0.0)), 0.7, 's2', 0)
    step(bus, w, 0.7)
    assert w.owner(L3) == BOUNDARY and w.state[L3] == HELD


def test_cleared_boundary_releases_and_a_boundary_overload_latch_ends_with_it():
    bus, sdk, w = make()
    held = Q[L3]
    w.set_boundary(_boundary(q2=Edge('upper', held, 0.0)), 0.0, 's1', 1)
    _put(sdk, _q(q2=held + 0.2))
    step(bus, w, 0.0)
    _hand(sdk, 3, TMAX)

    def refresh(t):
        w.set_boundary(_boundary(q2=Edge('upper', held, 0.0)), t, 's1', int(t * 1000))
    t, ev = run(bus, w, 0.0, OT + 0.2, hook=refresh)
    assert w.state[L3] == LIMP and w.outside() == ['limb3 upper boundary (overload release)']
    w.clear_boundary()
    ev = step(bus, w, t + DT)
    assert w.state[L3] == FREE and 'boundary gone' in ev[0]
    # held by the boundary, then cleared: torque off
    refresh(t + 2 * DT)
    step(bus, w, t + 2 * DT)
    assert w.state[L3] == HELD
    w.clear_boundary()
    step(bus, w, t + 3 * DT)
    assert w.state[L3] == FREE and sdk.regs[3][lb.ADDR_TORQUE] == 0


def test_boundary_needs_one_edge_per_servo():
    bus, sdk, w = make()
    with pytest.raises(ValueError):
        w.set_boundary([None], 0.0, 's1', 1)


def test_a_boundary_overload_latch_survives_crossing_the_band_edge():
    lo = BAND[0][0]                                      # limb1 lower wall, tick 508
    bus, sdk, w = make()
    b = _boundary(q0=Edge('lower', lo + 0.2, 0.0))
    _put(sdk, _q(q0=lo + 0.1))
    _hand(sdk, 1, TMAX)
    seq = [0]

    def refresh(t):
        seq[0] += 1
        w.set_boundary(b, t, 's1', seq[0])
    refresh(0.0)
    step(bus, w, 0.0)
    t, _ = run(bus, w, 0.0, OT + 0.3, hook=refresh)
    assert w.state[0] == LIMP and w.owner(0) == BOUNDARY
    n = len(sdk.writes)
    _put(sdk, _q(q0=lo - 0.01))                          # across the band edge
    refresh(t + DT)
    ev = step(bus, w, t + DT)
    assert w.state[0] == LIMP and w.owner(0) == BY_BAND and 'latch moved' in ev[0]
    for k in range(2, 6):
        refresh(t + k * DT)
        step(bus, w, t + k * DT)
    assert sdk.writes[n:] == []                          # no fresh push
    _put(sdk, _q(q0=lo + INSIDE))                        # back inside the band
    refresh(t + 6 * DT)
    step(bus, w, t + 6 * DT)
    assert w.state[0] == FREE and sdk.writes[n:] == []


def test_arming_clears_a_boundary_set_while_unarmed():
    bus, sdk = bus_at(ticks_at(Q))
    w = LeaderWall(bus, CAL, cfg(), NAMES, list(BAND))
    w.set_boundary(_boundary(q2=Edge('upper', Q[L3], 0.0)), 0.0, 's1', 1)
    w.arm(bus.read_positions())
    _put(sdk, _q(q2=Q[L3] + 0.1))
    step(bus, w, 0.0)
    assert w.state[L3] == FREE and sdk.writes == []


# ------------------------------------------------------------------ the band
def test_band_is_the_follower_band_read_from_its_config():
    m, tpr = _margin(), _follower_tpr()
    for i, (lo, hi) in enumerate(BAND):
        assert lo >= ARM_JOINT_LOWER[i] + m * 2 * math.pi / tpr - 1e-9
        assert hi <= ARM_JOINT_UPPER[i] - m * 2 * math.pi / tpr + 1e-9
    # robot case 2026-10-15: limb2 lower limit -3.3427
    assert round(ARM_JOINT_LOWER[1], 4) == -3.3427 and EDGE > -3.3427


def test_band_follows_a_changed_margin_in_the_follower_config(tmp_path):
    """Changing the follower's file changes the wall: the number lives there only."""
    doc = yaml.safe_load(open(ARM_YAML))
    doc['arm_action_server']['ros__parameters']['leader']['limit_command_margin_ticks'] = 30
    p = tmp_path / 'arm_safety_filter.yaml'
    p.write_text(yaml.safe_dump(doc))
    shutil.copy(ARM_CAL, tmp_path / 'arm_calibration.yaml')
    wide = command_band(str(p))
    assert wide[L2][0] - EDGE == pytest.approx((30 - _margin()) * 2 * math.pi / _follower_tpr())
    del doc['arm_action_server']['ros__parameters']['leader']['limit_command_margin_ticks']
    p.write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match='limit_command_margin_ticks'):
        command_band(str(p))


# ------------------------------------------------------------------ node
def _str(text):
    from std_msgs.msg import String
    return String(data=text)


class _Msgs:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _armed_node(node_env):
    node, bus, sdk = node_env
    _put(sdk, Q)                     # inside every band (the leader zero has limb3 beyond)
    _let_go(sdk)
    node._rad_pub = _Msgs()
    node._wall_state_pub = _Msgs()
    node._on_enable(ta._bool(True))
    node._on_follower_leader_state(_str('active'))
    node._tick()
    assert node._wall.armed and sdk.writes == []
    return node, bus, sdk


def _engage_node(node, sdk):
    _put(sdk, _q(q1=EDGE - 0.005))
    _let_go(sdk)
    node._tick()
    assert node._wall.state[L2] == HELD, node._wall_text
    assert node._wall_text.startswith('armed; held: limb2 lower')


def test_node_wall_needs_enable_and_follower_active(node_env):
    node, bus, sdk = node_env
    _put(sdk, _q(q1=EDGE - 0.005))
    node._on_enable(ta._bool(True))
    node._tick()                                           # follower not active yet
    node._on_enable(ta._bool(False))
    node._on_follower_leader_state(_str('active'))
    node._tick()                                           # enable not requested
    assert sdk.writes == [] and not node._wall.armed


def test_node_streaming_continues_while_held(node_env):
    node, bus, sdk = _armed_node(node_env)
    _engage_node(node, sdk)
    n = len(node._rad_pub.msgs)
    for _ in range(5):
        node._tick()
    assert len(node._rad_pub.msgs) == n + 5
    assert node._rad_pub.msgs[-1].position[L2] == pytest.approx(EDGE, abs=0.006)


@pytest.mark.parametrize('trigger', ['space', 'enable_false', 'follower_off', 'read_error',
                                     'shutdown', 'j_align', 'j_cancel'])
def test_node_every_release_is_torque_off_on_all_servos(node_env, trigger):
    from std_msgs.msg import Empty
    node, bus, sdk = _armed_node(node_env)
    _engage_node(node, sdk)
    assert sdk.regs[2][lb.ADDR_TORQUE] == 1
    n = len(sdk.writes)
    if trigger == 'space':
        node._on_stop(Empty())
    elif trigger == 'enable_false':
        node._on_enable(ta._bool(False))
    elif trigger == 'follower_off':
        node._on_follower_leader_state(_str('off (MEASURED pose violates the envelope)'))
    elif trigger == 'read_error':
        sdk.fail_read = True
        node._tick()
        sdk.fail_read = False
        node._tick()                                       # the retry confirms
    elif trigger == 'shutdown':
        node.shutdown()
    elif trigger == 'j_align':
        node._on_align(ta._bool(True))
    elif trigger == 'j_cancel':
        node._on_align(ta._bool(False))
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS)
    assert set(s for s, a, v in sdk.writes[n:] if a == lb.ADDR_TORQUE and v == 0) == set(IDS)
    assert not node._wall.armed and node._wall.unconfirmed == []
    assert goals(sdk) == [HOLD2]                           # the hold tick only


def test_node_does_not_rearm_after_space_until_a_fresh_activation(node_env):
    from std_msgs.msg import Empty
    node, bus, sdk = _armed_node(node_env)
    _engage_node(node, sdk)
    node._on_stop(Empty())
    n = len(sdk.writes)
    _put(sdk, _q(q1=EDGE + 0.04))
    node._tick()
    _put(sdk, _q(q1=EDGE - 0.005))                         # at the wall again
    node._tick()
    assert sdk.writes[n:] == [] and not node._wall.armed  # follower still says 'active'
    node._on_follower_leader_state(_str('off (Space)'))
    node._on_follower_leader_state(_str('active'))
    _put(sdk, _q(q1=EDGE + 0.04))
    node._tick()
    assert node._wall.armed
    _put(sdk, _q(q1=EDGE - 0.005))
    node._tick()
    assert node._wall.state[L2] == HELD


def test_node_unconfirmed_wall_torque_off_refuses_alignment(node_env):
    from std_msgs.msg import Empty
    node, bus, sdk = _armed_node(node_env)
    _engage_node(node, sdk)
    sdk.torque_stuck.add(2)
    node._on_stop(Empty())
    assert node._wall.unconfirmed == [2] and node._wall_text.startswith('released')
    node._on_enable(ta._bool(False))
    ta._follower(node, Q)
    node._on_align(ta._bool(True))
    assert node.align_state.startswith('refused: wall torque-off still unconfirmed')
    sdk.torque_stuck.clear()
    node._tick()
    assert node._wall.unconfirmed == [] and sdk.regs[2][lb.ADDR_TORQUE] == 0


@pytest.mark.parametrize('node_env', [['-p', 'wall_enabled:=false']], indirect=True)
def test_node_wall_disabled_writes_nothing(node_env):
    node, bus, sdk = node_env
    assert node._wall is None and node._wall_text.startswith('disabled')
    node._on_enable(ta._bool(True))
    node._on_follower_leader_state(_str('active'))
    _put(sdk, _q(q1=EDGE - 0.005))
    for _ in range(3):
        node._tick()
    assert sdk.writes == []


@pytest.mark.parametrize('node_env', [['-p', 'follower_safety_filter_file:=/nonexistent.yaml']],
                         indirect=True)
def test_node_without_the_follower_band_has_no_wall(node_env):
    node, bus, sdk = node_env
    assert node._wall is None and node._wall_text.startswith('unavailable')


def test_node_state_names_outside_joints_and_release_reasons(node_env):
    """The web teleop shows these texts (wall_state): held, outside, released + reason;
    a fault carries 'wall fault', an unconfirmed torque-off 'NOT confirmed'."""
    from std_msgs.msg import Empty
    node, bus, sdk = node_env
    _put(sdk, _q(q1=EDGE - 0.05))                          # limb2 beyond at arming
    node._wall_state_pub = _Msgs()
    node._on_enable(ta._bool(True))
    node._on_follower_leader_state(_str('active'))
    node._tick()                                           # held from the first tick
    assert node._wall_text == 'armed; held: limb2 lower wall (spring, tick 111)'
    node._on_stop(Empty())
    assert node._wall_text.startswith('released: Space (/arm/stop); torque off confirmed')
    node._on_follower_leader_state(_str('off (Space)'))
    node._on_follower_leader_state(_str('active'))
    _put(sdk, _q(q1=EDGE + 0.04))
    node._tick()
    _put(sdk, _q(q1=EDGE - 0.005))
    _let_go(sdk)
    node._tick()
    sdk.regs[2][lb.ADDR_PRESENT_VOLTAGE] = 60               # monitor fault while held
    node._tick()
    assert node._wall_text.startswith('released: wall fault: servo 2')
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in IDS)


@pytest.mark.parametrize('node_env', [['-p', 'wall_overload_time_s:=0.05']], indirect=True)
def test_node_overload_release_is_shown_in_the_wall_state(node_env):
    """Held deep beyond the wall with a high load: released after wall_overload_time_s,
    'overload release' on /arm/leader/wall_state under 'outside:' (shown by the web page),
    limp until back inside, no full release."""
    import time
    node, bus, sdk = _armed_node(node_env)
    _put(sdk, _q(q1=EDGE - 1.5 * FULL))
    sdk.regs[2][lb.ADDR_PRESENT_LOAD] = TMAX
    deadline = time.monotonic() + 2.0
    while node._wall.state[L2] != LIMP and time.monotonic() < deadline:
        node._tick()
        time.sleep(0.005)
    assert node._wall_text == 'armed; outside: limb2 lower wall (overload release)'
    assert node._wall.armed and sdk.regs[2][lb.ADDR_TORQUE] == 0
    n = len(sdk.writes)
    for _ in range(5):
        node._tick()
    assert sdk.writes[n:] == []
    _put(sdk, _q(q1=EDGE + INSIDE))
    node._tick()
    assert node._wall_text == 'armed' and node._wall.state[L2] == FREE


def test_node_parameters_reach_the_wall_config(node_env):
    """The node's values are the shipped yaml's; the removed wall2 parameters are gone."""
    node, bus, sdk = node_env
    c = node._wall._cfg
    assert (c.torque_limit, c.min_torque_limit, c.full_force_depth_rad,
            c.engage_depth_rad, c.overload_time_s, c.overload_load,
            c.overload_progress_rad) == (TMAX, TMIN, FULL, ENG, OT, OL, OP)
    assert not any(node.has_parameter(k) for k in REMOVED)


@pytest.mark.parametrize('node_env', [['-p', 'wall_full_force_depth_rad:=0.2',
                                       '-p', 'wall_min_torque_limit:=50']], indirect=True)
def test_node_new_parameters_are_wired(node_env):
    node, bus, sdk = node_env
    assert (node._wall._cfg.full_force_depth_rad, node._wall._cfg.min_torque_limit) == (0.2, 50)


def test_node_records_the_wall_timing_while_held(node_env):
    node, bus, sdk = _armed_node(node_env)
    assert node.wall_tick_max_ms == 0.0
    _engage_node(node, sdk)
    for _ in range(3):
        node._tick()
    assert node.wall_tick_max_ms > 0.0 and node.wall_interval_max_ms > 0.0


def test_engage_depth_below_two_ticks_is_refused_at_startup_with_the_calibrations_ticks():
    """Audit LOW-3: the noise property needs >= 2 ticks of the leader's own calibration
    (ticks_per_rev), so LeaderWall refuses less at construction; 2 ticks exactly is fine."""
    bus, _ = bus_at(ticks_at(Q))
    assert CAL.ticks_per_rev / (2 * math.pi) == pytest.approx(K)
    LeaderWall(bus, CAL, cfg(engage_depth_rad=2.0 / K), NAMES, list(BAND))
    with pytest.raises(ValueError, match='below 2'):
        LeaderWall(bus, CAL, cfg(engage_depth_rad=2.0 / K - 1e-6), NAMES, list(BAND))


@pytest.mark.parametrize('node_env', [['-p', 'wall_engage_depth_rad:=0.002']], indirect=True)
def test_node_with_an_engage_depth_below_two_ticks_has_no_wall(node_env):
    node, bus, sdk = node_env
    assert node._wall is None and 'below 2' in node._wall_text


@pytest.mark.parametrize('persist', [False, True])
def test_node_alignment_retries_a_pending_wall_restore_first(node_env, persist):
    """Audit LOW-2: after a wall release whose Torque_Limit restore failed, the register
    holds the wall's spring value; the aligner reads it as the servo's own limit. Before an
    alignment the pending restore is retried; still failing -> alignment refused, nothing
    written."""
    from std_msgs.msg import Empty
    node, bus, sdk = _armed_node(node_env)
    _engage_node(node, sdk)
    leftover = sdk.regs[2][lb.ADDR_TORQUE_LIMIT]
    assert leftover < 1000
    sdk.fail_write.add((2, lb.ADDR_TORQUE_LIMIT))
    node._on_stop(Empty())
    assert sdk.regs[2][lb.ADDR_TORQUE] == 0 and sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == leftover
    assert 'restore failed' in node._wall_text and not node._wall.armed
    if not persist:
        sdk.fail_write.clear()
    node._on_enable(ta._bool(False))
    _put(sdk, Q)                                           # leader back at the follower's pose
    ta._follower(node, Q)
    n = len(sdk.writes)
    node._on_align(ta._bool(True))
    if persist:
        assert node.align_state == 'refused: wall Torque_Limit restore still pending on servo(s) [2]'
        assert [x for x in sdk.writes[n:] if x[1] != lb.ADDR_TORQUE_LIMIT] == []
        assert sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == leftover
    else:
        assert sdk.writes[n] == (2, lb.ADDR_TORQUE_LIMIT, 1000)   # restored before anything
        assert node.align_state.startswith('direct'), node.align_state   # aligned, no motion
        assert sdk.regs[2][lb.ADDR_TORQUE_LIMIT] == 1000


def test_diagnostics_read_the_dead_zone_registers_without_writing():
    """STS3215 CW/CCW_Dead_Zone (26/27): read for the startup log, never written."""
    bus, sdk = bus_at(ticks_at(Q))
    sdk.regs[2][lb.ADDR_CW_DEAD_ZONE], sdk.regs[2][lb.ADDR_CCW_DEAD_ZONE] = 3, 4
    d = bus.read_diagnostics()
    assert (d[2]['cw_dead_zone'], d[2]['ccw_dead_zone']) == (3, 4) and sdk.writes == []


# ------------------------------------------------------------------ gripper (id 6), 2026-11-13
# FR-17 amendment (user decision 2026-11-13): the gripper has a band and a wall like
# limb1..limb5. Bands from the active follower config; calibration the shipped file.

G = 5                                    # index of the gripper (servo id 6)


def test_command_band_covers_the_gripper_from_the_follower_files():
    """Six bands, the gripper's computed by gripperx_arm.leader_band from the geometry
    limits, the follower calibration and the follower's margin - as for every limb."""
    from gripperx_arm.leader_band import command_band as band
    doc = yaml.safe_load(open(ARM_CAL))['arm_joint_state_relay']['ros__parameters']
    assert len(BAND_ALL) == len(ARM_JOINT_LOWER) == 6
    for i in range(6):
        assert BAND_ALL[i] == band(ARM_JOINT_LOWER[i], ARM_JOINT_UPPER[i],
                                   float(doc['zero_ticks'][i]), int(doc['signs'][i]),
                                   float(doc['ticks_per_rev']), _margin())
        assert ARM_JOINT_LOWER[i] < BAND_ALL[i][0] < BAND_ALL[i][1] < ARM_JOINT_UPPER[i]


def test_gripper_beyond_its_band_edge_gets_the_wall_on_its_own_servo():
    bus, sdk = ta.bus6_at(ta.ticks6_at(ta.Q6))
    edge = BAND_ALL[G][0]                                    # closing edge
    w = LeaderWall(bus, ta.CAL6, cfg(), ta.NAMES6, list(BAND_ALL))
    w.arm(bus.read_positions())
    assert sdk.writes == [] and w.state[G] == FREE
    q = list(ta.Q6)
    q[G] = edge - 0.05
    for sid, t in ta.ticks6_at(q).items():
        sdk.regs[sid][lb.ADDR_PRESENT] = t
        sdk.held.add(sid)
    ev = w.step(bus.read_positions(), 0.0)
    assert w.state[G] == HELD and any('gripper' in e for e in ev)
    assert {s for s, _a, _v in sdk.writes} == {6}            # only the gripper's servo
    assert sdk.regs[6][lb.ADDR_TORQUE] == 1
    w.release_all('test')
    assert all(sdk.regs[s][lb.ADDR_TORQUE] == 0 for s in ta.IDS6)
