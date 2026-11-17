"""The arm node near the ground (user decisions 2026-10-20; SR-17 rules 5, 11, 13): leader
re-plans with a soft start, validated from the measured pose before they are adopted;
the leader's ground recovery (rise); the hold status line; the soft-started automatic
return home. Same mock bus and rig as test_arm_node, the lidar in the robot description
as on the robot. The filter-level rules: test_soft_start.py."""
import math
import random
import sys
import threading
import time
from pathlib import Path

import pytest
import xacro
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_arm_node as tan  # noqa: E402
from test_arm_hold import SagBus  # noqa: E402
from test_arm_node import (  # noqa: E402,F401
    DETERMINISTIC, HOME_D, _approve_groups, _status, rig_factory)
from test_ground_recovery import INCIDENT_TICKS, LIMB5  # noqa: E402
from test_planner import JOINTS  # noqa: E402

from gripperx_arm import arm_action_server as aas  # noqa: E402
from gripperx_arm.exec_steps import ramp_count, soft_start_fractions, ticks_at  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

PARAMS = yaml.safe_load((tan.PKG / 'config' / 'arm_safety_filter.yaml').read_text())[
    'arm_action_server']['ros__parameters']
SOFT = PARAMS['planner']['soft_start_step_fractions']
LIMB2 = 2                              # servo id of limb2
LIMB2_JOINT = 'arm_limb2_joint'


@pytest.fixture(autouse=True)
def lidar_description():
    old = tan.URDF
    tan.URDF = xacro.process_file(str(tan._desc() / 'urdf' / 'gripperx_v1.urdf.xacro'),
                                  mappings={'use_lidar': 'true'}).toxml()
    yield
    tan.URDF = old


def _ticks(d):
    t = list(INCIDENT_TICKS)
    t[1] += d                          # + = limb2 toward the ground
    return t


def _q(node, d):
    return node._cal.to_radians(_ticks(d))


def _sag_rig(rig_factory, monkeypatch, held, sag, **over):
    """The arm HOLDS goals at the incident pose with limb2 `held` ticks off it; under
    gravity limb2 sits `sag` ticks further toward the ground (SagBus: present = goal -
    sag, so a downward sag is negative)."""
    monkeypatch.setattr(tan, 'MockBus', lambda ticks: SagBus(ticks, {LIMB2: -sag}))
    r = rig_factory(start_q=lambda node: _q(node, held), **over)
    goal = r.node._ticks_of(_q(r.node, held))
    r.bus.goal.update(goal)
    for sid in aas.IDS:
        r.bus.settle(sid)
    assert r.bus.pos[LIMB2] == goal[LIMB2] + sag
    return r


def _activate_at_goal(r):
    n = r.node
    r.calv.publish(Bool(data=True))
    r.leader_q = n._q_of(dict(r.bus.goal))
    assert r.wait(lambda: n._gateway_armed is False and n._leader_cal_verified is True
                  and n.count_publishers('/arm/leader/joint_states') == 1
                  and n.count_publishers('/arm/leader/active') == 1
                  and time.monotonic() - n._leader_rx < 0.5, 20.0), 'gate inputs missing'
    r.rising_edge()
    assert r.wait(lambda: n._leader_state == aas.LEADER_ACTIVE), n.leader_text


def _lead_to(r, d, limb1=0.0):
    target = _q(r.node, d)
    target['arm_limb1_joint'] += limb1
    r.leader_q = {n: target[n] for n in r.node._cal.joints[:5]}
    return r.node._ticks_of(target)


def _log(r, k=0):
    return list(r.node.status_log)[k:]


# ---------------------------------------------------------------- leader re-plan

@pytest.mark.parametrize('held,sag', [(-12, 2), (-10, 3), (-14, 1), (-12, 5), (-16, 5),
                                      (-16, 9)])
def test_replan_near_the_ground_executes_with_a_sagging_arm(rig_factory, monkeypatch,
                                                           held, sag):
    """The robot case of 2026-10-20 (held limb5 15.7-17 mm, measured 1-3 ticks lower):
    the direct leader step is refused, the re-plan from the held goals is adopted and
    executed to the leader - never accepted and then failed at its first step, never
    recorded as a failed search. Sag 5 and 9 (audit 2026-10-20: at rest limb1-4 sit 5-9
    ticks off their goals): a constant lag is no ratchet - the growth gate lets it recover.
    The (-16, x) cases start off the robot case: held limb5 17.7 mm, above its 15.7-17 mm
    band - hence the band check below ends at 18 mm."""
    r = _sag_rig(rig_factory, monkeypatch, held, sag)
    node = r.node
    assert 0.0155 < node._filter.ground_heights(node._q_of(dict(r.bus.goal)))[LIMB5] < 0.018
    _activate_at_goal(r)
    k = len(node.status_log)
    goal = _lead_to(r, -80)
    assert r.wait(lambda: all(r.bus.goal[s] == goal[s] for s in aas.LIMB_IDS), 60.0), \
        (node.leader_text, _log(r, k)[-6:])
    log = _log(r, k)
    assert any('leader setpoint REJECTED' in t for t in log), log[-6:]
    assert any('leader re-plan: path direct' in t for t in log), log[-6:]
    assert not any('recorded as failed' in t for t in log), log
    assert not any('DISCARDED' in t for t in log), log
    assert node._leader_state == aas.LEADER_ACTIVE


def test_the_replanned_path_starts_soft(rig_factory, monkeypatch):
    """The first written setpoints of an adopted re-plan path move limb2 by the
    configured fractions of a full leader step (plus whole-tick rounding). Held limb5
    17.7 mm: clear of the margin for a full step, so no lift (a lift takes full steps),
    and 3 ticks of sag make the direct leader step fail first."""
    r = _sag_rig(rig_factory, monkeypatch, -16, 3)
    node = r.node
    _activate_at_goal(r)
    n0 = len(node.audit)
    k = len(node.status_log)
    goal = _lead_to(r, -80)
    assert r.wait(lambda: all(r.bus.goal[s] == goal[s] for s in aas.LIMB_IDS), 60.0), \
        _log(r, k)[-6:]
    i = next(j for j, t in enumerate(list(node.audit)[n0:]) if t[0] == 'approve')
    written = [sp for kind, sp in list(node.audit)[n0 + i:] if kind == 'approve']
    start = node._ticks_of(_q(node, -16))[LIMB2]
    full = PARAMS['leader']['max_joint_velocity_rad_s'] / PARAMS['leader']['rate_hz']
    tick = 2 * 3.141592653589793 / node._cal.ticks_per_rev
    prev = start
    for f, sp in zip(SOFT[:3], written[:3]):
        assert abs(sp[LIMB2] - prev) <= f * full / tick + 1.0, (f, sp[LIMB2] - prev)
        prev = sp[LIMB2]


def test_the_leader_lifts_a_held_arm_out_of_the_ground_margin(rig_factory, monkeypatch):
    """Rule 13 change (2026-10-20): held limb5 10.5 mm, measured 1 tick lower (still
    statically clear, rule 8), leader upward and turned (limb2 AND limb1 differ): the
    re-plan lifts one joint at a time, strictly rising at every written setpoint, then
    follows the leader."""
    r = _sag_rig(rig_factory, monkeypatch, 6, 1, **DETERMINISTIC)
    node, flt = r.node, r.node._filter
    _activate_at_goal(r)
    n0 = len(node.audit)
    k = len(node.status_log)
    goal = _lead_to(r, -80, limb1=0.1)
    assert r.wait(lambda: all(r.bus.goal[s] == goal[s] for s in aas.LIMB_IDS), 60.0), \
        (node.leader_text, _log(r, k)[-6:])
    log = _log(r, k)
    assert any('lifting out of the ground margin' in t for t in log), log[-6:]
    held = node._ticks_of(_q(node, 6))
    poses, here = [node._q_of(held)], dict(held)
    for kind, sp in list(node.audit)[n0:]:
        if kind == 'approve':
            here = {**here, **sp}
            poses.append(node._q_of(here))
    h = [flt.ground_heights(p)[LIMB5] for p in poses]
    # a full leader step's motion bound costs ~6 mm of limb5 clearance here: a step
    # leaving a setpoint less than 5 mm above the margin is still part of the lift
    lift = [i for i in range(1, len(h)) if h[i - 1] < flt.ground_need(LIMB5) + 0.005]
    assert lift and lift == list(range(1, len(lift) + 1)), lift
    for i in lift:
        assert h[i] > h[i - 1], h[:len(lift) + 1]
        assert sum(poses[i][n] != poses[i - 1][n] for n in JOINTS) == 1
    assert min(h) == h[0]


def test_without_the_leader_rise_the_held_arm_holds_and_says_so(rig_factory, monkeypatch):
    """ground_recovery: operator_go_home (the operator's I only) - the same case finds
    no path: nothing is written, and the hold is reported by the status line."""
    r = _sag_rig(rig_factory, monkeypatch, 6, 1, **DETERMINISTIC, **{
        'safety_filter.ground_recovery': 'operator_go_home',
        'leader.hold_status_period_sec': 1.0, 'planner.max_checks': 60})
    node = r.node
    _activate_at_goal(r)
    n0 = len(node.audit)
    k = len(node.status_log)
    _lead_to(r, -80)
    assert r.wait(lambda: any('re-plan: path NONE' in t for t in _log(r, k)), 60.0), \
        _log(r, k)[-6:]
    assert r.wait(lambda: any('leader: HOLDING for' in t and 'no path to the leader' in t
                              and 'measured lowest ground clearance' in t
                              for t in _log(r, k)), 10.0), _log(r, k)[-6:]
    assert 'approve' not in [kind for kind, _ in list(node.audit)[n0:]]
    assert node._leader_state == aas.LEADER_ACTIVE and node._held


# ---------------------------------------------------------------- hold status line

def test_hold_status_line_is_throttled_to_the_configured_period(rig_factory):
    """One line per leader.hold_status_period_sec (read from the active config) while
    the hold lasts: since when, why, and the measured lowest ground clearance."""
    r = rig_factory()
    node = r.node
    period = float(PARAMS['leader']['hold_status_period_sec'])
    assert node._hold_status_period == period
    node._held, node._held_since, node._hold_status_t = True, 100.0, 100.0
    node._hold_reason = 'test reason'
    k = len(node.status_log)
    q = r.measured_q()
    for f in (0.1, 0.5, 0.99, 1.0, 1.5, 1.99, 2.0, 2.01, 3.5):
        node._hold_status(100.0 + f * period, q)
    lines = [t for t in _log(r, k) if 'leader: HOLDING' in t]
    assert len(lines) == 3, lines
    assert lines[0].startswith(f'leader: HOLDING for {period:.1f} s - test reason; '
                               'measured lowest ground clearance ')
    assert f'{2 * period:.1f} s' in lines[1] and f'{3.5 * period:.1f} s' in lines[2]


# ---------------------------------------------------------------- automatic return home

def test_soft_started_return_home_leaves_a_near_ground_pose(rig_factory, monkeypatch):
    """Measured limb5 17.7 mm: before, the automatic return home was refused at sub-step
    0 by its first full ramp step; now it starts soft and arrives. The ramp writes,
    setpoint for setpoint, what the planner validated - soft start included."""
    r = rig_factory(start_q=lambda node: _q(node, -16), **DETERMINISTIC)
    node = r.node
    here = dict(r.bus.pos)
    n0 = len(node.audit)
    node._schedule_home('test')
    assert r.wait(lambda: r.bus.pos == HOME_D, 60.0), _log(r)[-4:]
    assert _status(r, 'automatic: direct path only; soft start): path direct')
    groups = _approve_groups(list(node.audit)[n0:])
    assert len(groups) == 1, len(groups)
    start, home = node._q_of(here), node._q_of({**here, **{s: HOME_D[s] for s in aas.LIMB_IDS}})
    planner = node._planner(0.0, direct_ms=aas.HOME_MOVE_MS, soft_start=True)
    expected = [{s: t[s] for s in aas.LIMB_IDS} for t in
                map(node._ticks_of, planner.executed(start, home, True, first=True)[1:])]
    assert groups[0] == expected
    n = ramp_count(aas.HOME_MOVE_MS / 1000.0, aas.MOVE_STEP_HZ)
    lims = {s: here[s] for s in aas.LIMB_IDS}
    soft = ticks_at(lims, {s: HOME_D[s] for s in aas.LIMB_IDS},
                    soft_start_fractions(n, SOFT), dedupe=True)
    assert groups[0][:len(SOFT)] == soft[:len(SOFT)]


@pytest.mark.parametrize('d', [0, 8])
def test_soft_started_return_home_still_holds_where_the_direct_path_is_too_low(
        rig_factory, d):
    """Measured limb5 12.4 mm (inside the margin for the ramp's full steps) and 9.9 mm
    (statically inside): still refused - no ground recovery for the automatic return
    home (rules 11/13); the arm holds, nothing is written."""
    r = rig_factory(start_q=lambda node: _q(node, d), **DETERMINISTIC)
    n0 = len(r.node.audit)
    r.node._schedule_home('test')
    assert r.wait(lambda: 'hold' in [k for k, _ in list(r.node.audit)[n0:]], 60.0), \
        _log(r)[-4:]
    assert 'approve' not in [k for k, _ in list(r.node.audit)[n0:]]
    assert any('automatic: direct path only; soft start' in t and '[ground]' in t
               for t in _log(r)), _log(r)[-4:]
    assert not any('ground recovery' in t for t in _log(r))



def test_a_rising_step_tolerates_gripper_reading_noise(rig_factory):
    """Audit 2026-10-20: the gripper is not planned (a leader path keeps it as measured).
    A rising step's planned re-check (one joint at a time) compared the previous
    commanded pose, carrying the gripper reading of the tick before, with the new one:
    one tick of gripper reading noise made it 'two joints moving', refused the lift and
    recorded the search as failed. Kills 'the planned rising step is checked with the
    stale gripper reading'."""
    r = rig_factory(start_q=lambda node: _q(node, 6), **DETERMINISTIC)
    node = r.node
    g = node._cal.joints[5]
    tick = 2 * 3.141592653589793 / node._cal.ticks_per_rev
    prev, written = _q(node, 6), _q(node, 5)
    written[g] = prev[g] + tick                       # this tick's gripper reading
    assert not node._filter.check_setpoints([prev, written], rising=(LIMB5,)).ok
    v, planned = node._step_verdict(written | {LIMB2_JOINT: prev[LIMB2_JOINT]}, prev,
                                    written, (LIMB5,), None)
    assert v.ok and planned, str(v)


def test_a_rising_step_is_refused_when_the_measured_box_may_reach_the_ground(
        rig_factory, monkeypatch):
    """_step_verdict: a rising step whose joint box from the MEASURED pose is not
    certified above the ground (_rising_floor <= 0) is refused, although the planned
    step and every other check pass - and it counts as a lag refusal (not a failed
    search). Kills 'the measured-pose bound is not checked' (its other checks normally
    refuse such a pose first; here the bound is forced)."""
    r = rig_factory(start_q=lambda node: _q(node, 6), **DETERMINISTIC)
    node, flt = r.node, r.node._filter
    prev, written = _q(node, 6), _q(node, 5)
    assert flt.check_setpoints([prev, written], rising=(LIMB5,)).ok
    measured = _q(node, 7)                            # one tick of sag below prev
    v, planned = node._step_verdict(measured, prev, written, (LIMB5,), None)
    assert v.ok and planned, str(v)
    monkeypatch.setattr(node, '_rising_floor',
                        lambda m, w, rising: {n: -1e-4 for n in rising})
    v, planned = node._step_verdict(measured, prev, written, (LIMB5,), None)
    assert not v.ok and v.rule == 'ground' and 'from the measured pose' in str(v), str(v)
    assert planned


def test_the_rising_floor_is_below_every_pose_in_the_joint_box(rig_factory):
    """MED-4: _rising_floor (two end poses grown by the motion bound) replaces
    SafetyFilter.ground_step_bound (2^k box vertices) on the tick. It must still certify
    the whole joint box measured -> written: no sampled pose of that box (its vertices
    and random interior points) is lower, across sag in three joints (+-3 ticks)."""
    import itertools
    import random
    r = rig_factory(start_q=lambda node: _q(node, 6), **DETERMINISTIC)
    node, flt = r.node, r.node._filter
    written = _q(node, 5)
    rnd = random.Random(20)
    for d in itertools.product((-3, 0, 3), repeat=3):
        t = _ticks(7)
        t[1] += d[0]
        t[2] += d[1]
        t[3] += d[2]
        m = node._cal.to_radians(t)
        floor = node._rising_floor(m, written, (LIMB5, 'TCP'))
        moving = [n for n in m if m[n] != written[n]]
        samples = [{**m, **{n: (written[n] if bit else m[n]) for n, bit in zip(moving, bits)}}
                   for bits in itertools.product((0, 1), repeat=len(moving))]
        samples += [{**m, **{n: m[n] + rnd.random() * (written[n] - m[n]) for n in moving}}
                    for _ in range(20)]
        for q in samples:
            h = flt.ground_heights(q)
            for n in (LIMB5, 'TCP'):
                assert floor[n] <= h[n] + 1e-9, (d, n, floor[n], h[n])


def test_the_adoption_chain_is_computed_off_the_tick(rig_factory, monkeypatch):
    """MED-4 (tick budget): the re-check of an adopted path's segments (_path_chain, ~75
    ms on the laptop) runs in the search thread when start and envelope are unchanged -
    the leader tick only checks the first step from the measured pose."""
    import threading
    names = []
    real = aas._path_chain

    def spy(*a, **kw):
        names.append(threading.current_thread().name)
        return real(*a, **kw)
    monkeypatch.setattr(aas, '_path_chain', spy)
    r = _sag_rig(rig_factory, monkeypatch, -16, 3)
    node = r.node
    _activate_at_goal(r)
    goal = _lead_to(r, -80)
    assert r.wait(lambda: all(r.bus.goal[s] == goal[s] for s in aas.LIMB_IDS), 60.0), \
        _log(r)[-6:]
    assert names and set(names) == {'leader_replan'}, names


# ---------------------------------------------------------------- the chassis hold of 2026-10-20
# Robot test 2026-10-20, leader mode: limb3 held 3.1 mm (padded boxes) off the chassis.
# Ticks reconstructed from the follow-debug log (incident reconstruction, scratchpad
# recon.py): the measured pose of the hold and the goals held at J.
CHASSIS_MEASURED = [2619, 3155, 1127, 2717, 2063]
CHASSIS_HELD = [2620, 3142, 1125, 2720, 2069]
# The soft start and the lag-growth gate were built against the BOX verdict of this
# hold; with safety_filter.body_hull_stage (active yaml) the hold is no longer a chassis
# refusal at all (test_leader_and_operator_leave_the_chassis_hold). Tests of those
# mechanisms therefore run on the boxes, a state the switch still allows.
BOXES_ONLY = {'safety_filter.body_hull_stage': False}


def _chassis_rig(rig_factory, monkeypatch, held, measured=CHASSIS_MEASURED, block=False,
                 **over):
    """The arm HOLDS goals `held` and sits at `measured`: a constant sag (SagBus), or
    stuck there whatever the goal (`block`)."""
    sag = {} if block else {sid: h - m for sid, h, m in zip(aas.LIMB_IDS, held, measured)}
    monkeypatch.setattr(tan, 'MockBus', lambda ticks: SagBus(ticks, sag))
    grip = tan.HOME_D[aas.GRIPPER_ID]
    r = rig_factory(start_q=lambda node: node._cal.to_radians(list(held) + [grip]), **over)
    r.bus.goal.update(dict(r.node._goal))
    if block:
        r.bus.block.update(dict(zip(aas.LIMB_IDS, measured)))
    for sid in aas.IDS:
        r.bus.settle(sid)
    assert [r.bus.pos[s] for s in aas.LIMB_IDS] == list(measured)
    assert [r.node._goal[s] for s in aas.LIMB_IDS] == list(held)
    return r


@pytest.mark.parametrize('soft', [True, False])
def test_operator_go_home_from_the_chassis_hold_starts_soft(rig_factory, monkeypatch, soft):
    """The operator's I from the incident hold: its direct path's first full ramp step,
    grown by its motion bound, overlaps the chassis (REJECTED [body] at sub-step 0); on
    the robot no detour was found within planner.time_budget_s -> NO_PATH. With the
    configured soft start (active yaml) the DIRECT path passes and is driven home; with
    the soft start switched off ([1.0] = one full first step) the direct path is still
    refused (only a detour, if the search finds one - here with an unbounded budget), so
    the soft start is what makes the difference (user decision 2026-10-20). [True] fails
    on the code without the soft start for I; [False] is a control and passes there too.
    On the box verdict (BOXES_ONLY): with the body hull stage the direct path passes
    even without the soft start."""
    over = dict(BOXES_ONLY) if soft else {**BOXES_ONLY,
                                          'planner.soft_start_step_fractions': [1.0]}
    r = _chassis_rig(rig_factory, monkeypatch, CHASSIS_HELD, **DETERMINISTIC, **over)
    node = r.node
    assert node._filter.check_pose(r.measured_q(), node._envelope()).ok
    k = len(node.status_log)
    r.cmd.publish(tan.String(data='go_home'))
    assert r.wait(lambda: any('go_home (detours allowed' in t for t in _log(r, k)), 60.0), \
        _log(r, k)[-4:]
    plan = next(t for t in _log(r, k) if 'go_home (detours allowed' in t)
    assert 'soft start): path' in plan, plan
    if soft:
        assert 'path direct' in plan, plan
        assert r.wait(lambda: all(r.bus.goal[s] == HOME_D[s] for s in aas.LIMB_IDS), 60.0), \
            _log(r, k)[-4:]
    else:
        assert 'path direct' not in plan, plan
        home = node._home_pos
        start = node._q_of(dict(zip(aas.IDS, CHASSIS_HELD + [HOME_D[aas.GRIPPER_ID]])))
        goal = node._q_of({**dict(zip(aas.IDS, CHASSIS_HELD + [HOME_D[aas.GRIPPER_ID]])),
                           **{s: home[s] for s in aas.LIMB_IDS}})
        why = node._planner(0.0, direct_ms=aas.HOME_MOVE_MS, recovery=True, soft_start=True,
                            rise_ground=True).check_segment(start, goal, node._envelope(),
                                                            direct=True, first=True)
        assert why is not None and '[body] at sub-step 0' in why and 'chassis_link' in why, why


@pytest.mark.parametrize('who', ['leader', 'operator_I'])
def test_leader_and_operator_leave_the_chassis_hold(rig_factory, monkeypatch, who):
    """Both 2026-10-20 changes together (soft start + body hull stage), through the
    node, active yaml unchanged, lidar fitted. From the incident hold - arm at the
    measured pose, goals held:
    - leader: limb2 2 ticks AWAY from the chassis (limb1 -1 tick, as logged). On the
      boxes alone that step was refused at sub-step 0 and leader mode stuck; now it
      passes from the measured pose and is executed;
    - operator_I: go_home takes the direct, soft-started path and arrives home (on the
      robot: NO_PATH)."""
    assert PARAMS['safety_filter']['body_hull_stage'] is True
    r = _chassis_rig(rig_factory, monkeypatch, CHASSIS_HELD)
    node = r.node
    held = node._q_of(dict(zip(aas.IDS, CHASSIS_HELD + [HOME_D[aas.GRIPPER_ID]])))
    if who == 'operator_I':
        k = len(node.status_log)
        r.cmd.publish(tan.String(data='go_home'))
        assert r.wait(lambda: any('go_home (detours allowed' in t for t in _log(r, k)),
                      60.0), _log(r, k)[-4:]
        plan = next(t for t in _log(r, k) if 'go_home (detours allowed' in t)
        assert 'soft start): path direct' in plan, plan
        assert r.wait(lambda: all(r.bus.goal[s] == HOME_D[s] for s in aas.LIMB_IDS),
                      60.0), _log(r, k)[-4:]
        return
    _activate_at_goal(r)
    k = len(node.status_log)
    tick = 2.0 * math.pi / node._cal.ticks_per_rev
    step = dict(held, arm_limb1_joint=held['arm_limb1_joint'] - tick,
                arm_limb2_joint=held['arm_limb2_joint'] + 2 * tick)
    v = node._filter.check_setpoints([r.measured_q(), step], node._envelope())
    assert v.ok, v
    want = node._ticks_of(step)
    r.leader_q = {n: step[n] for n in node._cal.joints[:5]}
    assert r.wait(lambda: all(r.bus.goal[s] == want[s] for s in aas.LIMB_IDS), 60.0), \
        _log(r, k)[-6:]
    assert node._leader_state == aas.LEADER_ACTIVE, node.leader_text
    assert not any('REJECTED' in t for t in _log(r, k)), _log(r, k)[-6:]


def _limb_lag(r):
    return max(abs(aas.tick_deviation(r.bus.goal[s], r.bus.pos[s])) for s in aas.LIMB_IDS)


def test_held_goals_no_longer_ratchet_away_from_a_stuck_arm(rig_factory, monkeypatch):
    """The ratchet of 2026-10-20: the arm stuck at the incident pose (it does not follow
    its goals), held goals 3 ticks of limb2 off it, leader toward the last approved
    setpoint. Before: every re-plan was adopted (its soft first step passes from the
    measured pose), its next step refused ('the arm lags it'), and the next search
    started from goals a step further away - lag 3 -> 18 ticks, writing all along. Now
    no path is adopted once the lag has GROWN by more than
    leader.replan_max_goal_lag_growth_ticks (active yaml) since the arm last tracked:
    the goals stop, the hold says why, once. With the gate closed, searches still run
    (user decision D3 (2026-10-23)), but the path toward this leader target moves the
    goals further from the arm: found, not adopted, and not searched again while
    nothing changes."""
    limit = int(PARAMS['leader']['replan_max_goal_lag_growth_ticks'])
    held = list(CHASSIS_MEASURED)
    held[1] -= 3
    r = _chassis_rig(rig_factory, monkeypatch, held, block=True, **BOXES_ONLY, **{
        'leader.hold_status_period_sec': 1.0})
    node = r.node
    adopted_at = []                        # the largest held-goal lag at every adoption
    real = node._adopt_replan

    def adopt(job, start, measured, here, env, **kw):
        h = node._held_start(here, 'test', report=False)
        lag = max(abs(aas.tick_deviation(h[s], here[s])) for s in aas.LIMB_IDS)
        ok = real(job, start, measured, here, env, **kw)
        if ok:
            adopted_at.append(lag)
        return ok
    node._adopt_replan = adopt
    _activate_at_goal(r)
    k = len(node.status_log)
    grip = tan.HOME_D[aas.GRIPPER_ID]
    q = node._cal.to_radians([2624, 3147, 1119, 2720, 2069, grip])
    q['arm_limb3_joint'] += 0.05
    r.leader_q = {n: q[n] for n in node._cal.joints[:5]}
    gated = r.wait(lambda: any('more than leader.replan_max_goal_lag_growth_ticks' in t
                               for t in _log(r, k)), 30.0)
    # the property itself first: every adoption from goals within the limit of the
    # first hold's lag (3 ticks) - never cumulatively further
    assert adopted_at and max(adopted_at) <= 3 + limit, adopted_at
    assert gated, _log(r, k)[-6:]
    lag, n_writes = _limb_lag(r), len(r.bus.writes)
    time.sleep(4.0)                        # > replan_period_sec x 4: several would-be cycles
    assert len(r.bus.writes) == n_writes and _limb_lag(r) == lag
    assert max(adopted_at) <= 3 + limit, adopted_at
    assert node._replan is None and node._replan_discarded is not None, \
        'the gated search found the growing path and parked it'
    assert any('DISCARDED (not a failed search), lag-growth gate closed: its first goal '
               'change moves' in t for t in _log(r, k)), _log(r, k)[-6:]
    assert not any('ADMITTED through the closed lag-growth gate' in t for t in _log(r, k))
    # reported once (M1), then by the hold status line - which names the binding
    # refusal, the lag and its growth
    assert sum('leader: the held goals moved away' in t for t in _log(r, k)) == 1, _log(r, k)
    assert r.wait(lambda: any(
        'leader: HOLDING' in t and 'replan_max_goal_lag_growth_ticks' in t
        and 'binding refusal: [body] arm_limb3_link vs chassis_link at sub-step 0' in t
        and f'held goal - measured max {lag} ticks (arm_limb' in t
        and f'limit {limit})' in t for t in _log(r, k)), 5.0), _log(r, k)[-3:]
    assert node._leader_state == aas.LEADER_ACTIVE and node._held


def test_hold_status_line_names_the_binding_refusal(rig_factory):
    """The status line names kind and pair of the last REJECTED verdict (not only the
    ground clearance) and the largest held goal - measured lag."""
    r = rig_factory()
    node = r.node
    v = aas.Verdict(False, 'body', 'arm_limb3_link vs chassis_link (clearance 0.0 mm, '
                    'wheels swept over the mechanical steering stops)', 0)
    node._reject_setpoint(str(v), verdict=v)
    node._held_since = node._hold_status_t = 100.0
    k = len(node.status_log)
    node._hold_status(100.0 + node._hold_status_period, r.measured_q())
    line = next(t for t in _log(r, k) if 'leader: HOLDING' in t)
    assert ('; binding refusal: [body] arm_limb3_link vs chassis_link at sub-step 0; '
            'held goal - measured max 0 ticks') in line, line
    assert 'wheels swept' not in line.split('binding refusal')[1], line


def test_lag_growth_baseline_and_its_resets(rig_factory):
    """The growth gate's reference (_lag_growth): the smallest lag since the baseline;
    a constant lag of any size passes, growth beyond the limit is reported, growth
    outpaced by the arm's own motion re-baselines, tracking (_lag_base None) drops it."""
    r = rig_factory()
    node = r.node
    limit = node._max_lag_growth
    assert limit == int(PARAMS['leader']['replan_max_goal_lag_growth_ticks'])
    here = dict(r.bus.pos)
    j2 = node._cal.joints[1]
    assert node._lag_growth(here, {j2: -9}) == {}                  # baseline: 9 ticks
    assert node._lag_growth(here, {j2: -9 - limit}) == {}          # grown by the limit
    assert node._lag_growth(here, {j2: -10 - limit}) == {j2: limit + 1}
    assert node._lag_growth(here, {j2: -6}) == {}                  # shrank: new minimum
    assert node._lag_growth(here, {j2: -7 - limit}) == {j2: limit + 1}
    # goals moved 40 ticks, the arm 30 after them: lag 6 -> 16, it tracked
    far = {**here, 2: here[2] - 30}
    assert node._lag_growth(far, {j2: -16}) == {}
    assert node._lag_base[0][j2] == far[2]                         # re-baselined there
    assert node._lag_growth(far, {j2: -16 - limit}) == {}
    node._lag_base = None                                          # the arm tracked
    assert node._lag_growth(here, {j2: -30}) == {}


def test_an_arm_sagging_away_from_its_goals_is_not_tracking(rig_factory):
    """Audit 2026-10-20 (MED-1): re-baselining on ANY motion let an arm that sags 5 ticks
    per cycle while its goals advance 4 count as tracking - lag 8 -> 98 ticks, the gate
    never closed (the 2026-10-12 pattern: a loaded arm lowered hold by hold). Motion that
    does not outpace the lag's growth is no tracking: the gate closes at the first cycle
    beyond the limit and stays closed."""
    r = rig_factory()
    node = r.node
    lim = node._max_lag_growth
    here, j2, lag = dict(r.bus.pos), node._cal.joints[1], 8
    assert node._lag_growth(here, {j2: -lag}) == {}
    gated = []
    for _ in range(10):
        here = {**here, 2: here[2] - (lim + 1)}      # the arm sags away
        lag += lim + (lim + 1)                       # and the goals advance
        gated.append(node._lag_growth(here, {j2: -lag}))
    assert all(gated), gated
    assert gated[0] == {j2: 2 * lim + 1}




def test_a_tracking_joint_does_not_reset_a_creeping_one(rig_factory):
    """Audit 2026-10-20 (MED-A): with ONE baseline for all joints, limb1 following its
    goals (goals +30 per cycle, arm 18 after them) re-baselined every joint each cycle,
    and limb2 - stuck while its goals crept 4 ticks per cycle - grew its lag 8 -> 48
    without the gate closing. Per joint, limb1 counts as tracking and limb2 does not:
    the gate closes once limb2 has grown beyond the limit and stays closed."""
    r = rig_factory()
    node = r.node
    lim = node._max_lag_growth
    j1, j2 = node._cal.joints[0], node._cal.joints[1]
    here, lag1, lag2 = dict(r.bus.pos), 8, 8
    assert node._lag_growth(here, {j1: lag1, j2: -lag2}) == {}
    gated = []
    for _ in range(10):
        here = {**here, 1: here[1] + 18}             # limb1 follows most of the way
        lag1 += 30 - 18
        lag2 += lim                                   # limb2 stuck, its goals creep
        gated.append(node._lag_growth(here, {j1: lag1, j2: -lag2}))
    assert all(j1 not in g for g in gated), gated
    first = next(k for k, g in enumerate(gated) if g)
    assert first == 1 and all(j2 in g for g in gated[first:]), gated


# ---------------------------------------------------------------- D3 (2026-10-23): shrinking re-plans
# User decision D3 (2026-10-23): while the lag-growth gate is closed, a re-plan is adopted
# if its goal changes bring the goals closer to the measured arm (_lag_shrink_why); the
# ratchet stays impossible.

def _ratchet_to_the_gate(rig_factory, monkeypatch):
    """The 2026-10-20 ratchet replay (test_held_goals_no_longer_ratchet_away_from_a_stuck_
    arm) up to the closed gate: arm stuck at CHASSIS_MEASURED, the leader held toward the
    chassis. Returns the rig and the leader target that ran the goals away."""
    held = list(CHASSIS_MEASURED)
    held[1] -= 3
    r = _chassis_rig(rig_factory, monkeypatch, held, block=True, **BOXES_ONLY, **{
        'leader.hold_status_period_sec': 1.0})
    node = r.node
    _activate_at_goal(r)
    grip = tan.HOME_D[aas.GRIPPER_ID]
    q = node._cal.to_radians([2624, 3147, 1119, 2720, 2069, grip])
    q['arm_limb3_joint'] += 0.05
    away = {n: q[n] for n in node._cal.joints[:5]}
    r.leader_q = away
    assert r.wait(lambda: node._lag_gated and node._replan_discarded is not None, 30.0), \
        _log(r)[-4:]
    return r, away


def _recorder(node, name):
    """Wraps node.<name>, recording (args, kwargs, result) - observation only."""
    real, calls = getattr(node, name), []

    def wrapped(*a, **kw):
        res = real(*a, **kw)
        calls.append((a, kw, res))
        return res
    setattr(node, name, wrapped)
    return calls


def test_a_shrinking_replan_passes_the_closed_gate_and_the_gate_reopens(rig_factory,
                                                                        monkeypatch):
    """D3 (2026-10-23) through the node: from the closed gate of the ratchet replay, the operator moves
    the leader to where the stuck arm IS. The path from the held goals toward it brings
    every goal closer to the arm: admitted (said once), executed without a single goal
    further from the arm than at the closing, and at the next hold the gate is open
    again (growth back within the limit) - re-plans are adopted the ordinary way."""
    r, away = _ratchet_to_the_gate(rig_factory, monkeypatch)
    node = r.node
    # leader writes carry the gripper too since 2026-11-13
    pos = {s: r.bus.pos[s] for s in aas.LEADER_IDS}
    dev0 = {s: abs(aas.tick_deviation(r.bus.goal[s], pos[s])) for s in aas.LEADER_IDS}
    ref = node._lag_base[1]
    gated = {s for s, n in zip(aas.LIMB_IDS, node._cal.joints[:5])
             if dev0[s] - ref[n] > node._max_lag_growth}
    assert gated, 'the replay closes the gate on a grown joint'
    gates = _recorder(node, '_lag_gate')
    k, n_writes = len(node.status_log), len(r.bus.writes)
    m = r.measured_q()
    r.leader_q = {n: m[n] for n in node._cal.joints[:5]}       # toward the arm
    assert r.wait(lambda: any('ADMITTED through the closed lag-growth gate' in t
                              for t in _log(r, k)), 30.0), _log(r, k)[-4:]
    assert r.wait(lambda: all(r.bus.goal[s] == pos[s] for s in gated)
                  and not node._follow_steps, 30.0), (dict(r.bus.goal), _log(r, k)[-4:])
    assert sum('ADMITTED through the closed lag-growth gate' in t for t in _log(r, k)) == 1
    writes = r.bus.writes[n_writes:]
    assert writes, 'the admitted path wrote'
    for sid, tick in writes:                                   # never further from the arm
        assert abs(aas.tick_deviation(tick, pos[sid])) <= dev0[sid], (sid, tick, dev0)
    # the gate reopens: the next hold (the leader back toward the chassis) finds the
    # growth within the limit, and its re-plan is adopted without the shrink rule
    n_gates = len(gates)
    k = len(node.status_log)
    r.leader_q = away
    assert r.wait(lambda: any(res[1] is None for _a, _kw, res in gates[n_gates:]), 30.0), \
        _log(r, k)[-4:]
    assert r.wait(lambda: any(t.startswith('leader re-plan: path') and 'DISCARDED' not in t
                              for t in _log(r, k)), 30.0), _log(r, k)[-4:]
    assert node._leader_state == aas.LEADER_ACTIVE


def test_the_ratchet_is_refused_again_after_noise_on_the_measured_arm(rig_factory,
                                                                     monkeypatch):
    """The ratchet replay at its closed gate, the stuck arm's reading flickering by one
    tick on every limb (reading noise): the growing path toward the leader is never
    adopted, nothing is written, the goals stay where they are."""
    r, _away = _ratchet_to_the_gate(rig_factory, monkeypatch)
    node = r.node
    adoptions = _recorder(node, '_adopt_replan')
    goals, n_writes = dict(r.bus.goal), len(r.bus.writes)
    base = {s: r.bus.pos[s] for s in aas.LIMB_IDS}
    t0 = time.monotonic()
    i = 0
    while time.monotonic() - t0 < 4.0:                         # > replan_period_sec x 4
        i += 1
        for s in aas.LIMB_IDS:
            r.bus.block[s] = base[s] + (1 if (i + s) % 2 else -1)
            r.bus.settle(s)
        time.sleep(0.07)
    for s in aas.LIMB_IDS:
        r.bus.block[s] = base[s]
        r.bus.settle(s)
    assert node._leader_state == aas.LEADER_ACTIVE, node.leader_text
    assert len(r.bus.writes) == n_writes and dict(r.bus.goal) == goals
    assert not any(res for _a, _kw, res in adoptions), adoptions
    assert not any('ADMITTED' in t for t in _log(r)), _log(r)[-4:]


@pytest.mark.parametrize('toward', [True, False])
def test_a_search_started_open_and_finished_gated_is_judged_at_adoption(
        rig_factory, monkeypatch, toward):
    """A search started while the gate was OPEN finishes while it is closed: adopted only
    if its first goal change brings the goals closer to the arm as measured at ADOPTION.
    The search is held back until the gate has closed; meanwhile limb1's goal moves 9
    ticks off the arm (a sag alone closes nothing: user decision 2026-10-30), to the far
    side of the arm from the leader target (the path brings the goal back to the arm:
    admitted) or to its near side (the path would take the goal further still: refused,
    nothing written). The direction is judged against the leader target
    at ADOPTION (audit 2026-10-29, MINOR 4): the leader moves limb3 by 3 ticks after the
    search started - within replan_repeat_tolerance_rad, so the search is kept - and
    the judgement sees the moved target, not the one searched for."""
    go = threading.Event()
    run = aas._ReplanJob._run

    def held_back(self):
        go.wait(30.0)
        run(self)
    monkeypatch.setattr(aas._ReplanJob, '_run', held_back)
    r = _sag_rig(rig_factory, monkeypatch, -20, 2, **{'leader.hold_status_period_sec': 1.0})
    node = r.node
    _activate_at_goal(r)
    adoptions = _recorder(node, '_adopt_replan')
    k = len(node.status_log)
    target = _lead_to(r, -80, limb1=0.2)
    assert r.wait(lambda: node._replan is not None, 20.0), _log(r, k)[-3:]
    job = node._replan
    assert not node._lag_gated
    goal1 = r.bus.goal[1]
    sign = 1 if target[1] > goal1 else -1
    delta = -9 * sign if toward else 9 * sign
    with node._serial_lock:                                    # present = goal - sag
        node._goal[1] = r.bus.goal[1] = goal1 + delta
        r.bus.sag[1] = delta
        r.bus.settle(1)
    goal1 += delta
    assert r.wait(lambda: node._lag_gated, 5.0), _log(r, k)[-3:]
    assert node._replan is job and job.result is None
    judged = _recorder(node, '_lag_shrink_why')
    tick = 2.0 * math.pi / node._cal.ticks_per_rev
    moved = dict(r.leader_q, arm_limb3_joint=r.leader_q['arm_limb3_joint'] + 3 * tick)
    r.leader_q = moved
    want3 = node._ticks_of({**r.measured_q(), **moved})[3]
    assert want3 != node._ticks_of(job.target)[3]
    time.sleep(0.3)
    n_writes = len(r.bus.writes)
    go.set()
    assert r.wait(lambda: adoptions, 30.0), _log(r, k)[-3:]
    (a, kw, res) = adoptions[0]
    assert a[0] is job and kw['gate'] is not None
    assert judged and judged[0][0][1][3] == want3, 'judged against the moved leader'
    if toward:
        assert res is True
        assert any('ADMITTED through the closed lag-growth gate' in t
                   and "'arm_limb1_joint': (9, " in t for t in _log(r, k)), _log(r, k)[-4:]
        assert r.wait(lambda: len(r.bus.writes) > n_writes, 10.0)
    else:
        assert res is False
        assert any('DISCARDED (not a failed search), lag-growth gate closed: its first goal '
                   "change moves ['arm_limb1_joint'] further" in t for t in _log(r, k)), \
            _log(r, k)[-4:]
        time.sleep(2.0)
        assert len(r.bus.writes) == n_writes and r.bus.goal[1] == goal1
        assert not any('ADMITTED' in t for t in _log(r, k))


# --- the rule itself, on the node's own judgement (_lag_shrink_why, _lag_gate)

def _judge(node, start, firsts, here, gated, threshold=4, ref=None, target=None):
    """node._lag_shrink_why on a path from `start` whose steps change `firsts`; the
    leader target by default far along each changed joint's direction."""
    names = node._cal.joints[:5]
    gate = ({names[aas.LIMB_IDS.index(s)] for s in gated}, threshold,
            ref or {n: 0 for n in names})
    steps = [{s: f.get(s, start[s]) for s in aas.LEADER_IDS} for f in firsts]   # gripper since 2026-11-13
    if target is None:
        target = dict(start)
        for f in firsts:
            for s, v in f.items():
                target[s] = start[s] + 100 * (1 if v > start[s] else -1)
    return node._lag_shrink_why(start, target, steps, here, gate)


START = {1: 1000, 2: 2000, 3: 1500, 4: 2500, 5: 2100, 6: 1500}


def test_a_mixed_first_step_is_refused(rig_factory):
    """One joint closer, another further: refused - whether the further one is gated or
    pushed beyond the gate's own allowance (reference + threshold). Unchanged steps are
    skipped (the soft start's first fractions round to the held ticks); a path that
    changes no goal, or brings no gated joint closer, is refused too."""
    node = rig_factory().node
    here = {**START, 2: 2010, 3: 1490}                         # limb2 -10, limb3 +10 ticks
    why, bounds, closer = _judge(node, START, [{2: 2003, 3: 1503}], here, {2, 3})
    assert bounds is None and "moves ['arm_limb3_joint'] further" in why, why
    why, bounds, closer = _judge(node, START, [{2: 2003, 1: 1006}], here, {2, 3})
    assert bounds is None and "moves ['arm_limb1_joint'] further" in why, why
    # controls: the gated one closer, the other gated one unchanged, limb1 within the
    # allowance - admitted, and the first CHANGING step is the one judged
    why, bounds, closer = _judge(node, START, [{}, {}, {2: 2003, 1: 1003}], here, {2, 3})
    assert why is None and closer == {'arm_limb2_joint': (10, 7)}, why
    # 4 ticks off would be AT the allowance read exactly - one tick of noise beyond it
    why, _b, _c = _judge(node, START, [{2: 2003, 1: 1004}], here, {2, 3})
    assert "moves ['arm_limb1_joint'] further" in why, why
    assert bounds[2] == (2010, 2003, None) and bounds[1] == (1000, 1000, 4)
    why, _b, _c = _judge(node, START, [{}, {}], here, {2, 3})
    assert 'does not change the held goals' in why, why
    why, _b, _c = _judge(node, START, [{1: 1002}], here, {2, 3})
    assert 'brings no gated joint' in why, why
    # closer to the arm, but against the leader (a detour's first step), or past it
    why, _b, _c = _judge(node, START, [{2: 2003}], here, {2, 3},
                         target={**START, 2: 1950})
    assert "moves ['arm_limb2_joint'] away from the leader target" in why, why
    why, _b, _c = _judge(node, START, [{2: 2003}], here, {2, 3},
                         target={**START, 2: 2002})
    assert "away from the leader target" in why, why
    why, _b, _c = _judge(node, START, [{2: 2003}], here, {2, 3},
                         target={**START, 2: 2003})
    assert why is None, why


def test_reading_noise_does_not_admit_a_growing_step(rig_factory):
    """A goal that crosses the arm: held 13 ticks above it, the first change 14 below it -
    one tick further. Read one tick off, the naive comparison at the reading says closer
    (13 < 14); judged over the noise band it is refused at every reading within a tick
    of the arm. A goal that stays on its side is judged exactly at any reading."""
    node = rig_factory().node
    noise = node._lag_noise
    assert noise == int(PARAMS['leader']['replan_lag_gate_noise_ticks']) == 1
    start = {**START, 2: 2013}
    for arm in (1999, 2000, 2001):
        here = {**START, 2: arm}
        why, bounds, _c = _judge(node, start, [{2: 1986}], here, {2})
        assert bounds is None and 'further' in why, (arm, why)
        why, bounds, _c = _judge(node, start, [{2: 2010}], here, {2})
        assert why is None, (arm, why)
    assert abs(1986 - 1999) < abs(2013 - 1999)                 # what the reading alone says


def test_a_gated_goal_does_not_cross_the_arm(rig_factory):
    """User decision 2026-10-29 (audit MINOR 2): while the gate is closed a gated
    joint's goal only approaches the measured arm, never crosses it - judged over the
    noise band; landing exactly on the reading is allowed. The auditor's case: held 5
    above the arm, first change 2 below - closer (2 < 5), but across: refused. The
    same shrink without crossing passes; so does one that lands on the reading."""
    node = rig_factory().node
    here = dict(START)                                          # the arm at 2000
    start = {**START, 2: 2005}
    why, bounds, _c = _judge(node, start, [{2: 1998}], here, {2})
    assert bounds is None and "moves ['arm_limb2_joint (across the arm)'] further" in why, why
    why, bounds, _c = _judge(node, start, [{2: 1999}], here, {2})
    assert bounds is None, why                                  # one below: across too
    why, bounds, closer = _judge(node, start, [{2: 2002}], here, {2})
    assert why is None and closer == {'arm_limb2_joint': (5, 2)}, why
    why, bounds, closer = _judge(node, start, [{2: 2000}], here, {2})
    assert why is None and closer == {'arm_limb2_joint': (5, 0)}, why
    # and a later step of an admitted path may not cross either (_follow_step's bound)
    assert not aas.lag_bound_ok(1998, 2000, 2002, None, node._lag_noise)
    assert aas.lag_bound_ok(2001, 2000, 2002, None, node._lag_noise)


def test_no_goal_crosses_the_arm_while_the_gate_is_closed(rig_factory):
    """User decision 2026-10-30 (2b): the crossing ban holds for every joint while the
    gate is closed, not only the gated ones - the servo's force reverses on any joint.
    Over the noise band, with landing exactly on the reading allowed. A held goal within
    the band of the reading has no known side and keeps its allowance: a joint settled
    on its goal must stay movable while another joint holds the gate closed."""
    node = rig_factory().node
    noise = node._lag_noise
    assert not aas.lag_bound_ok(1998, 2000, 2005, 7, noise)    # within the floor, across
    assert not aas.lag_bound_ok(2001, 2000, 1995, 7, noise)    # the same from below
    assert not aas.lag_bound_ok(1999, 2000, 2005, 7, noise)    # one tick past the reading
    assert not aas.lag_bound_ok(4094, 0, 5, 7, noise)          # across the 0/4095 wrap
    assert aas.lag_bound_ok(2000, 2000, 2005, 7, noise)        # on the reading: allowed
    assert aas.lag_bound_ok(2003, 2000, 2005, 7, noise)        # same side
    assert aas.lag_bound_ok(2006, 2000, 2005, 7, noise)        # same side, within floor
    assert aas.lag_bound_ok(2003, 2000, 2000, 7, noise)        # settled: no side, floor
    assert aas.lag_bound_ok(1997, 2000, 2001, 7, noise)        # within the band: as well
    assert not aas.lag_bound_ok(2010, 2000, 2000, 7, noise)    # and only within the floor
    # through the judgement: a non-gated limb1 across the arm refuses the whole path, and
    # the bound it is admitted with refuses a later step across it (_follow_step)
    here = {**START, 2: 2010, 1: 1000}
    start = {**START, 1: 1003}
    why, bounds, _c = _judge(node, start, [{2: 2003, 1: 998}], here, {2})
    assert bounds is None and "['arm_limb1_joint (across the arm)'] further" in why, why
    why, bounds, _c = _judge(node, start, [{2: 2003, 1: 1001}], here, {2})
    assert why is None, why
    assert not aas.lag_bound_ok(998, *bounds[1], noise)
    assert aas.lag_bound_ok(1000, *bounds[1], noise)


def test_a_direct_step_across_the_arm_is_refused_on_a_joint_the_gate_does_not_hold(
        rig_factory, monkeypatch):
    """2b on the direct path: the gate closed by the ratchet replay, limb1 (not gated)
    holding a goal 4 ticks off the stuck arm, the leader asks for 2 ticks past the
    reading - within the gate's allowance, but across. Refused, nothing written."""
    r, _away = _ratchet_to_the_gate(rig_factory, monkeypatch)
    node = r.node
    assert 'arm_limb1_joint' not in node._lag_gate_now[0]
    with node._serial_lock:                    # no tick between these
        arm = r.bus.pos[1]
        node._goal[1] = r.bus.goal[1] = arm + 4
        held = node._q_of({**r.bus.goal, aas.GRIPPER_ID: r.bus.pos[aas.GRIPPER_ID]})
        past = node._q_of({**r.bus.goal, 1: arm - 2, aas.GRIPPER_ID: r.bus.pos[aas.GRIPPER_ID]})
        r.leader_q = {n: past[n] for n in node._cal.joints[:5]}
        node._set_commanded(held)
        node._held = False
    k, n_writes = len(node.status_log), len(r.bus.writes)
    assert r.wait(lambda: any("lag-growth gate closed: the leader step would move "
                              "{'arm_limb1_joint'" in t for t in _log(r, k)), 5.0), \
        _log(r, k)[-4:]
    assert len(r.bus.writes) == n_writes and r.bus.goal[1] == arm + 4


def _lag_rig(rig_factory):
    r = rig_factory()
    return r.node, dict(r.bus.pos), r.node._cal.joints[1]


def test_a_sag_alone_does_not_count_as_growth(rig_factory):
    """User decision 2026-10-30 (1a): with a fixed goal a sag of N ticks grows the
    deviation by N, and no goal moved: nothing counts, however far it sags."""
    node, here, j2 = _lag_rig(rig_factory)
    goal = here[2]
    assert node._lag_growth(here, {j2: 2}) == {}               # goal 2 above the arm
    goal += 2
    for sag in (5, 10, 40, 90):
        arm = {**here, 2: here[2] - sag}
        assert node._lag_growth(arm, {j2: goal - arm[2]}) == {}, sag
    assert node._lag_gate(arm, {j2: goal - arm[2]}) == ({}, None)


@pytest.mark.parametrize('sign', [1, -1])
def test_a_goal_moved_a_little_then_sagging_counts_only_the_goal_travel(rig_factory, sign):
    """The moved-a-little case: the goal moved 2 ticks away from the arm, then the arm
    sags 10 - growth 12, of which only the 2 of goal travel count. It takes goal travel
    beyond the limit to close the gate, and the sag stays uncounted at that point too."""
    node, here, j2 = _lag_rig(rig_factory)
    lim = node._max_lag_growth
    assert node._lag_growth(here, {j2: sign * 3}) == {}        # reference 3
    arm = {**here, 2: here[2] - sign * 10}
    assert node._lag_growth(arm, {j2: sign * (3 + 2 + 10)}) == {}
    assert node._lag_counted(j2, arm[2], sign * (3 + 2 + 10)) == 2
    assert node._lag_growth(arm, {j2: sign * (3 + lim + 10)}) == {}
    assert node._lag_growth(arm, {j2: sign * (3 + lim + 1 + 10)}) == {j2: lim + 1}


def test_a_goal_moved_toward_a_sagging_arm_counts_nothing(rig_factory):
    """The goal followed the operator 6 ticks TOWARD the arm, and the arm sagged 20 away
    from it: the deviation grew and the goal travelled, but not away from the arm."""
    node, here, j2 = _lag_rig(rig_factory)
    assert node._lag_growth(here, {j2: 8}) == {}
    arm = {**here, 2: here[2] - 20}
    assert node._lag_growth(arm, {j2: 8 - 6 + 20}) == {}
    assert node._lag_counted(j2, arm[2], 8 - 6 + 20) <= 0


def test_counted_growth_handles_travel_back_and_the_wrap(rig_factory):
    """Travel is the net distance from the goal at the reference: travelling back uncounts
    it; the goal moved below 0 ticks across the wrap counts the short way round."""
    node, here, j2 = _lag_rig(rig_factory)
    lim = node._max_lag_growth
    near = {**here, 2: 2}                                      # the arm next to the wrap
    assert node._lag_growth(near, {j2: -1}) == {}              # goal 1, deviation 1
    assert node._lag_growth(near, {j2: -1 - lim}) == {}        # goal 1 - lim: at the limit
    grown = node._lag_growth(near, {j2: -2 - lim})             # one further: across 0
    assert grown == {j2: lim + 1}
    assert node._lag_growth(near, {j2: -1 - lim}) == {}        # back: gate open again
    far = {**near, 2: 2 + 40}                                  # sag up the other way
    assert node._lag_growth(far, {j2: -1 - 40}) == {}          # goal back at 1: travel 0


def test_a_true_ratchet_still_closes_the_gate_at_the_same_tick(rig_factory):
    """The ratchet under the new count: an arm that does not move, goals one step further
    on each hold. The counted growth IS the plain growth, the gate closes at limit + 1
    and only a return of the goal opens it. Oscillating back and forth does not reset the
    reference either, and a sag underneath the ratchet does not buy it any travel."""
    node, here, j2 = _lag_rig(rig_factory)
    lim = node._max_lag_growth
    assert node._lag_growth(here, {j2: 8}) == {}
    for grown in range(1, lim + 1):
        assert node._lag_growth(here, {j2: 8 + grown}) == {}, grown
        assert node._lag_counted(j2, here[2], 8 + grown) == grown
    assert node._lag_growth(here, {j2: 8 + lim + 1}) == {j2: lim + 1}
    # a goal shrunk to a new minimum moves the reference with it: the next run starts there
    assert node._lag_growth(here, {j2: 3}) == {}
    assert node._lag_growth(here, {j2: 3 + lim + 1}) == {j2: lim + 1}
    assert node._lag_growth(here, {j2: 3}) == {}
    # sag under the ratchet: the arm goes away, the goal still only gets the limit
    arm = {**here, 2: here[2] - 30}
    assert node._lag_growth(arm, {j2: 3 + 30 + lim}) == {}
    assert node._lag_growth(arm, {j2: 3 + 30 + lim + 1}) == {j2: lim + 1}


@pytest.mark.parametrize('held,sag0,sag1', [(-16, 2, 8), (-12, 1, 7), (-16, 3, 9)])
def test_the_leader_lifts_an_arm_whose_sag_grew_after_activation(rig_factory, monkeypatch,
                                                                 held, sag0, sag1):
    """Audit 2026-10-29 (false lock-up): activated at its goals, limb2 sags further with
    no goal written (load, pose of other joints). The gate stays open, and the leader
    lifts the arm out - with fa4b2b6 two of the three stayed stuck until I or J."""
    r = _sag_rig(rig_factory, monkeypatch, held, sag0)
    node = r.node
    _activate_at_goal(r)
    time.sleep(0.5)
    r.bus.sag[LIMB2] = -sag1
    r.bus.settle(LIMB2)
    time.sleep(0.5)
    k = len(node.status_log)
    goal = _lead_to(r, -80)
    assert r.wait(lambda: all(r.bus.goal[s] == goal[s] for s in aas.LIMB_IDS), 20.0), \
        _log(r, k)[-5:]
    assert not node._lag_gated
    assert not any('the held goals moved away' in t for t in _log(r, k)), _log(r, k)[-5:]


def test_the_gate_opens_only_a_noise_margin_below_the_limit(rig_factory):
    """Hysteresis of _lag_gate: closed beyond the limit, it stays closed while one tick of
    reading noise puts the growth back AT the limit, and opens at limit - noise."""
    node = rig_factory().node
    lim, noise = node._max_lag_growth, node._lag_noise
    here, j2 = dict(START), node._cal.joints[1]
    assert node._lag_gate(here, {j2: -9}) == ({}, None)        # baseline 9
    assert node._lag_gate(here, {j2: -9 - lim}) == ({}, None)  # at the limit: open
    grown, gate = node._lag_gate(here, {j2: -10 - lim})
    assert grown == {j2: lim + 1} and gate[0] == {j2} and node._lag_gated
    grown, gate = node._lag_gate(here, {j2: -9 - lim})         # noise: back at the limit
    assert grown == {j2: lim} and gate is not None and node._lag_gated
    grown, gate = node._lag_gate(here, {j2: -9 - lim + noise})
    assert grown == {} and gate is None and not node._lag_gated


# --- audit 2026-10-29 regressions: the gate must not go blind

def _limb3_lag(r):
    return abs(aas.tick_deviation(r.bus.goal[3], r.bus.pos[3]))


def test_no_runaway_cycle_after_a_path_admitted_through_the_gate(rig_factory, monkeypatch):
    """Audit trace (MAJOR 1): from the closed gate of the ratchet replay the leader moves
    3 ticks toward the stuck arm (a path admitted through the gate, completed), then
    back away - three times. Before: the completed path handed over to direct following,
    one direct step put limb3 17 ticks further off, the direct write dropped the gate's
    baseline, and _held_start's fallback to the measured pose made the gate blind -
    re-plans from the measured pose then ran limb3 to 31 ticks off, cycle after cycle.
    Now the baseline is never dropped while leader mode is active, the lag is judged
    against the goals the servos hold, and a direct step may not move a goal away while
    the gate is closed: the deviation stays within the gate's own bound and does not
    grow from cycle to cycle."""
    r, away = _ratchet_to_the_gate(rig_factory, monkeypatch)
    node = r.node
    limit = node._max_lag_growth
    pos = {s: r.bus.pos[s] for s in aas.LIMB_IDS}
    grip = r.bus.pos[aas.GRIPPER_ID]
    drops = []
    tick_once = node._leader_tick_once

    def tick():
        before = node._lag_base
        tick_once()
        if before is not None and node._lag_base is None:
            drops.append(time.monotonic())
    node._leader_tick_once = tick
    lag0 = _limb_lag(r)
    peaks = []
    for _cycle in range(3):
        k = len(node.status_log)
        goal = dict(r.bus.goal)
        toward = dict(goal)
        for s in aas.LIMB_IDS:
            d = aas.tick_deviation(pos[s], goal[s])
            if abs(d) >= 2:
                toward[s] = goal[s] + max(-3, min(3, d))
        tq = node._q_of({**toward, aas.GRIPPER_ID: grip})
        r.leader_q = {n: tq[n] for n in node._cal.joints[:5]}
        r.wait(lambda: any('ADMITTED' in t for t in _log(r, k)) and not node._follow_steps,
               15.0)
        r.leader_q = away
        peak, t0 = 0, time.monotonic()
        while time.monotonic() - t0 < 5.0:
            peak = max(peak, _limb_lag(r))
            time.sleep(0.03)
        peaks.append(peak)
    assert not drops, 'the gate baseline was dropped while leader mode was active'
    # bounded by the gate (closing at the replay's lag plus the limit), and not
    # growing from cycle to cycle - the ordinary re-plans the reopened gate allows
    # overshoot the closing lag by their own steps, never cumulatively
    assert max(peaks) <= lag0 + limit, (lag0, peaks)
    assert peaks[-1] <= max(peaks[:-1]), peaks
    assert node._leader_state == aas.LEADER_ACTIVE


def test_no_runaway_cycle_from_direct_following(rig_factory, monkeypatch):
    """Audit trace (MAJOR 1, pre-existing on Theo): leader mode activated with limb3 6
    ticks and limb2 4 ticks off a stuck arm, the leader toward the chassis. One direct
    step (velocity-limited, before any hold) is ordinary following; before the fix the
    hold that followed baselined the gate at that lag, re-plans from the measured pose
    wrote 1126, 1123, 1118, 1109, 1096 and repeated about once a second (limb3 31 ticks
    off). Now the baseline is the activation pose: the first hold closes the gate, the
    re-plans from the measured pose are refused, and nothing more is written."""
    held = list(CHASSIS_MEASURED)
    held[2] -= 6
    held[1] -= 4
    r = _chassis_rig(rig_factory, monkeypatch, held, block=True, **BOXES_ONLY, **{
        'leader.hold_status_period_sec': 1.0})
    node = r.node
    _activate_at_goal(r)
    step = math.ceil(float(node._p('leader.max_joint_velocity_rad_s'))
                     / float(node._p('leader.rate_hz'))
                     / (2.0 * math.pi / node._cal.ticks_per_rev))
    grip = tan.HOME_D[aas.GRIPPER_ID]
    q = node._cal.to_radians([2624, 3147, 1119, 2720, 2069, grip])
    q['arm_limb3_joint'] += 0.05
    n_writes = len(r.bus.writes)
    r.leader_q = {n: q[n] for n in node._cal.joints[:5]}
    lags, t0 = [], time.monotonic()
    while time.monotonic() - t0 < 8.0:
        lags.append(_limb3_lag(r))
        time.sleep(0.05)
    limb3 = [t for sid, t in r.bus.writes[n_writes:] if sid == 3]
    assert max(lags) <= 6 + step, (max(lags), step, limb3)
    assert len(limb3) <= 1, limb3
    assert lags[-1] == max(lags) and node._lag_gated, (lags[-1], max(lags))
    assert node._leader_state == aas.LEADER_ACTIVE


# --- user decisions 2026-10-30, third audit: the clamped start and the catch-up rule

SAG_HELD = [2697, 3116, 1198, 2715, 2024]      # kept goals rejected at a sag of 8: the hull


def _sag_at_the_chassis(rig_factory, monkeypatch, off, sag, lift=0):
    """Audit finding B, unforced: held goals near the chassis, limb2 2 ticks off them at
    activation, then `sag` ticks, the leader `lift` ticks up (0: where the goals are)."""
    held = list(SAG_HELD)
    held[1] += off
    measured = list(held)
    measured[1] += 2
    r = _chassis_rig(rig_factory, monkeypatch, held, measured=measured,
                     **{'leader.hold_status_period_sec': 1.0})
    node = r.node
    _activate_at_goal(r)
    time.sleep(0.3)
    if lift:
        tq = node._q_of({**r.bus.goal, LIMB2: r.bus.goal[LIMB2] + lift,
                         aas.GRIPPER_ID: r.bus.pos[aas.GRIPPER_ID]})
        r.leader_q = {j: tq[j] for j in node._cal.joints[:5]}
    r.bus.sag[LIMB2] = -sag
    r.bus.settle(LIMB2)
    return r


@pytest.mark.parametrize('off,sag,lift', [(0, 8, -120), (0, 8, 0), (0, 9, 0), (3, 6, 0),
                                          (3, 6, -120)])
def test_no_goal_runs_with_the_sag_where_the_kept_goals_are_refused(rig_factory, monkeypatch,
                                                                    off, sag, lift):
    """Finding B: with the kept goals refused for the motion, a re-plan from the measured
    pose wrote limb2 3122, 3130, 3138 into the chassis hull, the goal running with the
    sag. The start is clamped to the held goals: nothing moves toward the sag, the arm
    stays clear. Near the hull the lift itself holds too: from the measured pose its first
    step's motion bound (the sag's gap included) exceeds the clearance left."""
    r = _sag_at_the_chassis(rig_factory, monkeypatch, off, sag, lift)
    node = r.node
    goal, n_writes = r.bus.goal[LIMB2], len(r.bus.writes)
    time.sleep(6.0)
    assert [t for sid, t in r.bus.writes[n_writes:] if sid == LIMB2 and t > goal] == []
    assert r.bus.goal[LIMB2] <= goal
    assert node._leader_state == aas.LEADER_ACTIVE, node.leader_text
    assert node._filter.check_pose(r.measured_q(), node._envelope()).ok


def _refuse_kept(node, monkeypatch, static_too=False):
    """_kept_goals_why refuses the motion to the kept goals, with `static_too` the pose."""
    def why(_measured, _kept, static=False, sids=None):
        return 'forced' if static_too or not static else None
    monkeypatch.setattr(node, '_kept_goals_why', why)
    node._held_memo = None


def test_a_lifting_leader_lifts_from_the_clamped_start(rig_factory, monkeypatch):
    """Clamped pose valid, away from any body: the leader lifts, and limb2's goal never
    moves toward the sag on the way."""
    r = _sag_rig(rig_factory, monkeypatch, -16, 2)
    node = r.node
    _activate_at_goal(r)
    time.sleep(0.3)
    r.bus.sag[LIMB2] = -8
    r.bus.settle(LIMB2)
    _refuse_kept(node, monkeypatch)
    g0, n_writes = r.bus.goal[LIMB2], len(r.bus.writes)
    goal = _lead_to(r, -80)
    node._reject_setpoint('a refused step')
    assert r.wait(lambda: r.bus.goal[LIMB2] == goal[LIMB2], 20.0), _log(r)[-4:]
    assert max(t for sid, t in r.bus.writes[n_writes:] if sid == LIMB2) <= g0


def test_a_clamped_pose_that_fails_the_envelope_holds(rig_factory, monkeypatch):
    """Fail closed: the held goals not valid even as a static pose - no search, no write,
    the hold says why; I refuses the same way, nothing written, and leader mode goes on
    holding. No quiet fall-back to the measured pose."""
    r = _sag_rig(rig_factory, monkeypatch, -16, 2, **{'leader.hold_status_period_sec': 0.5})
    node = r.node
    _activate_at_goal(r)
    time.sleep(0.3)
    r.bus.sag[LIMB2] = -8
    r.bus.settle(LIMB2)
    _refuse_kept(node, monkeypatch, static_too=True)
    n_writes = len(r.bus.writes)
    _lead_to(r, -80)
    node._reject_setpoint('a refused step')
    k = len(node.status_log)
    assert r.wait(lambda: any('leader: HOLDING' in t and 'not a valid start' in t
                              for t in _log(r, k)), 5.0), _log(r, k)[-3:]
    assert node._replan is None and len(r.bus.writes) == n_writes
    with pytest.raises(aas.HeldPoseInvalid):
        node._held_start(dict(r.bus.pos), 'test', report=False)


def test_an_invalid_clamped_pose_refuses_the_activation(rig_factory, monkeypatch):
    """The same at activation: the first direct step would start at the measured pose."""
    r = _sag_rig(rig_factory, monkeypatch, -16, 8)
    node = r.node
    _refuse_kept(node, monkeypatch, static_too=True)
    r.calv.publish(Bool(data=True))
    r.leader_q = node._q_of(dict(r.bus.goal))
    assert r.wait(lambda: node._gateway_armed is False and node._leader_cal_verified is True
                  and node.count_publishers('/arm/leader/joint_states') == 1
                  and node.count_publishers('/arm/leader/active') == 1
                  and time.monotonic() - node._leader_rx < 0.5, 20.0), 'gate inputs missing'
    n_writes = len(r.bus.writes)
    r.rising_edge()
    assert r.wait(lambda: 'not a valid start' in node.leader_text, 5.0), node.leader_text
    assert node._leader_state == aas.LEADER_OFF and len(r.bus.writes) == n_writes


@pytest.mark.parametrize('sag,lift', [(110, 0), (150, 0), (110, -150)])
def test_a_servo_beyond_the_contact_threshold_is_clamped_too(rig_factory, monkeypatch, sag,
                                                             lift):
    """The per-joint fallback: limb2 more than hold.contact_threshold_ticks off its goal.
    Before, the first write re-anchored it: 2649 -> 2758 with the leader at the goal.
    Now nothing moves toward the sag; a lift goes the leader's way from the held goal."""
    r = _sag_rig(rig_factory, monkeypatch, -400, 2)
    node = r.node
    _activate_at_goal(r)
    time.sleep(0.3)
    assert sag > node._contact_ticks
    g0, n_writes = r.bus.goal[LIMB2], len(r.bus.writes)
    tq = node._q_of({**r.bus.goal, LIMB2: g0 + lift, aas.GRIPPER_ID: r.bus.pos[aas.GRIPPER_ID]})
    r.leader_q = {j: tq[j] for j in node._cal.joints[:5]}
    r.bus.sag[LIMB2] = -sag
    r.bus.settle(LIMB2)
    time.sleep(0.15)
    node._reject_setpoint('a refused step')
    time.sleep(4.0)
    writes = [t for sid, t in r.bus.writes[n_writes:] if sid == LIMB2]
    assert all(t <= g0 for t in writes), writes
    assert bool(writes) == bool(lift), writes


def test_i_gets_home_from_a_clamped_start(rig_factory, monkeypatch):
    """I, leader mode off, on a held arm whose kept goals are refused for the motion:
    planned from the held goals, home reached, no setpoint toward the sag."""
    r = _sag_rig(rig_factory, monkeypatch, -16, 8)
    node = r.node
    _refuse_kept(node, monkeypatch)
    g0 = r.bus.goal[LIMB2]
    n_writes = len(r.bus.writes)
    r.cmd.publish(String(data='go_home'))
    home = {s: node._home_pos[s] for s in aas.LIMB_IDS}
    assert r.wait(lambda: all(r.bus.goal[s] == home[s] for s in home), 40.0), _log(r)[-4:]
    assert max(t for sid, t in r.bus.writes[n_writes:] if sid == LIMB2) <= g0
    assert _status(r, 'go_home: starts at the held goals as a static pose'), _log(r)[-4:]


def test_j_gets_home_away_from_the_chassis(rig_factory, monkeypatch):
    """J away from any body, the kept goals refused only for the gap: _hold keeps them,
    so the arm does not drop by the sag first, and the return home runs."""
    r = _sag_rig(rig_factory, monkeypatch, -16, 2)
    node = r.node
    _activate_at_goal(r)
    time.sleep(0.3)
    r.bus.sag[LIMB2] = -8
    r.bus.settle(LIMB2)
    _refuse_kept(node, monkeypatch)
    r.enable.publish(Bool(data=False))
    home = {s: node._home_pos[s] for s in aas.LIMB_IDS}
    assert r.wait(lambda: all(r.bus.goal[s] == home[s] for s in home), 40.0), _log(r)[-4:]
    assert node._leader_state == aas.LEADER_OFF


@pytest.mark.parametrize('off,sag', [(3, 6), (0, 8)])
def test_j_holds_at_the_chassis_and_i_gets_home(rig_factory, monkeypatch, off, sag):
    """User decision 2026-11-02: in the state of finding B, I is refused while leader mode
    is on; J leaves it, and _hold keeps the goals (they pass as static poses, only the
    growth for the gap refuses them) - the first hold and the one after the refused
    return home alike, where writing the measured pose ran the goal twice the sag into the
    hull. The automatic return home stays NO_PATH (direct path only, SR-17 rule 11) and
    says so; the operator's I then brings the arm home."""
    r = _sag_at_the_chassis(rig_factory, monkeypatch, off, sag)
    node = r.node
    time.sleep(1.0)
    goal = r.bus.goal[LIMB2]
    r.cmd.publish(String(data='go_home'))
    assert _status(r, 'go_home: leader mode is active'), _log(r)[-4:]
    assert r.bus.goal[LIMB2] == goal
    r.enable.publish(Bool(data=False))
    assert r.wait(lambda: any('return home after abort NOT possible, arm holds: NO_PATH' in t
                              for t in _log(r)), 20.0), _log(r)[-4:]
    assert node._leader_state == aas.LEADER_OFF
    assert r.bus.goal[LIMB2] == goal
    assert node._filter.check_pose(r.measured_q(), node._envelope()).ok
    assert sum('kept as static poses' in t for t in _log(r)) == 2, _log(r)[-4:]
    n_writes = len(r.bus.writes)
    r.cmd.publish(String(data='go_home'))
    home = {s: node._home_pos[s] for s in aas.LIMB_IDS}
    assert r.wait(lambda: all(r.bus.goal[s] == home[s] for s in home), 40.0), _log(r)[-4:]
    assert max(t for sid, t in r.bus.writes[n_writes:] if sid == LIMB2) <= goal


def _wheel_rig(rig_factory, monkeypatch, wanted):
    """A real steered-wheel envelope, no mock: seeded search for goals valid at one steering
    angle for which wanted(node, goals, angle) yields (measured ticks, envelope)."""
    r = _sag_rig(rig_factory, monkeypatch, -16, 2)
    node, flt = r.node, r.node._filter
    margin = float(node._p('safety_filter.steer_motion_margin_rad'))
    envs = {a: flt.envelope_measured({k: a for k in flt.mechanical_ranges}, margin)
            for a in (-1.2, -0.8, -0.4, 0.3, 0.8, 1.2)}
    rng = random.Random(3)
    grip = r.bus.pos[aas.GRIPPER_ID]
    for _ in range(6000):
        goals = {s: t + rng.randint(-500, 500) for s, t in zip(aas.LIMB_IDS, SAG_HELD)}
        a0, a1 = rng.sample(sorted(envs), 2)
        if not flt.check_pose(node._q_of({**goals, aas.GRIPPER_ID: grip}), envs[a0]).ok:
            continue
        here = wanted(node, {**goals, aas.GRIPPER_ID: grip}, envs[a1], rng)
        if here is not None:
            with node._serial_lock:
                node._goal.update(goals)
                r.bus.goal.update(goals)
                r.bus.sag.clear()
                r.bus.block.update({s: here[s] for s in aas.LIMB_IDS})
                for s in aas.LIMB_IDS:
                    r.bus.settle(s)
            node._envelope = lambda env=envs[a1]: env
            return r, goals, here
    pytest.fail('no such pose in the sample')


def test_a_goal_in_a_steered_wheel_is_not_held(rig_factory, monkeypatch):
    """The requirement behind the kept-goal check, on the real envelope: a wheel steered
    into the held goals. They fail as a static pose: every servo takes its measured
    position, the goal in the wheel is not held under force."""
    def in_the_wheel(node, goals, env, rng):
        v = node._filter.check_pose(node._q_of(goals), env)
        if v.ok or 'wheel' not in str(v):
            return None
        return {**goals, 2: goals[2] + 5}
    r, _goals, here = _wheel_rig(rig_factory, monkeypatch, in_the_wheel)
    node = r.node
    node._hold('test')
    assert 'kept goals NOT valid' in _log(r)[-1], _log(r)[-1]
    assert all(r.bus.goal[s] == here[s] for s in aas.LIMB_IDS)


def test_an_arm_touching_a_wheel_is_not_pushed_on_to_a_clear_goal(rig_factory, monkeypatch):
    """Why the measured pose is checked too: goals clear of the wheel, the arm blocked
    against it 40 ticks short. The kept pose passes as a static pose and says nothing
    about the way there: the arm's own pose is what is in the wheel. Not kept."""
    def touching(node, goals, env, rng):
        for sid in aas.LIMB_IDS:
            here = {**goals, sid: goals[sid] + rng.choice((-40, 40))}
            v = node._filter.check_pose(node._q_of(here), env)
            if (node._filter.check_pose(node._q_of(goals), env).ok and not v.ok
                    and 'wheel' in str(v)):
                return here
        return None
    r, _goals, here = _wheel_rig(rig_factory, monkeypatch, touching)
    node = r.node
    node._hold('test')
    assert 'kept goals NOT valid' in _log(r)[-1], _log(r)[-1]
    assert all(r.bus.goal[s] == here[s] for s in aas.LIMB_IDS)


def test_a_followed_move_does_not_count_against_a_later_sag(rig_factory):
    """Finding C, the catch-up rule: goal moved T, the arm after it, the deviation back
    at its reference: `at` and the travel's origin move there, the reference stays. A
    sag S then counts nothing, for every T that used to lock (T <= 2 S + limit)."""
    node, here, j2 = _lag_rig(rig_factory)
    for T in (5, 12, 20, 24):
        node._lag_base = None
        assert node._lag_growth(here, {j2: -2}) == {}
        arm = {**here, 2: here[2] - T}                 # goal and arm T ticks up
        node._held_lag = lambda _h, d={j2: -2}: d
        node._lag_caught_up(arm)
        assert node._lag_base[0][j2] == arm[2] and node._lag_base[1][j2] == 2, T
        sagged = {**arm, 2: arm[2] + 8}
        assert node._lag_gate(sagged, {j2: -2 - 8}) == ({}, None), T


def test_a_stuck_arm_cannot_catch_up(rig_factory):
    """The rule needs the arm to move more than the limit, or a deviation within the
    reopening threshold: neither a stuck arm's noise, nor a running-away goal fires it."""
    node, here, j2 = _lag_rig(rig_factory)
    lim = node._max_lag_growth
    assert node._lag_growth(here, {j2: 2}) == {}
    goal_at = dict(node._lag_base[2])
    node._held_lag = lambda _h, d={j2: 2}: d
    node._lag_caught_up({**here, 2: here[2] + lim})    # moved AT the limit, not beyond
    assert node._lag_base[2] == goal_at
    node._held_lag = lambda _h, d={j2: 2 + lim + 1}: d  # moved, but the deviation is not back
    node._lag_caught_up({**here, 2: here[2] + 50})
    assert node._lag_base[2] == goal_at


@pytest.mark.parametrize('T', [12, 20, 24])
def test_the_leader_lifts_after_a_followed_move_and_a_sag(rig_factory, monkeypatch, T):
    """Finding C through the node, with a hold after the sag: on c2be5d9 T=12 and T=20
    locked until I or J."""
    r = _sag_rig(rig_factory, monkeypatch, -16, 2)
    node = r.node
    _activate_at_goal(r)
    time.sleep(0.4)
    g0 = r.bus.goal[LIMB2]
    tq = node._q_of({**r.bus.goal, LIMB2: g0 - T})
    r.leader_q = {j: tq[j] for j in node._cal.joints[:5]}
    assert r.wait(lambda: r.bus.goal[LIMB2] == g0 - T, 5.0)
    time.sleep(0.3)
    r.bus.sag[LIMB2] = -(2 + 8)
    r.bus.settle(LIMB2)
    time.sleep(0.3)
    goal = _lead_to(r, -80)
    node._reject_setpoint('a refused step')
    assert r.wait(lambda: r.bus.goal[LIMB2] == goal[LIMB2], 12.0), _log(r)[-4:]
    assert not node._lag_gated


def test_the_side_rule_of_a_settled_joint(rig_factory):
    """User decision 2026-10-30 (deviation from the literal crossing ban): a non-gated
    joint whose held goal is within the noise band of the reading has no known side and
    keeps its floor; just outside the band, or gated, the ban holds. Through the
    judgement: limb1 settled on its goal moves with the path, its side is not asked."""
    node = rig_factory().node
    noise = node._lag_noise
    for ref in (2000 - noise, 2000, 2000 + noise):
        assert aas.lag_bound_ok(2004, 2000, ref, 7, noise), ref
        assert aas.lag_bound_ok(1996, 2000, ref, 7, noise), ref
        assert not aas.lag_bound_ok(1996, 2000, ref, None, noise), ref
    assert not aas.lag_bound_ok(1996, 2000, 2000 + noise + 1, 7, noise)
    here = {**START, 2: 2010}
    for lag in (0, noise):
        why, _b, _c = _judge(node, {**START, 1: 1000 + lag}, [{2: 2003, 1: 997}], here, {2})
        assert why is None, (lag, why)


@pytest.mark.parametrize('noise', [0, -1, 4, 5, 1.5])
def test_the_noise_parameter_is_validated(rig_factory, noise):
    """MINOR 3 (audit 2026-10-29): 0 would switch the crossing protection off, a value
    at or above the growth limit leaves no reopening threshold (and a huge one stalls
    the tick), a float would be truncated - refused at startup."""
    with pytest.raises(Exception):
        rig_factory(**{'leader.replan_lag_gate_noise_ticks': noise})
