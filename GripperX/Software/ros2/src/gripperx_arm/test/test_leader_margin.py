"""Leader setpoints keep leader.limit_command_margin_ticks inside the joint limits (user
decision 2026-10-15). Robot finding 2026-10-15 15:17:04: limb2 led to its lower limit
-3.3427 rad, commanded exactly there, measured 1 tick beyond (gravity sag) -> the
measured-pose check (SR-17 rule 8) ended leader mode and the arm returned home.
Same rig as test_arm_node (mock bus, discovery off)."""
import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '226')

import pytest  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_arm_node as tan  # noqa: E402
from test_arm_node import ALLOW, rig_factory  # noqa: E402,F401
from gripperx_arm import arm_action_server as aas  # noqa: E402
from gripperx_arm.leader_band import (braked_fractions, check_edge_slowdown,  # noqa: E402
                                      command_band, edge_speed, step_scale)
from gripperx_arm.planner import Planner  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
L2 = 'arm_limb2_joint'
ROBOT_LOWER = -3.3427        # the logged limit of 2026-10-15 (rounded to 4 places)


def _margin_from_config():
    """The value the node is launched with - not a copy in this test."""
    doc = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())
    return doc['arm_action_server']['ros__parameters']['leader']['limit_command_margin_ticks']


def _cal_from_config():
    return yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())[
        'arm_joint_state_relay']['ros__parameters']


def _tick(q, i, cal):
    return cal['zero_ticks'][i] + cal['signs'][i] * q * cal['ticks_per_rev'] / (2 * math.pi)


# ------------------------------------------------------------------ pure band
def test_band_is_whole_ticks_inside_every_limit():
    cal, m = _cal_from_config(), _margin_from_config()
    from gripperx_geometry.constants import ARM_JOINT_LOWER, ARM_JOINT_UPPER
    for i in range(5):
        lo, hi = ARM_JOINT_LOWER[i], ARM_JOINT_UPPER[i]
        b_lo, b_hi = command_band(lo, hi, cal['zero_ticks'][i], cal['signs'][i],
                                  cal['ticks_per_rev'], m)
        for edge, limit in ((b_lo, lo), (b_hi, hi)):
            t_edge, t_lim = _tick(edge, i, cal), _tick(limit, i, cal)
            assert abs(t_edge - round(t_edge)) < 1e-6           # a whole tick
            assert abs(t_edge - t_lim) >= m - 1e-6              # m ticks inside
            assert abs(t_edge - t_lim) < m + 1                  # not more than needed
        assert lo < b_lo < b_hi < hi


def test_robot_case_limb2_lower_limit_and_five_tick_sag():
    cal, m = _cal_from_config(), _margin_from_config()
    from gripperx_geometry.constants import ARM_JOINT_LOWER, ARM_JOINT_UPPER
    lo, hi = ARM_JOINT_LOWER[1], ARM_JOINT_UPPER[1]
    assert round(lo, 4) == ROBOT_LOWER
    b_lo, _ = command_band(lo, hi, cal['zero_ticks'][1], cal['signs'][1],
                           cal['ticks_per_rev'], m)
    rad_per_tick = 2 * math.pi / cal['ticks_per_rev']
    assert b_lo >= lo + m * rad_per_tick - 1e-9
    # the follower held at the band edge and sagging 5 ticks (the largest sag logged
    # 2026-10-15, by the return home) stays inside the limit
    assert b_lo - 5 * rad_per_tick > lo


@pytest.mark.parametrize('bad', [-1, 1.5, True])
def test_band_refuses_invalid_margin(bad):
    with pytest.raises(ValueError):
        command_band(-1.0, 1.0, 2048, 1, 4096, bad)


def test_band_refuses_a_range_narrower_than_twice_the_margin():
    k = 2 * math.pi / 4096
    with pytest.raises(ValueError):
        command_band(0.0, 15 * k, 2048, 1, 4096, 10)


def test_zero_margin_band_is_the_limits_rounded_inward():
    k = 2 * math.pi / 4096
    lo, hi = command_band(-100.4 * k, 100.4 * k, 2048, -1, 4096, 0)
    assert lo == pytest.approx(-100 * k) and hi == pytest.approx(100 * k)


# ------------------------------------------------------------------ node
def _near_limb2_limit(ticks_inside):
    """limb2 `ticks_inside` ticks above its lower limit, limb3 folded so the pose is
    clear of the chassis (filter-checked offline 2026-10-15)."""
    def start(node):
        q = {j: 0.0 for j in node._cal.joints}
        k = 2 * math.pi / node._cal.ticks_per_rev
        q[L2] = node._filter.kin.limits[L2][0] + ticks_inside * k
        q['arm_limb3_joint'] = 2.5
        return q
    return start


def _l2_tick(node, q):
    return node._unwrapped_ticks({**{j: 0.0 for j in node._cal.joints}, L2: q})[2]


def test_node_band_reads_the_config_margin_and_covers_the_gripper(rig_factory):
    r = rig_factory(**ALLOW)
    n = r.node
    assert n._p('leader.limit_command_margin_ticks') == _margin_from_config()
    band = n._leader_band()
    assert list(band) == list(n._cal.joints)       # the gripper too since 2026-11-13
    lo = n._filter.kin.limits[L2][0]
    m = _margin_from_config()
    assert abs(_l2_tick(n, band[L2][0]) - _l2_tick(n, lo)) >= m


def test_robot_case_follower_command_stays_inside_and_sag_keeps_leader_mode(rig_factory):
    """Leader led to limb2's lower limit and beyond: every written limb2 goal keeps the
    margin; the arm then sags 5 ticks outward and leader mode stays on."""
    r = rig_factory(start_q=_near_limb2_limit(60), **ALLOW)
    n = r.node
    tan._activate(r)
    m = _margin_from_config()
    lo = n._filter.kin.limits[L2][0]
    t_lim = _l2_tick(n, lo)
    sgn = n._cal.signs[1]
    n0 = len(n.audit)
    for target in (lo, lo - 0.05):                     # at the limit, then beyond it
        q = dict(r.leader_q)
        q[L2] = target
        r.leader_q = q
        edge = n._leader_band()[L2][0]
        assert r.wait(lambda: abs(n._commanded()[L2] - edge) < 1e-9, 5.0), n.leader_text
    goals = [sp[2] for k, sp in list(n.audit)[n0:] if k == 'approve' and 2 in sp]
    assert goals, 'no limb2 goal written'
    # lower angle limit = tick side given by the sign: inside means away from t_lim
    assert all(sgn * (t_lim - g) <= -m for g in goals), (goals, t_lim, m)
    assert n._leader_state == aas.LEADER_ACTIVE
    # gravity sag of 5 ticks outward from the held goal (robot: 1 tick at the limit,
    # 5 by the return home)
    r.bus.pos[2] = r.bus.pos[2] - sgn * 5
    assert n._filter.outside_limits(r.measured_q()) == {}
    time.sleep(1.0)
    assert n._leader_state == aas.LEADER_ACTIVE, n.leader_text
    assert 'MEASURED' not in n.leader_text


def test_activation_inside_the_margin_does_not_move_and_never_goes_further_out(rig_factory):
    """An arm resting 3 ticks above the limit (inside the 10-tick margin; the CAD home
    has limb3 8 ticks above its lower limit): activation is accepted (no refusal loop)
    and moves nothing; a leader further out commands nothing; a leader inward is
    followed, and back at the limit the follower stops at the band edge."""
    r = rig_factory(start_q=_near_limb2_limit(3), **ALLOW)
    n = r.node
    start = r.bus.pos[2]
    n0 = len(n.audit)
    tan._activate(r)                                    # leader = measured (inside margin)
    lo = n._filter.kin.limits[L2][0]
    q = dict(r.leader_q)
    q[L2] = lo - 0.02                                   # leader beyond the limit
    r.leader_q = q
    time.sleep(0.6)
    assert 'approve' not in [k for k, _ in list(n.audit)[n0:]]
    assert r.bus.pos[2] == start and n._leader_state == aas.LEADER_ACTIVE, n.leader_text
    q = dict(q)
    q[L2] = lo + 0.1                                    # leader inward: followed
    r.leader_q = q
    assert r.wait(lambda: abs(r.measured_q()[L2] - (lo + 0.1)) < 0.002, 3.0)
    q = dict(q)
    q[L2] = lo                                          # back to the limit: band edge
    r.leader_q = q
    edge_tick = _l2_tick(n, n._leader_band()[L2][0])
    assert r.wait(lambda: r.bus.pos[2] == edge_tick, 3.0), (r.bus.pos[2], edge_tick)


def test_replan_target_is_clamped_and_window_edge_is_decided_before_the_clamp(rig_factory):
    r = rig_factory(**ALLOW)
    n = r.node
    lo = n._filter.kin.limits[L2][0]
    ref = {j: 0.0 for j in n._cal.joints}
    ref[L2] = -1.0
    n._leader_q = dict(ref)
    n._leader_q[L2] = lo - 0.2
    target, far = n._leader_target(ref)
    assert target[L2] == n._leader_band()[L2][0] and far == {}
    n._leader_q[L2] = -1.0
    assert n._leader_target(ref)[0][L2] == -1.0
    # 2 pi off (a crossed window edge): reported as far, although the clamp alone would
    # have produced a valid band-edge target
    n._leader_q[L2] = -1.0 - 2 * math.pi + 0.02
    assert L2 in n._leader_target(ref)[1]
    # a joint held inside the margin at the start of a session keeps its own edge
    n._leader_edges = None
    ref[L2] = lo + 0.001
    n._leader_q[L2] = lo - 0.05
    assert n._leader_target(ref)[0][L2] == ref[L2]


def test_leader_planner_samples_inside_the_band(rig_factory):
    r = rig_factory(**ALLOW)
    n = r.node
    p = n._planner(1.0, executor='leader')
    band = n._leader_band()
    for i, j in enumerate(p.joints):
        assert (p.s_lo[i], p.s_hi[i]) == pytest.approx(band[j])
        assert (p.lo[i], p.hi[i]) == pytest.approx(n._filter.kin.limits[j])
    ramp = n._planner(1.0)
    assert list(ramp.s_lo) == list(ramp.lo) and list(ramp.s_hi) == list(ramp.hi)
    assert isinstance(p, Planner)


def test_invalid_margin_refuses_leader_mode(rig_factory):
    r = rig_factory(**ALLOW, **{'leader.limit_command_margin_ticks': -1})
    r.calv.publish(tan.Bool(data=True))
    r.leader_q = r.measured_q()
    assert r.wait(lambda: r.node._leader_cal_verified is True
                  and r.node._gateway_armed is False, 20.0)
    r.rising_edge()
    assert tan._text(r, 'limit_command_margin_ticks', 5.0), r.node.leader_text
    assert r.node._leader_state == aas.LEADER_OFF


def test_repeated_reanchoring_at_a_sagged_pose_never_ratchets_the_edge_out(rig_factory):
    """Audit LW1 L1: a re-plan starting from the MEASURED pose (_held_start fallback:
    torque off, contact, invalid kept goals) sagged 3 ticks below the last target must
    not move the edge outward - before the fix 10 -> 7 -> 4 -> 1 ticks -> the limit."""
    r = rig_factory(**ALLOW)
    n = r.node
    lo = n._filter.kin.limits[L2][0]
    k = n._cal.ticks_per_rev / (2 * math.pi)
    edge = n._leader_band()[L2][0]
    ref = {j: 0.0 for j in n._cal.joints}
    ref[L2] = edge
    n._enter_leader({**ref, n._cal.joints[5]: 0.0})          # session edge at the band
    n._leader_q = dict(ref)
    n._leader_q[L2] = lo                                     # leader pushed to the limit
    for _ in range(8):
        target, _ = n._leader_target(ref)
        assert target[L2] >= edge - 1e-12
        ref = dict(ref)
        ref[L2] = target[L2] - 3 / k                         # re-anchored, sagged 3 ticks


def test_session_edge_inside_the_margin_only_moves_inward(rig_factory):
    r = rig_factory(**ALLOW)
    n = r.node
    lo = n._filter.kin.limits[L2][0]
    k = n._cal.ticks_per_rev / (2 * math.pi)
    edge = n._leader_band()[L2][0]
    ref = {j: 0.0 for j in n._cal.joints}
    ref[L2] = lo + 3 / k                                     # activated inside the margin
    n._enter_leader({**ref, n._cal.joints[5]: 0.0})
    n._leader_q = dict(ref)
    n._leader_q[L2] = lo
    assert n._leader_target(ref)[0][L2] == pytest.approx(lo + 3 / k)
    sagged = dict(ref)
    sagged[L2] = lo + 1 / k                                  # measured 2 ticks lower
    assert n._leader_target(sagged)[0][L2] == pytest.approx(lo + 3 / k)
    inward = dict(ref)
    inward[L2] = lo + 6 / k                                  # follower moved inward
    assert n._leader_target(inward)[0][L2] == pytest.approx(lo + 6 / k)
    assert n._leader_target(sagged)[0][L2] == pytest.approx(lo + 6 / k)   # never back out
    far_in = dict(ref)
    far_in[L2] = -1.0
    n._leader_target(far_in)
    assert n._leader_target(sagged)[0][L2] == pytest.approx(edge)       # capped at the band


# ------------------------------------------------------------------ edge slowdown (2026-10-16)
# Robot finding 2026-10-16 at 0.8 rad/s: limb2 led fast downward reached its band edge at
# full speed, overshot by momentum to 2 ticks beyond its lower limit, and the measured-pose
# check (SR-17 rule 8) ended leader mode. User decision: brake toward the edges.
def _leader_cfg(key):
    """The value the node is launched with - not a copy in this test."""
    doc = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())
    return doc['arm_action_server']['ros__parameters']['leader'][key]


def _slow_cfg():
    return (float(_leader_cfg('max_joint_velocity_rad_s')), float(_leader_cfg('rate_hz')),
            float(_leader_cfg('edge_slowdown_zone_rad')),
            float(_leader_cfg('edge_slowdown_velocity_rad_s')))


def test_edge_speed_profile_is_constant_deceleration():
    vmax, _, zone, vslow = _slow_cfg()
    assert 0.0 < vslow < vmax and zone > 0.0
    assert edge_speed(zone, vmax, zone, vslow) == pytest.approx(vmax)
    assert edge_speed(2 * zone, vmax, zone, vslow) == vmax
    assert edge_speed(0.0, vmax, zone, vslow) == pytest.approx(vslow)
    assert edge_speed(-0.01, vmax, zone, vslow) == pytest.approx(vslow)   # beyond: slowest
    ds = [zone * k / 20 for k in range(21)]
    v = [edge_speed(d, vmax, zone, vslow) for d in ds]
    assert all(a < b for a, b in zip(v, v[1:]))
    # v^2 linear in d: one deceleration (vmax^2 - vslow^2) / (2 zone) over the zone
    a = (vmax ** 2 - vslow ** 2) / (2 * zone)
    for d, vv in zip(ds, v):
        assert vv ** 2 == pytest.approx(vslow ** 2 + 2 * a * d)
    assert edge_speed(0.0, vmax, 0.0, vslow) == vmax                      # zone 0 = off
    assert edge_speed(0.0, 0.5, zone, 0.8) == 0.5                         # never above vmax


@pytest.mark.parametrize('zone, vslow', [(-0.1, 0.2), (0.15, 0.0), (0.15, -0.2),
                                         (float('nan'), 0.2)])
def test_edge_slowdown_parameters_are_checked(zone, vslow):
    with pytest.raises(ValueError):
        check_edge_slowdown(zone, vslow)


def test_step_scale_slows_toward_the_edge_only_and_keeps_the_direction():
    vmax, rate, zone, vslow = _slow_cfg()
    edges = {'a': (-1.0, 1.0), 'b': (-1.0, 1.0)}
    cmd = {'a': -0.95, 'b': 0.0}                 # a 0.05 rad above its lower edge
    goal = {'a': -1.0, 'b': 0.02}                # a toward its edge, b far from both
    sc = step_scale(cmd, goal, edges, vmax, rate, zone, vslow)
    step = {n: (goal[n] - cmd[n]) * sc for n in cmd}
    # a binds at the profile speed of its 0.05 rad; b is scaled with it (one factor)
    assert abs(step['a']) * rate == pytest.approx(edge_speed(0.05, vmax, zone, vslow))
    assert edge_speed(0.05, vmax, zone, vslow) < vmax
    assert step['b'] / step['a'] == pytest.approx((goal['b'] - cmd['b']) / (goal['a'] - cmd['a']))
    # moving away from the same edge: the plain velocity limit
    away = {'a': -0.5, 'b': 0.0}
    sc = step_scale(cmd, away, edges, vmax, rate, zone, vslow)
    assert abs((away['a'] - cmd['a']) * sc) * rate == pytest.approx(vmax)
    # outside the zone: the plain limit, vmax / rate on the largest joint
    far = {'a': 0.0, 'b': 0.0}
    goal = {'a': -0.5, 'b': 0.1}
    sc = step_scale(far, goal, edges, vmax, rate, zone, vslow)
    assert 0.5 * sc * rate == pytest.approx(vmax)
    # zone 0: exactly the former uniform limit dmax / max |d|
    sc0 = step_scale(cmd, {'a': -1.0, 'b': 0.5}, edges, vmax, rate, 0.0, vslow)
    assert sc0 == pytest.approx(min(1.0, vmax / rate / 0.5))


def test_step_scale_uses_the_measured_pose_when_it_is_closer_to_the_edge():
    """A servo sagging AHEAD of its setpoint toward the edge brakes the step; a servo
    trailing it (the normal approach) does not loosen it."""
    vmax, rate, zone, vslow = _slow_cfg()
    edges = {'a': (-1.0, 1.0)}
    cmd, goal = {'a': -0.9}, {'a': -1.0}
    plain = step_scale(cmd, goal, edges, vmax, rate, zone, vslow)
    ahead = step_scale(cmd, goal, edges, vmax, rate, zone, vslow, measured={'a': -0.99})
    trail = step_scale(cmd, goal, edges, vmax, rate, zone, vslow, measured={'a': -0.8})
    assert ahead < plain and trail == plain
    assert 0.1 * ahead * rate == pytest.approx(edge_speed(0.01, vmax, zone, vslow))
    beyond = step_scale(cmd, goal, edges, vmax, rate, zone, vslow, measured={'a': -1.01})
    assert 0.1 * beyond * rate == pytest.approx(vslow)


def _limb2_setpoints(n, n0):
    """The limb2 angles of the leader goals written since audit index n0."""
    out = []
    for k, sp in list(n.audit)[n0:]:
        if k == 'approve' and 2 in sp:
            out.append(n._q_of({**tan.HOME_D, **sp})[L2])
    return out


def test_node_reads_the_slowdown_from_the_config(rig_factory):
    r = rig_factory(**ALLOW)
    for key in ('edge_slowdown_zone_rad', 'edge_slowdown_velocity_rad_s',
                'max_joint_velocity_rad_s'):
        assert r.node._p(f'leader.{key}') == _leader_cfg(key)


def test_robot_case_fast_leader_beyond_the_lower_edge_brakes_in_the_zone(rig_factory):
    """The 2026-10-16 case: the leader moves limb2 fast to beyond its lower limit. Outside
    the zone the setpoint steps at the full limit, inside it at most at the profile speed
    of its distance to the edge; it stops exactly at the band edge."""
    vmax, rate, zone, vslow = _slow_cfg()
    r = rig_factory(start_q=_near_limb2_limit(330), **ALLOW)   # ~0.5 rad above the limit
    n = r.node
    tan._activate(r)
    n0 = len(n.audit)
    start = n._commanded()[L2]
    lo = n._filter.kin.limits[L2][0]
    edge = n._leader_band()[L2][0]
    q = dict(r.leader_q)
    q[L2] = lo - 0.05                                  # beyond the limit, at once
    r.leader_q = q
    assert r.wait(lambda: abs(n._commanded()[L2] - edge) < 1e-9, 10.0), n.leader_text
    sp = [start] + _limb2_setpoints(n, n0)
    tick = 2 * math.pi / n._cal.ticks_per_rev
    full = [b - a for a, b in zip(sp, sp[1:]) if a - edge >= zone + vmax / rate]
    assert full and all(abs(s) == pytest.approx(vmax / rate, abs=tick) for s in full), full
    zone_steps = [(a - edge, a - b) for a, b in zip(sp, sp[1:]) if a - edge < zone]
    assert len(zone_steps) >= 5, zone_steps
    for d, s in zone_steps:
        assert 0.0 < s <= edge_speed(d, vmax, zone, vslow) / rate + tick + 1e-9, (d, s)
    # the last approach (inside 0.02 rad) is well below the full speed
    close = [s * rate for d, s in zone_steps if d <= 0.02]
    assert close and max(close) <= edge_speed(0.02, vmax, zone, vslow) + tick * rate
    assert max(close) < 0.75 * vmax                    # a real slowdown, whatever the config
    assert all(x >= edge - 1e-9 for x in sp)           # never past the band edge
    assert n._leader_state == aas.LEADER_ACTIVE, n.leader_text


def test_slow_speed_is_reached_at_the_edge_with_a_wide_zone(rig_factory):
    """Zone wider than the travel and vslow = 0.1: the final steps run at about vslow."""
    r = rig_factory(start_q=_near_limb2_limit(60), **ALLOW,
                    **{'leader.edge_slowdown_zone_rad': 1.0,
                       'leader.edge_slowdown_velocity_rad_s': 0.1})
    n = r.node
    tan._activate(r)
    n0 = len(n.audit)
    start = n._commanded()[L2]
    edge = n._leader_band()[L2][0]
    q = dict(r.leader_q)
    q[L2] = n._filter.kin.limits[L2][0]
    r.leader_q = q
    assert r.wait(lambda: abs(n._commanded()[L2] - edge) < 1e-9, 10.0), n.leader_text
    sp = [start] + _limb2_setpoints(n, n0)
    vmax, rate = float(_leader_cfg('max_joint_velocity_rad_s')), float(_leader_cfg('rate_hz'))
    tick = 2 * math.pi / n._cal.ticks_per_rev
    steps = [a - b for a, b in zip(sp, sp[1:])]
    assert max(steps) * rate <= edge_speed(start - edge, vmax, 1.0, 0.1) + tick * rate
    assert steps[-1] * rate <= edge_speed(0.01, vmax, 1.0, 0.1) + tick * rate


def test_moving_away_from_the_edge_is_not_slowed(rig_factory):
    """At the band edge, the leader moves limb2 inward: full speed from the first step."""
    vmax, rate, _, _ = _slow_cfg()
    m = _margin_from_config()
    r = rig_factory(start_q=_near_limb2_limit(m), **ALLOW)
    n = r.node
    tan._activate(r)
    n0 = len(n.audit)
    start = n._commanded()[L2]
    q = dict(r.leader_q)
    q[L2] = start + 0.3
    r.leader_q = q
    assert r.wait(lambda: len(_limb2_setpoints(n, n0)) >= 3, 5.0), n.leader_text
    sp = [start] + _limb2_setpoints(n, n0)
    tick = 2 * math.pi / n._cal.ticks_per_rev
    for a, b in zip(sp[:4], sp[1:4]):
        assert b - a == pytest.approx(vmax / rate, abs=tick)


def test_slowdown_keeps_the_step_direction(rig_factory):
    """limb2 near its edge brakes the whole step: limb1 moves with the same factor, so
    every written step stays on the line to the target the safety filter checks."""
    vmax, rate, zone, vslow = _slow_cfg()
    r = rig_factory(start_q=_near_limb2_limit(60), **ALLOW)
    n = r.node
    tan._activate(r)
    n0 = len(n.audit)
    c0 = dict(n._commanded())
    edge = n._leader_band()[L2][0]
    L1 = 'arm_limb1_joint'
    q = dict(r.leader_q)
    q[L2] = n._filter.kin.limits[L2][0] - 0.05           # clamped to the edge
    q[L1] = c0[L1] + 0.05                                # limb2's edge binds, not limb1
    r.leader_q = q
    assert r.wait(lambda: len([k for k, _ in list(n.audit)[n0:] if k == 'approve']) >= 1,
                  5.0), n.leader_text
    sp = next(sp for k, sp in list(n.audit)[n0:] if k == 'approve')
    first = n._q_of({**tan.HOME_D, **sp})
    tick = 2 * math.pi / n._cal.ticks_per_rev
    d1, d2 = first[L1] - c0[L1], first[L2] - c0[L2]
    g1, g2 = 0.05, edge - c0[L2]
    # one factor for both joints (whole ticks: one tick of rounding each)
    assert abs(d1 * g2 - d2 * g1) <= tick * (abs(g1) + abs(g2)) + 1e-9, (d1, d2, g1, g2)
    # limb2 at the profile speed of its distance, limb1 slowed with it
    assert -d2 * rate == pytest.approx(edge_speed(c0[L2] - edge, vmax, zone, vslow),
                                       abs=tick * rate)
    assert 0.0 < d1 < vmax / rate - tick


def test_invalid_slowdown_refuses_leader_mode(rig_factory):
    r = rig_factory(**ALLOW, **{'leader.edge_slowdown_velocity_rad_s': 0.0})
    r.calv.publish(tan.Bool(data=True))
    r.leader_q = r.measured_q()
    assert r.wait(lambda: r.node._leader_cal_verified is True
                  and r.node._gateway_armed is False, 20.0)
    r.rising_edge()
    assert tan._text(r, 'edge_slowdown_velocity_rad_s', 5.0), r.node.leader_text
    assert r.node._leader_state == aas.LEADER_OFF



# ------------------------------------------------------------------ audit ES1 (2026-10-16)
def test_slow_speed_below_one_tick_per_tick_is_refused():
    """L1: below one servo tick per leader tick the joint that sets the step rounds to
    the same tick every tick - the arm would freeze without a word."""
    _, rate, zone, _ = _slow_cfg()
    one = 2 * math.pi / 4096 * rate
    check_edge_slowdown(zone, one, one)
    with pytest.raises(ValueError, match='one servo tick'):
        check_edge_slowdown(zone, one * 0.9, one)
    assert float(_leader_cfg('edge_slowdown_velocity_rad_s')) >= one


def test_slow_speed_below_one_tick_per_tick_refuses_leader_mode(rig_factory):
    rate = float(_leader_cfg('rate_hz'))
    r = rig_factory(**ALLOW, **{'leader.edge_slowdown_velocity_rad_s':
                                0.5 * 2 * math.pi / 4096 * rate})
    r.calv.publish(tan.Bool(data=True))
    r.leader_q = r.measured_q()
    assert r.wait(lambda: r.node._leader_cal_verified is True
                  and r.node._gateway_armed is False, 20.0)
    r.rising_edge()
    assert tan._text(r, 'one servo tick', 5.0), r.node.leader_text
    assert r.node._leader_state == aas.LEADER_OFF


def test_braked_fractions_stay_on_the_segment_and_follow_the_profile():
    vmax, rate, zone, vslow = _slow_cfg()
    edges = {'a': (-1.0, 1.0), 'b': (-1.0, 1.0)}
    qa, qb = {'a': -0.5, 'b': 0.0}, {'a': -1.0, 'b': 0.1}
    fr = braked_fractions(qa, qb, ['a', 'b'], edges, vmax, rate, zone, vslow)
    assert fr and fr[-1] == 1.0 and all(x < y for x, y in zip(fr, fr[1:]))
    prev = 0.0
    for f in fr:
        a0 = qa['a'] + prev * (qb['a'] - qa['a'])
        step = abs(f - prev) * 0.5                       # limb a travels 0.5 rad in all
        assert step <= edge_speed(a0 + 1.0, vmax, zone, vslow) / rate + 1e-12
        prev = f
    # away from the edge, or zone 0: no braking -> the caller's equal steps
    assert braked_fractions(qb, qa, ['a', 'b'], edges, vmax, rate, zone, vslow) is None
    assert braked_fractions(qa, qb, ['a', 'b'], edges, vmax, rate, 0.0, vslow) is None


def test_replanned_path_brakes_into_the_band_edge(rig_factory):
    """H1 (auditor's demo, 2026-10-16): a held arm (any rejected setpoint) re-plans to a
    leader target clamped at limb2's lower band edge. Before the fix the planned path ran
    into the edge at the full 0.78 rad/s every step - the overshoot of the robot case."""
    vmax, rate, zone, vslow = _slow_cfg()
    r = rig_factory(start_q=_near_limb2_limit(330), **ALLOW)
    n = r.node
    tan._activate(r)
    n0 = len(n.audit)
    start = n._commanded()[L2]
    edge = n._leader_band()[L2][0]
    n._held = True                                     # as after any rejected setpoint
    q = dict(r.leader_q)
    q[L2] = n._filter.kin.limits[L2][0] - 0.05
    r.leader_q = q
    assert r.wait(lambda: abs(n._commanded()[L2] - edge) < 1e-9, 20.0), n.leader_text
    assert any('re-plan' in t for t in n.status_log), list(n.status_log)[-5:]
    sp = [start] + _limb2_setpoints(n, n0)
    tick = 2 * math.pi / n._cal.ticks_per_rev
    zone_steps = [(a - edge, a - b) for a, b in zip(sp, sp[1:]) if a - edge < zone]
    assert len(zone_steps) >= 5, zone_steps
    for d, s in zone_steps:
        assert s <= edge_speed(d, vmax, zone, vslow) / rate + tick + 1e-9, (d, s)
    assert max(s for d, s in zone_steps if d <= 0.02) * rate < 0.75 * vmax
    assert all(x >= edge - 1e-9 for x in sp)
    assert n._leader_state == aas.LEADER_ACTIVE, n.leader_text


def test_measured_pose_sagging_ahead_of_the_setpoint_brakes(rig_factory):
    """L2: the servo sags 30 ticks below its held goal toward the lower edge: the first
    step toward the edge brakes for the MEASURED distance, not the setpoint's."""
    vmax, rate, zone, vslow = _slow_cfg()
    r = rig_factory(start_q=_near_limb2_limit(60), **ALLOW)
    n = r.node
    tan._activate(r)
    tick = 2 * math.pi / n._cal.ticks_per_rev
    sgn = n._cal.signs[1]
    c0 = n._commanded()[L2]
    edge = n._leader_band()[L2][0]
    r.bus.pos[2] = r.bus.pos[2] - sgn * 30             # 30 ticks lower, toward the edge
    sag = r.measured_q()[L2]
    assert sag == pytest.approx(c0 - 30 * tick) and sag > edge
    time.sleep(0.2)
    n0 = len(n.audit)
    q = dict(r.leader_q)
    q[L2] = n._filter.kin.limits[L2][0] - 0.05
    r.leader_q = q
    assert r.wait(lambda: len(_limb2_setpoints(n, n0)) >= 1, 5.0), n.leader_text
    step = c0 - _limb2_setpoints(n, n0)[0]
    assert step <= edge_speed(sag - edge, vmax, zone, vslow) / rate + tick + 1e-9
    # the setpoint distance alone would allow clearly more
    assert step < edge_speed(c0 - edge, vmax, zone, vslow) / rate - 2 * tick


def _near_limb2_upper(ticks_inside):
    def start(node):
        q = {j: 0.0 for j in node._cal.joints}
        k = 2 * math.pi / node._cal.ticks_per_rev
        q[L2] = node._filter.kin.limits[L2][1] - ticks_inside * k
        return q
    return start


def test_upper_edge_brakes_too(rig_factory):
    """L3: limb2's UPPER edge (limb2 is a sign -1 servo: the upper angle edge is its LOW
    tick side)."""
    vmax, rate, zone, vslow = _slow_cfg()
    assert _cal_from_config()['signs'][1] == -1
    r = rig_factory(start_q=_near_limb2_upper(260), **ALLOW)
    n = r.node
    assert n._filter.check_pose(r.measured_q()).ok
    tan._activate(r)
    n0 = len(n.audit)
    start = n._commanded()[L2]
    edge = n._leader_band()[L2][1]
    q = dict(r.leader_q)
    q[L2] = n._filter.kin.limits[L2][1] + 0.05
    r.leader_q = q
    assert r.wait(lambda: abs(n._commanded()[L2] - edge) < 1e-9, 10.0), n.leader_text
    sp = [start] + _limb2_setpoints(n, n0)
    tick = 2 * math.pi / n._cal.ticks_per_rev
    zone_steps = [(edge - a, b - a) for a, b in zip(sp, sp[1:]) if edge - a < zone]
    assert len(zone_steps) >= 5, zone_steps
    for d, s in zone_steps:
        assert 0.0 < s <= edge_speed(d, vmax, zone, vslow) / rate + tick + 1e-9, (d, s)
    assert max(s for d, s in zone_steps if d <= 0.02) * rate < 0.75 * vmax
    assert all(x <= edge + 1e-9 for x in sp)
    assert n._leader_state == aas.LEADER_ACTIVE, n.leader_text
