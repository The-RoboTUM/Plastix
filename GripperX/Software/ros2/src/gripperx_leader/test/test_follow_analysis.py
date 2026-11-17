"""record_follow's analysis on synthetic recordings (no ROS, no hardware)."""

import io
import math
import os
import sys

import pytest

from gripperx_leader import follow_analysis as fa


def _leader_q(t, amp=0.5, f=0.3):
    return [amp * math.sin(2 * math.pi * f * t), 0.0, 0.2, 0.0, 0.0]


def _clamped_follower(lag, vmax, dt, t_end, f=0.3):
    """The follower chases the leader of `lag` s ago at most vmax rad/s."""
    q, out, t = 0.0, [], 0.0
    while t <= t_end:
        target = _leader_q(t - lag, f=f)[0]
        q += max(-vmax * dt, min(vmax * dt, target - q))
        out.append((t, q))
        t += dt
    return out


def _rows(lag=0.15, vmax=0.8, foll_dt=1 / 30, t_end=20.0, f=0.3, net=0.004, pi_offset=3.0):
    rows = []
    for k in range(int(t_end * 30) + 1):
        t = k / 30
        rows.append(fa.joint_row(t + net, t, fa.LEADER, fa.JOINTS, _leader_q(t, f=f)))
    for t, q in _clamped_follower(lag, vmax, foll_dt, t_end, f=f):
        # follower header stamps on the Pi clock, offset from the laptop's
        rows.append(fa.joint_row(t + net, t + pi_offset, fa.FOLLOWER,
                                 [f'arm_{j}_joint' for j in fa.JOINTS],
                                 [q, 0.0, 0.2, 0.0, 0.0]))
    rows.append(fa.text_row(1.0, fa.WALL_STATE, 'armed; held: limb2 lower (tick 109)'))
    rows.sort(key=lambda r: r['recv_s'])
    return rows


def test_lag_and_errors_of_a_delayed_follower():
    res = fa.analyze(_rows(lag=0.15, vmax=5.0), max_velocity=0.8)
    r = res['joints']['limb1']
    assert r['lag_s'] == pytest.approx(0.15, abs=0.006) and r['corr'] > 0.999
    # at shift 0 the error is the delay times the leader speed, ~0.94 * 0.15
    assert r['max_err_rad'] == pytest.approx(0.5 * 2 * math.pi * 0.3 * 0.15, rel=0.1)
    assert r['max_err_at_lag_rad'] < 0.01
    assert r['leader_max_vel'] == pytest.approx(0.5 * 2 * math.pi * 0.3, rel=0.02)


def test_velocity_against_the_limit():
    """Leader peak 0.94 rad/s: a follower clamped at 0.8 is within, an unclamped one is over."""
    clamped = fa.analyze(_rows(vmax=0.8), max_velocity=0.8)['joints']['limb1']
    assert clamped['follower_max_vel'] == pytest.approx(0.8, rel=0.01)
    assert clamped['over_limit'] is False
    free = fa.analyze(_rows(vmax=5.0), max_velocity=0.8)['joints']['limb1']
    assert free['follower_max_vel'] > 0.9 and free['over_limit'] is True


def test_joints_that_did_not_move_have_no_lag():
    res = fa.analyze(_rows(), max_velocity=0.8)
    for j in ('limb2', 'limb3', 'limb4', 'limb5'):
        assert res['joints'][j]['lag_s'] is None
        assert res['joints'][j]['max_err_rad'] == pytest.approx(0.0, abs=1e-9)


def test_intervals_use_receive_time_and_header_stamps():
    res = fa.analyze(_rows(t_end=10.0), max_velocity=0.8)
    for key in ('recv_s', 'stamp_s'):
        st = res['intervals'][fa.FOLLOWER][key]
        assert st['mean_s'] == pytest.approx(1 / 30, rel=0.01)
        assert st['max_s'] == pytest.approx(1 / 30, rel=0.01)
    assert len(res['warnings']) == 1 and 'no /arm/leader/follow_debug' in res['warnings'][0]
    assert res['ticks'] is None


def test_interval_percentiles():
    st = fa.interval_stats([0.0] + [sum([0.03] * k) + (0.1 if k >= 100 else 0.0)
                                    for k in range(1, 101)])
    assert st['n'] == 100 and st['max_s'] == pytest.approx(0.13)
    assert st['p95_s'] == pytest.approx(0.03) and st['mean_s'] == pytest.approx(0.031)


def test_two_hz_follower_is_flagged_and_still_finds_the_lag():
    """/joint_states is a 2 Hz timer on the robot: the analysis says so."""
    res = fa.analyze(_rows(lag=0.2, vmax=5.0, foll_dt=0.5, t_end=60.0), max_velocity=0.8)
    assert res['joints']['limb1']['lag_s'] == pytest.approx(0.2, abs=0.02)
    assert any('2 Hz timer' in w for w in res['warnings'])
    # velocity between 0.5 s samples is an average: the 0.94 rad/s peak is understated
    assert res['joints']['limb1']['follower_max_vel'] < 0.94


def test_leader_gap_is_not_interpolated_across():
    rows = [r for r in _rows(vmax=5.0)
            if not (r['source'] == fa.LEADER and 5.0 < r['recv_s'] < 8.0)]
    full = fa.analyze(_rows(vmax=5.0), max_velocity=0.8)['joints']['limb1']
    gap = fa.analyze(rows, max_velocity=0.8)['joints']['limb1']
    assert gap['lag_s'] == pytest.approx(full['lag_s'], abs=0.006)
    assert gap['max_err_rad'] <= full['max_err_rad'] + 1e-9


def test_csv_roundtrip(tmp_path):
    p = tmp_path / 'rec.csv'
    with open(p, 'w', newline='') as fh:
        w = fa.CsvWriter(fh)
        for r in _rows(t_end=3.0):
            w.write(r)
    rows = fa.read_csv(str(p))
    assert rows[0].keys() >= {'recv_s', 'stamp_s', 'source', 'limb1', 'text'}
    texts = [r for r in rows if r['source'] == fa.WALL_STATE]
    assert texts[0]['text'] == 'armed; held: limb2 lower (tick 109)' and texts[0]['limb1'] is None
    a = fa.analyze(rows, max_velocity=0.8)
    b = fa.analyze(_rows(t_end=3.0), max_velocity=0.8)
    assert a['joints']['limb1']['lag_s'] == b['joints']['limb1']['lag_s']
    assert 'limb1' in fa.format_report(a)


def test_joint_names_of_both_sides_and_others_ignored():
    assert fa.joint_index('limb3') == 2 and fa.joint_index('arm_limb3_joint') == 2
    assert fa.joint_index('gripper') is None and fa.joint_index('wheel_fl') is None
    row = fa.joint_row(1.0, 2.0, fa.FOLLOWER, ['wheel_fl', 'arm_limb2_joint'], [9.9, 0.3])
    assert row == {'recv_s': 1.0, 'stamp_s': 2.0, 'source': fa.FOLLOWER, 'limb2': 0.3}


def test_speed_limit_is_read_from_the_follower_config():
    """The analysis limit is the follower's own value, not a copy."""
    import yaml
    from gripperx_leader.follower_config import leader_max_velocity
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    from test_align import ARM_YAML
    v, path = leader_max_velocity(ARM_YAML)
    doc = yaml.safe_load(open(ARM_YAML))
    assert v == doc['arm_action_server']['ros__parameters']['leader']['max_joint_velocity_rad_s']
    assert path == ARM_YAML


def test_recorder_publishes_nothing():
    """record_follow is a pure subscriber: no publisher, no service client in its source."""
    src = open(os.path.join(os.path.dirname(fa.__file__), 'record_follow.py')).read()
    for bad in ('create_publisher', 'create_client', 'ActionClient', '.publish('):
        assert bad not in src


def test_writer_formats_floats():
    buf = io.StringIO()
    w = fa.CsvWriter(buf)
    w.write(fa.joint_row(1.23456789, 0.0, fa.LEADER, ['limb1'], [0.1234567]))
    assert buf.getvalue().splitlines()[1].startswith('1.234568,0.000000,leader,0.123457,')


# ------------------------------------------------------------------ follow_debug (per tick)
NAMES = [f'arm_{j}_joint' for j in fa.JOINTS]


def _ticks(amp=0.3, f=0.3, t_end=20.0, rate=30.0, vmax=0.8, transport=0.05, servo_ticks=3,
           step_cap=None, pi_offset=3.0, slow_every=50, net=0.004, stalls=0.0, drop=(),
           off=None):
    """A simulated Pi tick: reads the servos (measured = the setpoint written
    `servo_ticks` ticks earlier), builds the target from the leader of `transport` s ago,
    steps the setpoint by at most vmax / rate (or step_cap) and publishes follow_debug.
    Every `slow_every`-th tick takes 40 ms and delays the next one. `stalls`: share of
    messages the network holds 0.15-0.3 s (else 0-10 ms); delivery keeps the order, so
    the ones behind a stall arrive in a bunch with it; `drop`: tick numbers whose message is
    lost; `off`: (t0, t1) leader mode off - no ticks published, and the setpoint
    restarts from the measured pose, as an activation does."""
    import random
    rnd = random.Random(7)
    rows = []
    for k in range(int(t_end * 30) + 1):
        t = k / 30
        rows.append(fa.joint_row(t + net, t, fa.LEADER, fa.JOINTS, _leader_q(t, amp, f)))
    dmax = step_cap if step_cap is not None else vmax / rate
    hist, sp, t, prev, k = [], 0.0, 0.5, None, 0
    durations, periods = [], []
    was_off, last_recv = False, 0.0
    while t < t_end - 0.5:
        if off and off[0] <= t < off[1]:
            was_off = True
            t += 1 / rate
            continue
        dur = 0.040 if k % slow_every == slow_every - 1 else 0.008
        meas = hist[-servo_ticks] if len(hist) >= servo_ticks else 0.0
        if was_off:                          # re-activation: commanded := measured
            sp, hist, was_off = meas, [meas] * servo_ticks, False
        target = _leader_q(t - transport, amp, f)[0]
        sp += max(-dmax, min(dmax, target - sp))
        hist.append(sp)
        period = 0.0 if prev is None else t - prev
        if k not in drop:
            delay = rnd.uniform(0.15, 0.3) if rnd.random() < stalls else rnd.uniform(0, 0.01)
            last_recv = max(last_recv, t + dur + net + delay)
            rows += fa.debug_rows(last_recv, t + pi_offset,
                                  NAMES, [sp, 0.0, 0.2, 0.0, 0.0], [target, 0.0, 0.2, 0.0, 0.0],
                                  [meas, 0.0, 0.2, 0.0, 0.0], dur, period, 'written')
            durations.append(dur)
            if prev is not None:
                periods.append(period)
        prev, k = t, k + 1
        t += max(1 / rate, dur)
    rows.sort(key=lambda r: r['recv_s'])
    return rows, durations, periods


def test_tick_duration_and_period():
    rows, durations, periods = _ticks()
    res = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)
    tk = res['ticks']
    assert tk['nominal_period_s'] == pytest.approx(1 / 30)
    assert tk['tick_s']['max_s'] == pytest.approx(0.040)
    assert tk['tick_s']['mean_s'] == pytest.approx(sum(durations) / len(durations))
    assert tk['period_s']['max_s'] == pytest.approx(0.040)
    assert tk['period_s']['p95_s'] == pytest.approx(1 / 30)
    assert tk['outcomes'] == {'written': len(durations)}
    assert not any('no /arm/leader/follow_debug' in w for w in res['warnings'])


def test_split_rate_limit_lag_and_servo_lag():
    """Unsaturated leader: leader->setpoint is the transport, setpoint->measured the servo.
    Network stalls on the receive times must not reach the servo lag: that one is on the
    Pi stamps of follow_debug (on receive times the stalls move it by ~10 ms)."""
    rows, _, _ = _ticks(amp=0.3, transport=0.05, servo_ticks=3)
    calm = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)['joints']['limb1']
    assert calm['lag_leader_setpoint_s'] == pytest.approx(0.05 + 0.012, abs=0.02)
    rows, _, _ = _ticks(amp=0.3, transport=0.05, servo_ticks=3, stalls=0.03)
    r = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)['joints']['limb1']
    # leader -> setpoint is on receive times: the stalls add to it
    assert r['lag_leader_setpoint_s'] >= calm['lag_leader_setpoint_s']
    assert r['lag_setpoint_measured_s'] == pytest.approx(3 / 30, abs=0.006)
    assert r['lag_leader_measured_s'] == pytest.approx(
        r['lag_leader_setpoint_s'] + r['lag_setpoint_measured_s'], abs=0.03)
    assert calm['corr_leader_setpoint'] > 0.99 and r['corr_setpoint_measured'] > 0.999
    assert r['setpoint_over_limit'] is False


def test_saturated_rate_limit_shows_in_the_setpoint():
    """Leader peak 0.94 rad/s > 0.8: the setpoint steps at the cap, never above it, and
    falls further behind the leader than the transport alone."""
    rows, _, _ = _ticks(amp=0.5, transport=0.05)
    r = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)['joints']['limb1']
    assert r['setpoint_max_step_rad'] == pytest.approx(0.8 / 30, rel=1e-3)
    assert r['setpoint_over_limit'] is False
    assert r['leader_max_vel'] > 0.9
    free, _, _ = _ticks(amp=0.5, transport=0.05, step_cap=1.0)
    rf = fa.analyze(free, max_velocity=0.8, rate_hz=30.0)['joints']['limb1']
    assert r['max_err_leader_setpoint_rad'] > rf['max_err_leader_setpoint_rad'] + 0.01


def test_setpoint_step_above_the_limit_is_flagged():
    rows, _, _ = _ticks(amp=0.5, step_cap=0.05)
    r = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)['joints']['limb1']
    assert r['setpoint_max_step_rad'] > 0.8 / 30 * 1.05
    assert r['setpoint_over_limit'] is True


def test_follow_debug_csv_roundtrip(tmp_path):
    rows, _, _ = _ticks(t_end=4.0)
    p = tmp_path / 'rec.csv'
    with open(p, 'w', newline='') as fh:
        w = fa.CsvWriter(fh)
        for r in rows:
            w.write(r)
    back = fa.read_csv(str(p))
    a = fa.analyze(back, max_velocity=0.8, rate_hz=30.0)
    b = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)
    assert a['ticks']['tick_s']['max_s'] == pytest.approx(b['ticks']['tick_s']['max_s'])
    assert a['joints']['limb1']['lag_setpoint_measured_s'] == \
        b['joints']['limb1']['lag_setpoint_measured_s']
    rep = fa.format_report(a)
    assert 'tick duration' in rep and 'nominal 33.3 ms' in rep


def test_debug_rows_omit_empty_arrays():
    rows = fa.debug_rows(1.0, 2.0, NAMES, [0.1] * 5, [], [], 0.01, 0.033, 'holding')
    assert [r['source'] for r in rows] == [fa.SETPOINT]
    assert rows[0]['text'] == 'holding' and rows[0]['tick_s'] == 0.01


def test_rate_is_read_from_the_follower_config():
    import yaml
    from gripperx_leader.follower_config import leader_rate_hz
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from test_align import ARM_YAML
    v, _ = leader_rate_hz(ARM_YAML)
    doc = yaml.safe_load(open(ARM_YAML))
    assert v == doc['arm_action_server']['ros__parameters']['leader']['rate_hz']


def test_dropped_message_is_not_a_double_step():
    """A lost best-effort follow_debug message: the pair around it spans two ticks
    (0.0533 rad at 0.8 rad/s) - left out and counted, not an over-limit."""
    rows, _, _ = _ticks(amp=0.5, drop=(100, 250))
    res = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)
    r = res['joints']['limb1']
    assert r['setpoint_over_limit'] is False
    assert r['setpoint_max_step_rad'] == pytest.approx(0.8 / 30, rel=1e-3)
    assert r['setpoint_skipped_gap'] == 2
    assert any('2 setpoint pair(s)' in w for w in res['warnings'])


def test_reactivation_gap_is_not_a_step():
    """Leader mode off for 2 s, then on again from the measured pose: the jump across
    the gap is no step."""
    rows, _, _ = _ticks(amp=0.5, off=(6.0, 8.0))
    r = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)['joints']['limb1']
    assert r['setpoint_over_limit'] is False and r['setpoint_skipped_gap'] == 1
    assert r['setpoint_max_step_rad'] <= 0.8 / 30 + 1e-9


def test_planned_path_steps_are_not_checked_against_the_leader_limit():
    rows, _, _ = _ticks(amp=0.5, step_cap=0.05, t_end=5.0)
    for r in rows:
        if r['source'] == fa.SETPOINT:
            r['text'] = 'follow_step'
    r = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)['joints']['limb1']
    assert r['setpoint_max_step_rad'] is None and r['setpoint_over_limit'] is None
    assert r['setpoint_skipped_other'] > 0


def _ramp_hold(t):
    """0 -> 0.5 rad in 1 s, hold 3 s, back in 1 s, hold 3 s (period 8 s)."""
    u = t % 8.0
    return 0.5 * (min(u, 1.0) if u < 4.0 else max(0.0, 1.0 - (u - 4.0)))


def test_lag_on_ramp_and_hold_motion():
    """Ramp-and-hold: long flat stretches; the lag is the RMS minimum, not a plateau of
    the correlation."""
    ref = [(k / 30, _ramp_hold(k / 30)) for k in range(30 * 32)]
    out = [(k / 30, _ramp_hold(k / 30 - 0.15)) for k in range(30 * 32)]
    shifts = [-0.2 + 0.005 * k for k in range(341)]
    res = fa.lag(ref, out, shifts, 0.2, 0.02)
    assert res['lag_s'] == pytest.approx(0.15, abs=0.006)
    assert res['rms_at_lag_rad'] < 0.002 and res['max_err_at_lag_rad'] < 0.01
    # a follower that only reaches 80 % (scale) still has its delay found
    out80 = [(t, 0.8 * q) for t, q in out]
    assert fa.lag(ref, out80, shifts, 0.2, 0.02)['lag_s'] == pytest.approx(0.15, abs=0.03)



# ------------------------------------------------------------------ tick phases (2026-10-16)
def _phased(n=40, slow_at=17):
    """Ticks with phase times: read 3 ms, check 10 ms (+ k % 3 ms), write 4 ms, the
    rest 1 ms; tick `slow_at` waits 800 ms in its read (a serial-lock wait)."""
    rows = []
    for k in range(n):
        ph = {'refusal': 0.0005, 'read': 0.003, 'envelope': 0.002, 'replan': 0.0,
              'target': 0.0005, 'check': 0.010 + (k % 3) * 0.001, 'write': 0.004}
        if k == slow_at:
            ph['read'] += 0.8
        ph['other'] = 0.001
        tick = sum(ph.values())
        rows += fa.debug_rows(k / 30 + 0.01, k / 30, NAMES, [0.0] * 5, [0.0] * 5, [0.0] * 5,
                              tick, 1 / 30, 'written', ph, 3 + k % 2)
    return rows


def test_phase_breakdown_mean_p95_max_and_the_slowest_tick():
    res = fa.analyze(_phased(), max_velocity=0.8, rate_hz=30.0)
    tk = res['ticks']
    ph = tk['phases']
    assert list(ph) == fa.PHASES
    assert ph['check']['mean_s'] == pytest.approx(0.010 + sum(k % 3 for k in range(40)) / 40e3)
    assert ph['check']['max_s'] == pytest.approx(0.012)
    assert ph['read']['max_s'] == pytest.approx(0.803)
    assert ph['read']['p95_s'] == pytest.approx(0.003)
    assert ph['replan']['max_s'] == 0.0
    assert tk['check_poses'] == {'n': 40, 'mean': 3.5, 'p95': 4, 'max': 4}
    slow = tk['slowest'][0]
    assert slow['longest'] == 'read' and slow['tick_s'] == pytest.approx(0.8215, abs=0.002)
    assert slow['stamp_s'] == pytest.approx(17 / 30)
    assert len(tk['slowest']) == fa.SLOWEST
    rep = fa.format_report(res)
    assert 'tick phases' in rep and 'slowest 5 ticks' in rep
    assert 'read 803.0' in rep


def test_phase_csv_roundtrip(tmp_path):
    rows = _phased()
    p = tmp_path / 'rec.csv'
    with open(p, 'w', newline='') as fh:
        w = fa.CsvWriter(fh)
        for r in rows:
            w.write(r)
    back = fa.read_csv(str(p))
    a = fa.analyze(back, max_velocity=0.8, rate_hz=30.0)['ticks']
    b = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)['ticks']
    for k in fa.PHASES:
        assert a['phases'][k]['max_s'] == pytest.approx(b['phases'][k]['max_s'], abs=1e-6)
    assert a['check_poses'] == b['check_poses']


def test_recording_without_phases_has_no_breakdown():
    """A recording of an arm_action_server without the phase fields (5380da3)."""
    rows, _, _ = _ticks(t_end=3.0)
    tk = fa.analyze(rows, max_velocity=0.8, rate_hz=30.0)['ticks']
    assert tk['phases'] is None and tk['slowest'] == [] and tk['check_poses'] is None
    assert 'tick phases' not in fa.format_report(fa.analyze(rows, max_velocity=0.8,
                                                            rate_hz=30.0))


def test_recorder_takes_the_phases_from_the_message():
    from gripperx_arm_msgs.msg import LeaderFollowDebug
    from gripperx_leader.record_follow import debug_phases
    m = LeaderFollowDebug()
    m.read_s, m.check_s, m.other_s = 0.003, 0.01, 0.002
    ph = debug_phases(m)
    assert list(ph) == fa.PHASES
    assert ph['read'] == 0.003 and ph['check'] == 0.01 and ph['write'] == 0.0

    class Old:                                     # a message type without the fields
        pass
    assert debug_phases(Old()) is None
