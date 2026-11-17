"""arm_wrap_offset: plan, offset encoding, the one-transaction tick patch, and the
round trip old ticks -> new ticks -> radians (identical). Mock bus only."""
import math
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest
import yaml

from gripperx_arm import eeprom_limits as el
from gripperx_arm import wrap_offset as wo
from gripperx_arm.kinematics import TickCalibration
from gripperx_arm.servo_bus import ADDR_HOMING_OFFSET, ADDR_LOCK, ADDR_TORQUE

PKG = Path(__file__).resolve().parents[1]


def test_offset_encoding_sign_bit_11():
    assert wo.encode_offset(5) == 5
    assert wo.encode_offset(-5) == 5 | 2048
    for v in (-2047, -1, 0, 1, 2047):
        assert wo.decode_offset(wo.encode_offset(v)) == v
    with pytest.raises(ValueError):
        wo.encode_offset(2048)
    assert wo.normalise_offset(4000) == -96
    assert wo.normalise_offset(2048) is None


def test_plan_centres_the_wrap_in_the_gap_including_min_gt_max():
    # limb1 normal travel 1000..3000; limb2 crosses the wrap (min > max): 3800 -> 500
    rows = wo.plan([1000, 3800, 0, 0, 0, None], [3000, 500, 4095, 2000, 2000, None], 256)
    r1, r2, r3 = rows[0], rows[1], rows[2]
    assert r1['ok'] and r1['gap'] == 2096 and r1['shift'] == (3000 + 1048) % 4096
    assert r2['ok'] and r2['travel'] == (500 - 3800) % 4096 and r2['shift'] == (500 + r2['gap'] // 2)
    assert not r3['ok'] and 'gap' in r3['why']            # ~ full turn
    assert not rows[5]['ok'] and 'unknown' in rows[5]['why']
    # after the shift the recorded travel no longer crosses the wrap
    for r, lo, hi in ((r1, 1000, 3000), (r2, 3800, 500)):
        assert wo.shifted(lo, r['shift']) < wo.shifted(hi, r['shift'])


def test_round_trip_radians_identical_across_the_wrap():
    rng = np.random.default_rng(0)
    zero = [337, 2036, 2662, 3718, 1755, 1527]
    signs = [1, -1, 1, -1, 1, 1]
    joints = [f'j{i}' for i in range(6)]
    old = TickCalibration(joints, zero, signs)
    for shift in (1, 900, 2047, 2049, 3999):
        new = TickCalibration(joints, [wo.shifted(z, shift) for z in zero], signs)
        for _ in range(200):
            t = [int(v) for v in rng.integers(0, 4096, 6)]
            a = old.to_radians(t)
            b = new.to_radians([wo.shifted(v, shift) for v in t])
            for j in joints:
                assert math.isclose(math.remainder(a[j] - b[j], 2 * math.pi), 0.0, abs_tol=1e-12)


def test_patch_rewrites_every_stored_tick_once(tmp_path):
    cfg = tmp_path / 'config'
    shutil.copytree(PKG / 'config', cfg)
    shifts = {sid: 100 * sid for sid in wo.IDS}
    diff = wo.patch(cfg, shifts)
    p = yaml.safe_load((cfg / 'arm_poses.yaml').read_text())['arm_action_server'][
        'ros__parameters']
    c = yaml.safe_load((cfg / 'arm_calibration.yaml').read_text())['arm_joint_state_relay'][
        'ros__parameters']
    new_grip = [wo.shifted(t, shifts[i]) for i, t in zip(wo.IDS, p['poses']['grip'])]
    assert '+      grip: ' + wo._fmt(new_grip) in diff, diff
    assert ', '.join(str(wo.shifted(z, shifts[i])) for i, z in zip(wo.IDS, c['zero_ticks'])) in diff
    assert f"open: {wo.shifted(p['gripper']['open'], shifts[6])}" in diff
    assert f"close: {wo.shifted(p['gripper']['close'], shifts[6])}" in diff
    # nothing applied
    assert yaml.safe_load((cfg / 'arm_poses.yaml').read_text())['arm_action_server'][
        'ros__parameters']['poses']['grip'] == p['poses']['grip']


def test_patched_config_keeps_every_angle(tmp_path):
    """Apply the patch to a copy and compare angles of every stored pose."""
    ws = tmp_path / 'Software/ros2/src/gripperx_arm/config'
    shutil.copytree(PKG / 'config', ws)
    shifts = {1: 3000, 2: 17, 3: 2048, 4: 4095, 5: 1, 6: 1234}
    diff = wo.patch(ws, shifts)
    r = subprocess.run(['patch', '-p1', '-d', str(tmp_path)], input=diff, text=True,
                       capture_output=True)
    assert r.returncode == 0, r.stdout + r.stderr

    def angles(cfg):
        p = yaml.safe_load((cfg / 'arm_poses.yaml').read_text())['arm_action_server'][
            'ros__parameters']
        c = yaml.safe_load((cfg / 'arm_calibration.yaml').read_text())[
            'arm_joint_state_relay']['ros__parameters']
        cal = TickCalibration(c['urdf_joints'], c['zero_ticks'], c['signs'], 4096,
                              c.get('range_center_rad'))
        g, home = p['gripper'], c['zero_ticks']
        return [cal.to_radians(home), cal.to_radians(p['poses']['grip']),
                cal.to_radians(home[:5] + [g['open']]),
                cal.to_radians(home[:5] + [g['close']])]
    for a, b in zip(angles(PKG / 'config'), angles(ws)):
        for j in a:
            assert math.isclose(math.remainder(a[j] - b[j], 2 * math.pi), 0.0, abs_tol=1e-12)


class Bus:
    def __init__(self, torque=0, settle=0):
        self.actual = {sid: 1000 + sid for sid in wo.IDS}
        self.off = {sid: 0 for sid in wo.IDS}
        self.torque_v = torque
        self.settle = settle
        self.log = []

    def read_byte(self, sid, addr):
        assert addr == ADDR_TORQUE
        return self.torque_v

    def read_pos(self, sid):
        return (self.actual[sid] - self.off[sid]) % 4096

    def write_byte(self, sid, addr, v):
        self.log.append((sid, addr, v))

    def write_word(self, sid, addr, v):
        self.log.append((sid, addr, v))
        if addr == ADDR_HOMING_OFFSET:
            self.off[sid] = wo.decode_offset(v)
            self.actual[sid] += self.settle


def test_write_requires_torque_off_and_verifies_delta():
    rows = [{'id': sid, 'shift': 500 + sid} for sid in wo.IDS]
    bus = Bus(torque=1)
    assert wo.write(bus, rows, {s: 0 for s in wo.IDS}, 3)
    assert bus.log == [], 'wrote with torque on'
    bus = Bus()
    assert wo.write(bus, rows, {s: 0 for s in wo.IDS}, 3) == []
    seq = [e for e in bus.log if e[0] == 1]
    assert seq == [(1, ADDR_LOCK, 0), (1, ADDR_HOMING_OFFSET, 501), (1, ADDR_LOCK, 1)]
    bus = Bus(settle=20)                    # arm moved while held: verification fails
    assert wo.write(bus, rows, {s: 0 for s in wo.IDS}, 3)


def test_limit_generator_on_new_ticks_refuses_crossing():
    """Step 5: after the shift the EEPROM generator runs on the new zero ticks and
    still refuses a joint range that crosses the wrap."""
    lim = {n: (-0.3, 0.3) for n in wo.NAMES}
    zero = [100, 2048, 2048, 2048, 2048, 2048]
    rows = el.compute(lim, wo.NAMES, zero, [1] * 6, 4096, 0)
    assert not rows[0]['ok']
    new_zero = [wo.shifted(z, s) for z, s in zip(zero, [100 - 2048, 0, 0, 0, 0, 0])]
    rows = el.compute(lim, wo.NAMES, new_zero, [1] * 6, 4096, 0)
    assert all(r['ok'] for r in rows)
