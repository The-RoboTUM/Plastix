"""Fixed poses: home = calibration zero (one source), angle variants converted
through the calibration, converter refuses an unverified calibration."""
import math
from pathlib import Path

import pytest
import yaml

from gripperx_arm import poses as P
from gripperx_arm.kinematics import TickCalibration

PKG = Path(__file__).resolve().parents[1]
CAL = TickCalibration(['a', 'b', 'c', 'd', 'e', 'g'], [2108, 1003, 3091, 1035, 2148, 1527],
                      [1, -1, 1, -1, 1, 1], 4096, [0, -1.55, 1.68, -1.55, 0, 0])


def test_home_is_the_calibration_zero_and_poses_home_is_refused():
    r = P.resolve({'poses': {'grip': [1] * 6}, 'gripper': {'open': 5, 'close': 6}}, CAL)
    assert r['home'] == [2108, 1003, 3091, 1035, 2148, 1527]
    with pytest.raises(ValueError, match='one source'):
        P.resolve({'poses': {'home': [0] * 6, 'grip': [1] * 6},
                   'gripper': {'open': 5, 'close': 6}}, CAL)


def test_repository_config_has_one_home():
    poses = yaml.safe_load((PKG / 'config' / 'arm_poses.yaml').read_text())[
        'arm_action_server']['ros__parameters']
    assert 'home' not in poses['poses']
    cal = P.calibration_from_doc(yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml')
                                                .read_text()))
    assert P.resolve(poses, cal)['home'] == [int(z) for z in cal.zero_ticks]


def test_angle_variants_convert_and_round_trip():
    ticks = {'poses': {'grip': [2106, 3036, 1513, 2934, 2113, 2652]},
             'gripper': {'open': 2656, 'close': 1497}}
    rad = P.to_rad(ticks, CAL)
    back = P.resolve({'poses': {'grip_rad': rad['grip_rad']},
                      'gripper': {'open_rad': rad['open_rad'], 'close_rad': rad['close_rad']}},
                     CAL)
    ref = P.resolve(ticks, CAL)
    assert back['grip'] == ref['grip'] and back['open'] == ref['open'] \
        and back['close'] == ref['close']
    assert back['sources'] == {'grip': 'rad', 'open': 'rad', 'close': 'rad'}


def test_exactly_one_variant():
    with pytest.raises(ValueError, match='exactly one'):
        P.resolve({'poses': {'grip': [1] * 6, 'grip_rad': [0.0] * 6},
                   'gripper': {'open': 5, 'close': 6}}, CAL)
    with pytest.raises(ValueError, match='exactly one'):
        P.resolve({'poses': {'grip': [1] * 6}, 'gripper': {'open': 5}}, CAL)


def test_absolute_angle_maps_to_one_servo_position():
    a = P.rad_to_ticks(CAL, {'c': 3.8626})
    b = P.rad_to_ticks(CAL, {'c': 3.8626 - 2 * math.pi})
    assert a == b and 0 <= a['c'] < 4096


def _config_dir_with_flag(tmp_path, verified):
    for f in ('arm_calibration.yaml', 'arm_poses.yaml'):
        doc = yaml.safe_load((PKG / 'config' / f).read_text())
        if f == 'arm_calibration.yaml':
            doc['arm_joint_state_relay']['ros__parameters']['calibration_verified'] = verified
        (tmp_path / f).write_text(yaml.safe_dump(doc))
    return tmp_path


def test_converter_accepts_the_shipped_verified_calibration(capsys):
    cal = yaml.safe_load((PKG / 'config' / 'arm_calibration.yaml').read_text())
    assert cal['arm_joint_state_relay']['ros__parameters']['calibration_verified'] is True
    assert P.main(['--config-dir', str(PKG / 'config')]) == 0
    assert 'REFUSED' not in capsys.readouterr().out


def test_converter_refuses_unverified_calibration(tmp_path, capsys):
    cfg = _config_dir_with_flag(tmp_path, False)
    assert P.main(['--config-dir', str(cfg)]) == 1
    assert 'REFUSED' in capsys.readouterr().out
    assert P.main(['--config-dir', str(cfg), '--preview-unverified']) == 0
    out = capsys.readouterr().out
    assert 'PREVIEW ONLY' in out and 'grip_rad' in out
