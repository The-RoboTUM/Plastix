"""joint_state_callback must not warn about messages that carry none of its joints.

/joint_states has two publishers on the real robot: joint_state_broadcaster
(wheels + steering) and arm_joint_state_relay (arm only). Before 2026-10-07 an
arm-only message counted as "missing joints" and every wheel message re-armed the
warning, i.e. ~2 WARN/s for as long as both ran.
"""
from types import SimpleNamespace

import pytest

node_mod = pytest.importorskip('gripperx_localization.localization_input_node')

DRIVE = ['fl_drive', 'fr_drive']
STEER = ['fl_steer', 'fr_steer']


class _Log:
    def __init__(self):
        self.warnings = []

    def warning(self, text):
        self.warnings.append(text)


def _stub():
    log = _Log()
    return SimpleNamespace(drive_joint_names=DRIVE, steer_joint_names=STEER,
                           missing_joint_warning_sent=False, get_logger=lambda: log), log


def _msg(names):
    return SimpleNamespace(name=names, position=[0.0] * len(names))


def test_message_without_any_required_joint_is_ignored_silently():
    stub, log = _stub()
    for _ in range(5):
        node_mod.LocalizationInputNode.joint_state_callback(stub, _msg(['arm_limb1_joint']))
        stub.missing_joint_warning_sent = False   # what a complete wheel message does
    assert log.warnings == []


def test_partial_message_still_warns_once():
    stub, log = _stub()
    for _ in range(3):
        node_mod.LocalizationInputNode.joint_state_callback(stub, _msg(['fl_drive', 'fl_steer']))
    assert len(log.warnings) == 1
    assert 'fr_drive' in log.warnings[0] and 'fr_steer' in log.warnings[0]
