"""The follower's leader-mode activation tolerance, read from the follower's own config.

arm_action_server accepts leader mode only while every leader joint is within
leader.activation_match_tolerance_rad of the measured follower angle (user decision
2026-10-12: 10 deg). The leader's direct path and its clamp limit (user decision
2026-10-15) must use the SAME number, so it is never repeated here: it is read from
gripperx_arm/config/arm_safety_filter.yaml - the file arm_action_server is launched with.

The leader's virtual wall (wall.py, user decision 2026-10-15) sits at the follower's
leader command band: the joint limits (gripperx_geometry) narrowed by
leader.limit_command_margin_ticks of the same file, in the follower's tick calibration
(safety_filter.calibration_file; "" = arm_calibration.yaml beside the safety filter
file, which is where arm_action_server's "" resolves to in the installed share). The
band itself is computed by gripperx_arm.leader_band, the function the follower uses.
"""

import os
from typing import List, Optional, Tuple

import yaml

ARM_PACKAGE = 'gripperx_arm'
ARM_FILE = 'arm_safety_filter.yaml'
ARM_NODE = 'arm_action_server'
KEY = 'activation_match_tolerance_rad'
MARGIN_KEY = 'limit_command_margin_ticks'
CAL_FILE = 'arm_calibration.yaml'
RELAY_NODE = 'arm_joint_state_relay'


def default_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(ARM_PACKAGE), 'config', ARM_FILE)


def _load(path: Optional[str], node: str = ARM_NODE):
    """(path, ros__parameters of `node`) of the follower's safety filter file."""
    if not path:
        try:
            path = default_path()
        except Exception as exc:  # noqa: BLE001 - PackageNotFoundError and friends
            raise ValueError(f'package {ARM_PACKAGE} not found ({exc}): cannot read '
                             f'{ARM_FILE}') from exc
    try:
        with open(path) as fh:
            doc = yaml.safe_load(fh) or {}
    except OSError as exc:
        raise ValueError(f'{path}: {exc}') from exc
    return path, ((doc.get(node) or {}).get('ros__parameters') or {})


def _get(params: dict, group: str, key: str):
    return (params.get(group) or {}).get(key, params.get(f'{group}.{key}'))


def activation_tolerance(path: Optional[str] = None) -> float:
    """leader.activation_match_tolerance_rad of arm_action_server. Raises ValueError
    with the reason when the file or the key is missing or not a positive number."""
    path, params = _load(path)
    value = _get(params, 'leader', KEY)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0.0:
        raise ValueError(f'{path}: {ARM_NODE} leader.{KEY} missing or not a positive number '
                         f'({value!r})')
    return float(value)


def command_band(path: Optional[str] = None) -> List[Tuple[float, float]]:
    """The follower's leader command band for servo ids 1..6 (limb1..limb5, gripper), in
    rad: gripperx_arm.leader_band.command_band of the joint limits (gripperx_geometry),
    the follower tick calibration and leader.limit_command_margin_ticks. The gripper has
    a band like every other joint (user decision 2026-11-13). Raises ValueError with the
    reason."""
    from gripperx_arm.leader_band import command_band as band
    from gripperx_geometry.constants import ARM_JOINT_LOWER, ARM_JOINT_UPPER
    path, params = _load(path)
    margin = _get(params, 'leader', MARGIN_KEY)
    if isinstance(margin, bool) or not isinstance(margin, int) or margin < 0:
        raise ValueError(f'{path}: {ARM_NODE} leader.{MARGIN_KEY} missing or not a whole '
                         f'number >= 0 ({margin!r})')
    cal_path = _get(params, 'safety_filter', 'calibration_file') or os.path.join(
        os.path.dirname(os.path.abspath(path)), CAL_FILE)
    _, cal = _load(cal_path, RELAY_NODE)
    try:
        zero, signs, tpr = cal['zero_ticks'], cal['signs'], float(cal['ticks_per_rev'])
        return [band(ARM_JOINT_LOWER[i], ARM_JOINT_UPPER[i], float(zero[i]), int(signs[i]),
                     tpr, margin) for i in range(len(ARM_JOINT_LOWER))]
    except (KeyError, TypeError, IndexError, ValueError) as exc:
        raise ValueError(f'{cal_path}: follower calibration unusable for the command '
                         f'band: {exc!r}') from exc


def _leader_number(key: str, path: Optional[str]) -> Tuple[float, str]:
    path, params = _load(path)
    value = _get(params, 'leader', key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not value > 0.0:
        raise ValueError(f'{path}: {ARM_NODE} leader.{key} missing or not a positive number '
                         f'({value!r})')
    return float(value), path


def leader_max_velocity(path: Optional[str] = None) -> Tuple[float, str]:
    """(leader.max_joint_velocity_rad_s of arm_action_server, the file it came from).
    The follower's per-joint speed limit in leader mode; read by the recording analysis
    (follow_analysis), never repeated. Raises ValueError with the reason."""
    return _leader_number('max_joint_velocity_rad_s', path)


def leader_rate_hz(path: Optional[str] = None) -> Tuple[float, str]:
    """(leader.rate_hz of arm_action_server, the file it came from): the leader tick rate."""
    return _leader_number('rate_hz', path)
