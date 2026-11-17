"""The aimed P's TF intake (tf_intake.py) without a node: which /tf messages are
deserialised, which transforms reach the buffer, statics. Frames from the ACTIVE
config (config/arm_safety_filter.yaml, aimed.*). Robot 2026-11-09: STALE_TF 1.05-1.16 s
from a full /tf queue of robot_state_publisher traffic the aimed P never looks up."""
import math
from pathlib import Path

import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer

from gripperx_arm.tf_intake import TfIntake

PKG = Path(__file__).resolve().parents[1]
AIMED = yaml.safe_load((PKG / 'config' / 'arm_safety_filter.yaml').read_text())[
    'arm_action_server']['ros__parameters']['aimed']
BASE, STILL = AIMED['base_frame'], AIMED['still_frame']
FRAMES = [BASE, STILL, *AIMED['target_frames']]
# The robot's tree (gripperx_description): base_footprint -> base_link static from the
# URDF, odom -> base_footprint from odometry, map -> odom from localisation.
FOOT = 'base_footprint'
DRIVE = [(BASE, f'{w}_steer_link') for w in ('fl', 'fr', 'rl', 'rr')] + \
        [(f'{w}_steer_link', f'{w}_wheel_link') for w in ('fl', 'fr', 'rl', 'rr')]


def tf(parent, child, sec, x=0.0):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec, t.header.stamp.nanosec = int(sec), int((sec % 1) * 1e9)
    t.transform.translation.x = x
    t.transform.rotation.w = 1.0
    return t


def raw(*transforms):
    return serialize_message(TFMessage(transforms=list(transforms)))


class Counting:
    def __init__(self):
        self.calls = 0

    def __call__(self, data):
        self.calls += 1
        return deserialize_message(data, TFMessage)


def make():
    buf, des = Buffer(), Counting()
    intake = TfIntake(buf, FRAMES, des)
    intake.on_tf_static(TFMessage(transforms=[tf(FOOT, BASE, 0)]))
    return buf, des, intake


def test_robot_state_publisher_traffic_is_never_deserialised():
    buf, des, intake = make()
    for k in range(200):
        intake.on_tf(raw(*(tf(p, c, 100 + k * 0.01) for p, c in DRIVE)))
        intake.on_tf(raw(*(tf(f'limb{i}', f'limb{i + 1}', 100 + k * 0.01)
                           for i in range(6))))
    assert des.calls == 0 and intake.skipped == 400
    assert not buf.can_transform(BASE, 'fl_wheel_link', Time())


def test_the_looked_up_chain_reaches_the_buffer_with_its_own_stamp():
    buf, des, intake = make()
    intake.on_tf(raw(tf(STILL, FOOT, 100.0, x=1.0)))
    intake.on_tf(raw(tf('map', STILL, 100.1)))
    intake.on_tf(raw(tf(STILL, FOOT, 100.5, x=2.0), *(tf(p, c, 100.5) for p, c in DRIVE)))
    t = buf.lookup_transform(STILL, BASE, Time())
    assert math.isclose(t.transform.translation.x, 2.0)
    assert (t.header.stamp.sec, t.header.stamp.nanosec) == (100, 500000000)
    for f in AIMED['target_frames']:
        assert buf.can_transform(BASE, f, Time()), f
    # the drive transforms of a message that was deserialised still stay out
    assert not buf.can_transform(BASE, 'fl_wheel_link', Time())


def test_parents_are_learnt_in_any_order():
    buf = Buffer()
    intake = TfIntake(buf, FRAMES, lambda d: deserialize_message(d, TFMessage))
    intake.on_tf(raw(tf(STILL, FOOT, 100.0)))          # FOOT not known yet: dropped
    assert not buf.can_transform(STILL, BASE, Time())
    intake.on_tf_static(TFMessage(transforms=[tf(FOOT, BASE, 0)]))
    intake.on_tf(raw(tf(STILL, FOOT, 100.2)))
    assert buf.can_transform(STILL, BASE, Time())


def test_every_static_is_kept():
    buf, _des, intake = make()
    intake.on_tf_static(TFMessage(transforms=[tf('limb6', 'camera', 0),
                                              tf(BASE, 'lidar', 0)]))
    assert buf.can_transform(BASE, 'lidar', Time())
    assert buf.can_transform('limb6', 'camera', Time())


def test_an_unknown_encoding_is_deserialised_not_guessed():
    _buf, des, intake = make()
    assert not intake.wants(raw(*(tf(p, c, 1.0) for p, c in DRIVE)))
    be = b'\x00\x00' + raw(*(tf(p, c, 1.0) for p, c in DRIVE))[2:]      # CDR big endian id
    assert intake.wants(be)


def test_a_frame_name_inside_a_longer_name_is_no_match():
    _buf, _des, intake = make()
    assert not intake.wants(raw(tf(BASE, f'x{STILL}', 1.0), tf(BASE, f'{STILL}_x', 1.0)))
