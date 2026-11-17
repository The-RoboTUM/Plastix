"""The aimed P's TF intake: only what its lookups can use, at the least GIL cost.

Robot 2026-11-09: /tf carries ~100 msg/s, mostly wheel, steering and arm joint transforms
from robot_state_publisher that the aimed P never looks up. Deserialising one 8-transform
message costs ~0.6 ms of GIL time on the laptop (set_transform of all 8 ~0.1 ms), and a
KEEP_LAST 100 reader that once fell behind stayed ~1 s behind: odom -> base_footprint
read 1.05-1.16 s old in the arm while 12 ms old on the wire (STALE_TF).

So /tf is taken RAW and a message is deserialised only if one of the frames it may have
to feed occurs in it as a CDR string; of those, only transforms whose child is such a
frame reach the buffer. The frames are the parent chains of the frames looked up
(base_frame, still_frame, target_frames), learnt from the transforms themselves.
/tf_static is taken whole (every static is kept). A skipped message is one the buffer
could not have used: freshness is still the stamp's, never improved by a drop.

No executor: poll() takes whatever both queues hold (the call rclpy's executor makes per
message), so a 100 msg/s topic costs one GIL wake-up per poll instead of one wait-set
round per message (laptop, robot-like /tf: 19.6 % of a core before, 5.7 % polled).
Not thread-safe: one thread polls.
"""
import struct

_CDR_LE = (b'\x00\x01', b'\x00\x03')     # CDR / PL_CDR little endian encapsulation ids


def cdr_string(name: str) -> bytes:
    """A frame name as it is serialised (little endian): length incl. NUL, bytes, NUL."""
    data = name.encode()
    return struct.pack('<I', len(data) + 1) + data + b'\x00'


class TfIntake:

    MAX_TAKE = 200

    def __init__(self, buffer, frames, deserialize, authority='arm_action_server'):
        """buffer: a tf2 Buffer; frames: the frames looked up; deserialize: bytes ->
        TFMessage (rclpy.serialization.deserialize_message bound to TFMessage)."""
        self._buffer, self._deserialize, self._authority = buffer, deserialize, authority
        self.needed = set()                # frames whose parent transform is kept
        self.static_parent = {}            # child -> parent of every static transform
        self._needles = ()
        self.taken = self.skipped = 0      # /tf messages deserialised / skipped raw
        for f in frames:
            self._need(str(f))
        self._rebuild()

    def _need(self, frame):
        while frame and frame not in self.needed:
            self.needed.add(frame)
            frame = self.static_parent.get(frame)

    def _rebuild(self):
        # A frame with a static parent gets its transform from /tf_static: no needle,
        # else every robot_state_publisher message (parent base_link) would match.
        self._needles = tuple(cdr_string(f) for f in sorted(self.needed)
                              if f not in self.static_parent)

    def wants(self, raw: bytes) -> bool:
        """False only if no transform in the message can have a needed child."""
        if raw[:2] not in _CDR_LE:
            return True                    # unknown encoding: deserialise, never guess
        return any(n in raw for n in self._needles)

    def on_tf(self, raw: bytes):
        if not self.wants(raw):
            self.skipped += 1
            return
        self.taken += 1
        grown = False
        for t in self._deserialize(raw).transforms:
            if t.child_frame_id not in self.needed:
                continue
            try:
                self._buffer.set_transform(t, self._authority)
            except Exception:              # noqa: BLE001 - one bad transform must not end it
                continue
            if t.header.frame_id not in self.needed:
                self._need(t.header.frame_id)
                grown = True
        if grown:
            self._rebuild()

    def poll(self, tf_sub, static_sub):
        """Drain both subscriptions (/tf raw, /tf_static deserialised) once; at most
        MAX_TAKE messages each, so a topic arriving as fast as it is taken cannot hold
        the thread."""
        for sub, handle in ((static_sub, self.on_tf_static), (tf_sub, self.on_tf)):
            for _ in range(self.MAX_TAKE):
                with sub.handle:
                    taken = sub.handle.take_message(sub.msg_type, sub.raw)
                if taken is None:
                    break
                handle(taken[0])

    def on_tf_static(self, msg):
        for t in msg.transforms:
            try:
                self._buffer.set_transform_static(t, self._authority)
            except Exception:              # noqa: BLE001 - one bad transform must not end it
                continue
            self.static_parent[t.child_frame_id] = t.header.frame_id
        for child, parent in list(self.static_parent.items()):
            if child in self.needed:
                self._need(parent)
        self._rebuild()
