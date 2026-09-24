#!/usr/bin/env python3
"""
Run the teleop UI with no ROS and no robot — a bench for the page itself.

Why this exists
---------------
The UI is the part of teleop most likely to be changed for cosmetic reasons,
and the least sensible thing to test by driving a real robot around. This
harness serves the same `web/` assets over the same HTTP API as
`web_teleop_node`, so the page cannot tell the difference, and drives them from
a simulated chassis.

It is NOT a mock of the control logic. The manoeuvre selection, the pose
computation and the `TransitionGuard` below are imported from `manoeuvre.py` —
the same code the robot runs — because they happen to be pure Python. Only
three things are faked: the ROS publishers (dropped), the steering servos
(slewed at a fixed rate towards the commanded pose) and the arm/gate services
(logged as events). So what you see on the page is the real state machine
reacting to real key input; only the hardware at the far end is imaginary.

Nothing here is on the robot's path. It is a bench, and it says so on the page.

    python3 test/web_ui_demo.py [--port 8080] [--host 127.0.0.1]
"""
import argparse
import math
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(_HERE)
_CONTROL = os.path.normpath(os.path.join(_SRC, '..', 'gripperx_control', 'src'))
for path in (_SRC, _CONTROL):
    if path not in sys.path:
        sys.path.insert(0, path)

from gripperx_control.steering_limits import (          # noqa: E402
    DEFAULT_INWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_LIMIT_DEG,
    DEFAULT_OUTWARD_SIGN,
    SteeringLimits,
)
from gripperx_control.swerve_kinematic_model import (   # noqa: E402
    FourWIS4WIDKinematicModel,
)
from gripperx_teleop.manoeuvre import (                 # noqa: E402
    CORNERING,
    HUMAN_LABEL,
    KEY_TO_MANOEUVRE,
    MANOEUVRE_KEY_PRECEDENCE,
    TransitionGuard,
    manoeuvre_pose,
    manoeuvre_twist,
)
from gripperx_teleop.web_server import TeleopWebServer  # noqa: E402

STEER_PATTERN = (1.0, 1.0, -1.0, -1.0)
HOLDABLE = frozenset(('w', 's', 'a', 'd') + tuple(KEY_TO_MANOEUVRE))

# Simulated servo slew rate. The real modules manage roughly 90 deg in
# 0.3-0.5 s, so ~4 rad/s is the optimistic end of measured behaviour.
SERVO_RATE_RAD_S = 4.0

TICK_HZ = 50.0


class DemoChassis:
    """Everything the page needs, computed without a robot attached."""

    def __init__(self, host, port):
        self.limit = math.radians(35.0)
        self.lin_vel = 0.5
        self.crab_speed = 0.25
        self.spin_speed = 1.4
        self.drive_hold = 0.6
        self.pose_scale = 0.02

        self.model = FourWIS4WIDKinematicModel(a=0.203, b=0.16556, wheel_radius=0.070)
        self.limits = SteeringLimits.from_outward_inward(
            math.radians(DEFAULT_OUTWARD_LIMIT_DEG),
            math.radians(DEFAULT_INWARD_LIMIT_DEG),
            list(DEFAULT_OUTWARD_SIGN),
        ).in_model_order()
        self.guard = TransitionGuard(
            release_sec=0.7, align_timeout_sec=1.5,
            align_tolerance_rad=math.radians(6.0),
        )

        self.lock = threading.Lock()
        self.key_t = {k: 0.0 for k in HOLDABLE}
        self.steer = 0.0
        self.measured = [0.0, 0.0, 0.0, 0.0]     # simulated servo angles
        self.snap = {}
        self.events = []
        self.event_seq = 0

        self.client_keys = frozenset()
        self.client_seen = 0.0
        self.client_timeout = 0.5
        self.estop_latched = False
        self.driver = None
        self.gate_until = 0.0

        self.server = TeleopWebServer(
            sink=self.on_input, snapshot=self.telemetry,
            host=host, port=port, stream_hz=20.0,
        )
        self.stop = threading.Event()

    # -- events ---------------------------------------------------------

    def event(self, text, level='info'):
        with self.lock:
            self.event_seq += 1
            self.events.append({
                'id': self.event_seq, 'level': level, 'text': text,
                'wall': time.strftime('%H:%M:%S'),
            })
            del self.events[:-60]

    # -- input ----------------------------------------------------------

    def on_input(self, session, keys, events, has_control):
        now = time.monotonic()
        wanted = frozenset(k for k in keys if k in HOLDABLE)

        for name in events:
            if name == 'estop':
                self.do_estop(now)
            elif not has_control:
                continue
            elif name == 'quit':
                self.event('shutdown requested from the UI', 'alarm')
                self.stop.set()
            elif name == 'arm_gate':
                self.gate_until = now + 120.0
                self.event('external gate ARMED (simulated, 120 s)', 'ok')
            elif name == 'disarm_gate':
                self.gate_until = 0.0
                self.event('external gate DISARMED (simulated)', 'ok')
            else:
                self.event(f'{name} (simulated — no ROS behind this bench)')

        if not has_control:
            return

        cleared = False
        with self.lock:
            if self.estop_latched:
                if wanted:
                    wanted = frozenset()
                else:
                    self.estop_latched = False
                    cleared = True
            self.client_keys = wanted
            self.client_seen = now
            self.driver = session
        # Outside the lock on purpose: event() takes the same lock, and
        # threading.Lock is not reentrant.
        if cleared:
            self.event('E-STOP cleared, control returned', 'ok')

    def do_estop(self, now):
        with self.lock:
            was_manoeuvring = self.guard.manoeuvre != CORNERING
            for key in self.key_t:
                self.key_t[key] = 0.0
            self.steer = 0.0
            self.client_keys = frozenset()
            self.estop_latched = True
            self.guard.force_cornering(now, rearm=not was_manoeuvring)
        self.event('EMERGENCY STOP — release every key to regain control', 'alarm')

    # -- tick -----------------------------------------------------------

    def held(self, key, window=0.15):
        return (time.monotonic() - self.key_t[key]) < window

    def active_manoeuvre(self, now):
        chosen, chosen_t = None, 0.0
        for key in MANOEUVRE_KEY_PRECEDENCE:
            stamp = self.key_t[key]
            if (now - stamp) < self.drive_hold and stamp > chosen_t:
                chosen, chosen_t = key, stamp
        return KEY_TO_MANOEUVRE[chosen] if chosen is not None else CORNERING

    def tick(self, dt):
        now = time.monotonic()
        with self.lock:
            fresh = (now - self.client_seen) <= self.client_timeout
            if fresh:
                # Same mutual exclusion the node's press() enforces: a
                # manoeuvre key beats W/S, and W/S beats a manoeuvre key.
                keys = set(self.client_keys)
                if keys & set(KEY_TO_MANOEUVRE):
                    keys -= {'w', 's'}
            else:
                keys = set()
            for key in self.key_t:
                # Held keys are refreshed; everything else is actively let go
                # rather than left to age out of the dead-man window. A
                # browser reports releases, so acting on them is honest --
                # and a silent link releases everything, same as the node.
                self.key_t[key] = now if key in keys else 0.0

            manoeuvre = self.active_manoeuvre(now)
            if self.guard.request(manoeuvre, now) and manoeuvre != CORNERING:
                self.steer = 0.0

            if self.held('a'):
                self.steer = min(self.limit, self.steer + 0.6 * dt)
            if self.held('d'):
                self.steer = max(-self.limit, self.steer - 0.6 * dt)

            if manoeuvre == CORNERING:
                drive = 1 if self.held('w', self.drive_hold) else (
                    -1 if self.held('s', self.drive_hold) else 0)
                target = [self.steer * factor for factor in STEER_PATTERN]
            else:
                drive = 0
                target = manoeuvre_pose(
                    manoeuvre, self.model, self.limits,
                    self.crab_speed, self.spin_speed, self.measured,
                )

            self.guard.update(now, target, self.measured)
            armed = self.guard.drive_allowed
            pose_on = self.guard.pose_commanded

            # Simulated servos: slew towards whatever was commanded. During
            # RELEASING nothing is commanded yet, so they stay put — which is
            # exactly the delay the guard exists to cover.
            if target is not None and (manoeuvre == CORNERING or pose_on):
                step = SERVO_RATE_RAD_S * dt
                for i, want in enumerate(target):
                    error = want - self.measured[i]
                    self.measured[i] += max(-step, min(step, error))

            vx = vy = wz = 0.0
            if manoeuvre == CORNERING:
                vx = self.lin_vel * drive if armed else 0.0
            elif pose_on:
                twist = manoeuvre_twist(manoeuvre, self.crab_speed, self.spin_speed)
                scale = 1.0 if armed else self.pose_scale
                vx, vy, wz = (component * scale for component in twist)

            self.snap = {
                'manoeuvre': manoeuvre,
                'manoeuvre_label': HUMAN_LABEL.get(manoeuvre, manoeuvre),
                'status': self.guard.describe(),
                'guard_state': self.guard.state,
                'drive_allowed': bool(armed),
                'pose_commanded': bool(pose_on),
                'pose_reachable': bool(self.guard.pose_reachable),
                'armed_without_feedback': bool(self.guard.armed_without_feedback),
                'steer_deg': math.degrees(self.steer),
                'target_deg': ([math.degrees(v) for v in target]
                               if target is not None else None),
                'measured_deg': [math.degrees(v) for v in self.measured],
                'cmd': {'vx': vx, 'vy': vy, 'wz': wz},
            }

    def telemetry(self):
        now = time.monotonic()
        with self.lock:
            frame = dict(self.snap)
            frame['events'] = list(self.events)
            silent = now - self.client_seen
            fresh = self.client_seen > 0.0 and silent <= self.client_timeout
            frame.update({
                'held': sorted(self.client_keys) if fresh else [],
                'link_fresh': fresh,
                'silent_sec': round(silent, 3) if self.client_seen > 0.0 else None,
                'estop_latched': self.estop_latched,
                'driver': self.driver,
                'mode': 'keyboard (bench)',
                'gate': {
                    'armed': self.gate_until > now,
                    'seconds_remaining': max(0.0, self.gate_until - now),
                    'armed_by': 'bench',
                    'last_disarm_detail': '',
                },
                'limits': {
                    'steer_limit_deg': math.degrees(self.limit),
                    'linear_vel_m_s': self.lin_vel,
                    'crab_speed_m_s': self.crab_speed,
                    'spin_speed_rad_s': self.spin_speed,
                    'drive_hold_sec': self.drive_hold,
                    'align_tolerance_deg': math.degrees(
                        self.guard.align_tolerance_rad),
                },
                'geometry': {
                    'a': self.model.a, 'b': self.model.b,
                    'wheel_radius': self.model.wheel_radius,
                },
            })
        return frame

    # -- lifecycle ------------------------------------------------------

    def run(self):
        self.server.start()
        self.event('bench mode — no ROS, no robot, simulated servos', 'warn')
        print(f'Teleop UI bench: {self.server.url}   (Ctrl+C to stop)')
        dt = 1.0 / TICK_HZ
        try:
            while not self.stop.is_set():
                self.tick(dt)
                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.server.stop()
            print('\nbench stopped.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    DemoChassis(args.host, args.port).run()


if __name__ == '__main__':
    main()
