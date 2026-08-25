#!/usr/bin/env python3
"""A ``/clock`` publisher for the acceptance suite. Test fixture, never shipped.

WHY THIS EXISTS
===============
SAFETY.md F-24: the twin runs on sim time and every timer-driven safety
mechanism in the gateway is measured on the clock ``use_sim_time`` selects. The
acceptance suite runs against mocks rather than Gazebo, so there was no
``/clock`` in it at all - which is why the whole suite was forced to
``use_sim_time:=false`` and why every green result in SAFETY.md revisions 2 and
3 was taken in a configuration the launch file did not produce.

This node is the missing half. It stands in for Gazebo's clock publication so
the suite can exercise BOTH clock modes:

* running, so the arming expiry, the link watchdog and the in-flight re-checks
  can be shown to work in sim time and not merely in wall time;
* **frozen**, by setting ``paused``, which is a Gazebo that was paused or died
  with a goal in flight - the failure F-24 is actually about.
* **slow**, by setting ``scale`` below 1.0 - a loaded Gazebo at a low real-time
  factor. This is the case SAFETY.md F-29 is about: the clock is advancing, so
  every "has it moved?" test answers yes, while every timeout measured on it
  runs slower than wall time by the same factor. Nothing looks wrong anywhere.
* **discontinuous**, by setting ``jump_back_sec`` - a ``/reset_simulation`` or
  any Gazebo world reset, i.e. SAFETY.md F-30. The publisher keeps running
  perfectly; what breaks is the continuity of the timeline.
* **discontinuous the OTHER way**, by setting ``jump_forward_sec`` - an NTP step
  on a wall clock, or a simulation that was stepped on rather than run.
  SAFETY.md F-40 is about this direction and was SUSPECTED rather than
  reproduced precisely because this parameter did not exist: the auditor could
  argue from the code that a forward jump re-baselines the reference and PROVES
  the clock, but could not run it. The two directions are separate parameters
  rather than one signed one, so a scenario cannot ask for a backwards jump and
  get a forward one out of a sign slip.

``paused`` KEEPS PUBLISHING THE SAME VALUE on purpose. A clock that stops by
going silent is the easy case, because something is missing and absence is
detectable; a clock that keeps arriving at 50 Hz with a stamp that never moves
looks perfectly alive to everything except a monotonic reference. That is the
case the gateway's watchdog has to catch, so it is the case this produces.

The node itself runs on WALL time (``use_sim_time`` is left false here): a clock
source that is driven by the clock it publishes would freeze itself the moment
it was asked to pause.

WHERE SIM TIME STARTS IS ITSELF A TEST DIMENSION - ``--epoch-mode``
==================================================================
Added 2026-08-21 by user decision, on a finding from the first real-Gazebo
campaign: this fixture seeded sim time at ``time.time()``, while a real Gazebo
starts it at **0**. That difference is not cosmetic, and it HID a mechanism.

Measured against a real Gazebo (SAFETY.md F-35, `use_sim_time:=false` with a live
``/clock``): with sim time starting at 0 the two epochs are ~1.787e9 seconds
apart, so the TF lookup does not merely go stale, it FAILS - `TF_UNAVAILABLE: no
map -> base_footprint transform` - and the gateway refuses a goal it is armed
for. Fail-safe, but by a mechanism nobody predicted.

With sim time seeded near wall time the epochs are CLOSE, so that protection is
absent by construction: the lookup succeeds and the ages are small. And that is
not the exotic case - it is the one the whole finding was argued from. §6.4 item
8 names *"there is no /clock on the robot"* as the premise that stops being true
the moment somebody starts a **bag replay**, and a replay carries stamps near
wall time.

So both are kept and neither is the "right" one:

* ``--epoch-mode wall`` (DEFAULT, and the behaviour every existing caller
  already gets) - a bag replay, epochs close;
* ``--epoch-mode gazebo`` - a real Gazebo, epochs ~57 years apart.

The default is deliberately NOT changed to `gazebo`: the wall seed is what every
scenario in the suite has been exercised against, and swapping it would trade
one blind spot for another.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock as ClockMsg


class SimClock(Node):
    def __init__(self, rate_hz: float, epoch: float, scale: float = 1.0) -> None:
        super().__init__("sim_clock")
        # `paused` is settable at runtime - `ros2 param set /sim_clock paused
        # true` - which is how the suite freezes time under a running goal.
        self.declare_parameter("paused", False)
        self.declare_parameter("offset_sec", 0.0)
        # Real-time factor. 1.0 is a Gazebo keeping up; 0.1 is a loaded one, and
        # is what SAFETY.md F-29 was measured at.
        self.declare_parameter("scale", float(scale))
        # One-shot: sim time is set back by this many seconds on the next tick
        # and the parameter resets itself, so a world reset is a single event
        # rather than a state (SAFETY.md F-30).
        self.declare_parameter("jump_back_sec", 0.0)
        # The same one-shot in the other direction (SAFETY.md F-40). An NTP step
        # forward, or a world that was stepped rather than run. It exists so
        # that F-40's consequence can be OBSERVED instead of predicted; it does
        # not imply the gateway has, or should have, any behaviour for it.
        self.declare_parameter("jump_forward_sec", 0.0)
        self._epoch = epoch
        self._last = epoch
        # Sim time is ACCUMULATED from the monotonic delta rather than computed
        # from a start instant. That is what makes `scale` possible at all, and
        # it also means pausing cannot make sim time JUMP when it resumes: a
        # Gazebo that is unpaused continues, it does not skip the wall time it
        # spent paused. (The earlier version subtracted a paused total to get
        # the same property; accumulating gets it for free.)
        self._prev_mono = time.monotonic()
        self._paused_since = None
        self._publishes = 0
        # rclpy's TimeSource subscribes /clock BEST_EFFORT depth 1
        # (rclpy/time_source.py:71-75); matching it here rather than guessing.
        self._pub = self.create_publisher(
            ClockMsg, "/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"sim_clock up: publishing /clock at {rate_hz:.0f} Hz from epoch "
            f"{epoch:.3f} at scale {scale:.2f}. Set `paused:=true` to freeze it "
            "WITHOUT going silent, `scale` to run it slow (F-29), "
            "`jump_back_sec` to reset the world (F-30), `jump_forward_sec` to "
            "step it on (F-40)."
        )

    def _tick(self) -> None:
        paused = bool(self.get_parameter("paused").value)
        now = time.monotonic()
        elapsed = now - self._prev_mono
        self._prev_mono = now
        if paused:
            if self._paused_since is None:
                self._paused_since = now
                self.get_logger().warn(
                    f"/clock PAUSED at {self._last:.3f}; still publishing at full "
                    "rate, so only a monotonic reference can tell"
                )
        else:
            if self._paused_since is not None:
                self._paused_since = None
                self.get_logger().info(f"/clock RESUMED at {self._last:.3f}")
            self._last += elapsed * float(self.get_parameter("scale").value)
        jump_back = float(self.get_parameter("jump_back_sec").value)
        if jump_back > 0.0:
            # ONE-SHOT, and it fires whether or not the clock is paused: a world
            # reset is an event, not a mode. Clearing the parameter here is what
            # keeps it from repeating on every tick.
            self._last -= jump_back
            self.set_parameters([
                Parameter("jump_back_sec", Parameter.Type.DOUBLE, 0.0)
            ])
            self.get_logger().warn(
                f"/clock JUMPED BACK {jump_back:.1f}s to {self._last:.3f} - the "
                "publisher is fine, the timeline is not (SAFETY.md F-30)"
            )
        jump_forward = float(self.get_parameter("jump_forward_sec").value)
        if jump_forward > 0.0:
            # Same one-shot discipline as the backwards jump above.
            self._last += jump_forward
            self.set_parameters([
                Parameter("jump_forward_sec", Parameter.Type.DOUBLE, 0.0)
            ])
            self.get_logger().warn(
                f"/clock JUMPED FORWARD {jump_forward:.1f}s to {self._last:.3f} "
                "- indistinguishable from healthy progress to anything that only "
                "asks whether the clock advanced (SAFETY.md F-40)"
            )
        published = self._last + float(self.get_parameter("offset_sec").value)
        msg = ClockMsg()
        msg.clock.sec = int(published)
        msg.clock.nanosec = int(round((published - int(published)) * 1e9))
        self._pub.publish(msg)
        self._publishes += 1
        if paused and self._publishes % 100 == 0:
            self.get_logger().warn(
                f"/clock is PAUSED at {self._last:.3f} and still publishing at "
                "full rate - a consumer sees a live topic and a stopped clock"
            )


def resolve_epoch(epoch, epoch_mode, now_sec):
    """WHERE SIM TIME STARTS. Pure, so the offline checks can pin both epochs.

    Kept out of ``main`` deliberately: the difference between these two seeds
    hid a mechanism for the whole life of this fixture (see the module
    docstring), so it is worth being a named, testable decision rather than a
    conditional expression inside argument handling.

    An explicit ``epoch`` always wins, including **0.0** - which used to be the
    sentinel for "use the wall clock" and now means literally zero. The default
    remains ``wall``, so no existing caller changes behaviour.
    """
    if epoch is not None:
        return float(epoch)
    if epoch_mode == "gazebo":
        return 0.0
    return float(now_sec)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="real-time factor; 0.1 is a heavily loaded twin (SAFETY.md F-29)",
    )
    parser.add_argument(
        "--epoch-mode",
        choices=("wall", "gazebo"),
        default="wall",
        help="WHERE SIM TIME STARTS, which decides how far apart the two epochs "
             "are. 'wall' (default, unchanged) seeds it at time.time(), which is "
             "a BAG REPLAY - stamps land next to wall time. 'gazebo' seeds it at "
             "0.0, which is what a real Gazebo does. See the module docstring: "
             "the two produce different behaviour downstream and only one of "
             "them had ever been exercised (SAFETY.md F-35)",
    )
    parser.add_argument(
        "--epoch",
        type=float,
        default=None,
        help="explicit sim time at start, overriding --epoch-mode. NOTE: 0 now "
             "means LITERALLY ZERO. It used to be a sentinel for 'use the wall "
             "clock'; that is `--epoch-mode wall`, which is still the default, "
             "so no existing caller changes behaviour",
    )
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=[sys.argv[0]] + ros_args)
    epoch = resolve_epoch(args.epoch, args.epoch_mode, time.time())
    node = SimClock(args.rate_hz, epoch, args.scale)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
