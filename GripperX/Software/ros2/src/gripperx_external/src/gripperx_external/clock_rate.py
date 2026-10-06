"""The observed rate of the ROS clock against the monotonic clock.

WHY THIS EXISTS
===============
SAFETY.md F-37. ``ArmingState.expires_at`` is a ``builtin_interfaces/Time`` on
the node's ROS clock, and it used to be computed as ``_ros_now() +
seconds_remaining``. Since SAFETY.md F-29 those two are on DIFFERENT clocks:
``seconds_remaining`` is monotonic WALL seconds - the operator's seconds, which
is the entire point of that fix - while ``_ros_now()`` is a sim-time instant.
Adding one to the other silently assumes the two clocks run at the same rate.

Measured at a real-time factor of 0.1 (auditor's probe R1): a window with 60.0
wall-seconds left advertised an expiry ~595 wall-seconds away, because 60 SIM
seconds at 0.1x take ten minutes to elapse. The window itself closed correctly
after 60.2 s. So the GATE was right and the ADVERTISEMENT was wrong by a factor
of ten - and `expires_at` is on a ``TRANSIENT_LOCAL`` topic, i.e. it is the
latched audit record of the gate.

The conversion the arithmetic was missing is one number: how many ROS seconds
pass per monotonic second. The clock watchdog already sees both references on
every tick, so nothing new has to be measured - it only has to be REMEMBERED
between ticks, which is what this module does.

WHAT THIS IS NOT
================
**It is not a discontinuity detector.** A jump is not a rate, so a sample whose
implied rate is absurd is dropped from the ESTIMATE - and that is all that
happens to it: nothing here logs, reports, disarms or otherwise decides
anything about it. A forward jump is SAFETY.md F-40, which is recorded as OPEN
and awaiting a user decision (internal REQUIREMENTS SR-15 rule 12); this module must
not pre-empt that decision by growing a detector as a side effect of fixing a
projection.

The estimate is deliberately ADVISORY. Nothing in the authority gate reads it:
the arming window is measured monotonically and stays so (SR-15 rule 12), and
``seconds_remaining`` remains the field that needs no interpretation. This only
makes a reported instant honest.
"""

from __future__ import annotations

import math
from typing import Optional

#: Instantaneous samples outside this band are not rates. A real-time factor
#: below 1/1000 is a clock that has effectively stopped - which is the clock
#: watchdog's business, not this module's - and one above 1000 cannot be
#: produced by a simulation running fast; both are what a discontinuity looks
#: like when it is divided by a short interval.
_MIN_PLAUSIBLE_RATE = 1.0e-3
_MAX_PLAUSIBLE_RATE = 1.0e3

#: Samples shorter than this are dominated by scheduling jitter rather than by
#: the rate: at a watchdog period of 0.1 s, one late callback is a 50% error.
_MIN_SAMPLE_SEC = 0.02


class ClockRateEstimator:
    """ROS seconds per monotonic second, smoothed, from watchdog observations.

    Fed from the clock watchdog's healthy branch, which is the one place in the
    gateway that holds a ROS instant and a monotonic instant taken together.

    An exponential moving average with a TIME CONSTANT rather than a fixed
    weight, because the watchdog's period is derived from a parameter and a
    per-sample alpha would silently change meaning with it. ``tau_sec`` is the
    time the estimate takes to cover ~63% of a step in the real rate: long
    enough that jitter does not show, short enough that a twin whose load
    changes is tracked within a few seconds.

    Seeded at 1.0 - the real robot's value, where the ROS clock IS the wall
    clock - so there is no warm-up interval in which the field it feeds would
    have to be left unset.
    """

    def __init__(self, tau_sec: float = 2.0) -> None:
        self._tau_sec = max(1.0e-3, float(tau_sec))
        self._rate = 1.0
        self._prev_ros: Optional[float] = None
        self._prev_mono: Optional[float] = None
        self._samples = 0
        self._dropped = 0

    @property
    def rate(self) -> float:
        """The current estimate. Always finite and always positive."""
        return self._rate

    @property
    def samples(self) -> int:
        return self._samples

    @property
    def dropped(self) -> int:
        """Implausible samples excluded from the estimate. Diagnostic only."""
        return self._dropped

    def reset(self) -> None:
        """Forget the reference pair, keep the estimate.

        Called when the timeline itself changed (a backwards jump): the NEXT
        sample must not be measured across the discontinuity. The estimate is
        kept because the clock's rate before and after a world reset is the
        same thing - what changed is its value, not its speed.
        """
        self._prev_ros = None
        self._prev_mono = None

    def note(self, ros_sec: float, mono_sec: float) -> None:
        """One observation of both clocks, taken at the same instant."""
        if not (math.isfinite(ros_sec) and math.isfinite(mono_sec)):
            return
        prev_ros, prev_mono = self._prev_ros, self._prev_mono
        self._prev_ros, self._prev_mono = ros_sec, mono_sec
        if prev_ros is None or prev_mono is None:
            return
        d_mono = mono_sec - prev_mono
        d_ros = ros_sec - prev_ros
        if d_mono < _MIN_SAMPLE_SEC:
            # Not an error and not dropped: too short to carry information, so
            # it is simply not a sample yet. The reference above has already
            # advanced, which is what makes the next one long enough.
            return
        instantaneous = d_ros / d_mono
        if not (_MIN_PLAUSIBLE_RATE <= instantaneous <= _MAX_PLAUSIBLE_RATE):
            # See the module docstring: dropped from the ESTIMATE and nothing
            # more. This is not where a discontinuity gets decided (F-40).
            self._dropped += 1
            return
        alpha = 1.0 - math.exp(-d_mono / self._tau_sec)
        self._rate += alpha * (instantaneous - self._rate)
        self._samples += 1


def project_ros_expiry(ros_now_sec: float, seconds_remaining: float,
                       ros_rate: float) -> float:
    """The ROS-clock instant at which a WALL-clock window closes. F-37.

    ``seconds_remaining`` is monotonic wall seconds. In that many wall seconds
    the ROS clock advances ``seconds_remaining * ros_rate``, so that - and not
    ``seconds_remaining`` itself - is what may be added to a ROS instant.

    At a rate of 1.0 this is the old arithmetic exactly, which is why the real
    robot's behaviour is unchanged: there the ROS clock IS the wall clock.
    """
    rate = float(ros_rate)
    if not math.isfinite(rate) or rate <= 0.0:
        rate = 1.0
    remaining = float(seconds_remaining)
    if not math.isfinite(remaining) or remaining < 0.0:
        remaining = 0.0
    return float(ros_now_sec) + remaining * rate
