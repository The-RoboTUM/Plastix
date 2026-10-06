# Layer 1 of the two-layer authority gate (SR-1).
#
# Layer 2 is the existing teleop_mux, which must be in `autonomous` before Nav2
# output reaches /cmd_vel at all, and which stays operator-owned.
#
# There is no field here that a launch argument, parameter or environment
# variable may set: an armed state can only ever come from an explicit
# SetArming service call at runtime.

std_msgs/Header header

bool armed

# Remaining arming window in seconds; 0.0 while disarmed. Arming ALWAYS has an
# explicit duration - there is no indefinite arm.
#
# THIS is the field to reason with. It is measured on a MONOTONIC clock
# (SAFETY.md F-29, SR-15 rule 12), so it is real elapsed seconds - the
# operator's seconds - whatever the ROS clock is doing.
float32 seconds_remaining

# The instant on the PUBLISHER'S ROS CLOCK at which the window closes, i.e. in
# the same timebase as `header.stamp` above.
#
# It is a PROJECTION, not a deadline: the window is measured monotonically and
# nothing in the arming gate reads this field. It is computed as
# `stamp + seconds_remaining * (observed ROS-to-monotonic rate)`, and the rate
# is what it used to be missing - before SAFETY.md F-37 it added wall seconds
# to a sim-time instant, which at a real-time factor of 0.1 advertised a gate
# closing ~595 wall-seconds away for one that closed in 60.2 s. On the real
# robot the ROS clock IS the wall clock, so the rate is 1.0 and the projection
# is exact.
#
# The rate is an estimate over the last few seconds, so a consumer that needs
# an exact number wants `seconds_remaining`. `/diagnostics` carries the rate
# used, as `external/clock` -> `ros_clock_rate`.
builtin_interfaces/Time expires_at

# Free text from the SetArming request, for the audit trail.
string armed_by

uint8 TRIGGER_NONE=0
uint8 TRIGGER_OPERATOR=1
uint8 TRIGGER_TIMEOUT=2
uint8 TRIGGER_LINK_LOST=3
uint8 TRIGGER_MODE_CHANGE=4        # covers the spacebar E-stop (set_mode=keyboard, SR-2)
uint8 TRIGGER_NAV2_UNAVAILABLE=5
uint8 TRIGGER_NODE_SHUTDOWN=6
uint8 TRIGGER_EXCESSIVE_ABORTS=7
uint8 TRIGGER_CLOCK_STALLED=8       # the ROS clock stopped advancing (SAFETY.md F-24)
# The ROS clock went BACKWARDS - a world reset, a /reset_simulation, an NTP
# step (SAFETY.md F-30). Its OWN constant, deliberately NOT folded into
# CLOCK_STALLED: a stall and a discontinuity are different conditions with
# different operator responses, and a consumer must be able to branch on that
# without parsing `last_disarm_detail` (SR-15 rule 7, decided by the user
# 2026-08-19; the wire-format change was the accepted cost). The BEHAVIOUR is
# identical to a stall - disarm, cancel, re-baseline; only the code differs.
uint8 TRIGGER_CLOCK_JUMPED_BACK=9
uint8 last_disarm_trigger
string last_disarm_detail

# True when disarming cancelled an active Nav2 goal and/or a running pick.
# Disarm never publishes zeros on a chain topic - that would make the gateway a
# second writer (SR-9). Stopping is Nav2's job.
bool last_disarm_cancelled

# Whether auto-pick may run in this build. Stubbed and false in the twin: there
# is no arm in the URDF or sim.
bool auto_pick_available
