# Controller intent echo — §3.1.2 stage 4b, OP-18a / W2, SR-11.
#
# Published by swerve_controller from inside its update() cycle, i.e. from the
# controller_manager UPDATE LOOP and never from a callback serviced by the
# controller_manager EXECUTOR. That distinction is the whole point of this
# message: if the echo stopped when the executor wedged, the watchdog's
# divergence check would collapse into its silence check, which is exactly the
# regression option W1 was rejected for (OP-18a item 1).
#
# The watchdog (GripperXInterface) subscribes this together with /cmd_vel and
# discriminates:
#   silence    = /cmd_vel itself stale beyond command_timeout_sec
#   divergence = /cmd_vel fresh and changing while `sequence` here is frozen
#                beyond command_timeout_sec
# Neither check needs the kinematics — a twist is compared with a twist.

# stamp = the controller update() cycle in which this echo was produced.
# It advances every cycle even when nothing else does, so it is the liveness
# signal of the update loop.
std_msgs/Header header

# The /cmd_vel the controller last CONSUMED — not the last one published on the
# topic. On a wedged executor the two diverge, and that divergence is the fault
# being detected.
geometry_msgs/Twist twist

# Monotonic count of /cmd_vel messages the controller has received and consumed.
# It is deliberately a message count and not an update-cycle count: a frozen
# counter under a still-arriving echo says "the update loop runs but the
# subscription is not being serviced", which is the wedged-executor signature.
# A legitimately constant command still increments it, because every republished
# Twist is a new message.
uint64 sequence
