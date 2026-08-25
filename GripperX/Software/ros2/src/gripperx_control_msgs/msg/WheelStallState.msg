# Per-wheel drive-motor stall state — HWR-30a tier 1.
#
# A tier-1 cut-off switches ONE motor off while the other three keep driving, so
# the robot generally keeps moving and NOTHING UPSTREAM NECESSARILY NOTICES. That
# is precisely the failure SR-13 item 1 (rev 4) names: "motion, but degraded, and
# no message". This message is that message.
#
# PROVISIONAL CARRIER, and stated as such. SR-13 asks for ONE diagnostic channel
# carrying chain and drivetrain health as data, consumed by the OP-21 (b) display
# state — "one channel, not four". That health node does not exist yet. Until it
# does, this topic is where the tier-1 state lives; it is shaped as DATA (per
# wheel, no prose) so the health node can consume it rather than re-derive it,
# and it should be folded into that channel when the channel is built.
#
# PUBLISHED LATCHED (TRANSIENT_LOCAL) AND EDGE-TRIGGERED: on activation and then
# only when a latch changes, never every cycle. A late subscriber still learns
# the current state — the same reason /hw/wheel_feedback_valid is latched.

std_msgs/Header header

# Joint order FL, FR, BL, BR — the same order as the controller's
# wheel_joint_names parameter and as /hw/wheel_feedback_valid.
string[] joint_names

# True while the wheel is latched off by a tier-1 stall cut-off. Its velocity
# command is being forced to exactly 0.0; the other wheels and the steering are
# untouched.
bool[] latched

# True while the detector is actually watching this wheel, i.e. all three arming
# conditions hold: commanded above the arming threshold, encoder provenance LIVE,
# and a readable finite wheel position. Carried so that "the detector is asleep"
# can never be mistaken for "the detector is happy" — a disarmed detector is the
# normal state at standstill and an abnormal one while driving.
bool[] armed

# Number of tier-1 trips per wheel since the controller was activated. A wheel
# that trips repeatedly is a different fault from one that trips once.
uint32[] trip_count

# True when a trip was REFUSED because it would have latched more wheels than
# the single-motor tier-1 cap allows. "More than one motor" is HWR-30 tier 2,
# and tier 2 (HWR-30b) is blocked on the unmeasured GB37-50 stall current — so
# the condition is reported and NOT acted on. It is also what a lost
# /hw/joint_states looks like: every wheel position freezes at once.
bool multi_wheel_condition
