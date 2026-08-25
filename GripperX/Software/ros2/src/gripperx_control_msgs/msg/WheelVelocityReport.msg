# Per-wheel commanded vs. measured wheel velocity — FR-11 items 1/2.
#
# Both halves travel in ONE message on purpose: FR-11 asks for them "alongside
# the commanded value, per wheel, so commanded and measured are comparable
# without correlating two topics by timestamp".
#
# The control law that produced `commanded` is OPEN-LOOP whenever
# `wheel_regulator_enabled` is false, which is its DEFAULT (FR-11 item 2,
# NFR-10 acceptance 10). `correction` and `regulating` at the end of this
# message are how that stops being a claim and becomes a reading: with the
# regulator off they are exactly 0.0 and false, on every wheel, every cycle.
#
# Provenance of `measured` (is it a measurement or the command echoed back?) is
# NOT carried here. It rides /hw/wheel_feedback_valid, latched, published by
# GripperXInterface — FR-11 items 5/6, already implemented. Duplicating it here
# would create a second, unlatched and possibly disagreeing copy.

std_msgs/Header header

# Joint order FL, FR, BL, BR — the same order as steer_servo.yaml and the
# controller's own wheel_joint_names parameter.
string[] joint_names

# rad/s, exactly the value written to the wheel VELOCITY command interface,
# i.e. AFTER wheel_command_multipliers. Post-multiplier on purpose: the state
# interface reports the joint's own axis convention, so a pre-multiplier value
# would not be comparable with `measured` on the sim's right-hand wheels.
float64[] commanded

# rad/s, read from the wheel velocity STATE interfaces this same cycle.
float64[] measured

# HWR-30a tier-1 stall state, per wheel, in the SAME message as commanded and
# measured — the natural home, because "commanded 7 rad/s, measured 0 rad/s" and
# "this wheel is latched off" are the two halves of one reading and separating
# them would force a consumer to correlate two topics by timestamp, which is the
# very thing FR-11 item 1 forbids for commanded/measured.
#
# `commanded` above is what was WRITTEN to the command interface, so a latched
# wheel reports commanded 0.0 here and `stall_latched[i] == true` says why. The
# authoritative, LATCHED copy of this state is
# /swerve_controller/stall_state (gripperx_control_msgs/WheelStallState); this
# one rides along at the report rate for whoever is already reading the report.
bool[] stall_latched

# Tier-1 trips per wheel since the controller was activated.
uint32[] stall_trip_count

# rad/s, the part of `commanded` that the per-wheel velocity regulator ADDED to
# the kinematic feedforward setpoint on this cycle. Exactly 0.0 on every wheel
# while `wheel_regulator_enabled` is false (the default).
#
# THIS FIELD IS NOT OPTIONAL DIAGNOSTICS, AND THE REASON IS A DOCUMENTED
# PROJECT INCIDENT. On 2026-08-17 wheel FL ran 27 % below BL under an identical
# command, and that deviation was the diagnostic signal that led the user to
# find a loose FL connection. A regulator would have compensated it: the wheel
# would simply have been driven harder, the reported velocities would have
# converged, and the loose connection would have stayed hidden until it failed
# outright. FR-11 therefore requires that when the loop is closed, "per-wheel
# control effort MUST be observable" — so the effort the regulator applies is
# carried as a first-class signal here, not left to be inferred from a
# difference between two other numbers.
#
# INVARIANT: kinematic setpoint == commanded[i] - correction[i]. It holds on
# every wheel EXCEPT one latched by HWR-30a: there the stall gate forces
# commanded to exactly 0.0 after the regulator has run, and both correction and
# regulating are reported as 0.0 / false because that is what the written
# command actually contains.
float64[] correction

# True while this wheel was actually being regulated on this cycle. False means
# pure feedforward.
#
# Carried for the same reason WheelStallState carries `armed`: "the regulator is
# not acting" must never be indistinguishable from "the regulator is acting and
# has nothing to correct". Both produce correction == 0.0.
bool[] regulating

# WHY, per wheel — FR-14 item 7 makes this binding, and acceptance A1/A3/A6/A8
# test it: "a regulator that quietly stops regulating one wheel looks exactly
# like a regulator that is working."
#
# REGULATOR_AT_AUTHORITY_LIMIT IS NOT A FAULT AND IS NOT A SUB-CASE OF "not
# regulating": the wheel IS regulated (`regulating[i]` is true) and its
# correction has reached the bound. It is the state that says THE FEEDFORWARD IS
# WRONG FOR THIS SURFACE — the OP-32 condition — and FR-14 requires it to be
# distinguishable from a regulator that simply has nothing to correct.
#
# REGULATOR_OFF_BELOW_FLOOR IS A STATEMENT ABOUT THE MACHINE, NOT ABOUT THE
# FEEDBACK, AND THAT DISTINCTION IS THE REASON IT IS ITS OWN CODE RATHER THAN A
# REUSE OF REGULATOR_OFF_NO_MEASUREMENT. Below the slow-end floor (FR-14 item
# 12) the plant is not proportional, not always reproducible and below the
# drop-out threshold does not move at all: there is nothing a gain can act on,
# whatever the feedback says. "The machine cannot be regulated here" and "the
# feedback is missing" are different findings with different remedies — the
# first is answered by the feedforward offset (OP-32 G1), the second by fixing
# an encoder — and FR-14 acceptance A17 requires the status to report THE
# reason.
#
# THE FLOOR IS NOT ITS OWN PARAMETER: it IS HWR-30a's arming threshold
# `stall_min_command_rad_s`, and the coupling is deliberate — the regulator may
# only add authority where the stall detector is watching. See the floor gate in
# wheel_regulator.cpp.
#
# REGULATOR_OFF_STALE_FEEDBACK IS A STATEMENT ABOUT INFORMATION, NOT ABOUT THE
# LINK, and it surprises on first reading: a machine standing perfectly still
# emits a feed in which nothing changes, and a changed value is the only evidence
# of a new frame a ros2_control state interface can carry. So a true standstill
# reports it legitimately. It has no control consequence there — at a zero
# setpoint the authority bound is zero anyway.
#
# Values MIRROR gripperx_swerve_controller::WheelRegulatorStatus and are
# static_asserted against it in swerve_controller.cpp. Change both or neither.
uint8 REGULATOR_DISABLED=0
uint8 REGULATOR_ACTIVE=1
uint8 REGULATOR_AT_AUTHORITY_LIMIT=2
uint8 REGULATOR_OFF_PROVENANCE=3
uint8 REGULATOR_OFF_NO_MEASUREMENT=4
uint8 REGULATOR_OFF_STALE_FEEDBACK=5
uint8 REGULATOR_OFF_STALL_LATCHED=6
uint8 REGULATOR_OFF_BELOW_FLOOR=7
uint8[] regulator_status

# ── Steering alignment gate ─────────────────────────────────────────────────
#
# Whether the drive is currently being WITHHELD because the steering modules are
# still travelling into a new pose (gripperx_swerve_controller/alignment_gate.hpp).
#
# IT RIDES HERE RATHER THAN ON ITS OWN TOPIC for the same reason the stall state
# does: "commanded 0.0 rad/s" and "because the modules are still slewing" are two
# halves of one reading, and separating them would force a consumer to correlate
# two topics by timestamp. A reader who sees `commanded` collapse to zero on all
# four wheels at once finds the reason in the same message.
#
# ALIGNMENT_TIMED_OUT IS NOT A FAULT CODE, and it is the one value that needs
# care: the drive IS flowing in that state. It says the gate released on its
# timeout rather than on the measured angles, i.e. the pose is PROBABLY reached
# and not KNOWN to be. On this robot that distinction is real — /hw/steer_states
# reports the SERVO, not the wheel, and OP-29 measured up to 17 deg of mechanical
# play between the two in a crab.
#
# Values MIRROR gripperx_swerve_controller::AlignmentGateStatus and are
# static_asserted against it in swerve_controller.cpp. Change both or neither.
uint8 ALIGNMENT_DISABLED=0
uint8 ALIGNMENT_PASSING=1
uint8 ALIGNMENT_SLEWING=2
uint8 ALIGNMENT_TIMED_OUT=3
uint8 alignment_status

# Largest |target - measured| steering error over the four wheels this cycle,
# rad. Diagnostics only, and it moves every cycle while the modules slew, so a
# consumer writing an on-change status line must not key it off this field.
float64 alignment_max_error_rad

# WHAT THE CONTROLLER THINKS IT IS DOING, classified from the CONSUMED TWIST.
#
# DERIVED, NEVER COMMANDED, and that distinction is the whole design. An earlier
# option had the manoeuvre arrive as its own topic so the controller could be put
# into "crab mode"; it was rejected because a twist (0, vy, 0) IS a crab, and
# encoding the same intent twice creates two sources of truth that can disagree
# about what the robot is doing. Nothing in the control path reads this field —
# it exists so the teleop can show the operator a name without owning the pose
# maths, and so a Nav2 log can say which manoeuvre a recovery actually produced.
uint8 MANOEUVRE_IDLE=0
uint8 MANOEUVRE_CORNERING=1
uint8 MANOEUVRE_CRAB=2
uint8 MANOEUVRE_SPIN=3
uint8 MANOEUVRE_GENERAL=4
uint8 manoeuvre
