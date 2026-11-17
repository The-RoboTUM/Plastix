# Whole-drive fault — user decision 2026-10-16.
#
# A wheel driven without a confirmed encoder, or a stall that did not stay
# temporary, stops ALL FOUR drive motors (velocity command exactly 0.0;
# steering untouched). The fault is LATCHED: no command, no controller
# deactivate/activate and no re-configure clears it. A stack restart clears
# every reason. The four STALL_* reasons can ALSO be cleared by an operator
# through the service /swerve_controller/clear_drive_fault
# (gripperx_control_msgs/srv/ClearDriveFault; SR-18 amendment 2026-11-09);
# ENCODER_MISSING and FEEDBACK_LOST stay restart-only. An operator clear is
# a motion release: a drive command still held drives off at once.
#
# PUBLISHED LATCHED (KeepLast(1), RELIABLE, TRANSIENT_LOCAL) by
# swerve_controller on /swerve_controller/drive_fault: once when the
# controller configures and activates (so a late subscriber always learns the
# current state, including "no fault"), on the cycle the fault latches, and
# once with active = false on the cycle an operator clear is applied.
# Never per cycle.

uint8 REASON_NONE = 0
# A wheel was commanded above encoder_confirm_min_command_rad_s for an
# accumulated encoder_confirm_window_sec while its provenance on
# /hw/wheel_feedback_valid was not LIVE.
uint8 REASON_ENCODER_MISSING = 1
# THE THREE STALL_* REASONS MEAN "commanded, encoder not moving": a stall OR a
# silent encoder -- NOT distinguishable without independent motion evidence
# (SR-13, 2026-09-22). They must not be read as a confirmed mechanical stall.
# A wheel re-tripped HWR-30a tier 1 within stall_escalation_window_sec of its
# latch clearing.
uint8 REASON_STALL_RETRIP = 2
# A wheel reached stall_escalation_trips tier-1 trips since activation.
uint8 REASON_STALL_TRIP_COUNT = 3
# A second wheel satisfied the stall criterion while one was latched
# (HWR-30 tier-2 territory). Also what a frozen /hw/joint_states looks like.
uint8 REASON_STALL_MULTI_WHEEL = 4
# A wheel stayed tier-1 latched for stall_escalation_latched_sec while its
# command was held above the release band (the OP-25 deadlock).
uint8 REASON_STALL_LATCHED_HELD = 5
# NOT one wheel: no wheel reports a LIVE encoder while driving (on the robot: a
# stale or absent /hw/joint_states makes GripperXInterface report UNKNOWN on all
# four; or the encoder supply is lost), or every wheel position froze on the same
# cycle. wheel_index is -1; other_wheel_index is the wheel whose criterion
# completed first.
uint8 REASON_FEEDBACK_LOST = 6

std_msgs/Header header

bool active

# Identity of the latched fault: a process-wide sequence number, new for every
# latch (a trip, or a fault adopted from the crash-surviving marker); kept when
# a re-loaded controller re-adopts the same fault; 0 when inactive. The clear
# service's expected_fault_id is compared against it.
uint32 fault_id

uint8 reason
# Same as `reason`, as text (e.g. "ENCODER_MISSING"), for humans and logs.
string reason_text

# Wheel that tripped it, joint order FL=0, FR=1, BL=2, BR=3; -1 when inactive.
int8 wheel_index
string wheel_name
# STALL_MULTI_WHEEL: the wheel that was already latched. FEEDBACK_LOST: the
# wheel whose criterion completed first. Else -1.
int8 other_wheel_index

# Controller time of the trip (zero when inactive).
builtin_interfaces/Time trip_stamp

# The tripping wheel's REQUESTED command on the trip cycle, rad/s.
float64 commanded_rad_s
# Its provenance code as used for the decision (-1 UNKNOWN, 0 NO_ENCODER,
# 1 INIT_FAILED, 2 LIVE_UNCONFIRMED, 3 LIVE).
int32 provenance
# ENCODER_MISSING: accumulated driven-without-LIVE time, s.
float64 accumulated_sec
# Stall reasons: that wheel's tier-1 trip count since activation.
uint32 stall_trip_count
# STALL_RETRIP: seconds from latch clear to re-trip, else -1.
float64 since_release_sec
# STALL_LATCHED_HELD: how long the latch was held under command, s.
float64 latched_held_sec

# One human-readable sentence, identical to the controller's ERROR line.
string detail

# ---- Operator clear (SR-18 amendment 2026-11-09). Valid whether or not a
# fault is active, so a page can tell "cleared" from "never faulted".
# Accepted operator clears since the controller PROCESS started (0 = none).
uint32 clear_count
# Controller time of the last accepted clear (zero when clear_count is 0).
builtin_interfaces/Time last_clear_stamp
# REASON_* of the fault the last accepted clear released (0 when none).
uint8 last_cleared_reason
# `requester` of the last accepted clear, as the caller claimed it (no
# authentication); "unspecified" when the caller sent none.
string last_clear_requester
