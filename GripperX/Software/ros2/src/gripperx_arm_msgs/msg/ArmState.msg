# arm_action_server's own state, latched, ~2 Hz (with /arm/joint_states). The goal
# gateway dispatches no NavigateToPose unless the newest one says idle AND at_home and
# is fresh (ARM_NOT_HOME; audit AP1 finding H-pre, user decision 2026-10-13 option a).
# Published also when a servo does not answer: then known is false (audit AP3 F2).
std_msgs/Header header
# Every servo answered this read. false: at_home and gripper_at_home are false.
bool known
# No arm motion owns the arm (pick, command, leader, return home), leader mode off, no
# return home pending and no leader exit running.
bool idle
# The servos at their home ticks (arm_calibration.yaml zero_ticks) within
# POS_TOLERANCE, measured: limb1..limb5 (arm_state.at_home_includes_gripper false, the
# default - user decision 2026-10-13, audit AP3 F1 variant a: the jaws may hold the
# litter); limbs AND gripper when true.
bool at_home
# The gripper alone within POS_TOLERANCE of its home tick (reported either way).
bool gripper_at_home
bool torque_on
# The current motion owner ('' when none), for the log.
string owner
