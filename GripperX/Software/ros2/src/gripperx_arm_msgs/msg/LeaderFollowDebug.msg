# Per-tick diagnostics of arm_action_server's leader mode (FR-17), published on
# /arm/leader/follow_debug once per leader tick while leader mode is ACTIVE.
# Read-only diagnostics: nothing subscribes to it for control.
# Arrays are in joint_names order; an array is EMPTY when this tick did not get that far.

std_msgs/Header header       # stamp: ROS clock at the start of the tick
string[] joint_names         # arm_limb1_joint .. arm_limb5_joint
float64[] setpoint           # rad: the commanded pose after this tick (written, or kept)
float64[] target             # rad: the clamped leader target the step was built from
float64[] measured           # rad: the positions this tick read from the servos
float64 tick_duration_s      # monotonic, tick start to its end (just after the servo write)
float64 period_s             # monotonic, since the previous tick start; 0 on the first
# written | unchanged (setpoint equals the held goals) | rejected (held) |
# holding (re-plan: searching / waiting) | follow_step (a planned path step) |
# activation (the tick that activated leader mode: gate checks, torque-on) | other
string outcome

# Where the tick's time went (added 2026-10-16), monotonic seconds per phase; 0 = the
# tick did not get there. The phases plus other_s add up to tick_duration_s.
float64 refusal_s            # gate checks: config, gateway, publisher counts, watchdog
float64 read_s               # reading the six servo positions, serial-lock wait included
float64 envelope_s           # steering envelope and the measured-pose check (SR-17 rule 8)
float64 replan_s             # held arm: re-plan bookkeeping, adopting a found path included
float64 target_s             # leader target, band clamp, rate limit and edge slowdown, ticks
float64 check_s              # safety filter on the step written (check_setpoints)
float64 write_s              # goal write of the limb servos, serial-lock wait included
float64 other_s              # tick_duration_s minus the phases above (activation, logging)
uint32 check_poses           # poses check_s checked: measured -> written, refined to
                             # safety_filter.max_joint_step_rad sub-steps
