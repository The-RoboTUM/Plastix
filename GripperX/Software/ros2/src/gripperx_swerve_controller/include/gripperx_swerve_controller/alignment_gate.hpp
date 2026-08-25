// Withhold drive while the steering modules are slewing into a new pose.
//
// WHAT THIS REPLACES, AND WHY IT MOVES HERE.
// gripperx_teleop/manoeuvre.py::TransitionGuard does this today, on the
// OPERATOR'S LAPTOP, for the keyboard teleop only. It works, and the reason it
// is being moved is not that it is broken:
//
//   * IT CANNOT SEE THE CONTROLLER'S TARGET, so it has to PREDICT it. manoeuvre.py
//     re-runs inverse_kinematics() + resolve_wheel_targets() in Python against a
//     mirrored copy of the geometry and the steering windows, purely to know
//     which angles to wait for. That is a SECOND IMPLEMENTATION of a
//     safety-relevant comparison, and its own docstring says so. In here there is
//     nothing to predict: the target was computed three lines up, in this cycle,
//     from these measured angles.
//   * IT ONLY GUARDS THE KEYBOARD. Nav2's crab recovery (gripperx_behaviors::
//     CrabWalk) commands vy with no guard at all, and nav2.yaml records the cost
//     from 16 twin runs: essentially the WHOLE heading error of a crab appears in
//     the two steering transients (-0.11 deg while vy was cut, -0.79 deg after
//     the steering unwound) and almost none in the steady lateral roll
//     (5e-5 .. 8e-4 rad/s). A guard here covers teleop, Nav2, the behaviors and
//     anything else that ever publishes a twist, because they all arrive as one.
//   * IT IS BYPASSABLE. It lives above /cmd_vel, so any other publisher simply
//     does not have it. Here it sits inside write_wheel_commands(), the single
//     funnel every branch of update() goes through — the same argument
//     stall_detector.hpp makes for HWR-30a's placement, and it transfers verbatim.
//
// GuardState::RELEASING HAS NO EQUIVALENT HERE, on purpose. It existed because
// the /teleop/direct_steer override lived in a DIFFERENT PROCESS (steer_servo_node),
// so the teleop could only wait out its timeout and hope. Since OP-23/A2-b the
// arbitration is inside update() and "is the override still winning" is an exact
// boolean (`direct_fresh`). A timeout that guesses at a fact we hold is not worth
// porting.
//
// WHAT THIS IS NOT: it is NOT a soft brake and it does NOT replace one.
// SwerveController::steer_alignment_scale() stays exactly as it is — tuned,
// hardware-accepted (NFR-10), and measuring residual TRACKING LAG in an ordinary
// corner. This class answers a different question: has the demanded GEOMETRY just
// JUMPED, i.e. are the modules currently travelling somewhere far from where they
// stand? The two compose: the gate holds the drive at exactly zero through a
// transition, the brake trims it afterwards. Removing the brake would be a
// separate decision and is not taken here.
//
// THE ONLY THING THIS CLASS CAN DO TO A WHEEL COMMAND IS REPLACE IT WITH EXACTLY
// 0.0 — no scaling, no bias, no error term. It never reads a measured VELOCITY.
// That keeps NFR-10 acceptance 10 ("the control law is demonstrably open-loop")
// untouched: a gate that can only zero cannot close a loop.
//
// SHIPS DISABLED (`enabled` false in the struct default AND in
// ros2_controllers.yaml), following WheelRegulator rather than StallDetector.
// The reason is not timidity: enabling it CHANGES HOW THE ROBOT DRIVES on the
// first deploy — a crab or spin entry that used to move immediately at the slew
// brake's floor now stands still for up to ~1 s first. That is the intended
// behaviour, and it is exactly why it should be switched on deliberately, in the
// twin, against the 16-run baseline, rather than arrive with a merge.
//
// NO rclcpp IN THIS HEADER OR ITS .cpp, exactly as swerve_kinematics,
// steering_limits, stall_detector and wheel_regulator — so the state machine is
// unit-checkable without a running stack (test/test_alignment_gate.cpp).

#ifndef GRIPPERX_SWERVE_CONTROLLER__ALIGNMENT_GATE_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__ALIGNMENT_GATE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "gripperx_swerve_controller/swerve_kinematics.hpp"

namespace gripperx_swerve_controller
{

/// Why the gate is doing what it is doing. PUBLISHED, not just logged (FR-14
/// item 7's lesson: a guard that quietly stops guarding looks exactly like one
/// that has nothing to do).
///
/// MONOTONE IN "the drive is flowing": kAlignDisabled and kAlignPassing let the
/// command through, kAlignSlewing does not, kAlignTimedOut does but says the
/// pose was never confirmed. Do not renumber.
enum AlignmentGateStatus : int
{
  /// Gate off in the configuration. The command is passed through untouched and
  /// this class is inert — the shipped default.
  kAlignDisabled = 0,
  /// Modules are where they are being told to go; drive flows.
  kAlignPassing = 1,
  /// The demanded pose jumped; drive is held at exactly zero until the modules
  /// arrive.
  kAlignSlewing = 2,
  /// Released on the timeout instead of on the measured angles. The drive flows,
  /// but "aligned" here means PROBABLY arrived, not KNOWN arrived, and the
  /// caller must be able to tell the two apart.
  kAlignTimedOut = 3,
};

struct AlignmentGateConfig
{
  /// Master switch. False in this default AND in ros2_controllers.yaml: merging
  /// and deploying this changes nothing about how the robot drives until someone
  /// turns it on. See the header comment for why that is not timidity.
  bool enabled{false};

  /// ENTRY, condition 1: the commanded pose moved by more than this on any wheel
  /// in ONE cycle. This is the direct analogue of TransitionGuard::request()
  /// returning true — "a transition just started" — derived from the twist
  /// instead of from a key press.
  ///
  /// 0.35 rad = 20 deg. TO-VERIFY. The number has to sit above the largest
  /// per-cycle target change an ordinary manoeuvre produces and below the
  /// smallest transition worth guarding. At the 100 Hz controller rate a DWB
  /// twist moves the target by a fraction of a degree per cycle, while a crab
  /// entry moves it by 90 deg in ONE cycle, so the two populations are three
  /// orders of magnitude apart and this value is nowhere near either edge. It is
  /// marked TO-VERIFY because that separation is ARGUED from the rates, not
  /// measured on this machine.
  double entry_jump_rad{0.35};

  /// ENTRY, condition 2: the pose ERROR on any wheel exceeds this, however it
  /// got that large.
  ///
  /// Condition 1 alone has a hole: it only sees CHANGES, so it cannot catch a
  /// controller that ACTIVATES with the wheels already at 90 deg from the first
  /// commanded pose, or wheels that were moved by hand, or the first cycle after
  /// a hold branch where the previous target is stale. This closes it.
  ///
  /// 0.50 rad = 28.6 deg. TO-VERIFY, and it MUST stay well above the steady
  /// tracking lag of an ordinary corner or the robot will refuse to drive
  /// through one. Deliberately larger than entry_jump_rad: a jump is evidence of
  /// a transition on its own, a standing error is only evidence once it is big.
  double entry_error_rad{0.50};

  /// EXIT: every wheel within this of its target. Carried over from the teleop
  /// guard's align_tolerance_rad, which is radians(6.0).
  ///
  /// MUST be smaller than both entry thresholds — that gap IS the hysteresis,
  /// and validate() refuses a configuration without it.
  double exit_tolerance_rad{0.10472};

  /// EXIT, fallback: release after this long regardless, reporting
  /// kAlignTimedOut. Carried over from the teleop guard's align_timeout_sec.
  ///
  /// A gate with no timeout is a gate that can immobilise the robot for ever on
  /// a steering-feedback fault, which is a worse failure than the one it
  /// prevents. Worst-case slew is ~140 deg at roughly 0.3-0.5 s per 90 deg.
  double timeout_sec{1.5};
};

struct AlignmentGateResult
{
  /// The wheel commands to actually write: `requested`, or ALL FOUR replaced by
  /// exactly 0.0 while slewing. Nothing else is ever modified, and single wheels
  /// are never zeroed individually — a partial pose is not a pose.
  std::array<double, kNumWheels> commands{};
  int status{kAlignDisabled};
  /// True on the cycle the status changed — the caller's cue to log or publish.
  /// Edge-triggered so a consumer cannot produce per-cycle spam.
  bool state_changed{false};
  /// Largest |target - measured| over the four wheels on this cycle, rad.
  /// Diagnostics: it changes every cycle while the modules slew, so a caller
  /// writing a status line to a raw terminal must not print it on change.
  double max_error_rad{0.0};
};

class AlignmentGate
{
public:
  AlignmentGate() = default;
  explicit AlignmentGate(const AlignmentGateConfig & config);

  /// Rejects a configuration that cannot behave — no hysteresis gap, a
  /// non-positive timeout — and explains why in `error`. A gate whose entry
  /// threshold sits at or below its exit threshold oscillates between holding
  /// and releasing the drive on one steady pose, which is worse than no gate.
  static bool validate(const AlignmentGateConfig & config, std::string & error);

  void configure(const AlignmentGateConfig & config);
  const AlignmentGateConfig & config() const { return config_; }

  /// Clears the state machine and the remembered target. Called on activate and
  /// deactivate: a gate must never come back from a lifecycle transition still
  /// holding the drive, and it must never come back believing the modules are
  /// where they were before the robot was switched off.
  void reset();

  /// One cycle.
  ///
  /// `requested` are the wheel commands the control law wants (rad/s, joint
  /// order, post-multiplier); `target_angles` are the steering angles being
  /// COMMANDED on this cycle and `measured_angles` those read from the state
  /// interfaces, both rad and joint order.
  ///
  /// `target_written` says whether the steering command interfaces are actually
  /// being written this cycle. Under OP-24/S1 "hold" means the ABSENCE of a
  /// write, and there is then no new target to align to — the gate keeps its
  /// state and passes the command through, because those branches have already
  /// zeroed the wheels for their own reasons.
  AlignmentGateResult update(
    double now_sec, const std::array<double, kNumWheels> & requested,
    const std::array<double, kNumWheels> & target_angles,
    const std::array<double, kNumWheels> & measured_angles, bool target_written);

  int status() const { return status_; }
  bool withholding() const { return status_ == kAlignSlewing; }
  /// Transitions guarded since the last reset. Published so that "the gate never
  /// fired" and "the gate is not running" cannot look alike.
  uint32_t engage_count() const { return engage_count_; }
  uint32_t timeout_count() const { return timeout_count_; }

private:
  /// Largest per-wheel |normalize_angle(target - measured)|.
  static double max_error(
    const std::array<double, kNumWheels> & target_angles,
    const std::array<double, kNumWheels> & measured_angles);

  AlignmentGateConfig config_{};

  int status_{kAlignDisabled};
  double slewing_since_sec_{0.0};
  double max_error_rad_{0.0};
  /// Last commanded target, and whether there is one at all. The flag is not a
  /// nicety: on the first cycle after reset there is no previous target, and a
  /// zero-initialised array would read as "the modules were commanded straight",
  /// which is a claim nobody made.
  std::array<double, kNumWheels> previous_target_{};
  bool previous_target_valid_{false};

  uint32_t engage_count_{0};
  uint32_t timeout_count_{0};
};

}  // namespace gripperx_swerve_controller

#endif  // GRIPPERX_SWERVE_CONTROLLER__ALIGNMENT_GATE_HPP_
