// Encoder-based drive-motor stall detection and the tier-1 response — HWR-30a.
//
// Not in firmware: HWR-30's written text places the cut-off in the ESP32
// drive firmware; this is a recorded DEVIATION, built on the ROS2 side
// instead, without current measurement, so it does not wait on HWR-30b and
// the outstanding GB37-50 stall-current measurement (WIRING_PLAN §8.5).
//
// Not in the hardware interface: GripperXInterface is replaced by
// gz_ros2_control/GazeboSimSystem in the twin (§3.1.6), so a detector placed
// there would be structurally UNTESTABLE in simulation. In the controller it
// runs identically in sim and on the robot, and shares its provenance gate
// and subscription with the wheel velocity regulator (wheel_regulator.hpp),
// which exists as structure and is disabled by default.
//
// WHAT THIS IS NOT: not a wheel velocity regulator. No error term, integrator
// or gain acts on the measured wheel velocity HERE. The control law stays
// open-loop feedforward unless wheel_regulator_enabled is turned on (FR-11
// item 2, NFR-10 acceptance 10). The only thing this class can do to a wheel
// command is replace it with EXACTLY ZERO — the LAST stage in the chain, after
// the regulator, so that zero is never trimmed by anything
// (SwerveController::write_wheel_commands).
//
// NO rclcpp IN THIS HEADER OR ITS .cpp, on purpose — the same property
// swerve_kinematics and steering_limits have, and why the whole state machine
// is unit-checkable without a running stack (test_stall_detector.cpp).

#ifndef GRIPPERX_SWERVE_CONTROLLER__STALL_DETECTOR_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__STALL_DETECTOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "gripperx_swerve_controller/swerve_kinematics.hpp"

namespace gripperx_swerve_controller
{

// Provenance of the per-wheel encoder feedback, as published latched on
// /hw/wheel_feedback_valid (FR-11 items 5/6). MIRRORS
// gripperx_hardware_interface::WheelFeedbackProvenance and, for codes 0..3, the
// ESP32 firmware's EncoderStatus — change all three or none.
//
// MONOTONE IN CONFIDENCE, and that is the property this file relies on: a
// zero-filled or absent array degrades to "not a measurement" rather than to
// "healthy". Do not renumber.
enum StallProvenance : int
{
  kStallProvenanceUnknown = -1,
  kStallProvenanceNoEncoder = 0,
  kStallProvenanceInitFailed = 1,
  kStallProvenanceLiveUnconfirmed = 2,
  kStallProvenanceLive = 3,
};

struct StallDetectorConfig
{
  /// Master switch. When false the detector is inert AND the controller does
  /// not claim the wheel POSITION state interfaces at all — the escape hatch
  /// for a hardware component that does not export them.
  bool enabled{true};

  /// All three of (command non-zero, provenance Live, position not moving) must
  /// hold continuously for this long before the wheel is latched off.
  double window_sec{1.0};

  /// Arming threshold of the NET-DISPLACEMENT criterion on |commanded wheel
  /// angular velocity|, rad/s. At or below it that criterion does not judge.
  /// TO-VERIFY, kept in step with stall_min_command_rad_s in
  /// ros2_controllers.yaml. Since 2026-10-22 it is no longer the wheel
  /// regulator's slow-end floor (WheelRegulatorConfig::floor_rad_s), which must
  /// not lie below it — WheelRegulator::validate_floor refuses that.
  double min_command_rad_s{2.0};

  /// How much the ACCUMULATED wheel position (rad) must change inside the
  /// window for the wheel to count as turning. One encoder count is
  /// 2*pi/COUNTS_PER_OUTPUT_REV = 2*pi/3200 = 0.00196 rad (the constant is
  /// MEASURED and confirmed to ~1 %, not assumed), so the default is 25.5
  /// counts. Full rationale in ros2_controllers.yaml.
  double min_position_delta_rad{0.05};

  /// OP-25 release band: |command| must fall to or below this before a rise
  /// back above the latching criterion's arming threshold (min_command_rad_s,
  /// or silent_min_command_rad_s for a zero-count latch) counts as a FRESH
  /// command (user decision 2026-10-29, option a). Must be below both.
  double release_command_rad_s{0.1};

  /// ZERO-COUNT CRITERION (user decision 2026-10-22, option c). A LIVE wheel
  /// whose |requested command| is above this and whose position shows NO
  /// encoder count change at all for an accumulated silent_window_sec trips —
  /// the same tier-1 latch as the net-displacement criterion. NOT a parameter of
  /// its own: the controller feeds encoder_confirm_min_command_rad_s, the one
  /// number meaning "a healthy wheel commanded above this at least twitches,
  /// and a twitch is a count". Must be <= min_command_rad_s.
  double silent_min_command_rad_s{0.71};

  /// TO-VERIFY. Accumulated watched time with no count change before the
  /// zero-count criterion trips, seconds. Accumulates while the wheel is LIVE,
  /// commanded above silent_min_command_rad_s and the drive is not withheld;
  /// pauses otherwise; resets on any count change, on leaving LIVE, on a latch
  /// and on reset().
  double silent_window_sec{2.0};

  /// One encoder count in wheel radians: 2*pi/COUNTS_PER_OUTPUT_REV =
  /// 2*pi/3200 (firmware include/motor_controller.hpp, confirmed to ~1 % by a
  /// rolling measurement). A "count change" is a position change of at least
  /// HALF of this from the position at the last change.
  double count_rad{2.0 * 3.14159265358979323846 / 3200.0};

  /// Tier-1 IS A SINGLE-MOTOR RESPONSE (HWR-30). A trip that would take the
  /// number of simultaneously latched wheels above this cap is REFUSED and
  /// reported instead: "more than one motor" is tier 2, and tier 2 (HWR-30b) is
  /// blocked on the unmeasured stall current. See the comment in stall_detector.cpp.
  std::size_t max_latched_wheels{1};

  /// Sim policy. Nothing publishes /hw/wheel_feedback_valid in the twin, so the
  /// provenance gate would keep the detector permanently disarmed there. TRUE
  /// only in swerve_controller.sim.yaml; FALSE on the real robot, where a
  /// missing provenance topic MUST disarm rather than assume.
  bool assume_live_provenance{false};
};

/// What happened to one wheel on one cycle. All three are EDGE flags: they are
/// true on the cycle of the transition and false afterwards, so a caller that
/// logs them cannot produce per-cycle spam.
struct StallWheelEvent
{
  bool tripped{false};            ///< latched off on this cycle
  bool released{false};           ///< latch cleared by a fresh command on this cycle
  /// Would have tripped, but that is tier 2 — see config.max_latched_wheels.
  bool multi_wheel_refused{false};
  /// Qualifies `tripped` / `multi_wheel_refused`: it was the ZERO-COUNT
  /// criterion (no encoder count change for silent_window_sec), not the
  /// net-displacement one.
  bool zero_count{false};
};

struct StallDetectorResult
{
  /// The wheel commands to actually write: `requested`, except that every
  /// latched wheel is replaced by exactly 0.0. Nothing else is modified.
  std::array<double, kNumWheels> commands{};
  std::array<StallWheelEvent, kNumWheels> events{};
  /// True when any latch changed on this cycle — the caller's cue to publish
  /// the latched stall-state topic (edge-triggered, never per cycle).
  bool state_changed{false};
};

class StallDetector
{
public:
  StallDetector() = default;
  explicit StallDetector(const StallDetectorConfig & config);

  /// Rejects a configuration that cannot behave (non-positive window, release
  /// band above the arming threshold, ...) and explains why in `error`.
  static bool validate(const StallDetectorConfig & config, std::string & error);

  void configure(const StallDetectorConfig & config);
  const StallDetectorConfig & config() const { return config_; }

  /// Clears every latch, every window and every counter. Called on activate and
  /// deactivate: a latch must never survive a controller lifecycle transition,
  /// or the robot comes back with a wheel silently disabled.
  void reset();

  /// OPERATOR CLEAR of the whole-drive fault (SR-18 amendment 2026-11-09):
  /// releases every tier-1 latch WITHOUT the OP-25 fall-and-rise edge (D2
  /// overrides OP-25 for this event only) and restarts detection from scratch
  /// on all four wheels -- displacement windows and zero-count accumulators.
  /// Unlike reset() it KEEPS the diagnostic trip counts and the time base. It
  /// emits no `released` event: the caller tells the drive-fault monitor about
  /// the clear itself. Never called for a tier-1 latch without a whole-drive
  /// fault; that one keeps its OP-25 release.
  void release_for_operator_clear();

  /// Latest per-wheel provenance, joint order FL, FR, BL, BR. A short or absent
  /// array must be passed as kStallProvenanceUnknown, never dropped.
  void set_provenance(const std::array<int, kNumWheels> & provenance);
  const std::array<int, kNumWheels> & provenance() const { return provenance_; }

  /// One cycle. `now_sec` is the controller clock in seconds; `requested` are
  /// the wheel commands the control law wants (rad/s, joint order, after
  /// wheel_command_multipliers); `position` is the accumulated wheel POSITION
  /// state in rad and `position_valid` says whether that sample could be read
  /// and is finite.
  ///
  /// `drive_withheld` is the one fact this class cannot derive for itself: that
  /// something between the control law and the actuator has already replaced
  /// the wheel commands with EXACTLY 0.0 on this cycle, so `requested` is not
  /// what the motor is getting. Today the only such stage is the steering
  /// alignment gate (alignment_gate.hpp, stage 2 of
  /// SwerveController::write_wheel_commands), which reports it as
  /// AlignmentGateResult::withheld. While it is true, every wheel is treated
  /// exactly as it is treated under a command below min_command_rad_s: the
  /// dwell window does not accumulate and nothing can trip. The full rationale
  /// is at arming gate (d) in stall_detector.cpp.
  ///
  /// IT IS A REQUIRED ARGUMENT — no default, and not a setter. A default or a
  /// setter can be forgotten by a call site, and forgetting it does not fail to
  /// compile, it silently re-creates the defect this parameter exists to close.
  /// The type is a plain bool rather than an AlignmentGateStatus on purpose:
  /// what the detector needs to know is that its premise is false, not which
  /// stage falsified it, and a status would invite this class to make policy
  /// over gate states it should not know about.
  StallDetectorResult update(
    double now_sec, const std::array<double, kNumWheels> & requested,
    const std::array<double, kNumWheels> & position,
    const std::array<bool, kNumWheels> & position_valid, bool drive_withheld);

  bool latched(std::size_t wheel) const { return latched_[wheel]; }
  uint32_t trip_count(std::size_t wheel) const { return trip_count_[wheel]; }
  /// Whether the arming conditions were all satisfied on the last update —
  /// i.e. whether this wheel is currently being watched at all. Published so
  /// that "the detector is asleep" can never look like "the detector is happy".
  bool armed(std::size_t wheel) const { return armed_[wheel]; }
  /// Whether the ZERO-COUNT criterion watched this wheel on the last update.
  bool silent_armed(std::size_t wheel) const { return silent_armed_[wheel]; }
  /// Accumulated watched time without a count change, seconds.
  double silent_sec(std::size_t wheel) const { return silent_sec_[wheel]; }
  /// Whether the current latch was set by the zero-count criterion.
  bool latched_by_zero_count(std::size_t wheel) const
  {
    return latched_[wheel] && latched_zero_count_[wheel];
  }
  std::size_t latched_count() const;
  /// How long this wheel has been armed with no position movement beyond the
  /// delta (the age of its open dwell window), seconds; 0 when not armed.
  /// Read-only diagnostics for the whole-drive fault's classification.
  double no_motion_sec(std::size_t wheel, double now_sec) const
  {
    return (armed_[wheel] && window_open_[wheel] && now_sec >= window_start_sec_[wheel]) ?
           now_sec - window_start_sec_[wheel] : 0.0;
  }

private:
  void restart_window(std::size_t wheel, double now_sec, double position);

  StallDetectorConfig config_{};

  std::array<int, kNumWheels> provenance_{
    kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
    kStallProvenanceUnknown};

  std::array<bool, kNumWheels> latched_{};
  std::array<bool, kNumWheels> release_armed_{};
  std::array<bool, kNumWheels> armed_{};
  std::array<bool, kNumWheels> window_open_{};
  std::array<double, kNumWheels> window_start_sec_{};
  std::array<double, kNumWheels> window_start_position_{};
  std::array<uint32_t, kNumWheels> trip_count_{};

  // Zero-count criterion.
  std::array<bool, kNumWheels> latched_zero_count_{};
  std::array<bool, kNumWheels> silent_armed_{};
  std::array<double, kNumWheels> silent_sec_{};
  std::array<bool, kNumWheels> silent_ref_valid_{};
  std::array<double, kNumWheels> silent_ref_position_{};
  bool have_prev_now_{false};
  double prev_now_sec_{0.0};
};

}  // namespace gripperx_swerve_controller

#endif  // GRIPPERX_SWERVE_CONTROLLER__STALL_DETECTOR_HPP_
