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

  /// Arming threshold on |commanded wheel angular velocity|, rad/s. Below it
  /// the wheel is not considered "commanded to move" and cannot trip.
  double min_command_rad_s{2.0};

  /// How much the ACCUMULATED wheel position (rad) must change inside the
  /// window for the wheel to count as turning. One encoder count is
  /// 2*pi/COUNTS_PER_OUTPUT_REV = 2*pi/3200 = 0.00196 rad (the constant is
  /// MEASURED and confirmed to ~1 %, not assumed), so the default is 25.5
  /// counts. Full rationale in ros2_controllers.yaml.
  double min_position_delta_rad{0.05};

  /// OP-25 release band: |command| must fall to or below this before a rise
  /// back above min_command_rad_s counts as a FRESH command. Must be
  /// <= min_command_rad_s.
  double release_command_rad_s{0.1};

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
  std::size_t latched_count() const;

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
};

}  // namespace gripperx_swerve_controller

#endif  // GRIPPERX_SWERVE_CONTROLLER__STALL_DETECTOR_HPP_
