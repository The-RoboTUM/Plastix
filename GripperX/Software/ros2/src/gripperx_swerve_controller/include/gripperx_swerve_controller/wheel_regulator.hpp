// Per-wheel velocity regulator — the closed-loop trim on top of the open-loop
// feedforward. Ships disabled by default (FR-14 R1) and the enable flag is
// runtime-settable in both directions (R2).
//
// READ BEFORE SETTING `wheel_regulator_enabled: true` ON HARDWARE. NFR-10
// acceptance item 10 ("the control law is demonstrably open-loop") holds only
// while this class is disabled — the shipped default. Enabling it is FR-14
// acceptance A11: a movement test in its own right, requiring its own SR-1
// approval.
//
// This regulator exists only to track VARIATION (other surfaces, payload,
// slope). A constant deficit is the firmware feedforward offset's job
// (`pwm = a + b*|rpm|`), not this class's.
//
// It trims, it does not replace: what leaves this class is always
// `setpoint + bounded correction`, never a raw controller output.
//
// No derivative term: the velocity estimate is a first difference over a
// >= 100 ms window from a quantised ~30 Hz feed (one encoder count is
// 2*pi/3200 rad); a D term would differentiate that quantisation noise and
// dominate the output.
//
// No rclcpp in this header or its .cpp, matching swerve_kinematics,
// steering_limits and stall_detector, so the control law is unit-checkable
// without a stack, without hardware and without moving anything
// (test/test_wheel_regulator.cpp).

#ifndef GRIPPERX_SWERVE_CONTROLLER__WHEEL_REGULATOR_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__WHEEL_REGULATOR_HPP_

#include <array>
#include <cstddef>
#include <string>

// For StallProvenance. The provenance codes are REUSED, not re-declared: FR-11
// items 5/6 give one latched per-wheel provenance on /hw/wheel_feedback_valid,
// and a second copy of the enum is a second thing to forget to renumber.
#include "gripperx_swerve_controller/stall_detector.hpp"
#include "gripperx_swerve_controller/swerve_kinematics.hpp"

namespace gripperx_swerve_controller
{

/// Per-wheel regulator state, published so that "not regulating" can never be
/// mistaken for "regulating and content" — FR-14 item 7, acceptance A1/A3/A6/A8.
///
/// THE NUMERIC VALUES MIRROR THE CONSTANTS IN
/// gripperx_control_msgs/msg/WheelVelocityReport.msg AND ARE STATIC_ASSERTED
/// AGAINST THEM in swerve_controller.cpp. Change both or neither.
enum WheelRegulatorStatus : int
{
  /// The regulator is switched off. FR-14 A1 requires this to be what a freshly
  /// launched stack reports on every wheel.
  kRegulatorDisabled = 0,
  /// Regulating, and not against its bound.
  kRegulatorActive = 1,
  /// Regulating AND sitting on its authority limit. FR-14 item 7 makes this a
  /// DISTINCT AND REQUIRED state, because a regulator on its bound is reporting
  /// "the feedforward is wrong for this surface" — the OP-32 condition — and
  /// that must not be indistinguishable from a regulator that is comfortable.
  kRegulatorAtAuthorityLimit = 2,
  /// Not regulated: encoder provenance is not Live (FR-14 item 5).
  kRegulatorOffProvenance = 3,
  /// Not regulated: no usable measurement or setpoint this cycle.
  kRegulatorOffNoMeasurement = 4,
  /// Not regulated: no NEW feedback sample within max_sample_age_sec. This is a
  /// statement about information, not the link: a machine standing perfectly
  /// still also emits a feed in which nothing changes, so a true standstill
  /// legitimately reaches this state too — harmless, since the authority bound
  /// at a zero setpoint is zero anyway.
  kRegulatorOffStaleFeedback = 5,
  /// Not regulated: the wheel is latched off by HWR-30a (FR-14 item 6).
  kRegulatorOffStallLatched = 6,
  /// Not regulated: the commanded magnitude is at or below the SLOW-END FLOOR
  /// (FR-14 item 12, acceptance A17) — a statement about the machine, not the
  /// feedback, which is why it is not folded into kRegulatorOffNoMeasurement:
  /// below the floor the plant is not proportional and does not move at all, so
  /// no gain can act on it even with perfect feedback. The remedy is the
  /// feedforward offset (OP-32 G1), not an encoder. See the floor gate in
  /// update() for the actual value.
  kRegulatorOffBelowFloor = 7,
};

struct WheelRegulatorConfig
{
  /// MASTER SWITCH, AND IT DEFAULTS TO FALSE. False here means the controller
  /// never calls update() at all (swerve_controller.cpp), so the command that
  /// reaches the command interface is the bit-identical feedforward value.
  bool enabled{false};

  /// Proportional gain, dimensionless (rad/s of correction per rad/s of error).
  /// TO-VERIFY. Not tuned, not derived from any measurement — 0.1 is chosen
  /// small enough that a first enable is an experiment, not a behaviour change.
  double kp{0.1};

  /// Integral gain, 1/s. TO-VERIFY, same status as kp — deliberately slow (a
  /// held disturbance-sized error needs several seconds to reach the 30 %
  /// authority bound) so a first enable cannot surprise anyone.
  double ki{0.1};

  /// AUTHORITY BOUND, as a fraction of |setpoint|. `setpoint[i]` IS the
  /// feedforward output at this layer — what the open-loop control law
  /// computed for that wheel this cycle, post wheel_command_multipliers — so
  /// `bound = max_correction_fraction * |setpoint[i]|` is literally "X % of the
  /// feedforward output", not an approximation of it.
  ///
  /// This bound is what stops the regulator from driving a slipping or blocked
  /// wheel to full effort: a wheel that cannot reach its setpoint produces a
  /// large, permanent error, and an unbounded PI would answer it with ever more
  /// effort. The integrator is clamped to the SAME bound (not merely the
  /// output), so it cannot accumulate authority it is not allowed to spend.
  /// HWR-30a's stall detector is the response to the fault; this bound only
  /// guarantees the regulator does not make it worse before the detector trips.
  ///
  /// Must be in [0, 1) — strictly below 1.0. At exactly 1.0 a correction of
  /// -|setpoint| would write exactly 0.0, indistinguishable at the actuator
  /// from an HWR-30a stall latch (FR-14 item 2: must not be able to reduce the
  /// feedforward contribution to zero).
  double max_correction_fraction{0.30};

  /// A wheel regulates only while the feedback is FRESH. Beyond this age since
  /// the last genuinely new sample, the wheel falls back to pure feedforward and
  /// its integrator resets, and a new sample arriving after a gap this long
  /// starts a fresh window instead of integrating across the hole.
  /// TO-VERIFY. 0.2 s is about six periods of the ~30 Hz feed, clear of
  /// ordinary jitter, but not derived from any measurement of a safe gap.
  double max_sample_age_sec{0.2};

  /// Ceiling on the CORRECTED command, rad/s. NOT ITS OWN ROS PARAMETER: the
  /// controller mirrors `max_wheel_angular_speed` into it, because the
  /// saturation in update() happens BEFORE the correction is added and a
  /// correction must not be able to push the command past a limit the control
  /// law already respected.
  double output_limit_rad_s{12.0};

  /// Sim-only escape hatch: nothing publishes /hw/wheel_feedback_valid in the
  /// twin, so the provenance gate would otherwise keep the regulator
  /// permanently disengaged there. TRUE only in swerve_controller.sim.yaml.
  ///
  /// MUST STAY FALSE ON THE REAL ROBOT. Setting it true there defeats FR-11
  /// item 6 (echoed feedback must disable closed-loop control, not feed it):
  /// the regulator would close the loop on a value that may be the command
  /// echoed back, whose error is identically zero — perfect apparent tracking,
  /// no control at all. One `assume_live_provenance` parameter feeds both
  /// configs so its two consumers cannot disagree about the same topic.
  bool assume_live_provenance{false};
};

/// One cycle's worth of inputs. Everything is in JOINT ORDER FL, FR, BL, BR and
/// in the joint's own axis convention, i.e. `setpoint` is POST
/// wheel_command_multipliers — the same frame `measured` arrives in, which is
/// the whole reason FR-11 item 1 fixed that convention for the report.
struct WheelRegulatorInput
{
  /// Controller clock, seconds.
  double now_sec{0.0};
  /// The kinematic feedforward command this cycle wants, rad/s.
  std::array<double, kNumWheels> setpoint{};
  /// Wheel VELOCITY state, rad/s.
  std::array<double, kNumWheels> measured{};
  std::array<bool, kNumWheels> measured_valid{};
  /// Accumulated wheel POSITION state, rad. Used ONLY as the novelty signal —
  /// see the "new measurement" note on update(). No control term reads it.
  std::array<double, kNumWheels> position{};
  std::array<bool, kNumWheels> position_valid{};
  /// Latest per-wheel provenance from the latched /hw/wheel_feedback_valid.
  std::array<int, kNumWheels> provenance{
    kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
    kStallProvenanceUnknown};
  /// HWR-30a tier-1 latch state per wheel, as it stands BEFORE the stall gate
  /// runs this cycle. A latched wheel must not be regulated — see update().
  std::array<bool, kNumWheels> stall_latched{};

  /// THE SLOW-END FLOOR (FR-14 item 12) is not a number of its own: it is
  /// HWR-30a's arming threshold `stall_min_command_rad_s`, handed in per cycle
  /// from the detector's own config (`stall_config_.min_command_rad_s` in
  /// SwerveController::write_wheel_commands()) rather than copied once, so the
  /// two provably cannot disagree — that coupling is the safety property. The
  /// default is likewise taken FROM StallDetectorConfig rather than duplicated
  /// as a literal, so a caller that forgets to set it gets a floor, not none.
  /// See the floor gate in update() for why this threshold and not a lower one.
  double stall_min_command_rad_s{StallDetectorConfig{}.min_command_rad_s};
};

struct WheelRegulatorResult
{
  /// What to command: `setpoint + correction`, clamped to output_limit_rad_s.
  std::array<double, kNumWheels> commands{};
  /// The correction actually contained in `commands`, rad/s. Published as a
  /// first-class signal — see the FR-11 rationale on the message field.
  std::array<double, kNumWheels> correction{};
  /// Whether this wheel was regulated on this cycle. False means pure
  /// feedforward: disabled, provenance not Live, no usable or no fresh
  /// measurement, or the wheel is latched off by HWR-30a.
  std::array<bool, kNumWheels> regulating{};
  /// WHY, per wheel — the half of the signal FR-14 item 7 makes binding. A
  /// regulator that quietly stops regulating one wheel looks exactly like a
  /// regulator that is working.
  std::array<int, kNumWheels> status{
    kRegulatorDisabled, kRegulatorDisabled, kRegulatorDisabled, kRegulatorDisabled};
};

class WheelRegulator
{
public:
  WheelRegulator() = default;
  explicit WheelRegulator(const WheelRegulatorConfig & config);

  /// Rejects a configuration that cannot behave (negative gains, an authority
  /// fraction outside [0, 1], a non-positive freshness window or output limit)
  /// and explains why in `error`.
  static bool validate(const WheelRegulatorConfig & config, std::string & error);

  void configure(const WheelRegulatorConfig & config);
  const WheelRegulatorConfig & config() const { return config_; }

  /// Clears every integrator and forgets the last sample. Called on activate and
  /// on deactivate: an integrator that survives a lifecycle transition would
  /// apply a correction earned before the controller stopped, against a robot
  /// that may have been moved in the meantime.
  void reset();

  /// One cycle.
  ///
  /// The integrator advances only on a cycle where a genuinely NEW measurement
  /// sample was detected, using the actual elapsed time since the previous
  /// detected sample — never once per control-loop cycle. update() and
  /// /hw/joint_states both run at ~30 Hz but from different, free-running
  /// clocks, so some cycles see no new sample and the next sees the effect of
  /// two; integrating the same sample twice would wind the integrator faster
  /// than the information justifies.
  ///
  /// A new sample is detected FRAME-WIDE, not per wheel: all four wheels ride
  /// in one /hw/joint_states message, so any one value changing proves the
  /// frame is new for all four. Both velocity and accumulated position are
  /// compared, because velocity can legitimately repeat for many frames at a
  /// steady speed while position cannot; a frame that repeats bit-for-bit is
  /// treated as NOT new, which under-counts rather than over-counts — the safe
  /// direction for an integrator.
  ///
  /// The state interface carries no sample timestamp, so elapsed time is
  /// measured between the CYCLES that detected the samples (quantised to the
  /// loop period) rather than a nominal per-cycle dt — the detected dt's sum to
  /// real elapsed time, which a nominal dt would not.
  WheelRegulatorResult update(const WheelRegulatorInput & input);

  /// Integrator state, rad/s of correction. Exposed for the unit tests and for
  /// nothing else.
  double integrator(std::size_t wheel) const { return integrator_[wheel]; }
  bool regulating(std::size_t wheel) const { return regulating_[wheel]; }

private:
  /// True when `input` carries at least one value that differs from the previous
  /// cycle's; updates the stored previous sample as a side effect.
  bool detect_new_sample(const WheelRegulatorInput & input);

  WheelRegulatorConfig config_{};

  std::array<double, kNumWheels> integrator_{};
  std::array<bool, kNumWheels> regulating_{};

  bool have_previous_sample_{false};
  std::array<double, kNumWheels> previous_measured_{};
  std::array<double, kNumWheels> previous_position_{};
  bool have_sample_time_{false};
  double last_sample_sec_{0.0};
};

}  // namespace gripperx_swerve_controller

#endif  // GRIPPERX_SWERVE_CONTROLLER__WHEEL_REGULATOR_HPP_
