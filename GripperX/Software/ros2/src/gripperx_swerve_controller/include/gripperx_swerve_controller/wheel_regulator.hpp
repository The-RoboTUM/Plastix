// Per-wheel velocity regulator — the closed-loop TRIM on top of the open-loop
// feedforward. STRUCTURE ONLY: DISABLED BY DEFAULT, AND NOTHING TUNES IT.
//
// THE REQUIREMENT IS FR-14, and this file is written against it. FR-11's
// 2026-08-17 deferral ("prepare, do not implement the loop") was REVERSED by the
// user on 2026-08-20; FR-14 carries the specification, R1 (ships disabled) and R2
// (the enable flag is runtime-settable in both directions) are the user's two
// binding constraints, and both are implemented here.
//
// READ THIS BEFORE SETTING `wheel_regulator_enabled: true` ON HARDWARE.
// NFR-10 acceptance item 10 ("the control law is demonstrably open-loop") is
// SATISFIED while this class is disabled — the shipped default — and becomes NOT
// SATISFIED from the first time it is set true with actuator power on. That
// first enable is FR-14 acceptance A11: a movement test in its own right,
// requiring its own SR-1 approval, and the event whose date NFR-10 item 10 is
// waiting for.
//
// WHY A REGULATOR AT ALL — MEASURED ON CARPET, 2026-08-20, NOT THEORY.
// The open-loop drive lost a CONSTANT 1.858 / 1.859 rad/s per wheel under load
// at two different commanded speeds (4.286 and 2.857 rad/s), four wheels within
// 1.3 % of each other (internal NAV2_HANDOVER §1). A CONSTANT deficit is
// not this class's job: it is removed by the firmware feedforward offset
// (`pwm = a + b*|rpm|`, a = 19 PWM counts) flashed the same day. What no constant
// can track is the VARIATION — other surfaces, payload, slope — and that is the
// only thing this regulator exists for.
//
// IT TRIMS, IT DOES NOT REPLACE. What leaves this class is always
// `setpoint + bounded correction`, never a raw controller output. With the
// authority bound at its default 30 % of |setpoint| the regulator cannot even
// close the measured 1.858 rad/s deficit on its own (30 % of 4.286 = 1.286) —
// that is the intended division of labour, not a shortfall.
//
// PI, NO D — a measurement decides this, not preference. The velocity estimate
// is a first difference over a >= 100 ms window from a 30 Hz feed; one encoder
// count is 2*pi/3200 = 0.00196 rad and the measured per-wheel standard deviation
// is 0.0093-0.0396 rad/s unloaded. A derivative term differentiates that
// quantisation noise and would dominate the output.
//
// NO rclcpp IN THIS HEADER OR ITS .cpp, exactly as swerve_kinematics,
// steering_limits and stall_detector — so the control law is unit-checkable
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
  /// Not regulated: no NEW feedback sample within max_sample_age_sec.
  ///
  /// READ THIS AS A STATEMENT ABOUT INFORMATION, NOT ABOUT THE LINK. A machine
  /// standing perfectly still emits a feed in which nothing changes, and a
  /// changed value is the only evidence of a new frame a state interface can
  /// carry — so a true standstill reaches this state legitimately. It has no
  /// control consequence there: at a zero setpoint the authority bound is zero.
  kRegulatorOffStaleFeedback = 5,
  /// Not regulated: the wheel is latched off by HWR-30a (FR-14 item 6).
  kRegulatorOffStallLatched = 6,
  /// Not regulated: the commanded magnitude is at or below the SLOW-END FLOOR
  /// (FR-14 item 12, acceptance A17). THIS IS A STATEMENT ABOUT THE MACHINE, NOT
  /// ABOUT THE FEEDBACK, which is why it is not folded into
  /// kRegulatorOffNoMeasurement: down there the plant is not proportional, is
  /// history-dependent between the drop-out and breakaway thresholds and below
  /// drop-out does not move at all, so there is nothing a gain can act on even
  /// with perfect feedback. The remedy is the feedforward offset (OP-32 G1), not
  /// an encoder. See the floor gate in update() for what the floor IS.
  kRegulatorOffBelowFloor = 7,
};

struct WheelRegulatorConfig
{
  /// MASTER SWITCH, AND IT DEFAULTS TO FALSE. False here means the controller
  /// never calls update() at all (swerve_controller.cpp), so the command that
  /// reaches the command interface is the bit-identical feedforward value.
  bool enabled{false};

  /// Proportional gain, dimensionless (rad/s of correction per rad/s of error).
  /// TO-VERIFY. NOT TUNED AND NOT DERIVED FROM ANY MEASUREMENT. 0.1 is chosen
  /// to be small enough that a first enable is an experiment rather than a
  /// change of behaviour: against the measured 1.858 rad/s deficit it asks for
  /// 0.186 rad/s, about 4 % of the 4.286 rad/s setpoint it was measured at.
  double kp{0.1};

  /// Integral gain, 1/s. TO-VERIFY, same status as kp. At 0.1 and a held
  /// 1.858 rad/s error the integrator needs ~7 s to reach the 30 % authority
  /// bound at that setpoint — deliberately slower than any disturbance this is
  /// meant to answer, so that a first enable cannot surprise anyone.
  double ki{0.1};

  /// AUTHORITY BOUND, as a FRACTION of |setpoint|. USER'S FIGURE, 2026-08-20.
  ///
  /// THE REFERENCE QUANTITY IS THE FEEDFORWARD OUTPUT — USER DECISION
  /// 2026-08-20, closing the open question FR-14 item 8 left. At this layer the
  /// kinematic setpoint IS the feedforward output: `setpoint[i]` is what the
  /// open-loop control law computed for that wheel this cycle, post
  /// wheel_command_multipliers, and there is no other feedforward quantity the
  /// controller produces. So `bound = max_correction_fraction * |setpoint[i]|`
  /// IS "30 % of the feedforward output", not an approximation of it.
  /// The only genuinely different alternatives were an ABSOLUTE bound in rad/s
  /// (which would be a fixed trim at every speed — 30 % of 4.286 rad/s and 30 %
  /// of 0.1 rad/s are very different amounts of authority, and the fraction is
  /// what makes the bound scale with the command) and a fraction of the
  /// firmware's PWM RANGE (which this layer CANNOT SEE: `pwm = a + b*|rpm|` lives
  /// in the ESP32 and no PWM value crosses the ros2_control boundary). Recorded
  /// so the question is not re-litigated.
  ///
  /// THIS BOUND IS WHAT STOPS A REGULATOR FROM DRIVING A SLIPPING OR BLOCKED
  /// WHEEL TO FULL EFFORT. A wheel that cannot reach its setpoint — jammed,
  /// spinning on ice, off the ground — produces a large, permanent error, and
  /// an unbounded PI answers it by commanding ever more effort into the fault.
  /// The bound caps that at 30 % of what was asked for, and the integrator is
  /// clamped to the SAME bound (not merely the output), so it cannot accumulate
  /// authority it is not allowed to spend. HWR-30a's stall detector is the
  /// response to the fault; this bound is only the guarantee that the regulator
  /// does not make it worse in the second before the detector trips.
  ///
  /// Must be in [0, 1) — STRICTLY below 1.0, and the exclusion of 1.0 itself is
  /// FR-14 item 2 ("MUST NOT be able to reduce the feedforward contribution to
  /// zero"): at exactly 1.0 a correction of -|setpoint| writes exactly 0.0,
  /// which is indistinguishable at the actuator from an HWR-30a stall latch.
  double max_correction_fraction{0.30};

  /// A wheel regulates only while the feedback is FRESH. Beyond this age since
  /// the last genuinely new sample, the wheel falls back to pure feedforward and
  /// its integrator resets, and a new sample arriving after a gap this long
  /// starts a fresh window instead of integrating across the hole.
  /// TO-VERIFY. 0.2 s is six sample periods of the measured 29.999 Hz feed
  /// (min 0.022 s, max 0.044 s), i.e. clear of ordinary jitter, and it is not
  /// derived from any measurement of how long a gap may safely be ignored.
  double max_sample_age_sec{0.2};

  /// Ceiling on the CORRECTED command, rad/s. NOT ITS OWN ROS PARAMETER: the
  /// controller mirrors `max_wheel_angular_speed` into it, because the
  /// saturation in update() happens BEFORE the correction is added and a
  /// correction must not be able to push the command past a limit the control
  /// law already respected.
  double output_limit_rad_s{12.0};

  /// Same meaning and the same parameter as HWR-30a's: nothing publishes
  /// /hw/wheel_feedback_valid in the twin, so the provenance gate would keep the
  /// regulator permanently disengaged there. TRUE only in
  /// swerve_controller.sim.yaml; FALSE on the real robot, where a missing
  /// provenance topic MUST disengage rather than assume.
  ///
  /// SCOPED TO SIM, AND SETTING IT TRUE ON THE REAL ROBOT DEFEATS FR-11 ITEM 6.
  /// That item requires echoed (non-measured) feedback to DISABLE a closed-loop
  /// law rather than feed it, and this flag is the one switch that makes the
  /// gate answer "Live" without any evidence. In the twin that is harmless —
  /// the physics is truthful by construction and there is no encoder that can
  /// lie. On hardware it would let the regulator close the loop on a value that
  /// may be the command echoed back, whose error is identically zero: perfect
  /// apparent tracking and no control whatsoever. The controller copies
  /// the ONE `assume_live_provenance` parameter into both configs — two
  /// parameters would be two chances for the two consumers of one topic to
  /// disagree about that topic.
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

  /// THE SLOW-END FLOOR (FR-14 item 12), AND IT IS NOT A NUMBER OF ITS OWN: it
  /// is HWR-30a's arming threshold `stall_min_command_rad_s`, handed in from the
  /// detector's OWN config. Why the floor is that threshold and not something
  /// lower is argued at the floor gate in update().
  ///
  /// IT ARRIVES PER CYCLE, AND THAT IS THE POINT. A copy taken once into this
  /// regulator's config would be a second variable that can drift away from the
  /// detector's; passing it every cycle means the two provably cannot disagree,
  /// which is the safety property the coupling exists for. The controller reads
  /// it straight out of `stall_config_.min_command_rad_s` — the same object the
  /// detector arms on — in SwerveController::write_wheel_commands().
  ///
  /// The default is taken FROM StallDetectorConfig rather than written out as a
  /// literal here, for the same reason: one declaration, not two. It is also the
  /// safe direction — a caller that forgets to set it gets a floor, not none.
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
  /// TICKS ON NEW MEASUREMENTS, NOT ON THE CONTROL LOOP — and this is a measured
  /// property of this system, not a hypothesis. update() runs at 30 Hz and
  /// /hw/joint_states also arrives at ~30 Hz, but from a FREE-RUNNING firmware
  /// clock: measured min 0.022 s / max 0.044 s against the 33.3 ms loop. Some
  /// cycles therefore see no new sample and the next sees the effect of two.
  /// Integrating the same sample twice winds the integrator faster than the
  /// information justifies, so the integrator advances ONLY on a cycle where a
  /// genuinely new sample was detected, using the ACTUAL elapsed time since the
  /// previous detected sample.
  ///
  /// HOW A NEW SAMPLE IS DETECTED, and why it is FRAME-WIDE rather than
  /// per-wheel: all four wheels ride in ONE /hw/joint_states message, so any one
  /// value changing proves the frame is new for all four. Both the velocity and
  /// the accumulated position are compared, because at a steady speed the
  /// quantised velocity estimate can legitimately repeat for many frames while
  /// the position accumulator cannot. A frame whose every value repeats bit for
  /// bit is treated as NOT new — that under-counts rather than over-counts, and
  /// under-counting is the safe direction for an integrator.
  ///
  /// THE STATE INTERFACE CARRIES NO SAMPLE TIMESTAMP, so the elapsed time is
  /// measured between the CYCLES that detected the samples and is therefore
  /// quantised to the loop period. It is unbiased where it matters: the detected
  /// dt's sum to real elapsed time, which a nominal per-cycle dt would not.
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
