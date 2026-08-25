// Unit checks for the per-wheel velocity regulator. No ROS in here on purpose —
// the whole control law is rclcpp-free (wheel_regulator.hpp), so every branch
// that matters on the robot can be exercised on a laptop without a stack,
// without hardware and without moving anything.
//
// WHAT THESE TESTS CANNOT SHOW: that the GAINS ARE RIGHT. kp, ki and the
// freshness window are TO-VERIFY and deliberately untuned — the regulator exists
// to answer the VARIATION in the load deficit (surface, payload, slope), and
// nobody has measured that variation. These tests check the LOGIC against
// whatever gains they are given, and above all they check the properties that
// must hold for ANY gains: bounded authority, a clamped integrator, no
// integration without new information, and no regulation without provenance.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

#include "gripperx_swerve_controller/wheel_regulator.hpp"

using gripperx_swerve_controller::kNumWheels;
using gripperx_swerve_controller::kStallProvenanceLive;
using gripperx_swerve_controller::kStallProvenanceLiveUnconfirmed;
using gripperx_swerve_controller::kStallProvenanceNoEncoder;
using gripperx_swerve_controller::kStallProvenanceUnknown;
using gripperx_swerve_controller::StallDetectorConfig;
using gripperx_swerve_controller::WheelRegulator;
using gripperx_swerve_controller::WheelRegulatorConfig;
using gripperx_swerve_controller::WheelRegulatorInput;
using gripperx_swerve_controller::WheelRegulatorResult;

namespace
{
constexpr double kDt = 1.0 / 30.0;  // controller_manager update_rate
constexpr double kEps = 1e-9;

/// The measured 2026-08-20 carpet reference point: 0.3 m/s of body speed is
/// 4.286 rad/s per wheel at the measured rolling radius 0.070 m.
constexpr double kReferenceSetpoint = 4.286;

WheelRegulatorConfig test_config()
{
  WheelRegulatorConfig config;
  config.enabled = true;
  config.kp = 0.1;
  config.ki = 0.1;
  config.max_correction_fraction = 0.30;
  config.max_sample_age_sec = 0.2;
  config.output_limit_rad_s = 12.0;
  config.assume_live_provenance = false;
  return config;
}

std::array<int, kNumWheels> all_live()
{
  return {kStallProvenanceLive, kStallProvenanceLive, kStallProvenanceLive, kStallProvenanceLive};
}

std::array<double, kNumWheels> uniform(double value) { return {value, value, value, value}; }

std::array<bool, kNumWheels> all_true() { return {true, true, true, true}; }

/// Drives the regulator the way the controller does: one call per 30 Hz cycle,
/// with the accumulated wheel POSITION advanced from the measured velocity so
/// that "a new /hw/joint_states frame arrived" is expressed the same way it is
/// on the robot.
struct Rig
{
  WheelRegulator regulator;
  double now{0.0};
  std::array<double, kNumWheels> position{};
  std::array<int, kNumWheels> provenance{all_live()};
  std::array<bool, kNumWheels> latched{};
  /// THE SLOW-END FLOOR, exactly as the controller supplies it: HWR-30a's own
  /// arming threshold, read from the detector's config and handed in per cycle.
  /// Mutable here so a test can MOVE the threshold and watch the floor follow —
  /// the property that distinguishes a coupling from a snapshot.
  double stall_min_command_rad_s{StallDetectorConfig{}.min_command_rad_s};

  explicit Rig(const WheelRegulatorConfig & config) : regulator(config) {}

  /// One cycle. `fresh_sample` false means the controller cycle saw the very
  /// same values again — the free-running-firmware-clock case this design is
  /// built around.
  WheelRegulatorResult step(
    const std::array<double, kNumWheels> & setpoint,
    const std::array<double, kNumWheels> & measured, bool fresh_sample = true)
  {
    now += kDt;
    if (fresh_sample) {
      for (std::size_t i = 0; i < kNumWheels; ++i) {
        position[i] += measured[i] * kDt;
      }
    }
    WheelRegulatorInput input;
    input.now_sec = now;
    input.setpoint = setpoint;
    input.measured = measured;
    input.measured_valid = all_true();
    input.position = position;
    input.position_valid = all_true();
    input.provenance = provenance;
    input.stall_latched = latched;
    input.stall_min_command_rad_s = stall_min_command_rad_s;
    return regulator.update(input);
  }

  WheelRegulatorResult run(
    double seconds, const std::array<double, kNumWheels> & setpoint,
    const std::array<double, kNumWheels> & measured)
  {
    WheelRegulatorResult result;
    const int cycles = static_cast<int>(seconds / kDt);
    for (int c = 0; c < cycles; ++c) {
      result = step(setpoint, measured);
    }
    return result;
  }
};
}  // namespace

// ---------------------------------------------------------------------------
// DISABLED IS INERT. The first property, and the one the whole change is
// allowed to exist under: with the regulator off the command is the setpoint,
// bit for bit.
// ---------------------------------------------------------------------------
TEST(WheelRegulator, DisabledProducesNoCorrectionAtAll)
{
  WheelRegulatorConfig config = test_config();
  config.enabled = false;
  config.kp = 100.0;  // absurd on purpose: disabled must beat any gain
  config.ki = 100.0;
  Rig rig(config);

  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(1.0);
  const auto result = rig.run(5.0, setpoint, measured);

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.commands[i], setpoint[i]) << "wheel " << i;
    EXPECT_EQ(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_FALSE(result.regulating[i]) << "wheel " << i;
    EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulator, DefaultConstructedConfigIsDisabled)
{
  // The struct default is the safety property, not just the YAML value.
  const WheelRegulatorConfig config;
  EXPECT_FALSE(config.enabled);
  EXPECT_DOUBLE_EQ(config.max_correction_fraction, 0.30);
}

// ---------------------------------------------------------------------------
// TRIM, NOT REPLACE
// ---------------------------------------------------------------------------
TEST(WheelRegulator, CommandIsAlwaysSetpointPlusReportedCorrection)
{
  Rig rig(test_config());
  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(2.4);  // the measured carpet shortfall

  for (int c = 0; c < 90; ++c) {
    const auto result = rig.step(setpoint, measured);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_NEAR(result.commands[i], setpoint[i] + result.correction[i], kEps);
    }
  }
}

// ---------------------------------------------------------------------------
// BOUNDED AUTHORITY
// ---------------------------------------------------------------------------
TEST(WheelRegulator, CorrectionSaturatesAtTheAuthorityBound)
{
  WheelRegulatorConfig config = test_config();
  config.kp = 5.0;  // large enough that P alone would blow past the bound
  Rig rig(config);

  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(1.0);
  const double bound = config.max_correction_fraction * kReferenceSetpoint;

  const auto result = rig.run(3.0, setpoint, measured);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_NEAR(result.correction[i], bound, kEps) << "wheel " << i;
    EXPECT_NEAR(result.commands[i], setpoint[i] + bound, kEps) << "wheel " << i;
  }
}

TEST(WheelRegulator, BlockedWheelCannotBeDrivenToFullEffort)
{
  // The case the bound exists for: a wheel that cannot move at all produces a
  // permanent maximum error, and an unbounded PI would answer it with unbounded
  // effort. HWR-30a latches such a wheel after its window; until then the bound
  // is what limits the damage.
  WheelRegulatorConfig config = test_config();
  config.kp = 2.0;
  config.ki = 5.0;
  Rig rig(config);

  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(0.0);  // blocked
  const double bound = config.max_correction_fraction * kReferenceSetpoint;

  const auto result = rig.run(20.0, setpoint, measured);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_LE(result.commands[i], setpoint[i] + bound + kEps) << "wheel " << i;
    EXPECT_LE(result.correction[i], bound + kEps) << "wheel " << i;
    EXPECT_LE(rig.regulator.integrator(i), bound + kEps) << "wheel " << i;
  }
}

// ---------------------------------------------------------------------------
// THE INTEGRATOR IS CLAMPED, NOT ONLY THE OUTPUT
// ---------------------------------------------------------------------------
TEST(WheelRegulator, IntegratorNeverWindsPastTheBound)
{
  WheelRegulatorConfig config = test_config();
  config.kp = 0.0;  // isolate the integrator
  config.ki = 10.0;
  Rig rig(config);

  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(1.0);
  const double bound = config.max_correction_fraction * kReferenceSetpoint;

  for (int c = 0; c < 600; ++c) {  // 20 s at 30 Hz
    rig.step(setpoint, measured);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_LE(std::fabs(rig.regulator.integrator(i)), bound + kEps) << "cycle " << c;
    }
  }
}

TEST(WheelRegulator, SaturatedIntegratorComesBackDownImmediately)
{
  // Anti-windup is only worth having if the integrator can RELIEVE saturation
  // without first unwinding a hidden surplus. Wind it up, then reverse the sign
  // of the error and require the correction to respond on the next samples.
  WheelRegulatorConfig config = test_config();
  config.kp = 0.0;
  config.ki = 10.0;
  Rig rig(config);

  const auto setpoint = uniform(kReferenceSetpoint);
  const double bound = config.max_correction_fraction * kReferenceSetpoint;
  rig.run(10.0, setpoint, uniform(1.0));
  EXPECT_NEAR(rig.regulator.integrator(0), bound, 1e-6);

  // Now the wheel overshoots instead of falling short.
  const auto after = rig.run(0.2, setpoint, uniform(kReferenceSetpoint + 2.0));
  EXPECT_LT(after.correction[0], bound - 1e-3);
  EXPECT_LT(rig.regulator.integrator(0), bound - 1e-3);
}

TEST(WheelRegulator, IntegratorIsFlushedWhenTheSetpointReturnsToZero)
{
  // The bound is a fraction of |setpoint|, so at standstill it is exactly zero
  // and the clamp empties the integrator. This is what makes the zero-twist and
  // stale-twist branches of update(), which call write_wheel_commands with
  // zeros, provably free of any correction.
  //
  // SINCE 2026-08-21 THE SLOW-END FLOOR REACHES THIS CASE FIRST — a zero setpoint
  // is below any positive floor, so the wheel is gated off before the bound is
  // even computed. The observable result is unchanged, which is the point: the
  // floor SUBSUMES the zero-bound argument rather than competing with it.
  Rig rig(test_config());
  rig.run(5.0, uniform(kReferenceSetpoint), uniform(2.4));
  ASSERT_GT(rig.regulator.integrator(0), 0.0);

  const auto result = rig.run(0.2, uniform(0.0), uniform(0.5));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_EQ(result.commands[i], 0.0) << "wheel " << i;
    EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }
}

// ---------------------------------------------------------------------------
// TICK ON NEW MEASUREMENTS, NOT ON THE CONTROL LOOP
// ---------------------------------------------------------------------------
TEST(WheelRegulator, NoIntegrationWithoutANewSample)
{
  WheelRegulatorConfig config = test_config();
  config.kp = 0.0;
  config.ki = 0.1;
  config.max_sample_age_sec = 10.0;   // isolate novelty from freshness
  config.max_correction_fraction = 0.99;  // and from the authority bound
  Rig rig(config);

  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(1.0);

  rig.step(setpoint, measured);              // first sample: sets the clock only
  rig.step(setpoint, measured);              // second: integrates one dt
  const double after_two = rig.regulator.integrator(0);
  EXPECT_GT(after_two, 0.0);

  for (int c = 0; c < 10; ++c) {
    rig.step(setpoint, measured, /*fresh_sample=*/false);
    EXPECT_DOUBLE_EQ(rig.regulator.integrator(0), after_two) << "repeat cycle " << c;
  }

  // And the next genuinely new sample integrates the WHOLE elapsed time, not a
  // nominal loop period: 11 cycles at 1/30 s.
  rig.step(setpoint, measured);
  const double expected_step = config.ki * (setpoint[0] - measured[0]) * 11.0 * kDt;
  ASSERT_LT(after_two + expected_step, config.max_correction_fraction * kReferenceSetpoint);
  EXPECT_NEAR(rig.regulator.integrator(0), after_two + expected_step, 1e-9);
}

TEST(WheelRegulator, RepeatedVelocityWithMovingPositionStillCountsAsNew)
{
  // At a steady speed the quantised velocity estimate legitimately repeats for
  // many frames. If novelty were judged on the velocity alone, the regulator
  // would systematically under-integrate exactly where it matters most.
  WheelRegulatorConfig config = test_config();
  config.kp = 0.0;
  config.ki = 10.0;
  Rig rig(config);

  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(2.4);  // bit-identical every cycle

  rig.step(setpoint, measured);
  const double first = rig.regulator.integrator(0);
  for (int c = 0; c < 5; ++c) {
    rig.step(setpoint, measured);  // position advances -> a new frame
  }
  EXPECT_GT(rig.regulator.integrator(0), first);
}

TEST(WheelRegulator, StaleFeedStopsRegulatingAndResetsTheIntegrator)
{
  Rig rig(test_config());
  const auto setpoint = uniform(kReferenceSetpoint);
  rig.run(5.0, setpoint, uniform(2.4));
  ASSERT_GT(rig.regulator.integrator(0), 0.0);

  // 0.3 s without a new frame, against max_sample_age_sec 0.2.
  WheelRegulatorResult result;
  for (int c = 0; c < 9; ++c) {
    result = rig.step(setpoint, uniform(2.4), /*fresh_sample=*/false);
  }
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(result.regulating[i]) << "wheel " << i;
    EXPECT_EQ(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_EQ(result.commands[i], setpoint[i]) << "wheel " << i;
    EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulator, ASampleAfterALongGapDoesNotIntegrateAcrossTheHole)
{
  WheelRegulatorConfig config = test_config();
  config.kp = 0.0;
  config.ki = 10.0;
  Rig rig(config);

  const auto setpoint = uniform(kReferenceSetpoint);
  rig.step(setpoint, uniform(2.4));
  // Jump the clock well past max_sample_age_sec, then deliver a new sample.
  rig.now += 5.0;
  rig.step(setpoint, uniform(2.4));
  // The gap re-seeded the timing; nothing was credited for the missing seconds.
  EXPECT_EQ(rig.regulator.integrator(0), 0.0);
}

// ---------------------------------------------------------------------------
// PER-WHEEL PROVENANCE GATE — FR-11 item 6 finally gets a consumer
// ---------------------------------------------------------------------------
TEST(WheelRegulator, ProvenanceBelowLiveDisablesThatWheelOnly)
{
  Rig rig(test_config());
  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(2.4);
  rig.run(5.0, setpoint, measured);
  ASSERT_GT(rig.regulator.integrator(1), 0.0);

  // FR wheel loses provenance. LiveUnconfirmed is NOT enough: it means the
  // encoder initialised and has never been seen to count.
  rig.provenance[1] = kStallProvenanceLiveUnconfirmed;
  const auto result = rig.step(setpoint, measured);

  EXPECT_FALSE(result.regulating[1]);
  EXPECT_EQ(result.correction[1], 0.0);
  EXPECT_EQ(result.commands[1], setpoint[1]);
  EXPECT_EQ(rig.regulator.integrator(1), 0.0);

  for (std::size_t i : {0u, 2u, 3u}) {
    EXPECT_TRUE(result.regulating[i]) << "wheel " << i;
    EXPECT_GT(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_GT(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulator, EveryProvenanceBelowLiveIsRefused)
{
  for (const int provenance :
       {kStallProvenanceUnknown, kStallProvenanceNoEncoder,
        gripperx_swerve_controller::kStallProvenanceInitFailed, kStallProvenanceLiveUnconfirmed})
  {
    Rig rig(test_config());
    rig.provenance = {provenance, provenance, provenance, provenance};
    const auto result = rig.run(2.0, uniform(kReferenceSetpoint), uniform(2.4));
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_FALSE(result.regulating[i]) << "provenance " << provenance << " wheel " << i;
      EXPECT_EQ(result.correction[i], 0.0) << "provenance " << provenance;
    }
  }
}

TEST(WheelRegulator, AssumeLiveProvenanceIsTheSimEscapeHatch)
{
  // Nothing publishes /hw/wheel_feedback_valid in the twin, so without this the
  // regulator would be permanently disengaged there while looking enabled.
  WheelRegulatorConfig config = test_config();
  config.assume_live_provenance = true;
  Rig rig(config);
  rig.provenance = {kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
                    kStallProvenanceUnknown};

  const auto result = rig.run(2.0, uniform(kReferenceSetpoint), uniform(2.4));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_TRUE(result.regulating[i]) << "wheel " << i;
    EXPECT_GT(result.correction[i], 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulator, AnUnusableMeasurementDisablesThatWheel)
{
  Rig rig(test_config());
  const auto setpoint = uniform(kReferenceSetpoint);
  rig.run(3.0, setpoint, uniform(2.4));

  WheelRegulatorInput input;
  rig.now += kDt;
  input.now_sec = rig.now;
  input.setpoint = setpoint;
  input.measured = uniform(2.4);
  input.measured_valid = {false, true, true, true};  // BL/BR/FR fine, FL absent
  input.position = rig.position;
  input.position_valid = all_true();
  input.provenance = rig.provenance;
  const auto result = rig.regulator.update(input);

  EXPECT_FALSE(result.regulating[0]);
  EXPECT_EQ(result.correction[0], 0.0);
  EXPECT_EQ(rig.regulator.integrator(0), 0.0);
  EXPECT_TRUE(result.regulating[1]);
}

// ---------------------------------------------------------------------------
// HWR-30a INTERACTION
// ---------------------------------------------------------------------------
TEST(WheelRegulator, ALatchedWheelGetsNoCorrectionAndAHeldResetIntegrator)
{
  Rig rig(test_config());
  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(2.4);
  rig.run(5.0, setpoint, measured);
  ASSERT_GT(rig.regulator.integrator(1), 0.0);

  // HWR-30a latches FR. Its command is about to be forced to exactly 0.0 by the
  // stall gate, which runs AFTER this — the regulator must not fight that, and
  // must not accumulate authority it would dump on the wheel at un-latch.
  rig.latched[1] = true;
  for (int c = 0; c < 60; ++c) {  // 2 s latched, measured drops to zero
    const auto result = rig.step(setpoint, {2.4, 0.0, 2.4, 2.4});
    EXPECT_FALSE(result.regulating[1]) << "cycle " << c;
    EXPECT_EQ(result.correction[1], 0.0) << "cycle " << c;
    EXPECT_EQ(result.commands[1], setpoint[1]) << "cycle " << c;
    EXPECT_EQ(rig.regulator.integrator(1), 0.0) << "cycle " << c;
  }

  // The latch clears: the wheel starts from zero authority, not from a wound-up
  // correction earned while it was switched off.
  rig.latched[1] = false;
  const auto first = rig.step(setpoint, {2.4, 0.0, 2.4, 2.4});
  EXPECT_TRUE(first.regulating[1]);
  // Exactly ONE integration step's worth of authority, not two seconds' worth.
  const double error = setpoint[1] - 0.0;
  EXPECT_NEAR(rig.regulator.integrator(1), test_config().ki * error * kDt, 1e-9);
  EXPECT_NEAR(
    first.correction[1], test_config().kp * error + test_config().ki * error * kDt, 1e-9);
}

// ---------------------------------------------------------------------------
// SIGN CORRECTNESS, BOTH DIRECTIONS OF TRAVEL
// ---------------------------------------------------------------------------
TEST(WheelRegulator, ForwardShortfallIsCorrectedUpwards)
{
  Rig rig(test_config());
  const auto setpoint = uniform(kReferenceSetpoint);
  const auto result = rig.run(2.0, setpoint, uniform(2.4));  // measured too slow
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_GT(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_GT(result.commands[i], setpoint[i]) << "wheel " << i;
  }
}

TEST(WheelRegulator, ReverseShortfallIsCorrectedDownwards)
{
  Rig rig(test_config());
  const auto setpoint = uniform(-kReferenceSetpoint);
  const auto result = rig.run(2.0, setpoint, uniform(-2.4));  // too slow, reversing
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_LT(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_LT(result.commands[i], setpoint[i]) << "wheel " << i;
    EXPECT_GE(
      result.correction[i], -test_config().max_correction_fraction * kReferenceSetpoint - kEps);
  }
}

TEST(WheelRegulator, ForwardOvershootIsCorrectedDownwards)
{
  // THE FIRST HARDWARE TEST'S CASE, ON BLOCKS: the carpet-calibrated feedforward
  // offset overshoots by roughly +39 % with the wheels off the ground. That is
  // beyond the 30 % authority bound, so the correction sits AT its NEGATIVE
  // bound and the error does NOT go to zero. Expected and correct, and the point
  // of writing it down as a test is that it must not be read as a sign error:
  // a sign error would push the correction to the POSITIVE bound and make the
  // overshoot worse.
  Rig rig(test_config());
  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(kReferenceSetpoint * 1.39);
  const double bound = test_config().max_correction_fraction * kReferenceSetpoint;

  const auto result = rig.run(30.0, setpoint, measured);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_NEAR(result.correction[i], -bound, 1e-6) << "wheel " << i;
    EXPECT_NEAR(result.commands[i], setpoint[i] - bound, 1e-6) << "wheel " << i;
    // Still moving in the commanded direction: a 30 %-bounded trim can never
    // reverse a wheel.
    EXPECT_GT(result.commands[i], 0.0) << "wheel " << i;
  }
}

// ---------------------------------------------------------------------------
// OUTPUT CEILING AND RESET
// ---------------------------------------------------------------------------
TEST(WheelRegulator, CorrectedCommandRespectsTheWheelSpeedCeiling)
{
  WheelRegulatorConfig config = test_config();
  config.output_limit_rad_s = 12.0;
  config.kp = 5.0;
  Rig rig(config);

  const auto setpoint = uniform(11.5);  // near the ceiling already
  const auto result = rig.run(3.0, setpoint, uniform(6.0));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_LE(result.commands[i], config.output_limit_rad_s + kEps) << "wheel " << i;
    EXPECT_NEAR(result.commands[i], setpoint[i] + result.correction[i], kEps) << "wheel " << i;
  }
}

TEST(WheelRegulator, ResetClearsEveryIntegrator)
{
  Rig rig(test_config());
  rig.run(5.0, uniform(kReferenceSetpoint), uniform(2.4));
  ASSERT_GT(rig.regulator.integrator(0), 0.0);

  rig.regulator.reset();
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i;
    EXPECT_FALSE(rig.regulator.regulating(i)) << "wheel " << i;
  }

  // And the forgotten sample means the first cycle back cannot integrate across
  // the gap: the first new sample only re-seeds the clock.
  const auto result = rig.step(uniform(kReferenceSetpoint), uniform(2.4));
  EXPECT_EQ(rig.regulator.integrator(0), 0.0);
  EXPECT_NEAR(result.correction[0], test_config().kp * (kReferenceSetpoint - 2.4), 1e-9);
}

TEST(WheelRegulator, ConfigureResetsToo)
{
  // The runtime enable/disable switch relies on this: configure() is what the
  // controller calls on both edges, and both edges must start from zero.
  Rig rig(test_config());
  rig.run(5.0, uniform(kReferenceSetpoint), uniform(2.4));
  ASSERT_GT(rig.regulator.integrator(0), 0.0);

  WheelRegulatorConfig off = test_config();
  off.enabled = false;
  rig.regulator.configure(off);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }

  rig.regulator.configure(test_config());
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }
}

// ---------------------------------------------------------------------------
// CONFIGURATION VALIDATION
// ---------------------------------------------------------------------------
TEST(WheelRegulator, ValidateAcceptsTheDefaults)
{
  std::string error;
  EXPECT_TRUE(WheelRegulator::validate(WheelRegulatorConfig{}, error)) << error;
  EXPECT_TRUE(WheelRegulator::validate(test_config(), error)) << error;
}

TEST(WheelRegulator, ValidateRejectsAnAuthorityFractionAtOrAboveOne)
{
  std::string error;

  WheelRegulatorConfig above = test_config();
  above.max_correction_fraction = 1.5;
  EXPECT_FALSE(WheelRegulator::validate(above, error));
  EXPECT_FALSE(error.empty());

  // EXACTLY 1.0 IS REFUSED TOO, and that is the case worth a test of its own:
  // there a correction of -|setpoint| writes exactly 0.0, which is
  // indistinguishable at the actuator from an HWR-30a stall latch — a wheel
  // switched off by the regulator wearing the appearance of the tier-1
  // response. FR-14 item 2: the trim must never be able to reduce the
  // feedforward contribution to zero.
  WheelRegulatorConfig exactly_one = test_config();
  exactly_one.max_correction_fraction = 1.0;
  EXPECT_FALSE(WheelRegulator::validate(exactly_one, error));

  // Just below it is legal: a full-trim experiment stays reachable, but the
  // number has to be typed and looks deliberate.
  WheelRegulatorConfig just_below = test_config();
  just_below.max_correction_fraction = 0.99;
  EXPECT_TRUE(WheelRegulator::validate(just_below, error)) << error;
}

TEST(WheelRegulator, ValidateRejectsNegativeGainsAndNonPositiveWindows)
{
  std::string error;
  WheelRegulatorConfig kp_negative = test_config();
  kp_negative.kp = -0.1;
  EXPECT_FALSE(WheelRegulator::validate(kp_negative, error));

  WheelRegulatorConfig ki_negative = test_config();
  ki_negative.ki = -0.1;
  EXPECT_FALSE(WheelRegulator::validate(ki_negative, error));

  WheelRegulatorConfig no_window = test_config();
  no_window.max_sample_age_sec = 0.0;
  EXPECT_FALSE(WheelRegulator::validate(no_window, error));

  WheelRegulatorConfig no_ceiling = test_config();
  no_ceiling.output_limit_rad_s = 0.0;
  EXPECT_FALSE(WheelRegulator::validate(no_ceiling, error));
}

TEST(WheelRegulator, ZeroGainsAreLegalAndInert)
{
  // A legitimate configuration: enabled for plumbing verification, with no
  // authority at all. It must produce exactly zero, not a NaN or a drift.
  WheelRegulatorConfig config = test_config();
  config.kp = 0.0;
  config.ki = 0.0;
  Rig rig(config);
  const auto result = rig.run(5.0, uniform(kReferenceSetpoint), uniform(2.4));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_TRUE(result.regulating[i]) << "wheel " << i;
    EXPECT_EQ(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_EQ(result.commands[i], kReferenceSetpoint) << "wheel " << i;
  }
}

// ---------------------------------------------------------------------------
// PER-WHEEL STATUS — FR-14 item 7, acceptance A1 / A3 / A6 / A8. "A regulator
// that quietly stops regulating one wheel looks exactly like a regulator that is
// working", so every wheel carries WHY and not only WHETHER.
// ---------------------------------------------------------------------------
TEST(WheelRegulatorStatus, DisabledReportsDisabledOnEveryWheel)
{
  WheelRegulatorConfig config = test_config();
  config.enabled = false;
  Rig rig(config);
  const auto result = rig.run(1.0, uniform(kReferenceSetpoint), uniform(2.4));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorDisabled) << "wheel " << i;
  }
}

TEST(WheelRegulatorStatus, RegulatingReportsActiveAndNotTheLimit)
{
  Rig rig(test_config());
  // A small error, well inside the 30 % bound.
  const auto result = rig.run(1.0, uniform(kReferenceSetpoint), uniform(kReferenceSetpoint - 0.05));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorActive) << "wheel " << i;
  }
}

TEST(WheelRegulatorStatus, SittingOnTheBoundIsItsOwnState)
{
  // FR-14: a regulator on its authority limit is reporting "the feedforward is
  // wrong for this surface" (OP-32). It must not look like a regulator with
  // nothing to do — and it is still REGULATING, not gated off.
  Rig rig(test_config());
  const auto result = rig.run(30.0, uniform(kReferenceSetpoint), uniform(1.0));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorAtAuthorityLimit)
      << "wheel " << i;
    EXPECT_TRUE(result.regulating[i]) << "wheel " << i;
  }
}

TEST(WheelRegulatorStatus, StandstillIsNotReportedAsSaturated)
{
  // At a zero setpoint the bound is zero and so is the correction. That is NOT
  // the authority limit being reached, and reporting it as such would make the
  // OP-32 signal fire every time the robot stands still.
  Rig rig(test_config());
  const auto result = rig.run(1.0, uniform(0.0), uniform(0.0));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_NE(result.status[i], gripperx_swerve_controller::kRegulatorAtAuthorityLimit)
      << "wheel " << i;
    EXPECT_EQ(result.correction[i], 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulatorStatus, APerfectlyStationaryMachineReportsTheFloorAndNotStaleness)
{
  // EXPECTATION CHANGED 2026-08-21 WITH THE SLOW-END FLOOR, and the old one is
  // written out because it is still HALF true. A machine that is genuinely
  // standing still emits a feed in which NOTHING CHANGES — same velocities, same
  // accumulated positions — and the only evidence of a new frame a state
  // interface can carry is a changed value. So a true standstill IS also stale,
  // and until the floor existed that is what this state reported.
  //
  // It now reports the FLOOR instead, because the floor is tested first, and
  // that ordering is A17's requirement rather than a preference: A17's own
  // scenario is "a command below the floor and a measured speed of zero", which
  // is precisely the case that is stale as well. Reporting staleness there would
  // hide the operative reason behind an incidental one. Staleness above the
  // floor is unaffected and is still exercised — see EachGateReportsItsOwnReason.
  //
  // Neither state has a control consequence here: at a zero setpoint the
  // authority bound is zero anyway, so a regulating wheel and a gated one command
  // exactly the same thing.
  Rig rig(test_config());
  const auto result = rig.run(1.0, uniform(0.0), uniform(0.0));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorOffBelowFloor)
      << "wheel " << i;
    EXPECT_EQ(result.commands[i], 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulatorStatus, EachGateReportsItsOwnReason)
{
  const auto setpoint = uniform(kReferenceSetpoint);
  const auto measured = uniform(2.4);

  {  // provenance
    Rig rig(test_config());
    rig.provenance[2] = kStallProvenanceLiveUnconfirmed;
    const auto result = rig.run(1.0, setpoint, measured);
    EXPECT_EQ(result.status[2], gripperx_swerve_controller::kRegulatorOffProvenance);
    EXPECT_EQ(result.status[0], gripperx_swerve_controller::kRegulatorActive);
  }
  {  // stall latch
    Rig rig(test_config());
    rig.latched[3] = true;
    const auto result = rig.run(1.0, setpoint, measured);
    EXPECT_EQ(result.status[3], gripperx_swerve_controller::kRegulatorOffStallLatched);
    EXPECT_EQ(result.status[0], gripperx_swerve_controller::kRegulatorActive);
  }
  {  // stale feedback
    Rig rig(test_config());
    rig.run(1.0, setpoint, measured);
    WheelRegulatorResult result;
    for (int c = 0; c < 9; ++c) {
      result = rig.step(setpoint, measured, /*fresh_sample=*/false);
    }
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorOffStaleFeedback)
        << "wheel " << i;
    }
  }
  {  // no usable measurement
    Rig rig(test_config());
    rig.run(1.0, setpoint, measured);
    WheelRegulatorInput input;
    rig.now += kDt;
    input.now_sec = rig.now;
    input.setpoint = setpoint;
    input.measured = measured;
    input.measured_valid = {true, true, true, false};
    input.position = rig.position;
    input.position_valid = all_true();
    input.provenance = rig.provenance;
    const auto result = rig.regulator.update(input);
    EXPECT_EQ(result.status[3], gripperx_swerve_controller::kRegulatorOffNoMeasurement);
    EXPECT_EQ(result.status[0], gripperx_swerve_controller::kRegulatorActive);
  }
}

// ---------------------------------------------------------------------------
// THE SLOW-END FLOOR — FR-14 item 12, acceptance A17, user decision 2026-08-21.
//
// THE FLOOR IS NOT A NUMBER OF ITS OWN: it is HWR-30a's arming threshold
// stall_min_command_rad_s, handed in per cycle from the detector's own config.
// The coupling IS the safety property — the regulator may only add authority
// where the stall detector is watching — so the last test in this block checks
// that it is a coupling and not a copy.
// ---------------------------------------------------------------------------
TEST(WheelRegulatorFloor, BelowTheFloorThereIsNoCorrectionAndTheIntegratorIsHeldReset)
{
  // A17 LITERALLY: "with a command below the configured floor and a measured
  // speed of zero, the published correction does not grow without bound and does
  // not sit at the authority limit indefinitely, and the per-wheel status
  // reports the reason". This is the machine below its drop-out threshold: a
  // maximal error against a wheel that will not start.
  Rig rig(test_config());
  const auto setpoint = uniform(1.9);  // 0.133 m/s at r = 0.070, below the 2.0 floor
  const auto measured = uniform(0.0);

  for (int c = 0; c < 900; ++c) {  // 30 s of a command that cannot move the machine
    const auto result = rig.step(setpoint, measured);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_EQ(result.correction[i], 0.0) << "wheel " << i << " cycle " << c;
      EXPECT_EQ(result.commands[i], setpoint[i]) << "wheel " << i << " cycle " << c;
      EXPECT_FALSE(result.regulating[i]) << "wheel " << i << " cycle " << c;
      EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i << " cycle " << c;
      EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorOffBelowFloor)
        << "wheel " << i << " cycle " << c;
      // The half of A17 that used to hold only by construction: it never even
      // approaches the authority limit, because it never regulates.
      EXPECT_NE(result.status[i], gripperx_swerve_controller::kRegulatorAtAuthorityLimit)
        << "wheel " << i << " cycle " << c;
    }
  }
}

TEST(WheelRegulatorFloor, TheReasonIsItsOwnCodeAndNotTheMissingMeasurementOne)
{
  // The user's constraint on this change: a reader must be able to tell "the
  // machine cannot be regulated here" from "the feedback is missing". Same
  // cycle, same wheel count, two different findings.
  Rig rig(test_config());
  rig.stall_min_command_rad_s = 2.0;

  WheelRegulatorInput input;
  input.now_sec = kDt;
  input.setpoint = {1.5, 4.286, 1.5, 4.286};
  input.measured = {0.0, 2.4, 0.0, 2.4};
  input.measured_valid = {true, true, false, false};  // wheels 2 and 3 have no feed
  input.position = uniform(0.0);
  input.position_valid = all_true();
  input.provenance = all_live();
  input.stall_min_command_rad_s = rig.stall_min_command_rad_s;

  const auto result = rig.regulator.update(input);
  EXPECT_EQ(result.status[0], gripperx_swerve_controller::kRegulatorOffBelowFloor);
  EXPECT_EQ(result.status[1], gripperx_swerve_controller::kRegulatorActive);
  // A fault outranks a design limit: an unusable measurement is reported even
  // though this wheel is ALSO below the floor.
  EXPECT_EQ(result.status[2], gripperx_swerve_controller::kRegulatorOffNoMeasurement);
  EXPECT_EQ(result.status[3], gripperx_swerve_controller::kRegulatorOffNoMeasurement);
}

TEST(WheelRegulatorFloor, ExactlyAtTheFloorIsOffBecauseTheDetectorIsNotArmedThere)
{
  // THE BOUNDARY IS THE DETECTOR'S BOUNDARY. stall_detector.cpp arms on
  // `command_magnitude > min_command_rad_s`, so AT the threshold nothing is
  // watching — and the regulator must be off in exactly that band, not one
  // comparison wider.
  Rig rig(test_config());
  const auto result = rig.run(1.0, uniform(2.0), uniform(1.0));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorOffBelowFloor)
      << "wheel " << i;
    EXPECT_EQ(result.correction[i], 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulatorFloor, JustAboveTheFloorItRegulatesNormally)
{
  // One machine epsilon of command above the boundary and everything works as it
  // does at any other speed: the floor is a gate, not a taper.
  Rig rig(test_config());
  const auto setpoint = uniform(2.0 + 1e-9);
  const auto result = rig.run(1.0, setpoint, uniform(1.0));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorActive) << "wheel " << i;
    EXPECT_TRUE(result.regulating[i]) << "wheel " << i;
    EXPECT_GT(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_GT(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulatorFloor, ANegativeCommandIsJudgedByItsMAGNITUDE)
{
  // Reverse is not "below" anything. -3.0 rad/s is above the floor; -1.0 is not.
  Rig rig(test_config());
  const auto result = rig.run(1.0, {-3.0, -1.0, -3.0, -1.0}, {-2.0, 0.0, -2.0, 0.0});
  EXPECT_EQ(result.status[0], gripperx_swerve_controller::kRegulatorActive);
  EXPECT_EQ(result.status[1], gripperx_swerve_controller::kRegulatorOffBelowFloor);
  EXPECT_LT(result.correction[0], 0.0);  // reverse shortfall, corrected downwards
  EXPECT_EQ(result.correction[1], 0.0);
}

TEST(WheelRegulatorFloor, CrossingTheFloorFromAboveCarriesNoIntegratorState)
{
  // The wind-up A17 forbids, approached from the other side: authority earned at
  // speed must not be waiting for the wheel when the command comes back up
  // through the floor after a slow crawl.
  Rig rig(test_config());
  const auto fast = uniform(kReferenceSetpoint);
  rig.run(5.0, fast, uniform(2.4));
  const double wound_up = rig.regulator.integrator(0);
  ASSERT_GT(wound_up, 0.0);

  // Down into the crawl. The integrator goes to zero on the FIRST cycle below
  // the floor and stays there, however long the crawl lasts.
  const auto crawl = uniform(1.0);
  const auto first_below = rig.step(crawl, uniform(0.3));
  EXPECT_EQ(rig.regulator.integrator(0), 0.0);
  EXPECT_EQ(first_below.correction[0], 0.0);
  rig.run(10.0, crawl, uniform(0.3));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }

  // And back up. The very first regulated cycle carries exactly ONE integration
  // step, not the authority earned before the crawl.
  const auto measured = uniform(2.4);
  const auto back_up = rig.step(fast, measured);
  const double error = kReferenceSetpoint - 2.4;
  EXPECT_TRUE(back_up.regulating[0]);
  EXPECT_NEAR(rig.regulator.integrator(0), test_config().ki * error * kDt, 1e-9);
  EXPECT_NEAR(
    back_up.correction[0], test_config().kp * error + test_config().ki * error * kDt, 1e-9);
  EXPECT_LT(back_up.correction[0], wound_up);
}

TEST(WheelRegulatorFloor, ChatterAcrossTheFloorCannotWindUp)
{
  // NO HYSTERESIS IS DELIBERATE (see the floor gate in wheel_regulator.cpp), so
  // a command hovering on the boundary toggles regulation every cycle and resets
  // the integrator every time it dips. THIS TEST IS THE JUSTIFICATION: the
  // toggling is bounded and safe. However long the chatter lasts, the correction
  // can never exceed one proportional term plus one integration step — it is
  // strictly LESS authority than a steady command would earn, never more.
  Rig rig(test_config());
  const std::array<double, kNumWheels> above = uniform(2.5);
  const std::array<double, kNumWheels> below = uniform(1.9);
  const auto measured = uniform(1.0);  // a real, changing feed: samples stay new
  const double error = 2.5 - 1.0;
  const double one_step = test_config().kp * error + test_config().ki * error * kDt;

  int active = 0;
  int floored = 0;
  for (int c = 0; c < 600; ++c) {  // 20 s of hovering on the boundary
    const auto result = rig.step((c % 2 == 0) ? above : below, measured);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_LE(std::fabs(result.correction[i]), one_step + 1e-9)
        << "wheel " << i << " cycle " << c;
    }
    if (result.status[0] == gripperx_swerve_controller::kRegulatorActive) {
      ++active;
    }
    if (result.status[0] == gripperx_swerve_controller::kRegulatorOffBelowFloor) {
      ++floored;
    }
  }
  // The STATUS toggles, which is cosmetic and is what a reader will see.
  EXPECT_EQ(active, 300);
  EXPECT_EQ(floored, 300);
}

TEST(WheelRegulatorFloor, TheFloorFOLLOWSTheStallThresholdAndIsNotASnapshotOfIt)
{
  // THE SAFETY PROPERTY OF THE WHOLE DESIGN. If someone moves HWR-30a's arming
  // threshold, the regulator's floor must move with it, so that "the regulator
  // only adds authority where the detector is watching" survives the edit. A
  // value copied once into the regulator's own config would pass every other
  // test in this file and fail this one.
  Rig rig(test_config());
  const auto setpoint = uniform(2.5);
  const auto measured = uniform(1.0);

  rig.stall_min_command_rad_s = 2.0;
  auto result = rig.run(1.0, setpoint, measured);
  ASSERT_EQ(result.status[0], gripperx_swerve_controller::kRegulatorActive);
  ASSERT_GT(result.correction[0], 0.0);

  // The detector is told to arm only above 3.0 rad/s. The regulator's coverage
  // must retreat with it on the very next cycle, with no reconfigure.
  rig.stall_min_command_rad_s = 3.0;
  result = rig.step(setpoint, measured);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorOffBelowFloor)
      << "wheel " << i;
    EXPECT_EQ(result.correction[i], 0.0) << "wheel " << i;
    EXPECT_EQ(rig.regulator.integrator(i), 0.0) << "wheel " << i;
  }

  // And back down again: the same command is inside the watched band once more.
  rig.stall_min_command_rad_s = 2.0;
  result = rig.step(setpoint, measured);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorActive) << "wheel " << i;
    EXPECT_GT(result.correction[i], 0.0) << "wheel " << i;
  }
}

TEST(WheelRegulatorFloor, TheDefaultFloorIsTheStallDetectorsOwnDefault)
{
  // ONE DECLARATION, NOT TWO. The input's default is taken from
  // StallDetectorConfig rather than written out as a literal, so a caller that
  // forgets to set the floor still gets HWR-30a's threshold — and gets a floor
  // rather than none, which is the safe direction.
  WheelRegulatorInput input;
  EXPECT_EQ(input.stall_min_command_rad_s, StallDetectorConfig{}.min_command_rad_s);
}
