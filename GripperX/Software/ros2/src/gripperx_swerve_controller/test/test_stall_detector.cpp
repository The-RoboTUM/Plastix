// Unit checks for HWR-30a's stall detection and tier-1 response. No ROS in here
// on purpose — the whole state machine is rclcpp-free (stall_detector.hpp), so
// every branch that matters on the robot can be exercised on a laptop without a
// stack, without hardware and without moving anything.
//
// WHAT THESE TESTS CANNOT SHOW: that the thresholds are RIGHT. Every threshold
// in StallDetectorConfig is TO-VERIFY because the window it protects is a
// thermal property of the thin motor lead at stall current and the GB37-50
// stall-current measurement does not exist. These tests check the LOGIC against
// whatever thresholds it is given.

#include <gtest/gtest.h>

#include <array>
#include <string>

#include "gripperx_swerve_controller/stall_detector.hpp"

using gripperx_swerve_controller::kNumWheels;
using gripperx_swerve_controller::kStallProvenanceLive;
using gripperx_swerve_controller::kStallProvenanceLiveUnconfirmed;
using gripperx_swerve_controller::kStallProvenanceNoEncoder;
using gripperx_swerve_controller::kStallProvenanceUnknown;
using gripperx_swerve_controller::StallDetector;
using gripperx_swerve_controller::StallDetectorConfig;
using gripperx_swerve_controller::StallDetectorResult;

namespace
{
constexpr double kDt = 1.0 / 30.0;  // controller_manager update_rate

StallDetectorConfig test_config()
{
  StallDetectorConfig config;
  config.enabled = true;
  config.window_sec = 1.0;
  config.min_command_rad_s = 2.0;
  config.min_position_delta_rad = 0.05;
  config.release_command_rad_s = 0.1;
  config.max_latched_wheels = 1;
  config.assume_live_provenance = false;
  return config;
}

std::array<int, kNumWheels> all_live()
{
  return {kStallProvenanceLive, kStallProvenanceLive, kStallProvenanceLive, kStallProvenanceLive};
}

std::array<bool, kNumWheels> all_valid() { return {true, true, true, true}; }

/// Run `seconds` worth of 30 Hz cycles with a constant command and constant
/// per-wheel position rates. Returns the last result.
struct Rig
{
  StallDetector detector;
  double now{0.0};
  std::array<double, kNumWheels> position{};

  explicit Rig(const StallDetectorConfig & config) : detector(config)
  {
    detector.set_provenance(all_live());
  }

  /// `drive_withheld` defaults to false here — and ONLY here, in the harness.
  /// The production signature deliberately has no default (stall_detector.hpp):
  /// the point of the default in this file is that every pre-existing test keeps
  /// asserting the ungated behaviour VERBATIM, so if the fix changed any of it
  /// the suite would say so.
  StallDetectorResult step(
    const std::array<double, kNumWheels> & command,
    const std::array<double, kNumWheels> & position_rate, bool drive_withheld = false)
  {
    now += kDt;
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      position[i] += position_rate[i] * kDt;
    }
    return detector.update(now, command, position, all_valid(), drive_withheld);
  }

  StallDetectorResult run(
    double seconds, const std::array<double, kNumWheels> & command,
    const std::array<double, kNumWheels> & position_rate, bool drive_withheld = false)
  {
    StallDetectorResult result;
    const int cycles = static_cast<int>(seconds / kDt);
    for (int c = 0; c < cycles; ++c) {
      result = step(command, position_rate, drive_withheld);
      for (std::size_t i = 0; i < kNumWheels; ++i) {
        seen_multi_wheel_refused = seen_multi_wheel_refused || result.events[i].multi_wheel_refused;
      }
    }
    return result;
  }

  /// The event flags are EDGES on single cycles, so a test that only inspects
  /// the last cycle of a run would miss them. Accumulated here.
  bool seen_multi_wheel_refused{false};
};

const std::array<double, kNumWheels> kDriving{7.0, 7.0, 7.0, 7.0};
const std::array<double, kNumWheels> kStopped{0.0, 0.0, 0.0, 0.0};
const std::array<double, kNumWheels> kTurning{7.0, 7.0, 7.0, 7.0};
}  // namespace

// --------------------------------------------------------------- validation
TEST(StallDetectorConfig, RejectsAReleaseBandThatRemovesTheHysteresis)
{
  std::string error;
  auto config = test_config();
  config.release_command_rad_s = config.min_command_rad_s;
  EXPECT_FALSE(StallDetector::validate(config, error));
  EXPECT_FALSE(error.empty());
}

TEST(StallDetectorConfig, RejectsANonPositiveWindow)
{
  std::string error;
  auto config = test_config();
  config.window_sec = 0.0;
  EXPECT_FALSE(StallDetector::validate(config, error));
}

TEST(StallDetectorConfig, AcceptsTheDefaults)
{
  std::string error;
  EXPECT_TRUE(StallDetector::validate(StallDetectorConfig{}, error)) << error;
}

// ---------------------------------------------------------------- detection
TEST(StallDetector, HealthyWheelsNeverTrip)
{
  Rig rig(test_config());
  const auto result = rig.run(10.0, kDriving, kTurning);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(rig.detector.latched(i));
    EXPECT_DOUBLE_EQ(result.commands[i], kDriving[i]);
    EXPECT_TRUE(rig.detector.armed(i));
  }
}

TEST(StallDetector, BlockedWheelTripsAfterTheWindowAndOnlyThatWheelIsZeroed)
{
  Rig rig(test_config());
  // FR wheel (index 1) is blocked; the other three turn.
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};

  auto result = rig.run(0.9, kDriving, rate);
  EXPECT_FALSE(rig.detector.latched(1)) << "tripped before the window elapsed";

  result = rig.run(0.3, kDriving, rate);
  EXPECT_TRUE(rig.detector.latched(1));
  EXPECT_EQ(rig.detector.trip_count(1), 1u);

  // TIER 1: only the affected wheel. This is the acceptance criterion's
  // "unaffected motors keep running".
  EXPECT_DOUBLE_EQ(result.commands[1], 0.0);
  EXPECT_DOUBLE_EQ(result.commands[0], 7.0);
  EXPECT_DOUBLE_EQ(result.commands[2], 7.0);
  EXPECT_DOUBLE_EQ(result.commands[3], 7.0);
  for (std::size_t i : {0u, 2u, 3u}) {
    EXPECT_FALSE(rig.detector.latched(i));
  }
}

TEST(StallDetector, CreepBelowTheDeltaStillTrips)
{
  // Slipping/creeping barely at all is a stall for this purpose: less than
  // min_position_delta_rad of movement inside the whole window.
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.01, 7.0, 7.0};  // 0.01 rad/s -> 0.01 rad in 1 s
  rig.run(1.5, kDriving, rate);
  EXPECT_TRUE(rig.detector.latched(1));
}

TEST(StallDetector, MovementJustAboveTheDeltaKeepsTheWindowOpen)
{
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.2, 7.0, 7.0};  // 0.2 rad/s > 0.05 rad per 0.25 s
  rig.run(30.0, kDriving, rate);
  EXPECT_FALSE(rig.detector.latched(1));
}

// -------------------------------------------------- the provenance gate (b)
TEST(StallDetector, DoesNotArmWithoutLiveProvenance)
{
  // The whole point of HWR-30a keying off encoder-valid: a wheel whose encoder
  // is not known-live must not be judged by its counts, because "no counts" and
  // "no encoder" are the same data.
  for (const int code :
       {kStallProvenanceUnknown, kStallProvenanceNoEncoder, kStallProvenanceLiveUnconfirmed})
  {
    Rig rig(test_config());
    rig.detector.set_provenance({code, code, code, code});
    const auto result = rig.run(10.0, kDriving, kStopped);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_FALSE(rig.detector.latched(i)) << "provenance code " << code;
      EXPECT_FALSE(rig.detector.armed(i)) << "provenance code " << code;
      EXPECT_DOUBLE_EQ(result.commands[i], kDriving[i]);
    }
  }
}

TEST(StallDetector, AssumeLiveProvenanceArmsWithoutTheTopic)
{
  // Sim policy: nothing publishes /hw/wheel_feedback_valid in the twin.
  auto config = test_config();
  config.assume_live_provenance = true;
  Rig rig(config);
  rig.detector.set_provenance(
    {kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
     kStallProvenanceUnknown});
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};
  rig.run(1.5, kDriving, rate);
  EXPECT_TRUE(rig.detector.latched(1));
}

TEST(StallDetector, UnreadablePositionDisarmsRatherThanTrips)
{
  StallDetector detector(test_config());
  detector.set_provenance(all_live());
  std::array<double, kNumWheels> position{};
  const std::array<bool, kNumWheels> invalid{false, false, false, false};
  double now = 0.0;
  StallDetectorResult result;
  for (int c = 0; c < 300; ++c) {
    now += kDt;
    result = detector.update(now, kDriving, position, invalid, false);
  }
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(detector.latched(i));
    EXPECT_DOUBLE_EQ(result.commands[i], kDriving[i]);
  }
}

// ----------------------------------------------------- the arming threshold
TEST(StallDetector, ACommandBelowTheArmingThresholdNeverTrips)
{
  // A command too small to break the motor away is not a stall. 1.0 rad/s is
  // below min_command_rad_s = 2.0.
  Rig rig(test_config());
  const std::array<double, kNumWheels> tiny{1.0, 1.0, 1.0, 1.0};
  const auto result = rig.run(10.0, tiny, kStopped);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(rig.detector.latched(i));
    EXPECT_DOUBLE_EQ(result.commands[i], tiny[i]);
  }
}

// --------------------------------------------------------------- OP-25 latch
TEST(StallDetector, AHeldCommandNeverReleasesTheLatch)
{
  // THE OP-25 PROPERTY. /cmd_vel runs at 30 Hz whether or not anything changed;
  // a latch that clears on "the next command received" re-energises ~33 ms
  // later and chatters with inrush current every cycle.
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};
  rig.run(1.5, kDriving, rate);
  ASSERT_TRUE(rig.detector.latched(1));

  for (int c = 0; c < 600; ++c) {  // 20 s of the SAME held command
    const auto result = rig.step(kDriving, rate);
    EXPECT_TRUE(rig.detector.latched(1));
    EXPECT_DOUBLE_EQ(result.commands[1], 0.0);
    EXPECT_FALSE(result.events[1].released);
  }
  EXPECT_EQ(rig.detector.trip_count(1), 1u) << "re-tripped, i.e. it chattered";
}

TEST(StallDetector, ReleaseNeedsBothEdgesInOrder)
{
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};
  rig.run(1.5, kDriving, rate);
  ASSERT_TRUE(rig.detector.latched(1));

  // Edge 1: the command falls into the release band. Still latched — falling is
  // not by itself a fresh command.
  rig.run(0.5, kStopped, kStopped);
  EXPECT_TRUE(rig.detector.latched(1));

  // Edge 2: it rises above the arming threshold again. NOW it is fresh.
  const auto result = rig.step(kDriving, rate);
  EXPECT_FALSE(rig.detector.latched(1));
  EXPECT_TRUE(result.events[1].released);
  EXPECT_DOUBLE_EQ(result.commands[1], 7.0);
}

TEST(StallDetector, AReleasedWheelGetsAFullFreshWindowBeforeItCanTripAgain)
{
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};
  rig.run(1.5, kDriving, rate);
  rig.run(0.5, kStopped, kStopped);
  rig.step(kDriving, rate);
  ASSERT_FALSE(rig.detector.latched(1));

  rig.run(0.9, kDriving, rate);
  EXPECT_FALSE(rig.detector.latched(1)) << "re-tripped inside the window";
  rig.run(0.3, kDriving, rate);
  EXPECT_TRUE(rig.detector.latched(1));
  EXPECT_EQ(rig.detector.trip_count(1), 2u);
}

// ------------------------------------------------------- tier 2 is not built
TEST(StallDetector, ASecondSimultaneousStallIsReportedNotLatched)
{
  // "More than one motor" is HWR-30 tier 2, blocked on the unmeasured stall
  // current. It must not be silently approximated by a second tier-1 latch.
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{0.0, 0.0, 7.0, 7.0};
  const auto result = rig.run(1.5, kDriving, rate);

  EXPECT_EQ(rig.detector.latched_count(), 1u);
  EXPECT_TRUE(rig.seen_multi_wheel_refused);
  // The second blocked wheel is still being DRIVEN — the condition is reported,
  // not acted on, because "more than one motor" is tier 2 and tier 2 is blocked.
  EXPECT_DOUBLE_EQ(result.commands[1], 7.0);
}

TEST(StallDetector, AFrozenFeedbackPathDoesNotBecomeAWholeMachineStop)
{
  // If /hw/joint_states stops arriving, every wheel POSITION freezes at once
  // while the commands keep flowing. That is one lost feedback path, not four
  // stalls, and at most one wheel may be switched off by it.
  Rig rig(test_config());
  const auto result = rig.run(5.0, kDriving, kStopped);
  EXPECT_EQ(rig.detector.latched_count(), 1u);
  int zeroed = 0;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    zeroed += (result.commands[i] == 0.0) ? 1 : 0;
  }
  EXPECT_EQ(zeroed, 1);
}

// ------------------------------------------------------------------ misc
TEST(StallDetector, DisabledDetectorIsInert)
{
  auto config = test_config();
  config.enabled = false;
  Rig rig(config);
  const auto result = rig.run(10.0, kDriving, kStopped);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(rig.detector.latched(i));
    EXPECT_DOUBLE_EQ(result.commands[i], kDriving[i]);
  }
}

TEST(StallDetector, ResetClearsEveryLatch)
{
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};
  rig.run(1.5, kDriving, rate);
  ASSERT_TRUE(rig.detector.latched(1));
  rig.detector.reset();
  EXPECT_FALSE(rig.detector.latched(1));
  EXPECT_EQ(rig.detector.trip_count(1), 0u);
  EXPECT_EQ(rig.detector.latched_count(), 0u);
}

TEST(StallDetector, ABackwardsClockRestartsTheWindowInsteadOfTripping)
{
  // A sim /clock that resets (a bag replayed from the start) must not read as a
  // long elapsed window. Related to D17's sim-clock hazard.
  StallDetector detector(test_config());
  detector.set_provenance(all_live());
  std::array<double, kNumWheels> position{};
  double now = 100.0;
  for (int c = 0; c < 15; ++c) {
    now += kDt;
    detector.update(now, kDriving, position, all_valid(), false);
  }
  now = 0.0;
  for (int c = 0; c < 15; ++c) {
    now += kDt;
    detector.update(now, kDriving, position, all_valid(), false);
  }
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(detector.latched(i));
  }
}

// ------------------------------------- the alignment gate's hold (F4 / P5a)
// THE DEFECT THESE PIN. Stage 2 of SwerveController::write_wheel_commands
// replaces all four wheel commands with exactly 0.0 while the steering modules
// slew, and stage 3 — this detector — is handed the REQUESTED command, which
// stage 2 never touched. The actuator gets 0.0, so the wheel cannot move, while
// the detector sees the requested command. Before `drive_withheld` existed, ANY
// hold of window_sec or more under a request above min_command_rad_s latched a
// motor off.
//
// 3.0 rad/s is used throughout: comfortably above min_command_rad_s (2.0).
namespace
{
const std::array<double, kNumWheels> kRequestDuringHold{3.0, 3.0, 3.0, 3.0};
}  // namespace

TEST(StallDetector, AGateHoldLongerThanTheWindowLatchesNothing)
{
  Rig rig(test_config());
  // 1.5 s of hold against window_sec = 1.0 s. The wheels do not move because
  // the actuator is receiving exactly 0.0 — that is the gate working, not a
  // stall.
  const auto result = rig.run(1.5, kRequestDuringHold, kStopped, /*drive_withheld=*/true);

  EXPECT_EQ(rig.detector.latched_count(), 0u);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(rig.detector.latched(i)) << "wheel " << i << " latched on a gate hold";
    EXPECT_EQ(rig.detector.trip_count(i), 0u) << "wheel " << i;
    // "The detector is asleep" must not look like "the detector is happy" —
    // armed() is published for exactly this reason.
    EXPECT_FALSE(rig.detector.armed(i)) << "wheel " << i;
    // And the detector's authority is unchanged: it neither zeroes nor trims.
    // The zero on the wire is the GATE's, applied in stage 2.
    EXPECT_DOUBLE_EQ(result.commands[i], kRequestDuringHold[i]);
  }
  EXPECT_FALSE(result.state_changed);
}

TEST(StallDetector, NoHoldIsLongEnoughToLatch)
{
  // The deployed alignment_timeout_sec (ros2_controllers.yaml) is several
  // times window_sec, and the gate can also re-engage back to back. Duration
  // must be irrelevant, not merely survivable — and no MULTI-WHEEL report may
  // be manufactured either, since during a hold all four wheels freeze
  // together and that is the signature ros2_controllers.yaml attributes to a
  // lost /hw/joint_states.
  Rig rig(test_config());
  rig.run(30.0, kRequestDuringHold, kStopped, /*drive_withheld=*/true);
  EXPECT_EQ(rig.detector.latched_count(), 0u);
  EXPECT_FALSE(rig.seen_multi_wheel_refused);
}

TEST(StallDetector, ReleasingTheGateGivesTheWheelAFullFreshWindow)
{
  // THE OTHER HALF OF THE FIX, and the half that could have been got wrong: the
  // dwell timer must RESTART at the release, not resume. A wheel breaking away
  // from standstill gets the whole window_sec to move min_position_delta_rad,
  // measured from a baseline sampled at the release — exactly what a command
  // rising from rest through min_command_rad_s already gets.
  Rig rig(test_config());
  rig.run(3.0, kRequestDuringHold, kStopped, /*drive_withheld=*/true);
  ASSERT_EQ(rig.detector.latched_count(), 0u);

  // Gate released. The wheel is STILL not moving — but it has only just been
  // given the drive back, so the first 0.9 s may not be held against it.
  rig.run(0.9, kRequestDuringHold, kStopped, /*drive_withheld=*/false);
  EXPECT_EQ(rig.detector.latched_count(), 0u) << "the pre-hold dwell carried over";

  // Past a full window with the drive flowing and the wheel still dead, this IS
  // a stall and it must latch. The fix must not blind the protective function.
  rig.run(0.3, kRequestDuringHold, kStopped, /*drive_withheld=*/false);
  EXPECT_EQ(rig.detector.latched_count(), 1u);
}

TEST(StallDetector, AHoldInTheMiddleOfAWindowDoesNotAccumulateAcrossIt)
{
  // The complement of the test above: dwell on either side of a hold must not
  // be added together. 0.9 s of genuine standstill, a short hold, another 0.9 s
  // of genuine standstill — 1.8 s of frozen wheel in total, but no CONTIGUOUS
  // armed stretch of a full second, so nothing may latch yet.
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};  // FR blocked
  rig.run(0.9, kDriving, rate, /*drive_withheld=*/false);
  ASSERT_FALSE(rig.detector.latched(1));

  rig.run(0.5, kRequestDuringHold, kStopped, /*drive_withheld=*/true);
  rig.run(0.9, kDriving, rate, /*drive_withheld=*/false);
  EXPECT_FALSE(rig.detector.latched(1)) << "dwell accumulated across the hold";

  rig.run(0.3, kDriving, rate, /*drive_withheld=*/false);
  EXPECT_TRUE(rig.detector.latched(1));
}

TEST(StallDetector, ARealStallWithNoHoldLatchesExactlyAsBefore)
{
  // THE REGRESSION THAT MATTERS MOST. Blinding the protective function would be
  // a worse defect than the phantom latches this change removes, so the
  // ungated timing is pinned to the cycle: nothing before 0.9 s, latched by
  // 1.2 s, only that wheel zeroed.
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};
  rig.run(0.9, kDriving, rate, /*drive_withheld=*/false);
  EXPECT_FALSE(rig.detector.latched(1)) << "tripped before the window elapsed";

  const auto result = rig.run(0.3, kDriving, rate, /*drive_withheld=*/false);
  EXPECT_TRUE(rig.detector.latched(1));
  EXPECT_EQ(rig.detector.trip_count(1), 1u);
  EXPECT_DOUBLE_EQ(result.commands[1], 0.0);
  for (std::size_t i : {0u, 2u, 3u}) {
    EXPECT_FALSE(rig.detector.latched(i));
    EXPECT_DOUBLE_EQ(result.commands[i], 7.0);
  }
}

TEST(StallDetector, AGateHoldNeitherClearsNorRetripsAnExistingLatch)
{
  // OP-25 is untouched by this change: the release edge is still judged on the
  // REQUESTED command, so a hold neither releases a latched wheel nor lets it
  // trip a second time, and the latched wheel keeps being written to exactly
  // 0.0 by stage 3 while the gate happens to be zeroing everything anyway.
  Rig rig(test_config());
  const std::array<double, kNumWheels> rate{7.0, 0.0, 7.0, 7.0};
  rig.run(1.5, kDriving, rate);
  ASSERT_TRUE(rig.detector.latched(1));

  const auto result = rig.run(3.0, kRequestDuringHold, kStopped, /*drive_withheld=*/true);
  EXPECT_TRUE(rig.detector.latched(1));
  EXPECT_EQ(rig.detector.trip_count(1), 1u);
  EXPECT_DOUBLE_EQ(result.commands[1], 0.0);

  // …and the two-edge release still works afterwards, in that order.
  rig.run(0.5, kStopped, kStopped);
  const auto released = rig.step(kDriving, rate);
  EXPECT_FALSE(rig.detector.latched(1));
  EXPECT_TRUE(released.events[1].released);
}
