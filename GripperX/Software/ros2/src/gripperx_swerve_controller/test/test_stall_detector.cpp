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

  StallDetectorResult step(
    const std::array<double, kNumWheels> & command,
    const std::array<double, kNumWheels> & position_rate)
  {
    now += kDt;
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      position[i] += position_rate[i] * kDt;
    }
    return detector.update(now, command, position, all_valid());
  }

  StallDetectorResult run(
    double seconds, const std::array<double, kNumWheels> & command,
    const std::array<double, kNumWheels> & position_rate)
  {
    StallDetectorResult result;
    const int cycles = static_cast<int>(seconds / kDt);
    for (int c = 0; c < cycles; ++c) {
      result = step(command, position_rate);
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
    result = detector.update(now, kDriving, position, invalid);
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
    detector.update(now, kDriving, position, all_valid());
  }
  now = 0.0;
  for (int c = 0; c < 15; ++c) {
    now += kDt;
    detector.update(now, kDriving, position, all_valid());
  }
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(detector.latched(i));
  }
}
