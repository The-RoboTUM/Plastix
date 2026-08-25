// Unit checks for the steering alignment gate. No ROS in here on purpose — the
// whole state machine is rclcpp-free (alignment_gate.hpp), so every branch that
// matters on the robot can be exercised on a laptop without a stack, without
// hardware and without moving anything.
//
// WHAT THESE TESTS CANNOT SHOW: that the thresholds are RIGHT. entry_jump_rad
// and entry_error_rad are TO-VERIFY — they are ARGUED from the three-orders-of-
// magnitude separation between a per-cycle DWB target change and a crab entry,
// not measured on this machine. These tests check the LOGIC against whatever
// thresholds it is given.
//
// The scenarios below are the ones gripperx_teleop/test/check_manoeuvres.py
// exercises against TransitionGuard, restated for the controller-side gate, so
// the behaviour the teleop guard was accepted for can be compared item by item
// against the behaviour that replaces it.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

#include "gripperx_swerve_controller/alignment_gate.hpp"

using gripperx_swerve_controller::AlignmentGate;
using gripperx_swerve_controller::AlignmentGateConfig;
using gripperx_swerve_controller::kAlignDisabled;
using gripperx_swerve_controller::kAlignPassing;
using gripperx_swerve_controller::kAlignSlewing;
using gripperx_swerve_controller::kAlignTimedOut;
using gripperx_swerve_controller::kNumWheels;

namespace
{

constexpr double kDeg = M_PI / 180.0;

using Wheels = std::array<double, kNumWheels>;

/// The wheel commands a drive request produces before the gate sees them.
const Wheels kDriving{4.0, 4.0, 4.0, 4.0};
const Wheels kStraight{0.0, 0.0, 0.0, 0.0};

/// The crab pose as resolved through the +-180 deg fold, joint order FL, FR, BL,
/// BR — the real one, taken from the 16 twin runs recorded in nav2.yaml.
const Wheels kCrab{-90.0 * kDeg, 90.0 * kDeg, 90.0 * kDeg, -90.0 * kDeg};

AlignmentGateConfig enabled_config()
{
  AlignmentGateConfig config;
  config.enabled = true;
  return config;
}

}  // namespace

// ── configuration ──────────────────────────────────────────────────────────

TEST(AlignmentGateConfig, ShipsDisabled)
{
  // The property that makes this mergeable: the shipped default changes nothing
  // about how the robot drives.
  EXPECT_FALSE(AlignmentGateConfig{}.enabled);
}

TEST(AlignmentGateConfig, AcceptsTheDefaults)
{
  std::string error;
  EXPECT_TRUE(AlignmentGate::validate(AlignmentGateConfig{}, error)) << error;
}

TEST(AlignmentGateConfig, RejectsAnEntryThresholdThatRemovesTheHysteresis)
{
  // Entry at or below exit makes one steady pose chop the drive on and off at
  // the controller rate — worse than not guarding at all.
  AlignmentGateConfig config = enabled_config();
  config.entry_jump_rad = config.exit_tolerance_rad;
  std::string error;
  EXPECT_FALSE(AlignmentGate::validate(config, error));
  EXPECT_NE(error.find("oscillates"), std::string::npos) << error;

  config = enabled_config();
  config.entry_error_rad = config.exit_tolerance_rad * 0.5;
  EXPECT_FALSE(AlignmentGate::validate(config, error));
}

TEST(AlignmentGateConfig, RejectsANonPositiveTimeout)
{
  AlignmentGateConfig config = enabled_config();
  config.timeout_sec = 0.0;
  std::string error;
  EXPECT_FALSE(AlignmentGate::validate(config, error));
}

// ── disabled behaviour ─────────────────────────────────────────────────────

TEST(AlignmentGate, DisabledPassesEveryCommandThroughUntouched)
{
  AlignmentGate gate{AlignmentGateConfig{}};  // enabled == false
  // Even the most violent transition imaginable: straight to full crab in one
  // cycle, with the modules still standing straight.
  const auto result = gate.update(0.0, kDriving, kCrab, kStraight, true);
  EXPECT_EQ(result.status, kAlignDisabled);
  EXPECT_EQ(result.commands, kDriving);
  EXPECT_EQ(gate.engage_count(), 0u);
}

// ── the case this class exists for ─────────────────────────────────────────

TEST(AlignmentGate, CrabEntryWithholdsDriveUntilTheModulesArrive)
{
  AlignmentGate gate{enabled_config()};

  // Driving straight: nothing to guard.
  auto result = gate.update(0.0, kDriving, kStraight, kStraight, true);
  EXPECT_EQ(result.status, kAlignPassing);
  EXPECT_EQ(result.commands, kDriving);

  // The operator asks for a crab. The target jumps 90 deg while the modules are
  // still straight — drive must go to EXACTLY zero, on all four wheels.
  result = gate.update(0.01, kDriving, kCrab, kStraight, true);
  EXPECT_EQ(result.status, kAlignSlewing);
  EXPECT_TRUE(result.state_changed);
  EXPECT_EQ(result.commands, kStraight);
  EXPECT_EQ(gate.engage_count(), 1u);

  // Modules slew. Half way there is still not there.
  const Wheels half{-45.0 * kDeg, 45.0 * kDeg, 45.0 * kDeg, -45.0 * kDeg};
  result = gate.update(0.3, kDriving, kCrab, half, true);
  EXPECT_EQ(result.status, kAlignSlewing);
  EXPECT_EQ(result.commands, kStraight);
  EXPECT_FALSE(result.state_changed);

  // Arrived, inside tolerance but not exactly on target — this is what the twin
  // actually reports (FL -89.95..-89.99 deg over 16 runs).
  const Wheels arrived{-89.95 * kDeg, 89.97 * kDeg, 89.96 * kDeg, -89.99 * kDeg};
  result = gate.update(0.6, kDriving, kCrab, arrived, true);
  EXPECT_EQ(result.status, kAlignPassing);
  EXPECT_TRUE(result.state_changed);
  EXPECT_EQ(result.commands, kDriving);
}

TEST(AlignmentGate, NeverZeroesASubsetOfTheWheels)
{
  // Three modules have arrived, one is still 40 deg out. Driving the three that
  // are ready is the motion "in a direction nobody asked for".
  AlignmentGate gate{enabled_config()};
  gate.update(0.0, kDriving, kStraight, kStraight, true);
  gate.update(0.01, kDriving, kCrab, kStraight, true);

  const Wheels nearly{-90.0 * kDeg, 90.0 * kDeg, 90.0 * kDeg, -50.0 * kDeg};
  const auto result = gate.update(0.4, kDriving, kCrab, nearly, true);
  EXPECT_EQ(result.status, kAlignSlewing);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_DOUBLE_EQ(result.commands[i], 0.0) << "wheel " << i;
  }
}

// ── what must NOT trigger it ───────────────────────────────────────────────

TEST(AlignmentGate, OrdinaryCorneringNeverEngages)
{
  // A DWB-style twist moves the commanded pose by a fraction of a degree per
  // cycle and the modules track it with a small standing lag. If this engaged,
  // the robot could not drive through a corner at all.
  AlignmentGate gate{enabled_config()};
  double target = 0.0;
  for (int cycle = 0; cycle < 500; ++cycle) {
    target += 0.05 * kDeg;                       // 0.05 deg per cycle
    const double measured = target - 4.0 * kDeg;  // a steady 4 deg of tracking lag
    Wheels targets;
    Wheels measurements;
    targets.fill(target);
    measurements.fill(measured);
    const auto result = gate.update(cycle * 0.01, kDriving, targets, measurements, true);
    ASSERT_EQ(result.status, kAlignPassing) << "engaged on cycle " << cycle;
    ASSERT_EQ(result.commands, kDriving);
  }
  EXPECT_EQ(gate.engage_count(), 0u);
}

TEST(AlignmentGate, AHeldSteeringCycleIsNotATransition)
{
  // OP-24/S1: a stale or exactly-zero twist HOLDS the steering, meaning the
  // command interfaces are not written. There is no new target to align to, and
  // those branches have already zeroed the wheels for their own reasons.
  AlignmentGate gate{enabled_config()};
  gate.update(0.0, kDriving, kStraight, kStraight, true);

  const auto result = gate.update(0.01, kStraight, kCrab, kStraight, false);
  EXPECT_EQ(result.status, kAlignPassing);
  EXPECT_EQ(gate.engage_count(), 0u);

  // …and the held cycle must not poison the next real one: the remembered
  // target is still `straight`, so the crab that follows is a genuine jump.
  const auto next = gate.update(0.02, kDriving, kCrab, kStraight, true);
  EXPECT_EQ(next.status, kAlignSlewing);
}

// ── entry condition 2: an error that was already large ─────────────────────

TEST(AlignmentGate, EngagesOnALargeStandingErrorWithoutAnyJump)
{
  // The controller activates with the wheels parked at the crab pose and the
  // first commanded pose is straight ahead. There is no PREVIOUS target, so
  // condition 1 cannot see anything — condition 2 is what catches this.
  AlignmentGate gate{enabled_config()};
  const auto result = gate.update(0.0, kDriving, kStraight, kCrab, true);
  EXPECT_EQ(result.status, kAlignSlewing);
  EXPECT_EQ(result.commands, kStraight);
  EXPECT_NEAR(result.max_error_rad, 90.0 * kDeg, 1e-9);
}

// ── the timeout, and coming back from it ───────────────────────────────────

TEST(AlignmentGate, ReleasesOnTheTimeoutAndSaysThePoseWasNotConfirmed)
{
  // No usable steering feedback: the modules never appear to arrive. A gate with
  // no timeout would immobilise the robot for ever, which is a worse failure
  // than the one it prevents.
  AlignmentGate gate{enabled_config()};
  gate.update(0.0, kDriving, kStraight, kStraight, true);
  gate.update(0.01, kDriving, kCrab, kStraight, true);

  auto result = gate.update(1.4, kDriving, kCrab, kStraight, true);
  EXPECT_EQ(result.status, kAlignSlewing) << "released before the timeout";

  result = gate.update(1.6, kDriving, kCrab, kStraight, true);
  EXPECT_EQ(result.status, kAlignTimedOut);
  EXPECT_EQ(result.commands, kDriving) << "the drive must flow again after the timeout";
  EXPECT_EQ(gate.timeout_count(), 1u);
}

TEST(AlignmentGate, RecoversFromTimedOutOnceThePoseIsActuallyConfirmed)
{
  // A one-way "pose not confirmed" flag would stay raised for the rest of the
  // session after a single feedback dropout, and a permanently raised warning is
  // a warning nobody reads.
  AlignmentGate gate{enabled_config()};
  gate.update(0.0, kDriving, kStraight, kStraight, true);
  gate.update(0.01, kDriving, kCrab, kStraight, true);
  ASSERT_EQ(gate.update(1.6, kDriving, kCrab, kStraight, true).status, kAlignTimedOut);

  const auto result = gate.update(1.7, kDriving, kCrab, kCrab, true);
  EXPECT_EQ(result.status, kAlignPassing);
  EXPECT_TRUE(result.state_changed);
}

// ── lifecycle ──────────────────────────────────────────────────────────────

TEST(AlignmentGate, ResetNeverLeavesTheGateHoldingTheDrive)
{
  // A gate must not come back from a lifecycle transition still withholding, and
  // must not come back believing the modules are where they were before.
  AlignmentGate gate{enabled_config()};
  gate.update(0.0, kDriving, kStraight, kStraight, true);
  ASSERT_EQ(gate.update(0.01, kDriving, kCrab, kStraight, true).status, kAlignSlewing);

  gate.reset();
  EXPECT_EQ(gate.status(), kAlignPassing);
  EXPECT_FALSE(gate.withholding());
  EXPECT_EQ(gate.engage_count(), 0u);

  // The forgotten target is the point: an identical command right after the
  // reset is not a jump, because there is nothing to have jumped from.
  const auto result = gate.update(0.02, kDriving, kCrab, kCrab, true);
  EXPECT_EQ(result.status, kAlignPassing);
}

TEST(AlignmentGate, ReturningFromCrabToStraightIsGuardedToo)
{
  // Releasing the arrow key is the same 90 deg transition in reverse, and the
  // teleop guard covers it (check_manoeuvres.py: "arrow released: W/S traction
  // withheld until the wheels are back straight"). It must not be one-way here.
  AlignmentGate gate{enabled_config()};
  gate.update(0.0, kDriving, kCrab, kCrab, true);

  const auto result = gate.update(0.01, kDriving, kStraight, kCrab, true);
  EXPECT_EQ(result.status, kAlignSlewing);
  EXPECT_EQ(result.commands, kStraight);
}
