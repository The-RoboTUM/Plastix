// Unit tests for the W2 command watchdog decision core (OP-18a, SR-11, §3.1.5).
//
// Every case the requirement names is here, including the ones that must NOT
// trip. A watchdog that only has positive tests is a watchdog nobody will leave
// enabled: the 2026-08-17 record of a BEST_EFFORT reader latching silence-STOP
// for 14 minutes is what a false positive looks like from the driver's seat.
//
// Time is injected, never slept on, so these run in microseconds and cannot go
// flaky on a loaded Pi.

#include <gtest/gtest.h>

#include <chrono>

#include "gripperx_hardware_interface/command_watchdog.hpp"

using gripperx_hardware_interface::TwistEchoWatchdog;
using gripperx_hardware_interface::TwistSample;
using gripperx_hardware_interface::TwistTolerance;

namespace
{
using Clock = TwistEchoWatchdog::Clock;

constexpr double kTimeout = 0.5;  // command_timeout_sec, unchanged by OP-18a.

Clock::time_point t0()
{
  // A fixed origin far from the epoch, so a subtraction bug cannot hide behind
  // a near-zero time_point.
  return Clock::time_point{} + std::chrono::seconds(1000);
}

Clock::time_point at(double seconds)
{
  return t0() + std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1e9));
}

TwistEchoWatchdog make()
{
  TwistEchoWatchdog watchdog;
  watchdog.configure(kTimeout, TwistTolerance{});
  watchdog.reset();
  return watchdog;
}

/// Drive `seconds` of normal operation: /cmd_vel at 20 Hz (teleop_mux's rate),
/// each message consumed by the controller and echoed with an advanced counter.
void run_healthy(
  TwistEchoWatchdog & watchdog, const TwistSample & twist, double from, double to,
  uint64_t & sequence)
{
  for (double t = from; t < to; t += 0.05) {
    watchdog.on_cmd_vel(twist, at(t));
    watchdog.on_echo(++sequence, at(t + 0.001));
  }
}
}  // namespace

// --- startup grace ---------------------------------------------------------

TEST(CommandWatchdog, DoesNotEnforceBeforeAnyCommand)
{
  auto watchdog = make();
  const auto verdict = watchdog.evaluate(at(60.0));
  EXPECT_FALSE(verdict.enforcing);
  EXPECT_FALSE(verdict.silence);
  EXPECT_FALSE(verdict.divergence);
}

TEST(CommandWatchdog, ReportsButDoesNotStopWhenEchoNeverArrives)
{
  auto watchdog = make();
  watchdog.on_cmd_vel(TwistSample{0.5, 0.0, 0.0}, at(0.0));
  watchdog.on_cmd_vel(TwistSample{0.25, 0.0, 0.0}, at(0.05));

  const auto verdict = watchdog.evaluate(at(0.06));
  EXPECT_TRUE(verdict.enforcing);
  EXPECT_TRUE(verdict.echo_never_seen);
  EXPECT_FALSE(verdict.silence);
  // Deliberately not a stop: the hardware component activates before the
  // controllers spawn, so this is also the normal startup window.
  EXPECT_FALSE(verdict.divergence);
}

// --- silence ---------------------------------------------------------------

TEST(CommandWatchdog, SilenceTripsWhenCmdVelGoesStale)
{
  auto watchdog = make();
  uint64_t sequence = 0;
  run_healthy(watchdog, TwistSample{0.5, 0.0, 0.0}, 0.0, 2.0, sequence);

  // Last /cmd_vel at t=1.95. Not yet stale at +0.5, stale just after.
  EXPECT_FALSE(watchdog.evaluate(at(2.44)).silence);
  const auto verdict = watchdog.evaluate(at(2.46));
  EXPECT_TRUE(verdict.silence);
  EXPECT_GT(verdict.cmd_vel_age_sec, kTimeout);
}

TEST(CommandWatchdog, SilenceClearsWhenCommandsResume)
{
  auto watchdog = make();
  uint64_t sequence = 0;
  run_healthy(watchdog, TwistSample{0.5, 0.0, 0.0}, 0.0, 1.0, sequence);
  EXPECT_TRUE(watchdog.evaluate(at(2.0)).silence);

  watchdog.on_cmd_vel(TwistSample{0.5, 0.0, 0.0}, at(2.0));
  watchdog.on_echo(++sequence, at(2.001));
  EXPECT_FALSE(watchdog.evaluate(at(2.01)).silence);
}

// --- divergence: THE check the design turns on -----------------------------

TEST(CommandWatchdog, DivergenceTripsWhenEchoFreezesAndCmdVelKeepsChanging)
{
  auto watchdog = make();
  uint64_t sequence = 0;
  run_healthy(watchdog, TwistSample{0.5, 0.0, 0.0}, 0.0, 1.0, sequence);
  ASSERT_FALSE(watchdog.evaluate(at(1.0)).divergence);

  // The controller_manager executor wedges at t=1.0: the update loop keeps
  // running, so the echo keeps arriving -- but with a FROZEN sequence, because
  // no /cmd_vel callback is being serviced any more. The operator meanwhile
  // steers, so /cmd_vel keeps arriving and keeps changing.
  const uint64_t frozen = sequence;
  double omega = 0.0;
  for (double t = 1.0; t < 1.45; t += 0.05) {
    omega += 0.05;
    watchdog.on_cmd_vel(TwistSample{0.5, 0.0, omega}, at(t));
    watchdog.on_echo(frozen, at(t + 0.001));  // echo alive, counter stuck
  }
  // Frozen for less than command_timeout_sec -> not yet a fault.
  EXPECT_FALSE(watchdog.evaluate(at(1.4)).divergence);

  for (double t = 1.45; t < 1.75; t += 0.05) {
    omega += 0.05;
    watchdog.on_cmd_vel(TwistSample{0.5, 0.0, omega}, at(t));
    watchdog.on_echo(frozen, at(t + 0.001));
  }
  const auto verdict = watchdog.evaluate(at(1.75));
  EXPECT_TRUE(verdict.divergence);
  EXPECT_FALSE(verdict.silence);  // /cmd_vel itself is perfectly healthy
  EXPECT_GT(verdict.echo_frozen_sec, kTimeout);
}

TEST(CommandWatchdog, DivergenceTripsWhenEchoStopsAltogether)
{
  // The other shape of the same fault: the update loop itself stops, so no echo
  // arrives at all. A sequence that no longer arrives is a sequence that no
  // longer advances, so this needs no separate rule.
  auto watchdog = make();
  uint64_t sequence = 0;
  run_healthy(watchdog, TwistSample{0.5, 0.0, 0.0}, 0.0, 1.0, sequence);

  for (double t = 1.0; t < 1.8; t += 0.05) {
    watchdog.on_cmd_vel(TwistSample{0.5, 0.0, t}, at(t));  // changing, no echo
  }
  const auto verdict = watchdog.evaluate(at(1.8));
  EXPECT_TRUE(verdict.divergence);
  EXPECT_FALSE(verdict.silence);
}

TEST(CommandWatchdog, DivergenceStaysLatchedWhileTheControllerStaysWedged)
{
  // Once the operator has changed the command and stopped changing it further
  // (e.g. released the key, so teleop_mux repeats zeros), the fault must NOT
  // clear itself. The change counter is snapshotted at the last consumption,
  // so "unconsumed change" survives until the controller consumes again.
  auto watchdog = make();
  uint64_t sequence = 0;
  run_healthy(watchdog, TwistSample{0.5, 0.0, 0.0}, 0.0, 1.0, sequence);
  const uint64_t frozen = sequence;

  watchdog.on_cmd_vel(TwistSample{0.0, 0.0, 0.0}, at(1.0));  // key released
  watchdog.on_echo(frozen, at(1.001));
  for (double t = 1.05; t < 4.0; t += 0.05) {
    watchdog.on_cmd_vel(TwistSample{0.0, 0.0, 0.0}, at(t));  // constant zeros
    watchdog.on_echo(frozen, at(t + 0.001));
  }
  EXPECT_TRUE(watchdog.evaluate(at(4.0)).divergence);
}

TEST(CommandWatchdog, DivergenceClearsWhenTheControllerRecovers)
{
  auto watchdog = make();
  uint64_t sequence = 0;
  run_healthy(watchdog, TwistSample{0.5, 0.0, 0.0}, 0.0, 1.0, sequence);
  const uint64_t frozen = sequence;
  for (double t = 1.0; t < 2.0; t += 0.05) {
    watchdog.on_cmd_vel(TwistSample{0.5, 0.0, t}, at(t));
    watchdog.on_echo(frozen, at(t + 0.001));
  }
  ASSERT_TRUE(watchdog.evaluate(at(2.0)).divergence);

  sequence = frozen + 1;
  watchdog.on_echo(sequence, at(2.0));
  EXPECT_FALSE(watchdog.evaluate(at(2.01)).divergence);
}

// --- false positives that must not happen ----------------------------------

TEST(CommandWatchdog, ConstantCommandHeldForSecondsIsNotAFault)
{
  // Driving straight at a fixed speed. The VALUE is frozen; the sequence is
  // not, because every republished Twist is a new consumed message. This is the
  // case option W4 was rejected for being unable to distinguish.
  auto watchdog = make();
  uint64_t sequence = 0;
  run_healthy(watchdog, TwistSample{0.5, 0.0, 0.0}, 0.0, 8.0, sequence);

  const auto verdict = watchdog.evaluate(at(8.0));
  EXPECT_TRUE(verdict.enforcing);
  EXPECT_FALSE(verdict.silence);
  EXPECT_FALSE(verdict.divergence);
  EXPECT_EQ(watchdog.cmd_vel_change_count(), 0u);
}

TEST(CommandWatchdog, Stage2IdleZerosAt20HzAreNotAFault)
{
  // OP-24 / S1 stage 2: teleop_mux publishes a default-constructed Twist at
  // 20 Hz whenever no source passes freshness, so /cmd_vel never goes silent at
  // idle. The controller consumes each one and holds the steering.
  auto watchdog = make();
  uint64_t sequence = 0;
  run_healthy(watchdog, TwistSample{0.0, 0.0, 0.0}, 0.0, 10.0, sequence);

  const auto verdict = watchdog.evaluate(at(10.0));
  EXPECT_FALSE(verdict.silence);
  EXPECT_FALSE(verdict.divergence);
}

TEST(CommandWatchdog, NormalDrivingWithAContinuouslyChangingCommandIsNotAFault)
{
  auto watchdog = make();
  uint64_t sequence = 0;
  double omega = 0.0;
  for (double t = 0.0; t < 10.0; t += 0.05) {
    omega += 0.01;
    watchdog.on_cmd_vel(TwistSample{0.5, 0.0, omega}, at(t));
    watchdog.on_echo(++sequence, at(t + 0.001));
    const auto verdict = watchdog.evaluate(at(t + 0.002));
    ASSERT_FALSE(verdict.divergence) << "false divergence at t=" << t;
    ASSERT_FALSE(verdict.silence) << "false silence at t=" << t;
  }
  EXPECT_GT(watchdog.cmd_vel_change_count(), 100u);
}

TEST(CommandWatchdog, OneCycleOfEchoLagIsNotAFault)
{
  // The echo carrying sequence N is published one control period after the
  // /cmd_vel that produced it. That lag is normal and is orders of magnitude
  // below command_timeout_sec; it must never be read as a freeze.
  auto watchdog = make();
  uint64_t sequence = 0;
  for (double t = 0.0; t < 5.0; t += 0.05) {
    watchdog.on_cmd_vel(TwistSample{0.5, 0.0, t}, at(t));
    watchdog.on_echo(++sequence, at(t + 0.03));  // one 33 Hz cycle behind
    ASSERT_FALSE(watchdog.evaluate(at(t + 0.04)).divergence) << "at t=" << t;
  }
}

// --- tolerance semantics ---------------------------------------------------

TEST(CommandWatchdog, ChangesBelowToleranceDoNotCountAsAChange)
{
  auto watchdog = make();
  uint64_t sequence = 0;
  watchdog.on_cmd_vel(TwistSample{0.5, 0.0, 0.0}, at(0.0));
  watchdog.on_echo(++sequence, at(0.001));
  watchdog.on_cmd_vel(TwistSample{0.5 + 1e-6, 0.0, 0.0}, at(0.05));
  EXPECT_EQ(watchdog.cmd_vel_change_count(), 0u);
}

TEST(CommandWatchdog, TheFinestCommandTheTreeCanEmitCountsAsAChange)
{
  // The derivation the default tolerance rests on: the smallest non-zero
  // /cmd_vel component any source in this repository emits is a manoeuvre-slew
  // twist -- crab_speed_m_s 0.25 x manoeuvre_pose_scale 0.02 = 0.005 m/s, and
  // spin_speed_rad_s 0.60 x 0.02 = 0.012 rad/s. If either of these stopped
  // registering as a change, the divergence check would go blind exactly where
  // the operator is moving most carefully.
  auto watchdog = make();
  watchdog.on_cmd_vel(TwistSample{0.0, 0.0, 0.0}, at(0.0));
  watchdog.on_cmd_vel(TwistSample{0.0, 0.005, 0.0}, at(0.05));
  EXPECT_EQ(watchdog.cmd_vel_change_count(), 1u);

  auto angular = make();
  angular.on_cmd_vel(TwistSample{0.0, 0.0, 0.0}, at(0.0));
  angular.on_cmd_vel(TwistSample{0.0, 0.0, 0.012}, at(0.05));
  EXPECT_EQ(angular.cmd_vel_change_count(), 1u);
}

TEST(CommandWatchdog, LinearAndAngularTolerancesAreIndependent)
{
  TwistEchoWatchdog watchdog;
  watchdog.configure(kTimeout, TwistTolerance{0.1, 1.0e-4});
  watchdog.reset();
  watchdog.on_cmd_vel(TwistSample{0.0, 0.0, 0.0}, at(0.0));
  watchdog.on_cmd_vel(TwistSample{0.05, 0.0, 0.0}, at(0.05));  // under linear eps
  EXPECT_EQ(watchdog.cmd_vel_change_count(), 0u);
  watchdog.on_cmd_vel(TwistSample{0.05, 0.0, 0.01}, at(0.10));  // over angular eps
  EXPECT_EQ(watchdog.cmd_vel_change_count(), 1u);
}
