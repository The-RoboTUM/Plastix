// Unit checks for the whole-drive fault (user decision 2026-10-16): a wheel driven
// without a LIVE encoder, or a persistent stall, stops all four wheels, latched
// until the process restarts.
//
// THE THRESHOLDS COME FROM THE ACTIVE CONFIG (active_config.hpp reads the
// installed ros2_controllers.yaml), and so do HWR-30a's -- the rig below chains
// StallDetector and DriveFaultMonitor in the same order and with the same
// arguments as SwerveController::write_wheel_commands stages 3 and 4. Command
// levels and durations are DERIVED from those thresholds, so the scenarios keep
// meaning what they say when a TO-VERIFY value is changed.
//
// WHAT THESE TESTS CANNOT SHOW: that the thresholds are RIGHT (all TO-VERIFY),
// and anything about rclcpp/lifecycle wiring -- that is
// test_swerve_controller_drive_fault.cpp.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "active_config.hpp"
#include "gripperx_swerve_controller/drive_fault_monitor.hpp"
#include "gripperx_swerve_controller/stall_detector.hpp"
#include "gripperx_swerve_controller/wheel_regulator.hpp"

using gripperx_swerve_controller::DriveFaultClearRefusal;
using gripperx_swerve_controller::DriveFaultConfig;
using gripperx_swerve_controller::drive_fault_reason_clearable;
using gripperx_swerve_controller::DriveFaultInput;
using gripperx_swerve_controller::DriveFaultMonitor;
using gripperx_swerve_controller::DriveFaultRecord;
using gripperx_swerve_controller::kDriveFaultEncoderMissing;
using gripperx_swerve_controller::kDriveFaultFeedbackLost;
using gripperx_swerve_controller::kDriveFaultNone;
using gripperx_swerve_controller::parse_drive_fault;
using gripperx_swerve_controller::serialize_drive_fault;
using gripperx_swerve_controller::kDriveFaultStallLatchedHeld;
using gripperx_swerve_controller::kDriveFaultStallMultiWheel;
using gripperx_swerve_controller::kDriveFaultStallRetrip;
using gripperx_swerve_controller::kDriveFaultStallTripCount;
using gripperx_swerve_controller::kNumWheels;
using gripperx_swerve_controller::kStallProvenanceInitFailed;
using gripperx_swerve_controller::kStallProvenanceLive;
using gripperx_swerve_controller::kStallProvenanceLiveUnconfirmed;
using gripperx_swerve_controller::kStallProvenanceNoEncoder;
using gripperx_swerve_controller::kStallProvenanceUnknown;
using gripperx_swerve_controller::StallDetector;
using gripperx_swerve_controller::StallDetectorConfig;
using gripperx_swerve_controller::WheelRegulator;
using gripperx_swerve_controller::WheelRegulatorConfig;
using gripperx_swerve_controller::WheelRegulatorInput;

namespace
{
constexpr double kDt = 1.0 / 30.0;  // controller_manager update_rate
constexpr std::size_t kBR = 3;

using Wheels = std::array<double, kNumWheels>;
using Prov = std::array<int, kNumWheels>;

const Prov kAllLive{kStallProvenanceLive, kStallProvenanceLive, kStallProvenanceLive,
  kStallProvenanceLive};
// 2026-10-13: three encoders counting, BR registered but never counted.
const Prov kOct13{kStallProvenanceLive, kStallProvenanceLive, kStallProvenanceLive,
  kStallProvenanceLiveUnconfirmed};
const Prov kAllUnconfirmed{kStallProvenanceLiveUnconfirmed, kStallProvenanceLiveUnconfirmed,
  kStallProvenanceLiveUnconfirmed, kStallProvenanceLiveUnconfirmed};

Wheels uniform(double v) { return {v, v, v, v}; }

/// write_wheel_commands stages 3 + 4, verbatim in order and arguments.
struct Rig
{
  StallDetectorConfig stall_cfg;
  DriveFaultConfig fault_cfg;
  /// Not part of the chain below; only its slow-end floor defines slow_command().
  WheelRegulatorConfig regulator_cfg{active_config::regulator_config()};
  StallDetector stall;
  DriveFaultMonitor fault;
  double now{100.0};
  Wheels position{};
  Prov provenance{kAllLive};
  bool provenance_received{true};
  Wheels last_out{};
  bool seen_trip_edge{false};
  int stall_trips_seen{0};
  int zero_count_trips_seen{0};
  int releases_seen{0};

  Rig()
  : stall_cfg(active_config::stall_config()), fault_cfg(active_config::drive_fault_config())
  {
    // The rig models the REAL robot unless a test says otherwise.
    stall_cfg.assume_live_provenance = false;
    fault_cfg.assume_live_until_provenance_received = false;
    stall.configure(stall_cfg);
    fault.configure(fault_cfg);
  }

  void step(const Wheels & cmd, const Wheels & rate, bool drive_withheld = false)
  {
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      position[i] += rate[i] * kDt;
    }
    cycle(cmd, drive_withheld);
  }

  /// One cycle with the wheel POSITIONS set explicitly (twitches, counts).
  void step_to(const Wheels & cmd, const Wheels & new_position, bool drive_withheld = false)
  {
    position = new_position;
    cycle(cmd, drive_withheld);
  }

  void cycle(const Wheels & cmd, bool drive_withheld)
  {
    now += kDt;
    const bool fault_before = fault.latched();
    Wheels effective = cmd;
    if (drive_withheld) {
      effective = uniform(0.0);
    }
    stall.set_provenance(provenance);
    const auto s = stall.update(
      now, cmd, position, {true, true, true, true}, drive_withheld || fault_before);
    std::array<bool, kNumWheels> latched{};
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      latched[i] = stall.latched(i);
      if (latched[i]) {
        effective[i] = 0.0;
      }
      stall_trips_seen += s.events[i].tripped ? 1 : 0;
      zero_count_trips_seen += (s.events[i].tripped && s.events[i].zero_count) ? 1 : 0;
      releases_seen += s.events[i].released ? 1 : 0;
    }
    DriveFaultInput in;
    in.now_sec = now;
    in.requested = cmd;
    in.provenance = provenance;
    in.provenance_received = provenance_received;
    in.drive_withheld = drive_withheld;
    in.stall_latched = latched;
    in.stall_events = s.events;
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      in.stall_no_motion_sec[i] = stall.no_motion_sec(i, now);
      in.stall_silent_sec[i] = stall.silent_sec(i);
      in.stall_latched_zero_count[i] = stall.latched_by_zero_count(i);
    }
    const auto r = fault.update(in, effective);
    last_out = r.commands;
    seen_trip_edge = seen_trip_edge || r.tripped;
  }

  void run(double seconds, const Wheels & cmd, const Wheels & rate, bool withheld = false)
  {
    const int cycles = static_cast<int>(std::ceil(seconds / kDt));
    for (int c = 0; c < cycles; ++c) {
      step(cmd, rate, withheld);
    }
  }

  bool all_zero() const
  {
    return std::all_of(last_out.begin(), last_out.end(), [](double v) { return v == 0.0; });
  }

  /// Operator clear as SwerveController::apply_pending_drive_fault_clear
  /// applies it (monitor + tier-1 latches; the marker and FR-14 are the
  /// controller's and are covered by the bench tests).
  bool clear(int expected_reason = kDriveFaultNone)
  {
    const bool applied = fault.clear(now, expected_reason);
    if (applied) {
      stall.release_for_operator_clear();
    }
    return applied;
  }
};

/// A command in the SLOW BAND: above encoder_confirm_min_command_rad_s (where
/// the zero-count criterion watches LIVE wheels), at or below
/// stall_min_command_rad_s (where the net-displacement criterion does not), and
/// so also below the regulator floor. A quarter of the way up the band -- its
/// low end, which is what the 2026-10-22 decision is about.
double slow_command(const Rig & rig)
{
  const double lo = rig.stall_cfg.silent_min_command_rad_s;
  return lo + 0.25 * (rig.stall_cfg.min_command_rad_s - lo);
}

/// A command well inside HWR-30a's armed band.
double fast_command(const Rig & rig) { return 2.0 * rig.stall_cfg.min_command_rad_s + 1.0; }

/// One encoder count, as the active config states it.
double count(const Rig & rig) { return rig.stall_cfg.count_rad; }

Wheels plus_all(const Wheels & p, double delta)
{
  Wheels out = p;
  for (auto & x : out) {
    x += delta;
  }
  return out;
}
}  // namespace

// ------------------------------------------------------------ active config
TEST(DriveFaultActiveConfig, IsEnabledValidAndLeavesNoBandWhereNeitherTriggerWatches)
{
  const auto fault_cfg = active_config::drive_fault_config();
  const auto stall_cfg = active_config::stall_config();
  std::string error;
  EXPECT_TRUE(DriveFaultMonitor::validate(fault_cfg, error)) << error;
  EXPECT_TRUE(StallDetector::validate(stall_cfg, error)) << error;
  EXPECT_TRUE(fault_cfg.enabled);
  EXPECT_TRUE(stall_cfg.enabled);
  // ONE number for "a healthy wheel commanded above this counts": the encoder
  // trigger (non-LIVE wheels) and the zero-count criterion (LIVE wheels).
  EXPECT_EQ(stall_cfg.silent_min_command_rad_s, fault_cfg.encoder_confirm_min_command_rad_s);
  // The displacement criterion does not judge below it.
  EXPECT_LE(fault_cfg.encoder_confirm_min_command_rad_s, stall_cfg.min_command_rad_s);
  // OP-25: the release band stays below both rise edges.
  EXPECT_LT(stall_cfg.release_command_rad_s, stall_cfg.silent_min_command_rad_s);
  EXPECT_LT(stall_cfg.release_command_rad_s, stall_cfg.min_command_rad_s);
  // The monitor classifies with the detector's windows.
  EXPECT_EQ(fault_cfg.stall_window_sec, stall_cfg.window_sec);
  EXPECT_EQ(fault_cfg.stall_silent_window_sec, stall_cfg.silent_window_sec);
  // The real robot must not assume provenance (the sim overlay may).
  EXPECT_FALSE(fault_cfg.assume_live_until_provenance_received);
}

TEST(DriveFaultActiveConfig, TheRegulatorOnlyAddsAuthorityWhereHwr30aWatches)
{
  const auto regulator_cfg = active_config::regulator_config();
  const auto stall_cfg = active_config::stall_config();
  std::string error;
  EXPECT_TRUE(WheelRegulator::validate(regulator_cfg, error)) << error;
  EXPECT_TRUE(WheelRegulator::validate_floor(regulator_cfg, stall_cfg, error)) << error;
  EXPECT_GE(regulator_cfg.floor_rad_s, stall_cfg.min_command_rad_s);
}

TEST(DriveFaultActiveConfig, HeaderDefaultsAreInStepWithTheYaml)
{
  // The struct defaults are what a caller gets when a key is missing; they must
  // not quietly describe a different machine from the installed YAML.
  const auto fault_cfg = active_config::drive_fault_config();
  const auto stall_cfg = active_config::stall_config();
  const auto regulator_cfg = active_config::regulator_config();
  const StallDetectorConfig stall_default;
  const DriveFaultConfig fault_default;
  EXPECT_EQ(stall_default.min_command_rad_s, stall_cfg.min_command_rad_s);
  EXPECT_EQ(stall_default.silent_min_command_rad_s, stall_cfg.silent_min_command_rad_s);
  EXPECT_EQ(stall_default.silent_window_sec, stall_cfg.silent_window_sec);
  EXPECT_NEAR(stall_default.count_rad, stall_cfg.count_rad, 1e-12);
  EXPECT_EQ(
    fault_default.encoder_confirm_min_command_rad_s, fault_cfg.encoder_confirm_min_command_rad_s);
  EXPECT_EQ(fault_default.stall_silent_window_sec, fault_cfg.stall_silent_window_sec);
  EXPECT_EQ(WheelRegulatorConfig{}.floor_rad_s, regulator_cfg.floor_rad_s);
}

TEST(DriveFaultActiveConfig, PersistentStallIsTheUsersThreeCriteria)
{
  // User decision 2026-10-16: multi-wheel, re-trip within N s, latched-and-held
  // longer than T. NOT "N trips".
  const auto fault_cfg = active_config::drive_fault_config();
  const auto stall_cfg = active_config::stall_config();
  EXPECT_TRUE(fault_cfg.stall_escalate_on_multi_wheel);
  EXPECT_GT(fault_cfg.stall_escalation_window_sec, 0.0);
  EXPECT_GT(fault_cfg.stall_escalation_latched_sec, 0.0);
  EXPECT_EQ(fault_cfg.stall_escalation_trips, 0u);
  // "Held" is HWR-30a's release band, so held and not-yet-released coincide.
  EXPECT_EQ(fault_cfg.stall_hold_min_command_rad_s, stall_cfg.release_command_rad_s);
}

// ------------------------------------------------- trigger 1: encoder missing
TEST(DriveFaultEncoder, TheOct13ShapeStopsAllFourWheels)
{
  Rig rig;
  rig.provenance = kOct13;
  const double v = fast_command(rig);
  // Three wheels count, BR's position never moves.
  const Wheels rate{v, v, v, 0.0};
  // Just short of the window: still driving on all four.
  rig.run(rig.fault_cfg.encoder_confirm_window_sec - 3 * kDt, uniform(v), rate);
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out[kBR], v);
  // Through the window.
  rig.run(6 * kDt, uniform(v), rate);
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_TRUE(rig.seen_trip_edge);
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultEncoderMissing);
  EXPECT_EQ(rig.fault.record().wheel, static_cast<int>(kBR));
  EXPECT_EQ(rig.fault.record().provenance, kStallProvenanceLiveUnconfirmed);
  EXPECT_GE(rig.fault.record().accumulated_sec, rig.fault_cfg.encoder_confirm_window_sec);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(rig.last_out[i], 0.0) << "wheel " << i;
    EXPECT_FALSE(std::signbit(rig.last_out[i])) << "wheel " << i << " is -0.0";
  }
  // HWR-30a never saw it: BR was never armed.
  EXPECT_EQ(rig.stall_trips_seen, 0);
}

TEST(DriveFaultEncoder, LiveUnconfirmedAtStandstillNeverTrips)
{
  Rig rig;
  rig.provenance = kAllUnconfirmed;
  rig.run(60.0, uniform(0.0), uniform(0.0));
  EXPECT_FALSE(rig.fault.latched());
  // Nor at exactly the threshold (strict >), however long.
  rig.run(60.0, uniform(rig.fault_cfg.encoder_confirm_min_command_rad_s), uniform(0.0));
  EXPECT_FALSE(rig.fault.latched());
}

TEST(DriveFaultEncoder, SlowDrivingIsCoveredAndTheNonLiveWheelIsTrigger1s)
{
  Rig rig;
  rig.provenance = kOct13;
  const double v = slow_command(rig);
  ASSERT_GT(v, rig.fault_cfg.encoder_confirm_min_command_rad_s);
  ASSERT_LE(v, rig.stall_cfg.min_command_rad_s);
  rig.run(0.5 * rig.fault_cfg.encoder_confirm_window_sec, uniform(v), {v, v, v, 0.0});
  ASSERT_FALSE(rig.fault.latched());
  // User decision 2026-10-22, point 1: a stall needs a LIVE encoder. The three
  // LIVE wheels are watched by HWR-30a's zero-count criterion at this speed
  // (the displacement criterion does not judge here); BR, never confirmed, is
  // not -- it is trigger 1's.
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_TRUE(rig.stall.silent_armed(i)) << "wheel " << i;
    EXPECT_FALSE(rig.stall.armed(i)) << "wheel " << i;
  }
  EXPECT_FALSE(rig.stall.silent_armed(kBR));
  rig.run(0.5 * rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), {v, v, v, 0.0});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultEncoderMissing);
  EXPECT_EQ(rig.fault.record().wheel, static_cast<int>(kBR));
  EXPECT_TRUE(rig.all_zero());
  EXPECT_EQ(rig.stall_trips_seen, 0);
}

TEST(DriveFaultEncoder, EveryNonLiveCodeTrips)
{
  for (const int code : {kStallProvenanceLiveUnconfirmed, kStallProvenanceNoEncoder,
      kStallProvenanceInitFailed, kStallProvenanceUnknown})
  {
    Rig rig;
    rig.provenance = kAllLive;
    rig.provenance[1] = code;
    const double v = slow_command(rig);
    rig.run(rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), uniform(v));
    ASSERT_TRUE(rig.fault.latched()) << "code " << code;
    EXPECT_EQ(rig.fault.record().wheel, 1);
    EXPECT_EQ(rig.fault.record().provenance, code);
  }
}

TEST(DriveFaultEncoder, TimeAccumulatesAcrossStopsAndOnlyLiveResetsIt)
{
  Rig rig;
  rig.provenance = kOct13;
  const double v = slow_command(rig);
  const double w = rig.fault_cfg.encoder_confirm_window_sec;
  // The three LIVE wheels turn; only BR's encoder is silent.
  const Wheels br_silent{v, v, v, 0.0};
  rig.run(0.6 * w, uniform(v), br_silent);
  rig.run(30.0, uniform(0.0), uniform(0.0));  // a long stop: pauses, does not reset
  EXPECT_FALSE(rig.fault.latched());
  rig.run(0.6 * w, uniform(v), br_silent);
  EXPECT_TRUE(rig.fault.latched()) << "accumulated " << rig.fault.encoder_unconfirmed_sec(kBR);

  Rig live;
  live.provenance = kOct13;
  live.run(0.8 * w, uniform(v), br_silent);
  live.provenance = kAllLive;  // BR counted once
  live.step(uniform(v), uniform(v));
  EXPECT_EQ(live.fault.encoder_unconfirmed_sec(kBR), 0.0);
  live.provenance = kOct13;  // e.g. an ESP32 reboot: judged afresh
  live.run(0.8 * w, uniform(v), br_silent);
  EXPECT_FALSE(live.fault.latched());
}

TEST(DriveFaultEncoder, AnAlignmentHoldDoesNotAccumulate)
{
  Rig rig;
  rig.provenance = kOct13;
  const double v = fast_command(rig);
  rig.run(10.0 * rig.fault_cfg.encoder_confirm_window_sec, uniform(v), uniform(0.0), true);
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.fault.encoder_unconfirmed_sec(kBR), 0.0);
}

// ------------------------------------------------- trigger 2: stall escalation
TEST(DriveFaultStall, ATemporaryStallStaysATier1SingleWheelLatch)
{
  Rig rig;
  ASSERT_GT(rig.fault_cfg.stall_escalation_latched_sec, 0.5)
    << "this scenario holds the latch under command for ~0.5 s";
  const double v = fast_command(rig);
  // FR blocked for longer than the stall window, the others turn.
  rig.run(rig.stall_cfg.window_sec + 0.5, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out[1], 0.0);
  EXPECT_EQ(rig.last_out[0], v);
  EXPECT_EQ(rig.last_out[2], v);
  EXPECT_EQ(rig.last_out[3], v);
  // Operator lets go, waits past the escalation window, drives again: the
  // obstruction is gone and the wheel turns.
  rig.run(rig.fault_cfg.stall_escalation_window_sec + 1.0, uniform(0.0), uniform(0.0));
  rig.run(3.0, uniform(v), uniform(v));
  EXPECT_FALSE(rig.stall.latched(1));
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out, uniform(v));
}

TEST(DriveFaultStall, AReTripInsideTheEscalationWindowStopsTheDrive)
{
  Rig rig;
  ASSERT_GT(rig.fault_cfg.stall_escalation_window_sec, rig.stall_cfg.window_sec + 0.5)
    << "a re-trip needs at least one stall window after the release";
  const double v = fast_command(rig);
  rig.run(rig.stall_cfg.window_sec + 0.2, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  rig.run(0.2, uniform(0.0), uniform(0.0));                 // release edge 1
  rig.run(rig.stall_cfg.window_sec + 0.3, uniform(v), {v, 0.0, v, v});  // still blocked
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallRetrip);
  EXPECT_EQ(rig.fault.record().wheel, 1);
  EXPECT_LE(rig.fault.record().since_release_sec, rig.fault_cfg.stall_escalation_window_sec);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultStall, ReachingTheTripCountStopsTheDriveWhenEnabled)
{
  // OFF in the active config (not the user's criterion); the mechanism is
  // still checked with it switched on locally.
  Rig rig;
  rig.fault_cfg.stall_escalation_trips = 3;
  rig.fault.configure(rig.fault_cfg);
  const double v = fast_command(rig);
  const double rest = rig.fault_cfg.stall_escalation_window_sec + 1.0;  // never a re-trip
  for (uint32_t trip = 1; trip <= rig.fault_cfg.stall_escalation_trips; ++trip) {
    EXPECT_FALSE(rig.fault.latched()) << "before trip " << trip;
    rig.run(0.2, uniform(0.0), uniform(0.0));
    // Turn freely for longer than the escalation window after the release ...
    rig.run(rest, uniform(v), uniform(v));
    // ... then block.
    rig.run(rig.stall_cfg.window_sec + 0.2, uniform(v), {v, v, 0.0, v});
  }
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallTripCount);
  EXPECT_EQ(rig.fault.record().wheel, 2);
  EXPECT_EQ(rig.fault.record().stall_trip_count, rig.fault_cfg.stall_escalation_trips);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultStall, TheTripCountDoesNotEscalateWithTheActiveConfig)
{
  Rig rig;
  ASSERT_EQ(rig.fault_cfg.stall_escalation_trips, 0u);
  const double v = fast_command(rig);
  const double rest = rig.fault_cfg.stall_escalation_window_sec + 1.0;
  for (int trip = 1; trip <= 6; ++trip) {
    rig.run(0.2, uniform(0.0), uniform(0.0));
    rig.run(rest, uniform(v), uniform(v));
    rig.run(rig.stall_cfg.window_sec + 0.2, uniform(v), {v, v, 0.0, v});
  }
  EXPECT_EQ(rig.fault.stall_trips(2), 6u);
  EXPECT_FALSE(rig.fault.latched());
}

TEST(DriveFaultStall, ALatchHeldUnderCommandStopsTheDrive)
{
  // OP-25 deadlock: the command never drops, so the latch can never clear.
  Rig rig;
  const double v = fast_command(rig);
  const double t = rig.fault_cfg.stall_escalation_latched_sec;
  rig.run(rig.stall_cfg.window_sec + 0.1, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  rig.run(t - 0.3, uniform(v), {v, 0.0, v, v});
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out[0], v) << "still a tier-1 single-wheel response";
  rig.run(0.6, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallLatchedHeld);
  EXPECT_EQ(rig.fault.record().wheel, 1);
  EXPECT_GE(rig.fault.record().latched_held_sec, t);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultStall, ALatchWhoseCommandIsLetGoDoesNotEscalate)
{
  // The command drops into the release band before T: that is the operator
  // letting go, not a deadlock -- and the held time restarts from zero.
  Rig rig;
  const double v = fast_command(rig);
  const double t = rig.fault_cfg.stall_escalation_latched_sec;
  const double small = 0.5 * rig.fault_cfg.stall_hold_min_command_rad_s;
  rig.run(rig.stall_cfg.window_sec + 0.1, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  rig.run(0.7 * t, uniform(v), {v, 0.0, v, v});
  rig.run(3.0 * t, uniform(small), uniform(0.0));  // let go, latched but not held
  EXPECT_TRUE(rig.stall.latched(1));
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.fault.latched_held_sec(1), 0.0);
}

TEST(DriveFaultStall, ASecondStallingWheelStopsTheDrive)
{
  // Sequential: FR stalls and is latched, later BR stalls too.
  Rig rig;
  ASSERT_TRUE(rig.fault_cfg.stall_escalate_on_multi_wheel);
  const double v = fast_command(rig);
  rig.run(rig.stall_cfg.window_sec + 0.3, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  ASSERT_FALSE(rig.fault.latched());
  rig.run(rig.stall_cfg.window_sec + 0.3, uniform(v), {v, 0.0, v, 0.0});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallMultiWheel);
  EXPECT_EQ(rig.fault.record().wheel, 3);
  EXPECT_EQ(rig.fault.record().other_wheel, 1);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultStall, TwoWheelsStallingTogetherWhileOthersTurnIsStillMultiWheel)
{
  Rig rig;
  const double v = fast_command(rig);
  rig.run(rig.stall_cfg.window_sec + 0.5, uniform(v), {v, 0.0, v, 0.0});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallMultiWheel);
}

// ------------------------------------------------- FEEDBACK_LOST (2026-10-16)
TEST(DriveFaultFeedback, EveryPositionFreezingTogetherIsFeedbackLost)
{
  // A frozen /hw/joint_states: every position stops on the same cycle.
  Rig rig;
  const double v = fast_command(rig);
  rig.run(0.5, uniform(v), uniform(v));
  rig.run(rig.stall_cfg.window_sec + 0.5, uniform(v), uniform(0.0));
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultFeedbackLost);
  EXPECT_EQ(rig.fault.record().wheel, -1) << "must not blame a single wheel";
  EXPECT_GE(rig.fault.record().other_wheel, 0);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultFeedback, FreezesAFewCyclesApartAreStillFeedbackLost)
{
  // The dwell windows of different wheels restart at different sub-cycle
  // moments, so a common freeze can complete their criteria a cycle or two
  // apart. Modelled by freezing FL two cycles before the rest.
  Rig rig;
  const double v = fast_command(rig);
  rig.run(0.5, uniform(v), uniform(v));
  rig.step(uniform(v), {0.0, v, v, v});
  rig.step(uniform(v), {0.0, v, v, v});
  rig.run(rig.stall_cfg.window_sec + 0.5, uniform(v), uniform(0.0));
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultFeedbackLost);
}

TEST(DriveFaultFeedback, NoWheelLiveWhileDrivingIsFeedbackLost)
{
  // What GripperXInterface publishes for a stale or absent /hw/joint_states:
  // UNKNOWN on all four -- and all four LIVE_UNCONFIRMED (no wheel counting).
  for (const int code : {kStallProvenanceUnknown, kStallProvenanceLiveUnconfirmed}) {
    Rig rig;
    rig.provenance = {code, code, code, code};
    const double v = slow_command(rig);
    rig.run(rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), uniform(v));
    ASSERT_TRUE(rig.fault.latched()) << code;
    EXPECT_EQ(rig.fault.record().reason, kDriveFaultFeedbackLost) << code;
    EXPECT_EQ(rig.fault.record().wheel, -1) << code;
  }
}

// ------------------------------------------------------------ accumulator (F5)
TEST(DriveFaultEncoder, ACommandFlickeringAcrossTheThresholdStillTrips)
{
  Rig rig;
  rig.provenance = kOct13;
  const double v = fast_command(rig);
  const int cycles = static_cast<int>(4.0 * rig.fault_cfg.encoder_confirm_window_sec / kDt);
  for (int c = 0; c < cycles && !rig.fault.latched(); ++c) {
    rig.step(uniform(c % 2 == 0 ? v : 0.0), {v, v, v, 0.0});
  }
  EXPECT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultEncoderMissing);
}

// ------------------------------------------- the slow band: zero-count criterion
// User decision 2026-10-22, option (c). In the band a LIVE wheel trips only if
// its encoder shows NO count change at all for an accumulated
// stall_silent_window_sec. These run at slow_command().
TEST(DriveFaultSlowBand, ASilentLiveWheelTripsAfterTheWindowNotBefore)
{
  Rig rig;
  const double v = slow_command(rig);
  const double w = rig.stall_cfg.silent_window_sec;
  ASSERT_GT(rig.fault_cfg.stall_escalation_latched_sec, 0.5);
  // Rolling on all four; then FR's encoder goes silent while it is driven.
  rig.run(0.5, uniform(v), uniform(v));
  EXPECT_TRUE(rig.stall.silent_armed(1));
  rig.run(w - 3 * kDt, uniform(v), {v, 0.0, v, v});
  EXPECT_FALSE(rig.stall.latched(1)) << "silent " << rig.stall.silent_sec(1) << " s";
  EXPECT_EQ(rig.last_out[1], v);
  rig.run(6 * kDt, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  EXPECT_TRUE(rig.stall.latched_by_zero_count(1));
  EXPECT_EQ(rig.zero_count_trips_seen, 1);
  EXPECT_FALSE(rig.fault.latched()) << "one wheel, once: tier 1 only";
  EXPECT_EQ(rig.last_out[1], 0.0);
  EXPECT_EQ(rig.last_out[0], v);
  EXPECT_EQ(rig.last_out[2], v);
  EXPECT_EQ(rig.last_out[3], v);
}

TEST(DriveFaultSlowBand, HeldThereItEscalatesToTheWholeDrive)
{
  // The latched-and-held criterion (OP-25 deadlock) after a zero-count latch.
  Rig rig;
  const double v = slow_command(rig);
  const double t = rig.fault_cfg.stall_escalation_latched_sec;
  rig.run(0.5, uniform(v), uniform(v));
  rig.run(rig.stall_cfg.silent_window_sec + 0.1, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  ASSERT_FALSE(rig.fault.latched());
  rig.run(t - 0.3, uniform(v), {v, 0.0, v, v});
  EXPECT_FALSE(rig.fault.latched());
  rig.run(0.6, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallLatchedHeld);
  EXPECT_EQ(rig.fault.record().wheel, 1);
  EXPECT_TRUE(rig.fault.record().zero_count) << "the record must name the zero-count criterion";
  EXPECT_GE(rig.fault.record().latched_held_sec, t);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultSlowBand, OneCountPerWindowNeverTrips)
{
  // Wheels that barely twitch -- ONE count, then nothing for most of a window,
  // again and again -- are not silent. Net displacement grows by one count per
  // period, far below anything the displacement criterion knows. All four.
  Rig rig;
  const double v = slow_command(rig);
  const int period = static_cast<int>(0.9 * rig.stall_cfg.silent_window_sec / kDt);
  Wheels p = rig.position;
  for (int c = 0; c < 30 * period; ++c) {
    if (c % period == 0) {
      p = plus_all(p, count(rig));
    }
    rig.step_to(uniform(v), p);
  }
  EXPECT_EQ(rig.stall_trips_seen, 0);
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_TRUE(rig.stall.silent_armed(2)) << "watched, not asleep";
}

TEST(DriveFaultSlowBand, AForwardAndBackTwitchWithZeroNetNeverTrips)
{
  // Count change, not net displacement: out one count and back on the next
  // sample, once per ~0.9 window, on all four. Net movement is exactly zero.
  Rig rig;
  const double v = slow_command(rig);
  const int period = static_cast<int>(0.9 * rig.stall_cfg.silent_window_sec / kDt);
  const Wheels rest = rig.position;
  for (int c = 0; c < 30 * period; ++c) {
    const bool out = (c % period == 0);
    rig.step_to(uniform(v), out ? plus_all(rest, count(rig)) : rest);
  }
  EXPECT_EQ(rig.position, rest);
  EXPECT_EQ(rig.stall_trips_seen, 0);
  EXPECT_FALSE(rig.fault.latched());
}

TEST(DriveFaultSlowBand, LessThanHalfACountIsNotACount)
{
  // The quantisation guard: a change of under half a count (float noise, not an
  // encoder edge) does not reset the accumulator. BR jitters by 0.4 count; the
  // other three roll normally.
  Rig rig;
  const double v = slow_command(rig);
  const int cycles = static_cast<int>(std::ceil((rig.stall_cfg.silent_window_sec + 0.3) / kDt));
  Wheels p = rig.position;
  const double br_rest = p[kBR];
  for (int c = 0; c < cycles && !rig.stall.latched(kBR); ++c) {
    p = plus_all(p, v * kDt);
    p[kBR] = br_rest + ((c % 2 == 0) ? 0.4 * count(rig) : 0.0);
    rig.step_to(uniform(v), p);
  }
  EXPECT_TRUE(rig.stall.latched(kBR));
  EXPECT_TRUE(rig.stall.latched_by_zero_count(kBR));
}

TEST(DriveFaultSlowBand, AHealthySlowWheelIsWatchedAndNeverTrips)
{
  // Two healthy rates: tracking the command, and a crawl at a QUARTER of the
  // displacement criterion's rate (0.05 rad would take four windows) that still
  // produces counts.
  Rig probe;
  const double v = slow_command(probe);
  const double crawl = 0.25 * probe.stall_cfg.min_position_delta_rad / probe.stall_cfg.window_sec;
  ASSERT_GT(crawl * probe.stall_cfg.silent_window_sec, probe.stall_cfg.count_rad);
  for (const double rate : {v, crawl}) {
    Rig rig;
    rig.run(60.0, uniform(v), uniform(rate));
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_TRUE(rig.stall.silent_armed(i)) << "rate " << rate << " wheel " << i;
      EXPECT_FALSE(rig.stall.latched(i)) << "rate " << rate << " wheel " << i;
    }
    EXPECT_EQ(rig.stall_trips_seen, 0) << "rate " << rate;
    EXPECT_FALSE(rig.fault.latched()) << "rate " << rate;
    EXPECT_EQ(rig.last_out, uniform(v)) << "rate " << rate;
  }
}

TEST(DriveFaultSlowBand, TheRegulatorStaysOffBelowItsFloorThere)
{
  // The regulator floor did not move: a slow wheel well short of its command
  // gets NO correction in the band.
  Rig rig;
  const double v = slow_command(rig);
  ASSERT_LT(v, rig.regulator_cfg.floor_rad_s);
  WheelRegulatorConfig config = rig.regulator_cfg;
  config.enabled = true;
  config.assume_live_provenance = false;
  WheelRegulator regulator(config);
  Wheels position{};
  const Wheels measured = uniform(0.5 * v);
  for (int c = 0; c < 90; ++c) {
    WheelRegulatorInput input;
    input.now_sec = 100.0 + c * kDt;
    input.setpoint = uniform(v);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      position[i] += measured[i] * kDt;
    }
    input.measured = measured;
    input.measured_valid = {true, true, true, true};
    input.position = position;
    input.position_valid = {true, true, true, true};
    input.provenance = kAllLive;
    const auto result = regulator.update(input);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_EQ(result.status[i], gripperx_swerve_controller::kRegulatorOffBelowFloor)
        << "cycle " << c << " wheel " << i;
      EXPECT_EQ(result.correction[i], 0.0) << "cycle " << c << " wheel " << i;
      EXPECT_EQ(result.commands[i], v) << "cycle " << c << " wheel " << i;
      EXPECT_EQ(regulator.integrator(i), 0.0) << "cycle " << c << " wheel " << i;
    }
  }
}

TEST(DriveFaultSlowBand, AnAlignmentHoldNeitherAccumulatesNorResets)
{
  // While the gate withholds the drive the wheel cannot turn: obedience, not a
  // silent encoder. The accumulator PAUSES -- it neither grows nor restarts.
  Rig rig;
  const double v = slow_command(rig);
  const double w = rig.stall_cfg.silent_window_sec;
  rig.run(0.6 * w, uniform(v), {v, 0.0, v, v});
  const double before = rig.stall.silent_sec(1);
  ASSERT_GT(before, 0.5 * w);
  rig.run(10.0 * w, uniform(v), uniform(0.0), true);
  EXPECT_FALSE(rig.stall.silent_armed(1));
  EXPECT_EQ(rig.stall.silent_sec(1), before) << "paused, not reset, not grown";
  EXPECT_FALSE(rig.stall.latched(1));
  EXPECT_FALSE(rig.fault.latched());
  // Released, still silent: it trips once the REST of the window has passed.
  rig.run(0.3 * w, uniform(v), {v, 0.0, v, v});
  EXPECT_FALSE(rig.stall.latched(1));
  rig.run(0.2 * w, uniform(v), {v, 0.0, v, v});
  EXPECT_TRUE(rig.stall.latched_by_zero_count(1));
}

TEST(DriveFaultSlowBand, AZeroCountLatchReleasesAtSlowDriving)
{
  // OP-25 release edge of a zero-count latch: fall into the release band, then
  // rise above encoder_confirm_min -- NOT above stall_min_command_rad_s, which
  // slow driving would never reach.
  Rig rig;
  const double v = slow_command(rig);
  ASSERT_LT(v, rig.stall_cfg.min_command_rad_s);
  rig.run(rig.stall_cfg.silent_window_sec + 0.2, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched_by_zero_count(1));
  // A held command never releases it.
  rig.run(1.0, uniform(v), {v, 0.0, v, v});
  EXPECT_TRUE(rig.stall.latched(1));
  // Let go, then a fresh slow command: released, and the wheel turns again.
  rig.run(0.3, uniform(0.0), uniform(0.0));
  EXPECT_TRUE(rig.stall.latched(1));
  rig.step(uniform(v), uniform(v));
  EXPECT_FALSE(rig.stall.latched(1));
  EXPECT_EQ(rig.releases_seen, 1);
  rig.run(3.0, uniform(v), uniform(v));
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out, uniform(v));
}

TEST(DriveFaultSlowBand, ADisplacementLatchStillNeedsTheRiseAboveStallMin)
{
  // OP-25 option (a), user decision 2026-10-29: the rise edge is the arming
  // threshold of the criterion that set the latch. A slow command after letting
  // go does NOT release a displacement latch; a fast one does.
  Rig rig;
  const double fast = fast_command(rig);
  const double v = slow_command(rig);
  rig.run(rig.stall_cfg.window_sec + 0.2, uniform(fast), {fast, 0.0, fast, fast});
  ASSERT_TRUE(rig.stall.latched(1));
  ASSERT_FALSE(rig.stall.latched_by_zero_count(1));
  rig.run(0.3, uniform(0.0), uniform(0.0));
  rig.run(0.5, uniform(v), uniform(v));
  EXPECT_TRUE(rig.stall.latched(1));
  EXPECT_EQ(rig.releases_seen, 0);
  rig.step(uniform(fast), uniform(fast));
  EXPECT_FALSE(rig.stall.latched(1));
  EXPECT_EQ(rig.releases_seen, 1);
}

TEST(DriveFaultSlowBand, ASlowCommandHeldOnADisplacementLatchEscalates)
{
  // The other half of option (a): the slow command keeps the wheel latched, and
  // because it is above the release band it is "held" -- the latched-and-held
  // criterion latches the whole drive after stall_escalation_latched_sec.
  Rig rig;
  const double fast = fast_command(rig);
  const double v = slow_command(rig);
  const double t = rig.fault_cfg.stall_escalation_latched_sec;
  ASSERT_GT(v, rig.fault_cfg.stall_hold_min_command_rad_s);
  rig.run(rig.stall_cfg.window_sec + 0.2, uniform(fast), {fast, 0.0, fast, fast});
  ASSERT_TRUE(rig.stall.latched(1));
  rig.run(0.3, uniform(0.0), uniform(0.0));
  rig.run(t - 0.3, uniform(v), {v, 0.0, v, v});
  EXPECT_TRUE(rig.stall.latched(1));
  EXPECT_FALSE(rig.fault.latched());
  rig.run(0.6, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallLatchedHeld);
  EXPECT_EQ(rig.fault.record().wheel, 1);
  EXPECT_FALSE(rig.fault.record().zero_count);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultSlowBand, ADitheringDisplacementLatchedWheelDrivenOnSlowlyEscalates)
{
  // Audit M3 (2026-10-29), the reason option (b) was rejected. A blocked wheel
  // whose LIVE encoder dithers by one count (net zero) -- the case a displacement
  // latch is typically set by -- and the operator drives on slowly. Released at
  // 0.71 (option b) it was watched by nothing: displacement does not judge below
  // stall_min, every dither resets the zero-count window, and the latched-and-held
  // escalation no longer applied. With the per-criterion edge it stays latched
  // and the whole drive latches as STALL_LATCHED_HELD.
  Rig rig;
  const double fast = fast_command(rig);
  const double v = slow_command(rig);
  int cycles = 0;
  const auto drive = [&](double seconds, double cmd) {
      for (int k = 0; k < static_cast<int>(std::ceil(seconds / kDt)); ++k) {
        ++cycles;
        Wheels p = rig.position;
        for (std::size_t i : {0u, 2u, 3u}) {
          p[i] += cmd * kDt;
        }
        p[1] = ((cycles / 10) % 2) ? count(rig) : 0.0;
        rig.step_to(uniform(cmd), p);
      }
    };
  drive(rig.stall_cfg.window_sec + 0.3, fast);
  ASSERT_TRUE(rig.stall.latched(1)) << "displacement latch on the blocked wheel";
  ASSERT_FALSE(rig.stall.latched_by_zero_count(1));
  drive(0.3, 0.0);  // the operator lets go
  drive(rig.fault_cfg.stall_escalation_latched_sec - 0.3, v);
  EXPECT_TRUE(rig.stall.latched(1)) << "released by a slow command: watched by nothing";
  EXPECT_EQ(rig.last_out[1], 0.0) << "the blocked wheel is being driven";
  EXPECT_FALSE(rig.fault.latched());
  drive(0.6, v);
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallLatchedHeld);
  EXPECT_EQ(rig.fault.record().wheel, 1);
  EXPECT_FALSE(rig.fault.record().zero_count);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultSlowBand, BothRiseEdgesAreStrictlyAboveTheirThresholds)
{
  // The documented boundary for each latch kind: the rise edge is a strict `>`,
  // the comparison with which the criterion arms. A command of EXACTLY the
  // threshold is not fresh; the next representable value above it is.
  const auto check = [](bool zero_count) {
      Rig rig;
      const double fast = fast_command(rig);
      const double v = slow_command(rig);
      if (zero_count) {
        rig.run(rig.stall_cfg.silent_window_sec + 0.2, uniform(v), {v, 0.0, v, v});
        ASSERT_TRUE(rig.stall.latched_by_zero_count(1));
      } else {
        rig.run(rig.stall_cfg.window_sec + 0.2, uniform(fast), {fast, 0.0, fast, fast});
        ASSERT_TRUE(rig.stall.latched(1));
        ASSERT_FALSE(rig.stall.latched_by_zero_count(1));
      }
      const double edge =
        zero_count ? rig.stall_cfg.silent_min_command_rad_s : rig.stall_cfg.min_command_rad_s;
      // The fall edge is inclusive: exactly the release value arms the release.
      rig.run(0.3, uniform(rig.stall_cfg.release_command_rad_s), uniform(0.0));
      // Held at exactly the edge: still latched (kept short of the latched-and-held
      // escalation, which a command at the edge does feed -- it is above the band).
      // The other three roll, or the zero-count criterion would rightly trip them.
      rig.run(
        0.5 * rig.fault_cfg.stall_escalation_latched_sec, uniform(edge), {edge, 0.0, edge, edge});
      EXPECT_TRUE(rig.stall.latched(1)) << "zero_count=" << zero_count;
      EXPECT_EQ(rig.releases_seen, 0) << "zero_count=" << zero_count;
      EXPECT_FALSE(rig.fault.latched()) << "zero_count=" << zero_count;
      const double above = std::nextafter(edge, std::numeric_limits<double>::infinity());
      rig.step(uniform(above), uniform(0.0));
      EXPECT_FALSE(rig.stall.latched(1)) << "zero_count=" << zero_count;
      EXPECT_EQ(rig.releases_seen, 1) << "zero_count=" << zero_count;
    };
  check(false);
  check(true);
}

TEST(DriveFaultSlowBand, ASlowReleaseOfAStillBlockedZeroCountWheelIsWatchedAgain)
{
  // A zero-count latch released by a slow rise lands in the band where the
  // displacement criterion does NOT judge. The wheel is still blocked (zero
  // counts): the zero-count criterion must catch it again after its window.
  Rig rig;
  const double v = slow_command(rig);
  const double w = rig.stall_cfg.silent_window_sec;
  rig.run(w + 0.2, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched_by_zero_count(1));
  rig.run(0.3, uniform(0.0), uniform(0.0));
  rig.step(uniform(v), {v, 0.0, v, v});
  ASSERT_FALSE(rig.stall.latched(1));
  ASSERT_EQ(rig.releases_seen, 1);
  // Driven again, unjudged by displacement, but watched by the zero-count one.
  rig.run(w - 0.2, uniform(v), {v, 0.0, v, v});
  EXPECT_FALSE(rig.stall.latched(1));
  EXPECT_FALSE(rig.stall.armed(1)) << "displacement criterion does not judge here";
  EXPECT_TRUE(rig.stall.silent_armed(1)) << "the released wheel must be watched";
  rig.run(0.4, uniform(v), {v, 0.0, v, v});
  EXPECT_TRUE(rig.stall.latched_by_zero_count(1));
  EXPECT_EQ(rig.stall.trip_count(1), 2u);
  EXPECT_EQ(rig.zero_count_trips_seen, 2);
  // A re-trip that soon is a persistent stall (user decision 2026-10-16).
  ASSERT_GT(rig.fault_cfg.stall_escalation_window_sec, w + 0.2);
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallRetrip);
  EXPECT_EQ(rig.fault.record().wheel, 1);
  EXPECT_TRUE(rig.fault.record().zero_count);
  EXPECT_TRUE(rig.all_zero());
}

// --------------------------------------------- slow start from standstill
// A slow start satisfies the multi-wheel / FEEDBACK_LOST path only if two or
// more LIVE wheels show NO count at all for a whole stall_silent_window_sec.
// The displacement criterion does not judge in the band, so the 2026-08-25
// finding (0.05 rad net only from 0.79-1.43 rad/s) cannot latch anything here.
TEST(DriveFaultSlowStart, BreakAwayInsideTheWindowIsNoFault)
{
  Rig rig;
  const double v = slow_command(rig);
  const double delay = 0.5 * rig.stall_cfg.silent_window_sec;
  rig.run(delay, uniform(v), uniform(0.0));
  rig.run(5.0, uniform(v), uniform(v));
  EXPECT_EQ(rig.stall_trips_seen, 0);
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out, uniform(v));
}

TEST(DriveFaultSlowStart, CountingButNotBreakingAwayIsNoFault)
{
  // The 2026-08-25 shape: wheels that twitch (counts, forward and back) but
  // never accumulate 0.05 rad. Ten seconds of it on all four wheels.
  Rig rig;
  const double v = slow_command(rig);
  const int period = static_cast<int>(0.25 * rig.stall_cfg.silent_window_sec / kDt);
  const Wheels rest = rig.position;
  Wheels jitter = rest;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    jitter[i] += count(rig);
  }
  const int cycles = static_cast<int>(10.0 / kDt);
  for (int c = 0; c < cycles; ++c) {
    rig.step_to(uniform(v), ((c / period) % 2 == 0) ? rest : jitter);
  }
  EXPECT_EQ(rig.stall_trips_seen, 0);
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out, uniform(v));
}

TEST(DriveFaultSlowStart, AllFourWithNoCountAtAllIsFeedbackLost)
{
  // Zero counts on every LIVE wheel for a whole window while commanded in the
  // band: not a slow start -- every encoder silent at once is the feedback
  // path. The first completes tier 1, the rest are the multi-wheel criterion.
  Rig rig;
  const double v = slow_command(rig);
  rig.run(rig.stall_cfg.silent_window_sec - 3 * kDt, uniform(v), uniform(0.0));
  EXPECT_FALSE(rig.fault.latched()) << "not before the window";
  rig.run(6 * kDt, uniform(v), uniform(0.0));
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultFeedbackLost);
  EXPECT_TRUE(rig.fault.record().zero_count);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultSlowStart, OneSilentWheelIsTier1AndReleasesOnAFreshSlowCommand)
{
  Rig rig;
  const double v = slow_command(rig);
  rig.run(rig.stall_cfg.silent_window_sec + 0.2, uniform(v), {v, v, 0.0, v});
  ASSERT_TRUE(rig.stall.latched(2));
  EXPECT_FALSE(rig.fault.latched());
  rig.run(0.5, uniform(0.0), uniform(0.0));
  rig.run(3.0, uniform(v), uniform(v));
  EXPECT_FALSE(rig.stall.latched(2));
  EXPECT_FALSE(rig.fault.latched());
}

// ------------------------------------------------------------- marker file
TEST(DriveFaultMarker, RoundTripsAndAnUnreadableMarkerStillLatches)
{
  DriveFaultRecord r;
  r.active = true;
  r.reason = kDriveFaultEncoderMissing;
  r.wheel = 3;
  r.trip_time_sec = 1792154059.526243312;
  r.commanded_rad_s = 4.2857;
  r.provenance = kStallProvenanceLiveUnconfirmed;
  r.accumulated_sec = 2.03;
  r.zero_count = true;
  DriveFaultRecord back;
  ASSERT_TRUE(parse_drive_fault(serialize_drive_fault(r), back));
  EXPECT_TRUE(back.active);
  EXPECT_EQ(back.reason, r.reason);
  EXPECT_EQ(back.wheel, 3);
  EXPECT_DOUBLE_EQ(back.trip_time_sec, r.trip_time_sec);
  EXPECT_DOUBLE_EQ(back.commanded_rad_s, r.commanded_rad_s);
  EXPECT_EQ(back.provenance, r.provenance);
  EXPECT_TRUE(back.zero_count);
  // A marker written before 2026-10-22 has no zero_count line: still readable.
  DriveFaultRecord old;
  std::string text = serialize_drive_fault(r);
  text.erase(text.find("zero_count="));
  ASSERT_TRUE(parse_drive_fault(text, old));
  EXPECT_FALSE(old.zero_count);

  DriveFaultRecord garbage;
  EXPECT_FALSE(parse_drive_fault("not a marker", garbage));
  EXPECT_TRUE(garbage.active) << "an unreadable marker is still a marker";
  EXPECT_EQ(garbage.reason, kDriveFaultNone);
  DriveFaultMonitor fresh;
  fresh.restore(garbage);
  EXPECT_TRUE(fresh.latched());
}

// ------------------------------------------------------------- the latch
TEST(DriveFaultLatch, ZeroAndNewCommandsDoNotClear)
{
  Rig rig;
  rig.provenance = kOct13;
  const double v = fast_command(rig);
  rig.run(rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), {v, v, v, 0.0});
  ASSERT_TRUE(rig.fault.latched());
  const DriveFaultRecord frozen = rig.fault.record();
  rig.run(30.0, uniform(0.0), uniform(0.0));
  EXPECT_TRUE(rig.fault.latched());
  rig.provenance = kAllLive;  // even if the encoder now reports LIVE
  rig.run(5.0, uniform(-v), uniform(0.0));
  EXPECT_TRUE(rig.fault.latched());
  EXPECT_TRUE(rig.all_zero());
  // The record is frozen at the trip.
  EXPECT_EQ(rig.fault.record().trip_time_sec, frozen.trip_time_sec);
  EXPECT_EQ(rig.fault.record().wheel, frozen.wheel);
}

TEST(DriveFaultLatch, LifecycleCallsDoNotClear)
{
  // What SwerveController calls on configure / activate / deactivate.
  Rig rig;
  rig.provenance = kOct13;
  const double v = fast_command(rig);
  rig.run(rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), {v, v, v, 0.0});
  ASSERT_TRUE(rig.fault.latched());
  rig.fault.reset_activation_state();
  rig.fault.configure(rig.fault_cfg);
  rig.fault.restore(DriveFaultRecord{});  // an inactive record can never clear
  rig.stall.reset();
  EXPECT_TRUE(rig.fault.latched());
  rig.run(1.0, uniform(v), uniform(v));
  EXPECT_TRUE(rig.all_zero());
  // A disabled config still ENFORCES an existing latch.
  auto off = rig.fault_cfg;
  off.enabled = false;
  rig.fault.configure(off);
  rig.run(1.0, uniform(v), uniform(v));
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultLatch, ARestoredRecordIsEnforcedByAFreshMonitor)
{
  // The controller re-load path: a new object adopts the process-wide record.
  Rig first;
  first.provenance = kOct13;
  const double v = fast_command(first);
  first.run(first.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), {v, v, v, 0.0});
  ASSERT_TRUE(first.fault.latched());

  Rig second;
  second.fault.restore(first.fault.record());
  second.run(1.0, uniform(v), uniform(v));
  EXPECT_TRUE(second.fault.latched());
  EXPECT_TRUE(second.all_zero());
  EXPECT_EQ(second.fault.record().wheel, static_cast<int>(kBR));
}

// ------------------------------------------------------------------ twin
TEST(DriveFaultSim, NoProvenanceTopicMeansLiveUntilAMessageArrives)
{
  Rig rig;
  rig.fault_cfg.assume_live_until_provenance_received = true;
  rig.fault.configure(rig.fault_cfg);
  rig.provenance = {kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
    kStallProvenanceUnknown};
  rig.provenance_received = false;
  const double v = slow_command(rig);
  rig.run(10.0 * rig.fault_cfg.encoder_confirm_window_sec, uniform(v), uniform(v));
  EXPECT_FALSE(rig.fault.latched()) << "the twin must not trip on a topic nobody publishes";
  // A provenance array published into the twin is believed: the feature is live.
  rig.provenance = kOct13;
  rig.provenance_received = true;
  rig.run(rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), {v, v, v, 0.0});
  EXPECT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().wheel, static_cast<int>(kBR));
}

TEST(DriveFaultSim, OnTheRobotNoMessageIsNotLive)
{
  Rig rig;
  rig.provenance = {kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
    kStallProvenanceUnknown};
  rig.provenance_received = false;
  const double v = slow_command(rig);
  rig.run(rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), uniform(v));
  EXPECT_TRUE(rig.fault.latched());
}

TEST(StallDetectorConfigRules, TheZeroCountThresholdsAreValidated)
{
  StallDetectorConfig c;  // the defaults are valid
  std::string error;
  ASSERT_TRUE(StallDetector::validate(c, error)) << error;

  // encoder_confirm_min <= stall_min: equal is allowed (empty band, still
  // contiguous); above is refused.
  c.silent_min_command_rad_s = c.min_command_rad_s;
  EXPECT_TRUE(StallDetector::validate(c, error)) << error;
  c.silent_min_command_rad_s =
    std::nextafter(c.min_command_rad_s, std::numeric_limits<double>::infinity());
  error.clear();
  EXPECT_FALSE(StallDetector::validate(c, error));
  EXPECT_NE(error.find("encoder_confirm_min_command_rad_s"), std::string::npos) << error;
  c.silent_min_command_rad_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(StallDetector::validate(c, error));

  // OP-25 hysteresis for a zero-count latch (and so for both rise edges).
  c = StallDetectorConfig{};
  c.silent_min_command_rad_s = c.release_command_rad_s;
  EXPECT_FALSE(StallDetector::validate(c, error));

  c = StallDetectorConfig{};
  c.silent_window_sec = 0.0;
  EXPECT_FALSE(StallDetector::validate(c, error));
  c.silent_window_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(StallDetector::validate(c, error));

  c = StallDetectorConfig{};
  c.count_rad = 0.0;
  EXPECT_FALSE(StallDetector::validate(c, error));
  c.count_rad = 2.0 * c.min_position_delta_rad;  // a "count" coarser than the delta
  EXPECT_FALSE(StallDetector::validate(c, error));
}

TEST(DriveFaultConfig, RejectsNonPositiveThresholds)
{
  DriveFaultConfig c;
  std::string error;
  c.encoder_confirm_min_command_rad_s = 0.0;
  EXPECT_FALSE(DriveFaultMonitor::validate(c, error));
  c = DriveFaultConfig{};
  c.encoder_confirm_window_sec = 0.0;
  EXPECT_FALSE(DriveFaultMonitor::validate(c, error));
}

// A YAML can carry .nan / .inf (audit 2026-10-29). stall_window_sec = NaN made
// `elapsed < window` false on every cycle, so every armed wheel tripped one cycle
// after arming; encoder_confirm_window_sec = inf meant SR-18's trigger 1 could
// never fire. EVERY double of both configs is checked, not just those two.
TEST(StallDetectorConfigRules, ValidationRefusesNonFiniteValues)
{
  const double non_finite[] = {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  struct Field
  {
    const char * name;
    double StallDetectorConfig::* member;
  };
  const Field fields[] = {
    {"window_sec", &StallDetectorConfig::window_sec},
    {"min_command_rad_s", &StallDetectorConfig::min_command_rad_s},
    {"min_position_delta_rad", &StallDetectorConfig::min_position_delta_rad},
    {"release_command_rad_s", &StallDetectorConfig::release_command_rad_s},
    {"silent_min_command_rad_s", &StallDetectorConfig::silent_min_command_rad_s},
    {"silent_window_sec", &StallDetectorConfig::silent_window_sec},
    {"count_rad", &StallDetectorConfig::count_rad},
  };
  std::string error;
  ASSERT_TRUE(StallDetector::validate(active_config::stall_config(), error)) << error;
  for (const auto & f : fields) {
    for (const double bad : non_finite) {
      StallDetectorConfig c = active_config::stall_config();
      c.*f.member = bad;
      error.clear();
      EXPECT_FALSE(StallDetector::validate(c, error)) << f.name << " = " << bad;
      EXPECT_FALSE(error.empty()) << f.name;
    }
  }
  // 0 disables the displacement criterion in disguise ("moved less than 0").
  StallDetectorConfig c = active_config::stall_config();
  c.min_position_delta_rad = 0.0;
  EXPECT_FALSE(StallDetector::validate(c, error));
  // ...while a release band of exactly 0 is meaningful and accepted.
  c = active_config::stall_config();
  c.release_command_rad_s = 0.0;
  EXPECT_TRUE(StallDetector::validate(c, error)) << error;
}

TEST(DriveFaultConfig, ValidationRefusesNonFiniteValues)
{
  const double non_finite[] = {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  struct Field
  {
    const char * name;
    double DriveFaultConfig::* member;
    bool zero_allowed;  // 0 = "criterion off" or a release band of exactly 0
  };
  const Field fields[] = {
    {"encoder_confirm_min_command_rad_s", &DriveFaultConfig::encoder_confirm_min_command_rad_s,
      false},
    {"encoder_confirm_window_sec", &DriveFaultConfig::encoder_confirm_window_sec, false},
    {"stall_escalation_window_sec", &DriveFaultConfig::stall_escalation_window_sec, true},
    {"stall_escalation_latched_sec", &DriveFaultConfig::stall_escalation_latched_sec, true},
    {"stall_hold_min_command_rad_s", &DriveFaultConfig::stall_hold_min_command_rad_s, true},
    {"stall_window_sec", &DriveFaultConfig::stall_window_sec, false},
    {"stall_silent_window_sec", &DriveFaultConfig::stall_silent_window_sec, false},
  };
  std::string error;
  ASSERT_TRUE(DriveFaultMonitor::validate(active_config::drive_fault_config(), error)) << error;
  for (const auto & f : fields) {
    for (const double bad : non_finite) {
      DriveFaultConfig c = active_config::drive_fault_config();
      c.*f.member = bad;
      error.clear();
      EXPECT_FALSE(DriveFaultMonitor::validate(c, error)) << f.name << " = " << bad;
      EXPECT_FALSE(error.empty()) << f.name;
    }
    DriveFaultConfig c = active_config::drive_fault_config();
    c.*f.member = -1.0;
    EXPECT_FALSE(DriveFaultMonitor::validate(c, error)) << f.name << " = -1";
    c.*f.member = 0.0;
    error.clear();
    EXPECT_EQ(DriveFaultMonitor::validate(c, error), f.zero_allowed) << f.name << " = 0 " << error;
  }
}

// ------------------------------------------------ audit regressions 2026-10-29
// Cases the independent audit of 8c54417 added; kept as regression tests.

TEST(DriveFaultAuditRegression, ASilentWheelWhoseCommandFlickersAcrossStallMinIsCaught)
{
  // stall_detector.cpp "NO UPPER BOUND": the zero-count accumulator catches a
  // silent LIVE wheel whose command FLICKERS across stall_min_command_rad_s,
  // which restarts the displacement window on every dip.
  Rig rig;
  const double hi = rig.stall_cfg.min_command_rad_s + 0.1;
  const double lo = rig.stall_cfg.min_command_rad_s - 0.1;
  ASSERT_GT(lo, rig.stall_cfg.silent_min_command_rad_s);
  const int period = static_cast<int>(0.5 * rig.stall_cfg.window_sec / kDt);
  const int max_cycles = static_cast<int>((rig.stall_cfg.silent_window_sec + 0.5) / kDt);
  int c = 0;
  for (; c < max_cycles && !rig.stall.latched(1); ++c) {
    const double v = (c % period == 0) ? lo : hi;
    rig.step(uniform(v), {v, 0.0, v, v});
  }
  EXPECT_TRUE(rig.stall.latched(1)) << "silent wheel never latched under a flickering command";
  EXPECT_TRUE(rig.stall.latched_by_zero_count(1));
  EXPECT_EQ(rig.stall_trips_seen - rig.zero_count_trips_seen, 0) << "a displacement trip";
  EXPECT_LE(c * kDt, rig.stall_cfg.silent_window_sec + 3 * kDt);
}

TEST(DriveFaultAuditRegression, ShortSeparatedBurstsWithoutACountAccumulateToATrip)
{
  // PAUSE, NOT RESET: short bursts with 1 s stops in between accumulate. (The
  // displacement criterion, by contrast, disarms and restarts its window.)
  Rig rig;
  const double v = rig.stall_cfg.silent_min_command_rad_s + 0.1;
  const int burst = static_cast<int>(0.2 / kDt);
  const int rest = static_cast<int>(1.0 / kDt);
  int bursts = 0;
  while (!rig.stall.latched(1) && bursts < 50) {
    for (int k = 0; k < burst && !rig.stall.latched(1); ++k) {
      rig.step(uniform(v), {v, 0.0, v, v});
    }
    for (int k = 0; k < rest; ++k) {
      rig.step(uniform(0.0), uniform(0.0));
    }
    ++bursts;
  }
  EXPECT_TRUE(rig.stall.latched(1));
  EXPECT_NEAR(bursts, std::ceil(rig.stall_cfg.silent_window_sec / (burst * kDt)), 1.0);
}

TEST(DriveFaultAuditRegression, OneRealCountIsStillACountAtAHugeAccumulatedPosition)
{
  // Firmware quantisation far from zero: the position as motor_controller.cpp
  // computes it, ((sign * c) / COUNTS_PER_OUTPUT_REV) * 2pi, with FL's mirrored
  // sign, at a count of 2^32 (~8.4e6 rad). COUNTS_PER_OUTPUT_REV is taken from
  // the active count size, which test_encoder_count_firmware.cpp ties to the
  // firmware header. One count per 0.9 window must register; none must trip.
  Rig rig;
  const double two_pi = 2.0 * 3.14159265358979323846;
  const double counts_per_rev = std::round(two_pi / rig.stall_cfg.count_rad);
  const auto fw_pos = [&](int64_t c) {
      return -(((1.0 * static_cast<double>(c)) / counts_per_rev) * two_pi);
    };
  const double v = rig.stall_cfg.silent_min_command_rad_s + 0.1;
  int64_t c = int64_t{1} << 32;
  const int per_count = static_cast<int>(0.9 * rig.stall_cfg.silent_window_sec / kDt);
  for (int k = 0; k < 10 * per_count; ++k) {
    if (k % per_count == 0) {
      ++c;
    }
    rig.step_to(uniform(v), uniform(fw_pos(c)));
  }
  EXPECT_EQ(rig.zero_count_trips_seen, 0) << "one real count per 0.9 window was not a count";
  for (int k = 0; k < static_cast<int>((rig.stall_cfg.silent_window_sec + 0.2) / kDt); ++k) {
    rig.step_to(uniform(v), uniform(fw_pos(c)));
  }
  EXPECT_GE(rig.zero_count_trips_seen, 1) << "no count at a huge position never tripped";
}

// ================================================= OPERATOR CLEAR (2026-11-09)
// SR-18 amendment 2026-11-09: STALL_* reasons may be cleared by an operator.
namespace
{
/// FR blocked under a held fast command until STALL_LATCHED_HELD latches.
void latch_latched_held(Rig & rig, double v)
{
  rig.run(rig.stall_cfg.window_sec + 0.1, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  rig.run(rig.fault_cfg.stall_escalation_latched_sec + 0.2, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.fault.latched());
  ASSERT_EQ(rig.fault.record().reason, kDriveFaultStallLatchedHeld);
}
}  // namespace

TEST(DriveFaultClear, TheClearableSetIsExactlyTheFourDecidedStallReasons)
{
  // D1 + OP-48 (a). ENCODER_MISSING restart-only; FEEDBACK_LOST is step 2.
  EXPECT_FALSE(drive_fault_reason_clearable(kDriveFaultNone));
  EXPECT_FALSE(drive_fault_reason_clearable(kDriveFaultEncoderMissing));
  EXPECT_TRUE(drive_fault_reason_clearable(kDriveFaultStallRetrip));
  EXPECT_TRUE(drive_fault_reason_clearable(kDriveFaultStallTripCount));
  EXPECT_TRUE(drive_fault_reason_clearable(kDriveFaultStallMultiWheel));
  EXPECT_TRUE(drive_fault_reason_clearable(kDriveFaultStallLatchedHeld));
  EXPECT_FALSE(drive_fault_reason_clearable(kDriveFaultFeedbackLost));
  EXPECT_FALSE(drive_fault_reason_clearable(99));
}

TEST(DriveFaultClear, NothingLatchedIsRefusedAlsoWithATier1LatchStanding)
{
  Rig rig;
  EXPECT_EQ(rig.fault.clear_refusal(0), DriveFaultClearRefusal::kNotLatched);
  EXPECT_FALSE(rig.clear());
  const double v = fast_command(rig);
  rig.run(rig.stall_cfg.window_sec + 0.2, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched(1));
  ASSERT_FALSE(rig.fault.latched());
  EXPECT_FALSE(rig.clear());
  EXPECT_TRUE(rig.stall.latched(1)) << "a tier-1 latch keeps its OP-25 release";
  rig.step(uniform(v), {v, 0.0, v, v});
  EXPECT_EQ(rig.last_out[1], 0.0);
}

TEST(DriveFaultClear, EncoderMissingAndFeedbackLostAreRefusedAndChangeNothing)
{
  {
    Rig rig;
    rig.provenance = kOct13;
    const double v = fast_command(rig);
    rig.run(rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), {v, v, v, 0.0});
    ASSERT_EQ(rig.fault.record().reason, kDriveFaultEncoderMissing);
    const DriveFaultRecord before = rig.fault.record();
    EXPECT_EQ(rig.fault.clear_refusal(0), DriveFaultClearRefusal::kNotClearable);
    EXPECT_FALSE(rig.clear(kDriveFaultEncoderMissing));
    EXPECT_TRUE(rig.fault.latched());
    EXPECT_EQ(rig.fault.record().trip_time_sec, before.trip_time_sec);
    rig.run(1.0, uniform(v), uniform(v));
    EXPECT_TRUE(rig.all_zero());
  }
  {
    Rig rig;
    const double v = fast_command(rig);
    rig.run(0.5, uniform(v), uniform(v));
    rig.run(rig.stall_cfg.window_sec + 0.5, uniform(v), uniform(0.0));
    ASSERT_TRUE(rig.fault.latched());
    ASSERT_EQ(rig.fault.record().reason, kDriveFaultFeedbackLost);
    EXPECT_EQ(rig.fault.clear_refusal(kDriveFaultFeedbackLost), DriveFaultClearRefusal::kNotClearable);
    EXPECT_FALSE(rig.clear(kDriveFaultFeedbackLost));
    EXPECT_TRUE(rig.fault.latched());
    rig.run(1.0, uniform(v), uniform(v));
    EXPECT_TRUE(rig.all_zero());
  }
}

TEST(DriveFaultClear, AnExpectedReasonMismatchIsRefusedAndChangesNothing)
{
  Rig rig;
  const double v = fast_command(rig);
  latch_latched_held(rig, v);
  EXPECT_EQ(
    rig.fault.clear_refusal(kDriveFaultStallRetrip),
    DriveFaultClearRefusal::kExpectedReasonMismatch);
  EXPECT_FALSE(rig.clear(kDriveFaultStallRetrip));
  EXPECT_TRUE(rig.fault.latched());
  EXPECT_TRUE(rig.stall.latched(1));
  EXPECT_EQ(rig.fault.clear_refusal(kDriveFaultStallLatchedHeld), DriveFaultClearRefusal::kNone);
  EXPECT_EQ(rig.fault.clear_refusal(0), DriveFaultClearRefusal::kNone);
}

TEST(DriveFaultClear, UnderAHeldCommandAllFourWheelsFollowItOnTheNextCycle)
{
  // D2: no precondition, no OP-25 edge -- the held command drives at once.
  Rig rig;
  const double v = fast_command(rig);
  latch_latched_held(rig, v);
  ASSERT_TRUE(rig.all_zero());
  ASSERT_TRUE(rig.clear(kDriveFaultStallLatchedHeld));
  EXPECT_FALSE(rig.fault.latched());
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_FALSE(rig.stall.latched(i)) << i;
    EXPECT_EQ(rig.fault.latched_held_sec(i), 0.0) << i;
  }
  rig.step(uniform(v), uniform(v));
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_EQ(rig.last_out[i], v) << "wheel " << i;
  }
  EXPECT_EQ(rig.fault.stall_trips(1), 1u) << "diagnostic trip count is kept";
}

TEST(DriveFaultClear, AReTripWithinTheWindowAfterTheClearEscalatesAtTheFirstTrip)
{
  // OP-49 (a): the clear counts as a latch release of the named wheel.
  Rig rig;
  const double v = fast_command(rig);
  ASSERT_GT(rig.fault_cfg.stall_escalation_window_sec, rig.stall_cfg.window_sec + 0.5);
  latch_latched_held(rig, v);
  ASSERT_TRUE(rig.clear());
  const double cleared_at = rig.now;
  const int trips_before = rig.stall_trips_seen;
  rig.run(rig.stall_cfg.window_sec - 0.2, uniform(v), {v, 0.0, v, v});
  EXPECT_FALSE(rig.fault.latched()) << "detection restarts from an empty window";
  rig.run(0.5, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.stall_trips_seen, trips_before + 1) << "escalated at the FIRST re-trip";
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallRetrip);
  EXPECT_EQ(rig.fault.record().wheel, 1);
  EXPECT_NEAR(
    rig.fault.record().since_release_sec, rig.fault.record().trip_time_sec - cleared_at, 1e-9);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultClear, AfterTheWindowAReTripIsTier1Only)
{
  Rig rig;
  const double v = fast_command(rig);
  latch_latched_held(rig, v);
  ASSERT_TRUE(rig.clear());
  rig.run(rig.fault_cfg.stall_escalation_window_sec + 0.5, uniform(v), uniform(v));
  rig.run(rig.stall_cfg.window_sec + 0.2, uniform(v), {v, 0.0, v, v});
  EXPECT_TRUE(rig.stall.latched(1));
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out[0], v);
}

TEST(DriveFaultClear, ASilentEncoderReLatchesAfterTheClearInTheSlowBand)
{
  // C3: a LIVE wheel whose encoder stays silent under a held command above
  // encoder_confirm_min_command_rad_s re-trips tier 1 after
  // stall_silent_window_sec and, inside the escalation window, re-latches.
  Rig rig;
  const double v = slow_command(rig);
  ASSERT_GT(rig.fault_cfg.stall_escalation_window_sec, rig.stall_cfg.silent_window_sec + 0.5);
  rig.run(rig.stall_cfg.silent_window_sec + 0.2, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.stall.latched_by_zero_count(1));
  rig.run(rig.fault_cfg.stall_escalation_latched_sec + 0.2, uniform(v), {v, 0.0, v, v});
  ASSERT_EQ(rig.fault.record().reason, kDriveFaultStallLatchedHeld);
  ASSERT_TRUE(rig.clear());
  rig.run(rig.stall_cfg.silent_window_sec - 0.2, uniform(v), {v, 0.0, v, v});
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.last_out[1], v) << "E4: the silent wheel is driven during the window";
  rig.run(0.5, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallRetrip);
  EXPECT_TRUE(rig.fault.record().zero_count);
  EXPECT_TRUE(rig.all_zero());
}

TEST(DriveFaultClear, E1ASilentWheelAtOrBelowTheThresholdIsNotReDetected)
{
  // Documents the known limit E1 -- it is NOT a requirement that this trips.
  Rig rig;
  const double v = fast_command(rig);
  latch_latched_held(rig, v);
  ASSERT_TRUE(rig.clear());
  const double low = rig.stall_cfg.silent_min_command_rad_s;  // "at or below"
  rig.run(3.0 * (rig.stall_cfg.silent_window_sec + rig.fault_cfg.stall_escalation_latched_sec),
    uniform(low), {low, 0.0, low, low});
  EXPECT_FALSE(rig.stall.latched(1));
  EXPECT_FALSE(rig.fault.latched());
}

TEST(DriveFaultClear, TheTripCountEscalationCounterRestartsForTheNamedWheel)
{
  // OP-48 (a): the escalation counter of the named wheel is reset at the
  // clear, the diagnostic count is kept.
  Rig rig;
  rig.fault_cfg.stall_escalation_trips = 2;
  rig.fault.configure(rig.fault_cfg);
  const double v = fast_command(rig);
  const double rest = rig.fault_cfg.stall_escalation_window_sec + 1.0;
  for (int trip = 0; trip < 2; ++trip) {
    rig.run(0.2, uniform(0.0), uniform(0.0));
    rig.run(rest, uniform(v), uniform(v));
    rig.run(rig.stall_cfg.window_sec + 0.2, uniform(v), {v, v, 0.0, v});
  }
  ASSERT_EQ(rig.fault.record().reason, kDriveFaultStallTripCount);
  ASSERT_TRUE(rig.clear(kDriveFaultStallTripCount));
  EXPECT_EQ(rig.fault.escalation_trips(2), 0u);
  EXPECT_EQ(rig.fault.stall_trips(2), 2u);
  // Free running past the re-trip window, then ONE trip: tier 1 only.
  rig.run(rest, uniform(v), uniform(v));
  rig.run(rig.stall_cfg.window_sec + 0.2, uniform(v), {v, v, 0.0, v});
  EXPECT_TRUE(rig.stall.latched(2));
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_EQ(rig.fault.stall_trips(2), 3u);
}

TEST(DriveFaultClear, ALongFaultDoesNotLeakIntoTheEncoderAccumulatorAfterTheClear)
{
  // The monitor does not advance its clock while latched; the first cycle
  // after a clear must not count the whole faulted period as driven time.
  Rig rig;
  const double v = fast_command(rig);
  latch_latched_held(rig, v);
  rig.run(10.0 * rig.fault_cfg.encoder_confirm_window_sec, uniform(0.0), uniform(0.0));
  ASSERT_TRUE(rig.clear());
  rig.provenance = kOct13;  // BR not LIVE (e.g. an ESP32 restart)
  rig.step(uniform(v), {v, v, v, 0.0});
  EXPECT_FALSE(rig.fault.latched());
  EXPECT_LE(rig.fault.encoder_unconfirmed_sec(kBR), 2.0 * kDt);
  // ...and trigger A is fully armed: it trips after its window as always.
  rig.run(rig.fault_cfg.encoder_confirm_window_sec + 0.2, uniform(v), {v, v, v, 0.0});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultEncoderMissing);
}

TEST(DriveFaultClear, AMultiWheelClearTreatsBothNamedWheelsAsReleased)
{
  Rig rig;
  const double v = fast_command(rig);
  rig.run(rig.stall_cfg.window_sec + 0.3, uniform(v), {v, 0.0, v, v});
  rig.run(rig.stall_cfg.window_sec + 0.3, uniform(v), {v, 0.0, v, 0.0});
  ASSERT_EQ(rig.fault.record().reason, kDriveFaultStallMultiWheel);
  ASSERT_TRUE(rig.clear(kDriveFaultStallMultiWheel));
  // The OTHER named wheel (FR, already latched) re-trips first: still A1.
  rig.run(rig.stall_cfg.window_sec + 0.3, uniform(v), {v, 0.0, v, v});
  ASSERT_TRUE(rig.fault.latched());
  EXPECT_EQ(rig.fault.record().reason, kDriveFaultStallRetrip);
  EXPECT_EQ(rig.fault.record().wheel, 1);
}
