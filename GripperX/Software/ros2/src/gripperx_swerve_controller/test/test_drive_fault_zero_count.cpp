// Controller-level zero-count criterion (user decision 2026-10-22, option c): a
// LIVE wheel commanded in the slow band whose encoder shows no count change is
// first a tier-1 single-wheel latch after stall_silent_window_sec, then --
// with the command held -- the whole-drive fault (STALL_LATCHED_HELD), and the
// published detail names the zero-count criterion. Exercises the new
// parameters, the stage-3/4 plumbing and the describe text in the real
// controller.

#include <cmath>
#include <filesystem>

#include "drive_fault_bench.hpp"

using drive_fault_bench::Bench;

TEST(SwerveControllerZeroCount, SilentLiveWheelInTheSlowBandLatchesThenEscalatesWhenHeld)
{
  const auto cfg = active_config::drive_fault_config();
  const auto stall = active_config::stall_config();
  const auto params = active_config::swerve_parameters();
  const double radius = params["wheel_radius"].as<double>();
  // A quarter of the way up the slow band, as body speed for straight driving.
  const double wheel =
    stall.silent_min_command_rad_s + 0.25 * (stall.min_command_rad_s - stall.silent_min_command_rad_s);
  const double vx = wheel * radius;

  Bench bench;
  auto controller = bench.load();
  bench.set_provenance({3, 3, 3, 3});
  bench.settle();
  const std::array<bool, 4> fr_silent{false, true, false, false};

  bench.drive_frozen(*controller, 0.5, vx, {false, false, false, false});
  ASSERT_GT(std::fabs(bench.wheel_command(1)), stall.silent_min_command_rad_s);
  ASSERT_LE(std::fabs(bench.wheel_command(1)), stall.min_command_rad_s);

  bench.drive_frozen(*controller, stall.silent_window_sec + 0.5, vx, fr_silent);
  EXPECT_FALSE(bench.last_fault_.active) << "a fresh zero-count trip is tier 1 only";
  EXPECT_EQ(bench.wheel_command(1), 0.0) << "FR latched off by the zero-count criterion";
  EXPECT_NE(bench.wheel_command(0), 0.0) << "the other wheels keep driving";

  bench.drive_frozen(*controller, cfg.stall_escalation_latched_sec + 0.5, vx, fr_silent);
  ASSERT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.reason, DriveFault::REASON_STALL_LATCHED_HELD);
  EXPECT_EQ(bench.last_fault_.wheel_index, 1);
  EXPECT_NE(bench.last_fault_.detail.find("ZERO-COUNT"), std::string::npos)
    << bench.last_fault_.detail;
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  bench.unload(controller);
  std::error_code ec;
  std::filesystem::remove_all(std::filesystem::path(bench.marker_path).parent_path(), ec);
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
