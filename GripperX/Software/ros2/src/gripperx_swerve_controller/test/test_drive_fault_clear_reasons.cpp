// Operator clear (SR-18 amendment 2026-11-09), C1/C2 per reason in the REAL
// controller: each clearable reason, latched by a crash-surviving marker as a
// previous process would have left it, is cleared under a HELD command and all
// four written wheel commands follow the held request on the cycle the clear
// is applied (no OP-25 edge; within the FR-14 regulator's authority, which is
// enabled in the active config); steering bit-identical to the no-fault run. Refusals: nothing
// latched, nothing latched but a single-wheel tier-1 latch, and -- last,
// because it stays latched for the rest of this process -- ENCODER_MISSING.
// The clear goes through request_drive_fault_clear() (the service body) on a
// second thread while this thread runs the control loop.

#include <cmath>
#include <filesystem>
#include <vector>

#include "drive_fault_bench.hpp"

using drive_fault_bench::Bench;
using gripperx_swerve_controller::DriveFaultRecord;

TEST(SwerveControllerDriveFaultClearReasons, EachStallReasonClearsAndTheRestAreRefused)
{
  const auto stall = active_config::stall_config();
  const auto params = active_config::swerve_parameters();
  const double radius = params["wheel_radius"].as<double>();
  const double wheel = 2.0 * stall.min_command_rad_s + 1.0;
  const double vx = wheel * radius;
  const std::array<bool, 4> none{false, false, false, false};

  Bench bench;
  bench.remove_marker_dir();
  auto controller = bench.load();
  bench.set_provenance({3, 3, 3, 3});
  bench.settle();
  bench.drive_frozen(*controller, 1.0, vx, none);
  ASSERT_TRUE(bench.wheels_follow(wheel));
  const auto ref_steer = bench.steer_commands();
  const auto regulator = active_config::regulator_config();
  if (regulator.enabled && wheel > regulator.floor_rad_s) {
    // The accessor reads the live FR-14 state: non-zero while regulating, so
    // the zero checked after each clear below is a statement, not a default.
    double sum = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
      sum += std::fabs(controller->wheel_regulator_integrator(i));
    }
    EXPECT_GT(sum, 0.0);
  }

  // ---- C2: nothing latched.
  auto nothing = bench.clear(*controller, "cli", 0, vx);
  EXPECT_FALSE(nothing.success);
  EXPECT_EQ(nothing.reason, 0);
  EXPECT_NE(nothing.message.find("no whole-drive fault is latched"), std::string::npos)
    << nothing.message;
  EXPECT_TRUE(bench.wheels_follow(wheel));

  // ---- C2: a single-wheel tier-1 latch is not a whole-drive fault.
  bench.drive_frozen(*controller, stall.window_sec + 0.3, vx, {false, true, false, false});
  ASSERT_EQ(bench.wheel_command(1), 0.0);
  ASSERT_NE(bench.wheel_command(0), 0.0);
  auto tier1 = bench.clear(*controller, "cli", 0, vx, {false, true, false, false});
  EXPECT_FALSE(tier1.success);
  EXPECT_NE(tier1.message.find("single-wheel"), std::string::npos) << tier1.message;
  EXPECT_EQ(bench.wheel_command(1), 0.0) << "the tier-1 latch is untouched";
  bench.unload(controller);

  // ---- C1: every clearable reason.
  const std::vector<std::pair<int, int>> clearable{
    {DriveFault::REASON_STALL_RETRIP, -1},
    {DriveFault::REASON_STALL_TRIP_COUNT, -1},
    {DriveFault::REASON_STALL_MULTI_WHEEL, 3},
    {DriveFault::REASON_STALL_LATCHED_HELD, -1}};
  uint32_t clears = 0;
  for (const auto & [reason, other] : clearable) {
    SCOPED_TRACE(gripperx_swerve_controller::drive_fault_reason_label(reason));
    DriveFaultRecord record;
    record.active = true;
    record.reason = reason;
    record.wheel = 1;
    record.other_wheel = other;
    record.provenance = 3;
    bench.write_marker(record);
    controller = bench.load();
    bench.settle();
    ASSERT_TRUE(bench.last_fault_.active);
    ASSERT_EQ(bench.last_fault_.reason, reason);
    bench.drive_frozen(*controller, 1.0, vx, none);
    ASSERT_TRUE(bench.all_wheels_exactly_zero());
    EXPECT_EQ(bench.steer_commands(), ref_steer);

    auto outcome = bench.clear(*controller, "web_teleop", reason, vx);
    ASSERT_TRUE(outcome.success) << outcome.message;
    ++clears;
    EXPECT_EQ(outcome.reason, reason);
    EXPECT_TRUE(bench.wheels_follow(wheel)) << "held command drives on the clear cycle";
    for (std::size_t i = 0; i < 4; ++i) {
      // C4 / item 3 (d): integrators start from zero, never from a stored value.
      EXPECT_EQ(controller->wheel_regulator_integrator(i), 0.0) << "wheel " << i;
    }
    EXPECT_EQ(bench.steer_commands(), ref_steer);
    EXPECT_FALSE(std::filesystem::exists(bench.marker_path));
    bench.drive_frozen(*controller, 0.5, vx, none);
    EXPECT_TRUE(bench.wheels_follow(wheel));
    bench.settle();
    EXPECT_FALSE(bench.last_fault_.active);
    EXPECT_EQ(bench.last_fault_.clear_count, clears);
    EXPECT_EQ(bench.last_fault_.last_cleared_reason, reason);
    EXPECT_EQ(bench.last_fault_.last_clear_requester, "web_teleop");
    bench.unload(controller);
  }

  // ---- C2: ENCODER_MISSING is restart-only (and stays latched from here on).
  DriveFaultRecord missing;
  missing.active = true;
  missing.reason = DriveFault::REASON_ENCODER_MISSING;
  missing.wheel = 3;
  missing.provenance = 2;
  bench.write_marker(missing);
  controller = bench.load();
  bench.settle();
  ASSERT_EQ(bench.last_fault_.reason, DriveFault::REASON_ENCODER_MISSING);
  auto rejected = bench.clear(*controller, "cli", 0, vx);
  EXPECT_FALSE(rejected.success);
  EXPECT_EQ(rejected.reason, DriveFault::REASON_ENCODER_MISSING);
  EXPECT_NE(rejected.message.find("stack restart required"), std::string::npos)
    << rejected.message;
  bench.drive_frozen(*controller, 0.5, vx, none);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  EXPECT_TRUE(std::filesystem::exists(bench.marker_path));
  bench.settle();
  EXPECT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.clear_count, clears);
  bench.unload(controller);
  bench.remove_marker_dir();
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
