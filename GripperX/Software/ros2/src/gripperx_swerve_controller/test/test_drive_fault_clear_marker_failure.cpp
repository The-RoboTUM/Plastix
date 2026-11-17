// Operator clear (SR-18 amendment 2026-11-09), failure rule (item 5) and C5:
// when the marker cannot be removed the clear FAILS and the fault stays
// latched in memory AND in the marker (also across a reload); once removal
// works the clear succeeds and a simulated crash-respawn (a fresh load that
// only has the marker to go by) does not re-latch. Then FEEDBACK_LOST is
// refused (restart-only until step 2). Removal is made to fail by a
// read-only marker directory, which does not hold for root -- skipped there.

#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>

#include "drive_fault_bench.hpp"

using drive_fault_bench::Bench;
using gripperx_swerve_controller::DriveFaultRecord;

namespace
{
int marker_reason(const std::string & path)
{
  std::ifstream in(path);
  std::stringstream text;
  text << in.rdbuf();
  DriveFaultRecord record;
  return gripperx_swerve_controller::parse_drive_fault(text.str(), record) ? record.reason : -1;
}
}  // namespace

TEST(SwerveControllerDriveFaultClearMarker, ARemovalFailureKeepsTheLatchEverywhere)
{
  if (::geteuid() == 0) {
    GTEST_SKIP() << "root ignores the read-only directory used to make removal fail";
  }
  const auto stall = active_config::stall_config();
  const double radius = active_config::swerve_parameters()["wheel_radius"].as<double>();
  const double vx = (2.0 * stall.min_command_rad_s + 1.0) * radius;

  Bench bench;
  bench.remove_marker_dir();
  DriveFaultRecord record;
  record.active = true;
  record.reason = DriveFault::REASON_STALL_LATCHED_HELD;
  record.wheel = 1;
  record.provenance = 3;
  bench.write_marker(record);
  const std::string dir = std::filesystem::path(bench.marker_path).parent_path().string();

  auto controller = bench.load();
  bench.set_provenance({3, 3, 3, 3});
  bench.settle();
  ASSERT_TRUE(bench.last_fault_.active);

  ASSERT_EQ(::chmod(dir.c_str(), 0500), 0);
  auto failed = bench.clear(*controller, "cli", 0, vx);
  EXPECT_FALSE(failed.success);
  EXPECT_NE(failed.message.find("could not remove the drive fault marker"), std::string::npos)
    << failed.message;
  bench.drive_frozen(*controller, 0.5, vx, {false, false, false, false});
  EXPECT_TRUE(bench.all_wheels_exactly_zero()) << "the fault stays latched in memory";
  EXPECT_EQ(marker_reason(bench.marker_path), DriveFault::REASON_STALL_LATCHED_HELD)
    << "and in the marker";
  bench.settle();
  EXPECT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.clear_count, 0u);
  bench.unload(controller);
  controller = bench.load();
  bench.settle();
  EXPECT_TRUE(bench.last_fault_.active) << "a reload still adopts it";

  ASSERT_EQ(::chmod(dir.c_str(), 0700), 0);
  auto ok = bench.clear(*controller, "cli", 0, vx);
  ASSERT_TRUE(ok.success) << ok.message;
  EXPECT_FALSE(std::filesystem::exists(bench.marker_path));
  bench.unload(controller);

  // Crash-respawn: a fresh load finds no marker -> not latched.
  controller = bench.load();
  bench.settle();
  EXPECT_FALSE(bench.last_fault_.active);
  bench.drive_frozen(*controller, 0.5, vx, {false, false, false, false});
  EXPECT_FALSE(bench.all_wheels_exactly_zero());
  bench.unload(controller);

  // FEEDBACK_LOST: refused, restart-only (step 2 is not decided to be built).
  DriveFaultRecord lost;
  lost.active = true;
  lost.reason = DriveFault::REASON_FEEDBACK_LOST;
  lost.wheel = -1;
  lost.other_wheel = 2;
  bench.write_marker(lost);
  controller = bench.load();
  bench.settle();
  ASSERT_EQ(bench.last_fault_.reason, DriveFault::REASON_FEEDBACK_LOST);
  auto refused = bench.clear(*controller, "cli", DriveFault::REASON_FEEDBACK_LOST, vx);
  EXPECT_FALSE(refused.success);
  EXPECT_NE(refused.message.find("stack restart required"), std::string::npos)
    << refused.message;
  bench.drive_frozen(*controller, 0.5, vx, {false, false, false, false});
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  EXPECT_TRUE(std::filesystem::exists(bench.marker_path));
  bench.unload(controller);
  bench.remove_marker_dir();
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
