// Controller-level stall escalation, STALL_LATCHED_HELD (user decision
// 2026-10-16, the OP-25 deadlock): a LIVE wheel whose position never moves
// under a held command is first a tier-1 single-wheel latch, then -- after
// stall_escalation_latched_sec -- the whole-drive fault. Exercises the stage-4
// wiring of stall_events / stall_latched in write_wheel_commands().

#include <filesystem>

#include "drive_fault_bench.hpp"

using drive_fault_bench::Bench;

namespace
{
void remove_marker(const Bench & bench)
{
  std::error_code ec;
  std::filesystem::remove_all(std::filesystem::path(bench.marker_path).parent_path(), ec);
}
}  // namespace

TEST(SwerveControllerStallEscalation, LiveWheelFrozenUnderHeldCommandIsLatchedHeld)
{
  const auto cfg = active_config::drive_fault_config();
  const auto stall = active_config::stall_config();
  Bench bench;
  auto controller = bench.load();
  bench.set_provenance({3, 3, 3, 3});
  bench.settle();
  const std::array<bool, 4> fr_frozen{false, true, false, false};

  bench.drive_frozen(*controller, stall.window_sec + 1.0, 0.3, fr_frozen);
  EXPECT_FALSE(bench.last_fault_.active) << "a fresh stall is tier 1 only";
  EXPECT_EQ(bench.wheel_command(1), 0.0) << "FR latched off by HWR-30a";
  EXPECT_NE(bench.wheel_command(0), 0.0) << "the other wheels keep driving";

  bench.drive_frozen(*controller, cfg.stall_escalation_latched_sec + 0.5, 0.3, fr_frozen);
  ASSERT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.reason, DriveFault::REASON_STALL_LATCHED_HELD);
  EXPECT_EQ(bench.last_fault_.wheel_index, 1);
  EXPECT_GE(bench.last_fault_.latched_held_sec, cfg.stall_escalation_latched_sec);
  EXPECT_NE(bench.last_fault_.detail.find("silent encoder"), std::string::npos)
    << "SR-13 2026-09-22: must not claim a stall";
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  bench.unload(controller);
  remove_marker(bench);
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
