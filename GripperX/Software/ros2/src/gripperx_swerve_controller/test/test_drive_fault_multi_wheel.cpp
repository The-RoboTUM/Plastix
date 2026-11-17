// Controller-level stall escalation, STALL_MULTI_WHEEL: FR stalls and is
// latched, later BR stalls too while FL/BL keep turning.

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

TEST(SwerveControllerStallEscalation, SecondWheelStallingIsMultiWheel)
{
  const auto stall = active_config::stall_config();
  Bench bench;
  auto controller = bench.load();
  bench.set_provenance({3, 3, 3, 3});
  bench.settle();
  bench.drive_frozen(*controller, stall.window_sec + 1.0, 0.3, {false, true, false, false});
  EXPECT_FALSE(bench.last_fault_.active);
  EXPECT_EQ(bench.wheel_command(1), 0.0);
  bench.drive_frozen(*controller, stall.window_sec + 1.0, 0.3, {false, true, false, true});
  ASSERT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.reason, DriveFault::REASON_STALL_MULTI_WHEEL);
  EXPECT_EQ(bench.last_fault_.wheel_index, 3);
  EXPECT_EQ(bench.last_fault_.other_wheel_index, 1);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  bench.unload(controller);
  remove_marker(bench);
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
