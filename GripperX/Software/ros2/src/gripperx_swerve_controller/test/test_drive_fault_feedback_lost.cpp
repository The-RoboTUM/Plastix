// Controller-level FEEDBACK_LOST: every wheel position freezes together while
// driving (what a frozen /hw/joint_states looks like) -- named as the feedback
// link, not as one wheel.

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

TEST(SwerveControllerStallEscalation, AllPositionsFreezingIsFeedbackLost)
{
  const auto stall = active_config::stall_config();
  Bench bench;
  auto controller = bench.load();
  bench.set_provenance({3, 3, 3, 3});
  bench.settle();
  bench.drive_frozen(*controller, 1.0, 0.3, {false, false, false, false});
  EXPECT_FALSE(bench.last_fault_.active);
  bench.drive_frozen(*controller, stall.window_sec + 1.0, 0.3, {true, true, true, true});
  ASSERT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.reason, DriveFault::REASON_FEEDBACK_LOST);
  EXPECT_EQ(bench.last_fault_.wheel_index, -1);
  EXPECT_EQ(bench.last_fault_.wheel_name, "");
  EXPECT_NE(bench.last_fault_.detail.find("/hw/joint_states"), std::string::npos);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  bench.unload(controller);
  remove_marker(bench);
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
