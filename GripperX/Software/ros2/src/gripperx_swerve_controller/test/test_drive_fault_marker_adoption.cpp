// A crash-respawn of ros2_control_node is a NEW process: the in-process mirror
// is gone, the tmpfs marker is not. A controller loaded with a marker present
// must come up latched (user decision 2026-10-16).

#include <fstream>
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

TEST(SwerveControllerDriveFaultMarker, ANewProcessAdoptsTheMarker)
{
  Bench bench;
  gripperx_swerve_controller::DriveFaultRecord record;
  record.active = true;
  record.reason = gripperx_swerve_controller::kDriveFaultEncoderMissing;
  record.wheel = 3;
  record.commanded_rad_s = 4.2857;
  record.provenance = 2;
  record.accumulated_sec = 2.03;
  std::filesystem::create_directories(std::filesystem::path(bench.marker_path).parent_path());
  {
    std::ofstream out(bench.marker_path);
    out << gripperx_swerve_controller::serialize_drive_fault(record);
  }
  auto controller = bench.load();
  bench.set_provenance({3, 3, 3, 3});
  bench.settle();
  ASSERT_TRUE(bench.last_fault_.active) << "the marker must latch a freshly started controller";
  EXPECT_EQ(bench.last_fault_.reason, DriveFault::REASON_ENCODER_MISSING);
  EXPECT_EQ(bench.last_fault_.wheel_index, 3);
  bench.drive_frozen(*controller, 1.0, 0.3, {false, false, false, false});
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  EXPECT_TRUE(std::filesystem::exists(bench.marker_path)) << "nothing in the controller deletes it";
  bench.unload(controller);
  remove_marker(bench);
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
