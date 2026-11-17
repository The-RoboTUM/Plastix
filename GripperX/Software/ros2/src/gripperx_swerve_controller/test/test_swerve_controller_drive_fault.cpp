// The whole-drive fault through the REAL controller, scenario 1: the
// 2026-10-13 shape through every lifecycle transition and a reload. The bench
// (in-memory interfaces, active YAML, real lifecycle) is drive_fault_bench.hpp.
// ONE test per executable on purpose: the fault is latched process-wide.

#include <filesystem>

#include "drive_fault_bench.hpp"

using drive_fault_bench::Bench;


TEST(SwerveControllerDriveFault, Oct13ShapeLatchesThroughEveryLifecycleAndAReload)
{
  const auto cfg = active_config::drive_fault_config();
  Bench bench;
  auto controller = bench.load();
  bench.settle();
  ASSERT_GT(bench.fault_messages_, 0) << "the latched fault topic must say 'no fault' too";
  EXPECT_FALSE(bench.last_fault_.active);

  // ---- phase A: LIVE_UNCONFIRMED everywhere, standing still -> nothing.
  bench.set_provenance({2, 2, 2, 2});
  bench.drive(*controller, 3.0 * cfg.encoder_confirm_window_sec, 0.0, false);
  EXPECT_FALSE(bench.last_fault_.active);

  // ---- phase B: the 2026-10-13 shape, driving straight at 0.3 m/s.
  bench.set_provenance({3, 3, 3, 2});
  bench.settle();
  bench.drive(*controller, 0.5 * cfg.encoder_confirm_window_sec, 0.3, true);
  EXPECT_FALSE(bench.last_fault_.active);
  EXPECT_GT(bench.max_br_command_, cfg.encoder_confirm_min_command_rad_s)
    << "BR must actually have been driven before the fault";
  EXPECT_FALSE(bench.all_wheels_exactly_zero());
  bench.drive(*controller, 0.5 * cfg.encoder_confirm_window_sec + 0.5, 0.3, true);
  ASSERT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.reason, DriveFault::REASON_ENCODER_MISSING);
  EXPECT_TRUE(std::filesystem::exists(bench.marker_path))
    << "the crash-surviving marker must be written on the trip";
  EXPECT_EQ(bench.last_fault_.wheel_index, 3);
  EXPECT_EQ(bench.last_fault_.wheel_name, bench.wheel_[3]);
  EXPECT_EQ(bench.last_fault_.provenance, 2);
  EXPECT_GE(bench.last_fault_.accumulated_sec, cfg.encoder_confirm_window_sec);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());

  // ---- zero command, then a new command: still latched, still zero.
  bench.drive(*controller, 1.0, 0.0, true);
  bench.set_provenance({3, 3, 3, 3});
  bench.drive(*controller, 1.0, -0.3, false);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());

  // ---- deactivate / activate (a controller switch).
  bench.deactivate(*controller);
  bench.activate(*controller);
  bench.drive(*controller, 1.0, 0.3, false);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  bench.settle();
  EXPECT_TRUE(bench.last_fault_.active);

  // ---- unload and load a NEW controller instance in the same process.
  bench.unload(controller);
  const int before_reload = bench.fault_messages_;
  controller = bench.load();
  bench.settle();
  EXPECT_GT(bench.fault_messages_, before_reload);
  EXPECT_TRUE(bench.last_fault_.active) << "a re-loaded controller must adopt the latch";
  EXPECT_EQ(bench.last_fault_.wheel_index, 3);
  bench.drive(*controller, 1.0, 0.3, false);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());

  bench.unload(controller);
  std::error_code ec;
  std::filesystem::remove_all(std::filesystem::path(bench.marker_path).parent_path(), ec);
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
