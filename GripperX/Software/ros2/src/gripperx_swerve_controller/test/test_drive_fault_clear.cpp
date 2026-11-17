// Operator clear of the whole-drive fault (SR-18 amendment 2026-11-09) through
// the REAL ROS service /swerve_controller/clear_drive_fault, with the
// controller node spun on its own multi-threaded executor (as in
// controller_manager) and update() driven by this thread.
//
// One scenario, in order:
//  1. a silent FR encoder in the slow band under a held command -> zero-count
//     tier-1 latch -> STALL_LATCHED_HELD (whole drive stopped, marker written);
//  2. a clear with a stale expected_reason is refused, nothing changes;
//  3. the clear, with the command still HELD (D2): accepted, all four written
//     wheel commands follow the held request on the cycle it is applied,
//     steering bit-identical, marker gone, drive_fault republished with
//     active=false and the clear fields -- also to a subscriber started after;
//     wheel commands are compared within the FR-14 regulator's authority
//     (enabled in the active config), steering exactly;
//  4. FR still silent: it re-trips after stall_silent_window_sec and, inside
//     stall_escalation_window_sec of the clear, re-latches at once as
//     STALL_RETRIP (OP-49 (a)); clear_count unchanged, marker rewritten;
//  5. a second clear (D3: no limit), then an unload/reload does NOT re-adopt.
// Thresholds come from the active ros2_controllers.yaml (active_config.hpp).

#include <cmath>
#include <filesystem>

#include "drive_fault_bench.hpp"

using drive_fault_bench::Bench;

TEST(SwerveControllerDriveFaultClear, ServiceClearReleasesTheDriveAndDetectionReArms)
{
  const auto cfg = active_config::drive_fault_config();
  const auto stall = active_config::stall_config();
  const auto params = active_config::swerve_parameters();
  const double radius = params["wheel_radius"].as<double>();
  const double wheel =
    stall.silent_min_command_rad_s + 0.25 * (stall.min_command_rad_s - stall.silent_min_command_rad_s);
  const double vx = wheel * radius;
  ASSERT_GT(cfg.stall_escalation_window_sec, stall.silent_window_sec + 0.5);

  Bench bench;
  bench.spin_controller_in_background = true;
  auto controller = bench.load();
  bench.set_provenance({3, 3, 3, 3});
  bench.settle();
  const std::array<bool, 4> none{false, false, false, false};
  const std::array<bool, 4> fr_silent{false, true, false, false};

  // ---- reference: the same held command, no fault.
  bench.drive_frozen(*controller, 1.0, vx, none);
  ASSERT_TRUE(bench.wheels_follow(wheel));
  const auto ref_steer = bench.steer_commands();

  // ---- 1. provoke STALL_LATCHED_HELD.
  bench.drive_frozen(*controller, stall.silent_window_sec + 0.3, vx, fr_silent);
  ASSERT_EQ(bench.wheel_command(1), 0.0);
  bench.drive_frozen(*controller, cfg.stall_escalation_latched_sec + 0.5, vx, fr_silent);
  ASSERT_TRUE(bench.last_fault_.active);
  ASSERT_EQ(bench.last_fault_.reason, DriveFault::REASON_STALL_LATCHED_HELD);
  EXPECT_EQ(bench.last_fault_.clear_count, 0u);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  EXPECT_EQ(bench.steer_commands(), ref_steer) << "steering untouched by the fault";
  ASSERT_TRUE(std::filesystem::exists(bench.marker_path));

  // ---- 2. stale dialog: refused, nothing changed.
  auto refused = bench.call_clear_service(
    *controller, "web_teleop", DriveFault::REASON_STALL_RETRIP, vx, none);
  EXPECT_FALSE(refused.success);
  EXPECT_EQ(refused.reason, DriveFault::REASON_STALL_LATCHED_HELD);
  EXPECT_NE(refused.message.find("expected reason STALL_RETRIP"), std::string::npos)
    << refused.message;
  bench.drive_frozen(*controller, 0.3, vx, none);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  EXPECT_TRUE(std::filesystem::exists(bench.marker_path));

  // ---- 2b. F-58: a request for ANOTHER fault (identity) is refused.
  const uint32_t shown = bench.last_fault_.fault_id;
  ASSERT_GT(shown, 0u) << "a latched fault carries an identity";
  auto other = bench.call_clear_service(
    *controller, "web_teleop", DriveFault::REASON_STALL_LATCHED_HELD, vx, none, shown + 1);
  EXPECT_FALSE(other.success);
  EXPECT_NE(other.message.find("fault changed, review again"), std::string::npos)
    << other.message;
  bench.drive_frozen(*controller, 0.2, vx, none);
  EXPECT_TRUE(bench.all_wheels_exactly_zero());

  // ---- 3. the clear, command HELD, block removed, with the identity shown.
  auto cleared = bench.call_clear_service(
    *controller, "cli", DriveFault::REASON_STALL_LATCHED_HELD, vx, none, shown);
  ASSERT_TRUE(cleared.success) << cleared.message;
  EXPECT_EQ(cleared.reason, DriveFault::REASON_STALL_LATCHED_HELD);
  EXPECT_NE(cleared.message.find("CLEARED"), std::string::npos) << cleared.message;
  EXPECT_GE(bench.cycles_until_answer, 1) << "applied by update(), not by the callback";
  EXPECT_TRUE(bench.wheels_follow(wheel)) << "D2: the held command drives at once";
  EXPECT_EQ(bench.steer_commands(), ref_steer);
  EXPECT_FALSE(std::filesystem::exists(bench.marker_path));
  bench.drive_frozen(*controller, 0.3, vx, none);
  EXPECT_TRUE(bench.wheels_follow(wheel));
  bench.settle();
  EXPECT_FALSE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.fault_id, 0u);
  EXPECT_EQ(bench.last_fault_.clear_count, 1u);
  EXPECT_EQ(bench.last_fault_.last_cleared_reason, DriveFault::REASON_STALL_LATCHED_HELD);
  EXPECT_EQ(bench.last_fault_.last_clear_requester, "cli");
  EXPECT_GT(rclcpp::Time(bench.last_fault_.last_clear_stamp).nanoseconds(), 0);
  {
    DriveFault late;
    int late_count = 0;
    auto late_sub = bench.node_->create_subscription<DriveFault>(
      "/swerve_controller/drive_fault", drive_fault_bench::latched_qos(),
      [&](const DriveFault::SharedPtr msg) {late = *msg; ++late_count;});
    bench.settle();
    ASSERT_GT(late_count, 0) << "a subscriber started after the clear must learn it";
    EXPECT_FALSE(late.active);
    EXPECT_EQ(late.clear_count, 1u);
  }

  // ---- 4. FR still silent: re-arms, re-trips, re-latches as STALL_RETRIP.
  // Cycles run slower than kDt in wall time, so stay well inside the window.
  bench.drive_frozen(*controller, 0.5 * stall.silent_window_sec, vx, fr_silent);
  EXPECT_FALSE(bench.last_fault_.active) << "detection restarts with an empty window";
  EXPECT_NE(bench.wheel_command(1), 0.0) << "E4: the silent wheel is driven meanwhile";
  bench.drive_frozen(*controller, 0.5 * stall.silent_window_sec + 1.0, vx, fr_silent);
  bench.settle();
  ASSERT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.reason, DriveFault::REASON_STALL_RETRIP);
  EXPECT_EQ(bench.last_fault_.wheel_index, 1);
  EXPECT_GT(bench.last_fault_.fault_id, shown) << "a new latch is a new fault";
  EXPECT_EQ(bench.last_fault_.clear_count, 1u) << "a re-latch is not a clear";
  // The dialog that showed the OLD fault cannot clear the new one.
  auto stale = bench.call_clear_service(
    *controller, "web_teleop", DriveFault::REASON_STALL_RETRIP, vx, fr_silent, shown);
  EXPECT_FALSE(stale.success);
  EXPECT_NE(stale.message.find("fault changed"), std::string::npos) << stale.message;
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  EXPECT_TRUE(std::filesystem::exists(bench.marker_path)) << "a new fault writes the marker again";

  // ---- 5. D3: clear again; a reload does not re-adopt the cleared fault.
  auto again = bench.call_clear_service(*controller, "", 0, vx, none);
  ASSERT_TRUE(again.success) << again.message;
  bench.settle();
  EXPECT_EQ(bench.last_fault_.clear_count, 2u);
  EXPECT_EQ(bench.last_fault_.last_clear_requester, "unspecified");
  bench.unload(controller);
  const int before_reload = bench.fault_messages_;
  controller = bench.load();
  bench.settle();
  EXPECT_GT(bench.fault_messages_, before_reload);
  EXPECT_FALSE(bench.last_fault_.active) << "C4: a reload must not re-adopt a cleared fault";
  EXPECT_EQ(bench.last_fault_.clear_count, 2u) << "since the PROCESS started";
  bench.drive_frozen(*controller, 1.0, vx, none);
  EXPECT_TRUE(bench.wheels_follow(wheel));

  // ---- an inactive controller refuses.
  bench.deactivate(*controller);
  auto inactive = controller->request_drive_fault_clear("cli", 0);
  EXPECT_FALSE(inactive.success);
  EXPECT_NE(inactive.message.find("not active"), std::string::npos) << inactive.message;
  bench.activate(*controller);

  bench.unload(controller);
  bench.remove_marker_dir();
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
