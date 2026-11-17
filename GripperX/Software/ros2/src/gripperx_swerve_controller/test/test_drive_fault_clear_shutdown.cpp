// F-57 (safety audit of the SR-18 operator clear, 2026-11-09): a clear call
// that is blocked waiting for the control loop must not outlive the
// controller. Unloading (cleanup) or destroying the controller while a call
// is in flight must not crash; the caller gets a "shutting down" rejection
// and the fault stays latched (process-wide and in the marker). update() is
// deliberately NOT run while the call is pending, so it stays blocked.

#include <chrono>
#include <filesystem>
#include <thread>

#include "drive_fault_bench.hpp"

using drive_fault_bench::Bench;
using gripperx_swerve_controller::DriveFaultRecord;

namespace
{
ClearDriveFault::Response wait_answer(
  Bench & bench, rclcpp::Client<ClearDriveFault>::SharedFuture future)
{
  for (int k = 0; k < 200; ++k) {
    bench.executor_.spin_some(std::chrono::milliseconds(10));
    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      return *future.get();
    }
  }
  ADD_FAILURE() << "no answer from the clear service";
  return ClearDriveFault::Response();
}
}  // namespace

TEST(SwerveControllerDriveFaultClearShutdown, AnInFlightClearIsRejectedOnCleanupAndDestruction)
{
  Bench bench;
  bench.spin_controller_in_background = true;
  bench.remove_marker_dir();
  DriveFaultRecord record;
  record.active = true;
  record.reason = DriveFault::REASON_STALL_LATCHED_HELD;
  record.wheel = 1;
  record.provenance = 3;
  bench.write_marker(record);

  // ---- 1. cleanup (what controller_manager does on unload) during the call.
  auto controller = bench.load();
  bench.settle();
  ASSERT_TRUE(bench.last_fault_.active);
  const uint32_t id = bench.last_fault_.fault_id;
  ASSERT_TRUE(bench.clear_client_->wait_for_service(std::chrono::seconds(5)));
  auto request = std::make_shared<ClearDriveFault::Request>();
  request->requester = "cli";
  auto future = bench.clear_client_->async_send_request(request).future.share();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));   // callback now blocked
  bench.deactivate(*controller);
  EXPECT_EQ(
    controller->get_node()->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  auto answer = wait_answer(bench, future);
  EXPECT_FALSE(answer.success);
  EXPECT_NE(answer.message.find("shutting down"), std::string::npos) << answer.message;
  bench.stop_background();
  controller->release_interfaces();
  controller.reset();
  EXPECT_TRUE(std::filesystem::exists(bench.marker_path));

  // Still latched: a new instance re-adopts the SAME fault.
  controller = bench.load();
  bench.settle();
  EXPECT_TRUE(bench.last_fault_.active);
  EXPECT_EQ(bench.last_fault_.fault_id, id);

  // ---- 2. destruction without cleanup, executor still spinning.
  auto future2 = bench.clear_client_->async_send_request(request).future.share();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  bench.deactivate(*controller);
  bench.background_->remove_node(controller->get_node()->get_node_base_interface());
  controller->release_interfaces();
  controller.reset();                                            // destructor
  auto answer2 = wait_answer(bench, future2);
  EXPECT_FALSE(answer2.success);
  EXPECT_NE(answer2.message.find("shutting down"), std::string::npos) << answer2.message;
  bench.stop_background();
  EXPECT_TRUE(std::filesystem::exists(bench.marker_path));

  controller = bench.load();
  bench.settle();
  EXPECT_TRUE(bench.last_fault_.active) << "the fault stays latched";
  bench.drive_frozen(*controller, 0.3, 0.3, {false, false, false, false});
  EXPECT_TRUE(bench.all_wheels_exactly_zero());
  bench.unload(controller);
  bench.remove_marker_dir();
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
