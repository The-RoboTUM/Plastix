// ack_probe — measurement harness for the nav2 BtActionNode goal-acknowledgement window.
//
// Two modes:
//   probe  : faithfully re-implements nav2_behavior_tree::BtActionNode's goal-ack loop
//            (dedicated MutuallyExclusive callback group + its own SingleThreadedExecutor,
//            WallRate(bt_loop_duration), spin window = min(remaining, bt_loop_duration/2),
//            budget = server_timeout measured on node->now()).  After the emulated budget
//            expires it KEEPS spinning so the TRUE ack latency is recorded too.
//   stub   : a nav2_util::SimpleActionServer for the same action type that accepts and
//            immediately succeeds — the transport/scheduling floor, with no planner work.
//
// Flags: --mode probe|stub  --action <name>  --count N  --period-ms P
//        --bt-loop-ms L  --server-timeout-ms T  --goal-x X --goal-y Y --frame F
//        --use-sim-time   budget measured on /clock, as bt_navigator does in the twin
//        --continuous     spin without interruption: the TRUE server+transport latency
//        --warm           keep the executor spinning between goals (FollowPath-like)
//                         instead of leaving it cold (ComputePathToPose-like)
//
// Output: one JSON object per line on stdout (jsonl).
//
// This tool sends ComputePathToPose goals only.  Planning commands no motion.
//
// NOT BUILT BY THIS PACKAGE, deliberately.  gripperx_planning is an ament_python
// package and does not depend on rclcpp / rclcpp_action / nav2_util; adding those
// for a diagnostic would pull a C++ toolchain into the robot's dependency closure.
// It lives here so the measurement behind the internal acknowledgement-window
// record of 2026-09-23 (tracked internally, not part of this repository) can be
// repeated.  Build it in a scratch ament_cmake package:
//   package.xml : depend on rclcpp, rclcpp_action, nav2_msgs, nav2_util
//   CMakeLists  : add_executable(ack_probe src/ack_probe.cpp)
//                 ament_target_dependencies(ack_probe rclcpp rclcpp_action
//                                           nav2_msgs nav2_util)
// and source Software/ros2/scripts/sim_env_nav2.sh first — twin domain 220 (SR-8).

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"

using Action = nav2_msgs::action::ComputePathToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
using namespace std::chrono_literals;

static double arg_d(int argc, char ** argv, const std::string & key, double def)
{
  for (int i = 1; i + 1 < argc; ++i) {if (key == argv[i]) {return std::atof(argv[i + 1]);}}
  return def;
}
static std::string arg_s(int argc, char ** argv, const std::string & key, const std::string & def)
{
  for (int i = 1; i + 1 < argc; ++i) {if (key == argv[i]) {return argv[i + 1];}}
  return def;
}
static bool arg_f(int argc, char ** argv, const std::string & key)
{
  for (int i = 1; i < argc; ++i) {if (key == argv[i]) {return true;}}
  return false;
}

static double steady_ms()
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ---------------------------------------------------------------------------- stub server
static int run_stub(int argc, char ** argv)
{
  const std::string action_name = arg_s(argc, argv, "--action", "stub_compute_path_to_pose");
  auto node = std::make_shared<rclcpp::Node>("ack_probe_stub");
  std::shared_ptr<nav2_util::SimpleActionServer<Action>> server;
  server = std::make_shared<nav2_util::SimpleActionServer<Action>>(
    node, action_name,
    [&server]() {
      auto result = std::make_shared<Action::Result>();
      result->error_code = 0;
      server->succeeded_current(result);
    },
    nullptr, 500ms, true /* spin_thread — exactly as planner_server does */);
  server->activate();
  RCLCPP_INFO(node->get_logger(), "stub action server up on '%s'", action_name.c_str());
  rclcpp::spin(node);
  return 0;
}

// ---------------------------------------------------------------------------- probe client
struct Record
{
  int seq;
  bool timed_out;          // would BtActionNode have declared an ack timeout?
  double notice_node_ms;   // node-clock elapsed at which the ack was noticed (-1 if timed out)
  double true_wall_ms;     // steady-clock send -> future ready (the honest latency)
  double true_node_ms;     // node-clock send -> future ready
  int ticks;               // BT ticks consumed before the ack was noticed / budget expired
  double max_tick_gap_ms;  // largest wall gap between two consecutive emulated BT ticks
  double max_clock_step_ms;// largest node-clock step seen between two consecutive ticks
  double verdict_wall_ms;  // wall time from send to the emulated BT verdict (-1 if none)
  bool rejected;
};

static int run_probe(int argc, char ** argv)
{
  const std::string action_name = arg_s(argc, argv, "--action", "compute_path_to_pose");
  const int count = static_cast<int>(arg_d(argc, argv, "--count", 60));
  const int period_ms = static_cast<int>(arg_d(argc, argv, "--period-ms", 1000));
  const int bt_loop_ms = static_cast<int>(arg_d(argc, argv, "--bt-loop-ms", 10));
  const int budget_ms = static_cast<int>(arg_d(argc, argv, "--server-timeout-ms", 20));
  const double gx = arg_d(argc, argv, "--goal-x", 0.0);
  const double gy = arg_d(argc, argv, "--goal-y", 0.0);
  const std::string frame = arg_s(argc, argv, "--frame", "map");
  const bool sim_time = arg_f(argc, argv, "--use-sim-time");

  rclcpp::NodeOptions opts;
  opts.parameter_overrides({rclcpp::Parameter("use_sim_time", sim_time)});
  auto node = std::make_shared<rclcpp::Node>("ack_probe", opts);

  // --- exactly BtActionNode's plumbing -------------------------------------
  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_callback_group(cb_group, node->get_node_base_interface());
  auto client = rclcpp_action::create_client<Action>(node, action_name, cb_group);

  if (!client->wait_for_action_server(10s)) {
    fprintf(stderr, "action server '%s' not available\n", action_name.c_str());
    return 2;
  }
  // BtActionNode's max_timeout_ = bt_loop_duration * 0.5
  const auto max_timeout = std::chrono::milliseconds(bt_loop_ms / 2);

  // Wait for a usable clock before the first measurement (sim time starts at 0).
  if (sim_time) {
    auto t0 = steady_ms();
    while (node->now().nanoseconds() == 0 && steady_ms() - t0 < 20000) {
      exec.spin_some();
      rclcpp::sleep_for(50ms);
    }
  }

  for (int seq = 0; seq < count && rclcpp::ok(); ++seq) {
    Action::Goal goal;
    goal.goal.header.frame_id = frame;
    goal.goal.header.stamp = node->now();
    goal.goal.pose.position.x = gx;
    goal.goal.pose.position.y = gy;
    goal.goal.pose.orientation.w = 1.0;
    goal.use_start = false;
    goal.planner_id = "";

    bool result_available = false;
    auto send_opts = rclcpp_action::Client<Action>::SendGoalOptions();
    send_opts.result_callback = [&](const GoalHandle::WrappedResult &) {result_available = true;};

    auto fut = client->async_send_goal(goal, send_opts);
    const double t_send_wall = steady_ms();
    const rclcpp::Time t_send_node = node->now();

    Record rec{seq, false, -1.0, -1.0, -1.0, 0, 0.0, 0.0, -1.0, false};

    // ---- phase 1: BtActionNode emulation --------------------------------
    // --continuous skips the emulation and spins without interruption, which
    // measures the TRUE server+transport ack latency (no BT sampling penalty).
    static const bool continuous = arg_f(argc, argv, "--continuous");
    rclcpp::WallRate loop{std::chrono::milliseconds(bt_loop_ms)};
    bool have_handle = false;
    if (continuous) {
      auto r = exec.spin_until_future_complete(fut, 5s);
      if (r == rclcpp::FutureReturnCode::SUCCESS) {
        rec.notice_node_ms = (node->now() - t_send_node).seconds() * 1000.0;
        rec.true_wall_ms = steady_ms() - t_send_wall;
        rec.true_node_ms = rec.notice_node_ms;
        rec.ticks = 1;
      }
      have_handle = (rec.true_wall_ms >= 0.0);
    } else {
    double last_tick_wall = t_send_wall;
    rclcpp::Time last_tick_node = t_send_node;

    while (rclcpp::ok()) {
      const double now_wall = steady_ms();
      const rclcpp::Time now_node = node->now();
      if (rec.ticks > 0) {
        rec.max_tick_gap_ms = std::max(rec.max_tick_gap_ms, now_wall - last_tick_wall);
        rec.max_clock_step_ms =
          std::max(rec.max_clock_step_ms, (now_node - last_tick_node).seconds() * 1000.0);
      }
      last_tick_wall = now_wall;
      last_tick_node = now_node;
      rec.ticks++;

      const double elapsed_node_ms = (now_node - t_send_node).seconds() * 1000.0;
      const double remaining_ms = budget_ms - elapsed_node_ms;
      if (remaining_ms <= 0.0) {          // BtActionNode: reset(), WARN, return FAILURE
        rec.timed_out = true;
        rec.verdict_wall_ms = now_wall - t_send_wall;
        break;
      }
      auto spin_for = std::chrono::milliseconds(
        static_cast<long>(std::min<double>(remaining_ms, max_timeout.count())));
      if (spin_for.count() <= 0) {spin_for = 1ms;}
      auto r = exec.spin_until_future_complete(fut, spin_for);
      if (r == rclcpp::FutureReturnCode::SUCCESS) {
        rec.notice_node_ms = (node->now() - t_send_node).seconds() * 1000.0;
        rec.true_wall_ms = steady_ms() - t_send_wall;
        rec.true_node_ms = rec.notice_node_ms;
        have_handle = true;
        break;
      }
      loop.sleep();
    }
    }

    // ---- phase 2: keep spinning to learn the TRUE ack latency ------------
    if (!have_handle) {
      auto r = exec.spin_until_future_complete(fut, 5s);
      if (r == rclcpp::FutureReturnCode::SUCCESS) {
        rec.true_wall_ms = steady_ms() - t_send_wall;
        rec.true_node_ms = (node->now() - t_send_node).seconds() * 1000.0;
        have_handle = true;
      }
    }
    if (have_handle && !fut.get()) {rec.rejected = true;}

    // drain the result so the server is not left with an open goal
    auto t_drain = steady_ms();
    while (!result_available && steady_ms() - t_drain < 5000.0 && rclcpp::ok()) {
      exec.spin_some();
      rclcpp::sleep_for(2ms);
    }

    printf(
      "{\"seq\":%d,\"timed_out\":%s,\"notice_node_ms\":%.3f,\"true_wall_ms\":%.3f,"
      "\"true_node_ms\":%.3f,\"ticks\":%d,\"max_tick_gap_ms\":%.3f,"
      "\"max_clock_step_ms\":%.3f,\"verdict_wall_ms\":%.3f,\"rejected\":%s}\n",
      rec.seq, rec.timed_out ? "true" : "false", rec.notice_node_ms, rec.true_wall_ms,
      rec.true_node_ms, rec.ticks, rec.max_tick_gap_ms, rec.max_clock_step_ms,
      rec.verdict_wall_ms, rec.rejected ? "true" : "false");
    fflush(stdout);

    // --warm keeps this node's executor spinning between goals, the way
    // FollowPath's does (it calls spin_some on every BT tick while it waits for
    // a result).  Without it the executor is cold for the whole inter-goal gap,
    // the way ComputePathToPose's is between two 1 Hz replans.
    static const bool warm = arg_f(argc, argv, "--warm");
    if (warm) {
      auto t_w = steady_ms();
      while (steady_ms() - t_w < period_ms && rclcpp::ok()) {
        exec.spin_some();
        rclcpp::sleep_for(1ms);
      }
    } else {
      rclcpp::sleep_for(std::chrono::milliseconds(period_ms));
    }
  }
  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = arg_s(argc, argv, "--mode", "probe");
  int rc = (mode == "stub") ? run_stub(argc, argv) : run_probe(argc, argv);
  rclcpp::shutdown();
  return rc;
}
