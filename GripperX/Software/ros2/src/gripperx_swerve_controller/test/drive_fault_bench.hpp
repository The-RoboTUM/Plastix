// The whole-drive fault through the REAL controller: SwerveController loaded
// with the ACTIVE ros2_controllers.yaml (as ros2_control_node is), its real
// lifecycle (configure / activate / deactivate / activate), the real
// choke point write_wheel_commands(), the latched provenance topic and the
// latched fault topic. No controller_manager, no hardware: the command and
// state interfaces are plain in-memory handles owned by this test.
//
// It runs as ONE test on purpose. The fault is latched process-wide (that is
// the feature), so a second test in this process would start with it set.
//
// Real time: the controller stamps /cmd_vel with its node clock and judges
// staleness against update()'s `time`, so this loop runs at ~30 Hz in wall
// time (a few seconds in total).

#ifndef GRIPPERX_SWERVE_CONTROLLER__TEST__DRIVE_FAULT_BENCH_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__TEST__DRIVE_FAULT_BENCH_HPP_

#include <gtest/gtest.h>

#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "active_config.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gripperx_control_msgs/msg/drive_fault.hpp"
#include "gripperx_control_msgs/srv/clear_drive_fault.hpp"
#include "gripperx_swerve_controller/swerve_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using gripperx_control_msgs::msg::DriveFault;
using gripperx_control_msgs::srv::ClearDriveFault;
using gripperx_swerve_controller::SwerveController;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;

namespace drive_fault_bench
{
constexpr double kDt = 1.0 / 30.0;

rclcpp::QoS latched_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  return qos;
}

/// The robot side: in-memory interfaces, plus a ROS node that publishes the
/// latched provenance and /cmd_vel and listens to the latched fault.
class Bench
{
public:
  Bench()
  {
    const auto params = active_config::swerve_parameters();
    steer_ = params["steering_joint_names"].as<std::vector<std::string>>();
    wheel_ = params["wheel_joint_names"].as<std::vector<std::string>>();
    for (const auto & j : steer_) {
      cmd_[j + "/position"] =
        std::make_shared<CommandInterface>(j, hardware_interface::HW_IF_POSITION);
      state_[j + "/position"] =
        std::make_shared<StateInterface>(j, hardware_interface::HW_IF_POSITION);
    }
    for (const auto & j : wheel_) {
      cmd_[j + "/velocity"] =
        std::make_shared<CommandInterface>(j, hardware_interface::HW_IF_VELOCITY);
      state_[j + "/velocity"] =
        std::make_shared<StateInterface>(j, hardware_interface::HW_IF_VELOCITY);
      state_[j + "/position"] =
        std::make_shared<StateInterface>(j, hardware_interface::HW_IF_POSITION);
    }
    for (auto & kv : cmd_) {
      EXPECT_TRUE(kv.second->set_value(0.0));
    }
    for (auto & kv : state_) {
      EXPECT_TRUE(kv.second->set_value(0.0));
    }

    node_ = std::make_shared<rclcpp::Node>("drive_fault_bench");
    provenance_pub_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/hw/wheel_feedback_valid", latched_qos());
    rclcpp::QoS command_qos(rclcpp::KeepLast(10));
    command_qos.reliable();
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", command_qos);
    fault_sub_ = node_->create_subscription<DriveFault>(
      "/swerve_controller/drive_fault", latched_qos(),
      [this](const DriveFault::SharedPtr msg) {
        last_fault_ = *msg;
        ++fault_messages_;
      });
    clear_client_ = node_->create_client<ClearDriveFault>("/swerve_controller/clear_drive_fault");
    executor_.add_node(node_);
  }

  ~Bench()
  {
    stop_background();
  }

  /// The operator-clear service blocks until update() has applied the
  /// request, so its callback must not run on the thread that calls update():
  /// with this set, load() spins the CONTROLLER node on a multi-threaded
  /// executor in a background thread (as controller_manager does), and the
  /// bench node stays on executor_ in the test thread.
  bool spin_controller_in_background{false};

  /// Where the controller writes its crash-surviving marker in this test:
  /// never /run/gripperx on the developer's machine.
  std::string marker_path{
    std::string("/tmp/gripperx_drive_fault_test_") + std::to_string(::getpid()) + "/drive_fault"};

  std::unique_ptr<SwerveController> load()
  {
    auto controller = std::make_unique<SwerveController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "swerve_controller";
    params.robot_description = "";
    params.update_rate = 30;
    params.controller_manager_update_rate = 30;
    params.node_options = controller->define_custom_node_options();
    params.node_options.arguments(
      {"--ros-args", "--params-file", active_config::controllers_yaml_path(), "-p",
        "drive_fault_marker_path:=" + marker_path});
    EXPECT_EQ(controller->init(params), controller_interface::return_type::OK);
    EXPECT_EQ(
      controller->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    std::vector<hardware_interface::LoanedCommandInterface> commands;
    for (const auto & name : controller->command_interface_configuration().names) {
      commands.emplace_back(cmd_.at(name), nullptr);
    }
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (const auto & name : controller->state_interface_configuration().names) {
      states.emplace_back(state_.at(name));
    }
    controller->assign_interfaces(std::move(commands), std::move(states));
    if (spin_controller_in_background) {
      background_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 2);
      background_->add_node(controller->get_node()->get_node_base_interface());
      background_thread_ = std::thread([this]() { background_->spin(); });
    } else {
      executor_.add_node(controller->get_node()->get_node_base_interface());
    }
    activate(*controller);
    return controller;
  }

  void stop_background()
  {
    if (background_) {
      background_->cancel();
      if (background_thread_.joinable()) {
        background_thread_.join();
      }
      background_.reset();
    }
  }

  void activate(SwerveController & c)
  {
    EXPECT_EQ(
      c.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  void deactivate(SwerveController & c)
  {
    EXPECT_EQ(
      c.get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }

  void unload(std::unique_ptr<SwerveController> & c)
  {
    deactivate(*c);
    if (background_) {
      stop_background();
    } else {
      executor_.remove_node(c->get_node()->get_node_base_interface());
    }
    c->release_interfaces();
    c.reset();
  }

  void set_provenance(const std::vector<int> & codes)
  {
    std_msgs::msg::Int32MultiArray msg;
    msg.data.assign(codes.begin(), codes.end());
    provenance_pub_->publish(msg);
  }

  /// Drive for `seconds` of wall time at body speed vx; FL/FR/BL positions
  /// follow the command (`counting`), BR's never moves when `br_dead`.
  void drive(SwerveController & c, double seconds, double vx, bool br_dead)
  {
    drive_frozen(c, seconds, vx, {false, false, false, br_dead});
  }

  /// As drive(), with any subset of wheel POSITIONS frozen (encoder not moving).
  void drive_frozen(
    SwerveController & c, double seconds, double vx, const std::array<bool, 4> & frozen)
  {
    const int cycles = static_cast<int>(seconds / kDt);
    for (int k = 0; k < cycles; ++k) {
      cycle(c, vx, frozen);
    }
    executor_.spin_some(std::chrono::milliseconds(20));
  }

  /// ONE controller cycle (publish, spin, feed the wheel states, update()).
  void cycle(SwerveController & c, double vx, const std::array<bool, 4> & frozen)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx;
    cmd_vel_pub_->publish(twist);
    executor_.spin_some(std::chrono::milliseconds(5));
    for (std::size_t i = 0; i < wheel_.size(); ++i) {
      const double commanded = wheel_command(i);
      const bool moves = !frozen[i];
      auto & position = state_.at(wheel_[i] + "/position");
      const double p = position->get_optional().value_or(0.0);
      EXPECT_TRUE(position->set_value(p + (moves ? commanded * kDt : 0.0)));
      EXPECT_TRUE(state_.at(wheel_[i] + "/velocity")->set_value(moves ? commanded : 0.0));
    }
    const auto now = c.get_node()->get_clock()->now();
    EXPECT_EQ(
      c.update(now, rclcpp::Duration::from_seconds(kDt)), controller_interface::return_type::OK);
    max_br_command_ = std::max(max_br_command_, std::fabs(wheel_command(3)));
    std::this_thread::sleep_for(std::chrono::duration<double>(kDt));
  }

  /// Operator clear through request_drive_fault_clear() (the service body) on
  /// a second thread, while this thread keeps cycling the controller under
  /// the held command `vx` -- as on the robot, where the service thread and
  /// the control loop run side by side. Returns the outcome and leaves
  /// `cycles_until_answer` set.
  SwerveController::DriveFaultClearOutcome clear(
    SwerveController & c, const std::string & requester, int expected_reason, double vx,
    const std::array<bool, 4> & frozen = {false, false, false, false},
    uint32_t expected_fault_id = 0)
  {
    auto future = std::async(
      std::launch::async, [&c, requester, expected_reason, expected_fault_id]() {
        return c.request_drive_fault_clear(requester, expected_reason, expected_fault_id);
      });
    cycles_until_answer = 0;
    while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      cycle(c, vx, frozen);
      ++cycles_until_answer;
      if (cycles_until_answer > 300) {
        ADD_FAILURE() << "no answer from request_drive_fault_clear";
        break;
      }
    }
    return future.get();
  }

  /// The same through the real ROS service (needs spin_controller_in_background).
  ClearDriveFault::Response call_clear_service(
    SwerveController & c, const std::string & requester, uint8_t expected_reason, double vx,
    const std::array<bool, 4> & frozen = {false, false, false, false},
    uint32_t expected_fault_id = 0)
  {
    EXPECT_TRUE(clear_client_->wait_for_service(std::chrono::seconds(5)));
    auto request = std::make_shared<ClearDriveFault::Request>();
    request->requester = requester;
    request->expected_reason = expected_reason;
    request->expected_fault_id = expected_fault_id;
    auto future = clear_client_->async_send_request(request);
    cycles_until_answer = 0;
    while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      cycle(c, vx, frozen);
      ++cycles_until_answer;
      if (cycles_until_answer > 300) {
        ADD_FAILURE() << "no answer from the clear service";
        return ClearDriveFault::Response();
      }
    }
    return *future.get();
  }

  std::array<double, 4> wheel_commands() const
  {
    return {wheel_command(0), wheel_command(1), wheel_command(2), wheel_command(3)};
  }

  /// Whether every written wheel command follows `requested` (rad/s, same on
  /// all four for a straight drive): non-zero, and within the FR-14 wheel
  /// regulator's authority (wheel_regulator_max_correction_fraction of the
  /// ACTIVE config; zero when the regulator is disabled there). The regulator
  /// is closed-loop on simulated wheels in wall time, so exact equality with a
  /// reference run is not reproducible while it is enabled.
  ::testing::AssertionResult wheels_follow(double requested) const
  {
    const auto reg = active_config::regulator_config();
    const double bound = (reg.enabled ? reg.max_correction_fraction : 0.0) *
      std::fabs(requested) + 1e-9;
    for (std::size_t i = 0; i < wheel_.size(); ++i) {
      const double c = wheel_command(i);
      if (c == 0.0 || std::fabs(c - requested) > bound) {
        return ::testing::AssertionFailure() << "wheel " << wheel_[i] << " wrote " << c
                                             << ", requested " << requested << " +- " << bound;
      }
    }
    return ::testing::AssertionSuccess();
  }

  std::array<double, 4> steer_commands() const
  {
    std::array<double, 4> out{};
    for (std::size_t i = 0; i < steer_.size() && i < 4; ++i) {
      out[i] = cmd_.at(steer_[i] + "/position")->get_optional().value_or(-999.0);
    }
    return out;
  }

  /// Write a crash-surviving marker as a previous process would have.
  void write_marker(const gripperx_swerve_controller::DriveFaultRecord & record)
  {
    std::filesystem::create_directories(std::filesystem::path(marker_path).parent_path());
    std::ofstream out(marker_path);
    out << gripperx_swerve_controller::serialize_drive_fault(record);
  }

  void remove_marker_dir()
  {
    std::error_code ec;
    std::filesystem::remove_all(std::filesystem::path(marker_path).parent_path(), ec);
  }

  int cycles_until_answer{0};

  double wheel_command(std::size_t i) const
  {
    return cmd_.at(wheel_[i] + "/velocity")->get_optional().value_or(-999.0);
  }

  bool all_wheels_exactly_zero() const
  {
    for (std::size_t i = 0; i < wheel_.size(); ++i) {
      if (wheel_command(i) != 0.0) {
        return false;
      }
    }
    return true;
  }

  void settle()
  {
    for (int k = 0; k < 20; ++k) {
      executor_.spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::vector<std::string> steer_, wheel_;
  std::map<std::string, CommandInterface::SharedPtr> cmd_;
  std::map<std::string, StateInterface::SharedPtr> state_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr provenance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<DriveFault>::SharedPtr fault_sub_;
  rclcpp::Client<ClearDriveFault>::SharedPtr clear_client_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> background_;
  std::thread background_thread_;
  DriveFault last_fault_;
  int fault_messages_{0};
  double max_br_command_{0.0};
};
}  // namespace drive_fault_bench

/// Every bench test publishes /cmd_vel: FORCED onto a twin domain, discovery
/// limited to this host -- never a graph shared with the robot (domain 20).
inline int run_bench_tests(int argc, char ** argv)
{
  setenv("ROS_DOMAIN_ID", "220", 1);
  setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

#endif  // GRIPPERX_SWERVE_CONTROLLER__TEST__DRIVE_FAULT_BENCH_HPP_
