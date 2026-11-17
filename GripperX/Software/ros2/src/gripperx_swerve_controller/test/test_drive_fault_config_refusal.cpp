// The threshold rules of 2026-10-22, through the REAL controller's parameter
// path: SwerveController loaded with the ACTIVE ros2_controllers.yaml plus one
// override, exactly as ros2_control_node would load it. Unit tests of the
// validate functions cannot show that on_init actually calls them.
//   * encoder_confirm_min_command_rad_s above stall_min_command_rad_s: refused
//     (the displacement criterion would judge where a healthy wheel is not even
//     expected to count).
//   * encoder_confirm_min_command_rad_s at or below stall_release_command_rad_s:
//     refused (no OP-25 hysteresis for a zero-count latch).
//   * wheel_regulator_floor_rad_s below stall_min_command_rad_s: refused
//     (the regulator may only add authority where HWR-30a is watching).
//   * a non-finite window (.nan / .inf in a parameter source): refused (audit
//     2026-10-29; the parameter path does deliver them to the validators).
// No command is written and no update() is run.

#include <sstream>
#include <string>
#include <vector>

#include "drive_fault_bench.hpp"

namespace
{
std::string as_param(double value)
{
  std::ostringstream out;
  out.precision(17);
  out << value;
  std::string text = out.str();
  if (text.find_first_of(".eE") == std::string::npos) {
    text += ".0";  // keep it a double parameter, not an integer one
  }
  return text;
}

/// on_init's verdict for the active YAML with `overrides` (name:=value) on top.
controller_interface::return_type load_with(const std::vector<std::string> & overrides)
{
  SwerveController controller;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "swerve_controller";
  params.robot_description = "";
  params.update_rate = 30;
  params.controller_manager_update_rate = 30;
  params.node_options = controller.define_custom_node_options();
  std::vector<std::string> args{
    "--ros-args", "--params-file", active_config::controllers_yaml_path(), "-p",
    // Never /run/gripperx on the developer's machine; nothing trips here, so
    // nothing is ever written to it.
    "drive_fault_marker_path:=/tmp/gripperx_threshold_rules_" + std::to_string(::getpid()) +
    "/drive_fault"};
  for (const auto & o : overrides) {
    args.push_back("-p");
    args.push_back(o);
  }
  params.node_options.arguments(args);
  return controller.init(params);
}
}  // namespace

TEST(SwerveControllerThresholdRules, ActiveConfigLoads)
{
  EXPECT_EQ(load_with({}), controller_interface::return_type::OK);
}

TEST(SwerveControllerThresholdRules, EncoderThresholdAboveStallThresholdIsRefused)
{
  const auto stall = active_config::stall_config();
  const double above = stall.min_command_rad_s + 0.5;
  EXPECT_NE(
    load_with({"encoder_confirm_min_command_rad_s:=" + as_param(above)}),
    controller_interface::return_type::OK);
  // Equal (an empty band; the two criteria contiguous) loads.
  EXPECT_EQ(
    load_with({"encoder_confirm_min_command_rad_s:=" + as_param(stall.min_command_rad_s)}),
    controller_interface::return_type::OK);
}

TEST(SwerveControllerThresholdRules, EncoderThresholdInsideTheReleaseBandIsRefused)
{
  const auto stall = active_config::stall_config();
  EXPECT_NE(
    load_with({"encoder_confirm_min_command_rad_s:=" + as_param(stall.release_command_rad_s)}),
    controller_interface::return_type::OK);
}

TEST(SwerveControllerThresholdRules, ZeroCountWindowMustBePositive)
{
  EXPECT_NE(load_with({"stall_silent_window_sec:=0.0"}), controller_interface::return_type::OK);
}

TEST(SwerveControllerThresholdRules, RegulatorFloorBelowStallThresholdIsRefused)
{
  const auto stall = active_config::stall_config();
  // Below the stall threshold but above the release band, so nothing else is
  // violated.
  const double below = 0.5 * (stall.release_command_rad_s + stall.min_command_rad_s);
  ASSERT_LT(below, stall.min_command_rad_s);
  EXPECT_NE(
    load_with({"wheel_regulator_floor_rad_s:=" + as_param(below)}),
    controller_interface::return_type::OK);
  EXPECT_EQ(
    load_with({"wheel_regulator_floor_rad_s:=" + as_param(stall.min_command_rad_s)}),
    controller_interface::return_type::OK);
}

TEST(SwerveControllerThresholdRules, NonFiniteWindowsAreRefused)
{
  // stall_window_sec = NaN: every armed wheel would trip one cycle after arming.
  // encoder_confirm_window_sec = inf: SR-18's encoder trigger could never fire.
  for (const char * o :
    {"stall_window_sec:=.nan", "stall_silent_window_sec:=.nan", "encoder_confirm_window_sec:=.inf",
      "stall_escalation_latched_sec:=.inf", "stall_log_period_sec:=.nan"})
  {
    EXPECT_NE(load_with({o}), controller_interface::return_type::OK) << o;
  }
}

int main(int argc, char ** argv) { return run_bench_tests(argc, argv); }
