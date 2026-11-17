// Test helper: the ACTIVE swerve_controller parameters, read from the
// ros2_controllers.yaml that gripperx_control INSTALLS (the file ros2_control_node
// is started with), never from constants copied into a test. CLAUDE.md §6: a test
// that reads a duplicated constant is a tautology -- with this helper, changing a
// threshold in the deployed config is what makes these tests fail or pass.

#ifndef GRIPPERX_SWERVE_CONTROLLER__TEST__ACTIVE_CONFIG_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__TEST__ACTIVE_CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gripperx_swerve_controller/drive_fault_monitor.hpp"
#include "gripperx_swerve_controller/stall_detector.hpp"
#include "gripperx_swerve_controller/wheel_regulator.hpp"

namespace active_config
{

inline std::string controllers_yaml_path()
{
  return ament_index_cpp::get_package_share_directory("gripperx_control") +
         "/config/ros2_controllers.yaml";
}

inline YAML::Node swerve_parameters()
{
  const YAML::Node root = YAML::LoadFile(controllers_yaml_path());
  const YAML::Node params = root["swerve_controller"]["ros__parameters"];
  if (!params) {
    throw std::runtime_error("swerve_controller.ros__parameters missing in " +
                             controllers_yaml_path());
  }
  return params;
}

/// A key the test depends on must be IN the file -- a silent fallback to the
/// struct default would turn this back into a duplicated constant.
template<typename T>
T required(const YAML::Node & params, const std::string & key)
{
  if (!params[key]) {
    throw std::runtime_error("ros2_controllers.yaml has no swerve_controller." + key);
  }
  return params[key].as<T>();
}

inline gripperx_swerve_controller::StallDetectorConfig stall_config()
{
  const auto p = swerve_parameters();
  gripperx_swerve_controller::StallDetectorConfig c;
  c.enabled = required<bool>(p, "stall_detection_enabled");
  c.window_sec = required<double>(p, "stall_window_sec");
  c.min_command_rad_s = required<double>(p, "stall_min_command_rad_s");
  c.min_position_delta_rad = required<double>(p, "stall_min_position_delta_rad");
  c.release_command_rad_s = required<double>(p, "stall_release_command_rad_s");
  // As the controller does it: the zero-count criterion's arming threshold IS
  // the encoder trigger's.
  c.silent_min_command_rad_s = required<double>(p, "encoder_confirm_min_command_rad_s");
  c.silent_window_sec = required<double>(p, "stall_silent_window_sec");
  c.count_rad = required<double>(p, "wheel_encoder_count_rad");
  c.max_latched_wheels = required<std::size_t>(p, "stall_max_latched_wheels");
  c.assume_live_provenance = required<bool>(p, "assume_live_provenance");
  return c;
}

inline gripperx_swerve_controller::DriveFaultConfig drive_fault_config()
{
  const auto p = swerve_parameters();
  gripperx_swerve_controller::DriveFaultConfig c;
  c.enabled = required<bool>(p, "drive_fault_enabled");
  c.encoder_confirm_min_command_rad_s = required<double>(p, "encoder_confirm_min_command_rad_s");
  c.encoder_confirm_window_sec = required<double>(p, "encoder_confirm_window_sec");
  c.stall_escalation_window_sec = required<double>(p, "stall_escalation_window_sec");
  c.stall_escalation_trips = required<uint32_t>(p, "stall_escalation_trips");
  c.stall_escalate_on_multi_wheel = required<bool>(p, "stall_escalate_on_multi_wheel");
  c.stall_escalation_latched_sec = required<double>(p, "stall_escalation_latched_sec");
  // As the controller does it: "held" IS HWR-30a's release band.
  c.stall_hold_min_command_rad_s = required<double>(p, "stall_release_command_rad_s");
  c.stall_window_sec = required<double>(p, "stall_window_sec");
  c.stall_silent_window_sec = required<double>(p, "stall_silent_window_sec");
  c.assume_live_until_provenance_received = required<bool>(p, "assume_live_provenance");
  return c;
}

/// As SwerveController::on_init builds it: the wheel_regulator_* keys, the
/// output ceiling mirrored from max_wheel_angular_speed, and the one shared
/// assume_live_provenance.
inline gripperx_swerve_controller::WheelRegulatorConfig regulator_config()
{
  const auto p = swerve_parameters();
  gripperx_swerve_controller::WheelRegulatorConfig c;
  c.enabled = required<bool>(p, "wheel_regulator_enabled");
  c.kp = required<double>(p, "wheel_regulator_kp");
  c.ki = required<double>(p, "wheel_regulator_ki");
  c.max_correction_fraction = required<double>(p, "wheel_regulator_max_correction_fraction");
  c.max_sample_age_sec = required<double>(p, "wheel_regulator_max_sample_age_sec");
  c.floor_rad_s = required<double>(p, "wheel_regulator_floor_rad_s");
  c.output_limit_rad_s = required<double>(p, "max_wheel_angular_speed");
  c.assume_live_provenance = required<bool>(p, "assume_live_provenance");
  return c;
}

}  // namespace active_config

#endif  // GRIPPERX_SWERVE_CONTROLLER__TEST__ACTIVE_CONFIG_HPP_
