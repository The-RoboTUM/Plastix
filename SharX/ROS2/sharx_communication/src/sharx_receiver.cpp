#include <fstream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;

class SharXReceiver : public rclcpp::Node
{
public:
  SharXReceiver() : Node("sharx_receiver")
  {
    device_id_ = declare_parameter<std::string>(
      "device_id",
      "sharx_1");

    command_topic_ = declare_parameter<std::string>(
      "command_topic",
      "/octopus/commands");

    status_topic_ = declare_parameter<std::string>(
      "status_topic",
      "/sharx/status");

    target_file_ = declare_parameter<std::string>(
      "target_file",
      "/tmp/sharx_latest_target.json");

    command_subscription_ =
      create_subscription<std_msgs::msg::String>(
      command_topic_,
      10,
      std::bind(
        &SharXReceiver::command_callback,
        this,
        std::placeholders::_1));

    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      status_topic_,
      10);

    RCLCPP_INFO(
      get_logger(),
      "SharX receiver started");
    RCLCPP_INFO(
      get_logger(),
      "Device ID: %s",
      device_id_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Command topic: %s",
      command_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Status topic: %s",
      status_topic_.c_str());
  }

private:
  void command_callback(
    const std_msgs::msg::String::SharedPtr message)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received command: %s",
      message->data.c_str());

    json command;

    try {
      command = json::parse(message->data);
    } catch (const json::parse_error & error) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid JSON: %s",
        error.what());

      publish_status(
        "",
        "rejected",
        "invalid_json");

      return;
    }

    if (!command.is_object()) {
      publish_status(
        "",
        "rejected",
        "command_is_not_json_object");

      return;
    }

    const std::string target_device =
      command.value("device_id", "");

    if (target_device != device_id_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Ignoring command intended for device: %s",
        target_device.c_str());

      return;
    }

    const std::string command_type =
      command.value("type", "");

    const std::string command_id =
      command.value("command_id", "");

    if (command_type != "collect_plastic") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unsupported command type: %s",
        command_type.c_str());

      publish_status(
        command_id,
        "rejected",
        "unsupported_command");

      return;
    }

    if (
      !command.contains("target") ||
      !command["target"].is_object() ||
      !command["target"].contains("latitude") ||
      !command["target"].contains("longitude"))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Target coordinates are missing");

      publish_status(
        command_id,
        "rejected",
        "missing_coordinates");

      return;
    }

    const double latitude =
      command["target"]["latitude"].get<double>();

    const double longitude =
      command["target"]["longitude"].get<double>();

    if (!save_target(command)) {
      publish_status(
        command_id,
        "rejected",
        "target_storage_failed");

      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Plastic target accepted: latitude=%.6f, longitude=%.6f",
      latitude,
      longitude);

    publish_accepted_status(
      command_id,
      latitude,
      longitude);
  }

  bool save_target(const json & command)
  {
    const std::string & file_path = target_file_;

    std::ofstream output_file(file_path);

    if (!output_file.is_open()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not open target file: %s",
        file_path.c_str());

      return false;
    }

    output_file << command.dump(2);
    output_file.close();

    RCLCPP_INFO(
      this->get_logger(),
      "Target saved to %s",
      file_path.c_str());

    return true;
  }

  void publish_accepted_status(
    const std::string & command_id,
    double latitude,
    double longitude)
  {
    const json response = {
      {"command_id", command_id},
      {"device_id", device_id_},
      {"status", "task_received"},
      {"accepted", true},
      {"latitude", latitude},
      {"longitude", longitude}
    };

    publish_json(response);
  }

  void publish_status(
    const std::string & command_id,
    const std::string & status,
    const std::string & reason)
  {
    const json response = {
      {"command_id", command_id},
      {"device_id", device_id_},
      {"status", status},
      {"accepted", false},
      {"reason", reason}
    };

    publish_json(response);
  }

  void publish_json(const json & response)
  {
    std_msgs::msg::String message;
    message.data = response.dump();

    status_publisher_->publish(message);

    RCLCPP_INFO(
      this->get_logger(),
      "Published status: %s",
      message.data.c_str());
  }

  std::string device_id_;
  std::string command_topic_;
  std::string status_topic_;
  std::string target_file_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    command_subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<SharXReceiver>());

  rclcpp::shutdown();
  return 0;
}
