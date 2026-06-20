#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;

class MovementStatus : public rclcpp::Node
{
public:
  MovementStatus()
  : Node("movement_status")
  {
    device_id_ = declare_parameter<std::string>(
      "device_id",
      "sharx_1");

    movement_threshold_ = declare_parameter<double>(
      "movement_threshold",
      0.01);

    thruster_subscription_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
      "/sharx/thruster_command",
      10,
      std::bind(
        &MovementStatus::thruster_callback,
        this,
        std::placeholders::_1));

    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      "/sharx/status",
      10);

    RCLCPP_INFO(
      get_logger(),
      "Movement status node started");
  }

private:
  void thruster_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (message->data.size() < 2) {
      RCLCPP_WARN(
        get_logger(),
        "Thruster command requires left and right values");
      return;
    }

    const double left = message->data[0];
    const double right = message->data[1];

    const bool moving =
      std::abs(left) > movement_threshold_ ||
      std::abs(right) > movement_threshold_;

    const std::string new_status =
      moving ? "moving" : "stopped";

    // Avoid publishing the same status repeatedly.
    if (new_status == last_status_) {
      return;
    }

    last_status_ = new_status;

    const json status = {
      {"device_id", device_id_},
      {"status", new_status},
      {"mode", "teleop"},
      {"left_thruster", left},
      {"right_thruster", right}
    };

    std_msgs::msg::String output;
    output.data = status.dump();

    status_publisher_->publish(output);

    RCLCPP_INFO(
      get_logger(),
      "Published movement status: %s",
      output.data.c_str());
  }

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr
    thruster_subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;

  std::string device_id_;
  std::string last_status_;

  double movement_threshold_{0.01};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovementStatus>());
  rclcpp::shutdown();

  return 0;
}
