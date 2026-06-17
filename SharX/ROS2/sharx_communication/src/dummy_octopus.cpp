#include <chrono>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

class DummyOctopus : public rclcpp::Node
{
public:
  DummyOctopus()
  : Node("dummy_octopus"), command_sent_(false)
  {
    command_publisher_ =
      this->create_publisher<std_msgs::msg::String>(
      "/octopus/commands",
      10);

    status_subscription_ =
      this->create_subscription<std_msgs::msg::String>(
      "/sharx/status",
      10,
      std::bind(
        &DummyOctopus::status_callback,
        this,
        std::placeholders::_1));

    timer_ = this->create_wall_timer(
      2s,
      std::bind(&DummyOctopus::publish_command, this));

    RCLCPP_INFO(this->get_logger(), "Dummy Octopus node started");
  }

private:
  void publish_command()
  {
    if (command_sent_) {
      return;
    }

    const json command = {
      {"command_id", "task_001"},
      {"type", "collect_plastic"},
      {"device_id", "sharx_1"},
      {
        "target",
        {
          {"latitude", 48.2621},
          {"longitude", 11.6683}
        }
      },
      {"plastic_type", "bottle"},
      {"confidence", 0.91},
      {"source_robot", "drone_1"}
    };

    std_msgs::msg::String message;
    message.data = command.dump();

    command_publisher_->publish(message);
    command_sent_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Published command: %s",
      message.data.c_str());
  }

  void status_callback(
    const std_msgs::msg::String::SharedPtr message)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received SharX status: %s",
      message->data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    command_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    status_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool command_sent_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<DummyOctopus>());

  rclcpp::shutdown();
  return 0;
}
