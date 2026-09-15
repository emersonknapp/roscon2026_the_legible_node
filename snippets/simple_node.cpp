#include <string>
#include <utility>

#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

class ReTalker : public rclcpp::Node
{
public:
  explicit ReTalker(const rclcpp::NodeOptions & options)
  : Node("talker", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast{10});
    sub_ = create_subscription<example_interfaces::msg::String>(
      "chatter", qos, std::bind(this, &ReTalker::republish, std::placeholders::_1));
    pub_ = create_publisher<example_interfaces::msg::String>("chatter", qos);
  }

  void republish(std::unique_ptr<example_interfaces::msg::String> msg)
  {
    msg->data = msg_prefix_ + msg->data;
    pub_->publish(std::move(msg));
  }

private:
  std::string msg_prefix_;
  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr pub_;
};
