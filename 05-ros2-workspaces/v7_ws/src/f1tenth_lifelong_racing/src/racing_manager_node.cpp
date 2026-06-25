#include "f1tenth_lifelong_racing/racing_manager_node.hpp"

#include <chrono>
#include <functional>
#include <string>

namespace f1tenth_lifelong_racing
{

RacingManagerNode::RacingManagerNode()
: rclcpp::Node("racing_manager_node"), state_(RacingState::EXPLORE)
{
  this->declare_parameter<std::string>("loop_closure_topic", "/slam/loop_closure_detected");
  const auto loop_closure_topic = this->get_parameter("loop_closure_topic").as_string();

  loop_closure_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    loop_closure_topic, rclcpp::QoS(10),
    std::bind(&RacingManagerNode::loop_closure_callback, this, std::placeholders::_1));

  state_pub_ = this->create_publisher<std_msgs::msg::Int32>("/racing/state", rclcpp::QoS(10));

  state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RacingManagerNode::publish_state, this));

  RCLCPP_INFO(
    this->get_logger(),
    "racing_manager_node started -- state=EXPLORE, listening for loop closure on '%s'",
    loop_closure_topic.c_str());
}

void RacingManagerNode::loop_closure_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && state_ == RacingState::EXPLORE) {
    transition_to(RacingState::OPTIMIZE);
  }
}

void RacingManagerNode::transition_to(RacingState new_state)
{
  state_ = new_state;
  RCLCPP_INFO(
    this->get_logger(), "racing_manager_node: transition -> %s",
    new_state == RacingState::OPTIMIZE ? "OPTIMIZE" : "EXPLORE");
  publish_state();
}

void RacingManagerNode::publish_state()
{
  std_msgs::msg::Int32 msg;
  msg.data = static_cast<int32_t>(state_);
  state_pub_->publish(msg);
}

}  // namespace f1tenth_lifelong_racing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<f1tenth_lifelong_racing::RacingManagerNode>());
  rclcpp::shutdown();
  return 0;
}
