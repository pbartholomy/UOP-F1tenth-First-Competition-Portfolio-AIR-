#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

namespace f1tenth_lifelong_racing
{

enum class RacingState : int32_t
{
  EXPLORE  = 0,  // unknown track: reactive controller drives, SLAM builds the map
  OPTIMIZE = 1,  // map confirmed via loop closure: MPCC takes over for racing laps
};

class RacingManagerNode : public rclcpp::Node
{
public:
  RacingManagerNode();

private:
  void loop_closure_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_state();
  void transition_to(RacingState new_state);

  RacingState state_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loop_closure_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

}  // namespace f1tenth_lifelong_racing
