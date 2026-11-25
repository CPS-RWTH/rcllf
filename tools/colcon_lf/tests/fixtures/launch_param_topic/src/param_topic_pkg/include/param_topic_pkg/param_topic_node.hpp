// Copyright 2026 rcllf Contributors
// Licensed under the Apache License, Version 2.0

#ifndef PARAM_TOPIC_PKG__PARAM_TOPIC_NODE_HPP_
#define PARAM_TOPIC_PKG__PARAM_TOPIC_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace param_topic_pkg
{

class ParamTopicNode : public rclcpp::Node
{
public:
  ParamTopicNode();

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

}  // namespace param_topic_pkg

#endif  // PARAM_TOPIC_PKG__PARAM_TOPIC_NODE_HPP_
