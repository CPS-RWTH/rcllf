// Copyright 2026 rcllf Contributors
// Licensed under the Apache License, Version 2.0

#ifndef BLIND_PKG__BLIND_NODE_HPP_
#define BLIND_PKG__BLIND_NODE_HPP_

// Shipped only inside a build tree (like the MuJoCo SDK).
#include <vendor_sdk/sdk.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace blind_pkg
{

class BlindNode : public rclcpp::Node
{
public:
  BlindNode();

private:
  void on_timer();
  void on_cmd(const std_msgs::msg::Float64::SharedPtr msg);

  sdk_handle_t * sdk_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
};

}  // namespace blind_pkg

#endif  // BLIND_PKG__BLIND_NODE_HPP_
