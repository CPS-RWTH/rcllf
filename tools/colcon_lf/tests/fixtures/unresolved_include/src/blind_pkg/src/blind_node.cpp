// Copyright 2026 rcllf Contributors
// Licensed under the Apache License, Version 2.0

#include "blind_pkg/blind_node.hpp"

#include <chrono>
#include <memory>

namespace blind_pkg
{

using namespace std::chrono_literals;

BlindNode::BlindNode()
: rclcpp::Node("blind_node")
{
  pub_ = create_publisher<std_msgs::msg::Float64>("state", 10);
  sub_ = create_subscription<std_msgs::msg::Float64>(
    "cmd", 10, std::bind(&BlindNode::on_cmd, this, std::placeholders::_1));
  timer_ = create_wall_timer(10ms, std::bind(&BlindNode::on_timer, this));
}

void BlindNode::on_timer()
{
  std_msgs::msg::Float64 out;
  out.data = sdk_step(sdk_);
  pub_->publish(out);
}

void BlindNode::on_cmd(const std_msgs::msg::Float64::SharedPtr msg)
{
  sdk_set(sdk_, msg->data);
}

}  // namespace blind_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<blind_pkg::BlindNode>());
  rclcpp::shutdown();
  return 0;
}
