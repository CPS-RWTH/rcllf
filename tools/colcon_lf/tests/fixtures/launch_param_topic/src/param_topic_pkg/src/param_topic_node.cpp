// Copyright 2026 rcllf Contributors
// Licensed under the Apache License, Version 2.0

#include "param_topic_pkg/param_topic_node.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace param_topic_pkg
{

using namespace std::chrono_literals;

ParamTopicNode::ParamTopicNode()
: rclcpp::Node("param_topic_node")
{
  // No C++ default: only the launch file names the topic, so the parser
  // records ${output_topic} and the launch overlay must fill it in.
  this->declare_parameter<std::string>("output_topic");
  std::string output_topic;
  this->get_parameter("output_topic", output_topic);
  pub_ = create_publisher<std_msgs::msg::Float64>(output_topic, 10);
  timer_ = create_wall_timer(10ms, std::bind(&ParamTopicNode::on_timer, this));
}

void ParamTopicNode::on_timer()
{
  std_msgs::msg::Float64 out;
  out.data = 1.0;
  pub_->publish(out);
}

}  // namespace param_topic_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<param_topic_pkg::ParamTopicNode>());
  rclcpp::shutdown();
  return 0;
}
