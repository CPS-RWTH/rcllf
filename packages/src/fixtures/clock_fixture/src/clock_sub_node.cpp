// Copyright 2026 rcllf Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Plain (non-component) node: logs now() next to the LF tag in a
// subscription callback and in its own 250 ms timer.

#include <chrono>
#include <cinttypes>
#include <memory>

#include "clock_fixture/clock_probe.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;
using clock_fixture::lf_logical_time_ns;

// Global scope on purpose: the generator names a header-less node class
// without its namespace, so a namespaced one does not build under rcllf.
class ClockSub : public rclcpp::Node
{
public:
  explicit ClockSub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("clock_sub", options)
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "clock_probe", 10, std::bind(&ClockSub::on_message, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(250ms, std::bind(&ClockSub::on_timer, this));
    // No LF tag exists yet while the node is constructed.
    RCLCPP_INFO(
      this->get_logger(), "CLOCKPROBE kind=ctor node=clock_sub now_ns=%" PRId64 " tag_ns=%" PRId64,
      static_cast<int64_t>(this->now().nanoseconds()), static_cast<int64_t>(lf_logical_time_ns()));
  }

private:
  void on_message(const std_msgs::msg::Int64::SharedPtr msg)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "CLOCKPROBE kind=sub node=clock_sub now_ns=%" PRId64 " tag_ns=%" PRId64 " msg_ns=%" PRId64,
      static_cast<int64_t>(this->now().nanoseconds()), static_cast<int64_t>(lf_logical_time_ns()),
      static_cast<int64_t>(msg->data));
  }

  void on_timer()
  {
    // A steady-clock sleep inside a reaction stays allowed and
    // does not move ROS time within the tag.
    steady_clock_.sleep_for(rclcpp::Duration(1ms));
    RCLCPP_INFO(
      this->get_logger(),
      "CLOCKPROBE kind=timer node=clock_sub now_ns=%" PRId64 " tag_ns=%" PRId64,
      static_cast<int64_t>(this->now().nanoseconds()), static_cast<int64_t>(lf_logical_time_ns()));
  }

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockSub>());
  rclcpp::shutdown();
  return 0;
}
