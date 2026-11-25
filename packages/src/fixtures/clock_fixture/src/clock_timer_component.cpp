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

// Component node: a 100 ms timer that logs now() next to the LF tag and
// publishes now() so the subscriber can compare stamps.

#include <chrono>
#include <cinttypes>
#include <memory>

#include "clock_fixture/clock_probe.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;

namespace clock_fixture
{

class ClockTimer : public rclcpp::Node
{
public:
  explicit ClockTimer(const rclcpp::NodeOptions & options)
  : Node("clock_timer", options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int64>("clock_probe", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&ClockTimer::on_timer, this));
    // No LF tag exists yet while the component is constructed.
    RCLCPP_INFO(
      this->get_logger(),
      "CLOCKPROBE kind=ctor node=clock_timer now_ns=%" PRId64 " tag_ns=%" PRId64,
      static_cast<int64_t>(this->now().nanoseconds()), static_cast<int64_t>(lf_logical_time_ns()));
  }

private:
  void on_timer()
  {
    const int64_t now_ns = this->now().nanoseconds();
    RCLCPP_INFO(
      this->get_logger(),
      "CLOCKPROBE kind=timer node=clock_timer now_ns=%" PRId64 " tag_ns=%" PRId64,
      static_cast<int64_t>(now_ns), static_cast<int64_t>(lf_logical_time_ns()));
    std_msgs::msg::Int64 msg;
    msg.data = now_ns;
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace clock_fixture

RCLCPP_COMPONENTS_REGISTER_NODE(clock_fixture::ClockTimer)
