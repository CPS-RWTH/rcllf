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

// Fixture: a timer callback that sleeps on ROS time. Under rcllf
// ROS time is LF logical time, which does not advance within a tag, so the
// sleep never returns unless the binding refuses it.

#include <chrono>
#include <cinttypes>
#include <memory>

#include "clock_fixture/clock_probe.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using clock_fixture::lf_logical_time_ns;

class ClockRosSleep : public rclcpp::Node
{
public:
  explicit ClockRosSleep(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("clock_ros_sleep", options)
  {
    timer_ = this->create_wall_timer(100ms, std::bind(&ClockRosSleep::on_timer, this));
  }

private:
  void on_timer()
  {
    RCLCPP_INFO(
      this->get_logger(),
      "CLOCKPROBE kind=before_ros_sleep node=clock_ros_sleep now_ns=%" PRId64 " tag_ns=%" PRId64,
      static_cast<int64_t>(this->now().nanoseconds()), static_cast<int64_t>(lf_logical_time_ns()));
    this->get_clock()->sleep_for(rclcpp::Duration(50ms));
    RCLCPP_INFO(
      this->get_logger(),
      "CLOCKPROBE kind=after_ros_sleep node=clock_ros_sleep now_ns=%" PRId64 " tag_ns=%" PRId64,
      static_cast<int64_t>(this->now().nanoseconds()), static_cast<int64_t>(lf_logical_time_ns()));
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockRosSleep>());
  rclcpp::shutdown();
  return 0;
}
