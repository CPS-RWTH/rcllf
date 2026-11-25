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


// Sampler of the tf2 acquisition showcase. A 30 ms timer asks
// rcllf_tf2::AcquisitionLookup (wait bound 55 ms, reject on timeout) for the
// transforms of cases T1 to T7 at fixed acquisition stamps; the expected
// outcomes are derived by hand in README.md and corpus/t1_t7_expected.tsv.
// The outcome callback only logs: it runs in the reaction that decided it,
// which declares no publisher (docs/features/tf2.md).

#include <chrono>
#include <cinttypes>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcllf_tf2/acquisition_lookup.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class AcqSamplerNode : public rclcpp::Node
{
public:
  AcqSamplerNode()
  : Node("acq_sampler")
  {
    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
    lookup_ = std::make_shared<rcllf_tf2::AcquisitionLookup>(
      *buffer_, this, 55ms, rcllf_tf2::OnTimeout::reject());
    timer_ = create_wall_timer(30ms, std::bind(&AcqSamplerNode::on_sample, this));
  }

private:
  void ask(const char * name, const char * source, int64_t stamp_ms)
  {
    const std::string c = name;
    const rclcpp::Time stamp = start_ + rclcpp::Duration(std::chrono::milliseconds(stamp_ms));
    const uint64_t id = lookup_->request(
      "world", source, stamp, [this, c](const rcllf_tf2::Result & r) {
        const auto & tr = r.transform.transform.translation;
        RCLCPP_INFO(
          get_logger(), "ACQ case=%s id=%" PRIu64 " outcome=%s decided_ms=%.3f x=%.9f y=%.9f",
          c.c_str(), r.id, r.name().c_str(),
          static_cast<double>(r.decided_time - start_.nanoseconds()) / 1e6, tr.x, tr.y);
      });
    RCLCPP_INFO(get_logger(), "ACQ_REQ case=%s id=%" PRIu64, name, id);
  }

  void on_sample()
  {
    if (tick_ == 0) {
      start_ = this->now();
      RCLCPP_INFO(get_logger(), "ACQ_START t0=%" PRId64, start_.nanoseconds());
    }
    switch (tick_) {
      case 2: ask("T1", "sensor_link", 30); break;
      case 3: ask("T2", "sensor_link", 80); ask("T5", "held_link", 90); break;
      case 4: ask("T3", "sensor_link", 125); break;
      case 5: ask("T6", "static_link", 150); break;
      case 6: ask("T7", "ghost_link", 170); break;
      case 11: ask("T4", "sensor_link", 320); break;
      default: break;
    }
    ++tick_;
  }

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<rcllf_tf2::AcquisitionLookup> lookup_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_;
  int tick_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AcqSamplerNode>());
  rclcpp::shutdown();
  return 0;
}
