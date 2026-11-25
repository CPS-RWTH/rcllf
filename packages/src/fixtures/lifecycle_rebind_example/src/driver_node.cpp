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

// A plain node that moves the reader from its own timer. Tick k
// runs at (k - 1) * 100 ms. Tick 3 configures and activates the reader, tick
// 8 deactivates it and cleans it up (its subscription goes), tick 12
// configures and activates it again (a new subscription).

#include <chrono>
#include <functional>
#include <memory>

#include "lifecycle_rebind_example/rebind_common.hpp"

using namespace std::chrono_literals;

namespace lifecycle_rebind_example
{

class DriverNode : public rclcpp::Node
{
public:
  explicit DriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("driver", options)
  {
    timer_ = this->create_wall_timer(100ms, std::bind(&DriverNode::tick, this));
  }

private:
  void tick()
  {
    ++count_;
    report(this, "drive", count_);
    if (g_reader == nullptr) {
      return;
    }
    if (count_ == 3 || count_ == 12) {
      g_reader->configure();
      g_reader->activate();
    } else if (count_ == 8) {
      g_reader->deactivate();
      g_reader->cleanup();
    } else {
      return;
    }
    report(this, "reader_state", g_reader->get_current_state().id());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_ = 0;
};

}  // namespace lifecycle_rebind_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lifecycle_rebind_example::DriverNode>());
  rclcpp::shutdown();
  return 0;
}
