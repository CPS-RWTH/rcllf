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

#ifndef LIFECYCLE_COMPONENT_EXAMPLE__LIFECYCLE_COMPONENT_NODE_HPP_
#define LIFECYCLE_COMPONENT_EXAMPLE__LIFECYCLE_COMPONENT_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace lifecycle_component_example
{

/// A managed lifecycle node that is ALSO registered as a component.
///
/// This is the ordinary way lifecycle nodes are written in ROS 2 (Nav2's
/// servers are all shaped this way), and it is the case this fixture covers:
/// the component path never drove the managed lifecycle, so the node stayed
/// `unconfigured`, owned none of the entities below, and bound nothing.
///
/// The entity placement is deliberate and mirrors `lifecycle_example`: the
/// publisher is created in `on_configure` and the timer in `on_activate`, so a
/// node that is never transitioned has literally nothing for
/// `ros2_lf_bind_node`'s callback-group walk to find. That is what makes the
/// failure silent rather than merely wrong.
class LifecycleComponentNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleComponentNode(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  void timer_callback();

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

}  // namespace lifecycle_component_example

#endif  // LIFECYCLE_COMPONENT_EXAMPLE__LIFECYCLE_COMPONENT_NODE_HPP_
