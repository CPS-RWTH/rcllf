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

#include "helper_method_example/helper_node.hpp"

#include <chrono>
#include <functional>

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace helper_method_example
{

HelperNode::HelperNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("helper_node", options)
{
}

HelperNode::CallbackReturn HelperNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  // Delegate resource creation to helper methods
  initPubSub();
  initServices();
  initParameters();
  return CallbackReturn::SUCCESS;
}

// Helper method that creates publishers and subscribers
void HelperNode::initPubSub()
{
  // Topic name from a declared parameter
  this->declare_parameter<std::string>("output_topic", "helper_output");
  output_topic_ = this->get_parameter("output_topic").as_string();

  publisher_ = this->create_publisher<std_msgs::msg::String>("helper_output", 10);
  subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "helper_input", 10, [this](std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
    });

  timer_ = this->create_wall_timer(500ms, std::bind(&HelperNode::onTimer, this));
}

// The timer's callback. Publishes, so this example actually exercises the
// publisher its helper method creates.
void HelperNode::onTimer()
{
  auto message = std_msgs::msg::String();
  message.data = "Helper tick";
  publisher_->publish(message);
}

// Helper method that creates services
void HelperNode::initServices() {this->declare_parameter<int>("service_timeout", 5);}

// Helper method that declares parameters
void HelperNode::initParameters()
{
  this->declare_parameter<std::string>("robot_name", "helper_bot");
  this->declare_parameter<double>("max_speed", 2.5);
}

}  // namespace helper_method_example

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<helper_method_example::HelperNode>(rclcpp::NodeOptions());

  // Drive the node to `active` before spinning, same as lifecycle_example.
  // Every resource this node owns is created in on_configure (via the helper
  // methods that are the point of this example), so a node left in
  // `unconfigured` has no publisher, no subscriber and no timer, and this
  // binary does nothing observable. That also makes it useless as the non-LF
  // reference to compare the wrapper against.
  //
  // This node does not override on_activate, so the base's implementation
  // runs and activates its managed publisher. A node that DOES override it
  // must chain to the base or its publishers stay deactivated.
  if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "configure() failed");
    rclcpp::shutdown();
    return 1;
  }
  if (node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_ERROR(node->get_logger(), "activate() failed");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
