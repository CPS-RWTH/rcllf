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

#include "correspondence_fixture/corr_subscriber.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <string>

#include "correspondence_fixture/nested_echo.hpp"
#include "correspondence_fixture/physical_deadline.hpp"

namespace correspondence_fixture
{

namespace
{

bool env_is_one(const char * name)
{
  const char * v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

/// W3: with CORR_DEADLINE_HOOK=1 the subscription registers a
/// requested-deadline-missed callback, with CORR_MESSAGE_LOST_HOOK=1 a
/// message_lost callback. The binding delivers a missing message at its tag
/// through each registered one. No deadline QoS is set unless
/// CORR_PHYSICAL_DEADLINE_MS asks for one (see physical_deadline_ms()).
/// CORR_DEADLINE_SHUTDOWN=1 makes the deadline callback call rclcpp::shutdown().
rclcpp::SubscriptionOptions missing_hook_options(rclcpp::Node * node)
{
  rclcpp::SubscriptionOptions options;
  if (env_is_one("CORR_DEADLINE_HOOK")) {
    options.event_callbacks.deadline_callback =
      [node](rclcpp::QOSDeadlineRequestedInfo & info) {
        RCLCPP_INFO(
          node->get_logger(), "MISSING sub=%s total=%d change=%d", node->get_name(),
          info.total_count, info.total_count_change);
        if (env_is_one("CORR_DEADLINE_SHUTDOWN")) {
          // W3: a deadline callback that ends the run, so no later
          // callback of the same missing message may run.
          RCLCPP_INFO(
            node->get_logger(), "MISSING sub=%s calls rclcpp::shutdown", node->get_name());
          rclcpp::shutdown();
        }
      };
  }
  if (env_is_one("CORR_MESSAGE_LOST_HOOK")) {
    options.event_callbacks.message_lost_callback =
      [node](rclcpp::QOSMessageLostInfo & info) {
        RCLCPP_INFO(
          node->get_logger(), "LOST sub=%s total=%zu change=%zu", node->get_name(),
          info.total_count, info.total_count_change);
      };
  }
  return options;
}

}  // namespace

CorrSubscriber::CorrSubscriber(const rclcpp::NodeOptions & options)
: Node("corr_sub_1", options)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto qos_a = qos;
  if (physical_deadline_ms() > 0) {
    qos_a.deadline(std::chrono::milliseconds(physical_deadline_ms()));
  }
  sub_a_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/a", qos_a, std::bind(&CorrSubscriber::on_a, this, std::placeholders::_1,
    std::placeholders::_2), missing_hook_options(this));
  sub_b_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/b", qos, std::bind(&CorrSubscriber::on_b, this, std::placeholders::_1,
    std::placeholders::_2));
  // W3: a best-effort, depth-1 subscription on a topic no node of the
  // system publishes. A burst from another process makes Fast DDS raise a
  // physical SAMPLE_LOST. The message_lost callback (if registered) must
  // never run for it. No node of the system publishes the topic, so under
  // rcllf it is declared external in lf/corr_bringup.execution.yaml,
  // and on_a() also reads the status at every tick, without delivering it.
  // It is created in every run: the analyzer gives it a reaction ID, and a
  // reaction ID without a subscription refuses to start.
  lost_probe_ = this->create_subscription<std_msgs::msg::Header>(
    "/corr/lost_probe", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
    std::bind(&CorrSubscriber::on_lost_probe, this, std::placeholders::_1),
    missing_hook_options(this));
  const char * scenario = std::getenv("CORR_SCENARIO");
  if (scenario != nullptr && std::string(scenario) == "nested") {
    // Runs inside the /corr/a publish on the publishing thread.
    sub_a_->set_on_new_message_callback(
      [](size_t) {
        auto & hook = nested_echo_hook();
        if (hook) {
          hook();
        }
      });
  }
}

void CorrSubscriber::on_a(const std_msgs::msg::Header & msg, const rclcpp::MessageInfo & info)
{
  RCLCPP_INFO(
    this->get_logger(), "corr_sub: node=%s topic=/corr/a payload=%s stamp=%d seq=%" PRId64,
    this->get_name(), msg.frame_id.c_str(), msg.stamp.sec,
    info.get_rmw_message_info().publication_sequence_number);
  log_lost_probe_status();
  if (physical_deadline_ms() > 0) {
    // W3: read, without delivering, what DDS counted in physical time.
    // A nonzero total proves physical deadline events occurred that never
    // reached the deadline callback.
    const auto & handlers = sub_a_->get_event_handlers();
    auto h = handlers.find(RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED);
    if (h != handlers.end() && h->second) {
      auto status = std::static_pointer_cast<rclcpp::QOSDeadlineRequestedInfo>(
        h->second->take_data());
      RCLCPP_INFO(
        this->get_logger(), "PHYSICAL deadline sub=%s total=%d", this->get_name(),
        status ? status->total_count : -1);
    }
  }
}

void CorrSubscriber::on_lost_probe(const std_msgs::msg::Header &)
{
  log_lost_probe_status();
}

void CorrSubscriber::log_lost_probe_status()
{
  // W3: read, without delivering, the SAMPLE_LOST status DDS counted on
  // the probe. A nonzero total proves a physical message-lost event occurred.
  if (!lost_probe_) {
    return;
  }
  const auto & handlers = lost_probe_->get_event_handlers();
  auto h = handlers.find(RCL_SUBSCRIPTION_MESSAGE_LOST);
  if (h == handlers.end() || !h->second) {
    return;
  }
  auto status = std::static_pointer_cast<rclcpp::QOSMessageLostInfo>(h->second->take_data());
  if (status && status->total_count != lost_probe_total_) {
    lost_probe_total_ = status->total_count;
    RCLCPP_INFO(
      this->get_logger(), "PHYSICAL lost sub=%s total=%zu", this->get_name(), lost_probe_total_);
  }
}

void CorrSubscriber::on_b(const std_msgs::msg::Header & msg, const rclcpp::MessageInfo & info)
{
  RCLCPP_INFO(
    this->get_logger(), "corr_sub: node=%s topic=/corr/b payload=%s stamp=%d seq=%" PRId64,
    this->get_name(), msg.frame_id.c_str(), msg.stamp.sec,
    info.get_rmw_message_info().publication_sequence_number);
}

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrSubscriber>());
  rclcpp::shutdown();
  return 0;
}
