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

#include "correspondence_fixture/corr_publisher.hpp"

#include <chrono>
#include <cstdlib>
#include <string>

#include "correspondence_fixture/nested_echo.hpp"
#include "correspondence_fixture/physical_deadline.hpp"
#include "rcl/publisher.h"
#include "rmw/rmw.h"

using namespace std::chrono_literals;

namespace correspondence_fixture
{

namespace
{
thread_local bool t_in_tick = false;
void publish_nested_echo();
rclcpp::Publisher<std_msgs::msg::Header> * g_echo_pub = nullptr;

std::string scenario_from_env()
{
  const char * env = std::getenv("CORR_SCENARIO");
  return (env != nullptr) ? std::string(env) : std::string("one");
}
}  // namespace

CorrPublisher::CorrPublisher(const rclcpp::NodeOptions & options)
: Node("corr_publisher", options), scenario_(scenario_from_env()), tick_(1)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto qos_a = qos;
  if (physical_deadline_ms() > 0) {  // W3: offered deadline matches the subscriber's
    qos_a.deadline(std::chrono::milliseconds(physical_deadline_ms()));
  }
  pub_a_ = this->create_publisher<std_msgs::msg::Header>("/corr/a", qos_a);
  pub_b_ = this->create_publisher<std_msgs::msg::Header>("/corr/b", qos);
  if (scenario_ == "nested") {
    g_echo_pub = pub_b_.get();
    nested_echo_hook() = publish_nested_echo;
  }

  RCLCPP_INFO(this->get_logger(), "corr_pub: scenario=%s", scenario_.c_str());

  if (scenario_ == "stale") {
    helper_thread_ = std::thread([this]() {
          const auto start = std::chrono::steady_clock::now();
          while (rclcpp::ok() && pub_a_->get_subscription_count() < 1) {
            if (std::chrono::steady_clock::now() - start > 5s) {
              break;
            }
            std::this_thread::sleep_for(10ms);
          }
          std_msgs::msg::Header msg;
          msg.frame_id = "H";
          pub_a_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "corr_pub: helper published H");
    });
  }

  timer_ = this->create_wall_timer(200ms, std::bind(&CorrPublisher::on_tick, this));
}

CorrPublisher::~CorrPublisher()
{
  if (helper_thread_.joinable()) {
    helper_thread_.join();
  }
}

void CorrPublisher::on_tick()
{
  const int tick = tick_++;
  const bool active = tick <= 10;
  struct InTick
  {
    InTick() {t_in_tick = true;}
    ~InTick() {t_in_tick = false;}
  } in_tick;

  if (active && scenario_ == "nested") {
    std_msgs::msg::Header msg;
    msg.frame_id = "t" + std::to_string(tick) + "k1";
    msg.stamp = rclcpp::Time(tick, 0);
    pub_a_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
      msg.frame_id.c_str());
  }

  if (active && scenario_ == "one") {
    std_msgs::msg::Header msg;
    msg.frame_id = "t" + std::to_string(tick) + "k1";
    msg.stamp = rclcpp::Time(tick, 0);
    pub_a_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
      msg.frame_id.c_str());
  }

  if (active && scenario_ == "many") {
    for (int k = 1; k <= 3; ++k) {
      std_msgs::msg::Header msg;
      msg.frame_id = "t" + std::to_string(tick);
      msg.stamp = rclcpp::Time(tick, 0);
      pub_a_->publish(msg);
      RCLCPP_INFO(
        this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
        msg.frame_id.c_str());
    }
  }

  if (active && scenario_ == "many_named") {
    // As "many", with the publishing node's name and
    // the publish index in the payload, so a trace shows which instance of a
    // fan-in group and which same-tag publication each delivery was.
    for (int k = 1; k <= 3; ++k) {
      std_msgs::msg::Header msg;
      msg.frame_id = std::string(this->get_name()) + ":t" + std::to_string(tick) + "k" +
        std::to_string(k);
      msg.stamp = rclcpp::Time(tick, 0);
      pub_a_->publish(msg);
      RCLCPP_INFO(
        this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
        msg.frame_id.c_str());
    }
  }

  if (active && scenario_ == "conditional" && (tick % 2 == 1)) {
    std_msgs::msg::Header msg;
    msg.frame_id = "t" + std::to_string(tick) + "k1";
    msg.stamp = rclcpp::Time(tick, 0);
    pub_a_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
      msg.frame_id.c_str());
  }

  if (active && scenario_ == "two_publishers") {
    std_msgs::msg::Header msg_a;
    msg_a.frame_id = "a" + std::to_string(tick);
    msg_a.stamp = rclcpp::Time(tick, 0);
    pub_a_->publish(msg_a);
    RCLCPP_INFO(
      this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
      msg_a.frame_id.c_str());

    std_msgs::msg::Header msg_b;
    msg_b.frame_id = "b" + std::to_string(tick);
    msg_b.stamp = rclcpp::Time(tick, 0);
    pub_b_->publish(msg_b);
    RCLCPP_INFO(
      this->get_logger(), "corr_pub: tick=%d topic=/corr/b payload=%s origin=timer", tick,
      msg_b.frame_id.c_str());
  }

  if (active && scenario_ == "stale") {
    std_msgs::msg::Header msg;
    msg.frame_id = "t" + std::to_string(tick) + "k1";
    msg.stamp = rclcpp::Time(tick, 0);
    pub_a_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
      msg.frame_id.c_str());
  }

  // One publish past the rcl gate, straight on the tracked
  // publisher's rmw handle. The writer's sequence advances without the gate
  // seeing it, so the trace must report path_bypass.
  if (active && scenario_ == "bypass") {
    std_msgs::msg::Header msg;
    msg.frame_id = "t" + std::to_string(tick) + "k1";
    msg.stamp = rclcpp::Time(tick, 0);
    pub_a_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
      msg.frame_id.c_str());
    if (tick == 5) {
      const rmw_publisher_t * rmw =
        rcl_publisher_get_rmw_handle(pub_a_->get_publisher_handle().get());
      std_msgs::msg::Header raw;
      raw.frame_id = "bypass";
      if (rmw != nullptr && rmw_publish(rmw, &raw, nullptr) == RMW_RET_OK) {
        RCLCPP_INFO(this->get_logger(), "corr_pub: tick=%d bypass rmw_publish", tick);
      }
    }
  }

  if (active && scenario_ == "fanout") {
    std_msgs::msg::Header msg;
    msg.frame_id = "t" + std::to_string(tick) + "k1";
    msg.stamp = rclcpp::Time(tick, 0);
    pub_a_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "corr_pub: tick=%d topic=/corr/a payload=%s origin=timer", tick,
      msg.frame_id.c_str());
  }
}

namespace
{
void publish_nested_echo()
{
  if (!t_in_tick || g_echo_pub == nullptr) {
    return;
  }
  std_msgs::msg::Header echo;
  echo.frame_id = "nested";
  g_echo_pub->publish(echo);
}
}  // namespace

}  // namespace correspondence_fixture

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<correspondence_fixture::CorrPublisher>());
  rclcpp::shutdown();
  return 0;
}
