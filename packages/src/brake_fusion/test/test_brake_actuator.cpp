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

// Modeled brake actuation delay d_b: a command stamped t
// acts in the first plant step strictly after t + d_b, by its stamp,
// never by its receipt time; d_b = 0 keeps the receipt latch. (The method is release, not
// take: the rcllf analyzer reads any .take( call as an rclcpp take, RCLLF-R002.)

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "brake_fusion/brake_actuator.hpp"
#include "brake_fusion/loss_events.hpp"
#include "brake_fusion/simulator_node.hpp"
#include "custom_interfaces/msg/brake_cmd.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr int64_t kMs = 1'000'000;

brake_fusion::QueuedBrake cmd(double level, int64_t stamp_ms, int64_t n)
{
  brake_fusion::QueuedBrake c;
  c.level = level;
  c.seq = static_cast<uint64_t>(n);
  c.tick = n;
  c.stamp_ns = stamp_ms * kMs;
  c.n = n;
  return c;
}

TEST(BrakeActuator, ActsAtStampPlusDelayNotAtReceipt)
{
  brake_fusion::BrakeActuator a(25 * kMs);
  ASSERT_TRUE(a.push(cmd(1.0, 50, 1)));
  // Received long before t + d_b = 75 ms: held until the first step strictly
  // after 75 ms, the step at 76 ms on the 1 ms grid.
  EXPECT_FALSE(a.release(51 * kMs).has_value())
    << "a command stamped 50 ms acted at 51 ms: applied at receipt, not after t + d_b = 75 ms";
  EXPECT_FALSE(a.release(74 * kMs).has_value()) << "acted before t + d_b = 75 ms";
  EXPECT_FALSE(a.release(75 * kMs).has_value())
    << "acted in the step at t + d_b = 75 ms, not strictly after it: one step early "
    "against rcllf, whose timers_first simulator runs the step at tag t + d_b before "
    "the command released there";
  const auto acting = a.release(76 * kMs);
  ASSERT_TRUE(acting.has_value()) << "no command acted in the step at t + d_b + 1 ms = 76 ms";
  EXPECT_EQ(acting->n, 1);
  EXPECT_EQ(a.pending(), 0u);
}

TEST(BrakeActuator, ReleasesInStampOrderAndTheNewestActs)
{
  brake_fusion::BrakeActuator a(100 * kMs);
  ASSERT_TRUE(a.push(cmd(0.39, 0, 1)));
  ASSERT_TRUE(a.push(cmd(1.0, 25, 2)));
  ASSERT_TRUE(a.push(cmd(1.0, 50, 3)));
  int64_t released = 0;
  // Step 126 ms: 0 + 100 and 25 + 100 are strictly before it, 50 + 100 is not.
  auto acting = a.release(126 * kMs, &released);
  ASSERT_TRUE(acting.has_value());
  EXPECT_EQ(acting->n, 2);
  EXPECT_EQ(released, 2);
  EXPECT_EQ(a.pending(), 1u);
}

TEST(BrakeActuator, AnOlderStampReceivedSecondStillActsAtItsTime)
{
  // Received out of stamp order, both before they are due: stamp order wins.
  brake_fusion::BrakeActuator a(25 * kMs);
  ASSERT_TRUE(a.push(cmd(0.39, 60, 1)));
  ASSERT_TRUE(a.push(cmd(1.0, 50, 2)))
    << "the command stamped 50 ms was dropped because one stamped 60 ms arrived first: "
    "receipt order, not stamp order";
  const auto first = a.release(76 * kMs);
  ASSERT_TRUE(first.has_value()) << "nothing acted in the step after 50 ms + d_b 25 ms";
  EXPECT_EQ(first->n, 2);
  const auto second = a.release(86 * kMs);
  ASSERT_TRUE(second.has_value()) << "nothing acted in the step after 60 ms + d_b 25 ms";
  EXPECT_EQ(second->n, 1);
}

TEST(BrakeActuator, RefusesAStampNotNewerThanTheActingOneAndANegativeDelay)
{
  brake_fusion::BrakeActuator a(25 * kMs);
  ASSERT_TRUE(a.push(cmd(1.0, 50, 1)));
  EXPECT_FALSE(a.push(cmd(0.0, 50, 2))) << "a duplicate stamp was queued";
  ASSERT_TRUE(a.release(76 * kMs).has_value());
  EXPECT_FALSE(a.push(cmd(0.0, 40, 3))) << "a stamp older than the acting command was queued";
  EXPECT_THROW(brake_fusion::BrakeActuator(-1), std::invalid_argument);
}

TEST(BrakeActuator, OffGridStepActsStrictlyAfterTheDueTime)
{
  // Stock: system-time steps are not on the 1 ms grid. Stamped 75.5 ms, due
  // 100.5 ms: a step at 100.4 ms does not act, the step at 100.5 ms exactly
  // does not act (not strictly after), the next nanosecond does.
  brake_fusion::BrakeActuator a(25 * kMs);
  brake_fusion::QueuedBrake c = cmd(1.0, 0, 1);
  c.stamp_ns = 75'500'000;
  ASSERT_TRUE(a.push(c));
  EXPECT_FALSE(a.release(100'400'000).has_value()) << "acted before t + d_b = 100.5 ms";
  EXPECT_FALSE(a.release(100'500'000).has_value())
    << "acted in a step at exactly t + d_b = 100.5 ms, not strictly after it";
  EXPECT_TRUE(a.release(100'500'001).has_value())
    << "a step 1 ns after t + d_b did not act: the step must be the first strictly after t + d_b";
}

struct StateSample
{
  int64_t stamp_ns;
  double velocity;
};

// Runs the simulator as the /clock source (state stamp = step * 1 ms) and
// publishes one brake command stamped `stamp_ms` (repeated, same stamp, so a
// best-effort loss cannot hide it) from the first state stamped at or after
// `send_from_ms` until `send_until_ms`. Returns every received state.
// `real_time_factor` paces the simulator's system-time step timer (1 ms /
// factor per step); the plant time and every stamp stay on the 1 ms grid.
std::vector<StateSample> run_with_command(
  int64_t delay_ms, double stamp_ms, int64_t send_from_ms, int64_t send_until_ms,
  int64_t max_steps, const std::string & log_dir = "", double real_time_factor = 1.0)
{
  std::vector<rclcpp::Parameter> params{
    {"publish_clock", true}, {"max_steps", max_steps}, {"expected_state_subscribers", 1},
    {"brake_actuation_delay_ms", delay_ms}, {"log_dir", log_dir},
    {"real_time_factor", real_time_factor}};
  auto sim = std::make_shared<brake_fusion::SimulatorNode>(
    rclcpp::NodeOptions().parameter_overrides(params));
  auto peer = std::make_shared<rclcpp::Node>("actuator_peer");
  auto brake = peer->create_publisher<custom_interfaces::msg::BrakeCmd>(
    "/brake_cmd", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().deadline(
      rclcpp::Duration(brake_fusion::kBrakeDeadline)));
  std::vector<StateSample> states;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto state = peer->create_subscription<nav_msgs::msg::Odometry>(
    "/sim/state", qos, [&](const nav_msgs::msg::Odometry & msg) {
      const int64_t t = rclcpp::Time(msg.header.stamp).nanoseconds();
      states.push_back({t, msg.twist.twist.linear.x});
      if (t >= send_from_ms * kMs && t <= send_until_ms * kMs) {
        custom_interfaces::msg::BrakeCmd c;
        c.level = 1.0;
        c.seq = 1;
        c.tick = 0;
        c.stamp_ns = static_cast<int64_t>(stamp_ms * static_cast<double>(kMs));
        brake->publish(c);
      }
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sim);
  executor.add_node(peer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while ((states.empty() || states.back().stamp_ns < max_steps * kMs) &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  return states;
}

// Stamp of the first state whose velocity is below the previous state's: the
// end of the first decelerated 1 ms step, so the onset is one step earlier.
std::optional<int64_t> first_decelerated(const std::vector<StateSample> & states)
{
  for (std::size_t i = 1; i < states.size(); ++i) {
    if (states[i].velocity < states[i - 1].velocity) {
      return states[i].stamp_ns;
    }
  }
  return std::nullopt;
}

// The brake_act records of a simulator log, as the raw JSONL lines.
std::vector<std::string> brake_acts(const std::filesystem::path & dir)
{
  std::vector<std::string> acts;
  std::ifstream in(dir / "simulator.jsonl");
  for (std::string line; std::getline(in, line); ) {
    if (line.find("\"ev\":\"brake_act\"") != std::string::npos) {
      acts.push_back(line);
    }
  }
  return acts;
}

std::filesystem::path fresh_dir(const std::string & name)
{
  const auto dir = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  return dir;
}

TEST(SimulatorActuation, OnsetIsStampPlusDelayEvenWhenReceivedAfterTheStamp)
{
  // Stamped 50 ms, received from plant time 60 ms on (10 ms after its stamp,
  // as in a stock run), d_b = 25 ms: it acts in the first step strictly after
  // 75 ms, the step stamped 76 ms, whose interval (75, 76] starts at the
  // onset 75 ms. Applied at receipt, the onset would be about 60 ms; applied
  // in the step at 75 ms (not strictly after), the first decelerated state
  // would be stamped 75 ms.
  const auto states = run_with_command(25, 50, 60, 70, 150);
  ASSERT_FALSE(states.empty()) << "no /sim/state received";
  const auto first = first_decelerated(states);
  ASSERT_TRUE(first.has_value()) << "the plant never decelerated";
  const char * why = "";
  if (*first < 75 * kMs) {
    why = ": the command was applied at receipt time";
  } else if (*first == 75 * kMs) {
    why = ": it acted in the step at t + d_b, not strictly after it";
  }
  EXPECT_EQ(*first, 76 * kMs)
    << "first decelerated step stamped " << *first / kMs
    << " ms, not 76 ms, the first step strictly after the command stamp 50 ms + d_b 25 ms"
    << why;
}

TEST(SimulatorActuation, ArrivalAtTheDueStepActsInTheNextStepWithoutCountingTheDelayTwice)
{
  // rcllf with `after d_b` on /brake_cmd: the command stamped 50 ms reaches
  // the timers_first simulator at tag 75 ms, after the step stamped 75 ms
  // (sent here from the state stamped 75 ms on). It acts in the next step,
  // stamped 76 ms, and is on time. Keyed on its arrival instead of its
  // stamp it would act after 75 + 25 = 100 ms (d_b counted twice).
  //
  // Wall-clock race (observed in CI: the command acted at 77 ms):
  // the command leaves the peer only after the state stamped 75 ms reached
  // it, and it must reach the simulator's brake_callback before the
  // system-time timer fires the step stamped 76 ms; the executor runs a ready
  // timer before a ready subscription. That DDS round trip cannot be made
  // deterministic here: the simulator is the /clock source on system time,
  // which is what this test checks. At real_time_factor 1 the round trip had
  // 1 ms, which a loaded 2-vCPU runner exceeds. 0.05 gives it one 20 ms step
  // (plant time and stamps unchanged); a 77 ms result now means a round trip
  // above 20 ms, not a missed 1 ms window.
  const auto dir = fresh_dir("bf_actuator_due_arrival");
  const auto states = run_with_command(25, 50, 75, 80, 100, dir.string(), 0.05);
  const auto first = first_decelerated(states);
  ASSERT_TRUE(first.has_value()) << "the plant never decelerated";
  EXPECT_EQ(*first, 76 * kMs)
    << "a command stamped 50 ms received at the step at t + d_b = 75 ms first "
    "decelerated the step stamped " << *first / kMs << " ms, not 76 ms"
    << (*first > 100 * kMs ? ": d_b was counted twice (queued by arrival, not by stamp)" : "");
  const auto acts = brake_acts(dir);
  ASSERT_FALSE(acts.empty()) << "no brake_act record in " << dir;
  EXPECT_NE(acts.front().find("\"late\":false"), std::string::npos)
    << "a command received right after the step at t + d_b (not strictly after it) "
    "was logged late: " << acts.front();
  EXPECT_NE(acts.front().find("\"act_ns\":76000000,\"prev_step_ns\":75000000"),
    std::string::npos) << acts.front();
}

TEST(SimulatorActuation, OnTimeCommandIsNotLoggedLate)
{
  // Received at about 60 ms, due at 75 ms: on time.
  const auto dir = fresh_dir("bf_actuator_on_time");
  run_with_command(25, 50, 60, 70, 150, dir.string());
  const auto acts = brake_acts(dir);
  ASSERT_FALSE(acts.empty()) << "no brake_act record in " << dir;
  EXPECT_NE(acts.front().find("\"late\":false"), std::string::npos) << acts.front();
}

TEST(SimulatorActuation, LateCommandActsAfterReceiptAndIsLoggedLate)
{
  // Stamped 50 ms, due at 75 ms, first received at about 100 ms: it acts at
  // the first step after receipt (it cannot act in the past) and is late.
  const auto dir = fresh_dir("bf_actuator_late");
  const auto states = run_with_command(25, 50, 100, 110, 200, dir.string());
  const auto first = first_decelerated(states);
  ASSERT_TRUE(first.has_value()) << "the plant never decelerated";
  EXPECT_GE(*first - kMs, 100 * kMs) << "a command received at 100 ms acted earlier";
  EXPECT_LE(*first - kMs, 111 * kMs);
  const auto acts = brake_acts(dir);
  ASSERT_FALSE(acts.empty()) << "no brake_act record in " << dir;
  EXPECT_NE(acts.front().find("\"late\":true"), std::string::npos)
    << "a command received about 25 ms after its due time was not logged late: "
    << acts.front();
}

TEST(SimulatorActuation, OffGridDueActsInTheFirstStepStrictlyAfterIt)
{
  // Stamped 75.5 ms, due at 100.5 ms (between two steps, as every due time of
  // a stock run), received from the state stamped 90 ms on: it acts in the
  // first step strictly after 100.5 ms, the step stamped 101 ms. The earlier
  // rule (the step's interval start, the previous state stamp, at or after
  // the due time) waited for the step stamped 102 ms.
  const auto dir = fresh_dir("bf_actuator_off_grid");
  const auto states = run_with_command(25, 75.5, 90, 95, 200, dir.string());
  const auto first = first_decelerated(states);
  ASSERT_TRUE(first.has_value()) << "the plant never decelerated";
  EXPECT_EQ(*first, 101 * kMs)
    << "a command due at 100.5 ms first decelerated the step stamped " << *first / kMs
    << " ms, not 101 ms, the first step strictly after its due time"
    << (*first == 102 * kMs ? " (the interval-start rule, one step late)" : "");
  const auto acts = brake_acts(dir);
  ASSERT_FALSE(acts.empty()) << "no brake_act record in " << dir;
  EXPECT_NE(acts.front().find("\"late\":false"), std::string::npos) << acts.front();
}

TEST(SimulatorActuation, CommandReceivedAfterItsStepRanIsLate)
{
  // Stamped 75.5 ms, due at 100.5 ms, received after the state stamped 101 ms:
  // the step it had to act in (101 ms, the first strictly after 100.5 ms) has
  // run, so it acts in the next step, stamped 102 ms, and is late.
  const auto dir = fresh_dir("bf_actuator_between_steps");
  const auto states = run_with_command(25, 75.5, 101, 110, 200, dir.string());
  const auto first = first_decelerated(states);
  ASSERT_TRUE(first.has_value()) << "the plant never decelerated";
  EXPECT_GE(*first, 102 * kMs)
    << "a command received after the state stamped 101 ms changed that step";
  EXPECT_LE(*first, 111 * kMs);
  const auto acts = brake_acts(dir);
  ASSERT_FALSE(acts.empty()) << "no brake_act record in " << dir;
  EXPECT_NE(acts.front().find("\"late\":true"), std::string::npos)
    << "a command received after the first step strictly after its due time ran "
    "was not logged late: " << acts.front();
}

TEST(SimulatorActuation, ZeroDelayKeepsTheReceiptLatch)
{
  // d_b = 0: today's behaviour, the command acts from the step after its
  // receipt. Received from 60 ms on, so the onset is at or after 60 ms, never
  // at its 50 ms stamp, and at most a few steps later.
  const auto states = run_with_command(0, 50, 60, 70, 150);
  const auto first = first_decelerated(states);
  ASSERT_TRUE(first.has_value()) << "the plant never decelerated";
  EXPECT_GE(*first - kMs, 60 * kMs);
  EXPECT_LE(*first - kMs, 71 * kMs);
}

TEST(SimulatorActuation, RefusesANegativeDelay)
{
  std::vector<rclcpp::Parameter> params{{"brake_actuation_delay_ms", int64_t{-1}}};
  EXPECT_THROW(
    brake_fusion::SimulatorNode(rclcpp::NodeOptions().parameter_overrides(params)),
    std::invalid_argument);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
