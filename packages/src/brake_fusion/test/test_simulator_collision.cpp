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

// Collision record of the simulator: a vehicle that reaches the
// obstacle logs a "collision" event whether or not it ever stops. Before the
// fix the only collision flag sat inside the "stop" event, so a run that never
// braked (a /wheel/speed stream that never starts) drove through the obstacle
// and its simulator.jsonl said nothing about it.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <random>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "brake_fusion/loss_events.hpp"
#include "brake_fusion/simulator_node.hpp"
#include "custom_interfaces/msg/brake_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace
{

constexpr double kV0 = 15.0;      // m/s, the simulator default
constexpr double kDecel = 8.0;    // m/s^2, the simulator default
constexpr double kStep = 0.001;   // s, one plant step

std::string make_log_dir()
{
  std::string tmpl = ::testing::TempDir() + "bf_collision_XXXXXX";
  std::vector<char> buf(tmpl.begin(), tmpl.end());
  buf.push_back('\0');
  const char * dir = ::mkdtemp(buf.data());
  if (dir == nullptr) {
    throw std::runtime_error("mkdtemp failed");
  }
  return dir;
}

// Every simulator.jsonl line whose "ev" is `ev`.
std::vector<std::string> events(const std::string & path, const std::string & ev)
{
  std::vector<std::string> out;
  std::ifstream in(path);
  std::string line;
  const std::string key = "\"ev\":\"" + ev + "\"";
  while (std::getline(in, line)) {
    if (line.find(key) != std::string::npos) {
      out.push_back(line);
    }
  }
  return out;
}

double number(const std::string & line, const std::string & field)
{
  std::smatch m;
  const std::regex re("\"" + field + "\":(-?[0-9.eE+-]+)");
  if (!std::regex_search(line, m, re)) {
    ADD_FAILURE() << "no field " << field << " in " << line;
    return std::nan("");
  }
  return std::stod(m[1].str());
}

// Remaps the three absolute topics of one run (/sim/state, /clock,
// /brake_cmd) under a prefix no other process uses, for the simulator and
// the peer alike, and returns the prefix. The plant gate counts every
// /sim/state subscriber and every /brake_cmd publisher on the domain, so a
// foreign one (another brake_fusion container on this host, network host,
// same domain) would release step 1 before the brake latch or feed the
// plant its own commands. A 64-bit random_device draw per process separates
// processes in different PID namespaces (containers share the host network,
// so getpid() alone can repeat); the counter separates runs in this process.
std::string isolate_topics(std::vector<std::string> & args)
{
  static const std::string process_tag = [] {
      std::random_device rd;
      const uint64_t r = (static_cast<uint64_t>(rd()) << 32) ^ rd();
      return std::to_string(::getpid()) + "_" + std::to_string(r);
    }();
  static int run = 0;
  const std::string prefix = "/bf_collision_" + process_tag + "_" + std::to_string(run++);
  args = {"--ros-args"};
  for (const char * topic : {"/sim/state", "/clock", "/brake_cmd"}) {
    args.insert(args.end(), {"-r", std::string(topic) + ":=" + prefix + topic});
  }
  return prefix;
}

// Runs the simulator for `max_steps` steps as fast as it goes (factor 0),
// with a peer that publishes /brake_cmd at `brake_level` every 5 ms of wall
// time (none when nullopt, a controller that never decides), and returns the
// path of its simulator.jsonl.
//
// The plant holds its first step until /sim/state has a subscriber
// (expected_state_subscribers = 1, SimulatorNode::step_callback). With a brake
// level the peer creates that subscriber only after the simulator has logged a
// brake_rx record, so the brake is latched (receipt latch, d_b = 0) before
// step 1 whatever discovery and the first wall-timer publish cost; no fixed
// grace period. Its log is line-buffered (JsonlLog), so the record is visible
// to this reader as soon as the callback returns. braked_from_step_one()
// checks the outcome.
std::string run_simulator(
  double obstacle, std::optional<double> brake_level, int64_t max_steps)
{
  const std::string dir = make_log_dir();
  const std::string log = dir + "/simulator.jsonl";
  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter("obstacle_distance", obstacle),
    rclcpp::Parameter("publish_clock", true),
    rclcpp::Parameter("real_time_factor", 0.0),
    rclcpp::Parameter("max_steps", max_steps),
    rclcpp::Parameter("expected_state_subscribers", static_cast<int64_t>(1)),
    rclcpp::Parameter("log_dir", dir)};
  std::vector<std::string> remaps;
  const std::string prefix = isolate_topics(remaps);
  {
    auto sim = std::make_shared<brake_fusion::SimulatorNode>(
      rclcpp::NodeOptions().parameter_overrides(params).arguments(remaps));
    auto peer = std::make_shared<rclcpp::Node>(
      "collision_peer", rclcpp::NodeOptions().arguments(remaps));
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    auto brake = peer->create_publisher<custom_interfaces::msg::BrakeCmd>(
      "/brake_cmd", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().deadline(
        rclcpp::Duration(brake_fusion::kBrakeDeadline)));
    EXPECT_EQ(std::string(brake->get_topic_name()), prefix + "/brake_cmd") << "remap not applied";
    int64_t last_clock_ns = -1;
    auto clock = peer->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", qos, [&](const rosgraph_msgs::msg::Clock & msg) {
        last_clock_ns = rclcpp::Time(msg.clock).nanoseconds();
      });
    rclcpp::TimerBase::SharedPtr timer;
    if (brake_level) {
      timer = peer->create_wall_timer(
        std::chrono::milliseconds(5), [&brake, level = *brake_level]() {
          custom_interfaces::msg::BrakeCmd cmd;
          cmd.level = level;
          brake->publish(cmd);
        });
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(sim);
    executor.add_node(peer);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    if (brake_level) {
      while (events(log, "brake_rx").empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(std::chrono::milliseconds(10));
      }
      EXPECT_FALSE(events(log, "brake_rx").empty())
        << "the simulator received no /brake_cmd within 60 s, the plant never started";
    }
    // Releases the plant (the step-1 gate above).
    auto state = peer->create_subscription<nav_msgs::msg::Odometry>(
      "/sim/state", qos, [](const nav_msgs::msg::Odometry &) {});
    EXPECT_EQ(std::string(state->get_topic_name()), prefix + "/sim/state") << "remap not applied";
    while (last_clock_ns < max_steps * 1'000'000 && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(last_clock_ns, max_steps * 1'000'000) << "the simulator did not reach its horizon";
  }  // the node closes its log here
  return log;
}

// The brake acts from step 1: the first brake_applied record (logged when the
// applied level changes, SimulatorNode::integrate_step) is at step 1 with
// `level`. A later first step means the plant ran unbraked steps because the
// command was not latched before it started, a harness start race and not a
// simulator fault; this names it instead of letting a position tolerance fail.
::testing::AssertionResult braked_from_step_one(const std::string & log, double level)
{
  const auto applied = events(log, "brake_applied");
  if (applied.empty()) {
    return ::testing::AssertionFailure()
           << "brake not latched at step 1: no brake_applied record at all, the plant never "
           << "braked (no /brake_cmd reached the simulator) in " << log;
  }
  const double step = number(applied[0], "step");
  if (step != 1.0) {
    return ::testing::AssertionFailure()
           << "brake not latched at step 1: the first brake_applied record is at step " << step
           << ", so the plant ran " << step - 1.0 << " unbraked steps before the brake command "
           << "arrived (a start race in the test harness, not a simulator fault): " << applied[0];
  }
  if (number(applied[0], "level") != level) {
    return ::testing::AssertionFailure()
           << "the brake applied at step 1 has level " << number(applied[0], "level")
           << ", not " << level << ": " << applied[0];
  }
  return ::testing::AssertionSuccess();
}

class SimulatorCollision : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

// The case: no brake command ever acts. At a constant 15 m/s the
// position after step n is 0.015 n m (MuJoCo, semi-implicit Euler, no force),
// so an obstacle at 3.0075 m is first reached at step ceil(3.0075 / 0.015) =
// 201, at 3.015 m. The vehicle never stops, so there is no stop event.
TEST_F(SimulatorCollision, UnbrakedVehicleLogsCollisionAtTheObstacle)
{
  const double obstacle = 3.0075;
  const std::string log = run_simulator(obstacle, std::nullopt, 400);
  const auto hits = events(log, "collision");
  ASSERT_EQ(hits.size(), 1U)
    << "no collision logged although the vehicle reached the obstacle: "
    << hits.size() << " collision events in " << log;
  const auto & hit = hits[0];
  const int64_t expect_step = static_cast<int64_t>(std::ceil(obstacle / (kV0 * kStep)));
  EXPECT_EQ(expect_step, 201);
  EXPECT_EQ(static_cast<int64_t>(number(hit, "step")), expect_step) << hit;
  EXPECT_NEAR(number(hit, "position"), 0.015 * 201, 1e-9) << hit;
  EXPECT_NEAR(number(hit, "range"), obstacle - 0.015 * 201, 1e-9) << hit;
  EXPECT_NEAR(number(hit, "velocity"), kV0, 1e-9) << hit;
  EXPECT_TRUE(events(log, "stop").empty()) << "an unbraked vehicle logged a stop";
  EXPECT_TRUE(events(log, "brake_applied").empty()) << "a brake acted without a command";
  EXPECT_EQ(events(log, "horizon").size(), 1U);
}

// A full brake from the start stops after v0^2 / (2 a) = 14.0625 m, short of
// an obstacle at 40 m: a stop event with collision false and no collision
// event, as before the fix.
TEST_F(SimulatorCollision, StopShortOfTheObstacleLogsNoCollision)
{
  const std::string log = run_simulator(40.0, 1.0, 2500);
  ASSERT_TRUE(braked_from_step_one(log, 1.0));
  EXPECT_TRUE(events(log, "collision").empty()) << "collision logged for a stop 26 m short";
  const auto stops = events(log, "stop");
  ASSERT_EQ(stops.size(), 1U);
  EXPECT_NE(stops[0].find("\"collision\":false"), std::string::npos) << stops[0];
  EXPECT_NEAR(number(stops[0], "position"), kV0 * kV0 / (2.0 * kDecel), 0.05) << stops[0];
}

// A full brake from the start against an obstacle at 10 m: the vehicle
// reaches it at sqrt(v0^2 - 2 a 10) = sqrt(65) = 8.06 m/s and stops at about
// 14.06 m. One collision event at the obstacle, and the stop event, logged
// later, still carries collision true.
TEST_F(SimulatorCollision, CollisionThenStopPastTheObstacle)
{
  const double obstacle = 10.0;
  const std::string log = run_simulator(obstacle, 1.0, 2500);
  ASSERT_TRUE(braked_from_step_one(log, 1.0));
  const auto hits = events(log, "collision");
  ASSERT_EQ(hits.size(), 1U)
    << "no collision logged although the vehicle reached the obstacle";
  EXPECT_GE(number(hits[0], "position"), obstacle) << hits[0];
  EXPECT_LT(number(hits[0], "position"), obstacle + 0.015) << hits[0];
  EXPECT_NEAR(number(hits[0], "velocity"), std::sqrt(kV0 * kV0 - 2.0 * kDecel * obstacle), 0.05)
    << hits[0];
  const auto stops = events(log, "stop");
  ASSERT_EQ(stops.size(), 1U);
  EXPECT_NE(stops[0].find("\"collision\":true"), std::string::npos) << stops[0];
  EXPECT_GT(number(stops[0], "step"), number(hits[0], "step"));
}

}  // namespace
