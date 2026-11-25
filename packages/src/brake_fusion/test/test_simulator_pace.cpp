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

// Simulator pacing: real_time_factor f runs simulation time (/clock)
// f times faster than real time, and the default keeps 1 ms per step.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
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

struct PaceResult
{
  int64_t last_clock_ns{-1};
  double wall_ms{-1.0};  // first to last /clock message
  double max_gap_ms{0.0};  // longest wall gap between two consecutive /clock messages
  double stall_ms{0.0};  // sum of the wall gaps longer than kStallGapMs
  int64_t steps{-1};  // plant steps the simulator ran (SimulatorNode::steps)
  int64_t step_ticks{-1};  // its timer ticks after start (SimulatorNode::step_ticks)
};

// One timer tick after start is one plant step. The period
// read-back and the wall ratio cannot see a simulator that installs the right
// period but steps on every other tick (pace f / 2, ratio about 0.5 at f = 4,
// inside kMaxRatio); this count can.
void expect_one_step_per_tick(const PaceResult & r, const std::string & run)
{
  EXPECT_EQ(r.step_ticks, r.steps)
    << "cadence mismatch, " << run << ": " << r.step_ticks << " timer ticks after start ran "
    << r.steps << " plant steps, not one step per tick";
}

// Diagnostics only, never part of a verdict: /clock gaps longer than this are
// summed per run and printed with each pair, so a failing pair shows whether
// the host stalled it. The slowest timer these tests set is 1 ms (factor 1);
// rcl skips the ticks a late timer missed. A gap can also be a slow step of the
// simulator itself, so subtracting the gaps would hide a simulator that does
// not keep its pace. MEASURED: unloaded, the longest gap was 4.4 ms in 4x runs
// and 6.8 ms (one) in 1x runs.
constexpr double kStallGapMs = 5.0;

// Runs the simulator for `max_steps` steps with /clock and returns the wall
// time between the first and the last /clock message it published.
PaceResult run_simulator(std::vector<rclcpp::Parameter> params, int64_t max_steps)
{
  params.emplace_back("publish_clock", true);
  params.emplace_back("max_steps", max_steps);
  params.emplace_back("expected_state_subscribers", 1);
  auto sim = std::make_shared<brake_fusion::SimulatorNode>(
    rclcpp::NodeOptions().parameter_overrides(params));
  auto peer = std::make_shared<rclcpp::Node>("pace_peer");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  // The controller's offer: the simulator's /brake_cmd subscription requests a deadline.
  auto brake = peer->create_publisher<custom_interfaces::msg::BrakeCmd>(
    "/brake_cmd", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().deadline(
      rclcpp::Duration(brake_fusion::kBrakeDeadline)));
  auto state = peer->create_subscription<nav_msgs::msg::Odometry>(
    "/sim/state", qos, [](const nav_msgs::msg::Odometry &) {});
  PaceResult result;
  std::chrono::steady_clock::time_point first;
  std::chrono::steady_clock::time_point previous;
  auto clock = peer->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", qos, [&](const rosgraph_msgs::msg::Clock & msg) {
      const auto now = std::chrono::steady_clock::now();
      if (result.last_clock_ns < 0) {
        first = now;
      } else {
        const double gap = std::chrono::duration<double, std::milli>(now - previous).count();
        result.max_gap_ms = std::max(result.max_gap_ms, gap);
        if (gap > kStallGapMs) {
          result.stall_ms += gap;
        }
      }
      previous = now;
      result.last_clock_ns = rclcpp::Time(msg.clock).nanoseconds();
      result.wall_ms = std::chrono::duration<double, std::milli>(now - first).count();
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(sim);
  executor.add_node(peer);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (result.last_clock_ns < max_steps * 1'000'000 &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  result.steps = sim->steps();
  result.step_ticks = sim->step_ticks();
  return result;
}

class SimulatorPace : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // UDPv4 only, no Fast DDS shared-memory transport. Under 16
    // stress workers one run of this binary hung >18 min in
    // SharedMemTransport::OpenInputChannel during its first node construction,
    // and ignored SIGTERM. Read by Fast DDS 2.14 when the participant is
    // created, so it must be set before rclcpp::init. Both endpoints live in
    // this process; UDP loopback carries the same samples.
    ASSERT_EQ(setenv("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4", 1), 0);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(SimulatorPace, DefaultKeepsOneMillisecondPerStep)
{
  const auto r = run_simulator({}, 300);
  ASSERT_EQ(r.last_clock_ns, 300'000'000) << "the simulator did not reach step 300";
  expect_one_step_per_tick(r, "default factor, 300 steps");
  // 299 step intervals of 1 ms each, minus scheduling slack.
  EXPECT_GE(r.wall_ms, 270.0) << "300 steps took " << r.wall_ms
                              << " ms of wall time at the default real_time_factor";
}

// The factor 4 run is compared with a 1x run of the same 800 steps
// measured just before it in this test, not with a fixed wall time. A fixed
// 400 ms bound flaked on a loaded CI runner (433 ms).
//
// Two checks, two jobs. The exact pace is proven by
// FactorFourSetsAQuarterMillisecondTick: the simulator reads its timer period
// back from rcl and the test requires 250000 ns, deterministic and load free.
// This test is the coarse check that the installed period paces the steps.
//
// Each pair is judged by its wall ratio, 4x wall / 1x wall, nothing removed.
// Every pair counts: there is no retry and no best of N, so one bad pair in
// kPairs fails the test. The /clock gaps of each run are printed beside the
// ratio to explain a failure, not to excuse it.
//
// kMaxRatio, MEASURED (dev image, test pinned to 4 CPUs beside stress-ng,
// wall ratio per pair): correct pacing 0.247..0.277
// unloaded, up to 0.416 with 8 stress workers and 0.625 with 16; the factor
// ignored 0.834..1.228 (150 pairs, 0 to 16 workers); 0.75 lies between them.
// No bound rejects an effective 2x pace: the factor halved gave 0.484..0.721,
// and correct pairs under 16 workers reach 0.625, so a bound below 0.5 would
// fail correct runs on a loaded host. The read-back check rejects 2x instead.
// With 24 workers (6 per CPU) correct pairs reach 1.635 and 5 of 20 runs fail:
// such a host cannot show pacing.
constexpr double kMaxRatio = 0.75;
constexpr int kPairs = 3;

TEST_F(SimulatorPace, FactorFourRunsFourTimesFaster)
{
  std::string seen;
  for (int pair = 1; pair <= kPairs; ++pair) {
    const auto one = run_simulator({}, 800);
    ASSERT_EQ(one.last_clock_ns, 800'000'000) << "the 1x reference run did not reach step 800";
    expect_one_step_per_tick(one, "pair " + std::to_string(pair) + " at 1x");
    const auto r = run_simulator({rclcpp::Parameter("real_time_factor", 4.0)}, 800);
    ASSERT_EQ(r.last_clock_ns, 800'000'000) << "the simulator did not reach step 800";
    expect_one_step_per_tick(r, "pair " + std::to_string(pair) + " at real_time_factor 4");
    // Nominal 200 ms: load only slows a run, so this holds on any host.
    EXPECT_GE(r.wall_ms, 180.0) << "800 steps took " << r.wall_ms
                                << " ms of wall time at real_time_factor 4, faster than 4x";
    const double ratio = r.wall_ms / one.wall_ms;
    const std::string text = "pair " + std::to_string(pair) + ": wall " +
      std::to_string(r.wall_ms) + " / " + std::to_string(one.wall_ms) + " ms = " +
      std::to_string(ratio) + " (stall " + std::to_string(r.stall_ms) + " / " +
      std::to_string(one.stall_ms) + " ms, max gap " + std::to_string(r.max_gap_ms) + " / " +
      std::to_string(one.max_gap_ms) + " ms)";
    seen += (seen.empty() ? "" : "; ") + text;
    // Nominal ratio 0.25.
    EXPECT_LT(ratio, kMaxRatio) << "800 steps at real_time_factor 4 against 1x, " << text
                                << ": factor 4 did not run 4x faster than 1x";
  }
  RecordProperty("ratios", seen);
}

// The exact factor, independent of load: the simulator reads the installed
// timer period back from rcl (rcl_timer_get_period) and logs it, and factor 4
// must give 1 ms / 4. A partly applied factor (2x gives ratio 0.5, 1.5x 0.67)
// passes the ratio bound above; this check refuses it.
TEST_F(SimulatorPace, FactorFourSetsAQuarterMillisecondTick)
{
  char dir[] = "/tmp/simulator_pace_XXXXXX";
  ASSERT_NE(mkdtemp(dir), nullptr);
  const auto r = run_simulator(
    {rclcpp::Parameter("real_time_factor", 4.0), rclcpp::Parameter("log_dir", std::string(dir))},
    10);
  ASSERT_EQ(r.last_clock_ns, 10'000'000) << "the simulator did not reach step 10";
  expect_one_step_per_tick(r, "real_time_factor 4, 10 steps");
  const std::string path = std::string(dir) + "/simulator.jsonl";
  std::ifstream in(path);
  ASSERT_TRUE(in.good()) << "no simulator log at " << path;
  std::string line;
  std::string pace;
  while (std::getline(in, line)) {
    if (line.find("\"ev\":\"pace\"") != std::string::npos) {
      pace = line;
    }
  }
  std::remove(path.c_str());
  rmdir(dir);
  const std::string key = "\"tick_period_ns\":";
  const auto at = pace.find(key);
  ASSERT_NE(at, std::string::npos) << "the simulator logged no pace event at real_time_factor 4";
  EXPECT_EQ(std::stoll(pace.substr(at + key.size())), 250'000)
    << "the installed timer period at real_time_factor 4 is not 1 ms / 4: " << pace;
}

// As above, factor 0 is compared with a 1x run of the same 1000
// steps measured just before it in this test, not with a fixed wall time. A
// fixed 500 ms bound failed at 516 ms beside 16 stress workers. Every pair
// counts, no retry and no best of N; the /clock gaps are printed beside each
// ratio to explain a failure, never to excuse it.
//
// kMaxZeroRatio, MEASURED (dev image, test pinned to 4 CPUs beside
// stress-ng --cpu 16): correct factor 0 gave 0.044..0.202 (240
// pairs in two series of 40 runs; longest factor 0 wall 302 ms); a simulator
// that sleeps 1 ms per tick at factor 0 gave 0.934..1.112 (30 pairs, 10 runs,
// all failed). 0.5 lies between them, 2.5 times the worst correct pair and
// 1.9 times below the best sleeping one.
constexpr double kMaxZeroRatio = 0.5;

TEST_F(SimulatorPace, FactorZeroRunsBackToBack)
{
  std::string seen;
  for (int pair = 1; pair <= kPairs; ++pair) {
    const auto one = run_simulator({}, 1000);
    ASSERT_EQ(one.last_clock_ns, 1'000'000'000) << "the 1x reference run did not reach step 1000";
    expect_one_step_per_tick(one, "pair " + std::to_string(pair) + " at 1x");
    const auto r = run_simulator({rclcpp::Parameter("real_time_factor", 0.0)}, 1000);
    ASSERT_EQ(r.last_clock_ns, 1'000'000'000) << "the simulator did not reach step 1000";
    expect_one_step_per_tick(r, "pair " + std::to_string(pair) + " at real_time_factor 0");
    const double ratio = r.wall_ms / one.wall_ms;
    const std::string text = "pair " + std::to_string(pair) + ": wall " +
      std::to_string(r.wall_ms) + " / " + std::to_string(one.wall_ms) + " ms = " +
      std::to_string(ratio) + " (stall " + std::to_string(r.stall_ms) + " / " +
      std::to_string(one.stall_ms) + " ms, max gap " + std::to_string(r.max_gap_ms) + " / " +
      std::to_string(one.max_gap_ms) + " ms)";
    seen += (seen.empty() ? "" : "; ") + text;
    EXPECT_LT(ratio, kMaxZeroRatio) << "1000 steps at real_time_factor 0 against 1x, " << text
                                    << ": factor 0 did not run the ticks back to back";
  }
  RecordProperty("ratios", seen);
}

TEST_F(SimulatorPace, RejectsNegativeFactorAndGridMode)
{
  EXPECT_THROW(
    run_simulator({rclcpp::Parameter("real_time_factor", -1.0)}, 10), std::invalid_argument);
  EXPECT_THROW(
    run_simulator(
      {rclcpp::Parameter("real_time_factor", 2.0),
        rclcpp::Parameter("grid_origin_ns", static_cast<int64_t>(1))}, 10),
    std::invalid_argument);
}

// The kinematic plant was deleted. Asking for it, by
// parameter or by $BF_PLANT, must refuse to start, not run another plant.
TEST_F(SimulatorPace, RejectsKinematicPlant)
{
  try {
    run_simulator({rclcpp::Parameter("plant", std::string("kinematic"))}, 10);
    ADD_FAILURE() << "plant:=kinematic started a simulator, the kinematic plant still runs";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("plant must be mujoco"), std::string::npos) << e.what();
  }
  ASSERT_EQ(setenv("BF_PLANT", "kinematic", 1), 0);
  try {
    run_simulator({}, 10);
    ADD_FAILURE() << "BF_PLANT=kinematic started a simulator, the kinematic plant still runs";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("plant must be mujoco"), std::string::npos) << e.what();
  }
  ASSERT_EQ(unsetenv("BF_PLANT"), 0);
}

}  // namespace
