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

/// @file test_wheel_startup_timeout.cpp
/// @brief Stock: a /wheel/speed stream that never starts is reported.
///
/// The DDS deadline counts per instance and an instance exists only after its
/// first sample, so stock ROS 2 raises nothing for a wheel speed node that
/// never publishes. The controller's startup timeout logs the stock deadline
/// record for every kWheelDeadline window instead. This binary does not link
/// the rcllf binding, so the controller runs as stock.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "brake_fusion/controller_node.hpp"
#include "brake_fusion/loss_events.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

std::string make_log_dir()
{
  std::string tmpl = ::testing::TempDir() + "bf_wheel_startup_XXXXXX";
  std::vector<char> buf(tmpl.begin(), tmpl.end());
  buf.push_back('\0');
  const char * dir = ::mkdtemp(buf.data());
  if (dir == nullptr) {
    throw std::runtime_error("mkdtemp failed");
  }
  return dir;
}

int64_t field(const std::string & line, const std::string & name)
{
  std::smatch m;
  const std::regex re("\"" + name + "\":(-?[0-9]+)");
  if (!std::regex_search(line, m, re)) {
    throw std::runtime_error("no " + name + " in " + line);
  }
  return std::stoll(m[1].str());
}

TEST(StartupWindows, CountsTheWindowsThatEnded)
{
  constexpr int64_t kD = 25000000;
  EXPECT_EQ(brake_fusion::startup_windows_elapsed(100, 100, kD), 0);
  EXPECT_EQ(brake_fusion::startup_windows_elapsed(100, 100 + kD - 1, kD), 0);
  EXPECT_EQ(brake_fusion::startup_windows_elapsed(100, 100 + kD, kD), 1);
  EXPECT_EQ(brake_fusion::startup_windows_elapsed(100, 100 + 3 * kD + 7, kD), 3);
  EXPECT_EQ(brake_fusion::startup_windows_elapsed(100, 50, kD), 0);
  EXPECT_EQ(brake_fusion::startup_windows_elapsed(0, 10 * kD, 0), 0);
}

TEST(WheelStartupTimeout, ANeverStartedWheelStreamIsReportedEveryDeadline)
{
  rclcpp::init(0, nullptr);
  const std::string dir = make_log_dir();
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("log_dir", dir)});
  {
    auto node = std::make_shared<brake_fusion::ControllerNode>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // No /wheel/speed publisher: 300 ms is 12 controller ticks (25 ms) and
    // 11 kWheelDeadline windows after the first tick, at least 8 on a slow host.
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < until) {
      executor.spin_some(std::chrono::milliseconds(5));
    }
  }
  rclcpp::shutdown();
  std::ifstream in(dir + "/controller.jsonl");
  ASSERT_TRUE(in.good()) << "no controller log in " << dir;
  std::vector<std::string> records;
  std::string line;
  while (std::getline(in, line)) {
    if (line.find("\"ev\":\"deadline\"") != std::string::npos &&
      line.find("\"topic\":\"/wheel/speed\"") != std::string::npos)
    {
      records.push_back(line);
    }
  }
  ASSERT_GE(records.size(), 4u)
    << "no deadline record for a /wheel/speed stream that never started (stock)";
  // Same shape as the DDS deadline record; totals count windows, the changes
  // add up to them, and no window is reported before one D has passed.
  int64_t total = 0;
  for (const auto & r : records) {
    EXPECT_NE(r.find("\"ctx\":false"), std::string::npos) << r;
    const int64_t change = field(r, "change");
    EXPECT_GE(change, 1) << r;
    total += change;
    EXPECT_EQ(field(r, "total"), total) << r;
  }
  const int64_t d_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    brake_fusion::kWheelDeadline).count();
  const int64_t first_ns = field(records.front(), "t_ns");
  const int64_t last_ns = field(records.back(), "t_ns");
  // From the first record to the last, total - total_first windows ended;
  // the records come on 25 ms ticks, so allow one D of tick phase.
  EXPECT_GE(last_ns - first_ns, (total - field(records.front(), "total")) * d_ns - d_ns)
    << "more windows reported than deadlines elapsed";
}

}  // namespace
