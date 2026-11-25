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


/// @file test_input_period.cpp
/// @brief The declared minimum input period is checked at run time.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "input_period.hpp"
#include "ros2_lf_binding/rid_table_api.h"

namespace ip = ros2_lf_binding::input_period;

namespace
{
constexpr int64_t kMs = 1'000'000;

class InputPeriod : public ::testing::Test
{
protected:
  void SetUp() override {ip::reset_for_test();}
  void TearDown() override {ip::reset_for_test();}
};
}  // namespace

TEST_F(InputPeriod, RunsAtTheDeclaredPeriodAreClean)
{
  ros2_lf_binding::register_min_input_period("Fusion", "/radar", 1 * kMs);
  ip::attach(7, "Fusion", "/radar");
  for (int i = 0; i < 5; ++i) {
    ip::observe(7, i * kMs, 0, "w:" + std::to_string(i));
  }
  const auto s = ip::summaries();
  ASSERT_EQ(s.size(), 1u);
  EXPECT_EQ(s[0].runs, 5u);
  EXPECT_EQ(s[0].violations, 0u);
  EXPECT_EQ(s[0].min_gap_ns, 1 * kMs);
}

TEST_F(InputPeriod, EveryFasterRunIsCountedWithTheFirstIdentities)
{
  ros2_lf_binding::register_min_input_period("Fusion", "/radar", 1 * kMs);
  ip::attach(7, "Fusion", "/radar");
  ip::observe(7, 0, 0, "w:1");
  ip::observe(7, 1 * kMs, 0, "w:2");
  ip::observe(7, 1 * kMs + 400'000, 0, "w:3");  // 0.4 ms after w:2
  ip::observe(7, 1 * kMs + 400'000, 1, "w:4");  // same time, next microstep
  ip::observe(7, 3 * kMs, 0, "w:5");
  const auto s = ip::summaries();
  ASSERT_EQ(s.size(), 1u);
  EXPECT_EQ(s[0].runs, 5u);
  EXPECT_EQ(s[0].violations, 2u) <<
    "a run closer than the declared min_input_period went uncounted";
  EXPECT_EQ(s[0].min_gap_ns, 0);
  EXPECT_EQ(s[0].first_identity, "w:3");
  EXPECT_EQ(s[0].first_prev_identity, "w:2");
  EXPECT_EQ(s[0].first_time, 1 * kMs + 400'000);
  EXPECT_EQ(s[0].first_prev_time, 1 * kMs);
}

TEST_F(InputPeriod, OnlyDeclaredSubscriptionsAreWatched)
{
  ros2_lf_binding::register_min_input_period("Fusion", "/radar", 1 * kMs);
  ip::attach(7, "Fusion", "/camera");
  ip::attach(8, "Planner", "/radar");
  ip::observe(7, 0, 0, "a");
  ip::observe(7, 0, 1, "b");
  EXPECT_TRUE(ip::summaries().empty());
}

TEST(InputPeriodIdentity, IsGidHexAndSequence)
{
  const uint8_t gid[3] = {0x01, 0xab, 0xff};
  EXPECT_EQ(ip::identity(gid, 3, 42), "01abff:42");
}
