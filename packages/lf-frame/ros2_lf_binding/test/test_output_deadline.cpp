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

/// @file test_output_deadline.cpp
/// @brief The output deadline check against a pinned tag and a
/// pinned physical clock: late exactly when now - tag > bound, traced with
/// the lateness, counted, never for an unarmed (reaction, topic).

#include <gtest/gtest.h>

#include <rcutils/logging.h>

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "fake_trace_sink.hpp"
#include "handle_table.hpp"
#include "output_deadline.hpp"
#include "semantic_trace.hpp"
#include "thread_role.hpp"
#include "ros2_lf_binding/ros2_lf_binding.h"

extern "C" {
#include "core/environment.h"
#include "core/lf_types.h"
}

extern "C" int _lf_get_environments(environment_t ** envs)
{
  *envs = nullptr;
  return 0;
}

std::atomic<int64_t> g_fake_now{0};

extern "C" int64_t lf_time_physical(void) {return g_fake_now.load();}

namespace
{

namespace od = ros2_lf_binding::output_deadline;
namespace st = ros2_lf_binding::semtrace;

constexpr int64_t kTag = 1'000'000'000'000LL;  // an absolute tag, 1000 s
constexpr int64_t kBound = 20'000'000;          // 20 ms
const char * const kTopic = "/actuation";

class OutputDeadline : public ::testing::Test
{
protected:
  void SetUp() override
  {
    od::reset_for_test();
    st::reset_for_test();
    st::set_sink_for_test(&sink);
    st::begin(1);
    ros2_lf_binding::set_lf_tag_for_test(std::make_pair(kTag, 0u));
  }
  void TearDown() override
  {
    od::flush_pending();
    ros2_lf_binding::set_lf_tag_for_test(std::nullopt);
    st::set_sink_for_test(nullptr);
    st::reset_for_test();
    od::reset_for_test();
  }
  rcllf_test::FakeSink sink;
};

TEST_F(OutputDeadline, UnarmedTopicIsNeverChecked)
{
  g_fake_now = kTag + 10 * kBound;
  EXPECT_FALSE(od::check(3, kTopic));
  EXPECT_EQ(sink.count("actuation_late"), 0u);
  EXPECT_TRUE(od::stats().empty());
}

TEST_F(OutputDeadline, LateExactlyAboveTheBound)
{
  od::arm(3, kTopic, kBound);
  g_fake_now = kTag + kBound;  // lateness == bound: on time
  EXPECT_FALSE(od::check(3, kTopic));
  g_fake_now = kTag + kBound + 1;  // one ns over: late
  EXPECT_TRUE(od::check(3, kTopic));
  const auto recs = sink.of("actuation_late");
  ASSERT_EQ(recs.size(), 1u);
  EXPECT_EQ(recs[0].rid, 3);
  EXPECT_EQ(recs[0].topic, kTopic);
  EXPECT_EQ(recs[0].tag_time, kTag);
  EXPECT_EQ(recs[0].physical_ns, kTag + kBound + 1);
  EXPECT_EQ(recs[0].lateness_ns, kBound + 1);
  EXPECT_EQ(recs[0].bound_ns, kBound);
  const auto s = od::stats().at({3, kTopic});
  EXPECT_EQ(s.checked, 2u);
  EXPECT_EQ(s.late, 1u);
  EXPECT_EQ(s.max_lateness_ns, kBound + 1);
}

TEST_F(OutputDeadline, OtherReactionOrNoReactionIsNotChecked)
{
  od::arm(3, kTopic, kBound);
  g_fake_now = kTag + 10 * kBound;
  EXPECT_FALSE(od::check(4, kTopic)) << "the bound belongs to RID 3's publishes";
  EXPECT_FALSE(od::check(-1, kTopic)) << "a publish outside any reaction";
  EXPECT_FALSE(od::check(3, "/other"));
  EXPECT_EQ(sink.count("actuation_late"), 0u);
  EXPECT_EQ(od::stats().at({3, kTopic}).checked, 0u);
}

TEST_F(OutputDeadline, ZeroBoundMakesEveryRealTimePublishLate)
{
  od::arm(3, kTopic, 0);
  g_fake_now = kTag;  // the same instant as the tag: not late
  EXPECT_FALSE(od::check(3, kTopic));
  g_fake_now = kTag + 1;
  EXPECT_TRUE(od::check(3, kTopic));
}

TEST_F(OutputDeadline, AheadOfPhysicalTimeIsNeverLate)
{
  // Under --fast, logical time runs ahead of physical time: negative lateness.
  od::arm(3, kTopic, 0);
  g_fake_now = kTag - 5 * kBound;
  EXPECT_FALSE(od::check(3, kTopic));
  EXPECT_EQ(od::stats().at({3, kTopic}).max_lateness_ns, -5 * kBound);
}

TEST_F(OutputDeadline, ReArmReplacesTheBound)
{
  od::arm(3, kTopic, kBound);
  od::arm(3, kTopic, 2 * kBound);
  g_fake_now = kTag + kBound + 1;
  EXPECT_FALSE(od::check(3, kTopic));
}


// Every line that reaches rcutils' installed output handler: rcl's handler,
// which also publishes on /rosout, sits there in a running system.
std::vector<std::string> g_handler_lines;

void recording_handler(
  const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
  const char * format, va_list * args)
{
  char buf[512];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(buf, sizeof(buf), format, copy);
  va_end(copy);
  g_handler_lines.emplace_back(buf);
}

TEST_F(OutputDeadline, UnresolvedTopicWarningNeverReachesTheOutputHandler)
{
  // The arm's failure branch used RCLCPP_WARN, which goes
  // through the installed output handler and so can reach /rosout. A RID whose
  // node class is not registered takes that branch.
  ASSERT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
  ros2_lf_binding::g_handle_table.clear();
  ros2_lf_binding::g_handle_table.resize(1);
  ros2_lf_binding::g_handle_table[0].node_class = "NoSuchNodeClass779";
  g_handler_lines.clear();
  const rcutils_logging_output_handler_t saved = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(recording_handler);
  ros2_lf_output_deadline_arm(0, "actuation", kBound);
  rcutils_logging_set_output_handler(saved);
  ros2_lf_binding::g_handle_table.clear();
  for (const auto & line : g_handler_lines) {
    ADD_FAILURE() << "a bound-only line reached the output handler (and so /rosout): " << line;
  }
  EXPECT_EQ(od::stats().count({0, "actuation"}), 1u) << "still armed, as written";
}

}  // namespace
