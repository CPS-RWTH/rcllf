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

/// @file test_match_probe.cpp
/// @brief The match probe survives a publisher struct copied after its init.
/// rcl does this for every node's rosout publisher, and a probe
/// that polled the address rcl_publisher_init saw crashed the reference system.
/// It also untracks entities finalized after the context shut down, and traces
/// a replacement reader's match as a new writer side increase.
/// On rmw_fastrtps it times matches from the rmw matched event callbacks, and
/// the subscription's wait set stays quiet afterwards.

#include <dlfcn.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rmw/rmw.h"
#include "std_msgs/msg/string.h"

extern "C" size_t rcllf_match_probe_tracked_for_test();
extern "C" uint64_t rcllf_match_probe_events_for_test(int reader);

namespace
{

class MatchProbe : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rcl_init_options_t opts = rcl_get_zero_initialized_init_options();
    ASSERT_EQ(RCL_RET_OK, rcl_init_options_init(&opts, rcl_get_default_allocator()));
    ASSERT_EQ(RCL_RET_OK, rcl_init(0, nullptr, &opts, &context_));
    ASSERT_EQ(RCL_RET_OK, rcl_init_options_fini(&opts));
    rcl_node_options_t node_opts = rcl_node_get_default_options();
    node_opts.enable_rosout = false;
    ASSERT_EQ(RCL_RET_OK, rcl_node_init(&node_, "match_probe_test", "", &context_, &node_opts));
  }

  void TearDown() override
  {
    EXPECT_EQ(RCL_RET_OK, rcl_node_fini(&node_));
    if (!shut_down_) {
      EXPECT_EQ(RCL_RET_OK, rcl_shutdown(&context_));
    }
    EXPECT_EQ(RCL_RET_OK, rcl_context_fini(&context_));
  }

  /// ctest stops this binary after TIMEOUT 60 (ros2_lf_binding CMakeLists.txt,
  /// test_match_probe). Keep the two in step.
  static constexpr std::chrono::seconds kCtestTimeout{60};

  /// Bound of a wait for a DDS match on a loaded host: a quarter of
  /// the ctest timeout. A test with two such waits that both run out still ends
  /// with its gtest message, inside the timeout, with half of it left for the
  /// rest of the suite (each other test takes well under a second).
  static constexpr std::chrono::seconds kMatchWait{kCtestTimeout / 4};

  /// Waits up to `bound` (3 s by default) for `pred`.
  template<typename Pred>
  static bool wait_for(Pred pred, std::chrono::milliseconds bound = std::chrono::seconds(3))
  {
    const auto end = std::chrono::steady_clock::now() + bound;
    while (!pred() && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
  }

  /// Events of one side and one source so far, or -1 when the probe has no
  /// per source count (looked up at run time, so the test reports what is missing).
  static int64_t source_events(int reader, int from_event)
  {
    using Fn = uint64_t (*)(int, int);
    auto fn = reinterpret_cast<Fn>(dlsym(RTLD_DEFAULT, "rcllf_match_probe_source_events_for_test"));
    return fn ? static_cast<int64_t>(fn(reader, from_event)) : -1;
  }

  static bool on_fastrtps()
  {
    return std::string(rmw_get_implementation_identifier()).rfind("rmw_fastrtps", 0) == 0;
  }

  /// rcl_wait on one subscription for 200 ms: RCL_RET_TIMEOUT when nothing woke it.
  rcl_ret_t wait_on(rcl_subscription_t * sub, rcl_event_t * event = nullptr)
  {
    rcl_wait_set_t ws = rcl_get_zero_initialized_wait_set();
    EXPECT_EQ(
      RCL_RET_OK, rcl_wait_set_init(
        &ws, 1, 0, 0, 0, 0, event ? 1 : 0, &context_, rcl_get_default_allocator()));
    EXPECT_EQ(RCL_RET_OK, rcl_wait_set_add_subscription(&ws, sub, nullptr));
    if (event) {
      EXPECT_EQ(RCL_RET_OK, rcl_wait_set_add_event(&ws, event, nullptr));
    }
    const rcl_ret_t ret = rcl_wait(&ws, RCL_MS_TO_NS(200));
    EXPECT_EQ(RCL_RET_OK, rcl_wait_set_fini(&ws));
    return ret;
  }

  bool shut_down_ = false;

  rcl_context_t context_ = rcl_get_zero_initialized_context();
  rcl_node_t node_ = rcl_get_zero_initialized_node();
};

TEST_F(MatchProbe, probe_is_in_the_call_path)
{
  Dl_info info{};
  ASSERT_NE(0, dladdr(dlsym(RTLD_DEFAULT, "rcl_publisher_init"), &info));
  EXPECT_NE(std::string::npos, std::string(info.dli_fname).find("rcllf_match_probe")) <<
    "rcl_publisher_init resolves to " << info.dli_fname << ", so this test checks nothing";
}

TEST_F(MatchProbe, survives_a_publisher_copied_after_init)
{
  const auto * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const rcl_publisher_options_t opts = rcl_publisher_get_default_options();
  // What rcl_logging_rosout_init_publisher_for_node does: init a local, then
  // copy it into the logger map. The local's storage is then reused.
  rcl_publisher_t local = rcl_get_zero_initialized_publisher();
  ASSERT_EQ(RCL_RET_OK, rcl_publisher_init(&local, &node_, ts, "/match_probe_copy", &opts));
  auto * kept = new rcl_publisher_t(local);
  std::memset(&local, 0xA5, sizeof(local));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));  // about 50 polls
  EXPECT_EQ(RCL_RET_OK, rcl_publisher_fini(kept, &node_));
  delete kept;
  // Polls after the fini of the copy must not touch the destroyed writer.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

TEST_F(MatchProbe, survives_a_subscription_copied_after_init)
{
  const auto * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const rcl_subscription_options_t opts = rcl_subscription_get_default_options();
  rcl_subscription_t local = rcl_get_zero_initialized_subscription();
  ASSERT_EQ(RCL_RET_OK, rcl_subscription_init(&local, &node_, ts, "/match_probe_copy", &opts));
  auto * kept = new rcl_subscription_t(local);
  std::memset(&local, 0xA5, sizeof(local));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(RCL_RET_OK, rcl_subscription_fini(kept, &node_));
  delete kept;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

TEST_F(MatchProbe, untracks_entities_finalized_after_the_context_shut_down)
{
  const auto * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const size_t before = rcllf_match_probe_tracked_for_test();
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
  ASSERT_EQ(RCL_RET_OK, rcl_publisher_init(&pub, &node_, ts, "/match_probe_shutdown", &pub_opts));
  ASSERT_EQ(
    RCL_RET_OK, rcl_subscription_init(&sub, &node_, ts, "/match_probe_shutdown", &sub_opts));
  ASSERT_EQ(rcllf_match_probe_tracked_for_test(), before + 2);
  ASSERT_EQ(RCL_RET_OK, rcl_shutdown(&context_));
  shut_down_ = true;
  // rclcpp finalizes its entities after shutdown in exactly this order.
  const rcl_ret_t sub_ret = rcl_subscription_fini(&sub, &node_);
  const rcl_ret_t pub_ret = rcl_publisher_fini(&pub, &node_);
  (void)sub_ret;
  (void)pub_ret;
  rcl_reset_error();
  EXPECT_EQ(rcllf_match_probe_tracked_for_test(), before) <<
    "an entity finalized after the context shut down is still polled through its freed rmw "
    "handle (use after free)";
}

TEST_F(MatchProbe, traces_a_replacement_reader_match_as_a_new_writer_increase)
{
  const auto * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const std::string topic = "/match_probe_replace_" + std::to_string(getpid());
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  ASSERT_EQ(RCL_RET_OK, rcl_publisher_init(&pub, &node_, ts, topic.c_str(), &pub_opts));
  const uint64_t w0 = rcllf_match_probe_events_for_test(0);
  const uint64_t r0 = rcllf_match_probe_events_for_test(1);
  size_t n = 0;
  rcl_subscription_t first = rcl_get_zero_initialized_subscription();
  ASSERT_EQ(RCL_RET_OK, rcl_subscription_init(&first, &node_, ts, topic.c_str(), &sub_opts));
  ASSERT_TRUE(wait_for([&] {return rcllf_match_probe_events_for_test(0) == w0 + 1;}));
  // The reader side is polled on its own: a reader finalized before its poll saw
  // the match never emits, and the r0 + 2 check below then fails at random.
  ASSERT_TRUE(wait_for([&] {return rcllf_match_probe_events_for_test(1) == r0 + 1;}));
  ASSERT_EQ(RCL_RET_OK, rcl_subscription_fini(&first, &node_));
  // The writer's matched count falls back to 0, then the replacement joins.
  ASSERT_TRUE(
    wait_for(
      [&] {
        return rcl_publisher_get_subscription_count(&pub, &n) == RCL_RET_OK && n == 0;
      }));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));  // the probe polls the 0
  rcl_subscription_t second = rcl_get_zero_initialized_subscription();
  ASSERT_EQ(RCL_RET_OK, rcl_subscription_init(&second, &node_, ts, topic.c_str(), &sub_opts));
  EXPECT_TRUE(wait_for([&] {return rcllf_match_probe_events_for_test(0) == w0 + 2;})) <<
    "the writer's match with the replacement reader was not traced: the probe kept the old "
    "count after the first reader left, so its losses get the old match time";
  EXPECT_TRUE(wait_for([&] {return rcllf_match_probe_events_for_test(1) == r0 + 2;}));
  EXPECT_EQ(RCL_RET_OK, rcl_subscription_fini(&second, &node_));
  EXPECT_EQ(RCL_RET_OK, rcl_publisher_fini(&pub, &node_));
}

TEST_F(MatchProbe, times_a_match_from_the_rmw_matched_event)
{
  if (!on_fastrtps()) {
    GTEST_SKIP() << "matched event callbacks are wired for rmw_fastrtps only";
  }
  const auto * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const std::string topic = "/match_probe_event_" + std::to_string(getpid());
  const int64_t w0 = source_events(0, 1);
  const int64_t r0 = source_events(1, 1);
  const int64_t wp0 = source_events(0, 0);
  ASSERT_NE(-1, w0) << "the probe counts no events per source: it has no rmw matched event "
    "path, so every match is timed by the 1 ms poll";
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
  ASSERT_EQ(RCL_RET_OK, rcl_publisher_init(&pub, &node_, ts, topic.c_str(), &pub_opts));
  ASSERT_EQ(RCL_RET_OK, rcl_subscription_init(&sub, &node_, ts, topic.c_str(), &sub_opts));
  // A fixed 3 s wait missed the writer's event once on a loaded CI
  // runner, so both waits use kMatchWait. The time taken goes into the XML.
  const auto start = std::chrono::steady_clock::now();
  const auto waited_ms = [&start] {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    };
  EXPECT_TRUE(wait_for([&] {return source_events(0, 1) == w0 + 1;}, kMatchWait)) <<
    "the writer's match was not traced from the rmw matched event in " << waited_ms() <<
    " ms; the 1 ms poll counted " << source_events(0, 0) - wp0 << " writer match(es) meanwhile";
  RecordProperty("writer_event_ms", std::to_string(waited_ms()));
  RecordProperty("writer_poll_events", std::to_string(source_events(0, 0) - wp0));
  EXPECT_TRUE(wait_for([&] {return source_events(1, 1) == r0 + 1;}, kMatchWait)) <<
    "the reader's match was not traced from the rmw matched event in " << waited_ms() << " ms";
  RecordProperty("reader_event_ms", std::to_string(waited_ms()));
  EXPECT_EQ(RCL_RET_OK, rcl_subscription_fini(&sub, &node_));
  EXPECT_EQ(RCL_RET_OK, rcl_publisher_fini(&pub, &node_));
}

/// rmw_fastrtps raises the matched count before it calls the
/// matched event callback. The probe's seam makes every callback wait for two
/// full poll sweeps, so the poll reads the raised count first, every run. The
/// match must still be traced from the event, never from the poll, on both
/// sides: the side whose callback was set before its peer existed, and the peer,
/// whose match can arrive while its own callback is being set (the callback
/// then runs from inside rmw_event_set_callback, and the poll used to treat the
/// entry as having no event path). Both orders of creation are checked.
TEST_F(MatchProbe, an_armed_match_the_poll_reads_first_is_traced_from_the_rmw_matched_event)
{
  if (!on_fastrtps()) {
    GTEST_SKIP() << "matched event callbacks are wired for rmw_fastrtps only";
  }
  using SeamFn = void (*)(int);
  auto poll_first =
    reinterpret_cast<SeamFn>(dlsym(RTLD_DEFAULT, "rcllf_match_probe_poll_first_for_test"));
  ASSERT_NE(nullptr, poll_first) <<
    "the probe has no seam to run the poll before the rmw matched event";
  ASSERT_NE(-1, source_events(0, 1)) << "the probe counts no events per source";
  struct SeamOff
  {
    SeamFn fn;
    ~SeamOff() {fn(0);}
  } seam_off{poll_first};
  const auto * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  for (const int reader : {0, 1}) {
    // `reader` set: the subscription comes first, else the publisher does.
    const char * first = reader ? "reader" : "writer";
    const std::string topic = "/match_probe_poll_first_" + std::string(first) + "_" +
      std::to_string(getpid());
    const int64_t e0[2] = {source_events(0, 1), source_events(1, 1)};
    const int64_t p0[2] = {source_events(0, 0), source_events(1, 0)};
    rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
    rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
    if (reader) {
      ASSERT_EQ(RCL_RET_OK, rcl_subscription_init(&sub, &node_, ts, topic.c_str(), &sub_opts));
    } else {
      ASSERT_EQ(RCL_RET_OK, rcl_publisher_init(&pub, &node_, ts, topic.c_str(), &pub_opts));
    }
    poll_first(1);
    if (reader) {
      ASSERT_EQ(RCL_RET_OK, rcl_publisher_init(&pub, &node_, ts, topic.c_str(), &pub_opts));
    } else {
      ASSERT_EQ(RCL_RET_OK, rcl_subscription_init(&sub, &node_, ts, topic.c_str(), &sub_opts));
    }
    const auto events = [&](int r) {return source_events(r, 1) - e0[r];};
    const auto polls = [&](int r) {return source_events(r, 0) - p0[r];};
    for (const int r : {reader, 1 - reader}) {
      const char * side = r ? "reader" : "writer";
      ASSERT_TRUE(wait_for([&] {return events(r) + polls(r) >= 1;}, kMatchWait)) <<
        "the " << side << "'s match was not traced at all in " << kMatchWait.count() <<
        " s (" << first << " created first)";
      // The held callback runs after the poll; give it the time to record.
      wait_for([&] {return events(r) >= 1;});
      EXPECT_TRUE(events(r) == 1 && polls(r) == 0) <<
        "the " << side << "'s match was recorded with source poll " << polls(r) <<
        " time(s) and from the rmw matched event " << events(r) << " time(s), " << first <<
        " created first: the poll read the raised count before the callback ran and took "
        "the match from it";
    }
    poll_first(0);
    EXPECT_EQ(RCL_RET_OK, rcl_subscription_fini(&sub, &node_));
    EXPECT_EQ(RCL_RET_OK, rcl_publisher_fini(&pub, &node_));
  }
}

TEST_F(MatchProbe, the_wait_check_sees_an_enabled_matched_status)
{
  // What the first probe did: init the matched event, never take
  // it. The subscription's wait set then returns at once. This checks that the
  // next test's method can see that state at all.
  if (!on_fastrtps()) {
    GTEST_SKIP() << "the shared status condition is a rmw_fastrtps detail";
  }
  const auto * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const std::string topic = "/match_probe_spin_ref_" + std::to_string(getpid());
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
  ASSERT_EQ(RCL_RET_OK, rcl_subscription_init(&sub, &node_, ts, topic.c_str(), &sub_opts));
  rcl_event_t event = rcl_get_zero_initialized_event();
  ASSERT_EQ(RCL_RET_OK, rcl_subscription_event_init(&event, &sub, RCL_SUBSCRIPTION_MATCHED));
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  ASSERT_EQ(RCL_RET_OK, rcl_publisher_init(&pub, &node_, ts, topic.c_str(), &pub_opts));
  size_t n = 0;
  ASSERT_TRUE(
    wait_for(
      [&] {return rcl_subscription_get_publisher_count(&sub, &n) == RCL_RET_OK && n == 1;}));
  EXPECT_EQ(RCL_RET_OK, wait_on(&sub)) <<
    "an untaken matched status did not wake the wait: the quiet wait test proves nothing";
  EXPECT_EQ(RCL_RET_OK, rcl_event_fini(&event));
  EXPECT_EQ(RCL_RET_OK, rcl_publisher_fini(&pub, &node_));
  EXPECT_EQ(RCL_RET_OK, rcl_subscription_fini(&sub, &node_));
}

TEST_F(MatchProbe, the_subscription_wait_stays_quiet_after_a_match)
{
  const auto * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const std::string topic = "/match_probe_quiet_" + std::to_string(getpid());
  rcl_publisher_options_t pub_opts = rcl_publisher_get_default_options();
  rcl_subscription_options_t sub_opts = rcl_subscription_get_default_options();
  rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
  ASSERT_EQ(RCL_RET_OK, rcl_subscription_init(&sub, &node_, ts, topic.c_str(), &sub_opts));
  const uint64_t r0 = rcllf_match_probe_events_for_test(1);
  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  ASSERT_EQ(RCL_RET_OK, rcl_publisher_init(&pub, &node_, ts, topic.c_str(), &pub_opts));
  ASSERT_TRUE(wait_for([&] {return rcllf_match_probe_events_for_test(1) == r0 + 1;}));
  EXPECT_EQ(RCL_RET_TIMEOUT, wait_on(&sub)) <<
    "the probe left the subscription's wait set ready after the match: a stock executor "
    "spins";
  EXPECT_EQ(RCL_RET_OK, rcl_publisher_fini(&pub, &node_));
  EXPECT_EQ(RCL_RET_OK, rcl_subscription_fini(&sub, &node_));
}

}  // namespace
