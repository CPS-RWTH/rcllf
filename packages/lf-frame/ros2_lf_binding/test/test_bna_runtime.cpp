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


// Binding side of the BNA synchronizer: registration in
// the construction scope, the drain dispatch context, wake bookkeeping,
// the missing wiring of fix 6 and the close hook. Outside an LF program the
// current tag is {-1, 0}, so every delay below is the deadline plus one.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "bna_runtime.hpp"
#include "corr_runtime.hpp"
#include "ros2_lf_binding/bna_runtime_api.h"
#include "ros2_lf_binding/node_registry.h"
// Both declarations of every hook in one translation unit: a signature drift
// is a conflicting declaration and fails to compile.
#include "../../rcllf_message_filters/include/rcllf_message_filters/bna_runtime_hooks.h"

extern "C" size_t rcllf_bna_policy_layout(int i);

namespace
{

struct Calls
{
  std::vector<std::string> log;
};

void on_missing(
  void * ctx, int input, uint64_t epoch, const char * gid, uint64_t ordinal, int64_t time,
  uint32_t micro)
{
  static_cast<Calls *>(ctx)->log.push_back(
    "missing " + std::to_string(input) + " " + std::to_string(epoch) + " " + gid + " " +
    std::to_string(ordinal) + " " + std::to_string(time) + "," + std::to_string(micro));
}

void on_missing_stamped(
  void * ctx, int input, uint64_t epoch, const char * gid, uint64_t ordinal, int64_t stamp,
  int64_t time, uint32_t micro)
{
  static_cast<Calls *>(ctx)->log.push_back(
    "stamped " + std::to_string(input) + " " + std::to_string(epoch) + " " + gid + " " +
    std::to_string(ordinal) + " " + std::to_string(stamp) + " " + std::to_string(time) + "," +
    std::to_string(micro));
}

// What a policy built against an older header leaves behind its 4-pointer
// struct: memory the binding must never read as a hook.
void on_garbage(
  void * ctx, int, uint64_t, const char *, uint64_t, int64_t stamp, int64_t, uint32_t)
{
  static_cast<Calls *>(ctx)->log.push_back("garbage hook called, stamp " + std::to_string(stamp));
}

void on_wake(void * ctx, int64_t time, uint32_t micro)
{
  static_cast<Calls *>(ctx)->log.push_back(
    "wake " + std::to_string(time) + "," + std::to_string(micro));
  // A hook may request the next wake (the engine does so after a deadline).
  ros2_lf_bna_request_wake(0, 500);
}

void on_close(void * ctx, const char * reason, int64_t time, uint32_t micro)
{
  static_cast<Calls *>(ctx)->log.push_back(
    std::string("close ") + reason + " " + std::to_string(time) + "," + std::to_string(micro));
}

class BnaRuntime : public ::testing::Test
{
protected:
  void SetUp() override {ros2_lf_binding::bna_rt::reset_for_test();}
  void TearDown() override {ros2_lf_binding::bna_rt::reset_for_test();}

  int register_node(const std::string & key)
  {
    Ros2LfNodeConstructionScope scope(key);
    hooks_ = rcllf_bna_hooks{&calls_, &on_missing, &on_wake, &on_close};
    return ros2_lf_bna_register(&hooks_, &epoch_);
  }

  // A current policy registers the 4-pointer struct, then routes
  // reports with a stamp bound to `fn`. RCLLF_BNA_FALSIFY_OLD_API builds this
  // test against the first draft API, where the hook was a fifth struct field.
  int register_stamped(const std::string & key, rcllf_bna_missing_stamped_fn fn)
  {
    Ros2LfNodeConstructionScope scope(key);
#ifdef RCLLF_BNA_FALSIFY_OLD_API
    hooks_ = rcllf_bna_hooks{&calls_, &on_missing, &on_wake, &on_close, fn};
    return ros2_lf_bna_register(&hooks_, &epoch_);
#else
    hooks_ = rcllf_bna_hooks{&calls_, &on_missing, &on_wake, &on_close};
    const int h = ros2_lf_bna_register(&hooks_, &epoch_);
    ros2_lf_bna_set_missing_stamped(h, fn);
    return h;
#endif
  }

  Calls calls_;
  rcllf_bna_hooks hooks_{};
  uint64_t epoch_ = 0;
};

TEST(BnaRuntimeLayout, PolicyAndBindingAgreeOnTheStructs)
{
  const size_t mine[] = {
    sizeof(rcllf_bna_hooks), offsetof(rcllf_bna_hooks, missing),
    offsetof(rcllf_bna_hooks, wake), offsetof(rcllf_bna_hooks, close),
    sizeof(rcllf_bna_dispatch), offsetof(rcllf_bna_dispatch, micro),
    offsetof(rcllf_bna_dispatch, epoch), offsetof(rcllf_bna_dispatch, ordinal),
    offsetof(rcllf_bna_dispatch, gid), 43210};
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(rcllf_bna_policy_layout(i), mine[i]) << "field " << i;
  }
}

TEST_F(BnaRuntime, RegistersOnlyInsideAConstructionScopeAndOncePerNode)
{
  EXPECT_EQ(ros2_lf_bna_active(), 0);
  EXPECT_EQ(ros2_lf_bna_register(&hooks_, &epoch_), RCLLF_BNA_NOT_CONSTRUCTING);
  {
    Ros2LfNodeConstructionScope scope("fusion");
    EXPECT_EQ(ros2_lf_bna_active(), 1);
  }
  EXPECT_EQ(register_node("fusion"), 0);
  EXPECT_EQ(epoch_, ros2_lf_binding::corr_rt::table().epoch());
  EXPECT_EQ(register_node("fusion"), RCLLF_BNA_MULTIPLE);
  EXPECT_EQ(register_node("other"), 1);
}

TEST_F(BnaRuntime, CurrentIsTheDeclaredDrainDispatchOnly)
{
  const int h = register_node("fusion");
  ASSERT_EQ(h, 0);
  rcllf_bna_dispatch d{};
  EXPECT_EQ(ros2_lf_bna_current(h, 1, &d), RCLLF_BNA_NOT_DRAIN);
  ros2_lf_bna_declare_input("fusion", 7, 1);
  {
    ros2_lf_binding::bna_rt::DispatchScope s(7, "0102ab", 5);
    ASSERT_EQ(ros2_lf_bna_current(h, 1, &d), RCLLF_BNA_OK);
    EXPECT_EQ(std::string(d.gid), "0102ab");
    EXPECT_EQ(d.ordinal, 5u);
    EXPECT_EQ(d.epoch, epoch_);
    EXPECT_EQ(d.time, -1);
    EXPECT_EQ(ros2_lf_bna_current(h, 0, &d), RCLLF_BNA_WRONG_INPUT);
  }
  {
    ros2_lf_binding::bna_rt::DispatchScope s(8, "0102ab", 6);  // undeclared drain
    EXPECT_EQ(ros2_lf_bna_current(h, 1, &d), RCLLF_BNA_WRONG_INPUT);
  }
  EXPECT_EQ(ros2_lf_bna_current(h, 1, &d), RCLLF_BNA_NOT_DRAIN);
}

TEST_F(BnaRuntime, EachDeadlineIsScheduledOnceInOrder)
{
  const int h = register_node("fusion");
  ros2_lf_bna_declare_input("fusion", 7, 0);
  {
    ros2_lf_binding::bna_rt::DispatchScope s(7, "aa", 1);
    ros2_lf_bna_request_wake(h, 300);
    ros2_lf_bna_request_wake(h, 100);
    ros2_lf_bna_request_wake(h, 100);
  }
  EXPECT_EQ(ros2_lf_bna_take_wake("fusion"), 101);
  EXPECT_EQ(ros2_lf_bna_take_wake("fusion"), 301);
  EXPECT_EQ(ros2_lf_bna_take_wake("fusion"), -1);
  {
    ros2_lf_binding::bna_rt::DispatchScope s(7, "aa", 2);
    ros2_lf_bna_request_wake(h, 100);  // already scheduled: no second wake
  }
  EXPECT_EQ(ros2_lf_bna_take_wake("fusion"), -1);
  EXPECT_EQ(ros2_lf_bna_take_wake("nobody"), -1);
  ros2_lf_bna_wake("fusion");  // the hook requests 500 from inside the reaction
  EXPECT_EQ(ros2_lf_bna_take_wake("fusion"), 501);
  ASSERT_EQ(calls_.log.size(), 1u);
  EXPECT_EQ(calls_.log[0], "wake -1,0");
}

TEST_F(BnaRuntime, MissingReachesTheDeclaredInputOnly)
{
  register_node("fusion");
  ros2_lf_bna_declare_input("fusion", 7, 1);
  EXPECT_TRUE(ros2_lf_binding::bna_rt::on_missing(7, "0a0b", 9));
  EXPECT_FALSE(ros2_lf_binding::bna_rt::on_missing(8, "0a0b", 10));
  ASSERT_EQ(calls_.log.size(), 1u);
  EXPECT_EQ(calls_.log[0], "missing 1 " + std::to_string(epoch_) + " 0a0b 9 -1,0");
}

constexpr int64_t kMs = 1000000;

TEST_F(BnaRuntime, ALostReportCarriesThePublishTagMinusTheDeclaredLag)
{
  // The relay counterexample: a relay stamps 20 ms before it
  // publishes. The lost message published at 1000 ms has stamp 980 ms, so the
  // engine may raise its watermark to 980 ms, never to the publish tag.
  register_stamped("fusion", &on_missing_stamped);
  ros2_lf_bna_declare_input("fusion", 7, 1);
  ros2_lf_bna_input_stamp_lag(7, 20 * kMs);
  EXPECT_TRUE(ros2_lf_binding::bna_rt::on_missing(7, "0a0b", 9, 1000 * kMs));
  ASSERT_EQ(calls_.log.size(), 1u);
  EXPECT_EQ(calls_.log[0], "stamped 1 " + std::to_string(epoch_) + " 0a0b 9 980000000 -1,0");
}

TEST_F(BnaRuntime, APolicyWithTheOldFourPointerStructKeepsThePlainReport)
{
  // A policy compiled against a header without the stamped hook
  // passes exactly four pointers. Whatever follows them in its memory is not
  // a hook, so the report must reach `missing`.
  struct FourPointers
  {
    void * ctx;
    decltype(&on_missing) missing;
    decltype(&on_wake) wake;
    decltype(&on_close) close;
  };
  struct OldPolicyMemory
  {
    FourPointers hooks;
    decltype(&on_garbage) next_member;
  };
  static_assert(offsetof(OldPolicyMemory, next_member) == 4 * sizeof(void *), "layout");
  OldPolicyMemory mem{{&calls_, &on_missing, &on_wake, &on_close}, &on_garbage};
  {
    Ros2LfNodeConstructionScope scope("fusion");
    const auto * old = reinterpret_cast<const rcllf_bna_hooks *>(&mem.hooks);
    ASSERT_EQ(ros2_lf_bna_register(old, &epoch_), 0);
  }
  ros2_lf_bna_declare_input("fusion", 7, 1);
  ros2_lf_bna_input_stamp_lag(7, 0);
  EXPECT_TRUE(ros2_lf_binding::bna_rt::on_missing(7, "0a0b", 9, 1000 * kMs));
  ASSERT_EQ(calls_.log.size(), 1u);
  EXPECT_EQ(calls_.log[0], "missing 1 " + std::to_string(epoch_) + " 0a0b 9 -1,0");
}

TEST_F(BnaRuntime, WithoutABoundTheReportStaysPlain)
{
  // No declared lag (lf_clock alone), an unknown publish tag, a
  // bound below zero, or no stamped hook (nullptr) each take the plain path.
  const int h = register_stamped("fusion", &on_missing_stamped);
  ros2_lf_bna_declare_input("fusion", 7, 1);
  ros2_lf_bna_declare_input("fusion", 8, 0);
  ros2_lf_bna_input_stamp_lag(8, 5 * kMs);
  namespace b = ros2_lf_binding::bna_rt;
  EXPECT_TRUE(b::on_missing(7, "0a0b", 1, 1000 * kMs));  // no lag declared
  EXPECT_TRUE(b::on_missing(8, "0a0b", 2, -1));          // publish tag unknown
  EXPECT_TRUE(b::on_missing(8, "0a0b", 3, 4 * kMs));     // bound below zero
  EXPECT_TRUE(b::on_missing(8, "0a0b", 4, 1000 * kMs));  // stamped: 995 ms
#ifndef RCLLF_BNA_FALSIFY_OLD_API
  ros2_lf_bna_set_missing_stamped(h, nullptr);
  EXPECT_TRUE(b::on_missing(8, "0a0b", 5, 1000 * kMs));  // hook removed
#else
  (void)h;
  calls_.log.push_back("missing 0 " + std::to_string(epoch_) + " 0a0b 5 -1,0");
#endif
  const std::string e = std::to_string(epoch_);
  const std::vector<std::string> want = {
    "missing 1 " + e + " 0a0b 1 -1,0", "missing 0 " + e + " 0a0b 2 -1,0",
    "missing 0 " + e + " 0a0b 3 -1,0", "stamped 0 " + e + " 0a0b 4 995000000 -1,0",
    "missing 0 " + e + " 0a0b 5 -1,0"};
  EXPECT_EQ(calls_.log, want);
}

TEST_F(BnaRuntime, CloseThenUnregisterSilencesEveryHook)
{
  const int h = register_node("fusion");
  ros2_lf_bna_declare_input("fusion", 7, 1);
  ros2_lf_bna_close("fusion");
  ASSERT_EQ(calls_.log.size(), 1u);
  EXPECT_EQ(calls_.log[0], "close shutdown -1,0");
  ros2_lf_bna_unregister(h);
  ros2_lf_bna_close("fusion");
  ros2_lf_bna_wake("fusion");
  EXPECT_FALSE(ros2_lf_binding::bna_rt::on_missing(7, "0a0b", 9));
  EXPECT_EQ(calls_.log.size(), 1u);
  EXPECT_EQ(register_node("fusion"), 1);  // the node may register again
}

TEST_F(BnaRuntime, AStaleHandleDoesNotReachTheReplacementNode)
{
  const int old_h = register_node("fusion");
  ros2_lf_bna_declare_input("fusion", 7, 1);
  ros2_lf_bna_unregister(old_h);
  const int h = register_node("fusion");
  ASSERT_EQ(h, 1);
  {
    ros2_lf_binding::bna_rt::DispatchScope s(7, "aa", 1);
    rcllf_bna_dispatch d{};
    EXPECT_EQ(ros2_lf_bna_current(old_h, 1, &d), RCLLF_BNA_WRONG_INPUT);
    EXPECT_EQ(ros2_lf_bna_current(h, 1, &d), RCLLF_BNA_OK);
    ros2_lf_bna_request_wake(old_h, 100);
  }
  EXPECT_EQ(ros2_lf_bna_take_wake("fusion"), -1);
  ros2_lf_bna_unregister(old_h);
  EXPECT_EQ(register_node("fusion"), RCLLF_BNA_MULTIPLE);  // the replacement stays registered
}

TEST_F(BnaRuntime, AWakeRequestOutsideADrainOrHookExitsThree)
{
  const int h = register_node("fusion");
  EXPECT_EXIT(
    ros2_lf_bna_request_wake(h, 100), ::testing::ExitedWithCode(3),
    "RCLLF_BNA refused: wake requested outside a declared drain or BNA reaction");
}

TEST(CorrReleaseTag, OverflowIsRefusedAndLeavesTheOutputsAlone)
{
  // Delay plus the deferred extra plus now must not
  // wrap. One checked sum feeds both the trace tag and the table.
  namespace rt = ros2_lf_binding::corr_rt;
  int64_t wait = -7, release = -7;
  ASSERT_TRUE(rt::release_tag(1000, 10, 100, &wait, &release));
  EXPECT_EQ(wait, 110);
  EXPECT_EQ(release, 1110);
  wait = release = -7;
  EXPECT_FALSE(rt::release_tag(INT64_MAX - 50, 10, 100, &wait, &release));
  EXPECT_FALSE(rt::release_tag(0, INT64_MAX, 1, &wait, &release));
  EXPECT_EQ(wait, -7);
  EXPECT_EQ(release, -7);
}

}  // namespace
