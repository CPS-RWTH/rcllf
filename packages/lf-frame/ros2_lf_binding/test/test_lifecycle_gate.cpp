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
//

// The lifecycle gate model: a state observed at tag T
// holds for dispatches at tags after T, and every effective change bumps the
// epoch once.

#include <gtest/gtest.h>

#include "lifecycle_gate.hpp"

namespace
{

using ros2_lf_binding::LfTag;
using ros2_lf_binding::LifecycleGate;

constexpr uint8_t kUnconfigured = 1;
constexpr uint8_t kInactive = 2;
constexpr uint8_t kActive = 3;
constexpr uint8_t kFinalized = 4;
constexpr uint8_t kDeactivating = 14;

}  // namespace

TEST(LifecycleGate, ChangeTakesEffectAtTheNextTag)
{
  LifecycleGate gate(kActive);
  EXPECT_EQ(gate.at({100, 0}).state, kActive);
  gate.observe(kInactive, {100, 0});
  // The other reactions at tag (100, 0) still see the state before the change.
  EXPECT_EQ(gate.at({100, 0}).state, kActive);
  EXPECT_EQ(gate.at({100, 0}).epoch, 0u);
  EXPECT_EQ(gate.at({100, 1}).state, kInactive);
  EXPECT_EQ(gate.at({100, 1}).epoch, 1u);
  EXPECT_EQ(gate.at({200, 0}).state, kInactive);
  EXPECT_EQ(gate.at({200, 0}).epoch, 1u) << "a change bumps the epoch exactly once";
}

TEST(LifecycleGate, MicrostepOrdersTags)
{
  LifecycleGate gate(kInactive);
  gate.observe(kActive, {50, 3});
  EXPECT_EQ(gate.at({50, 2}).state, kInactive);
  EXPECT_EQ(gate.at({50, 3}).state, kInactive);
  EXPECT_EQ(gate.at({50, 4}).state, kActive);
}

TEST(LifecycleGate, TransitionStatesAreIgnored)
{
  LifecycleGate gate(kActive);
  gate.observe(kDeactivating, {10, 0});
  EXPECT_EQ(gate.at({20, 0}).state, kActive);
  EXPECT_EQ(gate.at({20, 0}).epoch, 0u);
}

TEST(LifecycleGate, RevertWithinOneTagStillReconciles)
{
  // Review finding 1: the hooks of a deactivate and activate at one tag may
  // have replaced the node's handles, so the epoch must move although the
  // state ends where it started.
  LifecycleGate gate(kActive);
  gate.observe(kInactive, {10, 0});
  gate.observe(kActive, {10, 0});
  EXPECT_EQ(gate.at({10, 0}).epoch, 0u);
  EXPECT_EQ(gate.at({11, 0}).state, kActive);
  EXPECT_EQ(gate.at({11, 0}).epoch, 1u) <<
    "deactivate then activate in one tag left the epoch, handles stay unreconciled";
}

TEST(LifecycleGate, TransitionStateAtStartIsNotActive)
{
  // Review finding 3: a node bound while activating starts gated, not active.
  LifecycleGate gate(13);
  EXPECT_NE(gate.at({0, 0}).state, kActive);
  gate.observe(kActive, {5, 0});
  EXPECT_EQ(gate.at({5, 1}).state, kActive);
}

TEST(LifecycleGate, LastObservationAtATagWins)
{
  LifecycleGate gate(kUnconfigured);
  gate.observe(kInactive, {10, 0});
  gate.observe(kActive, {10, 0});
  EXPECT_EQ(gate.at({10, 0}).state, kUnconfigured);
  EXPECT_EQ(gate.at({10, 1}).state, kActive);
  EXPECT_EQ(gate.at({10, 1}).epoch, 1u);
}

TEST(LifecycleGate, SuccessiveChangesEachBumpTheEpoch)
{
  LifecycleGate gate(kActive);
  gate.observe(kInactive, {10, 0});
  EXPECT_EQ(gate.at({20, 0}).epoch, 1u);
  gate.observe(kActive, {20, 0});
  // Observing at a later tag promotes the pending change without a read.
  gate.observe(kFinalized, {30, 0});
  EXPECT_EQ(gate.at({30, 0}).state, kActive);
  EXPECT_EQ(gate.at({30, 0}).epoch, 2u);
  EXPECT_EQ(gate.at({31, 0}).state, kFinalized);
  EXPECT_EQ(gate.at({31, 0}).epoch, 3u);
}

TEST(LifecycleGate, PrimaryStatePredicate)
{
  EXPECT_FALSE(ros2_lf_binding::is_primary_lifecycle_state(0));
  for (int s = 1; s <= 4; ++s) {
    EXPECT_TRUE(ros2_lf_binding::is_primary_lifecycle_state(s));
  }
  EXPECT_FALSE(ros2_lf_binding::is_primary_lifecycle_state(-1));
  EXPECT_FALSE(ros2_lf_binding::is_primary_lifecycle_state(kDeactivating));
}
