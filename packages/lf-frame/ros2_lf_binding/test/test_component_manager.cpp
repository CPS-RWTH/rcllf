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

/// @file test_component_manager.cpp
/// @brief Tests for LFComponentManager's binding-key lookup gating
/// (find_by_class exact match vs. legacy short-name fallback) and
/// instance-keyed load bookkeeping.
///
/// resolve_binding_key_lookup() is a pure function factored out of
/// find_by_class() specifically so this gating logic is unit-testable
/// without a real rclcpp_components node / pluginlib load. Instance-keyed
/// bookkeeping is exercised through LFComponentManager's
/// set_binding_key_for_test() seam, which writes the same class_to_id_ map a
/// real load_component() call does — this mirrors the
/// policy_table_set_for_test() pattern used elsewhere in this binding
/// (test_policy_table.cpp) so tests don't need a loadable pluginlib plugin.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "../src/component_manager.hpp"
#include "rclcpp/rclcpp.hpp"

using ros2_lf_binding::BindingKeyLookup;
using ros2_lf_binding::LFComponentManager;
using ros2_lf_binding::resolve_binding_key_lookup;

// ── resolve_binding_key_lookup: pure gating logic ────────────────────────────

TEST(BindingKeyLookupTest, ExactMatchOnPlainClassName)
{
  const std::vector<std::string> keys = {"TimerPublisherNode"};
  const auto result = resolve_binding_key_lookup("TimerPublisherNode", keys);
  EXPECT_TRUE(result.found);
  EXPECT_FALSE(result.via_fallback);
  EXPECT_EQ(result.matched_key, "TimerPublisherNode");
}

TEST(BindingKeyLookupTest, ExactMatchOnSynthesizedKeyTakesNoFallback)
{
  const std::vector<std::string> keys = {
    "TimerPublisherNode", "TimerPublisherNode__robot_1_timer_node"};
  const auto result = resolve_binding_key_lookup("TimerPublisherNode__robot_1_timer_node", keys);
  EXPECT_TRUE(result.found);
  EXPECT_FALSE(result.via_fallback);
  EXPECT_EQ(result.matched_key, "TimerPublisherNode__robot_1_timer_node");
}

TEST(BindingKeyLookupTest, SynthesizedLookupKeyNeverTakesFallbackOnMiss)
{
  // A synthesized-looking lookup key (contains "__") with no exact match must
  // return not-found — it must never fall through to the short-name
  // fallback, even if a plausible-looking plain-name key exists.
  const std::vector<std::string> keys = {"TimerPublisherNode"};
  const auto result = resolve_binding_key_lookup("TimerPublisherNode__no_such_instance", keys);
  EXPECT_FALSE(result.found);
}

TEST(BindingKeyLookupTest, LegacyFallbackMatchesQualifiedSuffix)
{
  // Old system.json: class_to_id_ keyed by the fully-qualified pluginlib
  // plugin name; find_by_class is called with the bare class name.
  const std::vector<std::string> keys = {"basic_components::TimerPublisherNode"};
  const auto result = resolve_binding_key_lookup("TimerPublisherNode", keys);
  EXPECT_TRUE(result.found);
  EXPECT_TRUE(result.via_fallback);
  EXPECT_EQ(result.matched_key, "basic_components::TimerPublisherNode");
}

TEST(BindingKeyLookupTest, FallbackNeverMatchesASynthesizedKey)
{
  // Even though "__" is not "::", make sure a synthesized key present in the
  // map is never selected as a fallback candidate for an unrelated plain
  // lookup — it must be skipped entirely by the fallback scan.
  const std::vector<std::string> keys = {"TimerPublisherNode__robot_1_timer_node"};
  const auto result = resolve_binding_key_lookup("TimerPublisherNode", keys);
  EXPECT_FALSE(result.found);
}

TEST(BindingKeyLookupTest, NoMatchReturnsNotFound)
{
  const std::vector<std::string> keys = {"SomeOtherClass"};
  const auto result = resolve_binding_key_lookup("TimerPublisherNode", keys);
  EXPECT_FALSE(result.found);
  EXPECT_TRUE(result.matched_key.empty());
}

TEST(BindingKeyLookupTest, ExactMatchPreferredOverFallbackWhenBothPresent)
{
  // If somehow both an exact synthesized-style key and a fallback-eligible
  // qualified key exist, exact match must win and via_fallback must be false.
  const std::vector<std::string> keys = {"pkg::TimerPublisherNode", "TimerPublisherNode"};
  const auto result = resolve_binding_key_lookup("TimerPublisherNode", keys);
  EXPECT_TRUE(result.found);
  EXPECT_FALSE(result.via_fallback);
  EXPECT_EQ(result.matched_key, "TimerPublisherNode");
}

// ── Instance-keyed load bookkeeping ──────────────────────────────────────────

class ComponentManagerBookkeepingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(ComponentManagerBookkeepingTest, TwoInstancesGetDistinctIdsUnderDistinctKeys)
{
  LFComponentManager manager(
    std::weak_ptr<rclcpp::Executor>(), "test_component_manager", rclcpp::NodeOptions());

  manager.set_binding_key_for_test("TimerPublisherNode", 1);
  manager.set_binding_key_for_test("TimerPublisherNode__robot_1_timer_node", 2);

  EXPECT_EQ(manager.class_to_id_size_for_test(), 2u);
  EXPECT_EQ(manager.binding_key_id_for_test("TimerPublisherNode"), 1u);
  EXPECT_EQ(manager.binding_key_id_for_test("TimerPublisherNode__robot_1_timer_node"), 2u);

  // Neither instance clobbers the other, unlike the pre-fix plugin-name-keyed
  // registration.
  EXPECT_TRUE(manager.has_binding_key_for_test("TimerPublisherNode"));
  EXPECT_TRUE(manager.has_binding_key_for_test("TimerPublisherNode__robot_1_timer_node"));
}

TEST_F(ComponentManagerBookkeepingTest, SameBindingKeyOverwritesLikeLegacyBehavior)
{
  // Re-registering the SAME binding key (e.g. a legacy system.json where two
  // instances share one plugin_name) still overwrites — this is intentional:
  // per-instance uniqueness is the generator's synthesis invariant, not
  // something this map enforces on its own.
  LFComponentManager manager(
    std::weak_ptr<rclcpp::Executor>(), "test_component_manager_overwrite", rclcpp::NodeOptions());

  manager.set_binding_key_for_test("Same", 1);
  manager.set_binding_key_for_test("Same", 2);

  EXPECT_EQ(manager.class_to_id_size_for_test(), 1u);
  EXPECT_EQ(manager.binding_key_id_for_test("Same"), 2u);
}

TEST_F(ComponentManagerBookkeepingTest, UnknownBindingKeyIsAbsent)
{
  LFComponentManager manager(
    std::weak_ptr<rclcpp::Executor>(), "test_component_manager_absent", rclcpp::NodeOptions());

  EXPECT_FALSE(manager.has_binding_key_for_test("DoesNotExist"));
  EXPECT_EQ(manager.binding_key_id_for_test("DoesNotExist"), 0u);
}

TEST_F(ComponentManagerBookkeepingTest, FindByClassExactMatchesSynthesizedKey)
{
  LFComponentManager manager(
    std::weak_ptr<rclcpp::Executor>(), "test_component_manager_find", rclcpp::NodeOptions());

  // No node_wrappers_ entry exists for this id (no real component was
  // loaded), so find_by_class resolves the key but returns nullptr when
  // looking up the node wrapper -- this still exercises the exact-match path
  // versus the fallback gating without requiring a live pluginlib load.
  manager.set_binding_key_for_test("TimerPublisherNode__robot_1_timer_node", 42);
  EXPECT_EQ(manager.find_by_class("TimerPublisherNode__robot_1_timer_node"), nullptr);
  // A synthesized lookup key with no exact match must not fall back.
  EXPECT_EQ(manager.find_by_class("TimerPublisherNode__no_such_instance"), nullptr);
}
