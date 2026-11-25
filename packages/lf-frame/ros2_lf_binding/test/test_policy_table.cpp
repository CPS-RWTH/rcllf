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

/// @file test_policy_table.cpp
/// @brief Tests for exec_config.json loading and (node_class, topic) policy
///        resolution: lookup hit, defaults fallback, slash-tolerant topic
///        match, per-key env override, absent file, corrupt file.

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "../src/policy_table.hpp"
#include "../src/take_mode.hpp"

using ros2_lf_binding::kExecConfigSchemaVersion;
using ros2_lf_binding::kMaxMissExamples;
using ros2_lf_binding::policy_table_defaults;
using ros2_lf_binding::policy_table_load;
using ros2_lf_binding::policy_table_log_miss_summary;
using ros2_lf_binding::policy_table_miss_count;
using ros2_lf_binding::policy_table_miss_examples;
using ros2_lf_binding::policy_table_reset;
using ros2_lf_binding::policy_table_row_count;
using ros2_lf_binding::resolve_policy;
using ros2_lf_binding::take_mode_env;
using ros2_lf_binding::take_timeout_ms_env;
using ros2_lf_binding::TakeMode;

namespace
{

/// Write `content` to a unique temp file and return its path.
/// The fixture removes it in TearDown.
std::string write_temp_file(const std::string & name, const std::string & content)
{
  // pid in the name, so concurrent runs never share a file.
  const std::string path = std::string(::testing::TempDir()) + "/" +
    std::to_string(getpid()) + "_" + name;
  std::ofstream ofs(path, std::ios::trunc);
  ofs << content;
  ofs.close();
  return path;
}

constexpr const char * kGoodConfig =
  R"json({
  "schema_version": 1,
  "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
  "subscriptions": [
    {"node_class": "SubscriberNode", "topic": "sensor_data",
     "take_mode": "block", "take_timeout_ms": 250},
    {"node_class": "FeatureNode", "topic": "/feature_input",
     "take_mode": "skip", "take_timeout_ms": 10}
  ]
})json";

}  // namespace

class PolicyTableTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    policy_table_reset();
    // The dev container exports both take-mode vars (docker-compose.yml
    // x-common-env); clear them so env-override cases are explicit.
    unsetenv("ROS2_LF_TAKE_MODE");
    unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
    unsetenv("ROS2_LF_MAXWAIT_MS");
  }

  void TearDown() override
  {
    for (const auto & path : temp_files_) {
      std::remove(path.c_str());
    }
    temp_files_.clear();
    policy_table_reset();
    unsetenv("ROS2_LF_TAKE_MODE");
    unsetenv("ROS2_LF_TAKE_TIMEOUT_MS");
    unsetenv("ROS2_LF_MAXWAIT_MS");
  }

  std::string temp_config(const std::string & name, const std::string & content)
  {
    const std::string path = write_temp_file(name, content);
    temp_files_.push_back(path);
    return path;
  }

  std::vector<std::string> temp_files_;
};

// ── Built-in defaults before any load ────────────────────────────────────────

TEST_F(PolicyTableTest, BuiltInDefaultsAreSkip100)
{
  const auto defaults = policy_table_defaults();
  EXPECT_EQ(defaults.take_mode, TakeMode::Skip);
  EXPECT_EQ(defaults.take_timeout_ms, 100);
  EXPECT_EQ(policy_table_row_count(), 0u);
}

// ── Lookup hit ───────────────────────────────────────────────────────────────

TEST_F(PolicyTableTest, LookupHitReturnsRowValues)
{
  const std::string path = temp_config("exec_config_hit.json", kGoodConfig);
  const auto load = policy_table_load(path);
  ASSERT_TRUE(load.ok) << load.error;
  EXPECT_TRUE(load.file_present);
  EXPECT_EQ(policy_table_row_count(), 2u);

  const auto policy = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(policy.take_mode, TakeMode::Block);
  EXPECT_EQ(policy.take_timeout_ms, 250);
  EXPECT_EQ(policy_table_miss_count(), 0u);
}

TEST_F(PolicyTableTest, PerKeyMergeIsPreservedFromPython)
{
  // The design's §2.4 example row: take_mode from subscription scope,
  // take_timeout_ms from node scope.  Python does the merge; the C++ side must
  // carry both values through verbatim, not re-derive either from defaults.
  const std::string path = temp_config(
    "exec_config_merge.json",
    R"json({"schema_version": 1,
            "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
            "subscriptions": [
              {"node_class": "SubscriberNode", "topic": "sensor_data",
               "take_mode": "block", "take_timeout_ms": 250}]})json");
  ASSERT_TRUE(policy_table_load(path).ok);
  const auto policy = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(policy.take_mode, TakeMode::Block);
  EXPECT_EQ(policy.take_timeout_ms, 250);
}

TEST_F(PolicyTableTest, RowInheritsDefaultsForUnsetKeys)
{
  const std::string path = temp_config(
    "exec_config_partial.json",
    R"json({"schema_version": 1,
            "defaults": {"take_mode": "skip", "take_timeout_ms": 77},
            "subscriptions": [
              {"node_class": "SubscriberNode", "topic": "sensor_data",
               "take_mode": "block"}]})json");
  ASSERT_TRUE(policy_table_load(path).ok);
  const auto policy = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(policy.take_mode, TakeMode::Block);
  EXPECT_EQ(policy.take_timeout_ms, 77);
}

// ── Defaults fallback on miss ────────────────────────────────────────────────

TEST_F(PolicyTableTest, MissFallsBackToDefaultsAndCounts)
{
  const std::string path = temp_config(
    "exec_config_defaults.json",
    R"json({"schema_version": 1,
            "defaults": {"take_mode": "block", "take_timeout_ms": 42},
            "subscriptions": [
              {"node_class": "SubscriberNode", "topic": "sensor_data"}]})json");
  ASSERT_TRUE(policy_table_load(path).ok);

  // Unknown class.
  auto policy = resolve_policy("NoSuchNode", "sensor_data");
  EXPECT_EQ(policy.take_mode, TakeMode::Block);
  EXPECT_EQ(policy.take_timeout_ms, 42);
  EXPECT_EQ(policy_table_miss_count(), 1u);

  // Known class, unknown topic.
  policy = resolve_policy("SubscriberNode", "other_topic");
  EXPECT_EQ(policy.take_timeout_ms, 42);
  EXPECT_EQ(policy_table_miss_count(), 2u);
}

TEST_F(PolicyTableTest, NodeClassMatchIsExact)
{
  const std::string path = temp_config("exec_config_class.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  // A namespace-qualified spelling is a different key: RIDs carry the bare
  // class_name the generator emitted, so no fuzzy class matching is done.
  const auto policy = resolve_policy("pkg::SubscriberNode", "sensor_data");
  EXPECT_EQ(policy.take_mode, TakeMode::Skip);
  EXPECT_EQ(policy_table_miss_count(), 1u);
}

// ── Slash-tolerant topic match ───────────────────────────────────────────────

TEST_F(PolicyTableTest, SlashTolerantTopicMatchRelativeRow)
{
  const std::string path = temp_config(
    "exec_config_slash_rel.json",
    R"json({"schema_version": 1,
            "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
            "subscriptions": [
              {"node_class": "SubscriberNode", "topic": "sensor_data",
               "take_mode": "block", "take_timeout_ms": 250}]})json");
  ASSERT_TRUE(policy_table_load(path).ok);

  // Row key is relative; the declared topic may carry a leading slash.
  const auto with_slash = resolve_policy("SubscriberNode", "/sensor_data");
  EXPECT_EQ(with_slash.take_mode, TakeMode::Block);
  EXPECT_EQ(with_slash.take_timeout_ms, 250);
  EXPECT_EQ(policy_table_miss_count(), 0u);
}

TEST_F(PolicyTableTest, SlashTolerantTopicMatchAbsoluteRow)
{
  const std::string path = temp_config("exec_config_slash_abs.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  // Row key "/feature_input"; declared topic is the relative model form.
  const auto policy = resolve_policy("FeatureNode", "feature_input");
  EXPECT_EQ(policy.take_timeout_ms, 10);
  EXPECT_EQ(policy_table_miss_count(), 0u);
}

TEST_F(PolicyTableTest, MatchesAgainstRuntimeResolvedTopic)
{
  // Under a namespace the declared topic ("sensor_data") and the runtime
  // resolved name ("/robot_1/sensor_data") differ; either may carry the key.
  const std::string path = temp_config(
    "exec_config_resolved.json",
    R"json({"schema_version": 1,
            "defaults": {"take_mode": "skip", "take_timeout_ms": 100},
            "subscriptions": [
              {"node_class": "SubscriberNode", "topic": "/robot_1/sensor_data",
               "take_mode": "block", "take_timeout_ms": 250}]})json");
  ASSERT_TRUE(policy_table_load(path).ok);

  const auto policy = resolve_policy(
    "SubscriberNode", "sensor_data", "/robot_1/sensor_data");
  EXPECT_EQ(policy.take_mode, TakeMode::Block);
  EXPECT_EQ(policy.take_timeout_ms, 250);
  EXPECT_EQ(policy_table_miss_count(), 0u);

  // Without the resolved topic the same lookup legitimately misses.
  const auto missed = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(missed.take_mode, TakeMode::Skip);
  EXPECT_EQ(policy_table_miss_count(), 1u);
}

// ── Per-key env override ─────────────────────────────────────────────────────
//
// The override lives in the dispatch path (reaction_dispatch.cpp uses
// take_mode_env().value_or(entry.take_mode)); these cases assert the same
// composition against resolved rows: each key is overridden independently and
// an unset var leaves the resolved per-subscription value intact.

TEST_F(PolicyTableTest, EnvOverridesOnlyItsOwnKey)
{
  const std::string path = temp_config("exec_config_env.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  const auto resolved = resolve_policy("SubscriberNode", "sensor_data");
  ASSERT_EQ(resolved.take_mode, TakeMode::Block);
  ASSERT_EQ(resolved.take_timeout_ms, 250);

  setenv("ROS2_LF_TAKE_MODE", "skip", 1);
  EXPECT_EQ(take_mode_env().value_or(resolved.take_mode), TakeMode::Skip);
  // The timeout var is unset, so the resolved 250 survives.
  EXPECT_EQ(take_timeout_ms_env().value_or(resolved.take_timeout_ms), 250);

  unsetenv("ROS2_LF_TAKE_MODE");
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "5", 1);
  EXPECT_EQ(take_mode_env().value_or(resolved.take_mode), TakeMode::Block);
  EXPECT_EQ(take_timeout_ms_env().value_or(resolved.take_timeout_ms), 5);
}

TEST_F(PolicyTableTest, NoEnvLeavesResolvedValuesIntact)
{
  const std::string path = temp_config("exec_config_noenv.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  const auto resolved = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(take_mode_env().value_or(resolved.take_mode), TakeMode::Block);
  EXPECT_EQ(take_timeout_ms_env().value_or(resolved.take_timeout_ms), 250);
}

TEST_F(PolicyTableTest, EnvOverrideLogDoesNotThrowWithOrWithoutVars)
{
  const std::string path = temp_config("exec_config_warn.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  EXPECT_NO_THROW(ros2_lf_binding::policy_table_log_env_overrides());
  setenv("ROS2_LF_TAKE_MODE", "block", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "250", 1);
  EXPECT_NO_THROW(ros2_lf_binding::policy_table_log_env_overrides());
}

// ── ROS2_LF_MAXWAIT_MS, ROS2_LF_TAKE_TIMEOUT_MS its alias ─────────
// kGoodConfig: defaults 100 ms, rows SubscriberNode/sensor_data 250 ms and
// FeatureNode/feature_input 10 ms. The counts below follow from those values.

TEST_F(PolicyTableTest, MaxwaitVarOverridesAndNamesItself)
{
  const std::string path = temp_config("exec_config_mw.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  setenv("ROS2_LF_MAXWAIT_MS", "5", 1);
  const auto resolved = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(take_timeout_ms_env().value_or(resolved.take_timeout_ms), 5);
  EXPECT_EQ(
    ros2_lf_binding::policy_table_take_timeout_override_message(),
    "ROS2_LF_MAXWAIT_MS=5 overrides maxwait_ms for 2 of 2 configured subscription(s) and "
    "for the defaults, which every subscription without its own entry uses. An admitted "
    "correspondence edge keeps its generated timeout, which this variable does not change");
}

TEST_F(PolicyTableTest, TakeTimeoutVarIsAnAliasOfMaxwait)
{
  const std::string path = temp_config("exec_config_alias.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "250", 1);
  const auto resolved = resolve_policy("FeatureNode", "/feature_input");
  EXPECT_EQ(take_timeout_ms_env().value_or(resolved.take_timeout_ms), 250);
  EXPECT_EQ(
    ros2_lf_binding::policy_table_take_timeout_override_message(),
    "ROS2_LF_TAKE_TIMEOUT_MS=250 (alias of ROS2_LF_MAXWAIT_MS) overrides maxwait_ms for 1 "
    "of 2 configured subscription(s) and for the defaults, which every subscription without "
    "its own entry uses. An admitted correspondence edge keeps its generated timeout, which "
    "this variable does not change");
}

TEST_F(PolicyTableTest, MaxwaitAndAliasWithTheSameValueOverride)
{
  const std::string path = temp_config("exec_config_same.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  setenv("ROS2_LF_MAXWAIT_MS", "100", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "100", 1);
  const auto resolved = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(take_timeout_ms_env().value_or(resolved.take_timeout_ms), 100);
  EXPECT_EQ(
    ros2_lf_binding::policy_table_take_timeout_override_message(),
    "ROS2_LF_MAXWAIT_MS=100 (and its alias ROS2_LF_TAKE_TIMEOUT_MS=100) overrides maxwait_ms "
    "for 2 of 2 configured subscription(s), not for the defaults, which already match. An "
    "admitted correspondence edge keeps its generated timeout, which this variable does not "
    "change");
}

TEST_F(PolicyTableTest, MaxwaitAndAliasThatDisagreeAreRefused)
{
  const std::string path = temp_config("exec_config_conflict.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  setenv("ROS2_LF_MAXWAIT_MS", "200", 1);
  setenv("ROS2_LF_TAKE_TIMEOUT_MS", "100", 1);
  // Refused: neither value applies, the configured 250 ms stands.
  const auto resolved = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(take_timeout_ms_env().value_or(resolved.take_timeout_ms), 250);
  EXPECT_EQ(
    ros2_lf_binding::policy_table_take_timeout_override_message(),
    "ROS2_LF_MAXWAIT_MS='200' and its alias ROS2_LF_TAKE_TIMEOUT_MS='100' disagree: the "
    "maxwait override is refused, so every subscription keeps its configured maxwait_ms; set "
    "only ROS2_LF_MAXWAIT_MS");
}

// ── Absent file -> silent defaults ───────────────────────────────────────────

TEST_F(PolicyTableTest, AbsentFileIsSilentDefaults)
{
  const auto load = policy_table_load(
    std::string(::testing::TempDir()) + "/definitely_not_here_exec_config.json");
  EXPECT_TRUE(load.ok);
  EXPECT_FALSE(load.file_present);
  EXPECT_TRUE(load.error.empty());
  EXPECT_EQ(policy_table_row_count(), 0u);
  const auto policy = resolve_policy("SubscriberNode", "sensor_data");
  EXPECT_EQ(policy.take_mode, TakeMode::Skip);
  EXPECT_EQ(policy.take_timeout_ms, 100);
}

TEST_F(PolicyTableTest, EmptyPathIsSilentDefaults)
{
  const auto load = policy_table_load("");
  EXPECT_TRUE(load.ok);
  EXPECT_FALSE(load.file_present);
  EXPECT_EQ(policy_table_row_count(), 0u);
}

// ── Corrupt / wrong-version file -> init failure (fail-closed) ───────────────

TEST_F(PolicyTableTest, TruncatedJsonFails)
{
  const std::string path = temp_config(
    "exec_config_truncated.json", R"json({"schema_version": 1, "subscriptions": [)json");
  const auto load = policy_table_load(path);
  EXPECT_FALSE(load.ok);
  EXPECT_TRUE(load.file_present);
  EXPECT_FALSE(load.error.empty());
}

TEST_F(PolicyTableTest, NonObjectTopLevelFails)
{
  const std::string path = temp_config("exec_config_array.json", "[1, 2, 3]");
  EXPECT_FALSE(policy_table_load(path).ok);
}

TEST_F(PolicyTableTest, MissingSchemaVersionFails)
{
  const std::string path = temp_config(
    "exec_config_noversion.json", R"json({"defaults": {"take_mode": "skip"}})json");
  const auto load = policy_table_load(path);
  EXPECT_FALSE(load.ok);
  EXPECT_NE(load.error.find("schema_version"), std::string::npos);
}

TEST_F(PolicyTableTest, WrongSchemaVersionFails)
{
  const std::string path = temp_config(
    "exec_config_v2.json",
    R"json({"schema_version": 2, "defaults": {"take_mode": "skip"}})json");
  const auto load = policy_table_load(path);
  EXPECT_FALSE(load.ok);
  EXPECT_NE(load.error.find("schema_version"), std::string::npos);
  EXPECT_EQ(kExecConfigSchemaVersion, 1);
}

TEST_F(PolicyTableTest, UnknownTakeModeValueFails)
{
  const std::string path = temp_config(
    "exec_config_badmode.json",
    R"json({"schema_version": 1,
            "subscriptions": [
              {"node_class": "A", "topic": "t", "take_mode": "BLOCK"}]})json");
  const auto load = policy_table_load(path);
  EXPECT_FALSE(load.ok);
  EXPECT_NE(load.error.find("take_mode"), std::string::npos);
}

TEST_F(PolicyTableTest, NegativeTimeoutFails)
{
  const std::string path = temp_config(
    "exec_config_negtimeout.json",
    R"json({"schema_version": 1,
            "subscriptions": [
              {"node_class": "A", "topic": "t", "take_timeout_ms": -1}]})json");
  EXPECT_FALSE(policy_table_load(path).ok);
}

TEST_F(PolicyTableTest, RowMissingKeysFails)
{
  const std::string no_class = temp_config(
    "exec_config_noclass.json",
    R"json({"schema_version": 1, "subscriptions": [{"topic": "t"}]})json");
  EXPECT_FALSE(policy_table_load(no_class).ok);

  const std::string no_topic = temp_config(
    "exec_config_notopic.json",
    R"json({"schema_version": 1, "subscriptions": [{"node_class": "A"}]})json");
  EXPECT_FALSE(policy_table_load(no_topic).ok);
}

TEST_F(PolicyTableTest, RejectedFileLeavesPreviousTableIntact)
{
  const std::string good = temp_config("exec_config_good.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(good).ok);
  ASSERT_EQ(policy_table_row_count(), 2u);

  const std::string bad = temp_config(
    "exec_config_bad.json",
    R"json({"schema_version": 1,
            "subscriptions": [
              {"node_class": "A", "topic": "t", "take_mode": "nope"}]})json");
  EXPECT_FALSE(policy_table_load(bad).ok);
  EXPECT_EQ(policy_table_row_count(), 2u);
  EXPECT_EQ(resolve_policy("SubscriberNode", "sensor_data").take_mode, TakeMode::Block);
}

// ── One-shot post-bind miss summary ──────────────────────────────────────────
//
// policy_table_log_miss_summary() is called once from
// ros2_lf_runtime_wait_ready() after all handles are bound (runtime.cpp); it
// is a thin wrapper (WARN + examples) whose only observable, testable
// behavior from here is that it does not throw whether or not there were
// misses, and that the underlying example collection is correct and bounded.

TEST_F(PolicyTableTest, MissSummaryIsSilentWhenNoMisses)
{
  const std::string path = temp_config("exec_config_nomiss.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  resolve_policy("SubscriberNode", "sensor_data");
  ASSERT_EQ(policy_table_miss_count(), 0u);
  EXPECT_TRUE(policy_table_miss_examples().empty());
  // Nothing to summarize; must not throw and must remain a no-op.
  EXPECT_NO_THROW(policy_table_log_miss_summary());
}

TEST_F(PolicyTableTest, MissSummaryCollectsExamplesAndDoesNotThrow)
{
  const std::string path = temp_config("exec_config_summary.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);

  resolve_policy("UnknownNode", "sensor_data");
  resolve_policy("SubscriberNode", "unknown_topic");
  ASSERT_EQ(policy_table_miss_count(), 2u);

  const auto & examples = policy_table_miss_examples();
  ASSERT_EQ(examples.size(), 2u);
  EXPECT_EQ(examples[0].first, "UnknownNode");
  EXPECT_EQ(examples[0].second, "sensor_data");
  EXPECT_EQ(examples[1].first, "SubscriberNode");
  EXPECT_EQ(examples[1].second, "unknown_topic");

  EXPECT_NO_THROW(policy_table_log_miss_summary());
}

TEST_F(PolicyTableTest, MissExamplesAreBounded)
{
  const std::string path = temp_config("exec_config_bounded.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);

  const size_t attempts = kMaxMissExamples + 3;
  for (size_t i = 0; i < attempts; ++i) {
    resolve_policy("UnknownNode" + std::to_string(i), "topic" + std::to_string(i));
  }

  EXPECT_EQ(policy_table_miss_count(), attempts);
  EXPECT_EQ(policy_table_miss_examples().size(), kMaxMissExamples);
  EXPECT_NO_THROW(policy_table_log_miss_summary());
}

TEST_F(PolicyTableTest, PolicyTableResetClearsMissExamples)
{
  const std::string path = temp_config("exec_config_resetmiss.json", kGoodConfig);
  ASSERT_TRUE(policy_table_load(path).ok);
  resolve_policy("UnknownNode", "sensor_data");
  ASSERT_FALSE(policy_table_miss_examples().empty());

  policy_table_reset();
  EXPECT_EQ(policy_table_miss_count(), 0u);
  EXPECT_TRUE(policy_table_miss_examples().empty());
}
