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

/// @file test_component_parsing.cpp
/// @brief Tests for parse_component_list: old-format (legacy dedup),
/// new-format (per-instance, no dedup), mixed-format (fail-closed), and
/// entry-scoped field extraction (long entries / fields beyond the old
/// 2000-char proximity window).

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "../src/runtime_internal.hpp"

using ros2_lf_binding::ComponentListParseResult;
using ros2_lf_binding::parse_component_list;

namespace
{

/// Write `content` to a unique temp file and return its path.
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

}  // namespace

class ComponentParsingTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    for (const auto & path : temp_files_) {
      std::remove(path.c_str());
    }
    temp_files_.clear();
  }

  std::string temp_json(const std::string & name, const std::string & content)
  {
    const std::string path = write_temp_file(name, content);
    temp_files_.push_back(path);
    return path;
  }

  std::vector<std::string> temp_files_;
};

// ── Absent file / no component_instances key ─────────────────────────────────

// A relative system.json read from the wrong cwd used to load 0
// components and leave every reaction unbound. It must fail and name the
// resolved path and the cwd.
TEST_F(ComponentParsingTest, AbsentFileFailsNamingResolvedPathAndCwd)
{
  const std::string rel = "lf-analysis/no_such_pkg/no_such_stem/system.json";
  const auto result = parse_component_list(rel);
  EXPECT_FALSE(result.ok) << "missing system.json was read as an empty component list";
  EXPECT_TRUE(result.instances.empty());
  const std::string cwd = std::filesystem::current_path().string();
  EXPECT_NE(result.error.find("system.json '" + rel + "' not found"), std::string::npos)
    << result.error;
  EXPECT_NE(result.error.find("resolved to '" + cwd + "/" + rel + "'"), std::string::npos)
    << result.error;
  EXPECT_NE(result.error.find("cwd '" + cwd + "'"), std::string::npos) << result.error;
}

TEST_F(ComponentParsingTest, MissingFileErrorIsEmptyForExistingOrUnsetPath)
{
  EXPECT_EQ(ros2_lf_binding::missing_file_error("exec_config.json", ""), "");
  const std::string path = temp_json("present.json", "{}");
  EXPECT_EQ(ros2_lf_binding::missing_file_error("exec_config.json", path), "");
  const std::string err =
    ros2_lf_binding::missing_file_error("exec_config.json", path + ".gone");
  EXPECT_NE(err.find("exec_config.json '" + path + ".gone' not found"), std::string::npos) << err;
}

TEST_F(ComponentParsingTest, NoComponentInstancesKeyReturnsEmptyOk)
{
  const std::string path = temp_json("no_key.json", R"json({"name": "sys", "nodes": {}})json");
  const auto result = parse_component_list(path);
  EXPECT_TRUE(result.ok);
  EXPECT_TRUE(result.instances.empty());
}

TEST_F(ComponentParsingTest, MalformedJsonFails)
{
  const std::string path = temp_json("malformed.json", R"json({"component_instances": [)json");
  const auto result = parse_component_list(path);
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

TEST_F(ComponentParsingTest, ComponentInstancesNotArrayFails)
{
  const std::string path =
    temp_json("not_array.json", R"json({"component_instances": "nope"})json");
  const auto result = parse_component_list(path);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("array"), std::string::npos);
}

TEST_F(ComponentParsingTest, EntryMissingPackageOrPluginClassFails)
{
  const std::string path =
    temp_json("missing_fields.json", R"json({"component_instances": [{"binding_key": "X"}]})json");
  const auto result = parse_component_list(path);
  EXPECT_FALSE(result.ok);
}

// ── Old format: no entry has binding_key -> legacy dedup on {package, class} ──

TEST_F(ComponentParsingTest, OldFormatDedupesOnPackageAndClass)
{
  const std::string path = temp_json(
    "old_format.json",
    R"json({"component_instances": [
      {"package": "basic_components", "plugin_class": "TimerPublisherNode"},
      {"package": "basic_components", "plugin_class": "TimerPublisherNode"},
      {"package": "basic_components", "plugin_class": "OtherNode"}
    ]})json");
  const auto result = parse_component_list(path);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.instances.size(), 2u);
  EXPECT_EQ(result.instances[0].plugin_class, "TimerPublisherNode");
  EXPECT_EQ(result.instances[0].binding_key, "TimerPublisherNode");
  EXPECT_EQ(result.instances[1].plugin_class, "OtherNode");
  EXPECT_EQ(result.instances[1].binding_key, "OtherNode");
}

TEST_F(ComponentParsingTest, OldFormatCarriesNodeNameAndNamespaceThrough)
{
  const std::string path = temp_json(
    "old_format_identity.json",
    R"json({"component_instances": [
      {"package": "pkg", "plugin_class": "Cls",
       "node_name": "my_node", "namespace": "/ns"}
    ]})json");
  const auto result = parse_component_list(path);
  ASSERT_TRUE(result.ok);
  ASSERT_EQ(result.instances.size(), 1u);
  EXPECT_EQ(result.instances[0].node_name, "my_node");
  EXPECT_EQ(result.instances[0].node_namespace, "/ns");
}

// Launch remaps reach the LoadNode request.
TEST_F(ComponentParsingTest, RemapRulesAreCarriedThrough)
{
  const std::string path = temp_json(
    "remap_rules.json",
    R"json({"component_instances": [
      {"package": "pkg", "plugin_class": "Cls", "binding_key": "Cls",
       "node_name": "n", "namespace": "", "remap_rules": ["data:=data_remapped"]},
      {"package": "pkg", "plugin_class": "Other", "binding_key": "Other",
       "node_name": "m", "namespace": ""}
    ]})json");
  const auto result = parse_component_list(path);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.instances.size(), 2u);
  EXPECT_EQ(result.instances[0].remap_rules, std::vector<std::string>{"data:=data_remapped"});
  EXPECT_TRUE(result.instances[1].remap_rules.empty());
}

// Launch parameters reach the component's node options.
TEST_F(ComponentParsingTest, ParamArgsAreCarriedThrough)
{
  const std::string path = temp_json(
    "param_args.json",
    R"json({"component_instances": [
      {"package": "pkg", "plugin_class": "Cls", "binding_key": "Cls",
       "node_name": "n", "namespace": "",
       "param_args": ["-p", "rate:=7", "--params-file", "/tmp/p.yaml"]},
      {"package": "pkg", "plugin_class": "Other", "binding_key": "Other",
       "node_name": "m", "namespace": ""}
    ]})json");
  const auto result = parse_component_list(path);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.instances.size(), 2u);
  EXPECT_EQ(
    result.instances[0].param_args,
    (std::vector<std::string>{"-p", "rate:=7", "--params-file", "/tmp/p.yaml"}));
  EXPECT_TRUE(result.instances[1].param_args.empty());
}

TEST_F(ComponentParsingTest, NonStringParamArgIsAnError)
{
  const std::string path = temp_json(
    "param_args_bad.json",
    R"json({"component_instances": [
      {"package": "pkg", "plugin_class": "Cls", "binding_key": "Cls",
       "param_args": [3]}
    ]})json");
  const auto result = parse_component_list(path);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("param_args"), std::string::npos) << result.error;
}

TEST_F(ComponentParsingTest, NonStringRemapRuleIsAnError)
{
  const std::string path = temp_json(
    "remap_rules_bad.json",
    R"json({"component_instances": [
      {"package": "pkg", "plugin_class": "Cls", "binding_key": "Cls",
       "remap_rules": [3]}
    ]})json");
  const auto result = parse_component_list(path);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("remap_rules"), std::string::npos) << result.error;
}

// ── New format: every entry has binding_key -> no dedup, one per instance ────

TEST_F(ComponentParsingTest, NewFormatLoadsEveryInstanceNoDedup)
{
  const std::string path = temp_json(
    "new_format.json",
    R"json({"component_instances": [
      {"package": "basic_components", "plugin_class": "TimerPublisherNode",
       "binding_key": "TimerPublisherNode", "node_name": "timer_node", "namespace": ""},
      {"package": "basic_components", "plugin_class": "TimerPublisherNode",
       "binding_key": "TimerPublisherNode__robot_1_timer_node",
       "node_name": "timer_node", "namespace": "/robot_1"}
    ]})json");
  const auto result = parse_component_list(path);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.instances.size(), 2u);
  EXPECT_EQ(result.instances[0].binding_key, "TimerPublisherNode");
  EXPECT_EQ(result.instances[0].node_namespace, "");
  EXPECT_EQ(result.instances[1].binding_key, "TimerPublisherNode__robot_1_timer_node");
  EXPECT_EQ(result.instances[1].node_namespace, "/robot_1");
  EXPECT_EQ(result.instances[1].plugin_class, "TimerPublisherNode");
}

TEST_F(ComponentParsingTest, NewFormatDuplicateBindingKeysAreNotDeduped)
{
  // Not this layer's job to assert uniqueness of binding_key (that is a
  // generator-side synthesis invariant); the parser must not silently drop
  // an entry just because it looks like a previous one.
  const std::string path = temp_json(
    "new_format_dup_key.json",
    R"json({"component_instances": [
      {"package": "pkg", "plugin_class": "Cls", "binding_key": "Cls__a",
       "node_name": "a", "namespace": ""},
      {"package": "pkg", "plugin_class": "Cls", "binding_key": "Cls__a",
       "node_name": "b", "namespace": ""}
    ]})json");
  const auto result = parse_component_list(path);
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.instances.size(), 2u);
}

// ── Mixed old/new -> loud fail-closed init failure ───────────────────────────

TEST_F(ComponentParsingTest, MixedFormatFailsClosed)
{
  const std::string path = temp_json(
    "mixed_format.json",
    R"json({"component_instances": [
      {"package": "pkg", "plugin_class": "A", "binding_key": "A"},
      {"package": "pkg", "plugin_class": "B"}
    ]})json");
  const auto result = parse_component_list(path);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("binding_key"), std::string::npos);
  // Fail-closed: no partial instance list leaks out on rejection.
  EXPECT_TRUE(result.instances.empty());
}

TEST_F(ComponentParsingTest, MixedFormatOrderIndependent)
{
  // Same assertion with the missing-key entry listed first, to make sure the
  // detection isn't accidentally order-sensitive.
  const std::string path = temp_json(
    "mixed_format_reordered.json",
    R"json({"component_instances": [
      {"package": "pkg", "plugin_class": "B"},
      {"package": "pkg", "plugin_class": "A", "binding_key": "A"}
    ]})json");
  const auto result = parse_component_list(path);
  EXPECT_FALSE(result.ok);
}

// ── Entry-scoped extraction: long entries / fields beyond old 2000-char window

TEST_F(ComponentParsingTest, LongEntryFieldsBeyond2000CharsAreStillFound)
{
  // The old hand-rolled scanner used a byte-offset proximity window (up to
  // 2000 chars) around each "class_name" occurrence and would silently
  // return "" for a field that scrolled out of range — which the new
  // mixed-schema rule would misread as "missing binding_key" and fail
  // closed spuriously.
  // Pad the entry with a long filler string field before binding_key
  // so any proximity-window-based approach would miss it; a real per-entry
  // JSON parse must not.
  const std::string filler(3000, 'x');
  const std::string path = temp_json(
    "long_entry.json",
    R"json({"component_instances": [
      {"package": "basic_components", "plugin_class": "TimerPublisherNode",
       "_filler": ")json"
    +
      filler +
      R"json(",
       "binding_key": "TimerPublisherNode__robot_1_timer_node",
       "node_name": "timer_node", "namespace": "/robot_1"}
    ]})json");
  const auto result = parse_component_list(path);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.instances.size(), 1u);
  EXPECT_EQ(result.instances[0].binding_key, "TimerPublisherNode__robot_1_timer_node");
  EXPECT_EQ(result.instances[0].node_name, "timer_node");
  EXPECT_EQ(result.instances[0].node_namespace, "/robot_1");
}

TEST_F(ComponentParsingTest, ManyLongEntriesEachResolveIndependently)
{
  // Five fields per entry, entries themselves padded and repeated many
  // times, so any global (non-entry-scoped) byte-offset window would have
  // long since drifted away from the fields it's supposed to match by the
  // time later entries are reached.
  std::string body = "{\"component_instances\": [";
  const std::string filler(2500, 'y');
  for (int i = 0; i < 5; ++i) {
    if (i > 0) {
      body += ",";
    }
    body += R"json({"package": "pkg", "plugin_class": "Cls)json" + std::to_string(i) +
      R"json(", "_filler": ")json" + filler + R"json(",
       "binding_key": "Cls)json" +
      std::to_string(i) + R"json(__inst)json" + std::to_string(i) +
      R"json(", "node_name": "n)json" + std::to_string(i) +
      R"json(", "namespace": "/ns)json" + std::to_string(i) + "\"}";
  }
  body += "]}";
  const std::string path = temp_json("many_long_entries.json", body);

  const auto result = parse_component_list(path);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.instances.size(), 5u);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(result.instances[i].plugin_class, "Cls" + std::to_string(i));
    EXPECT_EQ(
      result.instances[i].binding_key, "Cls" + std::to_string(i) + "__inst" + std::to_string(i));
    EXPECT_EQ(result.instances[i].node_name, "n" + std::to_string(i));
    EXPECT_EQ(result.instances[i].node_namespace, "/ns" + std::to_string(i));
  }
}
