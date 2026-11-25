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

// The binding reports the settings of the running program. This
// test executable stands in for an lfc program: it exports the environment,
// the worker count and the patch-marker bounds, as the generated one does.

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "../src/readback.hpp"
#include "ros2_lf_binding/rid_table_api.h"

extern "C" {
#include "core/environment.h"
}

static trigger_t g_timer_a;
static trigger_t g_timer_b;
static trigger_t * g_timers[] = {&g_timer_a, &g_timer_b};
static environment_t g_env;

extern "C" int _lf_get_environments(environment_t ** envs)
{
  g_env.timer_triggers = g_timers;
  g_env.timer_triggers_size = 2;
  g_env.worker_thread_count = 3;
  *envs = &g_env;
  return 1;
}

extern "C" {
__attribute__((visibility("default"))) unsigned int _lf_number_of_workers = 3;
}

static const char kMarkerA[] __attribute__((section("rcllf_reactor_c_patches"), used)) =
  "rcllf-reactor-c-patch:0002-test";
extern "C" __attribute__((weak)) const char __start_rcllf_reactor_c_patches[];
extern "C" __attribute__((weak)) const char __stop_rcllf_reactor_c_patches[];
extern "C" __attribute__((visibility("default"), used))
const char * const rcllf_reactor_c_patch_markers[2] = {
  __start_rcllf_reactor_c_patches,
  __stop_rcllf_reactor_c_patches,
};

TEST(Readback, SplitsMarkersAndSkipsPadding)
{
  const char buf[] = "a\0\0\0bc\0";
  auto v = ros2_lf_binding::split_patch_markers(buf, buf + sizeof(buf) - 1);
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], "a");
  EXPECT_EQ(v[1], "bc");
}

TEST(Readback, ParsesSchedFieldsOfAStatLine)
{
  // A comm with a space and ')' and SCHED_FIFO priority 42 (fields 40, 41).
  std::string line = "123 (lf w) 1) S";
  for (int field = 4; field <= 39; ++field) {
    line += " 0";
  }
  line += " 42 1 0 0";
  auto t = ros2_lf_binding::thread_sched_from_stat(line);
  ASSERT_FALSE(t.is_null());
  EXPECT_EQ(t["comm"], "lf w) 1");
  EXPECT_EQ(t["rt_priority"], 42);
  EXPECT_EQ(t["policy"], 1);
  EXPECT_TRUE(ros2_lf_binding::thread_sched_from_stat("1 (x) S 1 2").is_null());
}

TEST(Readback, WritesTheEffectiveSettingsOfTheProgram)
{
  g_timer_a.offset = 0;
  g_timer_a.period = 100000000;
  g_timer_b.offset = 5;
  g_timer_b.period = 33000000;
  ros2_lf_binding::RidEntry e{};
  e.reaction_id = 0;
  e.node_class = "Sensor";
  e.callback_name = "on_timer";
  e.kind = "timer";
  e.period_ns = 100000000;
  ros2_lf_binding::register_rid_entry(e);

  const auto dir = std::filesystem::temp_directory_path() /
    ("rcllf_readback_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(dir);
  const std::string path = ros2_lf_binding::write_readback(dir.string());
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(
    std::filesystem::path(path).filename().string(),
    "rcllf_readback." + std::to_string(getpid()) + ".json");
  std::ifstream f(path);
  const auto doc = nlohmann::json::parse(f);
  std::filesystem::remove_all(dir);

  EXPECT_EQ(doc["schema"], "rcllf.readback/1");
  EXPECT_EQ(doc["pid"], getpid());
  ASSERT_EQ(doc["rid_entries"].size(), 1u);
  EXPECT_EQ(doc["rid_entries"][0]["callback_name"], "on_timer");
  EXPECT_EQ(doc["rid_entries"][0]["period_ns"], 100000000);
  ASSERT_TRUE(doc["lf_timers"]["available"].get<bool>());
  ASSERT_EQ(doc["lf_timers"]["timers"].size(), 2u);
  EXPECT_EQ(doc["lf_timers"]["timers"][1]["period_ns"], 33000000);
  EXPECT_EQ(doc["lf_timers"]["timers"][1]["offset_ns"], 5);
  EXPECT_EQ(doc["lf_timers"]["worker_thread_count"], 3);
  EXPECT_EQ(doc["lf_workers"], 3);
  ASSERT_EQ(doc["reactor_c_patches"].size(), 1u) << doc["reactor_c_patches"].dump();
  EXPECT_EQ(doc["reactor_c_patches"][0], "rcllf-reactor-c-patch:0002-test");
  ASSERT_FALSE(doc["threads"].empty());
  int workers = 0;
  for (const auto & t : doc["threads"]) {
    EXPECT_EQ(t["policy"], 0) << t.dump();
    workers += t["lf_worker"].get<bool>() ? 1 : 0;
  }
  EXPECT_EQ(workers, 1);
}

TEST(Readback, WritesNothingWithoutTheDirectory)
{
  unsetenv("RCLLF_READBACK_DIR");
  ros2_lf_binding::write_readback_once();  // must not crash or write
  EXPECT_TRUE(ros2_lf_binding::write_readback("/nonexistent/dir").empty());
}
