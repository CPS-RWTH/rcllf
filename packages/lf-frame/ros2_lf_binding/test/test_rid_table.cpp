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

#include <gtest/gtest.h>

#include "../src/handle_table.hpp"
#include "../src/rid_table.hpp"
#include "ros2_lf_binding/rid_table_api.h"

// Stub for lfc-generated symbol required by ros2_lf_binding.so at link time.
// reactor-c headers lack extern "C" guards.
extern "C" {
#include "core/environment.h"
}
static environment_t g_stub_env;
extern "C" int _lf_get_environments(environment_t ** envs)
{
  *envs = &g_stub_env;
  return 1;
}

using ros2_lf_binding::g_handle_table;
using ros2_lf_binding::get_rid_entries;
using ros2_lf_binding::HandleKind;
using ros2_lf_binding::register_rid_entry;
using ros2_lf_binding::resize_handle_table;
using ros2_lf_binding::RidEntry;

class RidTableTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    // Reset globals between tests.
    g_handle_table.clear();
    // get_rid_entries returns a ref to a static; we can't clear it, but each
    // test registers its own entries and checks only those.
  }
};

TEST_F(RidTableTest, RegisterTimerEntry)
{
  RidEntry e;
  e.reaction_id = 0;
  e.node_class = "MyNode";
  e.callback_name = "timer_cb";
  e.kind = "timer";
  e.topic = "";
  e.period_ns = 100'000'000LL;  // 100 ms

  register_rid_entry(e);

  const auto & entries = get_rid_entries();
  bool found = false;
  for (const auto & entry : entries) {
    if (
      entry.reaction_id == 0 && entry.node_class == "MyNode" && entry.kind == "timer" &&
      entry.period_ns == 100'000'000LL)
    {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(RidTableTest, RegisterSubscriptionEntry)
{
  RidEntry e;
  e.reaction_id = 1;
  e.node_class = "ListenerNode";
  e.callback_name = "sub_cb";
  e.kind = "sub";
  e.topic = "/chatter";
  e.period_ns = 0;

  register_rid_entry(e);

  const auto & entries = get_rid_entries();
  bool found = false;
  for (const auto & entry : entries) {
    if (entry.reaction_id == 1 && entry.topic == "/chatter" && entry.kind == "sub") {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(RidTableTest, ResizeHandleTableGrowsOnly)
{
  resize_handle_table(5);
  EXPECT_EQ(g_handle_table.size(), 5u);

  // A smaller resize must not shrink the table.
  resize_handle_table(3);
  EXPECT_EQ(g_handle_table.size(), 5u);

  // A larger resize must grow.
  resize_handle_table(10);
  EXPECT_EQ(g_handle_table.size(), 10u);
}

TEST_F(RidTableTest, NewHandleEntriesAreUnbound)
{
  resize_handle_table(4);
  for (const auto & entry : g_handle_table) {
    EXPECT_EQ(entry.kind, HandleKind::UNBOUND);
    EXPECT_TRUE(entry.timer.expired());
    EXPECT_TRUE(entry.subscription.expired());
    EXPECT_TRUE(entry.topic.empty());
  }
}

TEST_F(RidTableTest, RegisterServiceServerEntry)
{
  RidEntry e;
  e.reaction_id = 10;
  e.node_class = "ServiceServerNode";
  e.callback_name = "handle_process_data";
  e.kind = "srv_server";
  e.topic = "/process_data";
  e.period_ns = 0;

  register_rid_entry(e);

  const auto & entries = get_rid_entries();
  bool found = false;
  for (const auto & entry : entries) {
    if (
      entry.reaction_id == 10 && entry.node_class == "ServiceServerNode" &&
      entry.kind == "srv_server" && entry.topic == "/process_data")
    {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST_F(RidTableTest, RegisterServiceClientEntry)
{
  RidEntry e;
  e.reaction_id = 11;
  e.node_class = "ServiceClientNode";
  e.callback_name = "response_callback";
  e.kind = "srv_client_cb";
  e.topic = "/process_data";
  e.period_ns = 0;

  register_rid_entry(e);

  const auto & entries = get_rid_entries();
  bool found = false;
  for (const auto & entry : entries) {
    if (
      entry.reaction_id == 11 && entry.node_class == "ServiceClientNode" &&
      entry.kind == "srv_client_cb" && entry.topic == "/process_data")
    {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}
