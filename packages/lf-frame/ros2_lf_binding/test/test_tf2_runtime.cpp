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


// Binding side of rcllf_tf2::AcquisitionLookup (S3): registration
// in the construction scope, several adapters per node, wake
// bookkeeping and the wake, ingest and close dispatch. Outside an LF program
// the current tag is {-1, 0}, so every delay below is the deadline plus one.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/tf2_runtime_api.h"
// Both declarations of every hook in one translation unit: a signature drift
// is a conflicting declaration and fails to compile.
#include "../../rcllf_tf2/include/rcllf_tf2/tf2_runtime_hooks.h"

namespace ros2_lf_binding
{
namespace tf2_rt
{
void reset_for_test();
}
}

namespace
{

std::vector<std::string> g_log;

void on_wake(void * ctx, int64_t time, uint32_t)
{
  g_log.push_back(std::string(static_cast<const char *>(ctx)) + " wake " + std::to_string(time));
}
void on_ingested(void * ctx, int64_t time, uint32_t)
{
  g_log.push_back(
    std::string(static_cast<const char *>(ctx)) + " ingested " + std::to_string(time));
}
void on_close(void * ctx, int64_t time, uint32_t)
{
  g_log.push_back(std::string(static_cast<const char *>(ctx)) + " close " + std::to_string(time));
}

class Tf2Runtime : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros2_lf_binding::tf2_rt::reset_for_test();
    g_log.clear();
  }
  void TearDown() override {ros2_lf_binding::tf2_rt::reset_for_test();}

  static int reg(const char * key, const char * name)
  {
    Ros2LfNodeConstructionScope scope(key);
    const rcllf_tf2_hooks h{const_cast<char *>(name), on_wake, on_ingested, on_close};
    return ros2_lf_tf2_register(&h);
  }
};

TEST_F(Tf2Runtime, RegistersOnlyInsideAConstructionScopeSeveralPerNode)
{
  const rcllf_tf2_hooks h{nullptr, on_wake, on_ingested, on_close};
  EXPECT_EQ(ros2_lf_tf2_register(&h), RCLLF_TF2_NOT_CONSTRUCTING);
  EXPECT_EQ(reg("cam", "a"), 0);
  EXPECT_EQ(reg("cam", "b"), 1);
  EXPECT_EQ(reg("lidar", "c"), 2);
  int64_t t = 0;
  EXPECT_EQ(ros2_lf_tf2_current(&t, nullptr), -1);  // no LF program
}

TEST_F(Tf2Runtime, WakesAreTakenOncePerDeadlineInOrderPerNode)
{
  const int a = reg("cam", "a");
  const int b = reg("cam", "b");
  ros2_lf_tf2_request_wake(a, 300);
  ros2_lf_tf2_request_wake(b, 100);
  ros2_lf_tf2_request_wake(b, 300);  // same deadline: one wake
  EXPECT_EQ(ros2_lf_tf2_take_wake("cam"), 101);
  EXPECT_EQ(ros2_lf_tf2_take_wake("cam"), 301);
  EXPECT_EQ(ros2_lf_tf2_take_wake("cam"), -1);
  ros2_lf_tf2_request_wake(a, 300);  // already scheduled: not again
  EXPECT_EQ(ros2_lf_tf2_take_wake("cam"), -1);
  EXPECT_EQ(ros2_lf_tf2_take_wake("nobody"), -1);
}

TEST_F(Tf2Runtime, WakeIngestAndCloseReachEveryAdapterOfTheNodeOnly)
{
  const int a = reg("cam", "a");
  reg("cam", "b");
  reg("lidar", "c");
  ros2_lf_tf2_wake("cam");
  ros2_lf_tf2_ingested("lidar");
  ros2_lf_tf2_unregister(a);
  ros2_lf_tf2_close("cam");
  EXPECT_EQ(
    g_log, (std::vector<std::string>{"a wake -1", "b wake -1", "c ingested -1", "b close -1"}));
}

}  // namespace
