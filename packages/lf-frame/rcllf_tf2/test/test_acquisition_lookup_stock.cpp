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

// Stock mode (no binding linked: the weak ros2_lf_tf2_* symbols are null).
// A MultiThreadedExecutor may run two callbacks of a node at once, so two
// requests can run concurrently; each must still get its own id
// (review finding F91-4: the id was incremented outside the mutex).

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rcllf_tf2/acquisition_lookup.hpp"

namespace
{

TEST(AcquisitionLookupStock, ConcurrentRequestsGetDistinctIds)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("acq_stock_test");
    tf2_ros::Buffer buffer(node->get_clock());
    rcllf_tf2::AcquisitionLookup acq(
      buffer, node.get(), std::chrono::milliseconds(55), rcllf_tf2::OnTimeout::reject());

    constexpr int kThreads = 8;
    constexpr int kPerThread = 20000;
    std::vector<std::vector<uint64_t>> ids(kThreads);
    // Stock mode traces every request to stderr; keep the log short.
    const int saved = dup(STDERR_FILENO);
    const int devnull = open("/dev/null", O_WRONLY);
    dup2(devnull, STDERR_FILENO);
    {
      std::vector<std::thread> threads;
      for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back(
          [&acq, &ids, t] {
            ids[t].reserve(kPerThread);
            for (int i = 0; i < kPerThread; ++i) {
              // Stamp 0 decides at the request (failure), so nothing stays pending.
              ids[t].push_back(acq.request("world", "sensor_link", rclcpp::Time(0), nullptr));
            }
          });
      }
      for (auto & th : threads) {
        th.join();
      }
    }
    dup2(saved, STDERR_FILENO);
    close(saved);
    close(devnull);

    std::set<uint64_t> unique;
    for (const auto & v : ids) {
      unique.insert(v.begin(), v.end());
    }
    const auto total = static_cast<std::size_t>(kThreads) * kPerThread;
    EXPECT_EQ(unique.size(), total)
      << (total - unique.size()) << " request id(s) were handed out twice by concurrent requests";
    EXPECT_EQ(*unique.rbegin(), total);
  }
  rclcpp::shutdown();
}

}  // namespace
