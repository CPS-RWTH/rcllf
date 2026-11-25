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

// A4 at the dispatch hook of the fault library: RCLLF_PLANT_FAULT
// blocks or delays one step at the plant thread's dispatch, the one place every
// backend's step passes through. The process-level A4 (exit code 3, nothing
// published) is brake_fusion's test_plant_timeout.py.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>
#include <utility>

#include "rcllf_plant/mujoco_backend.hpp"
#include "rcllf_plant/transactor.hpp"

namespace rcllf_plant
{
namespace
{
constexpr int64_t kStepNs = 1'000'000;
using std::chrono::milliseconds;

StepRequest request(uint64_t step_id)
{
  StepRequest r;
  r.episode = 1;
  r.step_id = step_id;
  r.logical_ns = static_cast<int64_t>(step_id) * kStepNs;
  r.step_ns = kStepNs;
  r.inputs = {{"brake", {0.0}}};
  return r;
}

// MuJoCo, plus a flag set when the backend is destroyed. The plant thread
// holds the last reference to the backend after a timed-out Transactor is
// destroyed, so the flag tells that the thread has exited.
std::atomic<bool> g_backend_destroyed{false};

struct Watched : PlantAdapter
{
  MujocoBackend inner{8.0, kStepNs};
  ~Watched() override {g_backend_destroyed = true;}
  Descriptor describe() const override {return inner.describe();}
  void reset(const ResetRequest & r) override {inner.reset(r);}
  StepResult step(const StepRequest & r) override {return inner.step(r);}
  Probe probe() const override {return inner.probe();}
};

void run_until_timeout(const char * fault)
{
  ASSERT_EQ(setenv("RCLLF_PLANT_FAULT", fault, 1), 0);
  g_backend_destroyed = false;
  auto t = std::make_unique<Transactor>(std::make_unique<Watched>(), milliseconds(100));
  unsetenv("RCLLF_PLANT_FAULT");
  EXPECT_STREQ(t->fault_spec(), fault);
  ResetRequest reset;
  reset.episode = 1;
  reset.seed = 0;
  reset.step_ns = kStepNs;
  reset.initial_state = {0.0, 15.0};
  ASSERT_EQ(t->reset(reset).status, Status::OK);
  ASSERT_EQ(t->step(request(1)).done.status, Status::OK);
  ASSERT_EQ(t->step(request(2)).done.status, Status::OK);
  const auto t0 = std::chrono::steady_clock::now();
  const StepResult r = t->step(request(3));
  const auto waited = std::chrono::steady_clock::now() - t0;
  EXPECT_EQ(r.done.status, Status::TIMEOUT) << fault << ": " << r.done.detail;
  EXPECT_LT(waited, milliseconds(1100)) << fault << ": the timeout is not supervised";
  EXPECT_TRUE(r.state.values.empty());
  t.reset();  // detaches the plant thread
}

TEST(TransactorFault, HangAtDispatchTimesOut) {
                                               run_until_timeout("hang:3");
}

// The step returns, but only after the timeout: the same outcome.
TEST(TransactorFault, DelayPastTheTimeoutTimesOut) {
                                                    run_until_timeout("delay:3:400");
}

// A step that completes late, after its Transactor was destroyed: the plant
// thread exits then instead of waiting for a job forever.
TEST(TransactorFault, ALateStepEndsThePlantThread) {
  run_until_timeout("delay:3:400");
  EXPECT_FALSE(g_backend_destroyed) << "the delayed step cannot have finished yet";
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!g_backend_destroyed && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(milliseconds(10));
  }
  EXPECT_TRUE(g_backend_destroyed) <<
    "the plant thread outlived its Transactor 3 s after the delayed step finished";
}

TEST(TransactorFault, MalformedSpecIsRefused)
{
  ASSERT_EQ(setenv("RCLLF_PLANT_FAULT", "hang:soon", 1), 0);
  EXPECT_THROW(
    Transactor(std::make_unique<MujocoBackend>(8.0, kStepNs), milliseconds(100)),
    std::invalid_argument);
  unsetenv("RCLLF_PLANT_FAULT");
}

}  // namespace
}  // namespace rcllf_plant
