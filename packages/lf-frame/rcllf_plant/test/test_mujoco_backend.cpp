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

// The MuJoCo backend behind the Transactor.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcllf_plant/hash.hpp"
#include "rcllf_plant/mujoco_backend.hpp"
#include "rcllf_plant/transactor.hpp"

namespace rcllf_plant
{
namespace
{
constexpr int64_t kStepNs = 1'000'000;

std::vector<CompletionRecord> run(uint64_t steps)
{
  Transactor t(std::make_unique<MujocoBackend>(8.0, kStepNs), std::chrono::milliseconds(1000));
  ResetRequest reset;
  reset.episode = 1;
  reset.seed = 0;
  reset.step_ns = kStepNs;
  reset.initial_state = {0.0, 15.0};
  std::vector<CompletionRecord> out{t.reset(reset)};
  for (uint64_t k = 1; k <= steps; ++k) {
    StepRequest r;
    r.episode = 1;
    r.step_id = k;
    r.logical_ns = static_cast<int64_t>(k) * kStepNs;
    r.step_ns = kStepNs;
    r.inputs = {{"brake", {k < 100 ? 0.0 : 0.85}}};
    out.push_back(t.step(r).done);
    if (out.back().status != Status::OK) {
      break;
    }
  }
  return out;
}

TEST(MujocoBackend, DescribesItself)
{
  MujocoBackend b(8.0, kStepNs);
  const Descriptor d = b.describe();
  EXPECT_EQ(d.backend, "mujoco");
  EXPECT_EQ(d.version, "3.14.0");
  EXPECT_EQ(d.step_ns, kStepNs);
  EXPECT_EQ(d.inputs, std::vector<std::string>{"brake"});
  EXPECT_EQ(d.state, (std::vector<std::string>{"position", "velocity"}));
}

TEST(MujocoBackend, RefusesAStepOtherThanTheModelTimestep)
{
  EXPECT_THROW(MujocoBackend(8.0, 2 * kStepNs), std::runtime_error);
}

TEST(MujocoBackend, ConstructedPausedAtZeroState)
{
  MujocoBackend b(8.0, kStepNs);
  const Probe p = b.probe();
  EXPECT_EQ(p.backend_steps, 0u);
  EXPECT_EQ(p.backend_clock_ns, 0);
  EXPECT_EQ(p.state_hash, canonical_hash({0.0, 0.0}));
}

// Two backends in one process give the same hashes at every step, and the
// vehicle stops (velocity locks to 0) under the brake.
TEST(MujocoBackend, SameInputsSameHashesAndAStop)
{
  const auto a = run(3000);
  const auto b = run(3000);
  ASSERT_EQ(a.size(), 3001u);
  ASSERT_EQ(b.size(), a.size());
  for (size_t k = 0; k < a.size(); ++k) {
    ASSERT_EQ(a[k].status, Status::OK) << k << ": " << a[k].detail;
    EXPECT_EQ(a[k].state_hash, b[k].state_hash) << "step " << k;
    EXPECT_EQ(a[k].output_hash, b[k].output_hash) << "step " << k;
  }
  // Stopped: the state no longer changes over the last steps.
  EXPECT_EQ(a[2999].state_hash, a[3000].state_hash);
}

// The plant is a forward-moving braking vehicle: a reset with a negative
// initial velocity is refused before it reaches the engine, and the Transactor
// stays usable. Without the check the lock (v < kLockSpeed -> 0) reports a stop
// after step 1 with brake 0 that never happened.
TEST(MujocoBackend, RefusesANegativeInitialVelocity)
{
  Transactor t(std::make_unique<MujocoBackend>(8.0, kStepNs), std::chrono::milliseconds(1000));
  ResetRequest reset;
  reset.episode = 1;
  reset.seed = 0;
  reset.step_ns = kStepNs;
  reset.initial_state = {0.0, -1.0};
  try {
    const CompletionRecord r0 = t.reset(reset);
    StepRequest r;
    r.episode = 1;
    r.step_id = 1;
    r.logical_ns = kStepNs;
    r.step_ns = kStepNs;
    r.inputs = {{"brake", {0.0}}};
    const StepResult s = t.step(r);
    ADD_FAILURE() << "reset with initial velocity -1 accepted (" << to_string(r0.status) <<
      "); after step 1 with brake 0 the plant reports velocity " << s.state.values.at(1) <<
      ", a stop that never happened";
    return;
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("velocity -1"), std::string::npos) << e.what();
  }
  reset.initial_state = {0.0, 15.0};
  EXPECT_EQ(t.reset(reset).status, Status::OK) << "a refused reset breaks the Transactor";
}

// The basis of the half-step clock tolerance: the float clock of 10^5 Euler
// steps of 1 ms stays far below it (10^6 steps drift by ~110 ns).
TEST(MujocoBackend, FloatClockDriftStaysFarBelowHalfAStep)
{
  const auto a = run(100000);
  ASSERT_EQ(a.size(), 100001u);
  int64_t worst = 0;
  for (const auto & r : a) {
    worst = std::max<int64_t>(worst, std::llabs(r.clock_drift_ns));
  }
  RecordProperty("worst_drift_ns", static_cast<int>(worst));
  EXPECT_LE(worst, 1000) << "worst drift " << worst << " ns";
}

}  // namespace
}  // namespace rcllf_plant
