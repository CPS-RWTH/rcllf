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

// Transactor rules: A2 no advancement without
// a request, A3 stale and duplicate requests, MISMATCH by counter and by
// clock, BACKEND_FAULT, and the supervised timeout in process.

#include <gtest/gtest.h>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <condition_variable>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rcllf_plant/mujoco_backend.hpp"
#include "rcllf_plant/transactor.hpp"

namespace rcllf_plant
{
namespace
{
constexpr int64_t kStepNs = 1'000'000;
using std::chrono::milliseconds;

StepRequest request(uint64_t step_id, double brake = 0.0, uint64_t episode = 1)
{
  StepRequest r;
  r.episode = episode;
  r.step_id = step_id;
  r.logical_ns = static_cast<int64_t>(step_id) * kStepNs;
  r.step_ns = kStepNs;
  r.inputs = {{"brake", {brake}}};
  return r;
}

ResetRequest reset_request(uint64_t episode = 1)
{
  ResetRequest r;
  r.episode = episode;
  r.seed = 0;
  r.step_ns = kStepNs;
  r.initial_state = {0.0, 15.0};
  return r;
}

std::unique_ptr<Transactor> mujoco(milliseconds timeout = milliseconds(1000))
{
  return std::make_unique<Transactor>(std::make_unique<MujocoBackend>(8.0, kStepNs), timeout);
}

// A backend whose misbehaviour is chosen per test.
struct Fake : PlantAdapter
{
  int steps_per_call{1};
  int64_t clock_skew_ns{0};
  bool throw_on_step{false};
  std::vector<double> state_min;
  bool block{false};
  std::mutex * mutex{nullptr};
  std::condition_variable * cv{nullptr};
  bool * released{nullptr};
  // Published (release) before a blocked step waits: the request it was given.
  std::atomic<const StepRequest *> * seen{nullptr};
  uint64_t steps{0};

  Descriptor describe() const override
  {
    Descriptor d;
    d.backend = "fake";
    d.version = "0";
    d.step_ns = kStepNs;
    d.inputs = {"brake"};
    d.state = {"position", "velocity"};
    d.state_min = state_min;
    return d;
  }
  void reset(const ResetRequest &) override {steps = 0;}
  StepResult step(const StepRequest & request) override
  {
    if (throw_on_step) {
      throw std::runtime_error("engine exploded");
    }
    if (block) {
      if (seen != nullptr) {
        seen->store(&request, std::memory_order_release);
      }
      std::unique_lock<std::mutex> lock(*mutex);
      cv->wait(lock, [this] {return *released;});
    }
    steps += steps_per_call;
    StepResult r;
    r.state.values = {static_cast<double>(steps), 0.0};
    return r;
  }
  Probe probe() const override
  {
    Probe p;
    p.backend_steps = steps;
    p.backend_clock_ns = static_cast<int64_t>(steps) * kStepNs + (steps ? clock_skew_ns : 0);
    p.state_hash = steps;
    return p;
  }
};

TEST(Transactor, StepsAndReportsCounterClockAndHashes)
{
  auto t = mujoco();
  const CompletionRecord r0 = t->reset(reset_request());
  ASSERT_EQ(r0.status, Status::OK) << r0.detail;
  for (uint64_t k = 1; k <= 5; ++k) {
    const StepResult r = t->step(request(k, 0.5));
    ASSERT_EQ(r.done.status, Status::OK) << r.done.detail;
    EXPECT_EQ(r.done.backend_steps, k);
    EXPECT_LE(std::llabs(r.done.clock_drift_ns), kStepNs / 2);
    EXPECT_EQ(r.state.values.size(), 2u);
    EXPECT_NE(r.done.state_hash, r0.state_hash);
  }
}

// A2: construct, reset, wait 2 s of wall time (longer than 1000 steps of the
// 1 ms plant would take), then the counter and the state hash are unchanged.
TEST(Transactor, A2NoAdvancementWithoutARequest)
{
  auto t = mujoco();
  const CompletionRecord r0 = t->reset(reset_request());
  ASSERT_EQ(r0.status, Status::OK);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  const CompletionRecord p = t->probe();
  ASSERT_EQ(p.status, Status::OK) << p.detail;
  EXPECT_EQ(p.backend_steps, 0u) << "the plant advanced without a request";
  EXPECT_EQ(p.backend_clock_ns, 0) << "the plant clock moved without a request";
  EXPECT_EQ(p.state_hash, r0.state_hash) << "the plant state moved without a request";
}

// A3: every stale request is refused before the backend is touched.
TEST(Transactor, A3StaleRequestsAreRefusedAndTheStateDoesNotMove)
{
  auto t = mujoco();
  ASSERT_EQ(t->reset(reset_request()).status, Status::OK);
  ASSERT_EQ(t->step(request(1, 0.3)).done.status, Status::OK);
  const CompletionRecord before = t->probe();

  StepRequest wrong_logical = request(2, 0.3);
  wrong_logical.logical_ns += 1;
  StepRequest wrong_step_ns = request(2, 0.3);
  wrong_step_ns.step_ns = 2 * kStepNs;
  wrong_step_ns.logical_ns = 2 * wrong_step_ns.step_ns;
  StepRequest no_inputs = request(2, 0.3);
  no_inputs.inputs.clear();
  StepRequest wrong_channel = request(2, 0.3);
  wrong_channel.inputs[0].name = "throttle";
  const std::pair<const char *, StepRequest> cases[] = {
    {"duplicate step_id", request(1, 0.3)},
    {"skipped step_id", request(3, 0.3)},
    {"wrong logical_ns", wrong_logical},
    {"foreign step_ns", wrong_step_ns},
    {"old episode", request(2, 0.3, 0)},
    {"incomplete inputs", no_inputs},
    {"foreign input channel", wrong_channel},
  };
  for (const auto & c : cases) {
    const StepResult r = t->step(c.second);
    EXPECT_EQ(r.done.status, Status::STALE) << c.first << " was not refused";
    const CompletionRecord after = t->probe();
    EXPECT_EQ(after.backend_steps, before.backend_steps) << c.first << " advanced the plant";
    EXPECT_EQ(after.state_hash, before.state_hash) << c.first << " moved the plant state";
  }
  // A retry of the refused step still goes through as step 2.
  EXPECT_EQ(t->step(request(2, 0.3)).done.status, Status::OK);
}

TEST(Transactor, StepBeforeResetIsStale)
{
  auto t = mujoco();
  EXPECT_EQ(t->step(request(1)).done.status, Status::STALE);
  EXPECT_EQ(t->probe().backend_steps, 0u);
}

TEST(Transactor, ARequestFromBeforeAResetIsStale)
{
  auto t = mujoco();
  ASSERT_EQ(t->reset(reset_request(1)).status, Status::OK);
  ASSERT_EQ(t->step(request(1)).done.status, Status::OK);
  ASSERT_EQ(t->reset(reset_request(2)).status, Status::OK);
  EXPECT_EQ(t->step(request(1, 0.0, 1)).done.status, Status::STALE);
  EXPECT_EQ(t->step(request(1, 0.0, 2)).done.status, Status::OK);
}

TEST(Transactor, ResetRefusesUnseededForeignStepAndWrongEpisode)
{
  auto t = mujoco();
  ResetRequest unseeded = reset_request();
  unseeded.seed.reset();
  EXPECT_THROW(t->reset(unseeded), std::invalid_argument);
  ResetRequest foreign = reset_request();
  foreign.step_ns = 2 * kStepNs;
  EXPECT_THROW(t->reset(foreign), std::invalid_argument);
  EXPECT_THROW(t->reset(reset_request(2)), std::invalid_argument);
  EXPECT_EQ(t->reset(reset_request(1)).status, Status::OK);
}

// The descriptor's state_min and finiteness are checked before the backend
// sees the reset; a refused reset leaves the Transactor usable.
TEST(Transactor, ResetRefusesANonFiniteOrBelowMinimumInitialState)
{
  auto t = mujoco();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<std::vector<double>, std::string>> cases = {
    {{0.0, -1.0}, "plant reset initial velocity -1 is below the mujoco backend's minimum 0"},
    {{0.0, -1e-300}, "plant reset initial velocity -1e-300 is below"},
    {{0.0, nan}, "plant reset initial velocity nan is not finite"},
    {{inf, 15.0}, "plant reset initial position inf is not finite"},
    {{-inf, 15.0}, "plant reset initial position -inf is not finite"},
  };
  for (const auto & c : cases) {
    ResetRequest r = reset_request();
    r.initial_state = c.first;
    try {
      t->reset(r);
      ADD_FAILURE() << "accepted: " << c.second;
    } catch (const std::invalid_argument & e) {
      EXPECT_NE(std::string(e.what()).find(c.second), std::string::npos) << e.what();
    }
  }
  ResetRequest zero = reset_request();
  zero.initial_state = {-3.0, 0.0};
  EXPECT_EQ(t->reset(zero).status, Status::OK) << "v0 = 0 and a negative position are allowed";
}

TEST(Transactor, RefusesABackendWhoseStateMinDoesNotMatchItsState)
{
  auto fake = std::make_unique<Fake>();
  fake->state_min = {0.0};
  EXPECT_THROW(Transactor(std::move(fake), milliseconds(1000)), std::invalid_argument);
}

TEST(Transactor, TwoEngineStepsForOneRequestIsMismatch)
{
  auto fake = std::make_unique<Fake>();
  fake->steps_per_call = 2;
  Transactor t(std::move(fake), milliseconds(1000));
  ASSERT_EQ(t.reset(reset_request()).status, Status::OK);
  const StepResult r = t.step(request(1));
  EXPECT_EQ(r.done.status, Status::MISMATCH) << r.done.detail;
  EXPECT_EQ(r.done.backend_steps, 2u);
  // Fail closed: no later step runs.
  EXPECT_EQ(t.step(request(2)).done.status, Status::MISMATCH);
}

TEST(Transactor, NoEngineStepForARequestIsMismatch)
{
  auto fake = std::make_unique<Fake>();
  fake->steps_per_call = 0;
  Transactor t(std::move(fake), milliseconds(1000));
  ASSERT_EQ(t.reset(reset_request()).status, Status::OK);
  EXPECT_EQ(t.step(request(1)).done.status, Status::MISMATCH);
}

// The float clock is checked within half a step, and the drift is reported.
TEST(Transactor, ClockWithinHalfAStepPassesBeyondItIsMismatch)
{
  auto inside = std::make_unique<Fake>();
  inside->clock_skew_ns = kStepNs / 2;
  Transactor ok(std::move(inside), milliseconds(1000));
  ASSERT_EQ(ok.reset(reset_request()).status, Status::OK);
  const StepResult r = ok.step(request(1));
  EXPECT_EQ(r.done.status, Status::OK) << r.done.detail;
  EXPECT_EQ(r.done.clock_drift_ns, kStepNs / 2);

  auto outside = std::make_unique<Fake>();
  outside->clock_skew_ns = -(kStepNs / 2 + 1);
  Transactor bad(std::move(outside), milliseconds(1000));
  ASSERT_EQ(bad.reset(reset_request()).status, Status::OK);
  const StepResult b = bad.step(request(1));
  EXPECT_EQ(b.done.status, Status::MISMATCH);
  EXPECT_EQ(b.done.clock_drift_ns, -(kStepNs / 2 + 1));
}

// A reset does not repair a failed plant: after a MISMATCH the next episode's
// reset is refused with that status and no step runs.
TEST(Transactor, ResetAfterMismatchIsRefused)
{
  auto fake = std::make_unique<Fake>();
  fake->steps_per_call = 2;
  Transactor t(std::move(fake), milliseconds(1000));
  ASSERT_EQ(t.reset(reset_request()).status, Status::OK);
  ASSERT_EQ(t.step(request(1)).done.status, Status::MISMATCH);
  const CompletionRecord r = t.reset(reset_request(2));
  EXPECT_EQ(r.status, Status::MISMATCH) <<
    "reset after MISMATCH returned " << to_string(r.status) << ": the failure was cleared";
  const StepResult s = t.step(request(1, 0.0, 2));
  EXPECT_EQ(s.done.status, Status::MISMATCH) <<
    "step after a refused reset returned " << to_string(s.done.status);
}

TEST(Transactor, BackendExceptionIsBackendFault)
{
  auto fake = std::make_unique<Fake>();
  fake->throw_on_step = true;
  Transactor t(std::move(fake), milliseconds(1000));
  ASSERT_EQ(t.reset(reset_request()).status, Status::OK);
  const StepResult r = t.step(request(1));
  EXPECT_EQ(r.done.status, Status::BACKEND_FAULT);
  EXPECT_NE(r.done.detail.find("engine exploded"), std::string::npos) << r.done.detail;
}

// A4 in process: a step that never returns gives TIMEOUT after the timeout,
// not a hang, and any later call aborts (one outstanding transaction).
TEST(Transactor, AStepThatNeverReturnsTimesOut)
{
  static std::mutex mutex;
  static std::condition_variable cv;
  static bool released = false;
  auto fake = std::make_unique<Fake>();
  fake->block = true;
  fake->mutex = &mutex;
  fake->cv = &cv;
  fake->released = &released;
  auto t = std::make_unique<Transactor>(std::move(fake), milliseconds(100));
  ASSERT_EQ(t->reset(reset_request()).status, Status::OK);
  const auto t0 = std::chrono::steady_clock::now();
  const StepResult r = t->step(request(1));
  const auto waited = std::chrono::steady_clock::now() - t0;
  EXPECT_EQ(r.done.status, Status::TIMEOUT) << r.done.detail;
  EXPECT_GE(waited, milliseconds(100));
  EXPECT_LT(waited, milliseconds(1100)) << "the timeout is not supervised";
  EXPECT_TRUE(r.state.values.empty()) << "a timed-out step returned outputs";
  // Fail closed: the Transactor stays broken and never touches the backend again.
  const StepResult next = t->step(request(2));
  EXPECT_EQ(next.done.status, Status::TIMEOUT) << next.done.detail;
  EXPECT_TRUE(next.state.values.empty());
  t.reset();  // detaches the stuck plant thread instead of joining it
  {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
  }
  cv.notify_all();
}

// One outstanding transaction: a second caller while a step is in flight
// aborts the process instead of queueing behind it, and aborts before it
// touches the in-flight request. A SIGABRT handler reads the request the
// blocked step was given (brake 0.25) after the second caller (brake 0.75)
// has aborted. The plant thread publishes the pointer through a lock-free
// atomic, so the handler's load is async-signal-safe, and the second caller
// starts only after it has seen that store (acquire), never after a sleep.
std::atomic<const StepRequest *> g_in_flight{nullptr};
static_assert(
  std::atomic<const StepRequest *>::is_always_lock_free,
  "the SIGABRT handler needs a lock-free atomic");

void report_in_flight(int signum)
{
  static_cast<void>(signum);
  char text[128];
  const StepRequest * in_flight = g_in_flight.load(std::memory_order_acquire);
  const double brake = in_flight != nullptr ? in_flight->inputs.at(0).values.at(0) : -1.0;
  const int n = brake == 0.25 ?
    std::snprintf(text, sizeof(text), "in-flight step request intact\n") :
    std::snprintf(text, sizeof(text), "in-flight step request overwritten: brake %g, sent 0.25\n",
    brake);
  if (write(2, text, static_cast<size_t>(n)) < 0) {
    _exit(98);
  }
  std::signal(SIGABRT, SIG_DFL);
  std::raise(SIGABRT);
}

TEST(TransactorDeathTest, ASecondConcurrentTransactionAborts)
{
  EXPECT_DEATH(
      {
        static std::mutex mutex;
        static std::condition_variable cv;
        static bool released = false;
        auto fake = std::make_unique<Fake>();
        fake->block = true;
        fake->mutex = &mutex;
        fake->cv = &cv;
        fake->released = &released;
        g_in_flight.store(nullptr);
        fake->seen = &g_in_flight;
        auto t = std::make_unique<Transactor>(std::move(fake), milliseconds(5000));
        t->reset(reset_request());
        std::signal(SIGABRT, report_in_flight);
        std::thread first([&t] {t->step(request(1, 0.25));});
        // Wait until the plant thread holds the first request inside the
        // backend; the deadline only turns a hang into a distinct failure.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (g_in_flight.load(std::memory_order_acquire) == nullptr) {
          if (std::chrono::steady_clock::now() > deadline) {
            std::fprintf(stderr, "first step never reached the backend\n");
            _exit(97);
          }
          std::this_thread::yield();
        }
        t->step(request(1, 0.75));
        first.join();
  }, "a second plant transaction while one is outstanding\n(.|\n)*in-flight step request intact");
}

TEST(Transactor, ProductionLibraryRefusesTheFaultVariable)
{
  ASSERT_EQ(setenv("RCLLF_PLANT_FAULT", "hang:3", 1), 0);
  try {
    mujoco();
    ADD_FAILURE() << "rcllf_plant honoured RCLLF_PLANT_FAULT outside a fault build";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("no fault injection"), std::string::npos) << e.what();
  }
  unsetenv("RCLLF_PLANT_FAULT");
}

}  // namespace
}  // namespace rcllf_plant
