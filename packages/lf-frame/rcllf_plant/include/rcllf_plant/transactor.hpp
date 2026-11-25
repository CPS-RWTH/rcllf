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

#ifndef RCLLF_PLANT__TRANSACTOR_HPP_
#define RCLLF_PLANT__TRANSACTOR_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "rcllf_plant/plant_adapter.hpp"

namespace rcllf_plant
{

struct TransactorShared;

// The wrapper every backend sits behind, so no backend can
// forget a rule:
//
// - One outstanding transaction. A second reset, step or probe while one
//   runs aborts the process before it touches any shared or validation
//   state: it is a programming error. After a TIMEOUT (the plant thread may
//   still be inside the backend) nothing is dispatched again: every later
//   call returns TIMEOUT.
// - A request is STALE, refused before the backend is touched, when no reset
//   happened, its episode or step_ns is not the current reset's, its step_id
//   is not the previous plus one, its logical_ns is not step_id * step_ns, or
//   its inputs are not exactly the Descriptor's channels in order.
// - Advancement is judged by the backend's own step counter (Probe): after a
//   step it must equal step_id, else MISMATCH. The backend clock is a float
//   in MuJoCo, so it is checked within a tolerance, |clock - logical_ns| <=
//   step_ns / 2, else MISMATCH; the signed drift is reported every step. Half
//   a step is the largest tolerance that still tells one step from zero or
//   two; the float error of 10^6 additions of 1 ms is about 110 ns.
// - The backend never runs on the caller's thread. One plant thread, created
//   here, makes every backend call; the caller waits at most `step_timeout`
//   and gets TIMEOUT when the backend has not completed, while the plant
//   thread stays wherever it is stuck. The caller then fails closed (the
//   simulator logs plant_timeout and leaves through std::quick_exit(3)).
//   Outputs are copied out of the plant thread only on completion.
// - A backend exception is BACKEND_FAULT. After any status but OK and STALE
//   the Transactor refuses every later step and reset with that status: a
//   reset does not repair a failed plant.
//
// Fault injection (acceptance A4) is compiled only into the test library
// rcllf_plant_fault: RCLLF_PLANT_FAULT=hang:<step> blocks the plant thread
// forever at the dispatch of that step, delay:<step>:<ms> sleeps <ms> there
// and then runs the step, engine_hang:<iteration> is accepted and left to
// the Gazebo backend's test library (a hang inside gz-sim's PreUpdate).
// The production library refuses to start when the
// variable is set, so a test can never pass against a build without the hook.
class Transactor
{
public:
  Transactor(std::unique_ptr<PlantAdapter> backend, std::chrono::milliseconds step_timeout);
  ~Transactor();
  Transactor(const Transactor &) = delete;
  Transactor & operator=(const Transactor &) = delete;

  const Descriptor & descriptor() const {return descriptor_;}
  std::chrono::milliseconds step_timeout() const {return step_timeout_;}

  // Starts episode request.episode (previous + 1). Throws
  // std::invalid_argument for an unseeded request, a wrong episode, a step_ns
  // other than the backend's or an initial state of the wrong size. The
  // record carries the step 0 state hash (output_hash 0, nothing is output at
  // a reset); MISMATCH when the backend's counter or clock is not 0 after it.
  // Refused with the earlier status after a TIMEOUT, BACKEND_FAULT or MISMATCH.
  CompletionRecord reset(const ResetRequest & request);

  StepResult step(const StepRequest & request);

  // The backend's counter, clock and state hash, without advancing (A2).
  CompletionRecord probe();

  // The fault this build honours ("" when none or no fault build).
  const char * fault_spec() const;

private:
  // Runs one job on the plant thread; false on timeout, with the time it
  // waited in timeout_wait_ns_ (the TIMEOUT record's wall_ns).
  bool dispatch(int kind);
  CompletionRecord refuse(const StepRequest & request, Status status, const std::string & why);

  std::shared_ptr<TransactorShared> shared_;
  std::thread thread_;
  Descriptor descriptor_;
  std::chrono::milliseconds step_timeout_;
  std::atomic<bool> busy_{false};
  bool timed_out_{false};
  int64_t timeout_wait_ns_{0};
  Status broken_{Status::OK};
  bool reset_done_{false};
  uint64_t episode_{0};
  int64_t step_ns_{0};
  uint64_t last_step_id_{0};
  Probe last_;
};

}  // namespace rcllf_plant

#endif  // RCLLF_PLANT__TRANSACTOR_HPP_
