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

#include "rcllf_plant/transactor.hpp"

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcllf_plant/hash.hpp"

namespace rcllf_plant
{

const char * to_string(Status status)
{
  switch (status) {
    case Status::OK: return "OK";
    case Status::STALE: return "STALE";
    case Status::TIMEOUT: return "TIMEOUT";
    case Status::BACKEND_FAULT: return "BACKEND_FAULT";
    case Status::MISMATCH: return "MISMATCH";
  }
  return "UNKNOWN";
}

namespace
{
enum JobKind {kNone = 0, kReset = 1, kStep = 2, kProbe = 3};

struct Fault
{
  enum Kind {kOff, kHang, kDelay, kEngine} kind{kOff};
  uint64_t step{0};
  int64_t delay_ms{0};
  std::string spec;
};

Fault parse_fault()
{
  Fault fault;
  const char * env = std::getenv("RCLLF_PLANT_FAULT");
  if (env == nullptr || env[0] == '\0') {
    return fault;
  }
#ifdef RCLLF_PLANT_FAULT_INJECTION
  fault.spec = env;
  unsigned long long step = 0;  // NOLINT(runtime/int): sscanf's %llu
  long long ms = 0;  // NOLINT(runtime/int): sscanf's %lld
  char tail = 0;
  const bool hang = std::sscanf(env, "hang:%llu%c", &step, &tail) == 1 && step > 0;
  const bool delay = !hang && std::sscanf(env, "delay:%llu:%lld%c", &step, &ms, &tail) == 2 &&
    step > 0 && ms > 0;
  // engine_hang:<k> is the Gazebo backend's own fault, read by
  // rcllf_plant_gazebo_fault: the dispatch here does nothing for it.
  const bool engine = !hang && !delay &&
    std::sscanf(env, "engine_hang:%llu%c", &step, &tail) == 1 && step > 0;
  if (hang) {
    fault.kind = Fault::kHang;
  } else if (delay) {
    fault.kind = Fault::kDelay;
    fault.delay_ms = ms;
  } else if (engine) {
    fault.kind = Fault::kEngine;
  } else {
    throw std::invalid_argument(
            std::string("RCLLF_PLANT_FAULT must be hang:<step>, delay:<step>:<ms> or "
            "engine_hang:<iteration>, got ") + env);
  }
  fault.step = step;
  return fault;
#else
  throw std::runtime_error(
          "RCLLF_PLANT_FAULT is set but this rcllf_plant build has no fault injection "
          "(test library rcllf_plant_fault only)");
#endif
}

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

// Everything the plant thread touches. Shared, so it outlives a Transactor
// whose plant thread is stuck in the backend after a TIMEOUT.
struct TransactorShared
{
  std::unique_ptr<PlantAdapter> backend;
  Fault fault;
  std::mutex mutex;
  std::condition_variable cv;
  int job{kNone};
  bool done{false};
  bool stop{false};
  ResetRequest reset_request;
  StepRequest step_request;
  StepResult result;
  Probe probe;
  int64_t wall_ns{0};
  std::exception_ptr error;
};

namespace
{
void plant_thread(std::shared_ptr<TransactorShared> s)
{
  std::unique_lock<std::mutex> lock(s->mutex);
  for (;; ) {
    s->cv.wait(lock, [&s] {return s->job != kNone || s->stop;});
    if (s->job == kNone) {
      return;  // stop
    }
    const int job = s->job;
    s->job = kNone;
    lock.unlock();
    StepResult result;
    Probe probe;
    std::exception_ptr error;
    int64_t wall_ns = 0;
    try {
      const int64_t t0 = steady_ns();
      if (job == kStep) {
        // A4: the one dispatch point every backend's step passes through.
        const Fault & f = s->fault;
        if ((f.kind == Fault::kHang || f.kind == Fault::kDelay) &&
          s->step_request.step_id == f.step)
        {
          if (f.kind == Fault::kHang) {
            for (;; ) {
              std::this_thread::sleep_for(std::chrono::hours(1));
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(f.delay_ms));
        }
        result = s->backend->step(s->step_request);
      } else if (job == kReset) {
        s->backend->reset(s->reset_request);
      }
      probe = s->backend->probe();
      wall_ns = steady_ns() - t0;
    } catch (...) {
      error = std::current_exception();
    }
    lock.lock();
    s->result = std::move(result);
    s->probe = probe;
    s->wall_ns = wall_ns;
    s->error = error;
    s->done = true;
    s->cv.notify_all();
  }
}

// One outstanding transaction. Taken first thing in every public transaction,
// before shared_ or the validation state is touched, and released only after
// the completion has been copied out and the transaction state updated, so a
// second caller can never overwrite a request in flight or a result not yet
// read.
class OneOutstanding
{
public:
  explicit OneOutstanding(std::atomic<bool> & busy)
  : busy_(busy)
  {
    if (busy_.exchange(true)) {
      std::fprintf(stderr, "rcllf_plant: a second plant transaction while one is outstanding\n");
      std::abort();
    }
  }
  ~OneOutstanding() {busy_.store(false);}
  OneOutstanding(const OneOutstanding &) = delete;
  OneOutstanding & operator=(const OneOutstanding &) = delete;

private:
  std::atomic<bool> & busy_;
};

std::string error_text(const std::exception_ptr & error)
{
  try {
    std::rethrow_exception(error);
  } catch (const std::exception & e) {
    return e.what();
  } catch (...) {
    return "unknown exception";
  }
}
}  // namespace

Transactor::Transactor(
  std::unique_ptr<PlantAdapter> backend, std::chrono::milliseconds step_timeout)
: shared_(std::make_shared<TransactorShared>()), step_timeout_(step_timeout)
{
  if (!backend) {
    throw std::invalid_argument("Transactor needs a backend");
  }
  if (step_timeout.count() <= 0) {
    throw std::invalid_argument("plant step timeout must be > 0 ms");
  }
  shared_->fault = parse_fault();
  // Construction is single-threaded: the plant thread does not exist yet.
  descriptor_ = backend->describe();
  if (descriptor_.step_ns <= 0) {
    throw std::invalid_argument("backend step_ns must be > 0");
  }
  if (!descriptor_.state_min.empty() && descriptor_.state_min.size() != descriptor_.state.size()) {
    throw std::invalid_argument("backend state_min must be empty or one value per state entry");
  }
  shared_->backend = std::move(backend);
  thread_ = std::thread(plant_thread, shared_);
}

Transactor::~Transactor()
{
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->stop = true;
  }
  shared_->cv.notify_all();
  if (timed_out_) {
    // The plant thread is inside the backend and may never return; joining
    // would hang this destructor. The shared state stays alive with it, and
    // stop makes the thread exit if the backend call ever does complete.
    thread_.detach();
    return;
  }
  thread_.join();
}

const char * Transactor::fault_spec() const
{
  return shared_->fault.spec.c_str();
}

bool Transactor::dispatch(int kind)
{
  std::unique_lock<std::mutex> lock(shared_->mutex);
  shared_->job = kind;
  shared_->done = false;
  shared_->cv.notify_all();
  const int64_t t0 = steady_ns();
  if (!shared_->cv.wait_for(lock, step_timeout_, [this] {return shared_->done;})) {
    // The plant thread may still be inside the backend: no later call
    // dispatches again (broken_ is TIMEOUT, probe() checks timed_out_).
    // The wait is recorded so a supervisor can check the timeout fired
    // on time (A4), not merely that the step failed.
    timeout_wait_ns_ = steady_ns() - t0;
    timed_out_ = true;
    return false;
  }
  return true;
}

CompletionRecord Transactor::refuse(
  const StepRequest & request, Status status, const std::string & why)
{
  CompletionRecord done;
  done.episode = request.episode;
  done.step_id = request.step_id;
  done.backend_steps = last_.backend_steps;
  done.backend_clock_ns = last_.backend_clock_ns;
  done.state_hash = last_.state_hash;
  done.status = status;
  done.detail = why;
  return done;
}

CompletionRecord Transactor::reset(const ResetRequest & request)
{
  OneOutstanding guard(busy_);
  if (broken_ != Status::OK && broken_ != Status::STALE) {
    // A failed plant is not repaired by a reset: TIMEOUT, BACKEND_FAULT and
    // MISMATCH are terminal for this Transactor.
    CompletionRecord done;
    done.episode = request.episode;
    done.backend_steps = last_.backend_steps;
    done.backend_clock_ns = last_.backend_clock_ns;
    done.state_hash = last_.state_hash;
    done.status = broken_;
    done.detail = "an earlier plant transaction failed";
    return done;
  }
  if (!request.seed) {
    throw std::invalid_argument("plant reset without a seed is refused");
  }
  if (request.episode != episode_ + 1) {
    throw std::invalid_argument(
            "plant reset episode " + std::to_string(request.episode) + ", want " +
            std::to_string(episode_ + 1));
  }
  if (request.step_ns != descriptor_.step_ns) {
    throw std::invalid_argument(
            "plant reset step_ns " + std::to_string(request.step_ns) + " differs from the " +
            descriptor_.backend + " backend's " + std::to_string(descriptor_.step_ns));
  }
  if (request.initial_state.size() != descriptor_.state.size()) {
    throw std::invalid_argument("plant reset initial state has the wrong size");
  }
  for (size_t i = 0; i < request.initial_state.size(); ++i) {
    const double v = request.initial_state[i];
    const bool below = !descriptor_.state_min.empty() && v < descriptor_.state_min[i];
    if (!std::isfinite(v) || below) {
      std::ostringstream what;
      what.precision(17);
      what << "plant reset initial " << descriptor_.state[i] << " " << v;
      if (below) {
        what << " is below the " << descriptor_.backend << " backend's minimum " <<
          descriptor_.state_min[i];
      } else {
        what << " is not finite";
      }
      throw std::invalid_argument(what.str());
    }
  }
  CompletionRecord done;
  done.episode = request.episode;
  shared_->reset_request = request;
  if (!dispatch(kReset)) {
    done.status = Status::TIMEOUT;
    done.detail = "reset not complete within " + std::to_string(step_timeout_.count()) + " ms";
    done.wall_ns = timeout_wait_ns_;
    broken_ = Status::TIMEOUT;
    return done;
  }
  reset_done_ = false;
  if (shared_->error) {
    done.status = broken_ = Status::BACKEND_FAULT;
    done.detail = error_text(shared_->error);
    return done;
  }
  last_ = shared_->probe;
  done.backend_steps = last_.backend_steps;
  done.backend_clock_ns = last_.backend_clock_ns;
  done.clock_drift_ns = last_.backend_clock_ns;
  done.state_hash = last_.state_hash;
  done.wall_ns = shared_->wall_ns;
  if (last_.backend_steps != 0 || last_.backend_clock_ns != 0) {
    done.status = broken_ = Status::MISMATCH;
    done.detail = "backend counter or clock not 0 after reset";
    return done;
  }
  broken_ = Status::OK;
  reset_done_ = true;
  episode_ = request.episode;
  step_ns_ = request.step_ns;
  last_step_id_ = 0;
  return done;
}

StepResult Transactor::step(const StepRequest & request)
{
  OneOutstanding guard(busy_);
  StepResult out;
  if (broken_ != Status::OK) {
    out.done = refuse(request, broken_, "an earlier plant transaction failed");
    return out;
  }
  if (!reset_done_) {
    out.done = refuse(request, Status::STALE, "no reset");
    return out;
  }
  if (request.episode != episode_) {
    out.done = refuse(
      request, Status::STALE, "episode " + std::to_string(request.episode) + ", current " +
      std::to_string(episode_));
    return out;
  }
  if (request.step_ns != step_ns_) {
    out.done = refuse(
      request, Status::STALE, "step_ns " + std::to_string(request.step_ns) + ", episode's " +
      std::to_string(step_ns_));
    return out;
  }
  if (request.step_id != last_step_id_ + 1) {
    out.done = refuse(
      request, Status::STALE, "step_id " + std::to_string(request.step_id) + ", want " +
      std::to_string(last_step_id_ + 1));
    return out;
  }
  const uint64_t max_id = static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / step_ns_);
  if (request.step_id > max_id ||
    request.logical_ns != static_cast<int64_t>(request.step_id) * step_ns_)
  {
    out.done = refuse(
      request, Status::STALE, "logical_ns " + std::to_string(request.logical_ns) +
      " is not step_id * step_ns");
    return out;
  }
  bool inputs_ok = request.inputs.size() == descriptor_.inputs.size();
  for (size_t i = 0; inputs_ok && i < request.inputs.size(); ++i) {
    inputs_ok = request.inputs[i].name == descriptor_.inputs[i];
  }
  if (!inputs_ok) {
    out.done = refuse(request, Status::STALE, "inputs are not the descriptor's channels");
    return out;
  }

  shared_->step_request = request;
  if (!dispatch(kStep)) {
    out.done = refuse(
      request, Status::TIMEOUT, "step not complete within " +
      std::to_string(step_timeout_.count()) + " ms");
    out.done.wall_ns = timeout_wait_ns_;
    broken_ = Status::TIMEOUT;
    return out;
  }
  CompletionRecord & done = out.done;
  done.episode = request.episode;
  done.step_id = request.step_id;
  if (shared_->error) {
    done = refuse(request, Status::BACKEND_FAULT, error_text(shared_->error));
    broken_ = Status::BACKEND_FAULT;
    return out;
  }
  out.state = std::move(shared_->result.state);
  out.outputs = std::move(shared_->result.outputs);
  last_ = shared_->probe;
  done.backend_steps = last_.backend_steps;
  done.backend_clock_ns = last_.backend_clock_ns;
  done.clock_drift_ns = last_.backend_clock_ns - request.logical_ns;
  done.state_hash = last_.state_hash;
  done.wall_ns = shared_->wall_ns;

  // Output hash: the state readout, then each due output as (channel index,
  // values), outputs in strictly increasing Descriptor order.
  CanonicalHash hash;
  hash.add(out.state.values);
  bool outputs_ok = out.state.values.size() == descriptor_.state.size();
  size_t next = 0;
  for (const SensorOutput & o : out.outputs) {
    size_t i = next;
    while (i < descriptor_.outputs.size() && descriptor_.outputs[i] != o.name) {
      ++i;
    }
    if (i == descriptor_.outputs.size()) {
      outputs_ok = false;
      break;
    }
    hash.add(static_cast<double>(i));
    hash.add(o.values);
    next = i + 1;
  }
  done.output_hash = hash.value();
  if (!outputs_ok) {
    done.status = broken_ = Status::BACKEND_FAULT;
    done.detail = "state or outputs do not match the descriptor";
    return out;
  }
  if (last_.backend_steps != request.step_id) {
    done.status = broken_ = Status::MISMATCH;
    done.detail = "backend counter " + std::to_string(last_.backend_steps) + " after step_id " +
      std::to_string(request.step_id);
    return out;
  }
  const int64_t drift = done.clock_drift_ns < 0 ? -done.clock_drift_ns : done.clock_drift_ns;
  if (drift > step_ns_ / 2) {
    done.status = broken_ = Status::MISMATCH;
    done.detail = "backend clock drift " + std::to_string(done.clock_drift_ns) +
      " ns exceeds half a step";
    return out;
  }
  done.status = Status::OK;
  last_step_id_ = request.step_id;
  return out;
}

CompletionRecord Transactor::probe()
{
  OneOutstanding guard(busy_);
  CompletionRecord done;
  done.episode = episode_;
  done.step_id = last_step_id_;
  if (timed_out_) {
    done.status = Status::TIMEOUT;
    done.detail = "an earlier plant transaction timed out";
    return done;
  }
  if (!dispatch(kProbe)) {
    done.status = broken_ = Status::TIMEOUT;
    done.detail = "probe not complete within " + std::to_string(step_timeout_.count()) + " ms";
    done.wall_ns = timeout_wait_ns_;
    return done;
  }
  if (shared_->error) {
    done.status = Status::BACKEND_FAULT;
    done.detail = error_text(shared_->error);
    return done;
  }
  const Probe & p = shared_->probe;
  done.backend_steps = p.backend_steps;
  done.backend_clock_ns = p.backend_clock_ns;
  done.state_hash = p.state_hash;
  return done;
}

}  // namespace rcllf_plant
