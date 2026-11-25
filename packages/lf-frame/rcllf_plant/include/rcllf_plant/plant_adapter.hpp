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

#ifndef RCLLF_PLANT__PLANT_ADAPTER_HPP_
#define RCLLF_PLANT__PLANT_ADAPTER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// The rcllf plant boundary. One transaction
// per logical step; the caller is the only clock authority. Backends implement
// PlantAdapter and are only ever called through the Transactor
// (transactor.hpp), which enforces the transaction rules.
namespace rcllf_plant
{

enum class Status {OK, STALE, TIMEOUT, BACKEND_FAULT, MISMATCH};

const char * to_string(Status status);

// One named input or output channel and its values.
struct Channel
{
  std::string name;
  std::vector<double> values;
};

// Every input channel of the step, complete and in Descriptor::inputs order.
// No hold-last inside the plant: the caller decides what an input is.
using InputSnapshot = std::vector<Channel>;

// The backend state the application reads, in Descriptor::state order.
struct StateSnapshot
{
  std::vector<double> values;
};

// A sensor output due at this step, `name` one of Descriptor::outputs.
using SensorOutput = Channel;

struct Descriptor
{
  std::string backend;               // "mujoco", ...
  std::string version;               // the engine's own version string
  int64_t step_ns{0};                // the backend's configured step
  std::vector<std::string> inputs;   // input channel names, request order
  std::vector<std::string> state;    // StateSnapshot value names
  std::vector<std::string> outputs;  // sensor output channel names, hash order
  // The lowest value a reset may start each state entry at, in state order;
  // empty means no bound. The Transactor refuses a reset below it, and any
  // non-finite initial state, before the backend sees the request.
  std::vector<double> state_min;
};

struct ResetRequest
{
  uint64_t episode{0};              // previous episode + 1, 1 on the first reset
  std::optional<uint64_t> seed;     // required: an unseeded reset is refused
  int64_t step_ns{0};               // fixed for the episode, must equal the backend's
  std::vector<double> initial_state;  // Descriptor::state order
};

struct StepRequest
{
  uint64_t episode{0};   // the episode of the current reset
  uint64_t step_id{0};   // previous step_id + 1, the first request is 1
  int64_t logical_ns{0};  // the caller's time of this step = step_id * step_ns
  int64_t step_ns{0};    // the episode's step
  InputSnapshot inputs;
};

struct CompletionRecord
{
  uint64_t episode{0};
  uint64_t step_id{0};
  uint64_t backend_steps{0};    // the backend's step counter, must equal step_id
  int64_t backend_clock_ns{0};  // the backend's own clock, checked within step_ns / 2
  int64_t clock_drift_ns{0};    // backend_clock_ns - logical_ns
  uint64_t state_hash{0};       // canonical hash of the full backend state (hash.hpp)
  uint64_t output_hash{0};      // canonical hash of every output of this step
  int64_t wall_ns{0};           // wall time of the step, for the timing budget only;
                                // on TIMEOUT the time waited before giving up
  Status status{Status::OK};
  std::string detail;           // why, for any status but OK
};

struct StepResult
{
  StateSnapshot state;
  std::vector<SensorOutput> outputs;
  CompletionRecord done;
};

// What a backend reports about itself without advancing: its step counter,
// its clock and the hash of its full state. The Transactor reads it after
// every reset and step, so advancement is judged by the backend's own
// counter, never by what step() claims.
struct Probe
{
  uint64_t backend_steps{0};
  int64_t backend_clock_ns{0};
  uint64_t state_hash{0};
};

// A backend. Constructed paused: nothing advances without a step() call.
// Called only from the Transactor's plant thread, one call at a time.
class PlantAdapter
{
public:
  virtual ~PlantAdapter() = default;
  // Backend name, version, step_ns and channels.
  virtual Descriptor describe() const = 0;
  // Episode, seed and initial state; the step counter and clock restart at 0.
  // The Transactor fills the record from probe().
  virtual void reset(const ResetRequest & request) = 0;
  // Exactly one engine step with the request's inputs. Returns the state the
  // application reads and every sensor output due at this step. Throws on a
  // backend fault.
  virtual StepResult step(const StepRequest & request) = 0;
  virtual Probe probe() const = 0;
};

}  // namespace rcllf_plant

#endif  // RCLLF_PLANT__PLANT_ADAPTER_HPP_
