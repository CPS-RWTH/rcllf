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

#ifndef RCLLF_PLANT__MUJOCO_BACKEND_HPP_
#define RCLLF_PLANT__MUJOCO_BACKEND_HPP_

#include <cstdint>

#include "rcllf_plant/plant_adapter.hpp"

// The MuJoCo types stay out of this header, so a user of the backend needs
// the MuJoCo headers neither to compile nor to be analyzed by rcllf.
struct mjModel_;
struct mjData_;

namespace rcllf_plant
{

// The brake_fusion vehicle on MuJoCo: one 1500 kg body on a slide joint,
// the model of aeb_mujoco_sim with a 1 ms timestep so one mj_step is one
// simulator grid step. Input "brake" in [0, 1] acts as a force through
// qfrc_applied, capped at the force that stops the vehicle within the step.
// State "position", "velocity" (qpos[0], qvel[0]). No sensor outputs.
//
// backend_steps counts the mj_step calls (MuJoCo has no engine counter),
// incremented next to the only call; backend_clock_ns is data->time in ns;
// the state hash covers qpos and qvel, not time (a tolerance-checked float
// has no place in a bit-exact hash). MuJoCo has no randomness in this model:
// the seed is required by the Transactor and recorded, nothing reads it.
class MujocoBackend : public PlantAdapter
{
public:
  // Loads the embedded model. Throws std::runtime_error when it does not load
  // or does not have one slide joint and a `step_ns` timestep. Paused: the
  // state is all zero until reset() and nothing advances without step().
  MujocoBackend(double max_decel, int64_t step_ns);
  ~MujocoBackend() override;
  MujocoBackend(const MujocoBackend &) = delete;
  MujocoBackend & operator=(const MujocoBackend &) = delete;

  Descriptor describe() const override;
  void reset(const ResetRequest & request) override;
  StepResult step(const StepRequest & request) override;
  Probe probe() const override;

private:
  mjModel_ * model_{nullptr};
  mjData_ * data_{nullptr};
  double mass_{0.0};
  double max_decel_;
  int64_t step_ns_;
  double step_seconds_;
  uint64_t steps_{0};
};

}  // namespace rcllf_plant

#endif  // RCLLF_PLANT__MUJOCO_BACKEND_HPP_
