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

#include "rcllf_plant/mujoco_backend.hpp"

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include "rcllf_plant/hash.hpp"

namespace rcllf_plant
{

namespace
{
// Below this speed after a step the wheels lock and the vehicle stands still.
constexpr double kLockSpeed = 1e-9;

// aeb_mujoco_sim's vehicle with a 1 ms step: no gravity, no contacts, no
// constraints, Euler (velocity first, then position with the new velocity).
constexpr char kModelXml[] =
  R"(
<mujoco model="brake_fusion_vehicle">
  <option timestep="0.001" integrator="Euler" gravity="0 0 0">
    <flag contact="disable" constraint="disable"/>
  </option>
  <worldbody>
    <body name="vehicle">
      <joint name="x" type="slide" axis="1 0 0"/>
      <inertial pos="0 0 0" mass="1500" diaginertia="1 1 1"/>
    </body>
  </worldbody>
</mujoco>
)";
}  // namespace

MujocoBackend::MujocoBackend(double max_decel, int64_t step_ns)
: max_decel_(max_decel), step_ns_(step_ns),
  // An exact integer over 1e9 rounds to the double nearest the true step,
  // the same double as the literal 0.001 of the model and the simulator.
  step_seconds_(static_cast<double>(step_ns) / 1e9)
{
  char error[1000] = "";
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(
    &vfs, "brake_fusion_vehicle.xml", kModelXml, static_cast<int>(std::strlen(kModelXml)));
  model_ = mj_loadXML("brake_fusion_vehicle.xml", &vfs, error, sizeof(error));
  mj_deleteVFS(&vfs);
  if (model_ == nullptr) {
    throw std::runtime_error(std::string("MuJoCo model load failed: ") + error);
  }
  if (model_->opt.timestep != step_seconds_ || model_->nq != 1 || model_->nv != 1) {
    mj_deleteModel(model_);
    throw std::runtime_error("MuJoCo model must have one slide joint and the simulator step");
  }
  data_ = mj_makeData(model_);
  mass_ = model_->body_mass[1];
}

MujocoBackend::~MujocoBackend()
{
  mj_deleteData(data_);
  mj_deleteModel(model_);
}

Descriptor MujocoBackend::describe() const
{
  Descriptor d;
  d.backend = "mujoco";
  d.version = mj_versionString();
  d.step_ns = step_ns_;
  d.inputs = {"brake"};
  d.state = {"position", "velocity"};
  // A forward-moving braking vehicle: the brake only opposes forward motion
  // and the lock below reads any velocity under kLockSpeed as a stop, so a
  // reset may not start it moving backwards.
  d.state_min = {-std::numeric_limits<double>::infinity(), 0.0};
  return d;
}

void MujocoBackend::reset(const ResetRequest & request)
{
  mj_resetData(model_, data_);
  data_->qpos[0] = request.initial_state.at(0);
  data_->qvel[0] = request.initial_state.at(1);
  steps_ = 0;
}

StepResult MujocoBackend::step(const StepRequest & request)
{
  const std::vector<double> & brake_in = request.inputs.at(0).values;
  if (brake_in.size() != 1) {
    throw std::invalid_argument("mujoco backend: input brake must have one value");
  }
  const double brake = brake_in[0];
  // A brake only resists motion, so the force is capped at what brings the
  // vehicle to rest within this step.
  const double velocity_before = data_->qvel[0];
  double force = 0.0;
  if (velocity_before > 0.0) {
    force = brake * max_decel_ * mass_;
    const double stop_force = mass_ * velocity_before / step_seconds_;
    if (force > stop_force) {
      force = stop_force;
    }
  }
  data_->qfrc_applied[0] = -force;
  mj_step(model_, data_);
  ++steps_;
  if (data_->qvel[0] < kLockSpeed) {
    data_->qvel[0] = 0.0;
  }
  StepResult result;
  result.state.values = {data_->qpos[0], data_->qvel[0]};
  return result;
}

Probe MujocoBackend::probe() const
{
  Probe p;
  p.backend_steps = steps_;
  p.backend_clock_ns = std::llround(data_->time * 1e9);
  CanonicalHash h;
  for (int i = 0; i < model_->nq; ++i) {
    h.add(data_->qpos[i]);
  }
  for (int i = 0; i < model_->nv; ++i) {
    h.add(data_->qvel[i]);
  }
  p.state_hash = h.value();
  return p;
}

}  // namespace rcllf_plant
