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

#ifndef BRAKE_FUSION__PUBLISH_DROP_HPP_
#define BRAKE_FUSION__PUBLISH_DROP_HPP_

#include <cstdint>

namespace brake_fusion
{

// Node-level injected loss of exactly one publication .
// A drop shim at rcl_take does not see an intra-process job, which never
// calls it, so such runs drop at the sender instead: the selected message
// is built and logged as usual and then not published. Off by default.
//   stamp_ms >= 0      drop the message whose header stamp is stamp_ms
//                      (a loss written `radar@MS` / `camera@MS`)
//   first_ge >= 0      drop the first message whose value is >= first_ge
//                      (a loss written `brake_first`, value = brake command)
// Either selector fires at most once per node.
class PublishDrop
{
public:
  PublishDrop(int64_t stamp_ms, double first_ge)
  : stamp_ns_(stamp_ms >= 0 ? stamp_ms * 1'000'000 : -1), first_ge_(first_ge) {}

  bool enabled() const {return stamp_ns_ >= 0 || first_ge_ >= 0.0;}

  // True when this publication is the one to drop (and marks it used).
  bool hit(int64_t stamp_ns, double value)
  {
    if (used_) {
      return false;
    }
    const bool selected = (stamp_ns_ >= 0 && stamp_ns == stamp_ns_) ||
      (first_ge_ >= 0.0 && value >= first_ge_);
    used_ = selected;
    return selected;
  }

private:
  int64_t stamp_ns_;
  double first_ge_;
  bool used_{false};
};

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__PUBLISH_DROP_HPP_
