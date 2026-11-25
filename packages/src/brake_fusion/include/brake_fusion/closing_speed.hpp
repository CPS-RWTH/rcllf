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

#ifndef BRAKE_FUSION__CLOSING_SPEED_HPP_
#define BRAKE_FUSION__CLOSING_SPEED_HPP_

#include <cmath>

namespace brake_fusion
{

struct ClosingSpeed
{
  double speed{0.0};  // 0: no closing speed, so no TTC and no brake decision
  const char * source{"none"};
};

// The controller's closing speed. The stamp estimate (range
// difference of the last two fused pairs over their stamp difference) is used
// only when a wheel sample exists and the two agree within the plausibility
// tolerance; that check is what bounds the speed under-read e_v of the
// full-stage threshold (stop_margin.hpp). Before the first wheel sample the
// estimate cannot be checked, so it is not used ("unchecked"): a stock
// two-host run braked partially at 126.6 ms on an unchecked 16.75 m/s estimate
// (true 15 m/s) and its full stage never fired. No rcllf run
// and no ideal-timing oracle tick has a fused estimate but no wheel sample (checked on
// the recorded runs), so this rule changes no rcllf decision.
inline ClosingSpeed select_closing_speed(
  bool have_estimate, double estimate, bool have_wheel, double wheel_speed, double tolerance)
{
  if (have_estimate && !have_wheel) {
    return {0.0, "unchecked"};
  }
  if (have_estimate && std::fabs(estimate - wheel_speed) > tolerance) {
    return {wheel_speed, "wheel_implausible"};
  }
  if (have_estimate) {
    return {estimate, "stamps"};
  }
  if (have_wheel) {
    return {wheel_speed, "wheel_fallback"};
  }
  return {};
}

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__CLOSING_SPEED_HPP_
