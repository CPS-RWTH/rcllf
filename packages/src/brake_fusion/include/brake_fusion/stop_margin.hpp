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

#ifndef BRAKE_FUSION__STOP_MARGIN_HPP_
#define BRAKE_FUSION__STOP_MARGIN_HPP_

#include <cmath>

namespace brake_fusion
{

// Full-stage TTC threshold from the stopping margin X (the car stops
// at least X before the obstacle). Worst case: the
// full stage alone, its threshold crossed at v0 (a lost or ineffective partial
// stage).
//
// Supported faults: none, or one
// lost message on any topic of the decision path (/radar/range of either
// radar, /camera/range, /radar/track, /fused/range, /wheel/speed,
// /brake_cmd), at any phase of the run. The bounds e_r and L below are the
// largest values over every such loss, so they hold with the loss, and the
// stop gap after one loss is still at least X (not merely positive).
// Outside the guarantee: two or more losses, a stall of a stream (stock
// reports it with the QoS deadlines while it lasts, rcllf not while it lasts,
// only with late deadline and lost events after the resume), a
// wheel speed that never arrives before t* (the controller then has no
// plausible closing speed, select_closing_speed, and keeps the last level),
// and /sim/state, which models the physical sensing, not a ROS message.
//
// Let t* be the moment the true range reaches T (v0 - e_v) - e_r. The fused
// range over-reads by at most e_r and the closing speed under-reads by at
// most e_v, so every fused sample stamped at or after t* gives
// r_est / v_est <= T. The newest fused sample at a tick is at most A old, so
// the first tick at least A after t* brakes fully, and the brake acts at most
// L after t*. From t* the car travels v0 L until the brake acts and
// v0^2 / (2 a) while braking. Stop gap >= X needs
//   T = (X + e_r + v0 L + v0^2 / (2 a)) / (v0 - e_v),
// rounded up to 1 ms (the plant step; rounding up only adds margin).
// The scenario terms below are the simulator defaults (simulator_node.cpp
// initial_velocity, max_decel). The worst case is at v0: the margin
// v (T - L) - v^2 / (2 a) is concave in v, so over the crossing speeds
// [6.8, 15] m/s (ideal-timing oracle: the partial stage leaves 6.81 m/s at the full
// crossing) its minimum is at an end, and at T >= 1.06 s that end is v0.
//
// X itself lives here, once: the controller parameter
// stop_margin_m defaults to kStopMargin, a negative value selects that
// default (what the launch files pass unless stop_margin_m:=... is given),
// and k7_model.py derives its threshold from these constants.
// Owner decision 2026-11-03: X = 0.45 m is final, the travel at v0
// during one 30 ms camera period (the recommendation of the stop-margin study, which compared
// candidates by their residual margin in the recorded runs).
constexpr double kStopMargin = 0.45;  // X, m
constexpr double kDesignSpeed = 15.0;  // v0, m/s
constexpr double kDesignDecel = 8.0;  // a, m/s^2, applied in the next 1 ms step
// L, s: the largest sensing-to-actuation latency of the rcllf topologies
// with up to one lost message, ideal-timing oracle at this commit. Scanned: every
// single loss of the decision path at every integer-ms phase of the 25 ms
// tick against the sensors (12820 oracle runs, full stage only, by a loss-scan script;
// test_latency_bound_under_one_loss rescans a subset that holds the maxima):
//   A, the fused age at a tick: 45 ms without a loss at tick phase 0, 46 ms
//   at the worst phase (two hosts, 5 ms after delay; one program 40 and
//   42 ms); a lost camera or fused sample leaves the newest pair one pair
//   interval older, 81 ms at the worst phase (one program 77 ms), the
//   largest of every loss;
//   plus the 25 ms tick that may just miss the crossing;
//   plus 7 ms from the commanding tick to the applied step (2 ms in one
//   program). A development branch (91e3ae71) models that term as 6 ms, matching
//   the two-host run logs; the larger 7 ms is kept until that model lands.
// That is 113 ms (one program 104 ms). A lost brake command instead adds
// one tick to an unchanged age, at most 103 ms. An earlier value of 77 ms (no
// loss, tick phase 0) was short by what one lost camera sample adds (a
// review finding: a radar2 loss at 2920 ms moved the full brake from 2952 to
// 3052 ms in the nominal run, relative to the no-loss decision, which acts
// before t* + L).
// With the generator default `after 0` on local links the same scan
// gives 111 ms (two hosts, d = 5 ms) and 102 ms (one program); 0.113 stays
// an upper bound under both rules (test_latency_bound_under_one_loss).
// Provenance: a design choice keeps 0.113 over
// a DERIVED bound two 1 ms steps above the 111 ms maximum, so the tune
// (X = 0.45 m, ttc_full) and the recorded campaign stay unchanged.
constexpr double kLatencyBound = 0.113;
// e_r, m: the largest fused range over-read before the stop, against the
// plant state the simulator publishes at the fused stamp (the state stamped
// t already holds the position after that step, so a sensor sample stamped
// t, which read the state stamped t - 1 ms, is 1 ms old), with up to one lost
// message. Without a loss: radar2 bias 0.5 m through the 0.5-gain tracker,
// the mean with the camera and that 1 ms of travel, 0.198336 m (ideal-timing oracle;
// 0.1983 m in every recorded rcllf run). A lost unbiased radar sample lets
// the tracker take two biased samples in a row: 0.323336 m in the oracle,
// the largest of every loss (a radar loss at 140 or 240 ms, at constant
// speed). One recorded Orin run (radar sample 600 ms lost) measured
// 0.315318 m, which the oracle reproduces exactly (the regression
// test_range_over_read_under_one_loss). Rounded up to 1 mm. An earlier
// value of 0.199 m held only without a loss.
constexpr double kRangeOverRead = 0.324;

// e_v is the controller's plausibility tolerance: the stamp estimate is used
// only within that of the wheel speed, and the wheel speed never reads below
// the true speed while braking (it is the last received state, up to 11 ms
// old), so the closing speed under-reads by at most the tolerance.
//
// d_b, s: the modeled brake actuation delay (simulator
// parameter brake_actuation_delay_ms): a command stamped t acts in the first
// plant step strictly after t + d_b (brake_actuator.hpp). The latency term
// becomes L + d_b. It is an upper bound:
// L already holds the command's transit to the applied step (the last 7 ms
// term above, at least the 1 ms of the first step after t), and with d_b the
// command acts in the first step strictly after max(t + d_b, its receipt),
// at most 1 ms after t + d_b or that transit term after t, so at most the
// transit term plus d_b after t. At X = 0.45 m and e_v = 0.5 m/s (sum of the numerator at d_b = 0:
// 0.45 + 0.324 + 15 * 0.113 + 15^2 / 16 = 16.5315 m, over 14.5 m/s):
//   d_b = 0:      16.5315 / 14.5 = 1.14010 s, rounded up 1.141 s (today's value)
//   d_b = 25 ms:  (16.5315 + 0.375) / 14.5 = 1.16597 s, rounded up 1.166 s
//   d_b = 100 ms: (16.5315 + 1.5) / 14.5 = 1.24355 s, rounded up 1.244 s
// (test_retune.py test_ttc_full_with_the_actuation_delay pins these.)
inline double full_stage_ttc(
  double stop_margin_m, double speed_under_read, double brake_delay_s = 0.0)
{
  const double t = (stop_margin_m + kRangeOverRead +
    kDesignSpeed * (kLatencyBound + brake_delay_s) +
    kDesignSpeed * kDesignSpeed / (2.0 * kDesignDecel)) / (kDesignSpeed - speed_under_read);
  return std::ceil(t * 1000.0 - 1e-9) / 1000.0;
}

}  // namespace brake_fusion

#endif  // BRAKE_FUSION__STOP_MARGIN_HPP_
