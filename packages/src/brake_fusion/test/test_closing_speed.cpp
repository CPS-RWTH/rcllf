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

#include <gtest/gtest.h>

#include <string>

#include "brake_fusion/closing_speed.hpp"

using brake_fusion::select_closing_speed;

// The recorded case. A stock two-host Orin run at idle, tick at 126.6 ms: stamp
// estimate 16.75 m/s, no wheel sample yet. The old rule used the estimate
// ("stamps", TTC 2.303 s below ttc_partial 2.45 s, partial brake at step 129).
TEST(ClosingSpeed, EstimateWithoutWheelIsNotUsed)
{
  const auto c = select_closing_speed(true, 16.75, false, 0.0, 0.5);
  EXPECT_EQ(std::string(c.source), "unchecked") <<
    "an estimate with no wheel sample to check it against was used";
  EXPECT_EQ(c.speed, 0.0);
}

TEST(ClosingSpeed, CheckedEstimateIsUsed)
{
  const auto c = select_closing_speed(true, 15.2, true, 15.0, 0.5);
  EXPECT_EQ(std::string(c.source), "stamps");
  EXPECT_DOUBLE_EQ(c.speed, 15.2);
}

TEST(ClosingSpeed, ImplausibleEstimateFallsBackToWheel)
{
  const auto c = select_closing_speed(true, -4.9, true, 15.0, 0.5);
  EXPECT_EQ(std::string(c.source), "wheel_implausible");
  EXPECT_DOUBLE_EQ(c.speed, 15.0);
}

TEST(ClosingSpeed, WheelAloneIsTheFallback)
{
  const auto c = select_closing_speed(false, 0.0, true, 15.0, 0.5);
  EXPECT_EQ(std::string(c.source), "wheel_fallback");
  EXPECT_DOUBLE_EQ(c.speed, 15.0);
  const auto none = select_closing_speed(false, 0.0, false, 0.0, 0.5);
  EXPECT_EQ(std::string(none.source), "none");
  EXPECT_EQ(none.speed, 0.0);
}
