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

#include "brake_fusion/radar_tracker.hpp"

using brake_fusion::TrackFilter;

// The first measurement sets the estimate, every later one moves
// it by gain * (z - est).
TEST(TrackFilter, FirstSetsThenBlends)
{
  TrackFilter f(0.5);
  EXPECT_DOUBLE_EQ(f.update(10.0), 10.0);
  EXPECT_DOUBLE_EQ(f.update(11.0), 10.5);
  EXPECT_DOUBLE_EQ(f.update(10.5), 10.5);
  EXPECT_EQ(f.count(), 3);
}

// The update is order dependent on purpose: radar (z) then radar2 (z + 0.5)
// ends elsewhere than radar2 then radar.
TEST(TrackFilter, OrderOfOneStampChangesTheEstimate)
{
  TrackFilter rr2(0.5);
  rr2.update(20.0);
  rr2.update(10.0);
  const double a = rr2.update(10.5);
  TrackFilter r2r(0.5);
  r2r.update(20.0);
  r2r.update(10.5);
  const double b = r2r.update(10.0);
  EXPECT_DOUBLE_EQ(a, 12.75);
  EXPECT_DOUBLE_EQ(b, 12.625);
  EXPECT_NE(a, b);
}

TEST(TrackFilter, GainIsUsed)
{
  TrackFilter f(0.25);
  f.update(0.0);
  EXPECT_DOUBLE_EQ(f.update(4.0), 1.0);
}
