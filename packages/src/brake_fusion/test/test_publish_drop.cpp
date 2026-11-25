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

// The node-level injected loss drops exactly the one
// selected publication, and nothing when it is off (the default).

#include <gtest/gtest.h>

#include "brake_fusion/publish_drop.hpp"

using brake_fusion::PublishDrop;

TEST(PublishDrop, OffByDefaultDropsNothing)
{
  PublishDrop d(-1, -1.0);
  EXPECT_FALSE(d.enabled());
  for (int64_t k = 0; k < 100; ++k) {
    EXPECT_FALSE(d.hit(k * 20'000'000, 1.0));
  }
}

TEST(PublishDrop, StampDropsOnlyThatStampOnce)
{
  PublishDrop d(600, -1.0);
  EXPECT_TRUE(d.enabled());
  int dropped = 0;
  int64_t at = -1;
  for (int64_t k = 0; k < 100; ++k) {
    if (d.hit(k * 20'000'000, 0.0)) {
      ++dropped;
      at = k * 20'000'000;
    }
  }
  EXPECT_EQ(dropped, 1);
  EXPECT_EQ(at, 600'000'000);
  EXPECT_FALSE(d.hit(600'000'000, 0.0));  // a repeated stamp is not dropped again
}

TEST(PublishDrop, FirstGeDropsTheFirstValueAtOrAboveOnce)
{
  PublishDrop d(-1, 0.4);
  EXPECT_FALSE(d.hit(-1, 0.0));
  EXPECT_FALSE(d.hit(-1, 0.39));
  EXPECT_TRUE(d.hit(-1, 0.4));
  EXPECT_FALSE(d.hit(-1, 1.0));
  EXPECT_FALSE(d.hit(-1, 0.4));
}
