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

// A set but unreadable SENTRY_SCENARIO refuses to start instead of
// silently running every node on its declared defaults.

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

#include "sentry_pipeline/scenario_env.hpp"

using sentry_pipeline::ScenarioOverlay;

TEST(ScenarioOverlay, UnsetVariableGivesAnEmptyOverlay)
{
  unsetenv("SENTRY_SCENARIO");
  const auto overlay = ScenarioOverlay::load_from_env();
  EXPECT_FALSE(overlay.get("camera_node", "qos_depth").has_value());
}

TEST(ScenarioOverlay, ReadableFileLoadsItsKeys)
{
  const std::string path = testing::TempDir() + "sentry_overlay_ok.env";
  {
    std::ofstream out(path);
    out << "# comment\ncamera_node.qos_depth=7\n";
  }
  setenv("SENTRY_SCENARIO", path.c_str(), 1);
  const auto overlay = ScenarioOverlay::load_from_env();
  ASSERT_TRUE(overlay.get("camera_node", "qos_depth").has_value());
  EXPECT_EQ(*overlay.get("camera_node", "qos_depth"), "7");
  std::remove(path.c_str());
  unsetenv("SENTRY_SCENARIO");
}

TEST(ScenarioOverlay, SetButMissingFileRefusesToStart)
{
  const std::string path = "relative/out/run/scenario_overlay.env";
  setenv("SENTRY_SCENARIO", path.c_str(), 1);
  try {
    ScenarioOverlay::load_from_env();
    ADD_FAILURE() << "load_from_env returned an overlay for an unreadable SENTRY_SCENARIO";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find(path), std::string::npos) << e.what();
  }
  unsetenv("SENTRY_SCENARIO");
}

TEST(ScenarioOverlay, SetToADirectoryRefusesToStart)
{
  // An ifstream opens a directory on Linux and only fails on the read.
  const std::string path = testing::TempDir();
  setenv("SENTRY_SCENARIO", path.c_str(), 1);
  try {
    ScenarioOverlay::load_from_env();
    ADD_FAILURE() << "load_from_env returned an overlay for a directory SENTRY_SCENARIO";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find(path), std::string::npos) << e.what();
  }
  unsetenv("SENTRY_SCENARIO");
}
