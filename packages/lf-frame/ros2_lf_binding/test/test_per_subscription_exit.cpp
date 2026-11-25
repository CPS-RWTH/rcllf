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

/// @file test_per_subscription_exit.cpp
/// @brief Runs per_subscription_exit_probe as a child process and
/// requires exit code 0. Before the fix the child died with SIGSEGV in
/// ~PerSubscription at static destruction (brake_fusion T2Approx).

#include <gtest/gtest.h>

#include <sys/wait.h>
#include <unistd.h>

#include <cstring>

TEST(PerSubscriptionExit, EntryOfASubscriptionFromAnUnloadedLibraryIsSafeAtExit)
{
  const pid_t pid = fork();
  ASSERT_GE(pid, 0) << "fork failed";
  if (pid == 0) {
    execl(PROBE_PATH, PROBE_PATH, PROBE_LIB_PATH, static_cast<char *>(nullptr));
    _exit(127);
  }
  int status = 0;
  ASSERT_EQ(waitpid(pid, &status, 0), pid);
  ASSERT_FALSE(WIFSIGNALED(status))
    << "the probe died with signal " << WTERMSIG(status) << " (" << strsignal(WTERMSIG(status))
    << ") at exit: the static PerSubscription released a control block of the unloaded library";
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0)
    << "probe exit code (2 setup failed, 3 dlclose did not unload in the setup check, "
    << "127 exec failed)";
}
