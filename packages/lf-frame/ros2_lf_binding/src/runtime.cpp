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

/// @file runtime.cpp
/// @brief rclcpp lifecycle: init, component loading, shutdown.
///
/// Reads SYSTEM_JSON env var to locate system.json, loads all component nodes
/// via LFComponentManager, then returns so lf_reactor_c_main() can start.
/// Also locates and loads exec_config.json (the resolved execution-policy
/// table, see policy_table.hpp) via the `exec_config` node parameter /
/// ROS2_LF_EXEC_CONFIG env var / --exec-config argv, in that order.

#include <dlfcn.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "action_channels.hpp"
#include "clock_publisher.hpp"
#include "component_manager.hpp"
#include "input_period.hpp"
#include "match_warmup.hpp"
#include "corr_runtime.hpp"
#include "external_input.hpp"
#include "output_deadline.hpp"
#include "plant_record.hpp"
#include "service_pairing.hpp"
#include "composition_interfaces/srv/load_node.hpp"
#include "handle_table.hpp"
#include "ingress.hpp"
#include "lifecycle_gate.hpp"
#include "node_registry_internal.h"  // NOLINT(build/include_subdir)
#include "policy_table.hpp"
#include "readback.hpp"
#include "rcl/context.h"
#include "rcl/error_handling.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#if RCLLF_HAVE_LTTNG
#include "rcllf_stop_tp.h"  // NOLINT(build/include_subdir)
#endif
#include "runtime_internal.hpp"
#include "take_mode.hpp"
#include "thread_role.hpp"
#include "time_source.hpp"
#include "user_callback_gate.hpp"

namespace ros2_lf_binding
{

// --- Signal-driven LF stop --------------------------------------
// rclcpp::init() installs rclcpp's SIGINT/SIGTERM handlers, which only shut
// rclcpp down. Nothing told the LF scheduler, so an lfc binary kept running
// after SIGTERM. rclcpp runs its on-shutdown callbacks on its signal-handling
// thread (not in async-signal context), so the callback below may request an
// LF stop. It must not do so before LF has started: ros2_lf_runtime_init()
// runs in a static constructor, before the LF environments exist. A signal in
// that window is recorded and the stop is issued from the first LF startup
// reaction instead (lf_mark_running()).
namespace
{
std::atomic<bool> g_lf_running{false};
std::atomic<bool> g_lf_stop_pending{false};
std::atomic<bool> g_lf_stop_issued{false};
std::atomic<bool> g_runtime_shutting_down{false};
// Set once the runtime shutdown has drained the samples still in flight
// at the stop; rclcpp's shutdown on a signal waits for it (the context stays
// valid until then).
std::atomic<bool> g_stop_settled{false};
int64_t stop_join_bound_ms();

/// A `rcllf_stop:mark` UST event (no-op without lttng-ust) for the
/// stop timeline of a run; `detail` is a count where one applies.
void stop_mark(const char * what, int64_t detail = 0)
{
#if RCLLF_HAVE_LTTNG
  tracepoint(rcllf_stop, mark, what, detail);
#else
  (void)what;
  (void)detail;
#endif
}

void request_lf_stop_once()
{
  if (g_lf_stop_issued.exchange(true)) {
    return;
  }
  stop_mark("stop_requested");
  // Every join pending below the stop tag ends within the join bound.
  corr_rt::note_stop_requested(stop_join_bound_ms());
  // Resolve the EXECUTABLE's lf_request_stop at runtime. The lfc binary
  // exports its threaded reactor-c; this library links its own
  // single-threaded copy, which a link-time reference would bind to and which
  // would skip the threaded scheduler's locking and wake-up.
  using StopFn = void (*)(void);
  auto stop = reinterpret_cast<StopFn>(dlsym(RTLD_DEFAULT, "lf_request_stop"));
  if (stop == nullptr) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ros2_lf_binding"),
      "rclcpp shut down but lf_request_stop is not resolvable; exiting without LF shutdown");
    std::_Exit(EXIT_FAILURE);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding"),
    "rclcpp shut down (signal); requesting LF stop");
  stop();
}

// A signal must end the process even when the LF stop cannot
// complete. A federate's stop needs the RTI's grant. When the RTI is gone (a
// group SIGTERM reached it too, or it crashed), reactor-c waits for that grant
// forever, or fails it with exit() on rclcpp's signal thread, whose atexit
// teardown then deadlocks joining that very thread. Either way the federate
// outlived its launch. ITIMER_REAL bounds the time from the signal to
// ros2_lf_runtime_shutdown(), and again from there to the process end;
// SIGALRM then ends the process from signal context. _exit skips the static
// destructors and atexit handlers, which must not run while LF workers still
// do. The handler writes nothing: stderr may be a full pipe nobody
// drains, and a blocking write would hang the bound itself. The bound and its
// exit code are logged when it is armed instead.
constexpr int64_t kDefaultStopTimeoutMs = 3000;
constexpr int kStopTimeoutExitCode = 124;  // docs/reference/ros2-lf-binding.md
// RCLLF_STOP_TIMEOUT_MS, read once at init: a signal handler cannot read it.
std::atomic<int64_t> g_stop_timeout_ms{kDefaultStopTimeoutMs};
// -1 until armed; then the bound in ms (0: disabled).
std::atomic<int64_t> g_stop_timeout_armed_ms{-1};
// The bound counts from the last progress, not from the signal: a run
// winding down under overload (a lagging tag with many dispatches) is not cut
// off while its dispatches still return. RCLLF_STOP_MAX_MS (default
// kStopMaxFactor times the bound) caps the time from the first arming, so a
// federate whose RTI is gone still ends.
constexpr int64_t kStopMaxFactor = 4;
constexpr int64_t kStopProgressMinGapNs = 250000000;  // one re-arm per 250 ms at most
std::atomic<int64_t> g_stop_max_ms{kDefaultStopTimeoutMs * kStopMaxFactor};
std::atomic<int64_t> g_stop_armed_at_ns{0};    // CLOCK_MONOTONIC at the first arming
std::atomic<int64_t> g_stop_rearmed_at_ns{0};  // CLOCK_MONOTONIC at the last (re-)arming

int64_t monotonic_now_ns()  // async-signal-safe
{
  struct timespec ts {};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
// Set just before ros2_lf_runtime_shutdown() calls rclcpp::shutdown() itself.
std::atomic<bool> g_own_rclcpp_shutdown{false};
static_assert(
  std::atomic<int64_t>::is_always_lock_free, "signal handler needs a lock-free bound");

void on_stop_timeout([[maybe_unused]] int signum)
{
  _exit(kStopTimeoutExitCode);
}

int64_t stop_timeout_ms_from_env()
{
  const char * v = std::getenv("RCLLF_STOP_TIMEOUT_MS");
  if (v == nullptr || *v == '\0') {
    return kDefaultStopTimeoutMs;
  }
  char * end = nullptr;
  const long long ms = std::strtoll(v, &end, 10);  // NOLINT(runtime/int)
  if (*end != '\0' || ms < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "RCLLF_STOP_TIMEOUT_MS='%s' is not a count of ms; using %lld", v,
      static_cast<long long>(kDefaultStopTimeoutMs));  // NOLINT(runtime/int)
    return kDefaultStopTimeoutMs;
  }
  return ms;
}

int64_t stop_max_ms_from_env(int64_t bound_ms)
{
  const char * v = std::getenv("RCLLF_STOP_MAX_MS");
  if (v == nullptr || *v == '\0') {
    return bound_ms * kStopMaxFactor;
  }
  char * end = nullptr;
  const long long ms = std::strtoll(v, &end, 10);  // NOLINT(runtime/int)
  if (*end != '\0' || ms < bound_ms) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "RCLLF_STOP_MAX_MS='%s' is not a count of ms at or above the bound; using %lld", v,
      static_cast<long long>(bound_ms * kStopMaxFactor));  // NOLINT(runtime/int)
    return bound_ms * kStopMaxFactor;
  }
  return ms;
}

// The bound as configured, or the default when the bound is off, so the
// stop still has a finite deadline for its joins and its drain.
int64_t stop_bound_or_default_ms()
{
  const int64_t ms = g_stop_timeout_ms.load();
  return ms > 0 ? ms : kDefaultStopTimeoutMs;
}

void set_stop_timer(int64_t ms)
{
  struct itimerval t {};
  t.it_value.tv_sec = static_cast<time_t>(ms / 1000);
  t.it_value.tv_usec = static_cast<suseconds_t>((ms % 1000) * 1000);
  setitimer(ITIMER_REAL, &t, nullptr);
  g_stop_rearmed_at_ns.store(monotonic_now_ns());
}

// Who ends a start still inside the startup span. The
// startup watchdog (below) and every stop-bound arming decide it through this
// one atomic, each with a single compare_exchange from kStartupArmed, so there
// is no window between a decision and its effect:
// - the watchdog's expiry commits to its exit with kStartupArmed ->
//   kStartupExpired; it exits only when that exchange succeeds;
// - a stop-bound arming above 0 (arm_stop_timeout_signal_safe, also from
//   signal context) hands the start over with kStartupArmed ->
//   kStartupHandedOver before it sets its timer; the watchdog then returns
//   and the stop bound governs the process end;
// - reaching the discovery wait (disarm_startup_bound) ends the span with
//   kStartupArmed -> kStartupHandedOver as well.
// If the watchdog won (kStartupExpired), its exit is committed: a later stop
// arming records its bound but sets no timer and logs nothing, and no re-arm
// touches the timer, so the expiry's 1000 ms write backstop stays in place and
// the process ends with the startup bound's message and code 124.
// RCLLF_STOP_TIMEOUT_MS=0 arms no stop bound, so it hands nothing over: a
// stuck start stays under the startup bound (only RCLLF_STARTUP_TIMEOUT_MS=0
// leaves a start unbounded). The state starts at kStartupArmed and is only
// read by a watchdog, which exists once the startup bound is armed.
constexpr int kStartupArmed = 0;
constexpr int kStartupExpired = 1;
constexpr int kStartupHandedOver = 2;
std::atomic<int> g_startup_state{kStartupArmed};
static_assert(
  std::atomic<int>::is_always_lock_free, "signal handler needs a lock-free startup state");

// Async-signal-safe (atomics, sigaction, setitimer): count_late_signal calls
// it too. True when this call armed the bound.
bool arm_stop_timeout_signal_safe()
{
  int64_t unarmed = -1;
  const int64_t ms = g_stop_timeout_ms.load();
  if (!g_stop_timeout_armed_ms.compare_exchange_strong(unarmed, ms) || ms == 0) {
    return false;  // armed already, or 0 disables the bound
  }
  int startup = kStartupArmed;
  if (!g_startup_state.compare_exchange_strong(startup, kStartupHandedOver) &&
    startup == kStartupExpired)
  {
    return false;  // the startup bound's exit is committed
  }
  struct sigaction sa {};
  sa.sa_handler = &on_stop_timeout;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGALRM, &sa, nullptr);
  g_stop_armed_at_ns.store(monotonic_now_ns());
  set_stop_timer(ms);
  return true;
}

void log_stop_timeout_armed()
{
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding"),
    "rcllf: process end bounded to %lld ms (RCLLF_STOP_TIMEOUT_MS) from the last progress, "
    "at most %lld ms in all (RCLLF_STOP_MAX_MS); past it the process exits with code %d "
    "without cleanup",
    static_cast<long long>(g_stop_timeout_armed_ms.load()),  // NOLINT(runtime/int)
    static_cast<long long>(g_stop_max_ms.load()),  // NOLINT(runtime/int)
    kStopTimeoutExitCode);
}

void disarm_startup_bound();

// Once armed, the stop bound governs the process end, so a start the
// signal or the refusal stops is not cut off by the startup bound as well.
void arm_stop_timeout()
{
  if (arm_stop_timeout_signal_safe()) {
    log_stop_timeout_armed();
    disarm_startup_bound();
  }
}

// A start can block for good inside Fast DDS. The graph update a
// new node publishes (rclcpp::Node::Node -> ParameterService -> __rmw_publish)
// waited in SharedMemTransport::send on a shared-memory port lock for the
// whole 60 s of a test loop (1 run in 11200, 16 parallel starts on one
// /dev/shm). The stop bound above is armed only by a signal or a refusal, so
// nothing ended that start. The startup bound runs from before rclcpp::init
// up to the first wait of ros2_lf_runtime_wait_ready(), whose waits carry
// their own timeout (ROS2_LF_DISCOVERY_TIMEOUT_MS): it covers rclcpp::init,
// every node construction, the component loading and the correspondence
// startup stage. A refusal or a shutdown in that span hands over to the stop
// bound (g_startup_state above decides it). On expiry the process ends with
// the stop bound's code without cleanup. An armed stop bound
// (RCLLF_STOP_TIMEOUT_MS above 0) replaces the startup bound wherever it is
// armed: a SIGINT 200 ms before the expiry starts a stop that its own bound
// governs. Not through runtime_fatal_exit(): the
// main thread is inside rclcpp with a node half built, so a release from the
// watchdog would race it, and its own graph-update publishes would wait on the
// same lock. A watchdog
// thread, not the SIGALRM timer, so the expiry can still say why the process
// ended; it writes with write(2), not the ROS logger, whose /rosout publish
// could wait on that lock too.
//
// 30000 ms: a stuck start, not a slow one. 200 healthy starts of
// pubsub_example, 4 in parallel, took at most 38.5 ms from the end of
// rclcpp::init to the discovery wait (dev journal
// entry on the startup bound). The default leaves a factor of several
// hundred for systems with many nodes and for loaded CI hosts; it stays below the 60 s that
// the loop, CI test timeouts and an operator's patience give a start, so
// the bound, not the harness, reports the hang. RCLLF_STARTUP_TIMEOUT_MS
// overrides it; 0 turns it off.
constexpr int64_t kDefaultStartupTimeoutMs = 30000;
constexpr int64_t kMaxStartupTimeoutMs = 24LL * 60 * 60 * 1000;  // larger: invalid
// The expiry's write and flush can block on a full stderr pipe nobody drains;
// SIGALRM ends the process this long after the expiry regardless. A write of
// one line to a pipe with room takes microseconds, so 1000 ms is a block.
constexpr int64_t kStartupExitWriteBoundMs = 1000;

struct StartupWatch
{
  std::mutex m;
  std::condition_variable cv;
  bool done = false;
};
// Never destroyed: the detached watchdog may still wait on it at exit().
StartupWatch & startup_watch()
{
  static StartupWatch * w = new StartupWatch();
  return *w;
}
std::atomic<bool> g_startup_bound_armed{false};

int64_t startup_timeout_ms_from_env()
{
  const char * v = std::getenv("RCLLF_STARTUP_TIMEOUT_MS");
  if (v == nullptr || *v == '\0') {
    return kDefaultStartupTimeoutMs;
  }
  char * end = nullptr;
  errno = 0;
  const long long ms = std::strtoll(v, &end, 10);  // NOLINT(runtime/int)
  // Above a day the wait's nanosecond duration can overflow and expire at once.
  if (*end != '\0' || errno == ERANGE || ms < 0 || ms > kMaxStartupTimeoutMs) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "RCLLF_STARTUP_TIMEOUT_MS='%s' is not a count of ms; using %lld", v,
      static_cast<long long>(kDefaultStartupTimeoutMs));  // NOLINT(runtime/int)
    return kDefaultStartupTimeoutMs;
  }
  return ms;
}

[[noreturn]] void on_startup_timeout(int64_t ms)
{
  // This thread inherited the signal mask of the thread that armed it: let the
  // backstop's SIGALRM reach it while the write below blocks, and keep a
  // closed stderr pipe from ending the process with SIGPIPE instead of 124.
  sigset_t mask;
  sigemptyset(&mask);
  sigaddset(&mask, SIGALRM);
  pthread_sigmask(SIG_UNBLOCK, &mask, nullptr);
  sigemptyset(&mask);
  sigaddset(&mask, SIGPIPE);
  pthread_sigmask(SIG_BLOCK, &mask, nullptr);
  struct sigaction sa {};
  sa.sa_handler = &on_stop_timeout;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGALRM, &sa, nullptr);
  set_stop_timer(kStartupExitWriteBoundMs);
  char line[320];
  const int n = std::snprintf(
    line, sizeof(line),
    "[FATAL] [ros2_lf_binding]: rcllf: startup did not reach the discovery wait within "
    "%lld ms (RCLLF_STARTUP_TIMEOUT_MS); a startup step is stuck (a node construction "
    "blocked in the middleware?); exiting with code %d without cleanup\n",
    static_cast<long long>(ms), kStopTimeoutExitCode);  // NOLINT(runtime/int)
  if (n > 0) {
    const size_t len = std::min(static_cast<size_t>(n), sizeof(line) - 1);
    [[maybe_unused]] const ssize_t written = write(STDERR_FILENO, line, len);
  }
  std::fflush(stdout);
  std::fflush(stderr);
  _exit(kStopTimeoutExitCode);
}

// Returns the bound in ms, 0 when it is off.
int64_t arm_startup_bound()
{
  const int64_t ms = startup_timeout_ms_from_env();
  if (ms == 0 || g_startup_bound_armed.exchange(true)) {
    return 0;
  }
  std::thread(
    [ms]() {
      pthread_setname_np(pthread_self(), "rcllf.startup");
      StartupWatch & w = startup_watch();
      std::unique_lock<std::mutex> lock(w.m);
      if (w.cv.wait_for(lock, std::chrono::milliseconds(ms), [&w]() {return w.done;})) {
        return;
      }
      lock.unlock();
      // The exit is decided by the exchange alone (round 3): a stop bound
      // armed up to that instant, also from signal context, has handed the
      // start over and wins; one armed after it finds the exit committed.
      if (rcllf_test_startup_expiry_hook != nullptr) {
        rcllf_test_startup_expiry_hook(0);
      }
      int armed = kStartupArmed;
      if (!g_startup_state.compare_exchange_strong(armed, kStartupExpired)) {
        return;
      }
      if (rcllf_test_startup_expiry_hook != nullptr) {
        rcllf_test_startup_expiry_hook(1);
      }
      on_startup_timeout(ms);
    }).detach();
  return ms;
}

void disarm_startup_bound()
{
  int armed = kStartupArmed;
  g_startup_state.compare_exchange_strong(armed, kStartupHandedOver);
  StartupWatch & w = startup_watch();
  {
    std::lock_guard<std::mutex> lock(w.m);
    w.done = true;
  }
  w.cv.notify_all();
}

// A fresh bound from now, within RCLLF_STOP_MAX_MS of the first arming. Only
// after a signal: once LF has stopped, for the teardown, and on progress.
void rearm_stop_timeout()
{
  const int64_t ms = g_stop_timeout_armed_ms.load();
  if (ms <= 0 || g_startup_state.load() == kStartupExpired) {
    return;  // no bound, or the startup bound's committed exit owns the timer
  }
  const int64_t spent_ms = (monotonic_now_ns() - g_stop_armed_at_ns.load()) / 1000000;
  const int64_t left_ms = g_stop_max_ms.load() - spent_ms;
  if (left_ms <= 0) {
    return;  // at the cap: the running timer ends the process
  }
  set_stop_timer(std::min(ms, left_ms));
}
}  // namespace

bool stop_bound_armed() {return g_stop_timeout_armed_ms.load() > 0;}

void stop_progress()
{
  if (g_stop_timeout_armed_ms.load() <= 0) {
    return;
  }
  if (monotonic_now_ns() - g_stop_rearmed_at_ns.load() < kStopProgressMinGapNs) {
    return;
  }
  rearm_stop_timeout();
}

// The one exit of a startup refusal; defined after g_manager.
[[noreturn]] void runtime_fatal_exit(const char * why);

namespace
{
// The stop deadline of the joins (half the bound) and the drain for the
// samples still in flight after LF returned (a quarter), both re-armed as
// progress; the teardown then gets the bound again.
int64_t stop_drain_bound_ms() {return std::max<int64_t>(1, stop_bound_or_default_ms() / 4);}
int64_t stop_join_bound_ms() {return std::max<int64_t>(1, stop_bound_or_default_ms() / 2);}

// ros2 launch answers a SIGTERM by cancelling itself without
// stopping its processes, so a federate launched by it would run on. The
// generated launch files set RCLLF_EXIT_WITH_PARENT to the launch process's
// pid: the kernel then sends SIGTERM when the launch process dies, and the
// federate stops as on any SIGTERM. The pid comes from launch, not from
// getppid() here: had launch died before this point, getppid() would already
// name init or a subreaper, and no death signal would ever come.
void exit_with_parent_if_requested()
{
  const char * v = std::getenv("RCLLF_EXIT_WITH_PARENT");
  if (v == nullptr || *v == '\0') {
    return;
  }
  char * end = nullptr;
  const long long launch_pid = std::strtoll(v, &end, 10);  // NOLINT(runtime/int)
  if (*end != '\0' || launch_pid <= 1) {
    // Also the old "1", which compared getppid() with itself and so missed
    // a parent that died before this point.
    RCLCPP_FATAL(
      rclcpp::get_logger("ros2_lf_binding"),
      "RCLLF_EXIT_WITH_PARENT='%s' is not the pid of the launching process; set it to "
      "str(os.getpid()) in the launch file", v);
    runtime_fatal_exit("RCLLF_EXIT_WITH_PARENT is invalid");
  }
  const auto parent_gone = [launch_pid]() {
      return static_cast<long long>(getppid()) != launch_pid;  // NOLINT(runtime/int)
    };
  if (!parent_gone() && prctl(PR_SET_PDEATHSIG, SIGTERM) != 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "RCLLF_EXIT_WITH_PARENT: prctl(PR_SET_PDEATHSIG) failed; the process may outlive its parent");
    return;
  }
  // Checked again after prctl: the parent may die in between.
  if (parent_gone()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ros2_lf_binding"),
      "RCLLF_EXIT_WITH_PARENT: the launching process %lld is gone (parent is now %lld); "
      "exiting before LF starts", launch_pid,
      static_cast<long long>(getppid()));  // NOLINT(runtime/int)
    runtime_fatal_exit("the launching process is gone");
  }
}

void on_rclcpp_shutdown()
{
  if (g_runtime_shutting_down.load()) {
    // LF is already stopping. A signal during the teardown, before the late
    // signal counter replaced rclcpp's handlers, still bounds the process
    // end; the binding's own rclcpp::shutdown() does not.
    if (!g_own_rclcpp_shutdown.load()) {
      arm_stop_timeout();
    }
    return;
  }
  arm_stop_timeout();
  // Store-then-check here and in lf_mark_running(): at least one side sees
  // the other, and request_lf_stop_once() deduplicates.
  g_lf_stop_pending.store(true);
  if (g_lf_running.load()) {
    request_lf_stop_once();
  }
}
// Controlled shutdown: rclcpp runs
// pre-shutdown callbacks and then invalidates the context, and only after
// that its on-shutdown callbacks. So when rclcpp shuts down while LF runs (a
// signal), stop LF first and wait, bounded, until LF has run every tag below
// its stop tag and its shutdown tag and the runtime shutdown has drained the
// samples still in flight (g_stop_settled). The context is valid meanwhile:
// the ingress still takes, user callbacks still run (so a join below the stop
// tag can still match and deliver), and the goals an action server's
// reaction(shutdown) aborts reach their clients. Not on an LF worker or inside
// a user callback, which LF would wait for in turn. An action server's
// pre-shutdown callbacks still kept after the bound run here, off any
// reaction, as rclcpp would have run them.
void stop_lf_before_context_shutdown(std::chrono::milliseconds bound)
{
  // No LF shutdown tag can run them before rcl_shutdown on these paths, so
  // they run here while the context is still valid.
  if (g_stop_settled.load() || !g_lf_running.load() || is_lf_worker_thread() ||
    t_user_callback_depth > 0)
  {
    action_pre_shutdown_run_left("on rclcpp's shutdown");
    return;
  }
  request_lf_stop_once();
  const auto deadline = std::chrono::steady_clock::now() + bound;
  while (!g_stop_settled.load()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_WARN(
        rclcpp::get_logger("ros2_lf_binding"),
        "rclcpp shutdown: LF did not settle its stop within %lld ms",
        static_cast<long long>(bound.count()));  // NOLINT(runtime/int)
      action_pre_shutdown_run_left("on rclcpp's shutdown");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// SIGINT/SIGTERM after ros2_lf_runtime_shutdown() began. Launch
// forwards the group SIGINT to the child about 1 ms after the child got it,
// so a second signal during the teardown is the normal case. It counts, and
// it bounds the process end if no earlier signal did: a federate that
// reached its stop time normally and whose teardown then blocks (RTI gone)
// still ends on the operator's SIGTERM.
static_assert(std::atomic<int>::is_always_lock_free, "signal handler needs a lock-free counter");
std::atomic<int> g_late_signals{0};

void count_late_signal([[maybe_unused]] int signum)
{
  g_late_signals.fetch_add(1, std::memory_order_relaxed);
  arm_stop_timeout_signal_safe();
}

void install_late_signal_counter()
{
  struct sigaction sa {};
  sa.sa_handler = &count_late_signal;
  sa.sa_flags = SA_RESTART;  // no EINTR in the teardown's syscalls
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
}
}  // namespace

bool lf_is_running() {return g_lf_running.load();}

void lf_mark_running()
{
  // Report the effective settings before the first bind, which
  // refuses a mismatching program and would leave no read-back behind. Every
  // binding worker passes here and waits until the file is written, so a
  // sibling's refusal (_Exit) cannot cut the write short.
  ros2_lf_binding::write_readback_once();
  if (g_lf_running.exchange(true)) {
    return;
  }
  // lf_reactor_c_main() replaced rclcpp's SIGINT handler with exit(), which
  // ends the process from signal context while LF workers may still run.
  // Re-install rclcpp's handlers with SIG_IGN as the saved previous SIGINT
  // and SIGTERM handlers. rclcpp chains to neither SIG_IGN nor SIG_DFL, so
  // SIGINT takes the same LF-stop path as SIGTERM, and when shutdown
  // uninstalls rclcpp's handlers no late signal finds SIG_DFL.
  rclcpp::uninstall_signal_handlers();
  std::signal(SIGINT, SIG_IGN);
  std::signal(SIGTERM, SIG_IGN);
  rclcpp::install_signal_handlers();
  // Ingress backstop (R8/R16). reactor-c registered its
  // `termination` handler in lf_reactor_c_main(), before this first startup
  // reaction, so this handler runs BEFORE it (atexit is LIFO): the ingress
  // thread is gone before environments are freed.
  std::atexit(ros2_lf_ingress_stop_atexit);
  if (g_lf_stop_pending.load()) {
    request_lf_stop_once();
  }
}

// Global component manager — kept alive for the process lifetime.
// Not static so node_binding.cpp can extern-declare it.
std::shared_ptr<LFComponentManager> g_manager;
// Registration executor: a never-spun sink that load_component() requires as a
// parameter. No node is ever added to it (R3): add_node() claims a
// node's services for that executor's wait set, and the ingress wait set must
// own the infrastructure services instead. LFComponentManager overrides
// add_node_to_executor() as a no-op for the same reason.
static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> g_registration_executor;
// Cached system_json path set during ros2_lf_runtime_init for use in
// ros2_lf_runtime_load_nodes(). Heap-allocated so cpplint is happy.
static const char * g_system_json_cstr = nullptr;

// Release every ROS entity and node the binding holds, shut
// rclcpp down and finalize the rmw context, which destroys the DDS participant
// and joins its threads. Shared by the runtime shutdown and a startup refusal.
// Returns the rcl_context_fini result: not RCL_RET_OK means a node is still
// alive, and the caller must end the process without static destructors.
static rcl_ret_t release_entities_and_finalize_context()
{
  corr_rt::release_subscriptions();
  corr_rt::complete_end_release();
  clock_publisher_shutdown();
  // Release every ROS entity and node while the rclcpp context and the DDS
  // participant still exist. The handle table and node registry are globals of
  // this library; left alone they are destroyed at library unload, after the
  // participant is gone, and the service destructors crash (SIGSEGV in
  // DomainParticipantImpl::find_type).
  g_handle_table.clear();
  ros2_lf_clear_node_registry();
  g_registration_executor.reset();
  g_manager.reset();
  g_own_rclcpp_shutdown.store(true);
  rclcpp::shutdown();
  // Finalize the rmw context here, on this thread. With the last
  // node gone, rmw has already joined its listener thread and destroyed the
  // DDS participant. Had a node survived (the correspondence edges once kept
  // theirs), Fast DDS's static DomainParticipantFactory would destroy the
  // participant inside exit() while its receive and listener threads still
  // run: heap corruption, SIGSEGV in __rmw_wait, "pure virtual method called".
  return rcl_context_fini(
    rclcpp::contexts::get_global_default_context()->get_rcl_context().get());
}

// A startup refusal once ended with rclcpp::shutdown() and
// std::exit(), so the manager node and its DDS participant lived into the
// static destructors, while the receive threads still paired endpoints of the
// other processes on the domain: a SIGSEGV in a generated
// executable. Every refusal after rclcpp::init ends here instead: the
// lifecycle pins go (the pre-shutdown callback that normally releases them may
// not be registered yet), then the shared release and context fini, and the
// process ends without static destructors, finalized or not.
void runtime_fatal_exit(const char * why)
{
  // As in the shutdown teardown: this is the binding's own shutdown, so
  // on_rclcpp_shutdown() arms nothing on its own, and a signal from here on
  // only counts instead of running rclcpp's shutdown concurrently with the
  // release. The release itself is bounded like the teardown's: a
  // node destructor once blocked in Fast DDS SharedMemTransport::send for the
  // whole 60 s of a test loop (1 run in 5600); past the bound the process
  // ends with 124, still without static destructors. That bound replaces the
  // startup bound.
  disarm_startup_bound();
  g_runtime_shutting_down.store(true);
  g_own_rclcpp_shutdown.store(true);
  rclcpp::uninstall_signal_handlers();
  install_late_signal_counter();
  arm_stop_timeout();
  lifecycle_gate_release_pins();
  const rcl_ret_t fini = release_entities_and_finalize_context();
  if (fini == RCL_RET_OK) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf: rmw context finalized before the startup refusal exit (%s)", why);
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf: rmw context not finalized before the startup refusal exit (%s, rcl_ret_t %d); "
      "exiting without static destructors", why, static_cast<int>(fini));
    rcl_reset_error();
  }
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(EXIT_FAILURE);
}

/// Read one field of a `component_instances[i]` JSON object as a string,
/// defaulting to "" when absent or the wrong type.  Parsing happens entirely
/// within `obj`'s own bounds (a real nlohmann::json object), so a long entry
/// or a field far from the start of the object can never scroll out of range
/// the way the old proximity-window scanner could.
static std::string entry_field(const nlohmann::json & obj, const std::string & key)
{
  if (!obj.contains(key)) {
    return "";
  }
  const nlohmann::json & v = obj.at(key);
  if (!v.is_string()) {
    return "";
  }
  return v.get<std::string>();
}

std::string missing_file_error(const std::string & what, const std::string & path)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  if (path.empty() || fs::is_regular_file(path, ec)) {
    return "";
  }
  const fs::path cwd = fs::current_path(ec);
  const fs::path resolved = fs::absolute(path, ec);
  return what + " '" + path + "' not found (resolved to '" + resolved.string() +
         "', cwd '" + cwd.string() + "')";
}

ComponentListParseResult parse_component_list(const std::string & json_path)
{
  ComponentListParseResult result;

  // A missing file is not "nothing to load": every reaction would run
  // unbound. Fail with the resolved path and the cwd.
  const std::string missing = missing_file_error("system.json", json_path);
  if (!missing.empty()) {
    result.ok = false;
    result.error = missing;
    return result;
  }
  std::ifstream ifs(json_path);
  if (!ifs.is_open()) {
    result.ok = false;
    result.error = json_path + ": cannot be opened";
    return result;
  }

  nlohmann::json doc;
  try {
    doc = nlohmann::json::parse(ifs);
  } catch (const nlohmann::json::exception & e) {
    result.ok = false;
    result.error = json_path + ": not valid JSON (" + e.what() + ")";
    return result;
  }

  if (!doc.is_object() || !doc.contains("component_instances")) {
    // No component_instances key at all: nothing to load (also covers
    // system.json files with zero component nodes).
    return result;
  }

  const nlohmann::json & entries = doc.at("component_instances");
  if (!entries.is_array()) {
    result.ok = false;
    result.error = json_path + ": 'component_instances' must be an array";
    return result;
  }

  // Entry-scoped extraction: every field is read from within its own JSON
  // object (`item`), never from a byte-offset window over the raw file.
  struct RawEntry
  {
    std::string package;
    std::string plugin_class;
    std::string binding_key;  // "" means the field was absent in this entry.
    std::string node_name;
    std::string node_namespace;
    std::vector<std::string> remap_rules;
    std::vector<std::string> param_args;
    bool has_binding_key = false;
  };
  std::vector<RawEntry> raw;
  raw.reserve(entries.size());
  size_t index = 0;
  for (const auto & item : entries) {
    const std::string where = json_path + ": component_instances[" + std::to_string(index) + "]";
    index++;
    if (!item.is_object()) {
      result.ok = false;
      result.error = where + ": must be an object";
      return result;
    }
    RawEntry e;
    e.package = entry_field(item, "package");
    e.plugin_class = entry_field(item, "plugin_class");
    e.has_binding_key = item.contains("binding_key") && item.at("binding_key").is_string();
    e.binding_key = entry_field(item, "binding_key");
    e.node_name = entry_field(item, "node_name");
    e.node_namespace = entry_field(item, "namespace");
    // Optional string-array fields: remap_rules, param_args.
    for (const auto & [key, out] : {
        std::pair<const char *, std::vector<std::string> *>{"remap_rules", &e.remap_rules},
        std::pair<const char *, std::vector<std::string> *>{"param_args", &e.param_args}})
    {
      if (!item.contains(key)) {
        continue;
      }
      const nlohmann::json & values = item.at(key);
      if (!values.is_array() ||
        !std::all_of(values.begin(), values.end(), [](const nlohmann::json & r) {
          return r.is_string();
        }))
      {
        result.ok = false;
        result.error = where + ": '" + key + "' must be an array of strings";
        return result;
      }
      for (const auto & r : values) {
        out->push_back(r.get<std::string>());
      }
    }
    if (e.package.empty() || e.plugin_class.empty()) {
      result.ok = false;
      result.error = where + ": missing string 'package' or 'plugin_class'";
      return result;
    }
    raw.push_back(std::move(e));
  }

  // Mixed-schema fail-closed rule (design §4): entries without binding_key are
  // legal only if NO entry in the file has one. Any entry having binding_key
  // while another lacks it means the file is corrupt (a hand- or partially-
  // regenerated mix of old and new generator output), which is a loud init
  // failure, not a best-effort partial load.
  const bool any_has_key =
    std::any_of(raw.begin(), raw.end(), [](const RawEntry & e) {return e.has_binding_key;});
  const bool any_missing_key =
    std::any_of(raw.begin(), raw.end(), [](const RawEntry & e) {return !e.has_binding_key;});
  if (any_has_key && any_missing_key) {
    result.ok = false;
    result.error = json_path +
      ": component_instances mixes entries with and without 'binding_key' — "
      "this file is corrupt (partly old-format, partly new-format); "
      "re-run the generator to regenerate it consistently";
    return result;
  }

  if (!any_has_key) {
    // Legacy (pre-per-instance) format: dedupe on {package, plugin_class},
    // binding_key == plugin_class, exactly as the original class-keyed
    // registry did.
    std::set<std::pair<std::string, std::string>> seen;
    for (const auto & e : raw) {
      const auto key = std::make_pair(e.package, e.plugin_class);
      if (seen.find(key) != seen.end()) {
        continue;
      }
      seen.insert(key);
      ComponentInstance inst;
      inst.package = e.package;
      inst.plugin_class = e.plugin_class;
      inst.binding_key = e.plugin_class;
      inst.node_name = e.node_name;
      inst.node_namespace = e.node_namespace;
      inst.remap_rules = e.remap_rules;
      inst.param_args = e.param_args;
      result.instances.push_back(std::move(inst));
    }
    return result;
  }

  // New per-instance format: no deduping — one entry per instance.
  for (const auto & e : raw) {
    ComponentInstance inst;
    inst.package = e.package;
    inst.plugin_class = e.plugin_class;
    inst.binding_key = e.binding_key;
    inst.node_name = e.node_name;
    inst.node_namespace = e.node_namespace;
    inst.remap_rules = e.remap_rules;
    inst.param_args = e.param_args;
    result.instances.push_back(std::move(inst));
  }
  return result;
}

/// Parse system.json and return unique class_names where is_component == false.
static std::vector<std::string> parse_non_component_list(const std::string & json_path)
{
  std::ifstream ifs(json_path);
  if (!ifs.is_open()) {
    return {};
  }
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  std::set<std::string> seen;
  std::vector<std::string> result;

  // Iterate every "class_name" key forward; for each, search forward within
  // 2000 chars for "is_component" and check its value. Non-component nodes
  // are out of scope for the per-instance-RIDs runtime slice (per-instance
  // identity realization for them is generator-side factory/NodeOptions work,
  // design §4); this proximity-window scanner is unchanged and tolerates any
  // key ordering.
  auto find_value = [&](size_t start, const std::string & key) -> std::string {
      std::string needle = "\"" + key + "\"";
      size_t pos = content.find(needle, start);
      if (pos == std::string::npos || pos > start + 2000) {
        return "";
      }
    // Skip past the key and colon, then read the quoted value.
      pos = content.find('"', pos + needle.size());
      if (pos == std::string::npos) {
        return "";
      }
      pos++;
      size_t end = content.find('"', pos);
      if (end == std::string::npos) {
        return "";
      }
      return content.substr(pos, end - pos);
    };

  size_t pos = 0;
  while (true) {
    pos = content.find("\"class_name\"", pos);
    if (pos == std::string::npos) {
      break;
    }
    // Extract the class_name value
    size_t q1 = content.find('"', pos + sizeof("\"class_name\"") - 1);
    if (q1 == std::string::npos) {
      pos++;
      continue;
    }
    q1++;
    size_t q2 = content.find('"', q1);
    if (q2 == std::string::npos) {
      pos++;
      continue;
    }
    std::string cls = content.substr(q1, q2 - q1);

    // Look for is_component in a 2000-char window around this class_name.
    // Search both forward and backward to tolerate either key order.
    size_t window_start = (pos > 1000) ? pos - 1000 : 0;
    size_t ic_pos = content.find("\"is_component\"", window_start);
    bool found_false = false;
    while (ic_pos != std::string::npos && ic_pos < pos + 2000) {
      size_t colon = content.find(':', ic_pos);
      if (colon == std::string::npos) {
        break;
      }
      size_t val_start = content.find_first_not_of(" \t\r\n", colon + 1);
      if (val_start == std::string::npos) {
        break;
      }
      if (content.substr(val_start, 5) == "false") {
        // Confirm this is_component is within the same node object as class_name
        // by checking it's within 1000 chars of the class_name occurrence
        if (ic_pos >= window_start && ic_pos < pos + 1000) {
          found_false = true;
        }
      }
      break;
    }

    if (found_false && !cls.empty() && seen.find(cls) == seen.end()) {
      seen.insert(cls);
      result.push_back(cls);
    }
    pos = q2 + 1;
  }

  // Suppress unused-variable warning: find_value is unused in this function
  // (class_name and is_component are both extracted inline above); kept only
  // for historical symmetry with the old parse_component_list scanner, which
  // no longer exists (parse_component_list is now a real per-entry JSON parse
  // in runtime_internal.hpp / this file, see parse_component_list() above).
  (void)find_value;
  return result;
}

static ros2_lf_binding::ClockPublishMode parse_clock_publish_mode()
{
  const char * val = std::getenv("ROS2_LF_CLOCK_PUBLISH");
  if (!val) {
    return ros2_lf_binding::ClockPublishMode::OnTimeChange;
  }
  std::string s(val);
  if (s == "off") {
    return ros2_lf_binding::ClockPublishMode::Off;
  }
  if (s == "every_reaction") {
    return ros2_lf_binding::ClockPublishMode::OnEveryReaction;
  }
  if (s != "on_change") {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "Unknown ROS2_LF_CLOCK_PUBLISH value '%s'; using 'on_change'", val);
  }
  return ros2_lf_binding::ClockPublishMode::OnTimeChange;
}

/// Locate exec_config.json.  Lookup order is the frozen contract (design
/// §2.4 item 2), the same triple SYSTEM_JSON uses:
///   1. the `exec_config` node parameter (set by the generated launch file,
///      reaching us through automatically_declare_parameters_from_overrides on
///      the component-manager node),
///   2. the ROS2_LF_EXEC_CONFIG env var,
///   3. the --exec-config argv option.
/// Returns "" when none is set; that is not an error (absent -> defaults).
static std::string find_exec_config_path(int argc, char ** argv)
{
  if (g_manager) {
    std::string from_param;
    if (g_manager->get_parameter("exec_config", from_param) && !from_param.empty()) {
      return from_param;
    }
  }
  if (const char * env = std::getenv("ROS2_LF_EXEC_CONFIG")) {
    if (env[0] != '\0') {
      return std::string(env);
    }
  }
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--exec-config") {
      return std::string(argv[i + 1]);
    }
  }
  return std::string();
}

/// Collect every subscription across all loaded nodes (component nodes from
/// the manager and non-component nodes from the registry).  Subscription
/// readiness (each sub has at least one matched publisher) is the condition
/// that gates the data chain, so the readiness wait keys on subscriptions.
static std::vector<rclcpp::SubscriptionBase::SharedPtr> collect_subscriptions()
{
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  std::set<const void *> seen;

  auto scan = [&](const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base) {
      if (!node_base) {
        return;
      }
      node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
          auto s = group->find_subscription_ptrs_if(
            [&seen](const rclcpp::SubscriptionBase::SharedPtr & sp) {
              return seen.find(sp.get()) == seen.end();
          });
          while (s) {
            seen.insert(s.get());
            // A component's TimeSource /clock subscription is fed by the
            // binding, not by DDS; it needs no publisher.
            if (!lf_is_time_source_subscription(s.get())) {
              subs.push_back(s);
            }
            s = group->find_subscription_ptrs_if(
              [&seen](const rclcpp::SubscriptionBase::SharedPtr & sp) {
                return seen.find(sp.get()) == seen.end();
            });
          }
      });
    };

  if (g_manager) {
    for (auto & wrapper : g_manager->get_all_components()) {
      scan(wrapper.get_node_base_interface());
    }
  }
  ros2_lf_for_each_registered_node(
    [&](const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base) {
      scan(node_base);
    });
  return subs;
}

/// Timeout (ms) for discovery; honor ROS2_LF_DISCOVERY_TIMEOUT_MS, default 5000.
/// 5000: a margin over DDS discovery on a loaded host, not a measured bound;
/// a start that does not match within it is refused, so too small a
/// value fails loudly and never starts early. A value <= 0 skips the wait (an
/// explicit opt-out: such a start is outside the determinism claim).
static int discovery_timeout_ms()
{
  int timeout_ms = 5000;
  if (const char * t = std::getenv("ROS2_LF_DISCOVERY_TIMEOUT_MS")) {
    try {
      timeout_ms = std::stoi(t);
    } catch (...) {
    }
  }
  return timeout_ms;
}

/// Wait until DDS matched every local endpoint of a topic that
/// crosses federates (system.json `cross_federate_topics`): a publisher to as
/// many readers as the system has, a reader to as many writers. A best-effort
/// sample published before the writer saw the reader is lost. A
/// checked barrier. An endpoint still unmatched at the bound refuses the start
/// (exit 3, as the peer check does), since its loss could not be counted. The
/// bound is the discovery budget (discovery_timeout_ms), the one budget for
/// "DDS discovery has completed": matching follows the discovery of the same
/// endpoints, and its value decides only how long a slow or broken discovery
/// is waited for before the refusal, never the output of a run that starts.
/// ROS2_LF_MATCH_WARMUP_MS overrides it. No-op in the one program.
static void run_match_warmup(const char * stage)
{
  if (g_system_json_cstr == nullptr) {
    return;
  }
  const auto topics = match_warmup::load_cross_topics(g_system_json_cstr);
  if (topics.empty()) {
    return;
  }
  const int64_t timeout_ms = match_warmup::timeout_ms_from_env(discovery_timeout_ms());
  const auto subs = collect_subscriptions();
  std::vector<match_warmup::Target> targets;
  for (const auto & t : topics) {
    if (t.subscribers > 0 && !corr_rt::publisher_match_counts(t.topic).empty()) {
      targets.push_back(
        {t.topic, true, t.subscribers, [topic = t.topic]() {
            const auto counts = corr_rt::publisher_match_counts(topic);
            return counts.empty() ? size_t{0} : *std::min_element(counts.begin(), counts.end());
          }});
    }
    if (t.publishers == 0) {
      continue;
    }
    for (const auto & s : subs) {
      if (t.topic == s->get_topic_name()) {
        targets.push_back({t.topic, false, t.publishers, [s]() {return s->get_publisher_count();}});
      }
    }
  }
  if (targets.empty()) {
    return;
  }
  match_warmup::enforce_ready(
    match_warmup::wait(targets, std::chrono::milliseconds(timeout_ms)), stage, timeout_ms);
}

/// Test only: ROS2_LF_TEST_START_DELAY="<name>:<ms>" delays the
/// start of a federate whose executable name contains <name> by <ms> after the
/// peer check, so a fast-mode test can force startup skew between federates.
/// The name is the real executable path (glibc program_invocation_name): the
/// generated startup passes a fixed argv[0].
static void test_named_delay(const char * env, const char * what)
{
  const char * v = std::getenv(env);
  const char * colon = v != nullptr ? std::strchr(v, ':') : nullptr;
  const std::string exe = program_invocation_name;
  if (colon == nullptr || exe.find(std::string(v, colon)) == std::string::npos) {
    return;
  }
  const long ms = std::strtol(colon + 1, nullptr, 10);  // NOLINT(runtime/int)
  if (ms > 0 && ms <= 10000) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "%s: delaying the %s of '%s' by %ld ms (test only)", env, what, exe.c_str(), ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }
}

static void test_start_delay()
{
  test_named_delay("ROS2_LF_TEST_START_DELAY", "start");
}

/// Set by a federate whose generated startup runs the peer check
/// once the RTI start time is known, not in ros2_lf_runtime_wait_ready().
static bool g_peer_check_deferred = false;

}  // namespace ros2_lf_binding

extern "C" {

void ros2_lf_runtime_init(int argc, char ** argv)
{
  // Armed before rclcpp::init, so every step up to the discovery
  // wait is bounded; logged once the ROS logger is configured.
  const int64_t startup_bound_ms = ros2_lf_binding::arm_startup_bound();
  rclcpp::init(argc, argv);
  if (startup_bound_ms > 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf: startup bounded to %lld ms (RCLLF_STARTUP_TIMEOUT_MS) up to the discovery wait; "
      "past it the process exits with code %d without cleanup",
      static_cast<long long>(startup_bound_ms),  // NOLINT(runtime/int)
      ros2_lf_binding::kStopTimeoutExitCode);
  }
  ros2_lf_binding::g_stop_timeout_ms.store(ros2_lf_binding::stop_timeout_ms_from_env());
  ros2_lf_binding::g_stop_max_ms.store(
    ros2_lf_binding::stop_max_ms_from_env(ros2_lf_binding::stop_bound_or_default_ms()));
  ros2_lf_binding::exit_with_parent_if_requested();
  auto context = rclcpp::contexts::get_global_default_context();
  // Runs before rcl_shutdown invalidates the context, so no user
  // callback (and no loaned publish in it) runs against an invalid context.
  context->add_pre_shutdown_callback(
    []() {
      // A signal's shutdown first stops LF and waits for its stop to
      // settle; the process end bound runs from here, re-armed on
      // progress and capped at RCLLF_STOP_MAX_MS, which bounds this wait too.
      if (!ros2_lf_binding::g_own_rclcpp_shutdown.load()) {
        ros2_lf_binding::arm_stop_timeout();
      }
      ros2_lf_binding::stop_lf_before_context_shutdown(
        std::chrono::milliseconds(ros2_lf_binding::g_stop_max_ms.load()));
      if (!ros2_lf_binding::close_user_callbacks(std::chrono::milliseconds(2000))) {
        RCLCPP_WARN(
          rclcpp::get_logger("ros2_lf_binding"),
          "rclcpp shutdown: a user callback still runs after 2000 ms; shutting down anyway");
      }
      // The lifecycle pins keep entities of nodes alive. They go before rclcpp
      // and the nodes do.
      ros2_lf_binding::lifecycle_gate_release_pins();
    });
  context->add_on_shutdown_callback(&ros2_lf_binding::on_rclcpp_shutdown);

  // Initialise the LF-driven ROS clock before any nodes are constructed
  // (ADR 0001). Node clocks are attached as nodes are constructed.
  ros2_lf_binding::lf_clock_init();

  const char * system_json_env = std::getenv("SYSTEM_JSON");
  ros2_lf_binding::g_system_json_cstr = system_json_env ? system_json_env : nullptr;

  // Fall back to --system-json argv argument
  if (!ros2_lf_binding::g_system_json_cstr) {
    for (int i = 1; i + 1 < argc; ++i) {
      if (std::string(argv[i]) == "--system-json") {
        ros2_lf_binding::g_system_json_cstr = argv[i + 1];
        break;
      }
    }
  }

  ros2_lf_binding::g_registration_executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  std::weak_ptr<rclcpp::Executor> weak_exec = ros2_lf_binding::g_registration_executor;
  // The manager's own clock is never read in a reaction. The components it
  // loads get LF time through LFComponentManager::create_node_options.
  ros2_lf_binding::g_manager = std::make_shared<ros2_lf_binding::LFComponentManager>(
    weak_exec, "lf_component_manager",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Execution-policy table.  Absent file -> built-in defaults silently;
  // corrupt/unparseable/wrong-schema_version -> loud init failure (fail-closed,
  // design §2.4 item 4).  Built before any ros2_lf_bind_node startup reaction
  // runs, so every handle registration sees a populated table.
  const std::string exec_config_path = ros2_lf_binding::find_exec_config_path(argc, argv);
  // A named exec_config.json that does not exist is a wrong path, not a
  // request for defaults: refuse to start. An unset path still
  // means built-in defaults.
  const std::string exec_config_missing =
    ros2_lf_binding::missing_file_error("exec_config.json", exec_config_path);
  if (!exec_config_missing.empty()) {
    RCLCPP_FATAL(
      ros2_lf_binding::g_manager->get_logger(), "ros2_lf_runtime_init: %s. Refusing to start.",
      exec_config_missing.c_str());
    ros2_lf_binding::runtime_fatal_exit("startup refused");
  }
  const auto policy_load = ros2_lf_binding::policy_table_load(exec_config_path);
  if (!policy_load.ok) {
    RCLCPP_FATAL(
      ros2_lf_binding::g_manager->get_logger(),
      "exec_config.json is invalid: %s. Refusing to start with an unknown execution "
      "policy — fix the file or remove it to fall back to built-in defaults.",
      policy_load.error.c_str());
    ros2_lf_binding::runtime_fatal_exit("startup refused");
  }
  if (policy_load.file_present) {
    RCLCPP_INFO(
      ros2_lf_binding::g_manager->get_logger(),
      "ros2_lf_runtime_init: loaded execution policy from %s (%zu subscription row(s))",
      exec_config_path.c_str(), ros2_lf_binding::policy_table_row_count());
  } else if (!exec_config_path.empty()) {
    RCLCPP_INFO(
      ros2_lf_binding::g_manager->get_logger(),
      "ros2_lf_runtime_init: exec_config '%s' not found; using built-in take policy defaults",
      exec_config_path.c_str());
  }
  ros2_lf_binding::policy_table_log_env_overrides();

  auto clock_mode = ros2_lf_binding::parse_clock_publish_mode();
  ros2_lf_binding::clock_publisher_init(clock_mode);

  if (!ros2_lf_binding::g_system_json_cstr) {
    RCLCPP_FATAL(
      ros2_lf_binding::g_manager->get_logger(),
      "SYSTEM_JSON not set and --system-json not provided. "
      "The lfc runtime cannot load any components without a system.json path. "
      "Pass it via the SYSTEM_JSON env var or --system-json <path>.");
    ros2_lf_binding::runtime_fatal_exit("startup refused");
  }

  const std::string system_json_missing =
    ros2_lf_binding::missing_file_error("system.json", ros2_lf_binding::g_system_json_cstr);
  if (ros2_lf_binding::g_system_json_cstr[0] == '\0' || !system_json_missing.empty()) {
    RCLCPP_FATAL(
      ros2_lf_binding::g_manager->get_logger(),
      "ros2_lf_runtime_init: %s. Refusing to start with no component list.",
      system_json_missing.empty() ? "SYSTEM_JSON is empty" : system_json_missing.c_str());
    ros2_lf_binding::runtime_fatal_exit("startup refused");
  }

  RCLCPP_INFO(
    ros2_lf_binding::g_manager->get_logger(), "ros2_lf_runtime_init: loading components from %s",
    ros2_lf_binding::g_system_json_cstr);
}

void ros2_lf_runtime_load_nodes(void)
{
  if (!ros2_lf_binding::g_manager || !ros2_lf_binding::g_system_json_cstr) {
    return;
  }
  const std::string system_json(ros2_lf_binding::g_system_json_cstr);

  const auto component_parse = ros2_lf_binding::parse_component_list(system_json);
  if (!component_parse.ok) {
    RCLCPP_FATAL(
      ros2_lf_binding::g_manager->get_logger(),
      "system.json component list is invalid: %s. Refusing to start with an unreadable "
      "component list.",
      component_parse.error.c_str());
    ros2_lf_binding::runtime_fatal_exit("startup refused");
  }
  const auto & components = component_parse.instances;
  RCLCPP_INFO(
    ros2_lf_binding::g_manager->get_logger(), "Found %zu component instance(s) to load",
    components.size());

  // One load_component call per instance (not per class): N instances of one
  // class load N times, each keyed by its own binding_key, so the second
  // instance no longer clobbers the first (design §4 / survey §3).
  for (const auto & inst : components) {
    const std::string qualified = inst.package + "::" + inst.plugin_class;
    Ros2LfNodeConstructionScope scope(inst.binding_key);  // admitted readers
    bool loaded = ros2_lf_binding::g_manager->load_component(
      inst.package, qualified, inst.binding_key, *ros2_lf_binding::g_registration_executor,
      inst.node_name, inst.node_namespace, inst.remap_rules, inst.param_args);
    if (!loaded) {
      // Retry without the package-qualified prefix (legacy plugin lookup
      // behavior, kept verbatim).
      loaded = ros2_lf_binding::g_manager->load_component(
        inst.package, inst.plugin_class, inst.binding_key,
          *ros2_lf_binding::g_registration_executor,
        inst.node_name, inst.node_namespace, inst.remap_rules, inst.param_args);
    }
    if (!loaded) {
      RCLCPP_WARN(
        ros2_lf_binding::g_manager->get_logger(),
        "Failed to load component instance '%s' (binding_key='%s') after retry", qualified.c_str(),
        inst.binding_key.c_str());
    }
  }

  auto non_components = ros2_lf_binding::parse_non_component_list(system_json);
  RCLCPP_INFO(
    ros2_lf_binding::g_manager->get_logger(), "Found %zu non-component node(s) in registry",
    non_components.size());
  for (const auto & cls : non_components) {
    // Only checked here: the node is bound by its startup reaction and its
    // infrastructure services are admitted by the ingress role.
    // Two or more instances register under `<cls>__<suffix>` keys.
    if (!ros2_lf_class_has_registered_node(cls)) {
      RCLCPP_WARN(
        ros2_lf_binding::g_manager->get_logger(),
        "Non-component class '%s' not in registry — "
        "node_factories.cpp may not have been linked",
        cls.c_str());
      continue;
    }
    RCLCPP_INFO(
      ros2_lf_binding::g_manager->get_logger(), "Registered non-component node: %s", cls.c_str());
  }
}

namespace ros2_lf_binding
{
/// Hand each subscription the writers the graph lists on its
/// topic before the first tag, so a reader overflow before the first sample
/// of one of them is reported (take_note_start_writers). The graph of any node
/// lists every endpoint of the domain, so the manager node's view is used.
static void note_start_writers(const std::vector<rclcpp::SubscriptionBase::SharedPtr> & subs)
{
  for (const auto & sub : subs) {
    std::vector<std::array<uint8_t, 16>> gids;
    for (const auto & info : g_manager->get_publishers_info_by_topic(sub->get_topic_name())) {
      std::array<uint8_t, 16> gid{};
      const auto & raw = info.endpoint_gid();
      std::copy_n(raw.begin(), std::min(raw.size(), gid.size()), gid.begin());
      gids.push_back(gid);
    }
    take_note_start_writers(*sub, gids);
  }
}
}  // namespace ros2_lf_binding

void ros2_lf_runtime_wait_ready(void)
{
  if (!ros2_lf_binding::g_manager) {
    ros2_lf_binding::disarm_startup_bound();
    return;
  }

  // Policy miss summary. This runs from the pre-main constructor, before LF
  // startup reactions call ros2_lf_bind_node and before any late bind of a
  // lifecycle handle. resolve_policy() runs only inside those
  // binds, so no lookup has happened yet and this call logs nothing: misses
  // of the binds that follow are not summarized. The startup stage below
  // cannot use the handle table for the same reason.
  ros2_lf_binding::policy_table_log_miss_summary();

  // Startup stage: resolve and check every admitted edge, admit
  // its subscription into ingress, or exit nonzero before tag 0.
  ros2_lf_binding::corr_rt::startup_or_exit(
    [](const std::string & key) {
      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
      if (ros2_lf_binding::g_manager) {
        base = ros2_lf_binding::g_manager->find_by_class(key);
      }
      if (!base) {
        base = ros2_lf_get_node(key);
      }
      return base;
    });
  // The rcllf trace of this epoch starts once edges are admitted.
  ros2_lf_binding::corr_rt::trace_begin();
  // From here on an action send needs an LF origin.
  ros2_lf_binding::action_refusal_arm(true);
  // The startup bound ends here; every wait below has its own
  // timeout, and a federate's RTI handshake that follows waits by design.
  ros2_lf_binding::disarm_startup_bound();

  const int timeout_ms = ros2_lf_binding::discovery_timeout_ms();
  // A cross-federate edge starts only with its designated peer
  // discovered, so that no early publication is lost before the match.
  if (!ros2_lf_binding::g_peer_check_deferred) {
    ros2_lf_binding::corr_rt::xfed_peer_check_or_exit(timeout_ms);
  }
  // In a federate this runs before the RTI handshake only. A run
  // after it would add to the startup skew of a fast federate.
  ros2_lf_binding::run_match_warmup("startup");
  if (timeout_ms <= 0) {
    return;
  }

  auto subs = ros2_lf_binding::collect_subscriptions();
  // A declared external topic has no publisher in the system, so its
  // subscriptions would only make every start wait the full timeout.
  const auto external_begin = std::remove_if(
    subs.begin(), subs.end(), [](const rclcpp::SubscriptionBase::SharedPtr & s) {
      return ros2_lf_binding::external_input::is_declared_topic(s->get_topic_name());
    });
  const size_t external = static_cast<size_t>(std::distance(external_begin, subs.end()));
  subs.erase(external_begin, subs.end());
  if (external > 0) {
    RCLCPP_INFO(
      ros2_lf_binding::g_manager->get_logger(),
      "ros2_lf_runtime_wait_ready: not waiting for %zu subscription(s) on external_topics",
      external);
  }
  if (subs.empty()) {
    return;
  }

  // Every reader has a writer and every local
  // writer sees every local reader before tag 0, else the start is refused.
  // A start before the match made the first tags depend on discovery: a
  // writer dropped what it published before it saw the reader, and a node
  // gated on its publisher's subscription count held its first ticks.
  std::vector<ros2_lf_binding::match_warmup::LocalReader> readers;
  for (const auto & s : subs) {
    readers.push_back({s->get_topic_name(), [s]() {return s->get_publisher_count();}});
  }
  const auto targets = ros2_lf_binding::match_warmup::local_targets(
    readers, [](const std::string & topic) {
      return ros2_lf_binding::corr_rt::publisher_match_counts(topic);
    });
  RCLCPP_INFO(
    ros2_lf_binding::g_manager->get_logger(),
    "ros2_lf_runtime_wait_ready: waiting for %zu subscription(s) and %zu local publisher "
    "topic(s) to match (timeout %d ms)",
    subs.size(), targets.size() - subs.size(), timeout_ms);
  ros2_lf_binding::match_warmup::enforce_ready(
    ros2_lf_binding::match_warmup::wait(targets, std::chrono::milliseconds(timeout_ms)),
    "wait_ready", timeout_ms);
  RCLCPP_INFO(
    ros2_lf_binding::g_manager->get_logger(),
    "ros2_lf_runtime_wait_ready: all subscriptions matched; LF start time anchored");
  ros2_lf_binding::note_start_writers(subs);
  // No settle sleep (was 50 ms, unjustified per audit F12): it stood in for
  // the writer side of the match, which the targets above now require.
}

void ros2_lf_runtime_defer_peer_check(void)
{
  ros2_lf_binding::g_peer_check_deferred = true;
}

void ros2_lf_runtime_peer_check(void)
{
  // From here the table of a fast federate judges cutoffs on its
  // logical tag (corr_rt::table_clock).
  ros2_lf_binding::corr_rt::set_federated(true);
  // The RTI sends the start time only once every federate has
  // connected, so every peer has created its endpoints before main(). The
  // timeout now covers discovery alone, not the launch spread of the federates.
  // The end-of-run COMPLETE channel, created before the peer
  // check so its discovery overlaps the run.
  ros2_lf_binding::corr_rt::complete_end_channel_init();
  ros2_lf_binding::corr_rt::xfed_peer_check_or_exit(ros2_lf_binding::discovery_timeout_ms());
  // The RTI start time is known here, before tag 0.
  // The stall watch starts from it, so an upstream federate that stops before
  // the start tag is reported like one that stops mid-run. The timers of a
  // non-lifecycle node are bound at node loading, before this point.
  ros2_lf_binding::corr_rt::stall_watch_start(ros2_lf_binding::lf_start_tag().first);
  ros2_lf_binding::test_start_delay();
}

// The shutdown runs once. In a federation it runs
// first from the lf_terminate_execution() wrapper, before the resign, and the
// exit it decides waits for the atexit call that follows reactor-c's
// termination(), so the resign and reactor-c's own teardown still happen.
// A second call while the first still runs comes from another thread's exit()
// (reactor-c's RTI listener exits on MSG_TYPE_FAILED): it ends the process at
// once, so no static destructor runs under live nodes.
enum : int { kShutdownIdle = 0, kShutdownRunning = 1, kShutdownDone = 2 };
static std::atomic<int> g_runtime_shutdown_state{kShutdownIdle};
static bool g_runtime_defer_exit = false;
static int g_runtime_deferred_exit = -1;

// The id of a federate that sends to this one and left the
// federation without resigning (an error exit or a crash), -1 while none did.
static std::atomic<int> g_upstream_failed_federate{-1};

static bool upstream_failed()
{
  return g_upstream_failed_federate.load() >= 0;
}

// reactor-c patch 0009 calls this weak hook on its RTI listener
// thread when the RTI reports that a federate feeding this one left without
// resigning. Its inputs here would stay absent with no record, so the run is
// invalid: log it now, and the end of the run exits 3. reactor-c requests the
// stop right after this returns, so the federation stops at one tag instead of
// running to its timeout.
extern "C" void rcllf_upstream_failed_hook(int fed_id)
{
  int none = -1;
  g_upstream_failed_federate.compare_exchange_strong(none, fed_id);
  RCLCPP_ERROR(
    rclcpp::get_logger("ros2_lf_binding"),
    "rcllf: federate %d, which sends to this federate, left the federation without resigning "
    "(an error exit or a crash); its inputs here end now. Stopping, exit code 3", fed_id);
}

static void runtime_end_process(int code)
{
  if (g_runtime_defer_exit) {
    g_runtime_deferred_exit = code;
    return;
  }
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(code);
}

void ros2_lf_runtime_shutdown_before_resign(void)
{
  g_runtime_defer_exit = true;
  ros2_lf_runtime_shutdown();
  g_runtime_defer_exit = false;
}

static void runtime_shutdown_teardown(void);

/// Test only: ROS2_LF_TEST_RESIGN_DELAY="<name>:<ms>"
/// holds the RTI resign of a federate whose executable name contains <name>,
/// the way the RTI held it behind a peer's resign in one recorded two-host run.
void ros2_lf_runtime_test_resign_delay(void)
{
  ros2_lf_binding::test_named_delay("ROS2_LF_TEST_RESIGN_DELAY", "RTI resign");
}

void ros2_lf_runtime_shutdown(void)
{
  int idle = kShutdownIdle;
  if (!g_runtime_shutdown_state.compare_exchange_strong(idle, kShutdownRunning)) {
    if (idle == kShutdownRunning) {
      std::fflush(stdout);
      std::fflush(stderr);
      std::_Exit(1);
    }
    if (g_runtime_deferred_exit >= 0) {
      runtime_end_process(g_runtime_deferred_exit);
    }
    return;
  }
  runtime_shutdown_teardown();
  // Reached only when the teardown returned (a deferred or a clean end), so a
  // later sequential call returns or applies the deferred code instead of
  // taking the concurrent-call exit above.
  g_runtime_shutdown_state.store(kShutdownDone);
}

static void runtime_shutdown_teardown(void)
{
  ros2_lf_binding::disarm_startup_bound();  // the stop bound takes over
  ros2_lf_binding::g_runtime_shutting_down.store(true);
  // LF has stopped; the teardown gets a bound of its own.
  ros2_lf_binding::rearm_stop_timeout();
  // LF has stopped, no later send can reach a port.
  ros2_lf_binding::action_refusal_arm(false);
  ros2_lf_binding::action_external_shutdown();
  // Item 6: normally run by the nodes' reaction(shutdown). The context never
  // got these callbacks, so whatever is left runs now, before it goes down.
  ros2_lf_binding::action_pre_shutdown_run_left("at the runtime shutdown");
  // This program's last publications reach their readers before it
  // exits (egress drain), and the samples still in flight at the stop are
  // taken and recorded late (not never) while the ingress and the context
  // are still up (ingress drain); a signal's
  // rclcpp shutdown waits for g_stop_settled (set after the end-of-run COMPLETE
  // exchange below) before it invalidates the context.
  ros2_lf_binding::stop_mark("egress_drain_begin");
  ros2_lf_binding::corr_rt::drain_egress(ros2_lf_binding::stop_drain_bound_ms());
  ros2_lf_binding::stop_mark("egress_drain_end");
  // The ingress thread stopped at the stop tag (last mailbox close) or
  // in the exit backstop, before this atexit handler; the drain takes on this
  // thread, so join first and there is one taker.
  // Normally already stopped by the drain hosts' shutdown reactions (R8).
  ros2_lf_binding::IngressRole::instance().stop_and_join();
  ros2_lf_binding::stop_mark("ingress_joined");
  ros2_lf_binding::stop_mark("ingress_drain_begin");
  ros2_lf_binding::corr_rt::drain_in_flight(ros2_lf_binding::stop_drain_bound_ms());
  ros2_lf_binding::stop_mark("ingress_drain_end");
  ros2_lf_binding::take_stats_log_summary("shutdown");
  ros2_lf_binding::service_pairing::log_summary("shutdown");
  // Every declared minimum input period with its violation count.
  ros2_lf_binding::input_period::log_summary();
  // Per external topic, how many messages were admitted, late,
  // overrun, collided, after the stop tag or refused.
  ros2_lf_binding::external_input::log_summary();
  ros2_lf_binding::output_deadline::log_summary();
  ros2_lf_binding::plant_record::close();  // flush, close, summary
  ros2_lf_binding::IngressRole::instance().log_corr_batch_summary();
  if (const uint64_t skipped = ros2_lf_binding::g_user_callbacks_skipped.load()) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf: %llu user callback(s) skipped after rclcpp shutdown began",
      static_cast<unsigned long long>(skipped));  // NOLINT(runtime/int)
  }
  // The one trace-close point. Reconcile every process, also
  // one without admitted edges, then snapshot and emit run_end before entity
  // release, so later activity takes no seq. This is also the only
  // reconcile, after LF returned, so every reaction of the stop tag in every
  // reactor of the process is counted.
  // The close first exchanges the end-of-run
  // COMPLETE counts of the cross-federate edges. A peer publishes its count
  // after its own LF return and its two drains, each at most a quarter of the
  // stop bound (stop_drain_bound_ms), so this side waits half the bound
  // (stop_join_bound_ms) and the teardown bound is re-armed for it. A count
  // later than that fails the reconcile (complete_end_missing): too short a
  // wait reports a failure, it never passes one silently.
  ros2_lf_binding::rearm_stop_timeout();
  ros2_lf_binding::corr_rt::close_and_reconcile(ros2_lf_binding::stop_join_bound_ms());
  ros2_lf_binding::stop_mark("reconciled");
  ros2_lf_binding::rearm_stop_timeout();
  ros2_lf_binding::corr_rt::complete_end_ack(ros2_lf_binding::stop_drain_bound_ms());
  ros2_lf_binding::stop_mark("complete_end_acked");
  // The stop settles only now. The end-of-run COMPLETE exchange
  // above publishes and takes over DDS on this context, so a signal's
  // rclcpp shutdown, held in its pre-shutdown callback until g_stop_settled,
  // must not invalidate the context before it. Settled after the drains:
  // otherwise the end-of-run counts of a SIGINT-ended federated run are
  // published on a dead context and every peer waits the full bound.
  ros2_lf_binding::g_stop_settled.store(true);
  // Join rclcpp's signal thread before anything is torn down and
  // let a later SIGINT/SIGTERM only count and bound the process end.
  // Uninstalling restores the SIG_IGN that lf_mark_running() saved, so no
  // signal kills the process in between. It joins the signal thread that
  // waits for g_stop_settled, so it comes after the store.
  rclcpp::uninstall_signal_handlers();
  ros2_lf_binding::install_late_signal_counter();
  ros2_lf_binding::corr_rt::trace_close();
  // The same release and context fini as a startup refusal.
  const rcl_ret_t fini = ros2_lf_binding::release_entities_and_finalize_context();
  ros2_lf_binding::stop_mark("context_fini", static_cast<int64_t>(fini));
  const int late_signals = ros2_lf_binding::g_late_signals.load();
  if (late_signals > 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf: %d signal(s) during shutdown, counted; they bound the process end",
      late_signals);
  }
  if (fini != RCL_RET_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_lf_binding"),
      "rcllf: rmw context not finalized at shutdown (a node is still alive, rcl_ret_t %d), "
      "exiting without static destructors", static_cast<int>(fini));
    rcl_reset_error();
    runtime_end_process(
      ros2_lf_binding::corr_rt::reconcile_failed() ||
      ros2_lf_binding::external_input::replay_failed() || upstream_failed() ? 3 : EXIT_SUCCESS);
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("ros2_lf_binding"), "rcllf: rmw context finalized at shutdown");
  if (ros2_lf_binding::corr_rt::reconcile_failed() ||
    ros2_lf_binding::external_input::replay_failed() || upstream_failed())
  {
    // Q3: a broken correspondence invariant fails the run after the
    // summary printed at epoch close. So does a replay reported FAILED.
    // So does an upstream federate that left without resigning.
    runtime_end_process(3);
  }
}

}  // extern "C"
