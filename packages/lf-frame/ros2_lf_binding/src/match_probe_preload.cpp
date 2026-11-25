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

/// @file match_probe_preload.cpp
/// @brief LD_PRELOAD probe that traces when DDS matches readers and writers.
///
/// A VOLATILE sample published before DDS matched the reader to the writer is
/// never delivered, and the ros2 tracepoints do not show when matching
/// happens. Preloaded into any ROS 2 process, stock or rcllf, this registers
/// every subscription rcl_subscription_init and every publisher
/// rcl_publisher_init creates with one background thread. Every millisecond
/// the thread asks each reader for its matched publisher count and each writer
/// for its matched subscription count. Every increase over the last polled
/// count emits `rcllf_dds:reader_matched` or `rcllf_dds:writer_matched` with
/// the age of the last poll that still found the lower count. A decrease is
/// recorded without an event, so a replacement peer's match is an increase
/// again. Matching is not symmetric in time: a reader
/// can see the writer before the writer sees the reader.
///
/// On rmw_fastrtps the probe also times each change from the rmw matched event.
/// It hands rmw_event_set_callback an event struct that points at
/// the entity, as rmw_*_event_init would fill it, without calling that init.
/// The init enables the matched status on the reader's status condition, which
/// the subscription's wait set shares, and since nothing took the event the wait
/// stayed triggered and every stock executor spun.
/// The callback alone changes no status condition: the rmw listener always
/// handles the matched status and calls the callback right after it updated
/// the count. The callback reads the count like a poll and emits the same event
/// with `source` "event". The poll keeps running, so it still supplies the last
/// observation of the lower count and records a decrease. Once the callback is
/// set, it owns every increase: rmw raises the count before it calls the
/// callback, and a poll that read the raised count first took the match from it.
/// The poll records an increase (`source` "poll") only for an
/// entry whose callback rmw refused. Other rmw implementations, or
/// RCLLF_MATCH_PROBE_EVENTS=0, use the poll alone. Not linked into rcllf.
///
/// Entries are polled by the rmw handle and keyed by the rcl `impl` pointer,
/// never by the rcl struct. rcl inits each node's rosout publisher in a stack
/// local and copies the struct into its logger map, so the address
/// rcl_publisher_init saw dies while the publisher lives on (a probe that kept
/// it crashed the reference system). Every copy shares `impl` and the rmw
/// handle until rcl_*_fini. The fini interposers untrack by `impl` before the
/// real fini runs, without an rcl validity check: after the context shut down
/// rcl_subscription_is_valid can be false while the rmw handle still exists,
/// and a skipped untrack let the poll thread use the freed handle.

#include <dlfcn.h>
#include <time.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rcl/publisher.h"
#include "rcl/subscription.h"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#define TRACEPOINT_CREATE_PROBES
#define TRACEPOINT_DEFINE
#include "rcllf_dds_tp.h"  // NOLINT(build/include_subdir)

namespace
{

struct Pending
{
  bool reader;  // rmw_handle is an rmw_subscription_t, else an rmw_publisher_t
  const void * key;  // the rcl impl pointer, shared by every copy of the struct
  const void * rmw_handle;
  std::string topic;
  size_t count;  // matched count at the last observation
  int64_t unmatched_ns;  // CLOCK_MONOTONIC of the last observation that found a lower count
  // The rmw matched event callback owns every increase: set, or being set
  // (track() clears it when rmw refuses the callback).
  bool armed = true;
  bool dead = false;  // a count query failed, the entity stays untraced from here on
};

// Entries live on the heap: the rmw callback holds a pointer to its entry.
struct Poller
{
  std::mutex mu;
  std::condition_variable cv;
  std::vector<std::unique_ptr<Pending>> pending;
  bool started = false;
};

// Leaked on purpose: the detached poll thread may still wait on it while the
// process runs its static destructors.
Poller & poller()
{
  static Poller * p = new Poller;
  return *p;
}

// Events emitted so far, reader side [1] and writer side [0], for the tests,
// and the same split by source, poll [0] and rmw matched event [1].
std::atomic<uint64_t> g_events[2];
std::atomic<uint64_t> g_source_events[2][2];

// Test seam: poll sweeps completed so far, and whether the rmw
// matched event callback waits for two of them before it observes. rmw raised
// the count before it called the callback, so a sweep that starts inside the
// wait reads the raised count first: the order the issue describes, forced
// instead of left to the scheduler. Off outside the test.
std::atomic<uint64_t> g_sweeps{0};
std::atomic<bool> g_poll_first{false};

int64_t mono_ns()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// One observation of one entry, under the poller lock. `from_event` says
// whether the rmw matched event callback or the poll made it.
void observe(Pending & e, bool from_event)
{
  if (e.dead) {
    return;
  }
  const int64_t before = mono_ns();
  size_t count = 0;
  const rmw_ret_t ret = e.reader ?
    rmw_subscription_count_matched_publishers(
    static_cast<const rmw_subscription_t *>(e.rmw_handle), &count) :
    rmw_publisher_count_matched_subscriptions(
    static_cast<const rmw_publisher_t *>(e.rmw_handle), &count);
  if (ret != RMW_RET_OK) {
    rmw_reset_error();
    e.dead = true;
    return;
  }
  if (count > e.count && e.armed && !from_event) {
    // The rmw matched event callback owns an increase once it is set.
    // rmw raises the count before it calls the callback, so a poll can
    // read it first; taking it here left the callback nothing to time. The
    // count and the last lower observation stay, so the callback emits the
    // match with the age it has when it runs first.
    return;
  }
  if (count <= e.count) {
    e.count = count;  // a decrease: the next increase is a new match
    e.unmatched_ns = before;
    return;
  }
  const int64_t age = mono_ns() - e.unmatched_ns;
  const char * source = from_event ? "event" : "poll";
  e.count = count;
  g_events[e.reader ? 1 : 0].fetch_add(1);
  g_source_events[e.reader ? 1 : 0][from_event ? 1 : 0].fetch_add(1);
  if (e.reader) {
    tracepoint(
      rcllf_dds, reader_matched, e.rmw_handle, e.topic.c_str(), static_cast<uint64_t>(count), age,
      source);
  } else {
    tracepoint(
      rcllf_dds, writer_matched, e.rmw_handle, e.topic.c_str(), static_cast<uint64_t>(count), age,
      source);
  }
  e.unmatched_ns = before;
}

// The rmw matched event callback. rmw_fastrtps calls it from its listener
// thread under its own event lock, right after it updated the matched count,
// for a decrease too. It reads the count under the poller lock like a poll.
void on_matched(const void * user_data, size_t /*number_of_events*/)
{
  if (g_poll_first.load()) {
    // A sweep in progress now may have read the entry before rmw raised the
    // count, so wait for the one after it. The bound only keeps a stalled poll
    // from blocking rmw's listener thread.
    const uint64_t target = g_sweeps.load() + 2;
    const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (g_sweeps.load() < target && std::chrono::steady_clock::now() < give_up) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }
  auto & pl = poller();
  std::lock_guard<std::mutex> lk(pl.mu);
  observe(*static_cast<Pending *>(const_cast<void *>(user_data)), true);
}

bool events_wanted(const char * identifier)
{
  static const bool off = [] {
      const char * v = std::getenv("RCLLF_MATCH_PROBE_EVENTS");
      return v != nullptr && std::strcmp(v, "0") == 0;
    }();
  return !off && identifier != nullptr && std::strncmp(identifier, "rmw_fastrtps", 12) == 0;
}

// The event struct rmw_*_event_init would fill, without its status condition change.
rmw_event_t matched_event(bool reader, const void * rmw_handle)
{
  rmw_event_t ev = rmw_get_zero_initialized_event();
  if (reader) {
    const auto * sub = static_cast<const rmw_subscription_t *>(rmw_handle);
    ev.implementation_identifier = sub->implementation_identifier;
    ev.data = sub->data;
    ev.event_type = RMW_EVENT_SUBSCRIPTION_MATCHED;
  } else {
    const auto * pub = static_cast<const rmw_publisher_t *>(rmw_handle);
    ev.implementation_identifier = pub->implementation_identifier;
    ev.data = pub->data;
    ev.event_type = RMW_EVENT_PUBLICATION_MATCHED;
  }
  return ev;
}

// Sets (or with a null entry clears) the callback. Never under the poller lock:
// rmw may call the callback from inside rmw_event_set_callback.
bool set_matched_callback(bool reader, const void * rmw_handle, Pending * entry)
{
  rmw_event_t ev = matched_event(reader, rmw_handle);
  if (!events_wanted(ev.implementation_identifier)) {
    return false;
  }
  if (rmw_event_set_callback(&ev, entry ? on_matched : nullptr, entry) != RMW_RET_OK) {
    rmw_reset_error();
    return false;
  }
  return true;
}

void poll_loop()
{
  auto & pl = poller();
  std::unique_lock<std::mutex> lk(pl.mu);
  while (true) {
    pl.cv.wait(lk, [&pl] {return !pl.pending.empty();});
    for (auto & e : pl.pending) {
      observe(*e, false);
    }
    g_sweeps.fetch_add(1);
    lk.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    lk.lock();
  }
}

void track(Pending entry)
{
  auto & pl = poller();
  auto owned = std::make_unique<Pending>(std::move(entry));
  Pending * e = owned.get();
  {
    std::lock_guard<std::mutex> lk(pl.mu);
    pl.pending.push_back(std::move(owned));
    if (!pl.started) {
      pl.started = true;
      std::thread(poll_loop).detach();
    }
  }
  pl.cv.notify_one();
  // The entry is armed while the callback is being set.
  // rmw_fastrtps sets it under the lock that every matched count change holds,
  // and calls it at once when a change happened before, which the probe never
  // took: every increase from here on reaches the callback, so a poll that took
  // one in this window would leave the callback nothing again. On refusal the
  // poll records the increase it skipped, as the stored count stayed.
  // Only fini erases the entry, and fini of an entity runs after its init returned.
  const bool armed = set_matched_callback(e->reader, e->rmw_handle, e);
  std::lock_guard<std::mutex> lk(pl.mu);
  e->armed = armed;
}

// Observations run under the poller lock, so none touches the handle once it is
// gone. The callback is cleared first, outside the lock, so rmw holds no pointer
// to the erased entry. A null key (a zero initialized entity) tracks nothing.
void untrack(const void * key)
{
  if (key == nullptr) {
    return;
  }
  auto & pl = poller();
  std::vector<std::pair<bool, const void *>> armed;
  {
    std::lock_guard<std::mutex> lk(pl.mu);
    for (auto & e : pl.pending) {
      if (e->key == key && e->armed) {
        armed.emplace_back(e->reader, e->rmw_handle);
      }
    }
  }
  for (const auto & [reader, handle] : armed) {
    set_matched_callback(reader, handle, nullptr);
  }
  std::lock_guard<std::mutex> lk(pl.mu);
  for (auto it = pl.pending.begin(); it != pl.pending.end(); ) {
    it = (*it)->key == key ? pl.pending.erase(it) : it + 1;
  }
}

}  // namespace

/// Test hooks: entities polled now, and events emitted so far on one side.
extern "C" size_t rcllf_match_probe_tracked_for_test()
{
  auto & pl = poller();
  std::lock_guard<std::mutex> lk(pl.mu);
  return pl.pending.size();
}

extern "C" uint64_t rcllf_match_probe_events_for_test(int reader)
{
  return g_events[reader != 0 ? 1 : 0].load();
}

/// Events of one side so far from one source: the poll (0) or the rmw matched event (1).
extern "C" uint64_t rcllf_match_probe_source_events_for_test(int reader, int from_event)
{
  return g_source_events[reader != 0 ? 1 : 0][from_event != 0 ? 1 : 0].load();
}

/// Test seam: with `on` set, every rmw matched event callback waits
/// for two full poll sweeps before it observes, so the poll reads a raised count first.
extern "C" void rcllf_match_probe_poll_first_for_test(int on)
{
  g_poll_first.store(on != 0);
}

extern "C" rcl_ret_t rcl_subscription_init(
  rcl_subscription_t * subscription, const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support, const char * topic_name,
  const rcl_subscription_options_t * options)
{
  using Fn = rcl_ret_t (*)(
    rcl_subscription_t *, const rcl_node_t *, const rosidl_message_type_support_t *,
    const char *, const rcl_subscription_options_t *);
  static Fn next = reinterpret_cast<Fn>(dlsym(RTLD_NEXT, "rcl_subscription_init"));
  // A reader is unmatched before its init returns.
  const int64_t created = mono_ns();
  const rcl_ret_t ret = next(subscription, node, type_support, topic_name, options);
  if (ret == RCL_RET_OK) {
    const char * topic = rcl_subscription_get_topic_name(subscription);
    track(
      {true, subscription->impl, rcl_subscription_get_rmw_handle(subscription),
        topic ? topic : "", 0, created});
  }
  return ret;
}

extern "C" rcl_ret_t rcl_subscription_fini(rcl_subscription_t * subscription, rcl_node_t * node)
{
  using Fn = rcl_ret_t (*)(rcl_subscription_t *, rcl_node_t *);
  static Fn next = reinterpret_cast<Fn>(dlsym(RTLD_NEXT, "rcl_subscription_fini"));
  if (subscription != nullptr) {
    untrack(subscription->impl);
  }
  return next(subscription, node);
}

extern "C" rcl_ret_t rcl_publisher_init(
  rcl_publisher_t * publisher, const rcl_node_t * node,
  const rosidl_message_type_support_t * type_support, const char * topic_name,
  const rcl_publisher_options_t * options)
{
  using Fn = rcl_ret_t (*)(
    rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *,
    const char *, const rcl_publisher_options_t *);
  static Fn next = reinterpret_cast<Fn>(dlsym(RTLD_NEXT, "rcl_publisher_init"));
  // A writer matches no reader before its init returns.
  const int64_t created = mono_ns();
  const rcl_ret_t ret = next(publisher, node, type_support, topic_name, options);
  if (ret == RCL_RET_OK) {
    const char * topic = rcl_publisher_get_topic_name(publisher);
    track(
      {false, publisher->impl, rcl_publisher_get_rmw_handle(publisher), topic ? topic : "", 0,
        created});
  }
  return ret;
}

extern "C" rcl_ret_t rcl_publisher_fini(rcl_publisher_t * publisher, rcl_node_t * node)
{
  using Fn = rcl_ret_t (*)(rcl_publisher_t *, rcl_node_t *);
  static Fn next = reinterpret_cast<Fn>(dlsym(RTLD_NEXT, "rcl_publisher_fini"));
  if (publisher != nullptr) {
    untrack(publisher->impl);
  }
  return next(publisher, node);
}
