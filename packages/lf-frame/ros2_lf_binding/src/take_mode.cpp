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

/// @file take_mode.cpp
/// @brief ROS2_LF_TAKE_MODE parsing, bounded-block take loop, take statistics.

#include "take_mode.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "per_subscription.hpp"
#include "rmw/serialized_message.h"
#include "rmw/rmw.h"
#include "corr_runtime.hpp"
#include "tag_log.hpp"
#include "thread_role.hpp"

namespace ros2_lf_binding
{
namespace
{

// Relaxed ordering throughout: counters feed a diagnostic summary only,
// there are no cross-counter invariants.
std::atomic<uint64_t> g_first_try{0};
std::atomic<uint64_t> g_skipped{0};
std::atomic<uint64_t> g_waited{0};
std::atomic<uint64_t> g_wait_total_ns{0};
std::atomic<uint64_t> g_wait_max_ns{0};
std::atomic<uint64_t> g_timeouts{0};
std::atomic<uint64_t> g_late_dropped{0};
std::atomic<uint64_t> g_untagged{0};
std::atomic<uint64_t> g_unsettled{0};  // samples in TagReader::ahead right now
std::atomic<uint64_t> g_poll_timeouts{0};
std::atomic<uint64_t> g_poll_timeout_ns{0};
std::atomic<uint64_t> g_reader_lost{0};  // samples replaced in a full reader
std::atomic<uint64_t> g_after_deadline{0};  // samples received after tag + T, dropped
std::atomic<uint64_t> g_unknown_missed{0};  // unknown-count federate misses
std::atomic<uint64_t> g_count_rule{0};  // drop-by-tag takes served by the count rule
std::atomic<uint64_t> g_debt_capped{0};  // skip misses past the debt cap

// Per-topic breakdown; only touched when the first take missed (rare),
// so a mutex is cheap enough even with concurrent LF workers.
std::mutex g_topic_mutex;
std::map<std::string, TopicTakeStats> g_topic_stats;

constexpr auto kPollInterval = std::chrono::microseconds(20);
// At most this many replaced samples of one gap are looked up in
// the tag log for their records; a larger gap (a reader stalled for ages) keeps
// the poll per missing record.
constexpr uint64_t kLostLookup = 4096;
constexpr auto kPeriodicLogInterval = std::chrono::seconds(10);

void update_max(std::atomic<uint64_t> & target, uint64_t value)
{
  uint64_t cur = target.load(std::memory_order_relaxed);
  while (cur < value &&
    !target.compare_exchange_weak(cur, value, std::memory_order_relaxed))
  {
  }
}

/// A sample taken ahead of its delivery, kept serialized so it can be
/// delivered into the caller's type-erased message later.
struct HeldSample
{
  std::shared_ptr<rclcpp::SerializedMessage> data;
  rclcpp::MessageInfo info;
};

/// Skip-mode state of one subscription (probe P17e). An LF reaction of one
/// subscription never runs on two workers at once, so only the map is locked.
struct SubDebt
{
  uint64_t debt = 0;  // owed samples: known-count misses not yet settled
  uint64_t cap = 0;
  std::deque<HeldSample> held;
  bool warned_miss = false;
  bool warned_drop = false;
  bool warned_multi = false;
  bool warned_lone = false;
  bool warned_after = false;  // a sample received after tag + T
  bool warned_unknown = false;  // an unknown-count federate miss
  // Fast mode: a lone sample with debt open was polled for its partner up to
  // the take timeout; until the debt changes, later lone samples do not wait.
  bool lone_poll_spent = false;
  // Reader overflow: the publication sequence number of the last
  // sample taken from each writer, by writer GID.
  std::map<tag_log::Gid, uint64_t> last_seq;
  bool warned_gap = false;
  int cft = -1;  // filter state at the last take; a change clears last_seq (holes are by design)
  // Samples the reader replaced, found by a gap at this tag, stand
  // in for that many empty takes of the same tag: they are declared lost once
  // (reader_lost) and owe no debt, no poll and no miss WARN.
  uint64_t lost_credit = 0;
  std::pair<int64_t, uint32_t> credit_tag{-1, 0};
};

std::mutex g_debt_mutex;
// Per live subscription, so a reader at a reused address owes nothing.
PerSubscription<SubDebt> g_debt;

SubDebt & debt_of(rclcpp::SubscriptionBase & sub)
{
  std::lock_guard<std::mutex> lock(g_debt_mutex);
  SubDebt & d = g_debt.at(sub);
  if (d.cap == 0) {
    // More late samples than the reader holds cannot come back. Provenance:
    // DERIVED from the reader's depth; 1000 for a KEEP_ALL reader (depth 0)
    // is a CONVENTION. A miss past the cap owes nothing, so a sample of it
    // that comes late is delivered one tag late; take_stats counts each
    // (`debt_capped=`) and the shutdown summary warns.
    const size_t depth = sub.get_actual_qos().depth();
    d.cap = depth > 0 ? depth : 1000;
  }
  return d;
}

void count_topic(const std::string & topic, uint64_t TopicTakeStats::* field, uint64_t n)
{
  std::lock_guard<std::mutex> lock(g_topic_mutex);
  g_topic_stats[topic].*field += n;
}

/// The take deadline of a trigger. In real
/// time reactor-c releases tag t once CLOCK_REALTIME reaches t, so a sample is
/// on time when the reader received it by t + T. `steady` is that instant on
/// steady_clock, so a take that runs late waits only for what is left of T,
/// and not at all past it; before this the wait was T from the take's own
/// start, and a take that ran late accepted a sample later than t + T without
/// a report. `realtime_ns` is t + T on CLOCK_REALTIME for the arrival check,
/// -1 when there is no tag deadline: in fast mode a tag says nothing about
/// physical time, and outside an LF program (the take tests) there is no tag,
/// so both keep T from the take's start.
struct TakeDeadline
{
  std::chrono::steady_clock::time_point steady;
  int64_t realtime_ns = -1;
};

TakeDeadline take_deadline(int64_t timeout_ms, std::chrono::steady_clock::time_point start)
{
  // Clamp so timeout_ms * 1e6 cannot overflow int64; 1e9 ms is 11.6 days,
  // longer than any take could sensibly wait.
  const int64_t ms = std::min<int64_t>(std::max<int64_t>(timeout_ms, 0), 1000000000);
  const auto tag = lf_current_tag();
  if (tag.first < 0 || lf_fast()) {
    return {start + std::chrono::milliseconds(ms), -1};
  }
  const int64_t due = tag.first + ms * 1000000;
  const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  return {start + std::chrono::nanoseconds(std::max<int64_t>(0, due - now)), due};
}

/// True when the reader received the sample after the tag deadline. A zero
/// received_timestamp (an rmw that does not fill it) is never judged late.
bool arrived_after(const rclcpp::MessageInfo & info, const TakeDeadline & dl)
{
  const int64_t rx = info.get_rmw_message_info().received_timestamp;
  return dl.realtime_ns >= 0 && rx > 0 && rx > dl.realtime_ns;
}

/// A sample received after its trigger's tag + T is lost by the owner's rule
/// (2026-11-02: a late message counts as lost and is reported): dropped,
/// counted in late_dropped and after_deadline, reported to the caller.
void drop_after_deadline(
  bool & warned, const std::string & topic, const rclcpp::MessageInfo & info,
  const TakeDeadline & dl, uint64_t & reported)
{
  reported += 1;
  g_late_dropped.fetch_add(1, std::memory_order_relaxed);
  g_after_deadline.fetch_add(1, std::memory_order_relaxed);
  count_topic(topic, &TopicTakeStats::late_dropped, 1);
  count_topic(topic, &TopicTakeStats::after_deadline, 1);
  if (!warned) {
    warned = true;
    const auto tag = lf_elapsed_tag();
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: topic '%s': the sample for the trigger at elapsed tag (%" PRId64
      " ns, %u) was received %" PRId64 " ns after tag + take_timeout_ms; dropped and reported "
      "as lost (further ones only in take_stats after_deadline)",
      topic.c_str(), tag.first, tag.second,
      info.get_rmw_message_info().received_timestamp - dl.realtime_ns);
  }
}

/// A take that polled for its sample and got it: the wait counters.
void record_wait(const std::string & topic, std::chrono::steady_clock::time_point start)
{
  const uint64_t waited_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start).count());
  g_waited.fetch_add(1, std::memory_order_relaxed);
  g_wait_total_ns.fetch_add(waited_ns, std::memory_order_relaxed);
  update_max(g_wait_max_ns, waited_ns);
  std::lock_guard<std::mutex> lock(g_topic_mutex);
  TopicTakeStats & ts = g_topic_stats[topic];
  ts.waited += 1;
  ts.wait_total_ns += waited_ns;
  ts.wait_max_ns = std::max(ts.wait_max_ns, waited_ns);
}

/// A fast-mode poll that ran to the take timeout: counted per
/// topic with its time, so a campaign sees where a federate stalled.
void record_poll_timeout(const std::string & topic, std::chrono::steady_clock::time_point start)
{
  const uint64_t ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - start).count());
  g_poll_timeouts.fetch_add(1, std::memory_order_relaxed);
  g_poll_timeout_ns.fetch_add(ns, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(g_topic_mutex);
  TopicTakeStats & ts = g_topic_stats[topic];
  ts.poll_timeouts += 1;
  ts.poll_timeout_ns += ns;
}

/// Reader overflow: a reliable KeepLast reader that is not
/// serviced replaces its oldest sample when full, and the middleware raises
/// no message_lost for the replaced ones (Fast DDS, probe: 300 publishes into
/// a KeepLast(10) reader, 10 taken, no event). brake_fusion's sensors
/// read /sim/state that way and a stalled sensor silently skipped plant
/// states (seen in a timeout-sweep run). Each writer numbers its
/// publications contiguously from 1, so a taken sample whose number is above
/// the writer's last taken one plus 1 counts the samples this reader was
/// never offered. The first sample of a writer only sets the baseline: a
/// volatile reader that joined late never saw the earlier ones, and that is
/// no loss (the tag path's LateJoinedWriterIsNoGap). A writer the reader
/// already saw before the first tag (take_note_start_writers) has no such
/// excuse: its numbering is expected from 1, so an overflow before its first
/// take is counted too. A content-filtered
/// reader is left alone: the samples its filter rejects leave the same holes
/// by design (the binding filters nothing; the tests inject losses that way).
/// The filter is read at every take (set_content_filter is public, a reaction
/// may call it; the flag is one bool in the rmw), and a change makes the
/// recorded numbers meaningless: each writer's next sample is a new baseline.
/// A filter turned on and off again between two takes is not seen (rclcpp
/// has no change hook); the holes it left then count as a loss. Known limit.
/// Returns the gap, counted in reader_lost and warned once per reader; the
/// caller reports it to the subscription's message_lost handler.
///
/// Reported once, whatever the reliability: on a best-effort
/// reader the gap also covers a sample lost on the wire, which DDS counts in
/// its own message_lost status, but rcllf never executes a physical QoS event
/// (the ingress takes and drops them), the corr take paths do not call
/// this function (a corr edge's loss is the MISSING ordinal), and so this gap
/// is the only report the user's handler gets for that sample. Restricting the
/// count to reliable readers would make a best-effort reader's loss silent
/// again: DDS raises nothing for a replacement in its history either way, and
/// its wire-loss event is dropped (measured with a probe run).
uint64_t note_sequence(
  SubDebt & d, rclcpp::SubscriptionBase & sub, const rclcpp::MessageInfo & info,
  const std::string & topic, uint64_t * first_lost = nullptr)
{
  const int cft = sub.is_cft_enabled() ? 1 : 0;
  if (cft != d.cft) {
    // -1 is "no take yet", not a change: the writers noted at start stay.
    if (d.cft != -1) {
      d.last_seq.clear();
    }
    d.cft = cft;
  }
  if (cft == 1) {
    return 0;
  }
  const auto & rmw = info.get_rmw_message_info();
  const uint64_t seq = rmw.publication_sequence_number;
  if (seq == RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED) {
    return 0;
  }
  tag_log::Gid gid{};
  std::copy_n(rmw.publisher_gid.data, gid.size(), gid.begin());
  const auto it = d.last_seq.find(gid);
  if (it == d.last_seq.end()) {
    d.last_seq.emplace(gid, seq);
    return 0;
  }
  const uint64_t last = it->second;
  it->second = seq;
  if (seq <= last + 1) {
    return 0;
  }
  const uint64_t gap = seq - last - 1;
  if (first_lost != nullptr) {
    *first_lost = last + 1;
  }
  g_reader_lost.fetch_add(gap, std::memory_order_relaxed);
  count_topic(topic, &TopicTakeStats::reader_lost, gap);
  if (!d.warned_gap) {
    d.warned_gap = true;
    const auto tag = lf_elapsed_tag();
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: topic '%s': %" PRIu64 " sample(s) of one writer never reached a take "
      "(publication %" PRIu64 " taken after %" PRIu64 "): replaced in the KeepLast depth %zu "
      "reader, or lost on the wire of a best-effort reader; at elapsed tag (%" PRId64
      " ns, %u); reported as lost once, here (further ones only in take_stats "
      "reader_lost)",
      topic.c_str(), gap, seq, last, sub.get_actual_qos().depth(), tag.first, tag.second);
  }
  return gap;
}

/// A gap of `gap` samples found at this tag credits the count
/// rule's next `gap` empty takes of the same tag (see SubDebt::lost_credit).
/// A credit does not outlive its tag: the replaced samples belong to the
/// burst that overflowed the reader, which the take of that tag drains.
/// A hole below the sample just taken is never filled (one writer's samples
/// arrive in order), and the oldest unfilled expectation comes first
/// (sequence order is tag order), so the hole settles open debt before it
/// credits this tag: an earlier miss the hole explains owes nothing more,
/// and a sample of this tag still on its way is never skipped on the
/// strength of an earlier tag's loss.
// Known limitation: SubDebt keeps one debt for the whole
// subscription, while the gap comes from one writer's sequence numbers. With
// several writers on one subscription, a gap of writer A settles debt that
// writer B may owe, and the rest is credited to this tag, so a loss can be
// attributed to the wrong writer. It is rare;
// fixing it needs per-writer debt.
void add_lost_credit(SubDebt & d, uint64_t gap)
{
  const uint64_t settled = std::min(gap, d.debt);
  d.debt -= settled;
  const uint64_t credit = gap - settled;
  if (credit == 0) {
    return;
  }
  const auto tag = lf_elapsed_tag();
  if (tag != d.credit_tag) {
    d.lost_credit = 0;
    d.credit_tag = tag;
  }
  d.lost_credit += credit;
}

/// An empty reader at a tag with a credit: one replaced sample
/// stands in for the take, counted as skipped, nothing owed and nothing polled.
bool use_lost_credit(SubDebt & d, const std::string & topic)
{
  if (d.lost_credit == 0 || d.credit_tag != lf_elapsed_tag()) {
    return false;
  }
  --d.lost_credit;
  g_skipped.fetch_add(1, std::memory_order_relaxed);
  count_topic(topic, &TopicTakeStats::skipped, 1);
  return true;
}

/// One owed sample came late and is dropped: report it.
void drop_late(
  SubDebt & d, rclcpp::SubscriptionBase & sub, const std::string & topic, uint64_t & reported)
{
  d.debt -= 1;
  d.lone_poll_spent = false;
  reported += 1;
  g_late_dropped.fetch_add(1, std::memory_order_relaxed);
  count_topic(topic, &TopicTakeStats::late_dropped, 1);
  if (!d.warned_drop) {
    d.warned_drop = true;
    const auto tag = lf_elapsed_tag();
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: topic '%s': dropped a late sample owed to an earlier trigger at "
      "elapsed tag (%" PRId64 " ns, %u), reported as lost (P17e; further drops only in "
      "take_stats)", topic.c_str(), tag.first, tag.second);
  }
  if (!d.warned_multi && sub.get_publisher_count() > 1) {
    // Arrival order across writers need not follow trigger order, so with
    // several writers the dropped sample may be an on-time one of another
    // writer (probe P17f).
    d.warned_multi = true;
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: topic '%s': late drop with %zu matched publishers; the oldest sample "
      "is dropped, which may belong to another writer than the owed one (P17f)",
      topic.c_str(), sub.get_publisher_count());
  }
}

/// Skip with debt, the count rule (probe P17e): take up to remaining + debt
/// samples, oldest first; the samples beyond `remaining` are the owed ones
/// that came late and are dropped, oldest first, and reported. A lone sample
/// held while debt is open cannot be told from an on-time one: it is delivered
/// and may be one tag late, and the debt stays open until two samples are held
/// at one take. No writer is ever charged a miss: an exact per-writer drop of
/// a lone late sample was tried and removed, because a lost sample followed by
/// one a tag ahead made it drop every later sample of that writer, a lasting
/// slip the count rule does not have. `remaining`
/// 0 means the trigger's write count is unknown (stub port counts callback
/// runs): such a miss owes nothing, since an empty trigger is
/// indistinguishable from a late payload (probe P17f).
///
/// A trigger with a known write count polls for its `remaining` samples up
/// to `timeout_ms` before it declares the miss, in every mode. In fast mode
/// the tag is granted as soon as every federate has reported it, so a sample
/// that is not in the reader yet is not late, it is still on its way.
/// In real time the same race exists on a count link: the
/// RTI grants the tag over TCP the moment the upstream federate completes
/// it, the sample itself is still on the UDP loopback, placed by the reader's
/// receive thread, and under host load the grant wins; a take that did not
/// wait dropped the counted sample as late and the program's input depended
/// on host scheduling. The one program never gets here with an empty reader
/// (Fast DDS delivers within one participant in the publish call), so only a
/// federation changes. An unknown count (`remaining` 0) waits only in a
/// federate, and a miss there is counted (`unknown_missed`) and warned.
/// In real time T runs from the trigger's tag, not from the take's start, and
/// a sample received after tag + T is dropped and reported as lost
/// (`take_deadline`).
/// With debt open, a lone sample may be the late owed one with the trigger's
/// own still on its way, so the poll also runs until `remaining + debt`
/// samples are held before it delivers a lone one;
/// that poll runs once per debt change, so a sample that is lost for good
/// costs one timeout, not one per trigger.
bool take_skip(
  rclcpp::SubscriptionBase & sub, void * message, rclcpp::MessageInfo & message_info,
  const std::string & topic, uint64_t remaining, uint64_t & reported, int64_t timeout_ms)
{
  SubDebt & d = debt_of(sub);
  const bool known = remaining > 0;
  remaining = std::max<uint64_t>(remaining, 1);
  // Known count: fast or real time alike. Unknown count:
  // only in a federate. There a sample written at the tag can
  // still be on the wire, and a take that did not wait made the program's
  // input depend on host scheduling. In one process Fast DDS delivers in the
  // publish call, so an empty reader is a trigger that wrote nothing, and a
  // wait would only cost T for every such trigger.
  const bool wait = known || corr_rt::federated();
  const auto start = std::chrono::steady_clock::now();
  const TakeDeadline dl = take_deadline(timeout_ms, start);
  const auto deadline = dl.steady;
  bool waited = false;
  auto deliver_or_drop = [&](const rclcpp::MessageInfo & info) {
      if (arrived_after(info, dl)) {
        // Its own trigger's sample, too late: lost and reported, and the
        // owed sample is settled, so no debt.
        drop_after_deadline(d.warned_after, topic, info, dl, reported);
        g_skipped.fetch_add(1, std::memory_order_relaxed);
        count_topic(topic, &TopicTakeStats::skipped, 1);
        return false;
      }
      if (waited) {
        record_wait(topic, start);
      } else {
        g_first_try.fetch_add(1, std::memory_order_relaxed);
      }
      return true;
    };
  // A fresh sample's gap is the reader's loss; at a known count it
  // also credits this tag's empty takes.
  auto note_gap = [&](const rclcpp::MessageInfo & info) {
      const uint64_t gap = note_sequence(d, sub, info, topic);
      if (gap != 0 && known) {
        add_lost_credit(d, gap);
      }
      return gap;
    };
  while (true) {
    const bool slow = d.debt > 0 || !d.held.empty();
    if (!slow) {
      // Nothing owed and nothing held: the historical single take.
      if (sub.take_type_erased(message, message_info)) {
        reported += note_gap(message_info);
        return deliver_or_drop(message_info);
      }
    } else {
      // Take up to remaining + debt samples, oldest first.
      while (d.held.size() < remaining + d.debt) {
        HeldSample h{std::make_shared<rclcpp::SerializedMessage>(), rclcpp::MessageInfo()};
        if (!sub.take_serialized(*h.data, h.info)) {
          break;
        }
        reported += note_gap(h.info);
        d.held.push_back(std::move(h));
      }
      // More than `remaining` came out: the oldest extras are the owed samples
      // that came late.
      while (d.debt > 0 && d.held.size() > remaining) {
        d.held.pop_front();
        drop_late(d, sub, topic, reported);
      }
      if (known && d.debt > 0 && !d.held.empty() && d.held.size() < remaining + d.debt &&
        !d.lone_poll_spent)
      {
        if (std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(kPollInterval);
          waited = true;
          continue;
        }
        d.lone_poll_spent = true;
        record_poll_timeout(topic, start);
      }
      if (d.debt > 0 && !d.held.empty() && !d.warned_lone) {
        d.warned_lone = true;
        RCLCPP_WARN(
          rclcpp::get_logger("ros2_lf_binding"),
          "take_with_mode: topic '%s': a sample is owed but the one taken cannot be told from an "
          "on-time one; it is delivered and may be one tag late (further cases not "
          "logged)", topic.c_str());
      }
      if (!d.held.empty()) {
        HeldSample h = std::move(d.held.front());
        d.held.pop_front();
        if (arrived_after(h.info, dl)) {
          return deliver_or_drop(h.info);
        }
        const rmw_ret_t ret = rmw_deserialize(
          &h.data->get_rcl_serialized_message(), &sub.get_message_type_support_handle(), message);
        if (ret == RMW_RET_OK) {
          message_info = h.info;
          return deliver_or_drop(h.info);
        }
        rmw_reset_error();
        RCLCPP_ERROR(
          rclcpp::get_logger("ros2_lf_binding"),
          "take_with_mode: topic '%s': cannot deserialize a held sample; it is lost",
          topic.c_str());
      }
    }
    if (known && use_lost_credit(d, topic)) {
      // The reader replaced this sample before the take; it was
      // declared lost by the gap, so the trigger owes nothing and no poll runs.
      return false;
    }
    if (!wait || std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(kPollInterval);
    waited = true;
  }
  if (wait) {
    record_poll_timeout(topic, start);
  }
  g_skipped.fetch_add(1, std::memory_order_relaxed);
  count_topic(topic, &TopicTakeStats::skipped, 1);
  if (!known) {
    if (wait) {
      // A federate's unknown-count trigger with nothing by tag + T: it may
      // have written nothing, or its sample is lost or late. Neither can be
      // told apart, so it owes no debt, but it is counted and warned, never
      // silent.
      g_unknown_missed.fetch_add(1, std::memory_order_relaxed);
      count_topic(topic, &TopicTakeStats::unknown_missed, 1);
      if (!d.warned_unknown) {
        d.warned_unknown = true;
        const auto tag = lf_elapsed_tag();
        RCLCPP_WARN(
          rclcpp::get_logger("ros2_lf_binding"),
          "take_with_mode: topic '%s': no sample by tag + take_timeout_ms for a trigger with an "
          "unknown write count at elapsed tag (%" PRId64 " ns, %u); it wrote nothing, or its "
          "sample is lost or late, which cannot be told apart (RCLLF_COUNT_PUBLISHES=0 or "
          "ros2_lf_invoke_sub; further ones only in take_stats unknown_missed)",
          topic.c_str(), tag.first, tag.second);
      }
    }
    return false;
  }
  if (d.debt < d.cap) {
    ++d.debt;
    d.lone_poll_spent = false;
  } else {
    // The miss cannot be owed, so its sample, if it comes late,
    // is taken as a fresh one. Counted, never silent.
    g_debt_capped.fetch_add(1, std::memory_order_relaxed);
    count_topic(topic, &TopicTakeStats::debt_capped, 1);
  }
  if (!d.warned_miss) {
    d.warned_miss = true;
    const auto tag = lf_elapsed_tag();
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: topic '%s': no sample for a trigger at elapsed tag (%" PRId64 " ns, %u) "
      "(take_mode=skip); the owed sample is dropped and reported if it comes late (P17e; "
      "further misses only in take_stats)", topic.c_str(), tag.first, tag.second);
  }
  return false;
}

/// A drop-by-tag take served by the count rule (its trigger's
/// records are not all here, or its write count is unknown). The count rule
/// may deliver a late sample one tag late, so every such take is counted for
/// the take_stats summary, which warns at shutdown.
void note_count_rule(const std::string & topic)
{
  g_count_rule.fetch_add(1, std::memory_order_relaxed);
  count_topic(topic, &TopicTakeStats::count_rule, 1);
}

/// Drop-by-tag state of one subscription reaction. It is keyed
/// by reaction, not subscription, so the writes of triggers that fire before a
/// lifecycle node makes its subscription, or while it is inactive, are used up
/// all the same. One reaction never runs on two workers at once, so only the
/// map is locked.
struct TagReader
{
  uint64_t used = 0;  // writes owned by earlier triggers: the next trigger's first record
  bool open = false;  // a trigger's invokes are under way
  bool count_rule = false;  // the open trigger is not served by tag
  // Count rule from now on: a trigger's writes were missing here (a
  // cross-federate link), or a trigger's write count was unknown, so the
  // records no longer line up with this reader's triggers.
  bool count_only = false;
  std::string topic;  // for take_stats, set by the first take
  uint64_t begin = 0;  // the open trigger owns records [begin, end)
  uint64_t end = 0;
  std::deque<std::pair<uint64_t, HeldSample>> held;  // samples of this or a later trigger
  // Samples on a remote topic whose writer's events have not arrived
  // yet: above the writer's last seen sequence number, of an
  // announced writer not seen yet, or of a writer not announced while a
  // writer federate has not announced yet.
  std::deque<HeldSample> ahead;
  bool warned_miss = false;
  bool warned_drop = false;
  bool warned_untagged = false;
  bool warned_plain = false;
  bool warned_no_import = false;
  bool warned_count_rule = false;
  // Records of this reader's triggers whose sample the reader
  // replaced before a take (a gap). Each stands in for one empty take
  // of its trigger: declared lost by the gap, no poll, no miss WARN.
  std::set<uint64_t> lost;
};

std::mutex g_tag_mutex;
std::map<int, TagReader> g_tag;

TagReader & tag_reader(int rid)
{
  std::lock_guard<std::mutex> lock(g_tag_mutex);
  return g_tag[rid];
}

/// Open the trigger of an invoke with `remaining` writes left, if none is open.
void tag_open(TagReader & r, uint64_t remaining)
{
  if (!r.open) {
    r.open = true;
    r.count_rule = false;
    r.begin = r.used;
    r.end = r.used + remaining;
    r.used = r.end;
  }
}

/// An invoke of a trigger on the count rule: the oldest sample an earlier take
/// held for it is delivered, as the count rule takes the oldest
/// sample. Held for it: a sample of a record of the open trigger, or
/// any once the reader is on the count rule for good. Without one the caller
/// takes with the count rule (`count_rule`).
void take_held(TagReader & r, TagTake & out)
{
  for (auto it = r.held.begin(); it != r.held.end(); ++it) {
    if (r.count_only || it->first < r.end) {
      out.deliver.push_back({std::move(it->second.data), it->second.info});
      r.held.erase(it);
      g_first_try.fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }
  if (r.count_only && !r.ahead.empty()) {
    HeldSample & h = r.ahead.front();
    out.deliver.push_back({std::move(h.data), h.info});
    r.ahead.pop_front();
    g_unsettled.fetch_sub(1, std::memory_order_relaxed);
    g_first_try.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  out.count_rule = true;
}

/// The skip-debt counters for a take_stats line, each empty while it is zero
/// so a program without late samples logs the line as before.
std::string debt_suffix(
  uint64_t late_dropped, uint64_t untagged = 0, uint64_t unsettled = 0, uint64_t reader_lost = 0)
{
  std::string out;
  char buf[48];
  if (late_dropped != 0) {
    std::snprintf(buf, sizeof(buf), " late_dropped=%" PRIu64, late_dropped);
    out += buf;
  }
  if (reader_lost != 0) {
    std::snprintf(buf, sizeof(buf), " reader_lost=%" PRIu64, reader_lost);
    out += buf;
  }
  if (untagged != 0) {
    std::snprintf(buf, sizeof(buf), " untagged=%" PRIu64, untagged);
    out += buf;
  }
  if (unsettled != 0) {
    std::snprintf(buf, sizeof(buf), " unsettled=%" PRIu64, unsettled);
    out += buf;
  }
  return out;
}

/// The tag-deadline counters for a take_stats line, each empty
/// while zero, so a run without late or unknown-count misses logs as before.
std::string deadline_suffix(uint64_t after_deadline, uint64_t unknown_missed)
{
  std::string out;
  char buf[48];
  if (after_deadline != 0) {
    std::snprintf(buf, sizeof(buf), " after_deadline=%" PRIu64, after_deadline);
    out += buf;
  }
  if (unknown_missed != 0) {
    std::snprintf(buf, sizeof(buf), " unknown_missed=%" PRIu64, unknown_missed);
    out += buf;
  }
  return out;
}

/// The rule-change counters for a take_stats line, each empty
/// while zero, so a run that never left drop-by-tag or the debt logs as before.
std::string rule_suffix(uint64_t count_rule, uint64_t debt_capped)
{
  std::string out;
  char buf[48];
  if (count_rule != 0) {
    std::snprintf(buf, sizeof(buf), " count_rule=%" PRIu64, count_rule);
    out += buf;
  }
  if (debt_capped != 0) {
    std::snprintf(buf, sizeof(buf), " debt_capped=%" PRIu64, debt_capped);
    out += buf;
  }
  return out;
}

/// The fast-mode poll timeouts for a take_stats line, empty while
/// zero. Named apart from `timeouts`/`wait_timeouts`, which count block-mode
/// timeouts and which benchmark parsers read.
std::string poll_suffix(uint64_t poll_timeouts, uint64_t poll_timeout_ns)
{
  if (poll_timeouts == 0) {
    return {};
  }
  char buf[80];
  std::snprintf(
    buf, sizeof(buf), " poll_timeouts=%" PRIu64 " poll_timeout_us=%" PRIu64, poll_timeouts,
    poll_timeout_ns / 1000);
  return buf;
}

}  // namespace

TakeMode parse_take_mode(const char * val)
{
  if (!val || val[0] == '\0') {
    return TakeMode::Skip;
  }
  const std::string s(val);
  if (s == "skip") {
    return TakeMode::Skip;
  }
  if (s == "block") {
    return TakeMode::Block;
  }
  RCLCPP_WARN(
    rclcpp::get_logger("ros2_lf_binding"),
    "Unknown ROS2_LF_TAKE_MODE value '%s'; using 'skip'", val);
  return TakeMode::Skip;
}

int64_t parse_take_timeout_ms(const char * val, const char * var)
{
  if (!val || val[0] == '\0') {
    return 100;
  }
  char * end = nullptr;
  const int64_t parsed = std::strtoll(val, &end, 10);
  if (end == val || *end != '\0' || parsed < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "Invalid %s value '%s'; using maxwait 100 ms", var, val);
    return 100;
  }
  return parsed;
}

std::optional<TakeMode> take_mode_env()
{
  const char * val = std::getenv("ROS2_LF_TAKE_MODE");
  if (!val || val[0] == '\0') {
    return std::nullopt;
  }
  return parse_take_mode(val);
}

MaxwaitEnv maxwait_env()
{
  const char * primary = std::getenv(kMaxwaitEnv);
  const char * alias = std::getenv(kMaxwaitEnvAlias);
  const bool has_primary = primary != nullptr && primary[0] != '\0';
  const bool has_alias = alias != nullptr && alias[0] != '\0';
  MaxwaitEnv out;
  if (!has_primary && !has_alias) {
    return out;
  }
  const std::string primary_str = std::string(kMaxwaitEnv) + "=";
  const std::string alias_str = std::string(kMaxwaitEnvAlias) + "=";
  if (has_primary && has_alias) {
    const int64_t a = parse_take_timeout_ms(primary, kMaxwaitEnv);
    const int64_t b = parse_take_timeout_ms(alias, kMaxwaitEnvAlias);
    if (a != b) {
      out.conflict = primary_str + "'" + primary + "' and its alias " + alias_str + "'" +
        alias + "' disagree: the maxwait override is refused, so every subscription keeps "
        "its configured maxwait_ms; set only " + kMaxwaitEnv;
      return out;
    }
    out.value = a;
    out.source = primary_str + std::to_string(a) + " (and its alias " + alias_str +
      std::to_string(b) + ")";
    return out;
  }
  if (has_primary) {
    out.value = parse_take_timeout_ms(primary, kMaxwaitEnv);
    out.source = primary_str + std::to_string(*out.value);
    return out;
  }
  out.value = parse_take_timeout_ms(alias, kMaxwaitEnvAlias);
  out.source = alias_str + std::to_string(*out.value) + " (alias of " + kMaxwaitEnv + ")";
  return out;
}

std::optional<int64_t> take_timeout_ms_env()
{
  // Called on every take: the same rule as maxwait_env() without building
  // its message strings.
  const char * primary = std::getenv(kMaxwaitEnv);
  const char * alias = std::getenv(kMaxwaitEnvAlias);
  const bool has_primary = primary != nullptr && primary[0] != '\0';
  const bool has_alias = alias != nullptr && alias[0] != '\0';
  if (!has_primary && !has_alias) {
    return std::nullopt;
  }
  if (!has_alias) {
    return parse_take_timeout_ms(primary, kMaxwaitEnv);
  }
  const int64_t b = parse_take_timeout_ms(alias, kMaxwaitEnvAlias);
  if (!has_primary) {
    return b;
  }
  const int64_t a = parse_take_timeout_ms(primary, kMaxwaitEnv);
  if (a != b) {
    return std::nullopt;
  }
  return a;
}

bool take_with_mode(
  rclcpp::SubscriptionBase & sub, void * message, rclcpp::MessageInfo & message_info,
  TakeMode mode, int64_t timeout_ms, const std::string & topic, int remaining,
  uint64_t * reported)
{
  if (mode == TakeMode::Skip) {
    uint64_t lost = 0;
    const bool taken = take_skip(
      sub, message, message_info, topic, static_cast<uint64_t>(std::max(remaining, 0)), lost,
      timeout_ms);
    if (reported != nullptr) {
      *reported = lost;
    }
    return taken;
  }
  if (reported != nullptr) {
    *reported = 0;
  }
  // Block mode reports reader overflow the same way; the pointer may
  // be null, so the gap is also kept in take_stats.
  SubDebt & d = debt_of(sub);
  auto note = [&]() {
      const uint64_t gap = note_sequence(d, sub, message_info, topic);
      if (reported != nullptr) {
        *reported += gap;
      }
    };

  // T runs from the trigger's tag, and a sample received after tag + T is
  // dropped and reported as lost (see take_deadline).
  const auto start = std::chrono::steady_clock::now();
  const TakeDeadline dl = take_deadline(timeout_ms, start);
  const auto deadline = dl.steady;
  auto late = [&]() {
      if (!arrived_after(message_info, dl)) {
        return false;
      }
      uint64_t lost = 0;
      drop_after_deadline(d.warned_after, topic, message_info, dl, lost);
      if (reported != nullptr) {
        *reported += lost;
      }
      g_timeouts.fetch_add(1, std::memory_order_relaxed);
      count_topic(topic, &TopicTakeStats::timeouts, 1);
      return true;
    };
  if (sub.take_type_erased(message, message_info)) {
    note();
    if (late()) {
      return false;
    }
    g_first_try.fetch_add(1, std::memory_order_relaxed);
    return true;
  }

  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(kPollInterval);
    if (sub.take_type_erased(message, message_info)) {
      note();
      if (late()) {
        return false;
      }
      const uint64_t waited_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - start).count());
      g_waited.fetch_add(1, std::memory_order_relaxed);
      g_wait_total_ns.fetch_add(waited_ns, std::memory_order_relaxed);
      update_max(g_wait_max_ns, waited_ns);
      std::lock_guard<std::mutex> lock(g_topic_mutex);
      TopicTakeStats & ts = g_topic_stats[topic];
      ts.waited += 1;
      ts.wait_total_ns += waited_ns;
      ts.wait_max_ns = std::max(ts.wait_max_ns, waited_ns);
      return true;
    }
  }

  g_timeouts.fetch_add(1, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(g_topic_mutex);
    g_topic_stats[topic].timeouts += 1;
  }
  RCLCPP_WARN(
    rclcpp::get_logger("ros2_lf_binding"),
    "take_with_mode: topic '%s': no message after %" PRId64 " ms block; skipping reaction",
    topic.c_str(), timeout_ms);
  return false;
}

TagTake take_by_tag(
  int rid, rclcpp::SubscriptionBase & sub, TakeMode mode, int64_t timeout_ms,
  const std::string & topic, int remaining)
{
  TagTake out;
  TagReader & r = tag_reader(rid);
  r.topic = topic;
  if (remaining <= 0) {
    // Unknown write count (ros2_lf_invoke_sub): the records this trigger owns
    // are unknown, so the reader can no longer find its next trigger's first
    // record and stays on the count rule.
    r.count_only = true;
  }
  if (r.count_only) {
    r.open = false;
    take_held(r, out);
    note_count_rule(topic);
    return out;
  }
  const std::string log_topic = sub.get_topic_name();
  const bool closing = remaining <= 1;
  // A take on a crossing topic before its link's first import.
  // Generated systems never get here: the take runs only in the
  // subscription reaction that the import reaction triggers (lf_emitter.py
  // _emit_xtag_reactions). The trigger keeps its records' place, so later
  // triggers stay aligned with the writer's; every sample taken now is held
  // until an import tells its writer, and one of this trigger is then late.
  const bool before_import = tag_log::remote(log_topic) && !tag_log::imported(log_topic);
  if (before_import && !r.warned_no_import) {
    r.warned_no_import = true;
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: topic '%s' (RID %d): taken before the cross-federate take link's first "
      "message was imported, so no writer is known yet; the samples are held until an import "
      "and this trigger's own are dropped as late", topic.c_str(), rid);
  }
  if (!r.open) {
    tag_open(r, static_cast<uint64_t>(remaining));
    if (!before_import && !tag_log::exact(log_topic, r.begin, r.end)) {
      // The writes are not all recorded here: a cross-federate link whose
      // events did not all arrive (a link that carries only the count, or an
      // gap in the link's events), or a write had no GID.
      r.count_rule = true;
      if (tag_log::end(log_topic) < r.end || tag_log::broken(log_topic)) {
        // Writes recorded in another process: local records no longer line up
        // with this reader's triggers, so it stays on the count rule.
        r.count_only = true;
      }
      if (!r.warned_count_rule) {
        r.warned_count_rule = true;
        RCLCPP_WARN(
          rclcpp::get_logger("ros2_lf_binding"),
          "take_with_mode: topic '%s' (RID %d): the writes of this trigger are not all "
          "recorded in this process (a cross-federate link carries only their count or lost "
          "events, or a writer has no GID), so the count rule applies and a late sample may be "
          "delivered one tag late",
          topic.c_str(), rid);
      }
    }
  }
  if (r.count_rule) {
    take_held(r, out);
    r.open = !closing && !r.count_only;
    note_count_rule(topic);
    return out;
  }

  bool got = false;
  auto drop = [&]() {
      ++out.lost;
      g_late_dropped.fetch_add(1, std::memory_order_relaxed);
      count_topic(topic, &TopicTakeStats::late_dropped, 1);
      if (!r.warned_drop) {
        r.warned_drop = true;
        const auto tag = lf_elapsed_tag();
        RCLCPP_WARN(
          rclcpp::get_logger("ros2_lf_binding"),
          "take_with_mode: topic '%s': dropped a late sample of an earlier trigger at elapsed "
          "tag (%" PRId64 " ns, %u), reported as lost (further drops only in "
          "take_stats)", topic.c_str(), tag.first, tag.second);
      }
    };
  // Place one sample of record `idx`: deliver, drop, or keep for later.
  auto place = [&](uint64_t idx, HeldSample && h) {
      if (idx < r.begin) {
        drop();
      } else if (idx < r.end && (!got || closing)) {
        got = true;
        out.deliver.push_back({std::move(h.data), h.info});
      } else {
        r.held.emplace_back(idx, std::move(h));
      }
    };
  const bool remote = tag_log::remote(log_topic);
  // A taken sample: by its record, or dropped as forgotten, or kept while its
  // writer's events may still be on the way, or else untagged. `was_held`:
  // it comes from `ahead`.
  auto classify = [&](HeldSample && h, bool was_held) {
      const auto & rmw = h.info.get_rmw_message_info();
      tag_log::Gid gid{};
      std::copy_n(rmw.publisher_gid.data, gid.size(), gid.begin());
      const uint64_t seq = rmw.publication_sequence_number;
      if (was_held) {
        g_unsettled.fetch_sub(1, std::memory_order_relaxed);
      } else {
        // Fresh from the reader: samples the reader replaced before this take
        // are a loss no record will ever settle. Their records of this
        // or a later trigger are each one empty take that polls nothing.
        uint64_t first_lost = 0;
        const uint64_t gap = note_sequence(debt_of(sub), sub, h.info, topic, &first_lost);
        out.lost += gap;
        for (uint64_t k = 0; k < std::min<uint64_t>(gap, kLostLookup); ++k) {
          const auto li = tag_log::find(log_topic, gid, first_lost + k);
          if (li && *li >= r.begin) {
            r.lost.insert(*li);
          }
        }
      }
      const auto idx = tag_log::find(log_topic, gid, seq);
      if (idx) {
        place(*idx, std::move(h));
        return;
      }
      if (tag_log::old(log_topic, gid, seq)) {
        drop();  // its record is forgotten: a sample of an earlier trigger
        return;
      }
      const auto seen = tag_log::seen(log_topic, gid);
      // Sequence numbers are 64-bit and never wrap (2^64 publishes of one writer).
      bool hold = seen && seq > *seen;
      tag_log::Writer writer = tag_log::Writer::kGated;
      if (remote && !hold) {
        // The writer federate announces its publishers with every message, so
        // the writer is told by GID: an announced
        // one not seen yet has its events on the way, as has any before the
        // first import or while an expected publisher is not announced yet.
        writer = tag_log::writer(log_topic, gid);
        hold = writer == tag_log::Writer::kUnknown ||
          (writer == tag_log::Writer::kGated && !seen);
      }
      if (hold) {
        // A remote writer's events come with its next trigger.
        r.ahead.push_back(std::move(h));
        g_unsettled.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      if (writer == tag_log::Writer::kPlain) {
        tag_log::note_plain(log_topic, gid);
        if (!r.warned_plain) {
          r.warned_plain = true;
          RCLCPP_WARN(
            rclcpp::get_logger("ros2_lf_binding"),
            "take_with_mode: topic '%s': a sample's writer is no publisher the writer "
            "federate announced (a writer outside rcllf); it is delivered as untagged",
            topic.c_str());
        }
      }
      // No record: published outside any reaction with no open claim.
      out.deliver.push_back({std::move(h.data), h.info});
      g_untagged.fetch_add(1, std::memory_order_relaxed);
      count_topic(topic, &TopicTakeStats::untagged, 1);
      if (!r.warned_untagged) {
        r.warned_untagged = true;
        RCLCPP_WARN(
          rclcpp::get_logger("ros2_lf_binding"),
          "take_with_mode: topic '%s': delivered a sample published outside any reaction and "
          "not claimed with ros2_lf_claim_publish, in addition to the tag's own samples "
          "(untagged; further ones only in take_stats)", topic.c_str());
      }
    };
  std::deque<std::pair<uint64_t, HeldSample>> held;
  held.swap(r.held);
  for (auto & [idx, h] : held) {
    place(idx, std::move(h));
  }
  std::deque<HeldSample> ahead;
  ahead.swap(r.ahead);
  for (auto & h : ahead) {
    classify(std::move(h), true);
  }

  const auto start = std::chrono::steady_clock::now();
  // T runs from the trigger's tag (see take_deadline). A
  // sample of a recorded write is judged by its record, not by arrival.
  const auto deadline = take_deadline(timeout_ms, start).steady;
  bool waited = false;
  bool credited = false;
  // An empty reader for a recorded write is a sample on its way, not a miss,
  // in every mode: in fast mode the tag is granted before the sample lands;
  // in real time a tag link's record arrives over the LF link
  // ahead of the DDS sample and under host load the grant wins that race.
  // Skip mode polls up to the take timeout like block mode and
  // then declares the miss. A record whose sample the reader replaced
  // is no sample on its way: the take ends at once, declared by the gap.
  while (!got || closing) {
    HeldSample h{std::make_shared<rclcpp::SerializedMessage>(), rclcpp::MessageInfo()};
    if (!sub.take_serialized(*h.data, h.info)) {
      if (!got && !r.lost.empty()) {
        r.lost.erase(r.lost.begin(), r.lost.lower_bound(r.begin));
        const auto li = r.lost.begin();
        if (li != r.lost.end() && *li < r.end) {
          r.lost.erase(li);
          credited = true;
          break;
        }
      }
      if (got || std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(kPollInterval);
      waited = true;
      continue;
    }
    classify(std::move(h), false);
  }
  if (closing) {
    r.open = false;
  }

  if (got && waited) {
    const uint64_t waited_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - start).count());
    g_waited.fetch_add(1, std::memory_order_relaxed);
    g_wait_total_ns.fetch_add(waited_ns, std::memory_order_relaxed);
    update_max(g_wait_max_ns, waited_ns);
    std::lock_guard<std::mutex> lock(g_topic_mutex);
    TopicTakeStats & ts = g_topic_stats[topic];
    ts.waited += 1;
    ts.wait_total_ns += waited_ns;
    ts.wait_max_ns = std::max(ts.wait_max_ns, waited_ns);
  } else if (got) {
    g_first_try.fetch_add(1, std::memory_order_relaxed);
  } else if (credited) {
    // Its sample was replaced in the reader and declared lost by
    // the gap WARN; one skipped take, no poll timeout, no miss WARN.
    g_skipped.fetch_add(1, std::memory_order_relaxed);
    count_topic(topic, &TopicTakeStats::skipped, 1);
  } else if (mode == TakeMode::Skip) {
    record_poll_timeout(topic, start);
    g_skipped.fetch_add(1, std::memory_order_relaxed);
    count_topic(topic, &TopicTakeStats::skipped, 1);
    if (!r.warned_miss) {
      r.warned_miss = true;
      const auto tag = lf_elapsed_tag();
      RCLCPP_WARN(
        rclcpp::get_logger("ros2_lf_binding"),
        "take_with_mode: topic '%s': no sample for a trigger at elapsed tag (%" PRId64 " ns, %u) "
        "(take_mode=skip); the sample is dropped and reported if it comes late ("
        "further misses only in take_stats)", topic.c_str(), tag.first, tag.second);
    }
  } else {
    g_timeouts.fetch_add(1, std::memory_order_relaxed);
    count_topic(topic, &TopicTakeStats::timeouts, 1);
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: topic '%s': no message after %" PRId64 " ms block; skipping reaction; "
      "the sample is dropped and reported if it comes late",
      topic.c_str(), timeout_ms);
  }
  return out;
}

uint64_t tag_pass(int rid, int remaining)
{
  if (remaining <= 0) {
    return 0;
  }
  TagReader & r = tag_reader(rid);
  if (r.count_only) {
    return 0;  // no records to use up; held samples go to the count rule
  }
  tag_open(r, static_cast<uint64_t>(remaining));
  if (remaining > 1) {
    return 0;
  }
  r.open = false;
  // Samples of this trigger already taken by an earlier one: nobody runs them.
  uint64_t dropped = 0;
  for (auto it = r.held.begin(); it != r.held.end(); ) {
    if (it->first < r.end) {
      it = r.held.erase(it);
      ++dropped;
    } else {
      ++it;
    }
  }
  if (dropped > 0) {
    g_late_dropped.fetch_add(dropped, std::memory_order_relaxed);
    if (!r.topic.empty()) {
      count_topic(r.topic, &TopicTakeStats::late_dropped, dropped);
    }
  }
  return dropped;
}

bool deserialize_taken(
  rclcpp::SubscriptionBase & sub, const TakenSample & sample, void * message)
{
  const rmw_ret_t ret = rmw_deserialize(
    &sample.data->get_rcl_serialized_message(), &sub.get_message_type_support_handle(), message);
  if (ret == RMW_RET_OK) {
    return true;
  }
  rmw_reset_error();
  RCLCPP_ERROR(
    rclcpp::get_logger("ros2_lf_binding"),
    "take_with_mode: topic '%s': cannot deserialize a taken sample; it is lost",
    sub.get_topic_name());
  return false;
}

TakeStatsSnapshot take_stats_snapshot()
{
  TakeStatsSnapshot snap;
  snap.first_try = g_first_try.load(std::memory_order_relaxed);
  snap.skipped = g_skipped.load(std::memory_order_relaxed);
  snap.waited = g_waited.load(std::memory_order_relaxed);
  snap.wait_total_ns = g_wait_total_ns.load(std::memory_order_relaxed);
  snap.wait_max_ns = g_wait_max_ns.load(std::memory_order_relaxed);
  snap.timeouts = g_timeouts.load(std::memory_order_relaxed);
  snap.late_dropped = g_late_dropped.load(std::memory_order_relaxed);
  snap.untagged = g_untagged.load(std::memory_order_relaxed);
  snap.unsettled = g_unsettled.load(std::memory_order_relaxed);
  snap.poll_timeouts = g_poll_timeouts.load(std::memory_order_relaxed);
  snap.poll_timeout_ns = g_poll_timeout_ns.load(std::memory_order_relaxed);
  snap.reader_lost = g_reader_lost.load(std::memory_order_relaxed);
  snap.after_deadline = g_after_deadline.load(std::memory_order_relaxed);
  snap.unknown_missed = g_unknown_missed.load(std::memory_order_relaxed);
  snap.count_rule = g_count_rule.load(std::memory_order_relaxed);
  snap.debt_capped = g_debt_capped.load(std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(g_topic_mutex);
  snap.topics = g_topic_stats;
  return snap;
}

void take_stats_log_summary(const char * reason)
{
  const TakeStatsSnapshot snap = take_stats_snapshot();
  // Mode/timeout are per-subscription now (policy_table.hpp); the summary can
  // only report the process-wide env override, or "per-sub" when none is set.
  const auto env_mode = take_mode_env();
  const auto env_timeout = take_timeout_ms_env();
  const std::string timeout_str =
    env_timeout ? (std::to_string(*env_timeout) + "(env)") : std::string("per-sub");
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding"),
    "ros2_lf_take_stats: reason=%s mode=%s timeout_ms=%s"
    " sub_first_try=%" PRIu64 " sub_skipped=%" PRIu64 " sub_waited=%" PRIu64
    " wait_total_us=%" PRIu64 " wait_max_us=%" PRIu64 " wait_timeouts=%" PRIu64 "%s",
    reason,
    env_mode ? (*env_mode == TakeMode::Block ? "block(env)" : "skip(env)") : "per-sub",
    timeout_str.c_str(),
    snap.first_try, snap.skipped, snap.waited,
    snap.wait_total_ns / 1000, snap.wait_max_ns / 1000, snap.timeouts,
    (debt_suffix(snap.late_dropped, snap.untagged, snap.unsettled, snap.reader_lost) +
    deadline_suffix(snap.after_deadline, snap.unknown_missed) +
    rule_suffix(snap.count_rule, snap.debt_capped) +
    poll_suffix(snap.poll_timeouts, snap.poll_timeout_ns)).c_str());
  if ((snap.count_rule != 0 || snap.debt_capped != 0) && std::strcmp(reason, "shutdown") == 0) {
    // The run left drop-by-tag delivery or its debt for part of
    // its takes, so a late sample may have been delivered one tag late.
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: this run changed delivery rules: %" PRIu64 " take(s) were served by the "
      "count rule instead of by tag and %" PRIu64 " miss(es) were not owed because the reader's "
      "debt was at its cap, so a late sample may have been delivered one tag late ("
      "per topic in ros2_lf_take_stats_topic count_rule= and debt_capped=)",
      snap.count_rule, snap.debt_capped);
  }
  if (snap.unsettled != 0 && std::strcmp(reason, "shutdown") == 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("ros2_lf_binding"),
      "take_with_mode: %" PRIu64 " sample(s) taken ahead of their writer's tag log events "
      "were never settled by an import and not delivered at shutdown",
      snap.unsettled);
  }
  for (const auto & [topic, ts] : snap.topics) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_take_stats_topic: topic=%s skipped=%" PRIu64 " waited=%" PRIu64
      " wait_total_us=%" PRIu64 " wait_max_us=%" PRIu64 " timeouts=%" PRIu64 "%s",
      topic.c_str(), ts.skipped, ts.waited,
      ts.wait_total_ns / 1000, ts.wait_max_ns / 1000, ts.timeouts,
      (debt_suffix(ts.late_dropped, ts.untagged, 0, ts.reader_lost) +
      deadline_suffix(ts.after_deadline, ts.unknown_missed) +
      rule_suffix(ts.count_rule, ts.debt_capped) +
      poll_suffix(ts.poll_timeouts, ts.poll_timeout_ns)).c_str());
  }
}

void take_stats_maybe_periodic_log()
{
  static std::atomic<int64_t> last_log_ns{0};
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  static const int64_t interval_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(kPeriodicLogInterval).count();
  int64_t last = last_log_ns.load(std::memory_order_relaxed);
  if (now_ns - last < interval_ns) {
    return;
  }
  if (last_log_ns.compare_exchange_strong(last, now_ns, std::memory_order_relaxed)) {
    take_stats_log_summary("periodic");
  }
}

void take_note_start_writers(
  rclcpp::SubscriptionBase & sub, const std::vector<std::array<uint8_t, 16>> & gids)
{
  SubDebt & d = debt_of(sub);
  for (const auto & gid : gids) {
    // 0: nothing taken yet, so publication 1 is the next one expected. A
    // writer already taken from keeps its recorded number.
    d.last_seq.emplace(gid, 0);
  }
}

void take_stats_reset()
{
  g_first_try.store(0, std::memory_order_relaxed);
  g_skipped.store(0, std::memory_order_relaxed);
  g_waited.store(0, std::memory_order_relaxed);
  g_wait_total_ns.store(0, std::memory_order_relaxed);
  g_wait_max_ns.store(0, std::memory_order_relaxed);
  g_timeouts.store(0, std::memory_order_relaxed);
  g_late_dropped.store(0, std::memory_order_relaxed);
  g_untagged.store(0, std::memory_order_relaxed);
  g_unsettled.store(0, std::memory_order_relaxed);
  g_poll_timeouts.store(0, std::memory_order_relaxed);
  g_poll_timeout_ns.store(0, std::memory_order_relaxed);
  g_reader_lost.store(0, std::memory_order_relaxed);
  g_after_deadline.store(0, std::memory_order_relaxed);
  g_unknown_missed.store(0, std::memory_order_relaxed);
  g_count_rule.store(0, std::memory_order_relaxed);
  g_debt_capped.store(0, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(g_tag_mutex);
    g_tag.clear();
  }
  tag_log::reset();
  {
    std::lock_guard<std::mutex> lock(g_debt_mutex);
    g_debt.clear();
  }
  std::lock_guard<std::mutex> lock(g_topic_mutex);
  g_topic_stats.clear();
}

}  // namespace ros2_lf_binding
