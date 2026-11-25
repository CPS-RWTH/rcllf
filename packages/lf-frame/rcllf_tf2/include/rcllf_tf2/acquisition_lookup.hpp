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


/// @file acquisition_lookup.hpp
/// @brief Bounded transform lookup at a sample's acquisition stamp.
///
/// `request(target, source, stamp, on_result)` asks tf2 for the transform at
/// `stamp`. tf2 stays the lookup and interpolation authority; the adapter only
/// decides when the Buffer is asked. Every request ends in exactly one
/// outcome: `ok` (tf2 returned the transform), or, at the logical deadline
/// `tag(request) + wait_bound`, the configured timeout outcome (`reject` or
/// `fallback:<name>`) when tf2 still extrapolates, and `failure` when tf2
/// reports any other error (an unknown frame, no connection). Never "latest".
///
/// Under rcllf (ros2_lf_binding linked, runtime hooks non-null) the request
/// is evaluated at its tag and again after every /tf or /tf_static ingest
/// reaction of the node, and the deadline is a logical wake the generated
/// tf2_wake reaction delivers. Under stock ROS (hooks null) the same rules run
/// on the node clock with a polling wall timer; the stock outcomes of
/// timing-dependent cases are not claimed equal to the rcllf ones.
///
/// Every request and outcome is one `RCLLF_TF2` JSON line on stderr
/// (`tf_request`, `tf_outcome`). Both carry `adapter`, the adapter's
/// construction ordinal on its node (0, 1, ...), so `(node, adapter, id)`
/// pairs a request with its outcome when a node has several adapters, and
/// the Buffer's available interval for the frame pair at that moment,
/// `[oldest_common, latest_common]` (see available()). The outcome callback runs inside the LF
/// reaction that decided it (a request, an ingest or the wake reaction); it
/// must not publish, since those reactions declare no publisher effects
/// (docs/features/tf2.md).

#ifndef RCLLF_TF2__ACQUISITION_LOOKUP_HPP_
#define RCLLF_TF2__ACQUISITION_LOOKUP_HPP_

#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcllf_tf2/tf2_runtime_hooks.h"
#include "tf2/buffer_core.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"

namespace rcllf_tf2
{

/// What a request yields when tf2 still extrapolates at its deadline.
struct OnTimeout
{
  enum class Kind { kReject, kFailure, kFallback };
  Kind kind = Kind::kReject;
  std::string name;  // the fallback's name (kFallback only)

  static OnTimeout reject() {return OnTimeout{Kind::kReject, ""};}
  static OnTimeout failure() {return OnTimeout{Kind::kFailure, ""};}
  static OnTimeout fallback(std::string fallback_name)
  {
    return OnTimeout{Kind::kFallback, std::move(fallback_name)};
  }

  /// Inverse of str(); throws std::invalid_argument on any other text.
  static OnTimeout parse(const std::string & text)
  {
    if (text == "reject") {return reject();}
    if (text == "failure") {return failure();}
    const std::string prefix = "fallback:";
    if (text.size() > prefix.size() && text.compare(0, prefix.size(), prefix) == 0) {
      return fallback(text.substr(prefix.size()));
    }
    throw std::invalid_argument("rcllf_tf2::OnTimeout: unknown on_timeout '" + text + "'");
  }

  /// "reject", "failure" or "fallback:<name>" (the execution.yaml spelling).
  std::string str() const
  {
    switch (kind) {
      case Kind::kReject: return "reject";
      case Kind::kFailure: return "failure";
      default: return "fallback:" + name;
    }
  }
};

enum class Outcome { kOk, kReject, kFailure, kFallback };

struct Result
{
  uint64_t id = 0;
  Outcome outcome = Outcome::kReject;
  std::string fallback;  // the fallback's name when outcome == kFallback
  geometry_msgs::msg::TransformStamped transform;  // valid when outcome == kOk
  std::string error;  // tf2's message for the other outcomes
  int64_t request_time = 0;  // ns, tag (rcllf) or node clock (stock)
  int64_t decided_time = 0;  // ns, same clock

  /// "ok", "reject", "failure" or "fallback:<name>".
  std::string name() const
  {
    switch (outcome) {
      case Outcome::kOk: return "ok";
      case Outcome::kReject: return "reject";
      case Outcome::kFailure: return "failure";
      default: return "fallback:" + fallback;
    }
  }
};

class AcquisitionLookup
{
public:
  using Callback = std::function<void (const Result &)>;

  /// @param buffer      the node's Buffer, on the node clock
  /// @param node        the node constructing the adapter (`this`)
  /// @param wait_bound  logical time a request may wait for its transform;
  ///                    a literal, so the analyzer can read it
  /// @param on_timeout  the outcome when tf2 still extrapolates at the deadline
  AcquisitionLookup(
    tf2_ros::Buffer & buffer, rclcpp::Node * node, std::chrono::nanoseconds wait_bound,
    OnTimeout on_timeout)
  : buffer_(buffer), node_(node), wait_bound_(wait_bound.count()),
    on_timeout_(std::move(on_timeout)), adapter_(next_ordinal(node))
  {
    if (wait_bound_ <= 0) {
      throw std::invalid_argument("rcllf_tf2::AcquisitionLookup: wait_bound must be positive");
    }
    if (ros2_lf_tf2_register != nullptr && ros2_lf_tf2_active != nullptr &&
      ros2_lf_tf2_active())
    {
      const rcllf_tf2_hooks hooks{this, &AcquisitionLookup::on_wake,
        &AcquisitionLookup::on_ingested, &AcquisitionLookup::on_close};
      handle_ = ros2_lf_tf2_register(&hooks);
      if (handle_ < 0) {
        throw std::runtime_error(
                "rcllf_tf2::AcquisitionLookup: construct it in the node's constructor "
                "(the rcllf runtime registers it there)");
      }
      apply_configured();
    } else {
      // Stock ROS: decide on the node clock. 5 ms polling is a design choice
      // (no claim on stock timing), see the class comment.
      timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(5), [this]() {
          const int64_t now = node_->now().nanoseconds();
          on_ingested(this, now, 0);
          on_wake(this, now, 0);
        });
    }
  }

  ~AcquisitionLookup()
  {
    if (handle_ >= 0 && ros2_lf_tf2_unregister != nullptr) {
      ros2_lf_tf2_unregister(handle_);
    }
  }

  AcquisitionLookup(const AcquisitionLookup &) = delete;
  AcquisitionLookup & operator=(const AcquisitionLookup &) = delete;

  /// Ask for target <- source at `stamp`. `on_result` runs exactly once, now
  /// when tf2 can already answer, else at a later ingest or at the deadline.
  /// Returns the request id (1, 2, ... per adapter).
  uint64_t request(
    const std::string & target, const std::string & source, const rclcpp::Time & stamp,
    Callback on_result)
  {
    int64_t tag = 0;
    uint32_t micro = 0;
    if (handle_ >= 0 && ros2_lf_tf2_current != nullptr && ros2_lf_tf2_current(&tag, &micro) != 0) {
      // Not an LF worker: the request would have no tag, and an outcome
      // decided now would run its callback on this thread.
      throw std::logic_error(
              "rcllf_tf2::AcquisitionLookup::request: called outside an LF reaction; "
              "request only from a callback of the node that owns the adapter");
    }
    Pending p;
    {
      // Under the mutex: in stock mode a MultiThreadedExecutor may run two
      // requests at once, and a bare ++ hands both the same id.
      std::lock_guard<std::recursive_mutex> lk(mu_);
      p.id = ++last_id_;
    }
    p.target = target;
    p.source = source;
    p.stamp = stamp.nanoseconds();
    p.cb = std::move(on_result);
    p.request_time = now(&p.request_micro);
    p.deadline = p.request_time + wait_bound_;
    p.steady_start = steady_ns();
    trace_request(p);
    std::vector<Done> done;
    if (p.stamp <= 0) {
      // tf2 reads TimePoint(0) as "latest": answering it would be the
      // nondeterministic lookup this adapter exists to rule out.
      Result r = result_of(p, p.request_time);
      r.outcome = Outcome::kFailure;
      r.error = "stamp " + std::to_string(p.stamp) +
        " ns is not an acquisition time (tf2 reads 0 as latest)";
      trace_outcome(p, r, p.request_micro);
      done.push_back(Done{p.cb, r});
      finish(done);
      return p.id;
    }
    {
      std::lock_guard<std::recursive_mutex> lk(mu_);
      if (!try_ok(p, p.request_time, p.request_micro, &done)) {
        pending_.push_back(p);
        if (handle_ >= 0) {
          ros2_lf_tf2_request_wake(handle_, p.deadline);
        }
      }
    }
    finish(done);
    return p.id;
  }

  /// Requests still waiting for their outcome.
  std::size_t pending() const
  {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    return pending_.size();
  }

  int64_t wait_bound_ns() const {return wait_bound_;}

  /// This adapter's construction ordinal on its node (0 for the first).
  uint32_t adapter_index() const {return adapter_;}

  /// The Buffer's available interval for target <- source now, in ns:
  /// `latest` is tf2's latest common time (0 when every link on the path is
  /// static, i.e. any stamp), `oldest` the latest of the per-link oldest
  /// stamps along the path, static links excluded (0 when there is no
  /// dynamic link). `oldest > latest` means the links' stamps do not overlap.
  /// Both are kNotConnected (-1) when tf2 cannot connect the two frames, and
  /// both are kUnavailable (-2) when the path's interval is not one exact
  /// range:
  ///  * a link whose child had another parent within the samples the Buffer
  ///    holds for it (tf2 allows reparenting: child under A at 10 ms, under B
  ///    at 20 ms, under A again at 100 ms is answerable at 10..20 ms and from
  ///    100 ms on, which no [oldest, latest] states);
  ///  * a dynamic link whose only samples are stamped 0: tf2 reads stamp 0 as
  ///    "latest", so the link answers no positive stamp, while [0, 0] means
  ///    an all-static path.
  ///
  /// tf2 has no public call for a link's samples (BufferCore::getFrame is
  /// private), so each dynamic link is bisected over the public
  /// _getParent(child, t), which succeeds exactly on [first, last] of the
  /// child's stored samples and names the parent of the sample at or before t:
  ///  1. `first`: the oldest t at which the child has any sample (contiguous);
  ///  2. `oldest`: the oldest t from which the child's parent is the link's
  ///     parent at every probe, bisected over [first, newest];
  ///  3. the link is exact only when oldest == first, i.e. the parent at the
  ///     child's first sample, at its newest and at every probe is the link's.
  /// At most 2 x 63 probes per link, exact to the nanosecond. What the public
  /// API cannot show is a reparent episode that starts and ends strictly
  /// between two probes of step 2 (docs/reference/trace-schema.md says so).
  static constexpr int64_t kNotConnected = -1;
  static constexpr int64_t kUnavailable = -2;
  struct Interval
  {
    int64_t oldest = kNotConnected;
    int64_t latest = kNotConnected;
  };
  Interval available(const std::string & target, const std::string & source)
  {
    std::lock_guard<std::recursive_mutex> lk(mu_);
    Interval iv;
    tf2::BufferCore & c = core();
    const auto t = c._lookupFrameNumber(target);
    const auto s = c._lookupFrameNumber(source);
    tf2::TimePoint latest;
    std::string err;
    if (t == 0 || s == 0 ||
      c._getLatestCommonTime(t, s, latest, &err) != tf2::TF2Error::TF2_NO_ERROR)
    {
      return iv;
    }
    iv.latest = ns_of(latest);
    // The path: both frames' ancestors (current parents), joined at the
    // first common one. tf2 bounds a tree's depth at 1000 (MAX_GRAPH_DEPTH).
    auto ancestors = [&c](const std::string & frame) {
        std::vector<std::string> chain{frame};
        std::string parent;
        while (chain.size() < 1000 && c._getParent(chain.back(), tf2::TimePointZero, parent)) {
          chain.push_back(parent);
        }
        return chain;
      };
    const std::vector<std::string> up_s = ancestors(source);
    const std::vector<std::string> up_t = ancestors(target);
    std::vector<std::pair<std::string, std::string>> links;  // (parent, child)
    std::size_t join_s = up_s.size();
    std::size_t join_t = up_t.size();
    for (std::size_t i = 0; i < up_s.size() && join_s == up_s.size(); ++i) {
      for (std::size_t j = 0; j < up_t.size(); ++j) {
        if (up_s[i] == up_t[j]) {
          join_s = i;
          join_t = j;
          break;
        }
      }
    }
    if (join_s == up_s.size()) {
      return Interval{};  // connected per tf2 a moment ago, not now: not connected
    }
    for (std::size_t i = 0; i < join_s; ++i) {
      links.emplace_back(up_s[i + 1], up_s[i]);
    }
    for (std::size_t j = 0; j < join_t; ++j) {
      links.emplace_back(up_t[j + 1], up_t[j]);
    }
    const Interval unavailable{kUnavailable, kUnavailable};
    int64_t oldest = 0;
    for (const auto & link : links) {
      const auto p = c._lookupFrameNumber(link.first);
      const auto ch = c._lookupFrameNumber(link.second);
      tf2::TimePoint newest;
      if (c._getLatestCommonTime(p, ch, newest, &err) != tf2::TF2Error::TF2_NO_ERROR) {
        return Interval{};
      }
      const int64_t hi = ns_of(newest);
      std::string parent;
      // The child's parent at t (t > 0; 0 would ask for the latest sample).
      auto parent_is = [&](int64_t ns, bool * any) {
          const bool got = c._getParent(link.second, at(ns), parent);
          if (any != nullptr) {
            *any = got;
          }
          return got && parent == link.first;
        };
      if (hi <= 0) {
        // tf2 reports time 0 for a static link (its cache answers any t) and
        // for a dynamic link sampled only at stamp 0 (it answers t = 0 only).
        if (parent_is(1, nullptr)) {
          continue;  // static: any stamp
        }
        return unavailable;
      }
      if (!parent_is(hi, nullptr)) {
        return unavailable;
      }
      // 1. first: the oldest t > 0 with any sample of the child; _getParent
      // succeeds exactly on the stored range, so this predicate is monotone.
      // A sample at stamp 0 makes first 1 (t = 0 is "latest" to tf2).
      int64_t lo = 1;
      int64_t up = hi;
      while (lo < up) {
        const int64_t mid = lo + (up - lo) / 2;
        bool any = false;
        parent_is(mid, &any);
        if (any) {
          up = mid;
        } else {
          lo = mid + 1;
        }
      }
      const int64_t first = up;
      // 2. The oldest t from which the parent is the link's at every probe.
      lo = first;
      up = hi;
      while (lo < up) {
        const int64_t mid = lo + (up - lo) / 2;
        if (parent_is(mid, nullptr)) {
          up = mid;
        } else {
          lo = mid + 1;
        }
      }
      // 3. Fixed parent: the child's first sample already has this parent.
      if (up != first || !parent_is(first, nullptr)) {
        return unavailable;
      }
      oldest = std::max(oldest, up);
    }
    iv.oldest = oldest;
    return iv;
  }

private:
  /// execution.yaml `nodes.<node>.tf2` wins over the constructor literals;
  /// the literals stay the stock-ROS values.
  void apply_configured()
  {
    if (ros2_lf_tf2_config == nullptr) {
      return;
    }
    int64_t bound = 0;
    char name[256] = {0};
    const int rc = ros2_lf_tf2_config(handle_, &bound, name, sizeof(name));
    if (rc < 0) {
      throw std::runtime_error(
              "rcllf_tf2::AcquisitionLookup: execution.yaml on_timeout name is longer than "
              "255 bytes");
    }
    if (rc != 1) {
      return;
    }
    const int64_t literal_bound = wait_bound_;
    const std::string literal_on_timeout = on_timeout_.str();
    if (bound > 0) {
      wait_bound_ = bound;
    }
    if (name[0] != '\0') {
      on_timeout_ = OnTimeout::parse(name);
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "rcllf_tf2: execution.yaml nodes.<node>.tf2 sets wait bound %" PRId64 " ns, on_timeout %s "
      "(constructor literal %" PRId64 " ns, %s)",
      static_cast<int64_t>(wait_bound_), on_timeout_.str().c_str(),
      static_cast<int64_t>(literal_bound), literal_on_timeout.c_str());
  }

public:
  const OnTimeout & on_timeout() const {return on_timeout_;}

private:
  struct Pending
  {
    uint64_t id = 0;
    std::string target;
    std::string source;
    int64_t stamp = 0;
    int64_t request_time = 0;
    uint32_t request_micro = 0;
    int64_t deadline = 0;
    int64_t steady_start = 0;
    Callback cb;
  };
  struct Done
  {
    Callback cb;
    Result result;
  };

  static int64_t steady_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  int64_t now(uint32_t * micro) const
  {
    int64_t t = 0;
    uint32_t m = 0;
    if (handle_ >= 0 && ros2_lf_tf2_current != nullptr && ros2_lf_tf2_current(&t, &m) == 0) {
      *micro = m;
      return t;
    }
    *micro = 0;
    return node_->now().nanoseconds();
  }

  static tf2::TimePoint at(int64_t ns)
  {
    return tf2::TimePoint(std::chrono::nanoseconds(ns));
  }

  tf2::BufferCore & core() {return buffer_;}

  static int64_t ns_of(tf2::TimePoint tp)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
  }

  /// The construction ordinal of a new adapter on `node`. Keyed by the
  /// node's base interface and reset when that node is gone, so a node
  /// created later at the same address starts at 0 again.
  static uint32_t next_ordinal(rclcpp::Node * node)
  {
    static std::mutex m;
    static std::map<const void *,
      std::pair<std::weak_ptr<rclcpp::node_interfaces::NodeBaseInterface>, uint32_t>> next;
    const auto base = node->get_node_base_interface();
    std::lock_guard<std::mutex> lk(m);
    auto & slot = next[base.get()];
    if (slot.first.expired() || slot.first.lock() != base) {
      slot = {base, 0};
    }
    return slot.second++;
  }

  /// `,"oldest_common":<ns|null|"unavailable">,"latest_common":<same>` of the
  /// pair now: null when tf2 cannot connect the frames, "unavailable" when
  /// the interval is not one exact range (see available()).
  std::string interval_json(const Pending & p)
  {
    const Interval iv = available(p.target, p.source);
    auto num = [](int64_t v) {
        return v == kUnavailable ? std::string("\"unavailable\"") :
               v < 0 ? std::string("null") : std::to_string(v);
      };
    return ",\"oldest_common\":" + num(iv.oldest) + ",\"latest_common\":" + num(iv.latest);
  }

  bool try_ok(const Pending & p, int64_t time, uint32_t micro, std::vector<Done> * done)
  {
    // An unseen frame is not transformable yet; asked directly, tf2 would
    // log a warning on every evaluation.
    if (core()._lookupFrameNumber(p.target) == 0 || core()._lookupFrameNumber(p.source) == 0 ||
      !core().canTransform(p.target, p.source, at(p.stamp), nullptr))
    {
      return false;
    }
    Result r = result_of(p, time);
    r.outcome = Outcome::kOk;
    r.transform = core().lookupTransform(p.target, p.source, at(p.stamp));
    trace_outcome(p, r, micro);
    done->push_back(Done{p.cb, r});
    return true;
  }

  Result result_of(const Pending & p, int64_t time) const
  {
    Result r;
    r.id = p.id;
    r.request_time = p.request_time;
    r.decided_time = time;
    return r;
  }

  /// The deadline outcome: tf2's error class decides between the timeout
  /// outcome (extrapolation: the transform may still arrive) and failure.
  void decide(const Pending & p, int64_t time, uint32_t micro, std::vector<Done> * done)
  {
    Result r = result_of(p, time);
    bool extrapolation = false;
    try {
      core().lookupTransform(p.target, p.source, at(p.stamp));
      r.error = "tf2 answered only at the deadline";  // try_ok ran first; kept for safety
    } catch (const tf2::ExtrapolationException & e) {
      extrapolation = true;
      r.error = e.what();
    } catch (const tf2::TransformException & e) {
      r.error = e.what();
    }
    if (extrapolation && mutant_latest_on_timeout()) {
      // Falsification mutant (test only, plan output verification 3): answer
      // with the latest transform, the behaviour the adapter exists to rule out.
      r.outcome = Outcome::kOk;
      r.error.clear();
      r.transform = core().lookupTransform(p.target, p.source, tf2::TimePointZero);
    } else if (!extrapolation) {
      r.outcome = Outcome::kFailure;
    } else if (on_timeout_.kind == OnTimeout::Kind::kReject) {
      r.outcome = Outcome::kReject;
    } else if (on_timeout_.kind == OnTimeout::Kind::kFailure) {
      r.outcome = Outcome::kFailure;
    } else {
      r.outcome = Outcome::kFallback;
      r.fallback = on_timeout_.name;
    }
    trace_outcome(p, r, micro);
    done->push_back(Done{p.cb, r});
  }

  static bool mutant_latest_on_timeout()
  {
    const char * m = std::getenv("RCLLF_TF2_TEST_MUTANT");
    return m != nullptr && std::string(m) == "latest_on_timeout";
  }

  /// Run the outcome callbacks outside the lock, in decision order.
  static void finish(std::vector<Done> & done)
  {
    for (auto & d : done) {
      if (d.cb) {
        d.cb(d.result);
      }
    }
  }

  static void on_ingested(void * ctx, int64_t time, uint32_t micro)
  {
    auto * self = static_cast<AcquisitionLookup *>(ctx);
    std::vector<Done> done;
    {
      std::lock_guard<std::recursive_mutex> lk(self->mu_);
      std::vector<Pending> keep;
      for (const auto & p : self->pending_) {
        if (!self->try_ok(p, time, micro, &done)) {
          keep.push_back(p);
        }
      }
      self->pending_.swap(keep);
    }
    finish(done);
  }

  static void on_wake(void * ctx, int64_t time, uint32_t micro)
  {
    auto * self = static_cast<AcquisitionLookup *>(ctx);
    std::vector<Done> done;
    {
      std::lock_guard<std::recursive_mutex> lk(self->mu_);
      std::vector<Pending> keep;
      for (const auto & p : self->pending_) {
        if (p.deadline > time) {
          keep.push_back(p);
        } else if (!self->try_ok(p, time, micro, &done)) {
          self->decide(p, time, micro, &done);
        }
      }
      self->pending_.swap(keep);
    }
    finish(done);
  }

  static void on_close(void * ctx, int64_t time, uint32_t micro)
  {
    auto * self = static_cast<AcquisitionLookup *>(ctx);
    std::lock_guard<std::recursive_mutex> lk(self->mu_);
    for (const auto & p : self->pending_) {
      self->trace(
        "{\"event\":\"tf_open_at_shutdown\",\"node\":\"" + self->node_name() +
        "\",\"adapter\":" + std::to_string(self->adapter_) + ",\"id\":" +
        std::to_string(p.id) + ",\"deadline\":" + std::to_string(p.deadline) +
        ",\"time\":" + std::to_string(time) + ",\"micro\":" + std::to_string(micro) + "}");
    }
  }

  std::string node_name() const {return node_->get_fully_qualified_name();}

  void trace(const std::string & json) const
  {
    if (handle_ >= 0 && ros2_lf_tf2_trace != nullptr) {
      ros2_lf_tf2_trace(json.c_str());
    } else {
      std::fprintf(stderr, "RCLLF_TF2 %s\n", json.c_str());
    }
  }

  void trace_request(const Pending & p)
  {
    trace(
      "{\"event\":\"tf_request\",\"node\":\"" + node_name() + "\",\"adapter\":" +
      std::to_string(adapter_) + ",\"id\":" +
      std::to_string(p.id) + ",\"target\":\"" + p.target + "\",\"source\":\"" + p.source +
      "\",\"stamp\":" + std::to_string(p.stamp) + ",\"time\":" +
      std::to_string(p.request_time) + ",\"micro\":" + std::to_string(p.request_micro) +
      ",\"wait_bound_ns\":" + std::to_string(wait_bound_) + ",\"on_timeout\":\"" +
      on_timeout_.str() + "\"" + interval_json(p) + "}");
  }

  void trace_outcome(const Pending & p, const Result & r, uint32_t micro)
  {
    char xyz[160] = "null";
    if (r.outcome == Outcome::kOk) {
      const auto & tr = r.transform.transform;
      std::snprintf(
        xyz, sizeof(xyz), "[%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f]", tr.translation.x,
        tr.translation.y, tr.translation.z, tr.rotation.x, tr.rotation.y, tr.rotation.z,
        tr.rotation.w);
    }
    std::string err;
    for (char c : r.error) {  // JSON string: drop quotes, backslashes, control chars
      if (c != '"' && c != '\\' && static_cast<unsigned char>(c) >= 0x20) {
        err.push_back(c);
      }
    }
    trace(
      "{\"event\":\"tf_outcome\",\"node\":\"" + node_name() + "\",\"adapter\":" +
      std::to_string(adapter_) + ",\"id\":" +
      std::to_string(p.id) + ",\"outcome\":\"" + r.name() + "\",\"stamp\":" +
      std::to_string(p.stamp) + ",\"time\":" + std::to_string(r.decided_time) +
      ",\"micro\":" + std::to_string(micro) + ",\"wait_logical_ns\":" +
      std::to_string(r.decided_time - p.request_time) + ",\"wait_physical_ns\":" +
      std::to_string(steady_ns() - p.steady_start) + interval_json(p) +
      ",\"transform\":" + xyz + ",\"error\":\"" + err + "\"}");
  }

  tf2_ros::Buffer & buffer_;
  rclcpp::Node * node_;
  int64_t wait_bound_;
  OnTimeout on_timeout_;
  uint32_t adapter_;
  int handle_ = -1;
  uint64_t last_id_ = 0;
  mutable std::recursive_mutex mu_;
  std::vector<Pending> pending_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rcllf_tf2

#endif  // RCLLF_TF2__ACQUISITION_LOOKUP_HPP_
