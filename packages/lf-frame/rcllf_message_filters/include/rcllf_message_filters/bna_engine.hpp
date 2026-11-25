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

/// @file bna_engine.hpp
/// @brief Bounded Nearest-to-Anchor decision engine.
///
/// Pure C++17, no ROS, no clock. Implements the BNA rules
/// (docs/features/bna-synchronizer.md): every event comes
/// with an explicit tag, and every admitted sample gets exactly one terminal
/// outcome, reported through the record callback. The Python oracle
/// (tools/lf_eval/lf_eval/bna/oracle.py) is the reference this engine is
/// replayed against.

#ifndef RCLLF_MESSAGE_FILTERS__BNA_ENGINE_HPP_
#define RCLLF_MESSAGE_FILTERS__BNA_ENGINE_HPP_

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace rcllf_message_filters
{
namespace bna
{

/// An LF tag (or a steady-clock time with a counter under stock ROS).
struct Tag
{
  int64_t time = 0;
  uint32_t micro = 0;
  bool operator<(const Tag & o) const {return std::tie(time, micro) < std::tie(o.time, o.micro);}
  bool operator==(const Tag & o) const {return time == o.time && micro == o.micro;}
  bool operator<=(const Tag & o) const {return !(o < *this);}
};

/// Publication key (run_epoch, writer GID, ordinal), text "<epoch>:<gid>:<ordinal>".
struct Key
{
  uint64_t epoch = 0;
  std::string gid;
  uint64_t ordinal = 0;

  std::string text() const
  {
    return std::to_string(epoch) + ":" + gid + ":" + std::to_string(ordinal);
  }
  bool operator<(const Key & o) const
  {
    return std::tie(epoch, gid, ordinal) < std::tie(o.epoch, o.gid, o.ordinal);
  }

  /// Parse "<epoch>:<gid>:<ordinal>". Returns false on a malformed key.
  static bool parse(const std::string & s, Key * out)
  {
    const auto a = s.find(':');
    const auto b = s.rfind(':');
    if (a == std::string::npos || b == a || b + 1 >= s.size()) {
      return false;
    }
    char * end = nullptr;
    const std::string e = s.substr(0, a);
    const std::string o = s.substr(b + 1);
    out->epoch = std::strtoull(e.c_str(), &end, 10);
    if (e.empty() || *end != '\0') {
      return false;
    }
    out->ordinal = std::strtoull(o.c_str(), &end, 10);
    if (*end != '\0') {
      return false;
    }
    out->gid = s.substr(a + 1, b - a - 1);
    return !out->gid.empty();
  }
};

struct Params
{
  uint64_t epoch = 0;
  int64_t delta = 0;          // ns, inclusive window
  int64_t lateness[2] = {0, 0};  // ns
  int64_t capacity[2] = {1, 1};
  int64_t retention = 1;      // ns, input-1 samples
  int64_t timeout = 1;        // ns, anchors

  bool valid() const
  {
    return delta >= 0 && lateness[0] >= 0 && lateness[1] >= 0 && capacity[0] >= 1 &&
           capacity[1] >= 1 && retention > 0 && timeout > 0;
  }
};

/// One terminal outcome: a decision for an anchor, a sample line for input 1.
struct Record
{
  bool decision = false;
  int sync = 0;
  int input = 0;
  std::string key;
  int64_t stamp = 0;
  std::string outcome;   // MATCHED | REJECTED | SELECTED | LATE | ... | FAULT
  std::string reason;
  bool has_partner = false;
  std::string partner;
  int64_t partner_stamp = 0;
  Tag tag;
};

#ifdef RCLLF_BNA_MUTANTS
/// Deliberate rule violations for the falsification tests. Compiled only
/// into the replay tool, never into the installed policy.
enum class Mutant
{
  NONE, TIE_LATER_STAMP, WATERMARK_INCLUSIVE, LATE_EXCLUSIVE, REUSE, TIMEOUT_LAZY,
  OVERFLOW_NEWEST, SKEW_EXCLUSIVE, EXPIRY_ON_ADMIT, LATE_ANCHOR_DOUBLE, NO_ORDER_CHECK
};

inline std::optional<Mutant> mutant_from_name(const std::string & n)
{
  static const std::map<std::string, Mutant> names = {
    {"none", Mutant::NONE}, {"tie_later_stamp", Mutant::TIE_LATER_STAMP},
    {"watermark_inclusive", Mutant::WATERMARK_INCLUSIVE},
    {"late_exclusive", Mutant::LATE_EXCLUSIVE}, {"reuse", Mutant::REUSE},
    {"timeout_lazy", Mutant::TIMEOUT_LAZY}, {"overflow_newest", Mutant::OVERFLOW_NEWEST},
    {"skew_exclusive", Mutant::SKEW_EXCLUSIVE}, {"expiry_on_admit", Mutant::EXPIRY_ON_ADMIT},
    {"late_anchor_double", Mutant::LATE_ANCHOR_DOUBLE},
    {"no_order_check", Mutant::NO_ORDER_CHECK}};
  auto it = names.find(n);
  if (it == names.end()) {
    return std::nullopt;
  }
  return it->second;
}
#endif

/// The engine. `Payload` travels with a sample and comes back in the match
/// callback (the policy stores the message event there).
template<class Payload>
class Engine
{
public:
  using RecordFn = std::function<void (const Record &)>;
  using MatchFn = std::function<void (Payload &, Payload &)>;
  using DeadlineFn = std::function<void (const Tag &)>;

  explicit Engine(const Params & p, int sync = 0)
  : p_(p), sync_(sync)
  {
    if (!p.valid()) {
      throw std::invalid_argument("BNA parameters out of range");
    }
  }

  void on_record(RecordFn f) {on_record_ = std::move(f);}
  void on_match(MatchFn f) {on_match_ = std::move(f);}
  void on_deadline(DeadlineFn f) {on_deadline_ = std::move(f);}

#ifdef RCLLF_BNA_MUTANTS
  void set_mutant(Mutant m) {mutant_ = m;}
#endif

  const Params & params() const {return p_;}
  bool closed() const {return closed_;}
  const Tag & last_tag() const {return last_tag_;}
  uint64_t missing_count(int i) const {return missing_[i];}

  /// The earliest live deadline, if any.
  std::optional<Tag> next_deadline() const
  {
    std::optional<Tag> d;
    for (const auto & x : anchors_) {
      if (!d || x.deadline < *d) {d = x.deadline;}
    }
    if (expiry_live()) {
      for (const auto & x : cands_) {
        if (!d || x.deadline < *d) {d = x.deadline;}
      }
    }
    return d;
  }

  void admit(int i, int64_t stamp, const Key & k, const Tag & t, Payload payload = Payload())
  {
    check(t);
    if (i != 0 && i != 1) {
      throw std::invalid_argument("input out of range");
    }
    catch_up(t, false);
    Sample x{i, stamp, k, k.text(), Tag{}, std::move(payload)};
    if (closed_) {
      return reject(x, t, "closed");
    }
    if (k.epoch != p_.epoch) {
      return reject(x, t, "identity");
    }
    if (!mut(MutantId::NO_ORDER_CHECK)) {
      auto it = last_ordinal_.find({i, k.gid});
      if (it != last_ordinal_.end() && k.ordinal == it->second) {
        return reject(x, t, "duplicate");
      }
      if (it != last_ordinal_.end() && k.ordinal < it->second) {
        return reject(x, t, "order");
      }
      last_ordinal_[{i, k.gid}] = k.ordinal;
    }
    if (stamp < 0) {
      return reject(x, t, "stamp");
    }
    int64_t w = 0;
    if (watermark(i, &w) && (mut(MutantId::LATE_EXCLUSIVE) ? stamp < w : stamp <= w)) {
      return reject(x, t, "late");
    }
#ifdef RCLLF_BNA_MUTANTS
    if (mutant_ == Mutant::EXPIRY_ON_ADMIT) {
      expire_on_admit(t);
    }
#endif
    x.deadline = Tag{t.time + (i == 0 ? p_.timeout : p_.retention), 0};
    const Tag dl = x.deadline;
    (i == 0 ? anchors_ : cands_).push_back(std::move(x));
    max_stamp_[i] = has_max_[i] ? std::max(max_stamp_[i], stamp) : stamp;
    has_max_[i] = true;
    if (on_deadline_ && (i == 0 || expiry_live())) {
      on_deadline_(dl);
    }
    supersede(t);
    capacity(i, t);
    decide(t);
  }

  void missing(int i, const Key & /*k*/, const Tag & t)
  {
    check(t);
    catch_up(t, false);
    if (i == 0 || i == 1) {
      ++missing_[i];
    }
  }

  /// A missing report with a lower bound `stamp` of the lost
  /// sample's header stamp (the binding passes publish tag minus the input's
  /// declared max_stamp_lag). Admitting the sample would have raised the
  /// watermark at least this far, so it rises as on admit and pending anchors
  /// may decide now instead of waiting for the next delivered sample or their
  /// timeout. A bound above the true stamp would decide too early and reject
  /// later in-window samples as late, so the caller must never pass one.
  void missing(int i, const Key & k, const Tag & t, int64_t stamp)
  {
    missing(i, k, t);
    if ((i != 0 && i != 1) || stamp < 0 || closed_) {
      return;
    }
    max_stamp_[i] = has_max_[i] ? std::max(max_stamp_[i], stamp) : stamp;
    has_max_[i] = true;
    decide(t);
  }

  void wake(const Tag & t)
  {
    check(t);
    if (mut(MutantId::TIMEOUT_LAZY)) {
      return;
    }
    catch_up(t, true);
  }

  void close(const std::string & reason, const Tag & t)
  {
    if (reason != "shutdown" && reason != "epoch" && reason != "cancelled") {
      throw std::invalid_argument("close reason " + reason);
    }
    check(t);
    catch_up(t, true);
    if (closed_) {
      return;
    }
    sort(anchors_);
    sort(cands_);
    auto anchors = std::move(anchors_);
    auto cands = std::move(cands_);
    anchors_.clear();
    cands_.clear();
    for (auto & a : anchors) {
      decision(a, t, "REJECTED", reason);
    }
    for (auto & y : cands) {
      sample(y, t, "FLUSHED");
    }
    closed_ = true;
  }

private:
  struct Sample
  {
    int input;
    int64_t stamp;
    Key key;
    std::string text;
    Tag deadline;
    Payload payload;
  };

  enum class MutantId
  {
    TIE_LATER_STAMP, WATERMARK_INCLUSIVE, LATE_EXCLUSIVE, REUSE, TIMEOUT_LAZY,
    OVERFLOW_NEWEST, SKEW_EXCLUSIVE, EXPIRY_ON_ADMIT, LATE_ANCHOR_DOUBLE, NO_ORDER_CHECK
  };

  bool mut(MutantId m) const
  {
#ifdef RCLLF_BNA_MUTANTS
    return static_cast<int>(mutant_) == static_cast<int>(m) + 1;
#else
    (void)m;
    return false;
#endif
  }

  bool expiry_live() const {return !mut(MutantId::EXPIRY_ON_ADMIT);}

  static bool sk_less(const Sample & a, const Sample & b)
  {
    return std::tie(a.stamp, a.key) < std::tie(b.stamp, b.key);
  }
  static void sort(std::vector<Sample> & v) {std::stable_sort(v.begin(), v.end(), sk_less);}

  void check(const Tag & t)
  {
    if (t < last_tag_) {
      throw std::invalid_argument("BNA event tag goes back in time");
    }
    last_tag_ = t;
  }

  bool watermark(int i, int64_t * w) const
  {
    if (!has_max_[i]) {
      return false;
    }
    *w = max_stamp_[i] - p_.lateness[i];
    return true;
  }

  bool in_window(const Sample & a, const Sample & y) const
  {
    const int64_t skew = std::llabs(y.stamp - a.stamp);
    return mut(MutantId::SKEW_EXCLUSIVE) ? skew < p_.delta : skew <= p_.delta;
  }

  void emit(const Record & r)
  {
    if (on_record_) {
      on_record_(r);
    }
  }

  void decision(
    const Sample & a, const Tag & t, const char * outcome, const std::string & reason,
    const Sample * partner = nullptr)
  {
    Record r;
    r.decision = true;
    r.sync = sync_;
    r.input = a.input;
    r.key = a.text;
    r.stamp = a.stamp;
    r.outcome = outcome;
    r.reason = reason;
    if (partner != nullptr) {
      r.has_partner = true;
      r.partner = partner->text;
      r.partner_stamp = partner->stamp;
    }
    r.tag = t;
    emit(r);
  }

  void sample(
    const Sample & y, const Tag & t, const char * outcome,
    const std::string & reason = "")
  {
    Record r;
    r.sync = sync_;
    r.input = y.input;
    r.key = y.text;
    r.stamp = y.stamp;
    r.outcome = outcome;
    r.reason = reason;
    r.tag = t;
    emit(r);
  }

  void reject(const Sample & x, const Tag & t, const std::string & reason)
  {
    if (x.input == 0) {
      decision(x, t, "REJECTED", reason);
      if (reason == "late" && mut(MutantId::LATE_ANCHOR_DOUBLE)) {
        sample(x, t, "LATE");
      }
    } else if (reason == "late") {
      sample(x, t, "LATE");
    } else {
      sample(x, t, "FAULT", reason);
    }
  }

  /// Remove and return every element matching `pred`, in (s, k) order.
  template<class Pred>
  std::vector<Sample> take(std::vector<Sample> & buf, Pred pred)
  {
    std::vector<Sample> out;
    std::vector<Sample> keep;
    for (auto & x : buf) {
      (pred(x) ? out : keep).push_back(std::move(x));
    }
    buf = std::move(keep);
    sort(out);
    return out;
  }

  void reject_window(const Sample & y, const Tag & t, const char * reason)
  {
    for (auto & a : take(anchors_, [&](const Sample & a) {return in_window(a, y);})) {
      decision(a, t, "REJECTED", reason);
    }
  }

  void catch_up(const Tag & t, bool inclusive)
  {
    for (;; ) {
      auto d = next_deadline();
      if (!d || !(*d < t || (inclusive && *d == t))) {
        return;
      }
      deadline_step(*d, mut(MutantId::TIMEOUT_LAZY) ? t : *d);
    }
  }

  /// Run the deadline step of `d`, recording its outcomes at tag `at`
  /// (always `d`, except under the timeout_lazy mutant).
  void deadline_step(const Tag & d, const Tag & at)
  {
    if (expiry_live()) {
      for (auto & y : take(cands_, [&](const Sample & y) {return y.deadline == d;})) {
        sample(y, at, "EXPIRED");
        reject_window(y, at, "candidate_expired");
      }
    }
    for (auto & a : take(anchors_, [&](const Sample & a) {return a.deadline == d;})) {
      decision(a, at, "REJECTED", "timeout");
    }
    decide(at);
  }

#ifdef RCLLF_BNA_MUTANTS
  void expire_on_admit(const Tag & t)
  {
    for (auto & y : take(cands_, [&](const Sample & y) {return y.deadline.time <= t.time;})) {
      sample(y, t, "EXPIRED");
      reject_window(y, t, "candidate_expired");
    }
  }
#endif

  void supersede(const Tag & t)
  {
    int64_t w0 = 0;
    if (!watermark(0, &w0)) {
      return;  // before any anchor the lower bound is minus infinity
    }
    int64_t a_lb = w0 + 1;
    for (const auto & a : anchors_) {
      a_lb = std::min(a_lb, a.stamp);
    }
    for (auto & y : take(cands_, [&](const Sample & y) {return y.stamp + p_.delta < a_lb;})) {
      sample(y, t, "SUPERSEDED");
    }
  }

  bool complete(const Sample & a) const
  {
    const int64_t bound = a.stamp + p_.delta;
    for (int i = 0; i < 2; ++i) {
      int64_t w = 0;
      if (!watermark(i, &w)) {
        return false;
      }
      if (mut(MutantId::WATERMARK_INCLUSIVE) ? !(w >= bound) : !(w > bound)) {
        return false;
      }
    }
    return true;
  }

  void decide(const Tag & t)
  {
    supersede(t);
    while (!anchors_.empty()) {
      auto least = std::min_element(anchors_.begin(), anchors_.end(), sk_less);
      if (!complete(*least)) {
        return;
      }
      Sample a = std::move(*least);
      anchors_.erase(least);
      auto best = cands_.end();
      for (auto it = cands_.begin(); it != cands_.end(); ++it) {
        if (in_window(a, *it) && (best == cands_.end() || better(a, *it, *best))) {
          best = it;
        }
      }
      if (best == cands_.end()) {
        decision(a, t, "REJECTED", "missing");
      } else {
        Sample c = mut(MutantId::REUSE) ? *best : std::move(*best);
        if (!mut(MutantId::REUSE)) {
          cands_.erase(best);
        }
        decision(a, t, "MATCHED", "", &c);
        sample(c, t, "SELECTED");
        if (on_match_) {
          on_match_(a.payload, c.payload);
        }
      }
      supersede(t);
    }
  }

  /// R9: does `x` beat `y` as the candidate of anchor `a`?
  bool better(const Sample & a, const Sample & x, const Sample & y) const
  {
    const int64_t sx = std::llabs(x.stamp - a.stamp);
    const int64_t sy = std::llabs(y.stamp - a.stamp);
    if (sx != sy) {
      return sx < sy;
    }
    if (x.stamp != y.stamp) {
      return mut(MutantId::TIE_LATER_STAMP) ? x.stamp > y.stamp : x.stamp < y.stamp;
    }
    return x.key < y.key;
  }

  void capacity(int i, const Tag & t)
  {
    auto & buf = i == 0 ? anchors_ : cands_;
    if (static_cast<int64_t>(buf.size()) <= p_.capacity[i]) {
      return;
    }
    auto v = mut(MutantId::OVERFLOW_NEWEST) ?
      std::max_element(buf.begin(), buf.end(), sk_less) :
      std::min_element(buf.begin(), buf.end(), sk_less);
    Sample victim = std::move(*v);
    buf.erase(v);
    if (i == 0) {
      decision(victim, t, "REJECTED", "anchor_evicted");
      return;
    }
    sample(victim, t, "OVERFLOW");
    reject_window(victim, t, "candidate_evicted");
  }

  Params p_;
  int sync_;
  std::vector<Sample> anchors_;
  std::vector<Sample> cands_;
  int64_t max_stamp_[2] = {0, 0};
  bool has_max_[2] = {false, false};
  std::map<std::pair<int, std::string>, uint64_t> last_ordinal_;
  uint64_t missing_[2] = {0, 0};
  bool closed_ = false;
  Tag last_tag_{INT64_MIN, 0};
  RecordFn on_record_;
  MatchFn on_match_;
  DeadlineFn on_deadline_;
#ifdef RCLLF_BNA_MUTANTS
  Mutant mutant_ = Mutant::NONE;
#endif
};

// -- JSON lines ------------------------------------------

inline std::string json_escape(const std::string & s)
{
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') {
      o += '\\';
      o += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      static const char * hex = "0123456789abcdef";
      o += "\\u00";
      o += hex[(c >> 4) & 0xf];
      o += hex[c & 0xf];
    } else {
      o += c;
    }
  }
  return o;
}

inline std::string tag_json(const Tag & t)
{
  return "[" + std::to_string(t.time) + "," + std::to_string(t.micro) + "]";
}

inline std::string to_json(const Record & r)
{
  std::string s;
  if (r.decision) {
    s = "{\"ev\":\"decision\",\"sync\":" + std::to_string(r.sync) + ",\"anchor\":\"" +
      json_escape(r.key) + "\",\"stamp\":" + std::to_string(r.stamp) + ",\"outcome\":\"" +
      r.outcome + "\",\"reason\":\"" + r.reason + "\",\"partner\":" +
      (r.has_partner ? "\"" + json_escape(r.partner) + "\"" : std::string("null")) +
      ",\"partner_stamp\":" +
      (r.has_partner ? std::to_string(r.partner_stamp) : std::string("null"));
  } else {
    s = "{\"ev\":\"sample\",\"sync\":" + std::to_string(r.sync) + ",\"input\":" +
      std::to_string(r.input) + ",\"key\":\"" + json_escape(r.key) + "\",\"stamp\":" +
      std::to_string(r.stamp) + ",\"outcome\":\"" + r.outcome + "\",\"reason\":\"" + r.reason +
      "\"";
  }
  return s + ",\"tag\":" + tag_json(r.tag) + "}";
}

inline std::string params_json(const Params & p, int sync)
{
  return "{\"ev\":\"params\",\"sync\":" + std::to_string(sync) + ",\"epoch\":" +
         std::to_string(p.epoch) + ",\"delta\":" + std::to_string(p.delta) + ",\"lateness\":[" +
         std::to_string(p.lateness[0]) + "," + std::to_string(p.lateness[1]) +
         "],\"capacity\":[" + std::to_string(p.capacity[0]) + "," +
         std::to_string(p.capacity[1]) + "],\"retention\":" + std::to_string(p.retention) +
         ",\"timeout\":" + std::to_string(p.timeout) + "}";
}

inline std::string admit_json(int sync, int input, int64_t stamp, const Key & k, const Tag & t)
{
  return "{\"ev\":\"admit\",\"sync\":" + std::to_string(sync) + ",\"input\":" +
         std::to_string(input) + ",\"stamp\":" + std::to_string(stamp) + ",\"key\":\"" +
         json_escape(k.text()) + "\",\"tag\":" + tag_json(t) + "}";
}

inline std::string missing_json(int sync, int input, const Key & k, const Tag & t)
{
  return "{\"ev\":\"missing\",\"sync\":" + std::to_string(sync) + ",\"input\":" +
         std::to_string(input) + ",\"key\":\"" + json_escape(k.text()) + "\",\"tag\":" +
         tag_json(t) + "}";
}

/// A missing report with the lost sample's stamp.
inline std::string missing_json(
  int sync, int input, const Key & k, const Tag & t, int64_t stamp)
{
  return "{\"ev\":\"missing\",\"sync\":" + std::to_string(sync) + ",\"input\":" +
         std::to_string(input) + ",\"stamp\":" + std::to_string(stamp) + ",\"key\":\"" +
         json_escape(k.text()) + "\",\"tag\":" + tag_json(t) + "}";
}

inline std::string wake_json(int sync, const Tag & t)
{
  return "{\"ev\":\"wake\",\"sync\":" + std::to_string(sync) + ",\"tag\":" + tag_json(t) + "}";
}

inline std::string close_json(int sync, const std::string & reason, const Tag & t)
{
  return "{\"ev\":\"close\",\"sync\":" + std::to_string(sync) + ",\"reason\":\"" + reason +
         "\",\"tag\":" + tag_json(t) + "}";
}

}  // namespace bna
}  // namespace rcllf_message_filters

#endif  // RCLLF_MESSAGE_FILTERS__BNA_ENGINE_HPP_
