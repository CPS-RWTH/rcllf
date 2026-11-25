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

#include <rcl/subscription.h>
#include <rmw/rmw.h>
#include <rmw/types.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int64.hpp"

namespace
{

using Header = std_msgs::msg::Header;
using Int64 = std_msgs::msg::Int64;
using json = nlohmann::json;
using namespace std::chrono_literals;

constexpr auto kScenarioTimeout = 8s;

// DDS discovery under heavy parallel load can take longer than the
// scenario timeout, so the match waits get their own, generous bound.
std::chrono::seconds match_timeout()
{
  const char * value = std::getenv("RCLLF_PROBE_MATCH_TIMEOUT_S");
  return std::chrono::seconds(value != nullptr ? std::atoi(value) : 30);
}

std::int64_t elapsed_ms(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count();
}

std::string gid_string(const rmw_gid_t & gid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    stream << std::setw(2) << static_cast<unsigned>(gid.data[i]);
  }
  return stream.str();
}

std::uintptr_t address_of(const std::shared_ptr<void> & message)
{
  return reinterpret_cast<std::uintptr_t>(message.get());
}

void emit(const json & value)
{
  // The subscriber's match monitor emits from its own thread.
  static std::mutex output_mutex;
  const std::string line = value.dump();
  std::lock_guard<std::mutex> lock(output_mutex);
  std::cout << line << std::endl;
}

// Block until the runner releases this process with `command` on stdin.
bool wait_for_runner(const std::string & command)
{
  std::string line;
  return static_cast<bool>(std::getline(std::cin, line)) && line == command;
}

int parse_int(const char * value, const char * name)
{
  try {
    return std::stoi(value);
  } catch (const std::exception & error) {
    throw std::runtime_error(std::string("invalid ") + name + ": " + error.what());
  }
}

Header identical_message()
{
  Header message;
  message.stamp.sec = 85;
  message.stamp.nanosec = 17;
  message.frame_id = "issue85-identical-payload";
  return message;
}

bool wait_for_match(
  const rclcpp::PublisherBase & publisher, std::size_t minimum,
  std::chrono::steady_clock::duration timeout = kScenarioTimeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (publisher.get_subscription_count() >= minimum) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

// `get_subscription_count()` reflects the discovery graph at the
// exact instant it is called, on a background GraphListener thread separate
// from this one. Two distinct races were found while reproducing this under
// load:
//
// 1. A genuine one-tick discovery flicker (e.g. a duplicate announcement
//    that de-duplicates a couple of milliseconds later) — a very short,
//    tight debounce (two consecutive identical reads a couple of
//    milliseconds apart) absorbs this without meaningfully widening the
//    sampling window.
// 2. A real unmatch: when the sample for the *last* publish in a fixed-size
//    exchange (`expected == count`) is delayed, the peer subscriber process
//    — which tears down immediately once it has taken its `expected`th
//    message — can finish and destroy its subscription before the delayed
//    sample runs, correctly reporting 0. An earlier version of this fix used
//    a long (50ms) stabilization window and reproduced this second race
//    deterministically instead of the first: the extra dwell time gave the
//    subscriber time to always win the race. The fix for that race lives on
//    the subscriber side (a short linger before process exit, see
//    `subscriber_mode`); this function must stay fast so it does not
//    reintroduce the same window.
struct SubscriptionCountSample
{
  std::size_t count;
  bool stable;
  std::chrono::microseconds wait_time;
};

SubscriptionCountSample stable_subscription_count(
  const rclcpp::PublisherBase & publisher,
  std::chrono::steady_clock::duration poll_interval = 1ms,
  int required_consecutive_reads = 2,
  std::chrono::steady_clock::duration timeout = 20ms)
{
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + timeout;
  std::size_t last = publisher.get_subscription_count();
  int consecutive = 1;
  while (consecutive < required_consecutive_reads) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return {
        last, false,
        std::chrono::duration_cast<std::chrono::microseconds>(now - start)};
    }
    std::this_thread::sleep_for(poll_interval);
    const std::size_t current = publisher.get_subscription_count();
    if (current == last) {
      ++consecutive;
    } else {
      last = current;
      consecutive = 1;
    }
  }
  return {
    last, true,
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - start)};
}

int publisher_mode(int argc, char ** argv)
{
  if (argc != 7 && argc != 8) {
    throw std::runtime_error(
            "publisher requires TOPIC EPOCH COUNT REQUIRED_MATCH INTERVAL_MS [GATED]");
  }
  const std::string topic = argv[2];
  const std::string epoch = argv[3];
  const int count = parse_int(argv[4], "count");
  const int required_match = parse_int(argv[5], "required match count");
  const int interval_ms = parse_int(argv[6], "interval");
  const bool gated = argc == 8 && parse_int(argv[7], "gated") != 0;
  auto node = std::make_shared<rclcpp::Node>("publication_identity_probe_publisher");
  auto publisher = node->create_publisher<Header>(topic, rclcpp::QoS(64));
  const auto match_start = std::chrono::steady_clock::now();
  if (required_match > 0 && !wait_for_match(*publisher, required_match, match_timeout())) {
    emit({
        {"event", "error"}, {"reason", "match_timeout"}, {"epoch", epoch},
        {"match_ms", elapsed_ms(match_start)},
    });
    return 2;
  }
  // This writer having matched the reader does not mean the reader
  // has matched this writer. A volatile FastDDS reader never delivers a sample
  // published before its own side of the match, so a gated publisher waits until
  // the runner has seen the subscriber's matched-publisher count include it.
  if (gated) {
    emit({{"event", "writer_matched"}, {"epoch", epoch}, {"match_ms", elapsed_ms(match_start)}});
    if (!wait_for_runner("publish")) {
      emit({{"event", "error"}, {"reason", "gate_not_released"}, {"epoch", epoch}});
      return 2;
    }
  }

  const std::string gid = gid_string(publisher->get_gid());
  for (int ordinal = 1; ordinal <= count; ++ordinal) {
    const Header message = identical_message();
    const auto publish_start = std::chrono::steady_clock::now();
    publisher->publish(message);
    const auto sample = stable_subscription_count(*publisher);
    const auto publish_to_sample_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - publish_start);
    emit({
        {"event", "publish"},
        {"boundary", "after_successful_publish"},
        {"epoch", epoch},
        {"publisher_gid", gid},
        {"adapter_ordinal", ordinal},
        {"matched_subscriptions", sample.count},
        {"matched_subscriptions_stable", sample.stable},
        {"match_stabilization_us", sample.wait_time.count()},
        {"publish_to_sample_us", publish_to_sample_us.count()},
        {"payload_frame_id", message.frame_id},
        {"stamp_sec", message.stamp.sec},
        {"stamp_nanosec", message.stamp.nanosec},
    });
    if (interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
  }
  // Every matched reader holds every sample before this process exits.
  if (!publisher->wait_for_all_acked(kScenarioTimeout)) {
    emit({
        {"event", "error"}, {"reason", "ack_timeout"}, {"epoch", epoch},
        {"matched_subscriptions", publisher->get_subscription_count()},
    });
    return 2;
  }
  emit({{"event", "all_acked"}, {"epoch", epoch}});
  return 0;
}

// Reports every change of the subscription's matched-publisher
// count (the reader side of discovery) so the runner can gate publishers on it.
class ReaderMatchMonitor
{
public:
  explicit ReaderMatchMonitor(rclcpp::SubscriptionBase::SharedPtr subscription)
  : subscription_(std::move(subscription)), thread_([this]() {run();}) {}

  ~ReaderMatchMonitor()
  {
    stop_ = true;
    thread_.join();
  }

private:
  void run()
  {
    std::size_t reported = 0;
    emit({{"event", "reader_matched"}, {"publishers", reported}});
    while (!stop_) {
      const std::size_t current = subscription_->get_publisher_count();
      if (current != reported) {
        reported = current;
        emit({{"event", "reader_matched"}, {"publishers", reported}});
      }
      std::this_thread::sleep_for(1ms);
    }
  }

  rclcpp::SubscriptionBase::SharedPtr subscription_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

struct RetainedSample
{
  std::shared_ptr<void> message;
  rclcpp::MessageInfo info;
  std::uintptr_t take_address;
};

int subscriber_mode(int argc, char ** argv)
{
  if (argc != 7) {
    throw std::runtime_error(
            "subscriber requires TOPIC EXPECTED DEPTH HOLD CAPACITY");
  }
  const std::string topic = argv[2];
  const int expected = parse_int(argv[3], "expected");
  const int depth = parse_int(argv[4], "depth");
  const bool hold = parse_int(argv[5], "hold") != 0;
  const int capacity = parse_int(argv[6], "capacity");
  if (expected < 1 || depth < 1 || capacity < expected) {
    throw std::runtime_error("expected/depth must be positive and capacity must cover expected");
  }

  auto node = std::make_shared<rclcpp::Node>("publication_identity_probe_subscriber");
  std::vector<std::uintptr_t> callback_addresses;
  std::vector<uint64_t> callback_sequences;
  auto subscription = node->create_subscription<Header>(
    topic, rclcpp::QoS(static_cast<std::size_t>(depth)),
    [&callback_addresses, &callback_sequences](
      Header::SharedPtr message, const rclcpp::MessageInfo & info) {
      const auto & rmw_info = info.get_rmw_message_info();
      callback_addresses.push_back(reinterpret_cast<std::uintptr_t>(message.get()));
      callback_sequences.push_back(rmw_info.publication_sequence_number);
      emit({
        {"event", "callback"},
        {"address", callback_addresses.back()},
        {"publication_sequence_number", rmw_info.publication_sequence_number},
        {"publisher_gid", gid_string(rmw_info.publisher_gid)},
      });
    });

  ReaderMatchMonitor match_monitor(subscription);
  emit({{"event", "subscriber_ready"}, {"topic", topic}, {"depth", depth}});
  // A holding subscriber takes nothing until the runner reports that
  // every publish was acknowledged, so the queue holds exactly what was published.
  if (hold && !wait_for_runner("take")) {
    emit({{"event", "error"}, {"reason", "hold_not_released"}});
    return 3;
  }
  std::vector<RetainedSample> retained;
  retained.reserve(static_cast<std::size_t>(capacity));
  rclcpp::WaitSet wait_set;
  rclcpp::SubscriptionWaitSetMask wait_mask;
  wait_mask.include_events = false;
  wait_mask.include_intra_process_waitable = false;
  wait_set.add_subscription(subscription, wait_mask);
  std::size_t create_count = 0;
  std::size_t take_attempts = 0;
  std::size_t max_retained = 0;
  // A live subscriber takes while publishers are still being
  // discovered. Its take window opens when the runner reports every publisher
  // matched (or closes stdin), so slow discovery cannot surface as take_timeout.
  // Shared, because the watcher outlives this frame when the takes finish first.
  auto armed = std::make_shared<std::atomic<bool>>(hold);
  std::thread arm_watcher;
  if (!hold) {
    arm_watcher = std::thread([armed]() {
          wait_for_runner("matched");
          *armed = true;
      });
  }
  auto deadline = std::chrono::steady_clock::time_point::max();
  while (rclcpp::ok() && retained.size() < static_cast<std::size_t>(expected)) {
    const auto now = std::chrono::steady_clock::now();
    if (*armed && deadline == std::chrono::steady_clock::time_point::max()) {
      deadline = now + kScenarioTimeout;
    }
    if (now >= deadline) {
      break;
    }
    const auto wait_result = wait_set.wait(std::min<std::chrono::nanoseconds>(deadline - now,
        100ms));
    if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      continue;
    }
    if (wait_result.kind() != rclcpp::WaitResultKind::Ready) {
      break;
    }
    auto message = subscription->create_message();
    ++create_count;
    rclcpp::MessageInfo info;
    ++take_attempts;
    if (!subscription->take_type_erased(message.get(), info)) {
      emit({{"event", "error"}, {"reason", "ready_subscription_take_failed"}});
      return 3;
    }
    const auto & rmw_info = info.get_rmw_message_info();
    const auto address = address_of(message);
    retained.push_back({message, info, address});
    max_retained = std::max(max_retained, retained.size());
    emit({
        {"event", "take"},
        {"address", address},
        {"publisher_gid", gid_string(rmw_info.publisher_gid)},
        {"publication_sequence_number", rmw_info.publication_sequence_number},
        {"reception_sequence_number", rmw_info.reception_sequence_number},
        {"from_intra_process", rmw_info.from_intra_process},
    });
  }
  if (arm_watcher.joinable()) {
    arm_watcher.detach();
  }
  if (retained.size() != static_cast<std::size_t>(expected)) {
    emit({
        {"event", "error"}, {"reason", "take_timeout"},
        {"received", retained.size()}, {"expected", expected},
    });
    return 3;
  }

  // This is the ownership boundary under test: dispatch each exact retained object.
  // There is deliberately no take call in this loop.
  for (auto & sample : retained) {
    subscription->handle_message(sample.message, sample.info);
  }
  bool pointer_continuity = callback_addresses.size() == retained.size();
  for (std::size_t i = 0; pointer_continuity && i < retained.size(); ++i) {
    pointer_continuity = callback_addresses[i] == retained[i].take_address;
  }
  emit({
      {"event", "subscriber_summary"},
      {"received", retained.size()},
      {"callback_count", callback_addresses.size()},
      {"create_count", create_count},
      {"take_attempts", take_attempts},
      {"dispatch_take_count", 0},
      {"pointer_continuity", pointer_continuity},
      {"max_retained", max_retained},
      {"capacity", capacity},
  });
  // Linger before tearing down the subscription. Once this
  // process has taken its `expected`th message it has nothing left to do,
  // but the peer publisher is still measuring `get_subscription_count()`
  // for that very message (see `stable_subscription_count`) — destroying
  // the node/subscription immediately can win that race and report a
  // spurious unmatch for the last publish. This keeps the match alive long
  // enough for the publisher's near-instant post-publish sample to observe
  // it, without the large delay a prior version of this fix used on the
  // publisher side (which only made that race easier to hit).
  std::this_thread::sleep_for(200ms);
  return pointer_continuity ? 0 : 4;
}

class AdmittedPublisher
{
public:
  explicit AdmittedPublisher(rclcpp::Publisher<Header>::SharedPtr publisher)
  : publisher_(std::move(publisher)), owner_thread_(std::this_thread::get_id()) {}

  std::string publish(const Header & message)
  {
    if (std::this_thread::get_id() != owner_thread_) {
      return "helper_thread";
    }
    if (origin_depth_ != 0) {
      return "reentrant";
    }
    ++origin_depth_;
    publisher_->publish(message);
    --origin_depth_;
    return "admitted";
  }

  std::string invoke_reentrantly(const Header & message)
  {
    ++origin_depth_;
    const std::string result = publish(message);
    --origin_depth_;
    return result;
  }

  std::string publish(rclcpp::LoanedMessage<Header> &&)
  {
    return "loaned";
  }

private:
  rclcpp::Publisher<Header>::SharedPtr publisher_;
  std::thread::id owner_thread_;
  int origin_depth_{0};
};

bool wait_take(
  const std::shared_ptr<rclcpp::SubscriptionBase> & subscription,
  std::shared_ptr<void> & message,
  rclcpp::MessageInfo & info)
{
  rclcpp::WaitSet wait_set;
  rclcpp::SubscriptionWaitSetMask wait_mask;
  wait_mask.include_events = false;
  wait_mask.include_intra_process_waitable = false;
  wait_set.add_subscription(subscription, wait_mask);
  const auto wait_result = wait_set.wait(kScenarioTimeout);
  if (wait_result.kind() != rclcpp::WaitResultKind::Ready) {
    return false;
  }
  return subscription->take_type_erased(message.get(), info);
}

int refusal_mode()
{
  auto node = std::make_shared<rclcpp::Node>("publication_identity_probe_refusals");
  auto publisher = node->create_publisher<Header>("/issue85/refusal", 10);
  AdmittedPublisher admitted(publisher);
  const Header message = identical_message();

  std::string helper_result;
  std::thread helper([&admitted, &message, &helper_result]() {
      helper_result = admitted.publish(message);
    });
  helper.join();
  const std::string reentrant_result = admitted.invoke_reentrantly(message);

  auto lifecycle_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "publication_identity_probe_lifecycle");
  auto lifecycle_publisher = lifecycle_node->create_publisher<Header>(
    "/issue85/lifecycle", 10);
  std::shared_ptr<rclcpp::PublisherBase> lifecycle_base = lifecycle_publisher;
  const bool lifecycle_detected =
    dynamic_cast<rclcpp_lifecycle::LifecyclePublisher<Header> *>(lifecycle_base.get()) != nullptr;

  auto loaned_message = publisher->borrow_loaned_message();
  loaned_message.get() = message;
  const std::string loaned_result = admitted.publish(std::move(loaned_message));

  auto raw_publisher = node->create_publisher<Header>("/issue85/uninstrumented", 10);
  auto raw_subscription = node->create_subscription<Header>(
    "/issue85/uninstrumented", 10, [](Header::SharedPtr) {});
  if (!wait_for_match(*raw_publisher, 1)) {
    emit({{"event", "error"}, {"reason", "uninstrumented_match_timeout"}});
    return 5;
  }
  raw_publisher->publish(message);
  auto received = raw_subscription->create_message();
  rclcpp::MessageInfo received_info;
  const bool raw_taken = wait_take(raw_subscription, received, received_info);
  std::unordered_set<std::string> instrumented_gids{gid_string(publisher->get_gid())};
  const bool unknown_gid = raw_taken &&
    instrumented_gids.count(gid_string(received_info.get_rmw_message_info().publisher_gid)) == 0;

  const bool passed = helper_result == "helper_thread" &&
    reentrant_result == "reentrant" && lifecycle_detected &&
    loaned_result == "loaned" && unknown_gid;
  emit({
      {"event", "refusal_summary"},
      {"helper_thread", helper_result == "helper_thread" ? "refused" : "not_detected"},
      {"reentrant", reentrant_result == "reentrant" ? "refused" : "not_detected"},
      {"lifecycle", lifecycle_detected ? "refused" : "not_detected"},
      {"loaned", loaned_result == "loaned" ? "refused" : "not_detected"},
      {"uninstrumented", unknown_gid ? "refused" : "not_detected"},
      {"passed", passed},
  });
  return passed ? 0 : 6;
}

int intra_process_mode()
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("publication_identity_probe_intra", options);
  auto publisher = node->create_publisher<Header>("/issue85/intra", 10);
  auto subscription = node->create_subscription<Header>(
    "/issue85/intra", 10, [](Header::SharedPtr) {});
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline &&
    publisher->get_intra_process_subscription_count() == 0)
  {
    std::this_thread::sleep_for(5ms);
  }
  publisher->publish(identical_message());
  std::this_thread::sleep_for(100ms);
  auto message = subscription->create_message();
  rclcpp::MessageInfo info;
  const bool rmw_taken = subscription->take_type_erased(message.get(), info);
  const bool actual_intra_metadata =
    rmw_taken && info.get_rmw_message_info().from_intra_process;
  const std::string classification = actual_intra_metadata ? "supported" : "refused";
  const std::string reason = actual_intra_metadata ? "exact_metadata_available" :
    (rmw_taken ? "rmw_take_not_marked_intra_process" : "no_rmw_sample");
  emit({
      {"event", "intra_process_summary"},
      {"classification", classification},
      {"reason", reason},
      {"rmw_taken", rmw_taken},
      {"from_intra_process", actual_intra_metadata},
      {"intra_process_matches", publisher->get_intra_process_subscription_count()},
  });
  return 0;
}

// Does a loaned write take the same Fast DDS sequence number as an
// ordinary write? One plain-type publisher interleaves loaned and ordinary
// publishes; each payload carries its 1-based ordinal, and every received
// sample's publication_sequence_number must equal it. A publisher that cannot
// loan makes the scenario vacuous, so it fails instead of passing.
int loaned_sequence_mode(int argc, char ** argv)
{
  if (argc != 4) {
    throw std::runtime_error("loaned-sequence requires TOPIC COUNT");
  }
  const std::string topic = argv[2];
  const int count = parse_int(argv[3], "count");
  auto pub_node = std::make_shared<rclcpp::Node>("publication_identity_probe_loan_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("publication_identity_probe_loan_sub");
  const auto qos = rclcpp::QoS(static_cast<std::size_t>(count)).reliable();
  auto publisher = pub_node->create_publisher<Int64>(topic, qos);
  auto subscription = sub_node->create_subscription<Int64>(
    topic, qos, [](Int64::SharedPtr) {});
  if (!wait_for_match(*publisher, 1)) {
    emit({{"event", "error"}, {"reason", "match_timeout"}});
    return 2;
  }
  const bool can_loan = publisher->can_loan_messages();
  int loaned = 0;
  int ordinary = 0;
  for (int ordinal = 1; ordinal <= count; ++ordinal) {
    if (ordinal % 2 == 1) {
      auto message = publisher->borrow_loaned_message();
      message.get().data = ordinal;
      publisher->publish(std::move(message));
      ++loaned;
    } else {
      Int64 message;
      message.data = ordinal;
      publisher->publish(message);
      ++ordinary;
    }
  }
  const std::string gid = gid_string(publisher->get_gid());
  int received = 0;
  int mismatches = 0;
  int foreign_gid = 0;
  json first_mismatch;
  rclcpp::WaitSet wait_set;
  rclcpp::SubscriptionWaitSetMask wait_mask;
  wait_mask.include_events = false;
  wait_mask.include_intra_process_waitable = false;
  wait_set.add_subscription(subscription, wait_mask);
  while (received < count) {
    if (wait_set.wait(kScenarioTimeout).kind() != rclcpp::WaitResultKind::Ready) {
      break;
    }
    auto message = subscription->create_message();
    rclcpp::MessageInfo info;
    if (!subscription->take_type_erased(message.get(), info)) {
      continue;
    }
    const auto & rmw_info = info.get_rmw_message_info();
    const int64_t data = static_cast<Int64 *>(message.get())->data;
    ++received;
    if (gid_string(rmw_info.publisher_gid) != gid) {
      ++foreign_gid;
    }
    if (rmw_info.publication_sequence_number != static_cast<uint64_t>(data)) {
      if (mismatches++ == 0) {
        first_mismatch = {
          {"data", data}, {"publication_sequence_number", rmw_info.publication_sequence_number}};
      }
    }
  }
  const bool passed = can_loan && received == count && mismatches == 0 && foreign_gid == 0;
  emit({
      {"event", "loaned_sequence_summary"},
      {"can_loan_messages", can_loan},
      {"loaned_publishes", loaned},
      {"ordinary_publishes", ordinary},
      {"received", received},
      {"mismatches", mismatches},
      {"first_mismatch", first_mismatch},
      {"foreign_gid", foreign_gid},
      {"passed", passed},
  });
  return passed ? 0 : 7;
}

// With rmw_fastrtps_cpp and SYNCHRONOUS
// publication, does a publish put the sample into the DDS reader of another
// node of the same context before rcl_publish returns? Two nodes, one context,
// KEEP_LAST 1, no executor: right after each publish the probe calls rcl_take
// on the other node's reader and requires exactly that sample (publisher GID,
// sequence number == ordinal). One miss fails with
// "local_sync: sample not in reader after publish".
int local_sync_mode(int argc, char ** argv)
{
  if (argc != 5) {
    throw std::runtime_error("local-sync requires TOPIC COUNT {reliable|best_effort}");
  }
  const std::string topic = argv[2];
  const int count = parse_int(argv[3], "count");
  const std::string reliability = argv[4];
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  if (reliability == "reliable") {
    qos.reliable();
  } else if (reliability == "best_effort") {
    qos.best_effort();
  } else {
    throw std::runtime_error("unknown reliability: " + reliability);
  }
  auto pub_node = std::make_shared<rclcpp::Node>("publication_identity_probe_local_pub");
  auto sub_node = std::make_shared<rclcpp::Node>("publication_identity_probe_local_sub");
  auto publisher = pub_node->create_publisher<Int64>(topic, qos);
  auto subscription = sub_node->create_subscription<Int64>(
    topic, qos, [](Int64::SharedPtr) {});
  // Both sides of the match: a volatile reader drops samples of a writer it
  // has not matched yet, which is discovery (R4), not this premise.
  const auto deadline = std::chrono::steady_clock::now() + match_timeout();
  while (std::chrono::steady_clock::now() < deadline &&
    (publisher->get_subscription_count() < 1 || subscription->get_publisher_count() < 1))
  {
    std::this_thread::sleep_for(5ms);
  }
  if (publisher->get_subscription_count() < 1 || subscription->get_publisher_count() < 1) {
    emit({{"event", "error"}, {"reason", "match_timeout"}});
    return 2;
  }
  const char * mode_env = std::getenv("RMW_FASTRTPS_PUBLICATION_MODE");
  const std::string gid = gid_string(publisher->get_gid());
  rcl_subscription_t * handle = subscription->get_subscription_handle().get();
  int found = 0;
  int missing = 0;
  int wrong_identity = 0;
  json first_miss;
  for (int ordinal = 1; ordinal <= count; ++ordinal) {
    Int64 message;
    message.data = ordinal;
    publisher->publish(message);
    Int64 taken;
    rmw_message_info_t info = rmw_get_zero_initialized_message_info();
    const rcl_ret_t ret = rcl_take(handle, &taken, &info, nullptr);
    if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
      if (missing++ == 0) {
        first_miss = {{"ordinal", ordinal}};
      }
      continue;
    }
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("rcl_take failed: " + std::to_string(ret));
    }
    if (taken.data != ordinal || gid_string(info.publisher_gid) != gid ||
      info.publication_sequence_number != static_cast<uint64_t>(ordinal))
    {
      ++wrong_identity;
      continue;
    }
    ++found;
  }
  const bool passed = found == count && missing == 0 && wrong_identity == 0;
  emit({
      {"event", "local_sync_summary"},
      {"reliability", reliability},
      {"publication_mode", mode_env != nullptr ? mode_env : ""},
      {"publishes", count},
      {"found", found},
      {"missing", missing},
      {"wrong_identity", wrong_identity},
      {"first_miss", first_miss},
      {"passed", passed},
      {"reason", passed ? "" : (missing > 0 ?
        "local_sync: sample not in reader after publish" :
        "local_sync: wrong sample identity in reader")},
  });
  return passed ? 0 : 8;
}

int environment_mode()
{
  emit({
      {"event", "environment"},
      {"rmw_implementation", rmw_get_implementation_identifier()},
  });
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    if (argc < 2) {
      throw std::runtime_error("missing mode");
    }
    const std::string mode = argv[1];
    if (mode == "publisher") {
      result = publisher_mode(argc, argv);
    } else if (mode == "subscriber") {
      result = subscriber_mode(argc, argv);
    } else if (mode == "refusals") {
      result = refusal_mode();
    } else if (mode == "intra-process") {
      result = intra_process_mode();
    } else if (mode == "loaned-sequence") {
      result = loaned_sequence_mode(argc, argv);
    } else if (mode == "local-sync") {
      result = local_sync_mode(argc, argv);
    } else if (mode == "environment") {
      result = environment_mode();
    } else {
      throw std::runtime_error("unknown mode: " + mode);
    }
  } catch (const std::exception & error) {
    emit({{"event", "error"}, {"reason", error.what()}});
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
