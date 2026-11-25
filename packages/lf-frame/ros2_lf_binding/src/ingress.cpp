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

/// @file ingress.cpp
/// @brief One-scheduler ingress role. See ingress.hpp.

#include "ingress.hpp"

#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <stdexcept>
#include <typeinfo>
#include <string>
#include <utility>
#include <vector>

#include "component_manager.hpp"
#include "corr_runtime.hpp"
#include "external_input.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rcl/error_handling.h"
#include "rcl/subscription.h"
#include "rcl/wait.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "ros2_lf_binding/node_registry.h"
#include "ros2_lf_binding/ros2_lf_binding.h"
#include "plant_record.hpp"
#include "semantic_trace.hpp"
#include "thread_role.hpp"
#include "time_source.hpp"
#include "user_callback_gate.hpp"
#include "type_description_interfaces/srv/get_type_description.hpp"

namespace ros2_lf_binding
{
extern std::shared_ptr<LFComponentManager> g_manager;

namespace
{
const rclcpp::Logger & logger()
{
  static const rclcpp::Logger l = rclcpp::get_logger("ros2_lf_binding");
  return l;
}

template<class SrvT>
bool is_service(const rclcpp::ServiceBase * s)
{
  if (s == nullptr) {
    return false;
  }
  if (dynamic_cast<const rclcpp::Service<SrvT> *>(s) != nullptr) {
    return true;
  }
  // Same concrete type instantiated in another shared object (rclcpp's own
  // get_type_description service, measured: dynamic_cast fails across the
  // DSO boundary): compare the mangled type names instead.
  return std::strcmp(typeid(*s).name(), typeid(rclcpp::Service<SrvT>).name()) == 0;
}

template<class SrvT, class Fill>
void reply_failure(const IngressRecord & r, Fill fill)
{
  auto svc = std::dynamic_pointer_cast<rclcpp::Service<SrvT>>(r.service);
  if (!svc) {
    return;
  }
  typename SrvT::Response resp;
  fill(*static_cast<typename SrvT::Request *>(r.request.get()), resp);
  try {
    svc->send_response(*r.header, resp);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger(), "rcllf ingress: failure reply not sent: %s", e.what());
  }
}
}  // namespace

bool ingress_suffix_allowed(const std::string & s)
{
  static const char * kAllowed[] = {
    "list_parameters", "get_parameters", "get_parameter_types", "describe_parameters",
    "set_parameters", "set_parameters_atomically", "get_state", "get_available_states",
    "get_available_transitions", "get_transition_graph", "change_state", "get_type_description"};
  for (const char * a : kAllowed) {
    if (s == a) {
      return true;
    }
  }
  return false;
}

bool ingress_suffix_mutating(const std::string & s)
{
  return s == "set_parameters" || s == "set_parameters_atomically" || s == "change_state";
}

bool ingress_type_matches(const std::string & s, const rclcpp::ServiceBase * svc)
{
  namespace ri = rcl_interfaces::srv;
  namespace lm = lifecycle_msgs::srv;
  if (s == "list_parameters") {return is_service<ri::ListParameters>(svc);}
  if (s == "get_parameters") {return is_service<ri::GetParameters>(svc);}
  if (s == "get_parameter_types") {return is_service<ri::GetParameterTypes>(svc);}
  if (s == "describe_parameters") {return is_service<ri::DescribeParameters>(svc);}
  if (s == "set_parameters") {return is_service<ri::SetParameters>(svc);}
  if (s == "set_parameters_atomically") {return is_service<ri::SetParametersAtomically>(svc);}
  if (s == "get_state") {return is_service<lm::GetState>(svc);}
  if (s == "get_available_states") {return is_service<lm::GetAvailableStates>(svc);}
  if (s == "get_available_transitions" || s == "get_transition_graph") {
    return is_service<lm::GetAvailableTransitions>(svc);
  }
  if (s == "change_state") {return is_service<lm::ChangeState>(svc);}
  if (s == "get_type_description") {
    // rclcpp (Jazzy) builds this service on a C-typesupport wrapper in an
    // anonymous namespace of node_type_descriptions.cpp, so the type cannot
    // be named here: match its mangled name, measured 2026-09-24 as
    // rclcpp::Service<(anonymous namespace)::GetTypeDescription__C>.
    return is_service<type_description_interfaces::srv::GetTypeDescription>(svc) ||
           (svc != nullptr &&
           std::strcmp(
             typeid(*svc).name(),
             "N6rclcpp7ServiceIN12_GLOBAL__N_121GetTypeDescription__CEEE") == 0);
  }
  return false;
}

IngressRole & IngressRole::instance()
{
  // Never destroyed: a static destructor at exit must not race the thread.
  static IngressRole * inst = new IngressRole();
  return *inst;
}

IngressRole::IngressRole() = default;

std::shared_ptr<NodeMailbox> IngressRole::mailbox(const std::string & key, bool create)
{
  std::lock_guard<std::mutex> lk(boxes_mu_);
  auto it = boxes_.find(key);
  if (it != boxes_.end()) {
    return it->second;
  }
  if (!create) {
    return nullptr;
  }
  auto mb = std::make_shared<NodeMailbox>();
  boxes_[key] = mb;
  return mb;
}

void IngressRole::trace(
  const std::string & key, const IngressRecord & r, const char * state,
  const std::string & detail)
{
  if (!trace_callbacks_enabled()) {
    return;
  }
  const auto wait_us = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - r.t_rx).count();
  RCLCPP_INFO(
    rclcpp::get_logger("ros2_lf_binding.trace"),
    "{\"event\":\"ingress\",\"record\":%llu,\"key\":\"%s\",\"service\":\"%s\",\"state\":\"%s\","
    "\"detail\":\"%s\",\"wait_us\":%lld,\"gettid\":%ld}",
    static_cast<unsigned long long>(r.id), key.c_str(),  // NOLINT(runtime/int)
    r.service_suffix.c_str(), state, detail.c_str(),
    static_cast<long long>(wait_us),  // NOLINT(runtime/int)
    static_cast<long>(syscall(SYS_gettid)));  // NOLINT(runtime/int)
}

bool IngressRole::admit(
  const std::string & key, const std::string & node_fqn,
  const rclcpp::ServiceBase::SharedPtr & service)
{
  const std::string name = service ? service->get_service_name() : "<null>";
  const std::string prefix = (node_fqn == "/" ? "" : node_fqn) + "/";
  std::string suffix;
  if (name.compare(0, prefix.size(), prefix) == 0) {
    suffix = name.substr(prefix.size());
  }
  if (!service || suffix.empty() || suffix.find('/') != std::string::npos ||
    !ingress_suffix_allowed(suffix) || !ingress_type_matches(suffix, service.get()))
  {
    RCLCPP_ERROR(logger(), "rcllf: ingress refused non-infrastructure entity '%s'", name.c_str());
    return false;
  }
  ensure_thread();
  Admitted a{key, suffix, service, nullptr, 0, -1, {}};
  return submit_admit(a, name);
}

bool IngressRole::admit_corr_subscription(
  uint32_t edge, const rclcpp::SubscriptionBase::SharedPtr & sub, std::vector<uint32_t> group)
{
  if (!sub) {
    return false;
  }
  ensure_thread();
  Admitted a{"", "", nullptr, sub, edge, -1, {}};
  a.group = std::move(group);
  return submit_admit(a, sub->get_topic_name());
}

namespace
{
/// A reader in the ingress wait set by its rcl handle alone. Ingress
/// never holds the SubscriptionBase of a rebind reader, so the strong
/// references the wait set takes while it waits never hide the node's drop.
class RclReaderWaitable final : public rclcpp::Waitable
{
public:
  explicit RclReaderWaitable(std::shared_ptr<rcl_subscription_t> handle)
  : handle_(std::move(handle)) {}
  using rclcpp::Waitable::add_to_wait_set;
  using rclcpp::Waitable::is_ready;
  using rclcpp::Waitable::execute;
  size_t get_number_of_ready_subscriptions() override {return 1;}
  void add_to_wait_set(rcl_wait_set_t & wait_set) override
  {
    const rcl_ret_t ret = rcl_wait_set_add_subscription(&wait_set, handle_.get(), &index_);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "rcllf ingress: reader not added");
    }
  }
  bool is_ready(const rcl_wait_set_t & wait_set) override
  {
    return index_ < wait_set.size_of_subscriptions && wait_set.subscriptions[index_] != nullptr;
  }
  std::shared_ptr<void> take_data() override {return nullptr;}
  std::shared_ptr<void> take_data_by_entity_id(size_t) override {return nullptr;}
  void execute(const std::shared_ptr<void> &) override {}
  void set_on_ready_callback(std::function<void(size_t, int)>) override {}
  void clear_on_ready_callback() override {}

private:
  std::shared_ptr<rcl_subscription_t> handle_;
  size_t index_ = 0;
};

using rosidl_typesupport_introspection_cpp::MessageMembers;

/// A default-initialized message of the reader's type, built from the
/// type's introspection data, so no SubscriptionBase is needed to make one.
std::shared_ptr<void> make_message(const MessageMembers * m)
{
  void * raw = std::malloc(m->size_of_);
  if (raw == nullptr) {
    return nullptr;
  }
  m->init_function(raw, rosidl_runtime_cpp::MessageInitialization::ALL);
  return std::shared_ptr<void>(raw, [m](void * p) {
             m->fini_function(p);
             std::free(p);
           });
}
}  // namespace

bool IngressRole::replace_corr_reader(
  uint32_t edge, const rclcpp::SubscriptionBase::SharedPtr & sub, int64_t created)
{
  if (!sub || refuse_corr_admit_.load()) {
    return false;
  }
  const rosidl_message_type_support_t * ts = get_message_typesupport_handle(
    &sub->get_message_type_support_handle(),
    rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (ts == nullptr || ts->data == nullptr) {
    rcutils_reset_error();
    RCLCPP_ERROR(
      logger(), "rcllf: ingress could not admit '%s': no C++ introspection type support",
      sub->get_topic_name());
    return false;
  }
  Admitted a{"", "", nullptr, nullptr, edge, -1, {}};
  a.rcl_sub = sub->get_subscription_handle();
  a.reader = sub;
  a.waitable = std::make_shared<RclReaderWaitable>(a.rcl_sub);
  a.members = ts->data;
  const rclcpp::QoS qos = sub->get_actual_qos();
  a.keep_last = qos.history() == rclcpp::HistoryPolicy::KeepLast;
  a.depth = qos.depth();
  a.created = created;
  ensure_thread();
  return submit_admit(a, sub->get_topic_name(), edge);
}

void IngressRole::retire_corr_reader(uint32_t edge)
{
  if (!thread_running_.load()) {
    std::lock_guard<std::mutex> lk(admitted_mu_);
    if (!thread_running_.load()) {
      retire_corr_reader_locked(edge);
      return;
    }
  }
  submit_admit(Admitted{}, "edge " + std::to_string(edge) + " reader", edge, false);
}

void IngressRole::retire_corr_reader_locked(uint32_t edge)
{
  // stable_partition, not remove_if: remove_if leaves the tail moved-from,
  // so the retired entries (their waitable, their take mutex) would be lost.
  const auto end = std::stable_partition(
    admitted_.begin(), admitted_.end(),
    [edge](const Admitted & x) {return !(x.rcl_sub && x.edge == edge);});
  std::vector<Admitted> gone(
    std::make_move_iterator(end), std::make_move_iterator(admitted_.end()));
  admitted_.erase(end, admitted_.end());
  for (const auto & a : gone) {
    try {
      if (wait_set_) {
        wait_set_->remove_waitable(a.waitable);
      }
    } catch (const std::exception &) {
      // Not in the wait set (its add failed): nothing to remove.
    }
    // The fence: a worker's take_corr_now that copied the entry finishes
    // before this returns, and finds none after.
    std::lock_guard<std::mutex> fence(*a.take_mu);
  }
}

bool IngressRole::has_corr_reader_for_test(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(admitted_mu_);
  return std::any_of(
    admitted_.begin(), admitted_.end(),
    [edge](const Admitted & x) {return x.rcl_sub && x.edge == edge;});
}

bool IngressRole::admit_external_subscription(
  int rid, const rclcpp::SubscriptionBase::SharedPtr & sub)
{
  if (!sub || rid < 0) {
    return false;
  }
  ensure_thread();
  Admitted a{"", "", nullptr, nullptr, 0, rid, sub};
  return submit_admit(a, sub->get_topic_name());
}

namespace
{
// How long a caller waits for the ingress thread to apply its request.
// The thread applies requests at the top of every loop pass, which is at most
// one 100 ms wait plus one batch of takes and handler calls away; 2 s is
// twenty such passes, so only a stuck thread misses it.
constexpr auto kAdmitResponseTimeout = std::chrono::seconds(2);
// Wait failures in a row after which the thread gives up. One failure
// can be a transient rcl_wait error (logged, retried); three in a row with
// nothing taken in between mean the wait set itself is broken.
constexpr int kMaxConsecutiveWaitFailures = 3;
}  // namespace

bool IngressRole::submit_admit(
  const Admitted & a, const std::string & name, int64_t retire, bool add)
{
  std::future<bool> result;
  PendingAdmit * mine = nullptr;
  {
    std::lock_guard<std::mutex> lk(admitted_mu_);
    if (!thread_running_.load()) {
      // Stopped, or exited on its own (3 wait errors): nothing would ever
      // take from this entity, so admitting it would be a silent lie. After
      // rclcpp shutdown the refusal is expected and stays quiet.
      if (rclcpp::ok()) {
        RCLCPP_ERROR(
          logger(), "rcllf: ingress could not admit '%s': ingress thread is not running",
          name.c_str());
      }
      return false;
    }
    auto p = std::make_unique<PendingAdmit>();
    p->entry = a;
    p->retire = retire;
    p->add = add;
    result = p->done.get_future();
    mine = p.get();
    pending_.push_back(std::move(p));
  }
  rclcpp::GuardCondition::SharedPtr guard;
  {
    std::lock_guard<std::mutex> tlk(thread_mu_);
    guard = stop_guard_;
  }
  if (guard) {
    guard->trigger();
  }
  if (result.wait_for(kAdmitResponseTimeout) != std::future_status::ready) {
    // The caller is told "not admitted", so the thread must never add it
    // later. Under admitted_mu_ the request is either already answered or
    // still queued: the thread sets each result and frees the queue in one
    // hold of that lock (apply_pending_admits, thread end).
    std::lock_guard<std::mutex> lk(admitted_mu_);
    if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      // Answered: `mine` may be freed and its address reused by another
      // caller's request, so it must not be looked up.
      return result.get();
    }
    auto it = std::find_if(
      pending_.begin(), pending_.end(),
      [mine](const std::unique_ptr<PendingAdmit> & p) {return p.get() == mine;});
    if (it != pending_.end()) {
      if ((*it)->retire >= 0) {
        // A replace keeps its retire (the old reader goes either way), a
        // retire-only request stays as it is: neither adds anything.
        (*it)->add = false;
      } else {
        pending_.erase(it);
      }
      if (add) {
        RCLCPP_ERROR(
          logger(), "rcllf: ingress could not admit '%s': thread not responding within %lld s; "
          "the request is withdrawn", name.c_str(),
          static_cast<long long>(kAdmitResponseTimeout.count()));  // NOLINT(runtime/int)
      }
      return false;
    }
  }
  return result.get();
}

// Caller holds admitted_mu_, and is the ingress thread or no thread runs.
bool IngressRole::add_to_wait_set(const Admitted & a)
{
  if (a.rcl_sub) {
    // By its rcl handle, see RclReaderWaitable.
    if (a.reader.expired()) {
      return false;  // the node dropped it before the thread got here
    }
    try {
      wait_set_->add_waitable(a.waitable);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "rcllf: ingress could not admit a reader: %s", e.what());
      return false;
    }
    admitted_.push_back(a);
    return true;
  }
  // rclcpp::WaitSet (dynamic storage) keeps a weak reference and prunes it.
  const auto sub = a.ext_rid >= 0 ? a.ext_sub.lock() : a.sub;
  if (a.ext_rid >= 0 && !sub) {
    return false;  // the node dropped it before the thread got here
  }
  try {
    if (sub) {
      // Default mask: the subscription, its QoS event handlers and its
      // intra-process waitable. The handlers must be in the set: see
      // take_qos_events.
      wait_set_->add_subscription(sub);
    } else {
      wait_set_->add_service(a.service);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger(), "rcllf: ingress could not admit '%s': %s",
      sub ? sub->get_topic_name() : a.service->get_service_name(), e.what());
    return false;
  }
  admitted_.push_back(a);
  return true;
}

// The binding never executes a physical QoS event (requested deadline
// missed, message lost, incompatible QoS or type): deadline and lost run
// logically at the correspondence drain (corr_runtime.cpp) and the
// others have no reader here. But the event handlers rclcpp creates with the
// subscription (user event_callbacks and the default incompatible-QoS and
// incompatible-type handlers) enable their statuses on the DDS reader's status
// condition, rmw_fastrtps attaches that same condition for the subscription
// itself, and rmw_wait reports OK whenever it is triggered, with nothing to
// take. Only taking the event clears it. Left untaken, one requested-deadline-
// missed event kept this wait set ready for the rest of the run and the thread
// spun at normal priority inside the LF workers' cpuset (35 to 180 takes per
// ms, 1.8 to 8 ms worker wake delay). So: take the ready events and drop them.
// The handler's callback is not run; nothing reaches it physically.
void IngressRole::take_qos_events(const rcl_wait_set_t & ws)
{
  std::lock_guard<std::mutex> lk(admitted_mu_);
  for (const auto & a : admitted_) {
    const auto sub = a.ext_rid >= 0 ? a.ext_sub.lock() : a.sub;
    if (!sub) {
      continue;
    }
    for (const auto & kv : sub->get_event_handlers()) {
      if (kv.second->is_ready(ws)) {
        qos_events_taken_.fetch_add(1);
        (void)kv.second->take_data();  // taken and dropped, never executed
      }
    }
  }
}

void IngressRole::apply_pending_admits()
{
  std::lock_guard<std::mutex> lk(admitted_mu_);
  for (auto & p : pending_) {
    if (p->retire >= 0) {
      retire_corr_reader_locked(static_cast<uint32_t>(p->retire));
    }
    p->done.set_value(p->add ? add_to_wait_set(p->entry) : true);
  }
  pending_.clear();
}

bool IngressRole::admit(const std::string &, const rclcpp::TimerBase::SharedPtr &)
{
  RCLCPP_ERROR(logger(), "rcllf: ingress refused non-infrastructure entity '%s'", "<timer>");
  return false;
}

bool IngressRole::admit(const std::string &, const rclcpp::SubscriptionBase::SharedPtr & s)
{
  RCLCPP_ERROR(
    logger(), "rcllf: ingress refused non-infrastructure entity '%s'",
    s ? s->get_topic_name() : "<subscription>");
  return false;
}

bool IngressRole::admit(const std::string &, const rclcpp::ClientBase::SharedPtr & c)
{
  RCLCPP_ERROR(
    logger(), "rcllf: ingress refused non-infrastructure entity '%s'",
    c ? c->get_service_name() : "<client>");
  return false;
}

bool IngressRole::admit(const std::string &, const rclcpp::Waitable::SharedPtr &)
{
  RCLCPP_ERROR(logger(), "rcllf: ingress refused non-infrastructure entity '%s'", "<waitable>");
  return false;
}

void IngressRole::ensure_thread()
{
  std::lock_guard<std::mutex> lk(thread_mu_);
  if (!wait_set_ && !stop_.load()) {
    // A signal during startup can shut rclcpp down before the
    // first registration; the guard condition and wait set then cannot be
    // created on the dead context. Leave ingress stopped (admits refuse) and
    // let the LF stop that the shutdown requested proceed.
    try {
      stop_guard_ = std::make_shared<rclcpp::GuardCondition>();
      wait_set_ = std::make_unique<rclcpp::WaitSet>();
      wait_set_->add_guard_condition(stop_guard_);
    } catch (const std::exception &) {
      wait_set_.reset();
      stop_guard_.reset();
      if (rclcpp::ok()) {
        throw;
      }
      stop_.store(true);
      return;
    }
  }
  if (thread_.joinable() || stop_.load()) {
    return;
  }
  thread_exited_.store(false);
  thread_running_.store(true);
  thread_ = std::thread([this]() {thread_main();});
}

void IngressRole::thread_main()
{
  set_thread_role(ThreadRole::INGRESS);
  RCLCPP_INFO(
    logger(), "rcllf ingress: thread started (gettid %ld)",
    static_cast<long>(syscall(SYS_gettid)));  // NOLINT(runtime/int)
  int consecutive_throws = 0;
  while (!stop_.load() && rclcpp::ok()) {
    apply_pending_admits();
    bool ready = false;
    try {
      if (forced_wait_failures_.load() > 0) {
        forced_wait_failures_.fetch_sub(1);
        throw std::runtime_error("forced by fail_waits_for_test");
      }
      // The result holds the wait set's references until it is destroyed.
      auto result = wait_set_->wait(std::chrono::milliseconds(100));
      ready = result.kind() == rclcpp::WaitResultKind::Ready;
      if (ready) {
        ready_wakeups_.fetch_add(1);
        take_qos_events(result.get_wait_set().get_rcl_wait_set());
      }
      consecutive_throws = 0;
    } catch (const std::exception & e) {
      if (rclcpp::ok() && !stop_.load()) {
        RCLCPP_WARN(logger(), "rcllf ingress: wait failed: %s", e.what());
      }
      if (++consecutive_throws >= kMaxConsecutiveWaitFailures) {
        // The end is permanent, every later admission is refused, so it
        // is an error. Logged here, inside the loop and only while rclcpp is
        // up, not after the loop (see below).
        if (rclcpp::ok() && !stop_.load()) {
          RCLCPP_ERROR(
            logger(), "rcllf ingress: thread stops after %d consecutive wait failures; every "
            "later admission is refused", kMaxConsecutiveWaitFailures);
        }
        break;
      }
      continue;
    }
    // The wait released its references, so a subscription the node dropped
    // meanwhile is gone now.
    retire_expired_external();
    if (!ready) {
      continue;
    }
    std::vector<Admitted> snapshot;
    {
      std::lock_guard<std::mutex> lk(admitted_mu_);
      snapshot = admitted_;
    }
    for (const auto & a : snapshot) {
      if (a.ext_rid >= 0) {
        if (auto sub = a.ext_sub.lock()) {
          take_external(a.ext_rid, sub, external_batch(*sub));
        }
        continue;
      }
      if (a.corr()) {
        take_corr_samples(a);
        continue;
      }
      while (!stop_.load()) {
        IngressRecord r;
        r.request = a.service->create_request();
        r.header = a.service->create_request_header();
        bool taken = false;
        try {
          taken = a.service->take_type_erased_request(r.request.get(), *r.header);
        } catch (const std::exception & e) {
          RCLCPP_WARN(logger(), "rcllf ingress: take failed: %s", e.what());
        }
        if (!taken) {
          break;
        }
        r.id = next_record_id();
        r.service = a.service;
        r.service_suffix = a.suffix;
        r.t_rx = std::chrono::steady_clock::now();
        on_request(a.key, std::move(r));
      }
    }
  }
  // No rclcpp logging here: the loop also ends because the process is
  // exiting (rclcpp::ok() false after a signal, or reactor-c exit() on RTI
  // loss), when rcutils logging may already be torn down. Measured: a log call
  // here crashed federated pubsub with SIGSEGV in rcutils_logging_format_message.
  {
    std::lock_guard<std::mutex> lk(admitted_mu_);
    // Later admits see thread_running_ false under admitted_mu_ and refuse
    // (submit_admit); the ones still queued are refused here.
    thread_running_.store(false);
    for (auto & p : pending_) {
      p->done.set_value(false);
    }
    pending_.clear();
  }
  thread_exited_.store(true);
}

void IngressRole::retire_expired_external()
{
  // A lifecycle node dropped an external subscription. The node may
  // have created the next one already, while the wait still held the old one
  // and the change_state poke saw it alive, so poke again now.
  bool retired = false;
  {
    std::lock_guard<std::mutex> lk(admitted_mu_);
    const auto end = std::remove_if(
      admitted_.begin(), admitted_.end(),
      [](const Admitted & x) {return x.ext_rid >= 0 && x.ext_sub.expired();});
    retired = end != admitted_.end();
    admitted_.erase(end, admitted_.end());
    // A rebind reader the node dropped. Its records are settled at
    // their drains (READER_REPLACED), so its rcl handle goes now, as stock.
    std::vector<uint32_t> dropped;
    for (const auto & x : admitted_) {
      if (x.rcl_sub && x.reader.expired()) {
        dropped.push_back(x.edge);
      }
    }
    for (const uint32_t e : dropped) {
      retire_corr_reader_locked(e);
    }
  }
  if (retired) {
    external_input::poke_lifecycle();  // no lock held (R16)
  }
}

size_t IngressRole::external_batch(const rclcpp::SubscriptionBase & sub)
{
  const size_t depth = sub.get_actual_qos().depth();
  return depth > 0 ? depth : 1;
}

size_t IngressRole::take_external(
  int rid, const rclcpp::SubscriptionBase::SharedPtr & sub, size_t max_batch)
{
  // At most one batch per wake, so a sustained publisher cannot keep
  // the ingress thread on one subscription. The rest stays queued, and the
  // wait set is ready again at once.
  size_t taken_count = 0;
  while (sub && taken_count < max_batch && !IngressRole::instance().stop_.load()) {
    if (sub.use_count() <= 1) {
      // Only this reference is left: the node dropped the subscription
      // (lifecycle cleanup). Stock takes nothing more from it, neither do we.
      break;
    }
    std::shared_ptr<void> msg = sub->create_message();
    rclcpp::MessageInfo info;
    bool taken = false;
    try {
      taken = sub->take_type_erased(msg.get(), info);
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "rcllf ingress: take failed: %s", e.what());
    }
    if (!taken) {
      break;
    }
    ++taken_count;
    external_input::on_sample(rid, sub, std::move(msg), info);
  }
  return taken_count;
}

void IngressRole::take_corr_samples(const Admitted & a)
{
  // One create_message() plus destructive take per sample; the
  // correspondence table classifies and retains it. Never a user callback.
  // Test pause: announce the loop before reading the flag, so a pause
  // that waits for no loop in flight cannot miss one that already passed it.
  corr_takes_in_flight_.fetch_add(1);
  if (corr_paused_.load()) {
    corr_takes_in_flight_.fetch_sub(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return;
  }
  struct InFlight
  {
    std::atomic<int> & n;
    ~InFlight() {n.fetch_sub(1);}
  } in_flight{corr_takes_in_flight_};
  const size_t batch = corr_batch_of(a);
  const size_t n = take_corr_loop(a, batch, thread_stats_);
  if (corr_loop_end_hooked_.load()) {  // r9 test hook: samples counted, call not yet
    corr_loop_end_hook_(n);
  }
  thread_stats_.record_call(n, n >= batch);
}

size_t IngressRole::corr_batch(bool keep_last, size_t depth)
{
  return keep_last ? std::max<size_t>(depth, 1) : kCorrKeepAllTakeBatch;
}

size_t IngressRole::corr_batch_of(const Admitted & a, bool * keep_last)
{
  // A rebind reader keeps the history read at its admission; a node's
  // own subscription reports the history DDS gave it.
  const bool kl = a.sub ?
    a.sub->get_actual_qos().history() == rclcpp::HistoryPolicy::KeepLast : a.keep_last;
  const size_t depth = a.sub ? a.sub->get_actual_qos().depth() : a.depth;
  if (keep_last != nullptr) {
    *keep_last = kl;
  }
  return corr_batch(kl, depth);
}

void IngressRole::AtomicTakeStats::record_call(uint64_t n, bool full_call)
{
  calls.fetch_add(1);
  if (full_call) {
    full.fetch_add(1);
  }
  uint64_t seen = max.load();
  while (n > seen && !max.compare_exchange_weak(seen, n)) {
  }
}

void IngressRole::AtomicTakeStats::record_sample(const rmw_message_info_t & info)
{
  samples.fetch_add(1);
  // The reception time the rmw stamps when the sample reached the reader
  // (rmw_message_info_t::received_timestamp, system time; 0 when the rmw
  // does not fill it). The delay is that to the take, both read on one host.
  if (info.received_timestamp <= 0) {
    return;
  }
  const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const uint64_t d = now > info.received_timestamp ?
    static_cast<uint64_t>(now - info.received_timestamp) : 0;
  timed.fetch_add(1);
  delay_sum_ns.fetch_add(d);
  uint64_t seen = delay_max_ns.load();
  while (d > seen && !delay_max_ns.compare_exchange_weak(seen, d)) {
  }
}

IngressRole::CorrTakeStats IngressRole::corr_take_stats(bool worker) const
{
  const AtomicTakeStats & a = worker ? worker_stats_ : thread_stats_;
  return {a.calls.load(), a.full.load(), a.max.load(), a.samples.load(), a.timed.load(),
    a.delay_sum_ns.load(), a.delay_max_ns.load()};
}

IngressRole::CorrTakeStats IngressRole::corr_publish_take_stats() const
{
  const AtomicTakeStats & a = publish_stats_;
  return {a.calls.load(), a.full.load(), a.max.load(), a.samples.load(), a.timed.load(),
    a.delay_sum_ns.load(), a.delay_max_ns.load()};
}

void IngressRole::log_corr_batch_summary() const
{
  // Called from ros2_lf_runtime_shutdown, an atexit handler. logger() is a
  // function-local static that may be built after that handler was
  // registered, so exit() destroys it first (LIFO) and logging through it
  // crashes (SIGSEGV in rcutils_logging_format_message). A
  // logger made here has no such lifetime.
  for (const int path : {0, 1, 2}) {
    const CorrTakeStats t = path == 2 ? corr_publish_take_stats() : corr_take_stats(path == 1);
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_lf_binding"),
      "ros2_lf_ingress_batch: path=%s calls=%llu full=%llu max=%llu samples=%llu timed=%llu "
      "delay_mean_us=%.1f delay_max_us=%.1f keep_all_batch=%zu",
      path == 2 ? "publish" : (path == 1 ? "worker" : "thread"),
      static_cast<unsigned long long>(t.calls),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(t.full),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(t.max),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(t.samples),  // NOLINT(runtime/int)
      static_cast<unsigned long long>(t.timed),  // NOLINT(runtime/int)
      t.timed > 0 ? static_cast<double>(t.delay_sum_ns) / static_cast<double>(t.timed) / 1e3 : 0.0,
      static_cast<double>(t.delay_max_ns) / 1e3, kCorrKeepAllTakeBatch);
  }
}

size_t IngressRole::take_corr_loop(
  const Admitted & a, size_t max_count, AtomicTakeStats & stats)
{
  // A worker may take for a local self edge (take_corr_now), and one
  // subscription is never taken from two threads at once. The lock is the
  // subscription's own, and the batch bounds how long one take holds it.
  // The rest stays queued, and the wait set is ready again at once.
  size_t n = 0;
  {
    // A publication holds this mutex under its slot lock, and printing
    // the R13 warnings takes reg_mu: never both at once here.
    corr_rt::WarningsDeferred defer;
    std::lock_guard<std::mutex> take_lk(*a.take_mu);
    n = take_corr_loop_held(a, max_count, stats);
  }
  corr_rt::print_pending_warnings();
  return n;
}

size_t IngressRole::take_corr_loop_held(
  const Admitted & a, size_t max_count, AtomicTakeStats & stats, bool publish)
{
  if (a.rcl_sub) {
    return take_rcl_loop_held(a, max_count, stats);
  }
  const auto & sub = a.sub;
  size_t taken_count = 0;
  while (sub && taken_count < max_count && (publish || !stop_.load())) {
    std::shared_ptr<void> msg = sub->create_message();
    rclcpp::MessageInfo info;
    bool taken = false;
    try {
      taken = sub->take_type_erased(msg.get(), info);
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "rcllf ingress: take failed: %s", e.what());
    }
    if (!taken) {
      break;
    }
    ++taken_count;
    stats.record_sample(info.get_rmw_message_info());
    corr_rt::on_sample(a.edge, std::move(msg), info);
  }
  return taken_count;
}

size_t IngressRole::take_rcl_loop_held(
  const Admitted & a, size_t max_count, AtomicTakeStats & stats)
{
  // As take_corr_loop, through the reader's rcl handle. Nothing more
  // once the node dropped the subscription: stock takes nothing from it either.
  const auto * members = static_cast<const MessageMembers *>(a.members);
  size_t taken_count = 0;
  while (taken_count < max_count && !stop_.load() && !a.reader.expired()) {
    std::shared_ptr<void> msg = make_message(members);
    if (!msg) {
      break;
    }
    rmw_message_info_t info = rmw_get_zero_initialized_message_info();
    const rcl_ret_t ret = rcl_take(a.rcl_sub.get(), msg.get(), &info, nullptr);
    if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
      break;
    }
    if (ret != RCL_RET_OK) {
      RCLCPP_WARN(logger(), "rcllf ingress: take failed: %s", rcl_get_error_string().str);
      rcl_reset_error();
      break;
    }
    ++taken_count;
    stats.record_sample(info);
    corr_rt::on_sample(a.edge, std::move(msg), rclcpp::MessageInfo(info), a.created);
  }
  return taken_count;
}

namespace
{
template<typename Entries>
std::optional<typename Entries::value_type> corr_entry(const Entries & admitted, uint32_t edge)
{
  for (const auto & a : admitted) {
    if (a.serves(edge)) {  // any member of a group
      return a;
    }
  }
  return std::nullopt;
}
}  // namespace

size_t IngressRole::take_corr_now(uint32_t edge, const std::function<bool()> & done)
{
  std::optional<Admitted> found;
  {
    std::lock_guard<std::mutex> lk(admitted_mu_);
    found = corr_entry(admitted_, edge);
  }
  if (!found) {
    return 0;
  }
  // Every sample present at entry precedes the ones written
  // later, and a KEEP_LAST reader holds at most its depth. So a limit of the
  // depth reaches every sample present at entry, and a writer that keeps
  // publishing cannot keep this worker here. A KEEP_ALL reader gets a fixed
  // limit. Each lock-held batch is corr_batch, so a KEEP_LAST
  // reader is taken under one lock hold.
  bool keep_last = true;
  const size_t batch = corr_batch_of(*found, &keep_last);
  const size_t limit = keep_last ? batch : kCorrDrainTakeMax;
  size_t total = 0;
  // The test pause holds the thread, not a worker.
  while (total < limit && !(done && done())) {
    // The last batch is cut to the limit, and the lock is released after each.
    const size_t want = std::min(batch, limit - total);
    const size_t n = take_corr_loop(*found, want, worker_stats_);
    total += n;
    if (n < want) {
      break;  // the reader is empty
    }
  }
  worker_stats_.record_call(total, total >= limit);
  return total;
}

IngressRole::LocalTake IngressRole::lock_local_readers(const std::vector<uint32_t> & edges)
{
  LocalTake held;
  {
    std::lock_guard<std::mutex> lk(admitted_mu_);
    for (uint32_t e : edges) {
      auto found = corr_entry(admitted_, e);
      // A rebind reader is not a local_now edge, its samples wait
      // in DDS until admission. Nothing to hold for it.
      if (!found || !found->sub || found->rcl_sub) {
        continue;
      }
      const bool seen = std::any_of(
        held.readers_.begin(), held.readers_.end(),
        [&found](const Admitted & a) {return a.take_mu == found->take_mu;});
      if (!seen) {
        held.readers_.push_back(*found);
      }
    }
  }
  // One global order: by the edge the reader is admitted on (a group's lead).
  std::sort(
    held.readers_.begin(), held.readers_.end(),
    [](const Admitted & x, const Admitted & y) {return x.edge < y.edge;});
  held.locks_.reserve(held.readers_.size());
  for (const auto & a : held.readers_) {
    held.locks_.emplace_back(*a.take_mu);
  }
  return held;
}

size_t IngressRole::take_local_held(
  LocalTake & held, const std::function<bool(uint32_t)> & done)
{
  size_t total = 0;
  for (const auto & a : held.readers_) {
    bool keep_last = true;
    const size_t batch = corr_batch_of(a, &keep_last);
    const size_t limit = keep_last ? batch : kCorrDrainTakeMax;
    size_t n = 0;
    do {
      // A KEEP_ALL reader is not cut at one batch.
      // Older samples ahead of this publication's would leave it behind and
      // settle_local would end it local_undelivered although it was delivered.
      n = take_corr_loop_held(a, limit, publish_stats_, true);
      publish_stats_.record_call(n, n >= limit);
      total += n;
    } while (!keep_last && n >= limit && !(done && done(a.edge)));
  }
  return total;
}

size_t IngressRole::take_corr_batch_for_test(uint32_t edge)
{
  std::optional<Admitted> found;
  {
    std::lock_guard<std::mutex> lk(admitted_mu_);
    found = corr_entry(admitted_, edge);
  }
  return found ? take_corr_loop(*found, corr_batch_of(*found), thread_stats_) : 0;
}

std::unique_lock<std::mutex> IngressRole::hold_corr_take_for_test(uint32_t edge)
{
  std::lock_guard<std::mutex> lk(admitted_mu_);
  for (const auto & a : admitted_) {
    if (a.serves(edge)) {
      return std::unique_lock<std::mutex>(*a.take_mu);
    }
  }
  return {};
}

void IngressRole::reject(const IngressRecord & r, const std::string & key, const char * why)
{
  trace(key, r, "REJECTED", why);
  send_failure_reply(r, why);
}

// A mutating request that ends without handle_request (REJECTED or TERMINAL)
// still gets an answer, so `ros2 param set` / `ros2 lifecycle set` fail fast
// instead of timing out. Read-only requests are dropped.
void IngressRole::send_failure_reply(const IngressRecord & r, const char * why)
{
  if (!rclcpp::ok()) {
    return;  // the context is gone; nothing can be sent
  }
  const std::string reason = std::string("rcllf ingress: request rejected (") + why + ")";
  if (r.service_suffix == "set_parameters") {
    reply_failure<rcl_interfaces::srv::SetParameters>(
      r, [&reason](const auto & req, auto & resp) {
        for (size_t i = 0; i < req.parameters.size(); ++i) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = false;
          res.reason = reason;
          resp.results.push_back(res);
        }
      });
  } else if (r.service_suffix == "set_parameters_atomically") {
    reply_failure<rcl_interfaces::srv::SetParametersAtomically>(
      r, [&reason](const auto &, auto & resp) {
        resp.result.successful = false;
        resp.result.reason = reason;
      });
  } else if (r.service_suffix == "change_state") {
    reply_failure<lifecycle_msgs::srv::ChangeState>(
      r, [](const auto &, auto & resp) {resp.success = false;});
  }
  // Read-only requests are dropped: the client times out, as with a dead node.
}

bool IngressRole::on_request(const std::string & key, IngressRecord record)
{
  auto mb = mailbox(key, false);
  if (!mb) {
    reject(record, key, "closed");
    return false;
  }
  const char * rejected = nullptr;
  size_t capacity = 0;
  {
    std::lock_guard<std::mutex> lk(mb->mu);
    if (mb->closed) {
      mb->counts.rejected_closed++;
      rejected = "closed";
    } else if (mb->stored.size() >= mb->capacity) {
      mb->counts.rejected_overflow++;
      rejected = "overflow";
      capacity = mb->capacity;
    } else {
      const uint64_t id = record.id;
      trace(key, record, "STORED");
      mb->stored.emplace(id, std::move(record));
      mb->unmoved.push_back(id);
      mb->counts.stored++;
      if (!mb->wake_pending) {
        mb->wake_pending = true;
        mb->counts.wakes++;
        // Lock order: mailbox, then the LF environment mutex inside wake.
        const int h = mb->wake ? mb->wake(mb->action) : 0;
        if (h <= 0) {
          close_locked(*mb, key, "schedule_refused", false);
        }
      }
    }
  }
  if (capacity > 0) {
    // An overflow is a configuration signal, so it is logged
    // whether or not RCLLF_TRACE_CALLBACKS is set.
    RCLCPP_WARN(
      logger(), "rcllf ingress: key '%s' holds %zu STORED request(s) (ingress_capacity %zu); "
      "request %llu on '%s' REJECTED (overflow)", key.c_str(), capacity, capacity,
      static_cast<unsigned long long>(record.id),  // NOLINT(runtime/int)
      record.service_suffix.c_str());
  }
  if (rejected != nullptr) {
    reject(record, key, rejected);
    return false;
  }
  return true;
}

bool IngressRole::move(const std::string & key)
{
  auto mb = mailbox(key, false);
  if (!mb) {
    return false;
  }
  std::lock_guard<std::mutex> lk(mb->mu);
  if (mb->closed) {
    return false;
  }
  while (!mb->unmoved.empty()) {
    mb->carrier.push_back(mb->unmoved.front());
    mb->unmoved.pop_front();
  }
  assert(mb->carrier.size() <= mb->capacity);
  if (mb->carrier.empty()) {
    mb->wake_pending = false;
    return false;
  }
  return true;
}

bool IngressRole::drain_one(const std::string & key)
{
  auto mb = mailbox(key, false);
  if (!mb) {
    return false;
  }
  IngressRecord rec;
  {
    std::lock_guard<std::mutex> lk(mb->mu);
    if (mb->closed) {
      mb->counts.stale_drains++;
      return false;
    }
    if (mb->carrier.empty()) {
      while (!mb->unmoved.empty()) {
        mb->carrier.push_back(mb->unmoved.front());
        mb->unmoved.pop_front();
      }
    }
    if (mb->carrier.empty()) {
      mb->wake_pending = false;
      return false;
    }
    const uint64_t id = mb->carrier.front();
    mb->carrier.pop_front();
    auto it = mb->stored.find(id);
    if (it == mb->stored.end()) {
      mb->counts.stale_drains++;
      const bool more = !mb->carrier.empty() || !mb->unmoved.empty();
      mb->wake_pending = more;
      return more;
    }
    rec = std::move(it->second);
    mb->stored.erase(it);
  }

  const char * terminal = nullptr;
  std::string detail;
  UserCallbackScope user;  // none after shutdown began
  if (!require_lf_worker("ros2_lf_ingress_drain", -1)) {
    terminal = "not_lf";
  } else if (!rclcpp::ok() || !user.admitted()) {
    terminal = "not_ok";
  } else {
    lf_time_advance_ros_clock();
    trace_callback_start(
      "ros2_lf_ingress_drain", -1, key, rec.service_suffix, static_cast<int64_t>(rec.id));
    {  // the same callback start as an rcllf LTTng event
      const auto tag = lf_current_tag();
      rcllf_tp_ingress_callback_start ev{};
      ev.entry = "ros2_lf_ingress_drain";
      ev.rid = -1;
      ev.node = key.c_str();
      ev.callback = rec.service_suffix.c_str();
      ev.lf_thread_id = executable_lf_thread_id();
      ev.gettid = static_cast<int64_t>(syscall(SYS_gettid));
      ev.tag_time = tag.first;
      ev.tag_microstep = tag.second;
      ev.ingress_record_id = rec.id;
      semtrace::ingress_callback_start(ev);
    }
    try {
      if (rec.local) {
        rec.local();
      } else {
        rec.service->handle_request(rec.header, rec.request);
      }
    } catch (const std::exception & e) {
      terminal = "exception";
      detail = e.what();
      RCLCPP_ERROR(
        logger(), "rcllf ingress: %s request %llu failed: %s", rec.service_suffix.c_str(),
        static_cast<unsigned long long>(rec.id), e.what());  // NOLINT(runtime/int)
    } catch (...) {
      terminal = "exception";
      detail = "non-std exception";
      RCLCPP_ERROR(
        logger(), "rcllf ingress: %s request %llu failed: non-std exception",
        rec.service_suffix.c_str(), static_cast<unsigned long long>(rec.id));  // NOLINT
    }
  }

  if (terminal != nullptr) {
    send_failure_reply(rec, terminal);
  } else if (rec.service_suffix == "change_state") {
    // The transition may have created an external subscription. Its
    // own reaction binds it late, as for every lifecycle handle.
    external_input::poke_lifecycle();
  }
  std::lock_guard<std::mutex> lk(mb->mu);
  if (terminal != nullptr) {
    mb->counts.terminal++;
    trace(key, rec, "TERMINAL", std::string(terminal) + (detail.empty() ? "" : ": " + detail));
  } else {
    mb->counts.consumed++;
    trace(key, rec, "CONSUMED");
  }
  if (mb->closed) {
    return false;
  }
  const bool more = !mb->carrier.empty() || !mb->unmoved.empty();
  if (!more) {
    mb->wake_pending = false;
  }
  return more;
}

void IngressRole::close_locked(
  NodeMailbox & mb, const std::string & key, const std::string & reason, bool clear_carrier)
{
  if (mb.closed) {
    return;
  }
  mb.closed = true;
  mb.wake_pending = false;
  mb.close_reason = reason;
  for (auto & kv : mb.stored) {
    mb.counts.terminal++;
    trace(key, kv.second, "TERMINAL", reason);
    send_failure_reply(kv.second, reason.c_str());
  }
  mb.stored.clear();
  mb.unmoved.clear();
  if (clear_carrier) {
    mb.carrier.clear();
  }
}

void IngressRole::schedule_refused(const std::string & key)
{
  close(key, "schedule_refused");
}

void IngressRole::close(const std::string & key, const char * reason)
{
  auto mb = mailbox(key, true);
  {
    std::lock_guard<std::mutex> lk(mb->mu);
    close_locked(*mb, key, reason, true);
  }
  bool all_closed = true;
  {
    std::lock_guard<std::mutex> lk(boxes_mu_);
    for (auto & kv : boxes_) {
      std::lock_guard<std::mutex> mlk(kv.second->mu);
      if (kv.second->registered && !kv.second->closed) {
        all_closed = false;
      }
    }
  }
  if (!all_closed) {
    return;
  }
  // The samples still in flight at the stop are taken by the
  // runtime's stop drain on its own thread (corr_rt::drain_in_flight), so the
  // thread is not needed past the last close; the exit backstop would stop it
  // before that drain runs anyway.
  stop_and_join();
}

void IngressRole::stop_and_join(bool log)
{
  std::lock_guard<std::mutex> lk(thread_mu_);
  stop_.store(true);
  if (thread_.joinable()) {
    if (stop_guard_) {
      stop_guard_->trigger();
    }
    thread_.join();
    if (log) {
      RCLCPP_INFO(logger(), "rcllf ingress: thread joined");
    }
  }
  thread_running_.store(false);
  {
    std::lock_guard<std::mutex> alk(admitted_mu_);
    admitted_.clear();
  }
  wait_set_.reset();
  stop_guard_.reset();
}

void IngressRole::stop_backstop()
{
  if (!thread_running_.load()) {
    return;
  }
  // No mailbox lock (R16): reactor-c may exit() holding env->mutex while the
  // ingress thread waits for it inside wake() under a mailbox lock.
  stop_.store(true);
  if (stop_guard_) {
    stop_guard_->trigger();
  }
  const auto deadline = std::chrono::steady_clock::now() + backstop_deadline_;
  while (!thread_exited_.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!thread_exited_.load()) {
    RCLCPP_FATAL(
      logger(), "rcllf ingress: thread did not stop within %lld ms at exit; terminating so "
      "reactor-c termination never frees environments under a live ingress thread",
      static_cast<long long>(backstop_deadline_.count()));  // NOLINT(runtime/int)
    std::_Exit(EXIT_FAILURE);
  }
  stop_and_join(false);  // exit path: rcutils logging may be torn down
}

void IngressRole::register_node(
  const std::string & key, IngressWakeFn wake, void * action,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base)
{
  auto mb = mailbox(key, true);
  {
    std::lock_guard<std::mutex> lk(mb->mu);
    mb->registered = true;
    mb->wake = wake;
    mb->action = action;
  }
  ensure_thread();
  int admitted = 0;
  if (node_base) {
    const std::string fqn = node_base->get_fully_qualified_name();
    const std::string prefix = (fqn == "/" ? "" : fqn) + "/";
    node_base->for_each_callback_group([&](rclcpp::CallbackGroup::SharedPtr group) {
        group->collect_all_ptrs(
          [](const rclcpp::SubscriptionBase::SharedPtr &) {},
          [&](const rclcpp::ServiceBase::SharedPtr & s) {
            const std::string name = s->get_service_name();
            if (name.compare(0, prefix.size(), prefix) != 0 ||
            !ingress_suffix_allowed(name.substr(prefix.size())))
            {
              return;  // user service: bound as srv_server, not ingress
            }
            if (admit(key, fqn, s)) {
              admitted++;
            }
          },
          [](const rclcpp::ClientBase::SharedPtr &) {},
          [](const rclcpp::TimerBase::SharedPtr &) {},
          [](const rclcpp::Waitable::SharedPtr &) {});
      });
    RCLCPP_INFO(
      logger(), "rcllf ingress: node '%s' (key '%s'): %d infrastructure service(s) admitted, "
      "ingress_capacity %zu", fqn.c_str(), key.c_str(), admitted, capacity(key));
  }
  std::lock_guard<std::mutex> lk(mb->mu);
  if (!mb->closed && !mb->stored.empty() && !mb->wake_pending) {
    mb->wake_pending = true;
    mb->counts.wakes++;
    if ((mb->wake ? mb->wake(mb->action) : 0) <= 0) {
      close_locked(*mb, key, "schedule_refused", false);
    }
  }
}

void IngressRole::set_capacity(const std::string & key, size_t capacity)
{
  auto mb = mailbox(key, true);
  std::lock_guard<std::mutex> lk(mb->mu);
  mb->capacity = capacity;
}

size_t IngressRole::capacity(const std::string & key)
{
  auto mb = mailbox(key, false);
  if (!mb) {
    return kIngressCap;
  }
  std::lock_guard<std::mutex> lk(mb->mu);
  return mb->capacity;
}

IngressCounts IngressRole::counts(const std::string & key)
{
  auto mb = mailbox(key, false);
  if (!mb) {
    return {};
  }
  std::lock_guard<std::mutex> lk(mb->mu);
  return mb->counts;
}

bool IngressRole::is_closed(const std::string & key)
{
  auto mb = mailbox(key, false);
  if (!mb) {
    return false;
  }
  std::lock_guard<std::mutex> lk(mb->mu);
  return mb->closed;
}

bool IngressRole::wake_pending(const std::string & key)
{
  auto mb = mailbox(key, false);
  if (!mb) {
    return false;
  }
  std::lock_guard<std::mutex> lk(mb->mu);
  return mb->wake_pending;
}

size_t IngressRole::carrier_size(const std::string & key)
{
  auto mb = mailbox(key, false);
  if (!mb) {
    return 0;
  }
  std::lock_guard<std::mutex> lk(mb->mu);
  return mb->carrier.size();
}

size_t IngressRole::admitted_count()
{
  std::lock_guard<std::mutex> lk(admitted_mu_);
  return admitted_.size();
}

void IngressRole::reset_for_test()
{
  stop_and_join();
  {
    std::lock_guard<std::mutex> lk(boxes_mu_);
    boxes_.clear();
  }
  std::lock_guard<std::mutex> lk(thread_mu_);
  stop_.store(false);
  thread_exited_.store(false);
  ready_wakeups_.store(0);
  qos_events_taken_.store(0);
  forced_wait_failures_.store(0);
  backstop_deadline_ = std::chrono::milliseconds(500);
}

}  // namespace ros2_lf_binding

extern "C" {

void ros2_lf_ingress_set_capacity(const char * key, int capacity)
{
  if (capacity < 1) {
    RCLCPP_WARN(
      ros2_lf_binding::logger(),
      "ros2_lf_ingress_set_capacity: key '%s': capacity %d < 1 ignored, keeping %zu", key,
      capacity, ros2_lf_binding::IngressRole::instance().capacity(key));
    return;
  }
  ros2_lf_binding::IngressRole::instance().set_capacity(key, static_cast<size_t>(capacity));
}

void ros2_lf_ingress_register(void * /*self*/, const char * key, int (*wake)(void *), void * action)
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  if (ros2_lf_binding::g_manager) {
    base = ros2_lf_binding::g_manager->find_by_class(key);
  }
  if (!base) {
    base = ros2_lf_get_node(key);
  }
  if (!base) {
    RCLCPP_WARN(
      ros2_lf_binding::logger(), "ros2_lf_ingress_register: no loaded node matches key '%s'", key);
  }
  ros2_lf_binding::IngressRole::instance().register_node(key, wake, action, base);
}

bool ros2_lf_ingress_move(const char * key)
{
  return ros2_lf_binding::IngressRole::instance().move(key);
}

bool ros2_lf_ingress_drain_one(void * /*self*/, const char * key)
{
  return ros2_lf_binding::IngressRole::instance().drain_one(key);
}

void ros2_lf_ingress_schedule_refused(const char * key)
{
  ros2_lf_binding::IngressRole::instance().schedule_refused(key);
}

void ros2_lf_ingress_close(const char * key)
{
  // This runs in a reaction(shutdown), at the stop tag.
  ros2_lf_binding::plant_record::lf_stop("shutdown_reaction");
  ros2_lf_binding::IngressRole::instance().close(key, "shutdown");
}

void ros2_lf_ingress_stop_atexit(void)
{
  ros2_lf_binding::IngressRole::instance().stop_backstop();
}

}  // extern "C"
