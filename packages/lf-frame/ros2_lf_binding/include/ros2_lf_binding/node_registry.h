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

#ifndef ROS2_LF_BINDING__NODE_REGISTRY_H_
#define ROS2_LF_BINDING__NODE_REGISTRY_H_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

/// Type-erased registration entry point (src/node_registry.cpp).
///
/// Every registry consumer only ever needs `NodeBaseInterface` (either
/// directly, or through `Executor::add_node`'s `NodeBaseInterface::SharedPtr`
/// overload), so the registry stores that plus an owning `shared_ptr<void>`
/// to keep the concrete node alive — holding only the base interface does
/// not extend the node's lifetime.  This lets the registry accept any node
/// type without depending on that type's header: `rclcpp_lifecycle::
/// LifecycleNode` is a sibling of `rclcpp::Node` (both wrap `NodeBaseInterface`),
/// not a subclass, so a registry hard-typed to `rclcpp::Node::SharedPtr`
/// cannot accept it.
void ros2_lf_register_node_erased(
  const std::string & class_name,
  std::shared_ptr < void > owner,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base);

/// Make a node's RCL_ROS_TIME clock follow LF logical time.
///
/// Enables the ROS time override on `clock` and sets it to the current LF
/// logical time before every reaction, together with the binding's own clock.
/// Timers created on that clock share its handle and follow it too. Defined in
/// src/time_source.cpp; `key` only names the node in error messages.
void ros2_lf_attach_node_clock(std::shared_ptr < rclcpp::Clock > clock, const std::string & key);

/// Take a node that set use_sim_time=true itself off /clock.
///
/// Sets use_sim_time=false and hands the node's TimeSource the matching
/// parameter event synchronously, since nothing spins its /parameter_events
/// subscription under LF. The TimeSource then joins its clock thread and
/// destroys its /clock subscription, so no late /clock sample can set the
/// clock back. Returns false, after logging an error, when any step fails or
/// the /clock subscription survives. Defined in src/time_source.cpp.
bool ros2_lf_detach_sim_time(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
  const std::string & key);

/// A node's managed-lifecycle bring-up, type-erased at registration time.
///
/// Returns true when the node reached the active state. Empty for a plain
/// `rclcpp::Node`, which has no lifecycle to drive.
using Ros2LfTransitionFn = std::function < bool () >;

/// Overload taking a transition thunk; see `ros2_lf_register_node` below for
/// why the thunk has to be captured at registration time.
void ros2_lf_register_node_erased(
  const std::string & class_name,
  std::shared_ptr < void > owner,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  Ros2LfTransitionFn transition);

/// A node's current primary lifecycle state ID (`lifecycle_msgs::msg::State`),
/// type-erased at registration. Empty for a plain `rclcpp::Node`.
using Ros2LfStateFn = std::function < uint8_t () >;

/// Overload also taking the state probe, which binding uses to defer the
/// handles of a lifecycle node that is not active yet.
void ros2_lf_register_node_erased(
  const std::string & class_name,
  std::shared_ptr < void > owner,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  Ros2LfTransitionFn transition,
  Ros2LfStateFn state);

/// Overload also naming the system.json class the node instantiates, which
/// differs from `binding_key` when the class has two or more instances
/// (`<class>__<suffix>`). The overloads without it record the
/// key as the class.
void ros2_lf_register_node_erased(
  const std::string & binding_key,
  const std::string & node_class,
  std::shared_ptr < void > owner,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base,
  Ros2LfTransitionFn transition,
  Ros2LfStateFn state);

/// Typed cleanup of a client when its lifecycle node leaves the active state.
/// The binding holds clients type erased, and rclcpp's prune
/// and cancel calls need the type, so the generated node factories register
/// one functor per service or action type the analysis found. A functor
/// returns -1 for a client of another type, else the number of pending
/// requests it pruned.
using Ros2LfClientCleanup = std::function < int64_t(rclcpp::ClientBase &) >;

/// Register a service client cleanup, see Ros2LfClientCleanup.
void ros2_lf_register_client_cleanup(Ros2LfClientCleanup cleanup);

/// A goal id (unique_identifier_msgs/UUID).
using Ros2LfGoalId = std::array < uint8_t, 16 >;

/// Typed operations on one rclcpp_action::Client<T> type,
/// registered by the generated node factories once per qualified action type
/// of a lifecycle node. Each returns false for a client of another type.
struct Ros2LfActionClientOps
{
  // cpplint reads `bool(` in a std::function signature as a C-style cast.
  /// Reads the goal id of a request `client` sends: a SendGoal request, or a
  /// GetResult request when `result` is true. Arguments: client, request,
  /// result, id.
  std::function < bool(  // NOLINT(readability/casting)
    rclcpp::Waitable &, const void *, bool, Ros2LfGoalId &) > goal_id;
  /// Reads `accepted` of a SendGoal response `client` took. Arguments:
  /// client, response, accepted.
  std::function < bool(  // NOLINT(readability/casting)
    rclcpp::Waitable &, const void *, bool &) > goal_accepted;
  /// Deactivate: `stop_callbacks(id)` for each of `ids`, then
  /// `async_cancel_all_goals()`. Arguments: client, ids.
  std::function < bool(  // NOLINT(readability/casting)
    rclcpp::Waitable &, const std::vector < Ros2LfGoalId > &) > deactivate;
};

/// Register the operations of one action client type, see Ros2LfActionClientOps.
void ros2_lf_register_action_client_ops(Ros2LfActionClientOps ops);

/// The primary lifecycle state ID of a registered lifecycle node, or -1 for a
/// node without a lifecycle or one not in the registry.
int ros2_lf_node_lifecycle_state(const std::string & class_name);

/// Run a registered node's lifecycle bring-up, if it has one.
///
/// Returns true when the node is ready to be bound: either it has no
/// lifecycle (nothing to do) or its transitions succeeded. Returns false only
/// when a lifecycle node's configure/activate actually failed.
bool ros2_lf_transition_node(const std::string & class_name);

/// Detects `configure()` on NodeT, which only `rclcpp_lifecycle::LifecycleNode`
/// has. Used instead of naming the lifecycle type so `ros2_lf_binding` keeps
/// its independence from `rclcpp_lifecycle` (the sibling-not-subclass
/// problem); the generated TU that instantiates this template already
/// includes the lifecycle header where relevant.
///
/// Written as an overloaded probe rather than a `std::void_t` partial
/// specialization because ament_uncrustify parses this header as C and
/// mis-indents the whole file after such a specialization.
template < typename NodeT >
auto ros2_lf_detail_probe_lifecycle(int)->decltype(
  std::declval < NodeT & > ().configure(), std::true_type {});

template < typename NodeT >
std::false_type ros2_lf_detail_probe_lifecycle(...);

template < typename NodeT >
using ros2_lf_detail_has_lifecycle = decltype(ros2_lf_detail_probe_lifecycle < NodeT > (0));

/// Primary lifecycle state IDs, from `lifecycle_msgs::msg::State`.
///
/// Spelled as local constants rather than by including `lifecycle_msgs` so
/// this header stays usable by translation units that know nothing about the
/// lifecycle stack. The values are ABI-stable message constants.
constexpr uint8_t ros2_lf_detail_kPrimaryStateUnconfigured = 1u;
constexpr uint8_t ros2_lf_detail_kPrimaryStateInactive = 2u;
constexpr uint8_t ros2_lf_detail_kPrimaryStateActive = 3u;

/// Builds the configure+activate thunk for a lifecycle node.
///
/// Captures a weak_ptr, never a shared_ptr: the registry already owns the node
/// through its `owner` entry, so a strong capture here would make the entry
/// keep itself alive forever.
///
/// Overload-resolved on the lifecycle probe rather than written as one
/// function with `if constexpr`, because ament_uncrustify parses this header
/// as C and rejects `if constexpr` outright.
template < typename NodeT >
Ros2LfTransitionFn ros2_lf_detail_make_transition_fn(
  const std::shared_ptr < NodeT > &node, std::true_type)
{
  std::weak_ptr < NodeT > weak = node;
  return [weak]()->bool {
           auto n = weak.lock();
           if (!n) {
             return false;
           }
           // Drive only the transitions the node is actually missing, rather
           // than assuming it starts `unconfigured`. rcl_lifecycle
           // answers an illegal transition with a WARN and the UNCHANGED
           // current state, never an exception, so firing configure()
           // unconditionally has two costs on a node some supervisor already
           // moved: it logs noise, and a failed configure() returning the
           // unchanged `inactive` is indistinguishable from a successful one.
           //
           // Falling through the two ifs also gives the right answer for
           // `active` (nothing to do) and for `finalized` / `unknown`
           // (unreachable, correctly reported as failure).
           uint8_t state = n->get_current_state().id();
           if (state == ros2_lf_detail_kPrimaryStateUnconfigured) {
             state = n->configure().id();
           }
           if (state == ros2_lf_detail_kPrimaryStateInactive) {
             state = n->activate().id();
           }
           return state == ros2_lf_detail_kPrimaryStateActive;
  };
}

template < typename NodeT >
Ros2LfTransitionFn ros2_lf_detail_make_transition_fn(
  const std::shared_ptr < NodeT > &, std::false_type)
{
  return Ros2LfTransitionFn {};
}

/// Builds the state probe for a lifecycle node. Weak capture, as above.
template < typename NodeT >
Ros2LfStateFn ros2_lf_detail_make_state_fn(
  const std::shared_ptr < NodeT > &node, std::true_type)
{
  std::weak_ptr < NodeT > weak = node;
  return [weak]()->uint8_t {
           auto n = weak.lock();
           return n ? n->get_current_state().id() : 0u;
  };
}

template < typename NodeT >
Ros2LfStateFn ros2_lf_detail_make_state_fn(
  const std::shared_ptr < NodeT > &, std::false_type)
{
  return Ros2LfStateFn {};
}

/// Dispatches on whether `NodeT` has a managed lifecycle.
template < typename NodeT >
Ros2LfTransitionFn ros2_lf_detail_make_transition_fn(const std::shared_ptr < NodeT > &node)
{
  return ros2_lf_detail_make_transition_fn < NodeT > (
    node, ros2_lf_detail_has_lifecycle < NodeT > {});
}

/// Register a constructed non-component node under its binding key, recording
/// the system.json class it instantiates (`node_class`). The two differ when the
/// class has two or more instances (`<class>__<suffix>`).
///
/// Header-only so the generated translation unit (which already includes the
/// user's node header, `rclcpp_lifecycle` included where relevant) can
/// register any node type without `ros2_lf_binding` itself depending on
/// `rclcpp_lifecycle`. `NodeT` only needs `get_node_base_interface()` —
/// `rclcpp::Node` and `rclcpp_lifecycle::LifecycleNode` both provide it,
/// despite being siblings rather than sharing an inheritance relationship.
/// This is also the last point at which `NodeT` is still concrete, so it is
/// where a lifecycle node's configure/activate thunk has to be captured: the
/// registry stores only `NodeBaseInterface`, which does not expose the managed
/// lifecycle interface at all.
///
/// `auto_transition` false declines the automatic bring-up entirely, for a node
/// whose activation some supervisor, state machine or operator owns.
/// No thunk is built, so `ros2_lf_transition_node` treats the node like a
/// plain one and binding sees whatever entities exist when it runs. The flag is
/// resolved at generation time from `execution.yaml`'s per-node
/// `auto_transition` key and baked into the generated factory, so nothing has
/// to be decided here at runtime.
template < typename NodeT >
void ros2_lf_register_node(
  const std::string & binding_key, const std::string & node_class,
  std::shared_ptr < NodeT > node, bool auto_transition = true)
{
  auto base = node ? node->get_node_base_interface() : nullptr;
  if (node) {
    // The concrete type is visible only here, so this is where the node's
    // clock is handed to the LF time source. A node that sets
    // use_sim_time=true itself has a /clock subscription that its TimeSource
    // spins on its own thread and that could set the clock back to a late
    // /clock sample, so it is taken off /clock first, or refused.
    bool use_sim_time = false;
    if (node->get_parameter("use_sim_time", use_sim_time) && use_sim_time &&
      !ros2_lf_detach_sim_time(base, node->get_node_parameters_interface(), binding_key))
    {
      throw std::runtime_error(
        "ros2_lf_register_node: node '" + binding_key + "' sets use_sim_time=true and "
        "could not be taken off /clock, so its now() would not follow LF logical time");
    }
    ros2_lf_attach_node_clock(node->get_clock(), binding_key);
  }
  auto transition = (node && auto_transition) ?
    ros2_lf_detail_make_transition_fn < NodeT > (node) :
    Ros2LfTransitionFn {};
  // The state probe exists whatever auto_transition says: a node whose
  // activation an operator owns is the one that needs deferred binding.
  auto state = node ?
    ros2_lf_detail_make_state_fn < NodeT > (node, ros2_lf_detail_has_lifecycle < NodeT > {}) :
    Ros2LfStateFn {};
  ros2_lf_register_node_erased(
    binding_key, node_class, std::move(node), std::move(base), std::move(transition),
    std::move(state));
}

/// Register a node whose binding key is its class name (one instance, or a
/// hand-written registration). Generated factories name the class explicitly
/// through the overload above.
template < typename NodeT >
void ros2_lf_register_node(
  const std::string & class_name, std::shared_ptr < NodeT > node,
  bool auto_transition = true)
{
  ros2_lf_register_node < NodeT > (class_name, class_name, std::move(node), auto_transition);
}


/// Register a constructed non-component `rclcpp::Node`.
/// Called from generated node_factories.cpp before ros2_lf_runtime_init
/// returns. Non-template overload kept for source/binary compatibility with
/// existing callers holding a plain `rclcpp::Node::SharedPtr`; delegates to
/// the template above rather than duplicating its body. Prefer the template
/// overload for any other node base (e.g. `rclcpp_lifecycle::LifecycleNode`).
inline void ros2_lf_register_node(
  const std::string & class_name, rclcpp::Node::SharedPtr node,
  bool auto_transition = true)
{
  ros2_lf_register_node < rclcpp::Node > (class_name, std::move(node), auto_transition);
}

/// Names the node a constructor creates subscriptions for.
///
/// The generated node factories and the component loader hold one of these
/// around each node's construction, so the subscription gate can tell which
/// binding key a new subscription belongs to. The subscription of an admitted
/// edge gets a DDS reader deep enough for ingress to take every sample of a
/// writer that publishes more than once before ingress runs.
class Ros2LfNodeConstructionScope
{
public:
  explicit Ros2LfNodeConstructionScope(const std::string & binding_key);
  ~Ros2LfNodeConstructionScope();
  Ros2LfNodeConstructionScope(const Ros2LfNodeConstructionScope &) = delete;
  Ros2LfNodeConstructionScope & operator = (const Ros2LfNodeConstructionScope &) = delete;

private:
  std::string previous_;
};

/// Look up a registered non-component node's base interface by C++ class name.
/// Returns nullptr if not found. Returns the base interface (not the
/// concrete node) because that is all any consumer needs, and it is the only
/// type both `rclcpp::Node` and `rclcpp_lifecycle::LifecycleNode` share.
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ros2_lf_get_node(
  const std::string & class_name);

/// Replace every `--rcllf-params-yaml <yaml>` pair in a node's `--ros-args`
/// with `--params-file <path>` of a temp file holding the YAML.
///
/// The generator files a launch `parameters=[...]` dict this way so the
/// dict keeps its place among the parameter files, exactly as launch_ros
/// writes each dict to a temp file keyed by the node's full name. A later
/// inline value then wins over an earlier file naming the node, which a `-p`
/// rule (always under `/**`) cannot do. Every other argument passes through.
/// Defined in src/node_registry.cpp; throws std::runtime_error when the pair
/// is incomplete or the temp file cannot be written. Like launch_ros, the
/// files are left in the temp directory.
std::vector < std::string > ros2_lf_materialize_param_args(
  const std::vector < std::string > &args);

#endif  // ROS2_LF_BINDING__NODE_REGISTRY_H_
