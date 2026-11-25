# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Data models for the node analysis layer.

All dataclasses are JSON-serializable (Path fields serialized as strings).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


# ---------------------------------------------------------------------------
# Managed-lifecycle detection
# ---------------------------------------------------------------------------

# Exact base-class spellings for a managed lifecycle node.
LIFECYCLE_BASES = frozenset({"rclcpp_lifecycle::LifecycleNode", "LifecycleNode"})

# Substrings that mark a lifecycle base even when the header chain cannot be
# fully resolved, e.g. `nav2_util::LifecycleNode` contains `LifecycleNode`.
LIFECYCLE_BASE_SUBSTRINGS = ("LifecycleNode",)

# Lifecycle hooks whose base implementation does work an override must not
# skip. `LifecycleNode` overrides exactly these two
# (rclcpp_lifecycle/lifecycle_node.hpp): `on_activate` calls
# `impl_->on_activate()` to activate every managed publisher, and
# `on_deactivate` is its mirror. The other hooks
# (`on_configure`/`on_cleanup`/`on_shutdown`/`on_error`) are no-ops on
# `LifecycleNodeInterface`, so not chaining them is harmless and must not be
# reported.
LIFECYCLE_CHAINED_HOOKS = ("on_activate", "on_deactivate")


def is_lifecycle_base(base_class: str, lifecycle_callbacks: list[str] | None = None) -> bool:
    """Whether *base_class* denotes a managed lifecycle node.

    One definition, shared by the parser (which needs it before a `NodeInfo`
    exists, to decide whether to scan `on_configure` / `on_activate` bodies)
    and by generation (which needs it to keep a lifecycle node off the
    component path). Duplicating the predicate is how the two
    would drift, and a drift here is silent: generation would route a node the
    parser had already treated as a lifecycle node.

    Deliberately broad. The substring arm catches wrapper bases like
    `nav2_util::LifecycleNode`, and the callbacks arm is the fallback for an
    unresolved base spelling, which happens whenever a third-party header is
    missing.
    """
    if base_class in LIFECYCLE_BASES:
        return True
    if any(s in base_class for s in LIFECYCLE_BASE_SUBSTRINGS):
        return True
    # Unresolved base: infer from the presence of lifecycle transition hooks.
    return not base_class and bool(lifecycle_callbacks)


# ---------------------------------------------------------------------------
# Callback layer
# ---------------------------------------------------------------------------

@dataclass
class CallbackParameter:
    """One input parameter of a ROS2 callback function."""

    name: str   # e.g. "msg", "request", "response", "goal", "uuid"
    type: str   # Fully-qualified C++ type, e.g.
                #   "custom_interfaces::msg::SensorData::SharedPtr"
                #   "custom_interfaces::srv::ProcessData::Request::SharedPtr"
                #   "custom_interfaces::action::Analyze::Goal::ConstSharedPtr"


@dataclass
class CallbackInfo:
    """A callback function inside a ROS2 node.

    Separates *what the callback receives* (parameters) from
    *what it does* (publishers_used, variables, etc.).
    """

    name: str                               # e.g. "timer_callback", "topic_callback"
    parameters: list[CallbackParameter] = field(default_factory=list)
                                            # [] for timer callbacks
    publishers_used: list[str] = field(default_factory=list)
                                            # Topics called via publish()
    service_clients_used: list[str] = field(default_factory=list)
                                            # Service names of async_send_request calls
    action_clients_used: list[str] = field(default_factory=list)
                                            # Action names of async_send_goal calls
    variables_read: list[str] = field(default_factory=list)
                                            # Member variables read inside this callback
    variables_written: list[str] = field(default_factory=list)
                                            # Member variables written inside this callback
    spawns_callbacks: list[str] = field(default_factory=list)
                                            # Callback names spawned via std::thread/std::async
    feedback_published: list[str] = field(default_factory=list)
                                            # Action names where publish_feedback() is called
    result_published: list[str] = field(default_factory=list)
                                            # Action names where succeed()/canceled() is called
    # The most publishes one invocation makes per topic,
    # proven from the call sites (an upper bound when some are conditional).
    # None: no bound, the reason is in publish_unbounded. A topic missing from
    # both was not analyzed, so its bound is unknown.
    publish_bounds: dict[str, int | None] = field(default_factory=dict)
    publish_unbounded: dict[str, str] = field(default_factory=dict)
    # Topics published by the subscription's requested-deadline-missed
    # handler. The binding runs that handler in the drain reaction of this
    # callback's subscription, so its publishes are effects of this callback.
    missing_handler_publishers: list[str] = field(default_factory=list)
    # The node's own functions whose bodies call this method,
    # a lambda callback by its synthesized name. A call from a constructor,
    # destructor or any other lambda, and a use that is not a call, is a
    # ``<...>`` entry. None: the set is not closed (the method is not private,
    # is virtual, or the class has a friend or a method without a visible
    # definition), so a caller may exist that the analysis cannot see.
    callers: list[str] | None = None


# ---------------------------------------------------------------------------
# Callback groups (trigger property, not callback property)
# ---------------------------------------------------------------------------

@dataclass
class CallbackGroupInfo:
    name: str   # C++ member variable name, e.g. "exclusive_group_"
    type: str   # "MutuallyExclusive" | "Reentrant"


# ---------------------------------------------------------------------------
# Triggers (invoke callbacks; callback_group is a trigger property)
# ---------------------------------------------------------------------------

@dataclass
class TimerInfo:
    period_ms: float | None             # None if period couldn't be resolved
    callback: str                       # Reference to CallbackInfo.name
    callback_group: str | None = None   # Reference to CallbackGroupInfo.name
    member_variable: str | None = None  # e.g. "timer_"
    created_in: str | None = None       # "constructor" | "on_configure" | "on_activate" | ...
    destroyed_in: str | None = None     # "on_cleanup" | "on_deactivate" | ...
    offset_ms: float = 0.0              # Timer offset (first-fire delay relative to T=0)
    # Source text of the callback argument when the analyzer could not model
    # it (neither std::bind of a member function nor an inline lambda), so
    # `callback` is "". Surfaced as RCLLF-H007.
    callback_expr: str | None = None


@dataclass
class SubscriberInfo:
    topic: str
    message_type: str
    callback: str                       # Reference to CallbackInfo.name
    qos_depth: int | None = None
    callback_group: str | None = None
    member_variable: str | None = None
    created_in: str | None = None
    destroyed_in: str | None = None
    qos_reliability: str | None = None  # "reliable" | "best_effort"
    qos_durability: str | None = None   # "volatile" | "transient_local"
    qos_history: str | None = None      # "keep_last" | "keep_all"
    qos_deadline_ms: float | None = None
    # True when the topic-name argument was not a lone resolvable string
    # literal (concatenation, member/parameter reference, computed
    # expression). `topic` is then a best-effort value or "" and the
    # connection may be missing from the extracted graph — surfaced as
    # RCLLF-H004 by `features/modules/dynamic_topic.py`.
    topic_dynamic: bool = False
    # The declared string parameter whose value the topic argument
    # holds (a member written once, from declare_parameter, in the
    # constructor). The topic is resolved from its launch value in
    # graph.hierarchical; None when the topic is not tied to a parameter.
    topic_param: str | None = None
    # See TimerInfo.callback_expr (RCLLF-H007).
    callback_expr: str | None = None
    # Set when this subscription is a message_filters::Subscriber input of a
    # recognised two-input synchronizer. `callback` is then a
    # synthetic per-input name, and this field names the user callback the
    # synchronizer calls from inside the input's reaction.
    sync_callback: str | None = None
    # Set (to the TransformListener member) when this is one of the
    # two subscriptions a tf2_ros::TransformListener creates on /tf and
    # /tf_static in its own callback group. `callback` is then the synthetic
    # ``tf_ingest_via_<member>`` and the binding claims the listener's handle
    # (group index kGroupTf2Listener) instead of exempting it.
    tf2_listener: str | None = None


@dataclass
class MessageFiltersSyncInfo:
    """A two-input message_filters synchronizer admitted by the analyzer."""

    member_variable: str                # e.g. "sync_"
    policy: str                         # "ExactTime" | "ApproximateTime" | "BoundedNearestToAnchor"
    queue_size: int | None              # policy constructor argument
    inputs: list[str] = field(default_factory=list)
                                        # Subscriber members, in input order
    callback: str = ""                  # registered user callback (CallbackInfo.name)
    # The BoundedNearestToAnchor parameters in nanoseconds (delta,
    # lateness0, lateness1, capacity0, capacity1, retention, timeout), from
    # the literal policy constructor arguments. None for other policies.
    bna_params: dict[str, int] | None = None
    # The ROS parameter a non-literal queue size reads
    # (``Policy(static_cast<T>(q))`` with ``q = declare_parameter("name", ...)``),
    # resolved to queue_size at generation time. None when queue_size is a
    # literal or the form is not recognised.
    queue_param: str | None = None


@dataclass
class MessageFiltersInfo:
    synchronizers: list[MessageFiltersSyncInfo] = field(default_factory=list)
    # One plain-language reason per unsupported message_filters use. Non-empty
    # means RCLLF-H003 refuses generation.
    unsupported: list[str] = field(default_factory=list)


@dataclass
class Tf2ListenerInfo:
    """A ``tf2_ros::TransformListener`` member."""

    member: str                         # e.g. "tf_listener_"
    buffer_member: str | None           # the Buffer it feeds, "" when not a member
    file: str = ""                      # source file name (no directory)
    line: int = 0


@dataclass
class Tf2BufferInfo:
    """A ``tf2_ros::Buffer`` member and the clock it was built with."""

    member: str
    clock: str                          # "node" for get_clock(), else the argument text
    file: str = ""
    line: int = 0


@dataclass
class Tf2BroadcasterInfo:
    """A ``tf2_ros::TransformBroadcaster`` / ``StaticTransformBroadcaster`` member."""

    member: str
    static: bool                        # True: /tf_static, False: /tf
    file: str = ""
    line: int = 0


@dataclass
class Tf2LookupCallInfo:
    """A direct ``lookupTransform``/``canTransform``/``waitForTransform`` call."""

    receiver: str                       # e.g. "tf_buffer_"
    method: str
    timeout: bool                       # a non-zero timeout argument (or waitForTransform)
    file: str = ""
    line: int = 0


@dataclass
class Tf2AdapterInfo:
    """An ``rcllf_tf2::AcquisitionLookup`` member with its literal arguments."""

    member: str
    buffer_member: str | None
    wait_bound_ns: int | None           # None when not an integer literal duration
    on_timeout: str | None              # "reject" | "failure" | "fallback:<name>" | None
    file: str = ""
    line: int = 0


@dataclass
class Tf2StampInfo:
    """The stamp argument of one ``<adapter>->request(...)`` call:
    where the sample's acquisition time comes from.

    ``kind`` is ``node_clock`` (this node's ``now()``, the Buffer's own clock),
    ``message`` (``<msg>->header.stamp`` of a subscription callback's message;
    ``callback`` or ``topic`` names the input), ``foreign_clock`` (a clock that
    is not the node's: system, steady, a separate ``rclcpp::Clock``) or
    ``unknown`` (``detail`` says what the scan could not follow).
    """

    adapter: str                        # the AcquisitionLookup member
    kind: str
    expr: str                           # the stamp argument as written
    detail: str = ""
    callback: str | None = None         # message: the member function holding the call
    topic: str | None = None            # message: the topic literal of a lambda subscription
    file: str = ""
    line: int = 0


@dataclass
class Tf2Refusal:
    """One reason the node's tf2 use is out of the admitted domain (RCLLF-R002)."""

    subcode: str                        # e.g. "tf2_direct_lookup"
    detail: str


@dataclass
class Tf2Info:
    """tf2 use recognised by ``node_analysis/tf2_scan.py``."""

    buffers: list[Tf2BufferInfo] = field(default_factory=list)
    listeners: list[Tf2ListenerInfo] = field(default_factory=list)
    broadcasters: list[Tf2BroadcasterInfo] = field(default_factory=list)
    lookups: list[Tf2LookupCallInfo] = field(default_factory=list)
    adapters: list[Tf2AdapterInfo] = field(default_factory=list)
    # The stamp of every request call on an adapter member (criterion 5).
    # None: system.json from before the stamp scan; generation refuses then.
    stamps: list[Tf2StampInfo] | None = None
    # Non-empty means RCLLF-R002 refuses generation with these subcodes.
    refusals: list[Tf2Refusal] = field(default_factory=list)


@dataclass
class ServiceServerInfo:
    service_name: str
    service_type: str
    callback: str                       # Reference to CallbackInfo.name
    callback_group: str | None = None
    member_variable: str | None = None
    created_in: str | None = None
    destroyed_in: str | None = None


@dataclass
class ActionServerInfo:
    action_name: str
    action_type: str
    goal_callback: str | None = None    # Reference to CallbackInfo.name
    cancel_callback: str | None = None
    accepted_callback: str | None = None
    callback_group: str | None = None
    member_variable: str | None = None
    created_in: str | None = None
    destroyed_in: str | None = None


# ---------------------------------------------------------------------------
# Resources (used by callbacks, not triggered directly)
# ---------------------------------------------------------------------------

@dataclass
class PublisherInfo:
    topic: str
    message_type: str                   # Fully qualified: custom_interfaces::msg::SensorData
    qos_depth: int | None = None
    member_variable: str | None = None  # e.g. "publisher_a_"
    is_lifecycle: bool = False          # True for rclcpp_lifecycle::LifecyclePublisher
    created_in: str | None = None
    destroyed_in: str | None = None
    qos_reliability: str | None = None  # "reliable" | "best_effort"
    qos_durability: str | None = None   # "volatile" | "transient_local"
    qos_history: str | None = None      # "keep_last" | "keep_all"
    qos_deadline_ms: float | None = None
    # See SubscriberInfo.topic_dynamic (RCLLF-H004).
    topic_dynamic: bool = False
    # See SubscriberInfo.topic_param.
    topic_param: str | None = None
    # Some call site publishes a serialized message
    # (rcl_publish_serialized_message), which the binding refuses at run time.
    serialized_publish: bool = False
    # Fifth review: True when the create call runs on
    # every path through the method that creates it (no enclosing branch,
    # loop, lambda or try block, no return before it); False when it may not;
    # None when the analysis did not decide it.
    unconditional: bool | None = None


@dataclass
class ServiceClientInfo:
    service_name: str
    service_type: str
    member_variable: str | None = None
    created_in: str | None = None
    destroyed_in: str | None = None
    response_callback: str | None = None  # From async_send_request() 2nd arg
    # "async_callback" = async_send_request with callback; safe for LF three-reaction pattern.
    # "blocking_spin"  = followed by spin_until_future_complete; needs physical-action wrap.
    # "blocking_future"= followed by future.wait()/get(); needs physical-action wrap.
    # "blocking_wait"  = then waits for the response in the same method (spin_some/
    #                    spin_once loop, condition variable, sleep loop on a flag).
    # None             = undetermined (treat conservatively as blocking).
    call_style: str | None = None
    callback_group: str | None = None   # Reference to CallbackGroupInfo.name


@dataclass
class ActionClientInfo:
    action_name: str
    action_type: str
    member_variable: str | None = None
    created_in: str | None = None
    destroyed_in: str | None = None
    goal_response_callback: str | None = None  # From SendGoalOptions binding
    feedback_callback: str | None = None
    result_callback: str | None = None
    callback_group: str | None = None   # Reference to CallbackGroupInfo.name


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class ParameterInfo:
    name: str
    type: str                           # "string" | "double" | "int" | "bool" | etc.
    default_value: Any | None = None
    description: str | None = None


# ---------------------------------------------------------------------------
# Full node model
# ---------------------------------------------------------------------------

@dataclass
class NodeInfo:
    name: str
    class_name: str
    package: str
    source_file: Path
    header_file: Path | None = None
    base_class: str = "rclcpp::Node"    # or "rclcpp_lifecycle::LifecycleNode"
    is_component: bool = False          # True if RCLCPP_COMPONENTS_REGISTER_NODE
    # Namespace-qualified class name read from the clang AST of the class
    # definition (``ns::Cls``); empty when unknown (older system.json).
    qualified_name: str = ""

    # Callbacks (core behavioral units)
    callbacks: list[CallbackInfo] = field(default_factory=list)
    callback_groups: list[CallbackGroupInfo] = field(default_factory=list)

    # Triggers (invoke callbacks)
    timers: list[TimerInfo] = field(default_factory=list)
    subscribers: list[SubscriberInfo] = field(default_factory=list)
    service_servers: list[ServiceServerInfo] = field(default_factory=list)
    action_servers: list[ActionServerInfo] = field(default_factory=list)

    # Resources (used by callbacks)
    publishers: list[PublisherInfo] = field(default_factory=list)
    service_clients: list[ServiceClientInfo] = field(default_factory=list)
    action_clients: list[ActionClientInfo] = field(default_factory=list)

    # Configuration
    parameters: list[ParameterInfo] = field(default_factory=list)
    member_variables: list[str] = field(default_factory=list)
    lifecycle_callbacks: list[str] = field(default_factory=list)
                                        # e.g. ["on_configure", "on_activate", ...]
    # Lifecycle hooks this class overrides WITHOUT chaining to the base
    # implementation, restricted to the hooks where the base does real work
    # (LIFECYCLE_CHAINED_HOOKS). `LifecycleNode::on_activate` activates every
    # managed publisher via `impl_->on_activate()`, so an override that skips
    # it leaves them deactivated and `publish()` drops silently. Feeds
    # RCLLF-H005 (features/modules/lifecycle_activation.py).
    lifecycle_hooks_not_chained: list[str] = field(default_factory=list)
    has_param_callback: bool = False    # True if add_on_set_parameters_callback used
    param_callback: str | None = None  # Callback name bound to add_on_set_parameters_callback

    # Parse warnings: missing headers or other libclang issues that may cause
    # incomplete extraction.  Surfaces actionable hints to the user.
    parse_warnings: list[str] = field(default_factory=list)

    # #include directives libclang could not resolve, one dict per directive:
    # {"header": "mujoco/mujoco.h", "included_from": "/abs/path.hpp",
    # "line": 18}. A missing header can drop every entity of the node from
    # the AST, so `colcon lf-analyze` names each one and fails when the node
    # is left with no visible entity.
    unresolved_includes: list[dict[str, Any]] = field(default_factory=list)

    # Allowlisted include-directive spellings detected via a source-level
    # include scan (not AST instantiation matching — robust to missing
    # third-party headers). Allowlist = "message_filters",
    # "tf2_ros". Feeds RCLLF-H003 detection
    # (features/modules/message_filters_sync.py) and, for "tf2_ros",
    # the resolved model's tf2 mechanism record (RCLLF-R002).
    detected_includes: list[str] = field(default_factory=list)

    # Explicit rclcpp::SubscriptionBase pull-style calls
    # (take/take_serialized/take_loaned) detected via a narrow member-call
    # token scan (node_analysis/clang_parser.py::_scan_take_calls). Feeds
    # the resolved model's take mechanism record (RCLLF-R002).
    detected_take_calls: list[str] = field(default_factory=list)

    # ``get_clock()->sleep_for(`` / ``sleep_until(`` calls found by a token
    # scan (clang_parser.py::_scan_ros_sleep_calls), as "<call>:<line>".
    # Feeds RCLLF-H008 (features/modules/ros_time_sleep.py).
    detected_ros_sleep_calls: list[str] = field(default_factory=list)

    # message_filters synchronizers recognised by the source scan
    # (node_analysis/message_filters_scan.py) plus the reasons any
    # other message_filters use is unsupported. None when the node does not
    # mention message_filters. Feeds RCLLF-H003
    # (features/modules/message_filters_sync.py).
    message_filters: MessageFiltersInfo | None = None

    # tf2 use recognised by node_analysis/tf2_scan.py: Buffers,
    # TransformListeners, broadcasters, direct lookups and rcllf_tf2 adapters.
    # None when the node does not mention tf2_ros. Feeds the resolved model's
    # mechanism.tf2 record (RCLLF-R002 subcodes) and the generator's /tf ingress.
    tf2: Tf2Info | None = None

    # Timers created in a method that is neither the constructor nor a
    # recognized lifecycle hook (e.g. ActionClientNode::cancel_timer_,
    # created inside goal_response_callback). NOT part of
    # `timers` above and never generated into an LF reaction: the runtime's
    # single ros2_lf_bind_node() collection pass runs once, from the
    # startup reaction, before any such callback can ever fire (
    # a timer created after that pass is not in the handle table), so a
    # generated reaction for one of these could never be bound. Facts-only,
    # like `lifecycle_hooks_not_chained`; feeds RCLLF-H006 detection
    # (features/modules/timers.py).
    unbound_timers: list[TimerInfo] = field(default_factory=list)

    # Threads the node starts itself (in the constructor or any method) that
    # send action feedback or a result, one dict per thread target:
    # {"thread": "execute", "started_in": "handle_accepted", "action":
    # "analyze"}. "?" as action when the node has several servers. Such a
    # send has no LF origin. Facts-only, feeds RCLLF-H011
    # (features/modules/thread_action_server.py).
    thread_action_senders: list[dict[str, str]] = field(default_factory=list)

    # Endpoints (subscription, service, client, publisher, ...) created in a
    # method that is neither the constructor nor a lifecycle hook, one dict per
    # call: {"kind": "subscription", "topic": "chatter", "method": "on_x",
    # "callback": "lambda at f.cpp:12"}. Not part of the node's endpoint
    # lists and never generated. Facts-only, feeds RCLLF-H013
    # (features/modules/endpoint_created_in_callback.py).
    unbound_endpoints: list[dict[str, str]] = field(default_factory=list)

    # Template base classes of the node that create endpoints in their
    # constructor: {"base": "EchoBase", "calls": "create_subscription"}. The
    # analyzer cannot walk a template's constructor, so such a node is refused
    # (RCLLF-H015, features/modules/template_base_endpoints.py).
    template_base_endpoints: list[dict[str, str]] = field(default_factory=list)

    # Work main() does on the node outside executor callbacks, one dict per
    # call: {"source": "mt_main.cpp", "call": "publish_from_main"}. rcllf
    # replaces main(), so it never runs. Facts-only, feeds RCLLF-H014
    # (features/modules/main_outside_callbacks.py).
    main_outside_callbacks: list[dict[str, str]] = field(default_factory=list)

    # The compile definitions that select this node's source variant could not
    # be told apart: {"source": "qos.cpp", "reason": "...", "fatal": "0|1"}.
    # Feeds RCLLF-H016 (features/modules/compile_variant_ambiguous.py).
    compile_variant_issues: list[dict[str, str]] = field(default_factory=list)

    # Workspace base classes whose constructors and methods were merged into
    # this node ("TickerBase"), most-base first. Facts-only.
    base_classes_merged: list[str] = field(default_factory=list)

    # ``-DNAME[=VALUE]`` definitions of the executable target this node was
    # analyzed under (CMake ``target_compile_definitions``). The generated
    # staged copy of the source is compiled with the same definitions.
    compile_defines: list[str] = field(default_factory=list)

    @property
    def is_lifecycle(self) -> bool:
        """Whether this node has a managed lifecycle.

        Derived rather than stored, so it needs no `system.json` schema change
        and cannot go stale against `base_class`. Both inputs are already
        serialized.
        """
        return is_lifecycle_base(self.base_class, self.lifecycle_callbacks)

    @property
    def has_visible_entities(self) -> bool:
        """Whether extraction found any timer, endpoint, service or action."""
        return any((
            self.timers, self.publishers, self.subscribers,
            self.service_servers, self.service_clients,
            self.action_servers, self.action_clients,
        ))

    @property
    def takes_component_path(self) -> bool:
        """Whether this node should be loaded by `LFComponentManager`.

        Not the same question as `is_component`, and that is the whole point.
        A managed lifecycle node registered as a component takes the
        non-component path instead, because the component path cannot drive its
        lifecycle: `LFComponentManager::load_component` records the node in
        `class_to_id_` only, never in the node registry, so
        `ros2_lf_transition_node` finds nothing to drive and the node is bound
        while still `unconfigured`, owning none of the entities its
        `on_configure` / `on_activate` would create.

        The non-component path constructs the node itself and registers it via
        `ros2_lf_register_node`, which is where the configure/activate thunk is
        captured while the concrete type is still visible.
        """
        return self.is_component and not self.is_lifecycle


# ---------------------------------------------------------------------------
# CMake layer (output of cmake_parser)
# ---------------------------------------------------------------------------

@dataclass
class LibraryTarget:
    name: str
    type: str           # "SHARED" | "STATIC" | "MODULE" | ""
    sources: list[str]  # Relative source paths
    component_plugins: list[str] = field(default_factory=list)  # From rclcpp_components_register_nodes


@dataclass
class CMakeInfo:
    """Parsed information from a single CMakeLists.txt file."""

    package_name: str
    # Executable targets: name -> first source file (relative path)
    executables: dict[str, str] = field(default_factory=dict)
    # Library targets: name -> LibraryTarget
    libraries: dict[str, LibraryTarget] = field(default_factory=dict)
    # ament_target_dependencies: target name -> list of deps
    dependencies: dict[str, list[str]] = field(default_factory=dict)
    # target_compile_definitions(target PRIVATE|PUBLIC X Y=1): target -> ["-DX", "-DY=1"]
    target_definitions: dict[str, list[str]] = field(default_factory=dict)
    # rosidl_generate_interfaces: list of interface file paths
    interfaces: list[str] = field(default_factory=list)
