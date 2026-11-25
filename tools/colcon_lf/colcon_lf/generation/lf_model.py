# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Intermediate data model for Lingua Franca code generation.

These dataclasses represent the LF-level structure before text emission.
Built from a SystemGraph by lf_generator, consumed by lf_emitter.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from colcon_lf.federation.partition import FederationMap


@dataclass
class LFPort:
    """An input or output port on a reactor."""

    name: str
    type: str  # LF type; use "stub_t" for lfc-path ports
    direction: str  # "input" or "output"


@dataclass
class LFTimer:
    """A periodic timer declaration."""

    name: str
    offset: str  # e.g. "0"
    period: str  # e.g. "500 msec"


@dataclass
class LFPhysicalAction:
    """A physical action for wrapping blocking service calls."""

    name: str
    min_delay_ns: int = 0


@dataclass
class LFReaction:
    """A reaction triggered by events, producing effects."""

    triggers: list[str]  # Port/timer/action names
    effects: list[str]   # Output port names
    body: str            # Code inside {= =}
    reaction_id: int = -1  # Assigned by _assign_reaction_ids; -1 = unassigned
    kind: str = "unknown"
    # kind values: "timer" | "sub" | "srv_send" | "srv_send_blocking" |
    #   "srv_server" | "srv_client_cb" | "srv_client_continuation" | "startup"
    rid_name: str | None = None
    # Explicit RID callback-name override for reactions whose body has no
    # ros2_lf_invoke_*(self, RID_...) call to extract one from (e.g. the
    # feedback/result/cancel poll-relay reactions in _build_action_reactions,
    # which just do a bare lf_set() — see binding_header_emitter._cb_name_from_body).
    # Without this, every such reaction falls back to the literal name
    # "unknown" and two of them on the same node collide on the same RID macro.
    # Non-triggering sources (LF "uses"), emitted between the trigger list and
    # "->". Every user-code reaction of a group sub-reactor reads
    # BIND_PORT so LF's precedence graph orders the node's bind first at tag 0.
    sources: list[str] = field(default_factory=list)
    # Publication correspondence (D4): admitted edge ids this
    # reaction emits records for (publisher side), and the edge whose
    # drain this reaction is (subscriber side). Read back after RID
    # assignment to fill CorrEdge.pub_rids / sub_rid.
    corr_emit_edges: list[int] = field(default_factory=list)
    corr_drain_edge: int | None = None
    # Every member edge of the fan-in group this drain serves,
    # ascending (``corr_drain_edge`` is the lead, the first). Each gets the
    # drain's RID as its ``sub_rid``.
    corr_drain_members: list[int] = field(default_factory=list)
    # An action server with no client inside the LF system
    # is triggered by this physical action (``physical action <name>: int64_t``)
    # instead of its request ports. The reactor declares it once and registers
    # it at startup with the goal server RID in ``external_register_rid``.
    external_action: str | None = None
    external_register_rid: str | None = None


@dataclass
class CorrEdge:
    """One topic edge (publisher node, subscriber node) of the correspondence table.

    ``status`` is ``"admitted"`` (generation-stage admission, D1) or
    ``"legacy"`` (today's stub-port path) with a short snake_case ``reason``.
    Only admitted edges carry a dense ``edge_id`` and RIDs.
    """

    topic: str          # Fully resolved topic name
    pub_node: str       # Launch-level node key of the publisher
    sub_node: str       # Launch-level node key of the subscriber
    pub_key: str        # Binding key of the publisher
    sub_key: str        # Binding key of the subscriber
    delay_ns: int       # Resolved after-delay d of the link
    status: str = "legacy"
    reason: str | None = None
    # One sentence on why a refused edge stays legacy, when the
    # reason alone does not say it (``fan_in_member_refused``).
    note: str | None = None
    edge_id: int | None = None
    pub_topic: str = ""     # Node-local topic at the publisher
    # The publisher is a lifecycle node's, so the binding binds
    # every publisher the node creates on the topic, also after startup.
    pub_rebind: bool = False
    # The subscription is a lifecycle node's, made or reset in a
    # hook. The binding admits every reader the node creates for it.
    sub_rebind: bool = False
    sub_topic: str = ""     # Node-local topic at the subscriber
    # The callback of the edge's one matching subscription, filled with
    # ``sub_topic`` before any refusal. ``sub_callback`` is set only once the
    # edge passes the subscriber checks, so an edge refused before them
    # (``transient_local``, ``publisher_callback``, ...) still names its
    # refusal to the BNA input check instead of ``no_edge``. Not
    # serialized.
    listed_sub_callback: str = ""
    # Every publishing callback, node_info.callbacks order.
    pub_callbacks: list[str] = field(default_factory=list)
    sub_callback: str = ""
    pub_rids: list[int] = field(default_factory=list)  # ascending
    sub_rid: int = -1
    # Schedule-derived bounds of an admitted edge (W3 design 3.3
    # to 3.6): the subscriber's take_timeout_ms T_e, the declared worker lag
    # bound L, the burst bound k, the retention bound N_e(d_e + L) + 1 and
    # the carrier bound N_e(d_e). ``bound_inputs`` records what they were
    # computed from (k, P, d, L, W per window) for the trace manifest.
    timeout_ns: int = 0
    max_lag_ns: int = 0
    burst: int = 0
    retain: int = 0
    carrier: int = 0
    # KEEP_LAST depth of the edge's reader (exec-config reader_depth).
    reader_depth: int = 0
    # The fan-in group of the edge, every admitted edge into one
    # subscription callback. ``group_lead`` is the lowest member edge id and
    # names the shared drain, ``fan_in_rank`` the position of the publisher's
    # LF instance name among the members' (the order tie-break), and
    # ``fan_in_group`` the group's index during admission. A group of one
    # leads itself with rank 0.
    group_lead: int | None = None
    fan_in_rank: int = 0
    fan_in_group: int | None = None
    # exec-config ``join: deferred``. The drain runs at the cutoff
    # tag and never waits for an absent sample. A self edge with the key is a
    # generation error; a cross edge carries it on the ingress row only.
    deferred: bool = False
    # An in-process d = 0 edge whose first record of a tag is
    # dispatched at the publication tag (``lf_generator.corr_same_tag``,
    # binding ``set_corr_edge_same_tag``). Set by ``_node_corr_from_edges``.
    same_tag: bool = False
    # An ingress row over a decentralized link. The binding drops a
    # record that arrives after its intended tag and reports its message lost
    # (``set_corr_edge_tardy_drop``), where a centralized link exits.
    tardy_drop: bool = False
    # exec-config ``control_failure_policy: stop``. On an
    # ingress row the binding stops LF on a missing COMPLETE record
    # (``set_corr_edge_control_stop``). Only the ingress row receives COMPLETE.
    control_stop: bool = False
    # An egress row that sends through the t + 1 ns flush over
    # `after d - 1 ns` (``lf_generator.xfed_flush``), not at the publishing tag.
    xfed_flush: bool = False
    bound_inputs: dict[str, Any] = field(default_factory=dict)
    # The ``inst.port`` ends a cross edge's after was looked up by
    # (``FederationMap.after_ns_for``), so the model cites the matching override.
    link_ports: tuple[str, str] | None = None
    # Where the edge lives in a federated build. ``local`` (both
    # ends in this process, or unfederated), ``egress`` (the publisher's
    # federate) or ``ingress`` (the subscriber's). ``federate`` is the
    # federate holding this row (None unfederated), ``global_id`` the
    # system-wide admitted id that names the outer ``xcorr_g<G>`` ports, and
    # ``peer_node`` the fully qualified name of the node at the other end.
    role: str = "local"
    federate: str | None = None
    peer_federate: str | None = None
    global_id: int | None = None
    peer_node: str = ""
    # The analyzed reliability of the subscription ("reliable",
    # "best_effort", or "" when the analysis has no value). The binding
    # compares it with the actual QoS at startup.
    sub_reliability: str = ""
    # The analyzed KEEP_LAST depth of the subscription (None when
    # the analysis has no value), compared at startup for RCLLF-H002.
    sub_depth: int | None = None

    @property
    def remote(self) -> bool:
        """True for an egress or ingress edge (the record crosses federates)."""
        return self.role != "local"

    def label(self) -> str:
        """Human-readable edge name for messages."""
        return f"{self.topic} ({self.pub_node} -> {self.sub_node})"

    def to_json(self) -> dict[str, Any]:
        """Row of ``correspondence.json``."""
        return {
            "id": self.edge_id,
            "topic": self.topic,
            "pub": self.pub_key,
            "sub": self.sub_key,
            "delay_ns": self.delay_ns,
            "status": self.status,
            "reason": self.reason,
        } | ({} if self.note is None else {"note": self.note}) | (
            {"pub_rebind": True} if self.pub_rebind and self.edge_id is not None else {}
        ) | (
            {"sub_rebind": True} if self.sub_rebind and self.edge_id is not None else {}
        ) | ({} if self.federate is None else {
            "role": self.role,
            "global_id": self.global_id,
            "peer_federate": self.peer_federate,
        })


@dataclass
class LFReactorInstance:
    """An instantiation of a reactor class."""

    name: str  # Instance name (snake_case)
    reactor_class: str  # Reactor type name (PascalCase)
    # ``(parameter, value)`` pairs, emitted as ``new Class(p = v)``.
    args: list[tuple[str, str]] = field(default_factory=list)


@dataclass
class LFConnection:
    """A connection between ports, optionally with an after delay."""

    source: str       # "instance.port" or just "port" for internal
    destination: str
    after_delay_ns: int | None = None  # Logical-time after delay in nanoseconds
    is_cross_federate: bool = False    # True when endpoints are in different federates
    federate_after_ns: int | None = None  # Override after-delay for cross-federate links
    # STP (safe-to-process) offset for this cross-federate link, only meaningful
    # under decentralized coordination. None under centralized coordination.
    federate_stp_ns: int | None = None
    # Resolved topic of a legacy stub topic link, None for every other link.
    # The stub fan-in merge groups these by destination.
    topic: str | None = None
    # The system link this connection realises, as the resolved
    # model names it: (kind, sending node key, receiving node key, name), kind
    # "topic", "service request", "service response" or "action <channel>".
    # None for forwarding and generated links. The program registers it, so
    # the read-back can tell which contract a delay belongs to.
    link: tuple[str, str, str, str] | None = None
    # A configured per-connection delay's value and reason, emitted
    # as a comment line above the connection. None for an unconfigured link.
    comment: str | None = None
    # Probe P17h: the subscriber port a stub fan-in link names
    # before route_fan_in moves it to <port>_fanin_k, set on every link of a
    # fan-in (k = 0 too). Overrides and provenance are looked up by it.
    user_destination: str | None = None


@dataclass
class LFReactor:
    """A reactor definition with ports, timers, reactions, and sub-instances."""

    name: str  # PascalCase reactor class name
    ports: list[LFPort] = field(default_factory=list)
    timers: list[LFTimer] = field(default_factory=list)
    physical_actions: list[LFPhysicalAction] = field(default_factory=list)
    reactions: list[LFReaction] = field(default_factory=list)
    instances: list[LFReactorInstance] = field(default_factory=list)
    connections: list[LFConnection] = field(default_factory=list)
    # Logical actions without payload (corr_drain_e<ID>).
    logical_actions: list[str] = field(default_factory=list)
    startup_body: str | None = None  # Emitted as reaction(startup) if set
    # Effects of the startup (bind) reaction, e.g. "default_group.rcllf_bound"
    # (bind writes each group's BIND_PORT before any dispatch).
    startup_effects: list[str] = field(default_factory=list)
    # The drain host's ingress reactions read BIND_PORT too.
    ingress_bound: bool = False
    # One-scheduler ingress: when set, this reactor is the node's
    # drain host and gets ingress_wake/ingress_drain plus their reactions.
    ingress_key: str | None = None
    # The node's ingress STORED capacity from execution.yaml's
    # ``ingress_capacity``; None keeps the binding default.
    ingress_capacity: int | None = None
    # ``(physical action, RID macro)`` per external subscription
    # here, emitted as ``physical action <name>: int64_t`` plus a startup reaction
    # that registers the action with the binding (outside the RID numbering).
    external_inputs: list[tuple[str, str]] = field(default_factory=list)
    # ``(name, type, default)`` parameters, ``reactor Name(p: t = d)``.
    parameters: list[tuple[str, str, str]] = field(default_factory=list)


@dataclass
class LFSystem:
    """A complete LF program: target, reactor definitions, main reactor."""

    name: str  # Main reactor name (PascalCase)
    target: str = "CCpp"
    target_options: dict[str, Any] = field(default_factory=dict)
    preamble: str = ""  # Emitted as preamble {= ... =} before reactors
    reactors: list[LFReactor] = field(default_factory=list)
    main_instances: list[LFReactorInstance] = field(default_factory=list)
    main_connections: list[LFConnection] = field(default_factory=list)
    # Parameters of the main (or federate) reactor, one per timer
    # whose offset is set at run time, ``(name, type, default)``.
    parameters: list[tuple[str, str, str]] = field(default_factory=list)
    # ``(node_key, timer callback, parameter)`` per such timer.
    runtime_offsets: list[tuple[str, str, str]] = field(default_factory=list)
    # Topic edge table with generation-stage admission (D1).
    corr_edges: list[CorrEdge] = field(default_factory=list)
    # Refused cross-federate edges read in this federate. They are
    # listed by the publisher's federate, the binding here only checks their reader.
    corr_legacy_ingress: list[CorrEdge] = field(default_factory=list)
    # Federation: when set, this system is one federate inside a federated reactor
    federation: "FederationMap | None" = None
    # Name of this federate (None = single-process, non-federated)
    federate_name: str | None = None
    # Ports exposed at the federate boundary for cross-federate connections.
    # Each entry: (port_name, direction, lf_type, inner_instance, inner_port,
    # stp_offset_ns). stp_offset_ns is only meaningful for direction == "input"
    # under decentralized coordination; None otherwise (including all of
    # centralized coordination, and always for direction == "output").
    # Used by lf_emitter to add ports + forwarding connections to the outer
    # reactor, and (input + decentralized) a logging-only STP reaction.
    cross_federate_ports: list[tuple[str, str, str, str, str, int | None]] = field(
        default_factory=list
    )
    # Legacy (stub) output ports with a positive after delay. The
    # federate reactor counts their writes per logical time.
    legacy_guard_ports: list[str] = field(default_factory=list)
    # Microsteps of one time a guarded port may be written at: 2 under
    # centralized, 1 under decentralized (a second message is tardy).
    legacy_guard_max: int = 2
    # Boundary ports of type rcllf_xtag_t, port -> (topic, edges
    # an input port receives). The emitter packs the writer's tag log events
    # into an output and imports them from an input instead of forwarding.
    xtag_ports: dict[str, tuple[str, list[str]]] = field(default_factory=dict)
    # Topic -> how many gated publishers
    # the one writer federate of the topic has (analysis count, at least one
    # per publisher node). The receiving federate's startup tells the binding,
    # which holds a sample of a writer not announced until that many are.
    xtag_expect: dict[str, int] = field(default_factory=dict)
    # (edge, topic, reason) of the cross-federate legacy links into
    # this federate that carry only a count (RCLLF_COUNT_PUBLISHES=0, or a writer
    # publisher count the analysis cannot prove exact), warned at startup.
    count_rule_edges: list[tuple[str, str, str]] = field(default_factory=list)
    # This federate's carriers of links across federates, rows
    # in the form ``readback_links.link_rows`` registers ("network" where a
    # link arrives, "network-out" where it leaves).
    network_links: list[dict] = field(default_factory=list)
