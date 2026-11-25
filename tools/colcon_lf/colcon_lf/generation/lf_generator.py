# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Convert a SystemGraph into an LF intermediate model.

Entry point: ``generate_lf_system(graph) -> LFSystem``
"""

from __future__ import annotations

import dataclasses
import logging
import os
import re
import warnings
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field

from colcon_lf.comm_analysis.endpoints import endpoint_key_map
from colcon_lf.comm_analysis.models import ServiceConnection, SystemCommunication, TopicConnection
from colcon_lf.comm_analysis.topic_mapper import resolve_name
from colcon_lf.generation.binding_key import build_binding_keys
from colcon_lf.generation.lf_model import (
    CorrEdge,
    LFConnection,
    LFPhysicalAction,
    LFPort,
    LFReaction,
    LFReactor,
    LFReactorInstance,
    LFSystem,
    LFTimer,
)
from colcon_lf.generation.lf_naming import (
    action_to_cancel_req_port_name,
    action_to_cancel_resp_port_name,
    action_to_feedback_port_name,
    action_to_goal_req_port_name,
    action_to_goal_resp_port_name,
    action_to_result_port_name,
    action_to_result_req_port_name,
    callback_to_timer_name,
    class_name_to_reactor_name,
    deduplicate_names,
    launch_stem_to_reactor_name,
    message_type_to_lf_type,
    sanitize_lf_identifier,
    service_to_request_port_name,
    service_to_response_port_name,
    topic_to_port_name,
)
from colcon_lf.exec_config.connections import LinkDelays
from colcon_lf.exec_config.policies import STAMP_PROVENANCE_LF_CLOCK, period_ms_to_ns
from colcon_lf.exec_config.resolve import CorrBoundsConfig
from colcon_lf.generation.lf_time import parse_lf_time_ns
from colcon_lf.generation.node_identity import NodeIdentity, build_node_identities
from colcon_lf.graph.models import SystemGraph
from colcon_lf.node_analysis.models import (
    ActionClientInfo,
    ActionServerInfo,
    CallbackGroupInfo,
    CallbackInfo,
    NodeInfo,
    PublisherInfo,
    ServiceClientInfo,
    ServiceServerInfo,
    SubscriberInfo,
    TimerInfo,
    is_lifecycle_base,
)

log = logging.getLogger(__name__)

# Default declared logical-time offset (LF `after`, LET-style) used when no
# per-link value is given. Not a latency measurement; a measured DDS WCET may
# inform the choice of a link's after_delay_ns, but the value itself is the
# logical-time contract, not the measurement. See docs/concepts/time-and-coordination.md.
DEFAULT_AFTER_DELAY_NS = 0
# Stub port type: the payload goes over DDS. The value counts the writes of
# the port at one tag: LF keeps one value per port per tag, so
# every writer adds one and every reader runs once per counted write.
STUB_TYPE = "stub_t"
# Input of every group sub-reactor, written by the node reactor's
# bind startup reaction and read (non-triggering) by every reaction that runs
# user code, so bind precedes the first dispatch at tag 0 by LF precedence,
# not by timing. Prefixed so no topic port (topic_to_port_name) can collide.
BIND_PORT = "rcllf_bound"
# Publication correspondence (D4): LF type of an admitted edge's
# port (typedef in ros2_lf_binding.h) and the reaction kinds that get no RID.
CORR_PORT_TYPE = "rcllf_corr_t"
# The node reactor's merge of one output port written by several
# group sub-reactors runs no user code either.
PORT_MERGE_KIND = "port_merge"
# The reaction that sends an egress edge's records of one time.
XCORR_FLUSH_KIND = "corr_flush"
# An action server node's reaction(shutdown). It runs under
# the RID of the node's first goal server, so it gets none of its own.
ACTION_SHUTDOWN_KIND = "action_shutdown"
# The BNA synchronizer's startup input declaration, wake and
# shutdown close run no user code either.
BNA_DECLARE_KIND = "bna_declare"
BNA_WAKE_KIND = "bna_wake"
BNA_CLOSE_KIND = "bna_close"
BNA_WAKE_ACTION = "bna_wake"
# The rcllf_tf2::AcquisitionLookup deadline wake and shutdown
# reactions of a node with an adapter.
TF2_WAKE_KIND = "tf2_wake"
TF2_CLOSE_KIND = "tf2_close"
TF2_WAKE_ACTION = "tf2_wake"
# The startup wake that arms a subscription's logical deadline
# window (the drain at (start, 1)); it needs no RID and no bound node (its one
# binding call, ros2_lf_corr_claim_wake, reads neither).
CORR_SILENCE_ARM_KIND = "corr_silence_arm"
CORR_NO_RID_KINDS = frozenset({
    "corr_arrive", "corr_close", PORT_MERGE_KIND, XCORR_FLUSH_KIND, ACTION_SHUTDOWN_KIND,
    BNA_DECLARE_KIND, BNA_WAKE_KIND, BNA_CLOSE_KIND, CORR_SILENCE_ARM_KIND,
    TF2_WAKE_KIND, TF2_CLOSE_KIND,
})
# Shutdown and startup reactions that need no bound node (no BIND_PORT source).
_UNBOUND_KINDS = frozenset({
    "corr_close", BNA_DECLARE_KIND, BNA_CLOSE_KIND, CORR_SILENCE_ARM_KIND, TF2_CLOSE_KIND,
})


def corr_port_name(edge_id: int) -> str:
    """Port of admitted edge *edge_id* (publisher output, subscriber input)."""
    return f"corr_e{edge_id}"


# A cross-federate edge carries the records themselves (D1), in a
# fixed-size struct of at most RCLLF_XCORR_MAX records (ros2_lf_binding.h).
XCORR_PORT_TYPE = "rcllf_xcorr_t"
XFED_RECORD_CAP = 8

# A legacy stub topic link across federates carries its
# write count together with the writer's tag log events since its last
# trigger, so the receiver's take drops a late sample by tag (ros2_lf_binding.h).
XTAG_PORT_TYPE = "rcllf_xtag_t"


def xcorr_port_name(edge_id: int) -> str:
    """Inner port of cross-federate edge *edge_id* (the federate's local id)."""
    return f"xcorr_e{edge_id}"


def xcorr_outer_port_name(global_id: int) -> str:
    """Federate port and top-level link of cross-federate edge *global_id*."""
    return f"xcorr_g{global_id}"


def xcorr_flush_action_name(edge_id: int) -> str:
    """Logical action that sends egress edge *edge_id*'s records."""
    return f"xcorr_flush_e{edge_id}"


def corr_drain_action_name(edge_id: int) -> str:
    """Subscriber-side logical action that paces the edge's drain."""
    return f"corr_drain_e{edge_id}"


@dataclass
class _NodeCorr:
    """Per-node view of the admitted edges."""

    # publishing callback -> admitted edge ids it emits, in edge order
    pub_edges: dict[str, list[int]] = field(default_factory=dict)
    # subscription callback -> its admitted edge id, the lead (lowest id) of
    # its fan-in group, which names the drain
    sub_edges: dict[str, int] = field(default_factory=dict)
    # subscription callback -> every member edge id of its group, ascending
    sub_members: dict[str, list[int]] = field(default_factory=dict)
    # node-local topics whose stub port is gone (no legacy edge left)
    dropped_pub_topics: set[str] = field(default_factory=set)
    dropped_sub_topics: set[str] = field(default_factory=set)
    # Subscription callback on a declared external topic -> the
    # physical action (fed by the binding's ingress thread) that triggers it.
    external_subs: dict[str, str] = field(default_factory=dict)
    # Short names of this node's action servers whose action
    # has no client inside the LF system. A physical action triggers them.
    external_actions: set[str] = field(default_factory=set)
    # Edge ids whose other end is in another federate, and the
    # publishing callback -> egress edges whose close reaction it carries.
    remote: set[int] = field(default_factory=set)
    egress_close: dict[str, list[int]] = field(default_factory=dict)
    # Egress edges sent at the publishing tag, every
    # egress edge without the flush (``xfed_flush``). The publishing reaction
    # sends their records itself over `after d`, d = 0 included.
    zero: set[int] = field(default_factory=set)
    # Self edges (publisher node == subscriber node). They have no
    # port: the publishing reaction arrives its records itself and schedules
    # the drain action, so its effect is that action.
    local: set[int] = field(default_factory=set)
    # In-process d = 0 edges (``corr_same_tag``). Their drain takes
    # the port as a trigger and runs the first record at the publication tag.
    same_tag: set[int] = field(default_factory=set)
    # Subscription callback of a legacy (unadmitted) edge -> the
    # edge's reason, and the BNA sync input callbacks -> input index (0 anchor).
    legacy_reasons: dict[str, str] = field(default_factory=dict)
    bna_inputs: dict[str, int] = field(default_factory=dict)
    # BNA input callbacks (lf_clock stamps) -> declared max_stamp_lag ns.
    bna_stamp_lag: dict[str, int] = field(default_factory=dict)
    # Subscription callback -> the link delays d in ns of every
    # admitted edge into it (a fan-in group may mix them; the mf-sync/1 domain
    # needs one common d over all edges into the inputs).
    sub_delays: dict[str, set[int]] = field(default_factory=dict)
    # A legacy edge carries no subscription callback, so its reason
    # is also kept per subscribed topic for the sync input refusal.
    legacy_topic_reasons: dict[str, str] = field(default_factory=dict)

    def port(self, edge_id: int) -> str:
        return xcorr_port_name(edge_id) if edge_id in self.remote else corr_port_name(edge_id)

    def emit_effect(self, edge_id: int) -> str:
        """What a publishing reaction writes: the port, the flush action of an
        egress edge, the port of a d = 0 egress edge, or
        the drain action of a self edge."""
        if edge_id in self.zero:
            return xcorr_port_name(edge_id)
        if edge_id in self.remote:
            return xcorr_flush_action_name(edge_id)
        if edge_id in self.local:
            return corr_drain_action_name(edge_id)
        return corr_port_name(edge_id)

    def sub_trigger(self, edge_id: int) -> str:
        """What triggers a subscription: the drain action of a self edge,
        else the edge's port (an ingress edge's ``xcorr_e``,
        never its egress flush action)."""
        if edge_id in self.local:
            return corr_drain_action_name(edge_id)
        return self.port(edge_id)

    def is_local_sub(self, cb_name: str) -> bool:
        return self.sub_edges.get(cb_name) in self.local

    def ports(self) -> list[LFPort]:
        # An edge written by several callbacks is one port.
        out_ids = dict.fromkeys(
            e for ids in self.pub_edges.values() for e in ids if e not in self.local)
        out = [LFPort(name=self.port(e), type=self._type(e), direction="output")
               for e in out_ids]
        inp = [LFPort(name=self.port(e), type=self._type(e), direction="input")
               for ids in self.sub_members.values() for e in ids if e not in self.local]
        return out + inp

    def sub_ports(self, cb_name: str) -> list[str]:
        """The input ports of a subscription's fan-in group members."""
        return [self.port(e) for e in self.sub_members.get(cb_name, []) if e not in self.local]

    def _type(self, edge_id: int) -> str:
        return XCORR_PORT_TYPE if edge_id in self.remote else CORR_PORT_TYPE


_NO_CORR = _NodeCorr()

# Every action channel is a counted port. A reaction raises it by
# the number of samples the binding saw it send on that channel during this
# invocation (ros2_lf_action_sent, fed by the executable-side action gate), so
# a port event exists only for a real send and carries how many samples the
# receiving reaction takes. There are no poll timers.
ACTION_PORT_TYPE = "int"
# rcllf_action_channel values (ros2_lf_binding/action_gate_api.h).
_ACTION_CHANNEL = {
    "goal_req": "RCLLF_ACTION_GOAL_REQ",
    "result_req": "RCLLF_ACTION_RESULT_REQ",
    "cancel_req": "RCLLF_ACTION_CANCEL_REQ",
    "goal_resp": "RCLLF_ACTION_GOAL_RESP",
    "result": "RCLLF_ACTION_RESULT_RESP",
    "cancel_resp": "RCLLF_ACTION_CANCEL_RESP",
    "feedback": "RCLLF_ACTION_FEEDBACK",
}
# Logical delay of the feedback, result and request relays (goal_req and
# goal_resp use the link delay, 0 by default). A delay on
# every action connection also keeps the request and response reactions free
# of zero-delay cycles. Why 1 msec: it is the value the action evidence
# was recorded with (identical client tags in 10 of 10 federated runs). Any other d > 0 keeps the
# order and moves the client's feedback and result tags by d; d = 0 would put
# them at (t, m + 1) and has no run, so the cross-federate rule leaves this value alone.
ACTION_ASYNC_AFTER_DELAY_NS = 1_000_000  # 1 msec



def apply_timer_period_overrides(
    node_name: str,
    node_info: NodeInfo,
    timer_period_overrides: dict[str, dict[str, float]] | None,
) -> NodeInfo:
    """Return *node_info* with the ``execution.yaml`` timer periods applied.

    ``timer_period_overrides`` is ``{node_key: {callback: period_ms}}`` from
    ``nodes.<node>.timers.<timer>.period``. The override wins over
    the analyzed ``period_ms``, null or not. The generator's timers and the
    binding's RID table both take their periods from here, so the timer the LF
    program fires and the period the binding matches handles by agree.
    """
    node_overrides = (timer_period_overrides or {}).get(node_name)
    if not node_overrides or not node_info.timers:
        return node_info
    return dataclasses.replace(
        node_info,
        timers=[
            dataclasses.replace(t, period_ms=node_overrides[t.callback])
            if t.callback in node_overrides else t
            for t in node_info.timers
        ],
    )


def apply_timer_offset_overrides(
    graph: SystemGraph,
    timer_offset_overrides: dict[str, dict[str, float]] | None,
) -> SystemGraph:
    """Return *graph* with the ``execution.yaml`` timer start offsets applied.

    ``timer_offset_overrides`` is ``{node_key: {callback: offset_ms}}`` from
    ``nodes.<node>.timers.<timer>.offset``. The override wins over
    the analyzed ``offset_ms``. The LF timer reads it in ``_make_timer``. The
    binding matches timer handles by period only, so the RID table needs no
    offset.
    """
    if not timer_offset_overrides:
        return graph
    nodes = dict(graph.nodes)
    for node_key, offsets in timer_offset_overrides.items():
        node_info = nodes[node_key]
        nodes[node_key] = dataclasses.replace(
            node_info,
            timers=[
                dataclasses.replace(t, offset_ms=offsets[t.callback])
                if t.callback in offsets else t
                for t in node_info.timers
            ],
        )
    return dataclasses.replace(graph, nodes=nodes)


def apply_sync_queue_params(graph: SystemGraph, resolved_model) -> SystemGraph:
    """Return *graph* with each synchronizer's parameter-read queue size resolved.

    A policy built as ``Policy(q)`` with
    ``q = declare_parameter("name", ...)`` records ``queue_param`` (the
    analysis cannot read the value). The resolved model holds the value the
    node gets at run time (launch inline, params file or the declared default,
    ``param:<node>/<name>#value``), so it becomes ``queue_size``, the Q of the
    ApproximateTime burst bound in ``_CorrRates.k``. An unresolved value
    leaves ``queue_size`` None and the edge without a proven bound.
    """
    values = {}
    for r in getattr(resolved_model, "records", None) or ():
        subj = r.subject or {}
        if subj.get("kind") == "param" and r.property == "value" and isinstance(r.value, int) \
                and not isinstance(r.value, bool):
            values[(subj.get("node"), subj.get("name"))] = r.value
    nodes = dict(graph.nodes)
    changed = False
    for key, info in graph.nodes.items():
        mf = info.message_filters
        if mf is None:
            continue
        syncs = []
        for s in mf.synchronizers:
            if s.queue_size is None and s.queue_param is not None:
                q = values.get((key, s.queue_param), values.get((info.name, s.queue_param)))
                if q is not None and q > 0:
                    s = dataclasses.replace(s, queue_size=q)
                    changed = True
            syncs.append(s)
        nodes[key] = dataclasses.replace(
            info, message_filters=dataclasses.replace(mf, synchronizers=syncs))
    return dataclasses.replace(graph, nodes=nodes) if changed else graph


def run_mode_target_options(*, fast: bool = False, timeout: str | None = None) -> dict:
    """Target options of the run mode. ``timeout`` ends the
    generated program at that logical time; it is validated and canonicalised
    (``20000 msec`` -> ``20 sec``). ``fast`` emits NO target property: lfc
    0.12 refuses ``fast: true`` in a program with physical actions, and every
    rcllf node reactor has one (the ``ingress_wake`` action). Fast mode is a
    reactor-c runtime argument instead, see :func:`run_mode_launch_args`;
    ``fast`` is accepted here so one call validates the whole option set."""
    from colcon_lf.generation.lf_time import format_lf_time_ns, parse_lf_time_ns

    del fast  # validated as a bool by argparse; no target property, see above
    options: dict = {}
    timeout_ns = parse_lf_time_ns(timeout, source="--timeout")
    if timeout_ns is not None:
        options["timeout"] = format_lf_time_ns(timeout_ns)
    return options


def run_mode_launch_args(*, fast: bool = False) -> list[str]:
    """reactor-c arguments the generated launch files bake in:
    ``-f true`` runs logical time as fast as the reactions allow instead of
    waiting for physical time. reactor-c reads the last ``-f`` it sees, so
    ``RCLLF_LF_ARGS="-f false"`` at launch turns a fast-generated program
    back to real time, and ``RCLLF_LF_ARGS="-f true"`` runs any program
    fast without regeneration."""
    return ["-f", "true"] if fast else []


LAUNCH_WORKERS_ARG = "rcllf_workers"


def launch_workers(graph: SystemGraph) -> int | None:
    """The ``rcllf_workers`` launch argument of the analyzed launch file, or None.

    A launch file can pin the LF worker count of the program
    generated from it by declaring ``DeclareLaunchArgument('rcllf_workers',
    default_value='<n>')``. Only the root file (the one passed to
    ``lf-analyze``) counts, so a launch file that includes it keeps the
    reactor-c default. A stock ``ros2 launch`` of the same file ignores the
    argument. reactor-c's ``-w`` (``RCLLF_LF_ARGS="-w N"`` at launch, or on the
    program's command line) still wins: ``determine_number_of_workers`` in
    ``reactor_threaded.c`` reads the target property only when no ``-w`` was
    given.
    """
    launch_graph = getattr(graph, "launch_graph", None)
    root = getattr(launch_graph, "root", None)
    for arg in getattr(root, "arguments", None) or []:
        if arg.name != LAUNCH_WORKERS_ARG:
            continue
        value = arg.default_value
        if value is None or not str(value).strip().isdigit() or int(value) <= 0:
            raise ValueError(
                f"launch argument '{LAUNCH_WORKERS_ARG}' in {root.file_path} must default "
                f"to an integer > 0, got {value!r}")
        return int(value)
    return None


def resolve_workers(graph: SystemGraph, config_workers: int | None) -> int | None:
    """The LF worker count from the execution config and the launch file.

    Either may set it; both set to different counts is a generation error
    rather than a silent choice.
    """
    from_launch = launch_workers(graph)
    if config_workers is not None and from_launch is not None and config_workers != from_launch:
        raise ValueError(
            f"the execution config sets workers: {config_workers}, the launch argument "
            f"'{LAUNCH_WORKERS_ARG}' {from_launch}; set one, or both to the same count")
    return config_workers if config_workers is not None else from_launch


def generate_lf_system(
    graph: SystemGraph,
    *,
    after_delay: str = "",  # Kept for backwards compat; after_delay_ns preferred
    after_delay_ns: int | None = None,
    target_lfc: bool = True,
    binding_keys: dict[str, str] | None = None,
    timer_period_overrides: dict[str, dict[str, float]] | None = None,
    federated: bool = False,
    external_topics: set[str] | None = None,
    corr_bounds: CorrBoundsConfig | None = None,
    corr_plan: list[CorrEdge] | None = None,
    ingress_capacity: dict[str, int] | None = None,
    stub_fan_in: dict[str, list[str]] | None = None,
    link_delays: LinkDelays | None = None,
    reaction_order: dict[str, str] | None = None,
    runtime_offsets: dict[str, set[str]] | None = None,
    output_deadlines: dict[str, dict[str, int]] | None = None,
) -> LFSystem:
    """Build an LFSystem from a SystemGraph.

    Parameters
    ----------
    graph:
        The extracted ROS2 system model.
    after_delay:
        String delay, e.g. "1 msec". Parsed into nanoseconds when
        *after_delay_ns* is None. Parsing happens here rather than only in the
        callers so no call path can silently drop the value.
    after_delay_ns:
        Default declared logical-time offset (LF ``after``, LET-style), in
        nanoseconds, for inter-node connections. This is a contract, not a
        latency measurement. When None, per-link TopicConnection.after_delay_ns
        is used if set, otherwise DEFAULT_AFTER_DELAY_NS.
    target_lfc:
        Deprecated and ignored: the output is always the
        lfc-compilable .lf (stub_t ports, startup/ingress reactions,
        ros2_lf_invoke_* bodies). The stock abstract rendering is gone.
    binding_keys:
        Pre-computed ``{node_key: binding_key}``.  Federation passes the
        **system-wide** map so every federate agrees with ``exec_config.json``
        (which is resolved once against the whole graph and shipped whole to
        each federate — 00_design.md section 2.4 S6).  Synthesizing per-slice
        would key a class plain in a federate that holds one instance while the
        resolved table used the per-instance key, and the lookup would miss.
        When None, keys are synthesized from *graph* alone.
    timer_period_overrides:
        ``{node_key: {callback: period_ms}}`` from ``execution.yaml``'s
        ``nodes.<node>.timers.<timer>.period`` (maintainer
        decision P3). Applied before building each node's reactor, so it
        wins over a null (or even a literal) ``period_ms`` — matching the
        resolved-model gate's RCLLF-R001 check, which reads the same
        override. Silently a no-op for a timer callback it does not name.
    federated:
        True when the system is (a slice of) a federated program. Every topic
        edge then stays on the legacy path with reason ``federated`` (the cross-federate link stays an LF connection).
    external_topics:
        Resolved topic names declared in ``execution.yaml``'s
        ``external_topics``. Each subscription on one of them is
        triggered by a physical action that the binding's ingress thread
        schedules at message arrival, instead of an unconnected input port.
    corr_bounds:
        Exec-config inputs of the correspondence bounds: declared
        burst bounds, take timeouts, input periods and the worker lag bound.
        None means nothing is declared, so a topic edge relies on the burst
        bound the analyzer proves (W3 slice S3) and stays legacy
        (``burst_unbounded``) when there is none.
    corr_plan:
        This federate's rows of the system-wide admission
        (``plan_federated_corr``), local ids already assigned. Replaces the
        per-slice admission, so ``federated`` no longer refuses its edges.
    ingress_capacity:
        ``{node_key: capacity}`` from ``execution.yaml``'s ``ingress_capacity``.
        Set on the node's drain host, which then sets the
        mailbox capacity before it registers. A node it does not name keeps
        the binding default.
    link_delays:
        Per-connection after-delays from ``execution.yaml``'s
        ``connections:``. A configured pair or name beats every other source.
    stub_fan_in:
        ``topic_fan_in`` of the whole system's links. Federation
        passes it so a federate routes and merges the publisher nodes of a
        subscriber whose other publishers live in other federates. None
        computes it from this system's links.
    reaction_order:
        ``{node_key: order}`` from ``execution.yaml``'s
        ``reaction_order``, ``subscriptions_first`` or ``timers_first``: which
        group of the node's reactions is declared first. A node it does not
        name takes the registry default, ``timers_first``.
    runtime_offsets:
        ``{node_key: {callback}}`` from ``execution.yaml``'s
        ``nodes.<node>.timers.<timer>.offset_runtime: true``. Each such timer
        reads its offset from a parameter of the main reactor, which the
        program takes on its command line (see ``_apply_runtime_offsets``).
        Nodes not in *graph* (another federate's) are skipped.
    output_deadlines:
        ``{node_key: {topic: output_deadline_ns}}`` from
        ``execution.yaml``'s ``publishers:``. Every reaction of the node whose
        callback publishes the topic arms the binding's actuation lateness
        check just before its ``ros2_lf_invoke_*`` call (see
        ``_arm_output_deadlines``). Nodes not in *graph* are skipped.
    """
    from colcon_lf.exec_config.policies import REACTION_ORDERS

    for node_name, order in (reaction_order or {}).items():
        if order not in REACTION_ORDERS:
            raise ValueError(
                f"Node {node_name!r}: unknown reaction order {order!r}, "
                f"expected one of {REACTION_ORDERS!r}"
            )
    # Resolve the string form into nanoseconds when no explicit ns value was
    # given. Doing it here means every caller — single-process, federated, and
    # any future one — gets the flag honored without repeating the parse.
    if after_delay_ns is None and after_delay:
        after_delay_ns = parse_lf_time_ns(after_delay, source="--after-delay")

    # Build one reactor definition per *binding key*: plain class name for a
    # single-instance class (byte-identical to the pre-per-instance output),
    # one definition per node instance for a class with N >= 2 instances.
    class_to_reactor: dict[str, LFReactor] = {}
    instance_list: list[LFReactorInstance] = []

    if binding_keys is None:
        binding_keys = build_binding_keys(graph.nodes)

    # Sanitize launch-level dict keys → valid LF instance names.
    key_to_instance: dict[str, str] = {}
    for node_name in graph.nodes:
        key_to_instance[node_name] = sanitize_lf_identifier(node_name)

    # Endpoint → launch-key mapping for connections: the identity on node
    # keys, or the node name of a system.json written before endpoints carried the node key.
    runtime_to_key = endpoint_key_map(graph)

    # Launch namespace and remaps per node: a remapped endpoint
    # is matched to its edge by the name the analyzer resolved it to.
    scopes = build_node_identities(graph)

    # Publication correspondence (D1): generation-stage admission
    # per topic edge, before the reactors, which carry the admitted ports.
    if corr_plan is not None:
        node_corr = _node_corr_from_edges(corr_plan)
        topic_edges = [e for e in corr_plan if not e.remote]
        # A refused cross-federate edge is listed once, by its publisher's federate.
        corr_edges = [e for e in corr_plan if e.role != "ingress" or e.edge_id is not None]
        corr_legacy_ingress = [e for e in corr_plan if e.role == "ingress" and e.edge_id is None]
    else:
        corr_legacy_ingress = []
        corr_edges, node_corr = _build_corr_edges(
            graph.communication,
            graph.nodes,
            after_delay_ns=after_delay_ns,
            link_delays=link_delays,
            runtime_to_key=runtime_to_key,
            binding_keys=binding_keys,
            federated=federated,
            bounds=corr_bounds,
            timer_period_overrides=timer_period_overrides,
            scopes=scopes,
            reaction_order=reaction_order,
        )
        topic_edges = corr_edges

    _add_external_subs(graph, external_topics, runtime_to_key, node_corr, scopes)
    _add_external_actions(graph, runtime_to_key, node_corr)
    _check_bna_stamp_provenance(graph, corr_bounds)
    _mark_bna_stamp_lag(graph, corr_bounds, node_corr)

    for node_name, node_info in graph.nodes.items():
        binding_key = binding_keys[node_name]
        reactor_class = class_name_to_reactor_name(binding_key)

        node_info = apply_timer_period_overrides(node_name, node_info, timer_period_overrides)

        if reactor_class not in class_to_reactor:
            reactor, sub_defs = _build_reactor(
                node_info, binding_key=binding_key,
                corr=node_corr.get(node_name, _NO_CORR),
                # The whole-system pass of a federated build (no plan) only
                # names the system. Each federate's pass checks its BNA inputs.
                check_bna=not (federated and corr_plan is None),
                timers_first=_timers_first(reaction_order, node_name),
            )
            _arm_output_deadlines(
                node_name, node_info, binding_key, [reactor, *sub_defs],
                (output_deadlines or {}).get(node_name, {}),
            )
            class_to_reactor[reactor_class] = reactor
            for sub in sub_defs:
                if sub.name not in class_to_reactor:
                    class_to_reactor[sub.name] = sub

        instance_list.append(
            LFReactorInstance(
                name=key_to_instance[node_name],
                reactor_class=reactor_class,
            )
        )

    # Build inter-node connections from topic communication
    main_connections = _build_connections(
        graph.communication,
        set(graph.nodes.keys()),
        after_delay_ns=after_delay_ns,
        link_delays=link_delays,
        runtime_to_key=runtime_to_key,
        key_to_instance=key_to_instance,
        nodes=graph.nodes,
        topic_edges=topic_edges,
        scopes=scopes,
    )
    # Several publisher nodes into one stub input.
    if stub_fan_in is None:
        stub_fan_in = topic_fan_in(main_connections)
    route_fan_in(main_connections, stub_fan_in)
    if not federated:
        # Probe P17h: unequal per-writer delays split one program's fan-in too.
        split = fan_in_tag_errors(main_connections, lambda _inst: "", federated=False)
        if split:
            raise ValueError(split[0])
    inst_to_key = {inst: key for key, inst in key_to_instance.items()}
    for dst, srcs in stub_fan_in.items():
        inst, port = dst.split(".", 1)
        if inst in inst_to_key:
            reactor = class_to_reactor[class_name_to_reactor_name(binding_keys[inst_to_key[inst]])]
            _merge_fan_in(reactor, port, len(srcs))

    # De-conflict main reactor name with reactor class names.
    # The check must be case-insensitive: lfc's C codegen lowercases reactor
    # names when emitting typedefs (e.g. `Foo` and `foo` both become `foo_t`),
    # so if the wrapper name and an inner class share a lowercased form lfc
    # produces a duplicate-typedef compile error.  This bites the per-node
    # federation mode where the federate name (e.g. `FrontLidarDriver`) is
    # also the inner reactor class name.
    main_name = launch_stem_to_reactor_name(graph.name)
    inner_lower = {n.lower() for n in class_to_reactor}
    if main_name.lower() in inner_lower:
        main_name = f"{main_name}System"

    # The drain host of each node carries its ingress capacity.
    for node_name, capacity in (ingress_capacity or {}).items():
        if node_name not in binding_keys:
            continue
        for reactor in class_to_reactor.values():
            if reactor.ingress_key == binding_keys[node_name]:
                reactor.ingress_capacity = capacity

    # Assign reaction IDs (DFS over nodes, then declaration order per reactor)
    _assign_reaction_ids(class_to_reactor, graph, binding_keys=binding_keys)
    _fill_corr_rids(corr_edges, class_to_reactor)

    target_options: dict = {}
    preamble: str = ""
    target_options = {
        "build-type": "RelWithDebInfo",
        "keepalive": True,
    }
    # The execution config's 'workers' pins the LF worker
    # count; unset, reactor-c picks min(host cores, reaction graph breadth).
    # So does the analyzed launch file's own rcllf_workers argument.
    workers = resolve_workers(graph, corr_bounds.workers if corr_bounds is not None else None)
    if workers is not None:
        target_options["workers"] = workers
    preamble = (
        '#include "ros2_lf_binding/ros2_lf_binding.h"\n'
        '#include "ros2_lf_reaction_ids.h"\n'
        "typedef int32_t stub_t;\n"
        "// Ingress wake thunk: called by the binding's ingress\n"
        "// thread; returns the lf_schedule handle, <= 0 means refused.\n"
        "// Guarded: federated codegen includes this preamble twice in one TU.\n"
        "#ifndef RCLLF_INGRESS_WAKE_DEFINED\n"
        "#define RCLLF_INGRESS_WAKE_DEFINED\n"
        "static inline int rcllf_ingress_wake(void * action) {\n"
        "    return (int) lf_schedule(action, 0);\n"
        "}\n"
        "// External input wake: schedules the subscription's\n"
        "// physical action at arrival with the 64-bit record id as payload.\n"
        "static inline int rcllf_external_wake(void * action, int64_t record) {\n"
        "    return (int) lf_schedule_copy(action, 0, &record, 1);\n"
        "}\n"
        "#endif"
    )

    system = LFSystem(
        name=main_name,
        target="CCpp",
        target_options=target_options,
        preamble=preamble,
        reactors=list(class_to_reactor.values()),
        main_instances=instance_list,
        main_connections=main_connections,
        corr_edges=corr_edges,
        corr_legacy_ingress=corr_legacy_ingress,
    )
    _apply_runtime_offsets(system, runtime_offsets, graph, key_to_instance)
    return system


def runtime_offset_parameter(instance: str, callback: str) -> str:
    """Name of the main-reactor parameter that sets the offset of the timer
    *callback* of node instance *instance*. It is also the
    command-line option: ``--<name> <value> <unit>``."""
    return f"{instance}__{callback_to_timer_name(callback)}_offset"


def _apply_runtime_offsets(
    system: LFSystem,
    runtime_offsets: dict[str, set[str]] | None,
    graph: SystemGraph,
    key_to_instance: dict[str, str],
) -> None:
    """Turn the offset of every run-time timer into a reactor parameter.

    The timer keeps its built offset as the parameter's default, so a run that
    sets nothing has the tags of the baked build. The value flows down
    main (or federate) reactor -> node instance -> group sub-reactor that
    declares the timer, as ``timer t(<param>, period)``. A main-reactor
    parameter is a command-line option of the lfc program: reactor-c sets it in
    ``main()`` before the start tag, so the offset is fixed before logical
    time starts. With no run-time timer nothing changes, and the
    emitted .lf stays byte-identical.
    """
    if not runtime_offsets:
        return
    classes = {r.name: r for r in system.reactors}

    def path_to(cls_name: str, timer: str) -> list[tuple[LFReactor, LFReactorInstance | None]]:
        """``[(class, instance in it to descend into), ..., (owner, None)]``."""
        reactor = classes[cls_name]
        if any(t.name == timer for t in reactor.timers):
            return [(reactor, None)]
        for inst in reactor.instances:
            if inst.reactor_class in classes:
                sub = path_to(inst.reactor_class, timer)
                if sub:
                    return [(reactor, inst)] + sub
        return []

    instances = {i.name: i for i in system.main_instances}
    for node_key in graph.nodes:
        callbacks = runtime_offsets.get(node_key)
        if not callbacks:
            continue
        inst = instances[key_to_instance[node_key]]
        for callback in sorted(callbacks):
            timer = callback_to_timer_name(callback)
            path = path_to(inst.reactor_class, timer)
            if not path:
                raise ValueError(
                    f"offset_runtime: node {node_key!r} has no LF timer {timer!r} "
                    f"(callback {callback!r})")
            owner = path[-1][0]
            lf_timer = next(t for t in owner.timers if t.name == timer)
            local = f"{timer}_offset"
            # A class shared by two run-time instances is converted once.
            done = next((p for p in owner.parameters if p[0] == local), None)
            default = done[2] if done else lf_timer.offset
            lf_timer.offset = local
            for reactor, child in path:
                if all(p[0] != local for p in reactor.parameters):
                    reactor.parameters.append((local, "time", default))
                if child is not None and all(a[0] != local for a in child.args):
                    child.args.append((local, local))
            name = runtime_offset_parameter(inst.name, callback)
            system.parameters.append((name, "time", default))
            inst.args.append((local, name))
            system.runtime_offsets.append((node_key, callback, name))


# ---------------------------------------------------------------------------
# Reaction-ID assignment
# ---------------------------------------------------------------------------

def _assign_reaction_ids(
    class_to_reactor: dict[str, LFReactor],
    graph: SystemGraph,
    *,
    binding_keys: dict[str, str] | None = None,
) -> None:
    """Assign sequential reaction IDs across all reactors in DFS order.

    Traversal order: nodes in graph.nodes insertion order, then within each
    reactor its reactions in declaration order (the startup reaction has no
    RID). ``_build_reactions`` sets that order: egress flush reactions first,
    then timer reactions before subscription reactions (the default), or the
    reverse with ``reaction_order: subscriptions_first``, source
    order within each,
    then the service and action reactions.
    IDs are sequential integers starting at 0, stable across regenerations
    for the same SystemGraph.

    Dedup is keyed on the **binding key** (``binding_key.py``) so a class with
    N >= 2 instances gets N disjoint RID ranges, one per instance.
    """
    rid = 0
    seen_reactors: set[str] = set()
    if binding_keys is None:
        binding_keys = build_binding_keys(graph.nodes)

    for node_name, node_info in graph.nodes.items():
        reactor_class = class_name_to_reactor_name(binding_keys[node_name])
        if reactor_class in seen_reactors:
            continue
        seen_reactors.add(reactor_class)

        reactor = class_to_reactor.get(reactor_class)
        if reactor is None:
            continue

        # Startup reaction gets ID 0 within the reactor (but global counter)
        if reactor.startup_body is not None:
            # Startup is not in reactor.reactions; it has no RID in the table
            # (it calls ros2_lf_bind_node, not ros2_lf_invoke_*)
            pass

        for reaction in reactor.reactions:
            if reaction.kind in CORR_NO_RID_KINDS:
                continue
            reaction.reaction_id = rid
            rid += 1

        # Recurse into sub-reactors (instances)
        for inst in reactor.instances:
            sub = class_to_reactor.get(inst.reactor_class)
            if sub is None or inst.reactor_class in seen_reactors:
                continue
            seen_reactors.add(inst.reactor_class)
            for reaction in sub.reactions:
                if reaction.kind in CORR_NO_RID_KINDS:
                    continue
                reaction.reaction_id = rid
                rid += 1


def _fill_corr_rids(
    corr_edges: list[CorrEdge], class_to_reactor: dict[str, LFReactor],
) -> None:
    """Copy the publishing and drain reactions' RIDs onto the admitted edges."""
    by_id = {e.edge_id: e for e in corr_edges if e.edge_id is not None}
    for edge in by_id.values():
        edge.pub_rids = []
    for reactor in class_to_reactor.values():
        for reaction in reactor.reactions:
            for eid in reaction.corr_emit_edges:
                by_id[eid].pub_rids.append(reaction.reaction_id)
            for eid in reaction.corr_drain_members or [reaction.corr_drain_edge]:
                if eid is not None:
                    by_id[eid].sub_rid = reaction.reaction_id
    for edge in by_id.values():
        edge.pub_rids.sort()
        # An egress edge has no drain here, an ingress edge no publisher.
        pubs = 0 if edge.role == "ingress" else len(edge.pub_callbacks)
        if len(edge.pub_rids) != pubs or (edge.sub_rid < 0) != (edge.role == "egress"):
            raise RuntimeError(
                f"admitted correspondence edge {edge.edge_id} {edge.label()}: "
                f"{len(edge.pub_rids)} publishing reaction(s) for callbacks "
                f"{edge.pub_callbacks} (pub_rids={edge.pub_rids}), "
                f"sub_rid={edge.sub_rid}"
            )


# ---------------------------------------------------------------------------
# Reactor building
# ---------------------------------------------------------------------------


def _build_reactor(
    node_info: NodeInfo,
    *,
    binding_key: str | None = None,
    corr: _NodeCorr = _NO_CORR,
    check_bna: bool = True,
    timers_first: bool = False,
) -> tuple[LFReactor, list[LFReactor]]:
    """Build an LFReactor (and optional sub-reactors) from a NodeInfo.

    *binding_key* is the per-instance key from ``binding_key.py``; it names the
    reactor, the ``ros2_lf_bind_node`` string, and every ``RID_*`` macro.
    Defaults to the plain class name (single-instance behavior).
    *timers_first* declares timer reactions before subscription reactions
    (``reaction_order``).
    """
    key = binding_key if binding_key is not None else node_info.class_name
    if check_bna:
        corr = _with_bna_inputs(node_info, corr)
        _check_sync_inputs(node_info, corr)
        node_info = _order_sync_drains(node_info)
    if not node_info.callbacks:
        return _build_reactor_simple(node_info, binding_key=key, corr=corr), []
    return _build_reactor_with_groups(
        node_info, binding_key=key, corr=corr, timers_first=timers_first,
    )


def _build_reactor_simple(
    node_info: NodeInfo,
    *,
    binding_key: str | None = None,
    corr: _NodeCorr = _NO_CORR,
) -> LFReactor:
    """Build a flat reactor (no nested sub-reactors)."""
    key = binding_key if binding_key is not None else node_info.class_name
    reactor_name = class_name_to_reactor_name(key)

    # --- Ports ---
    ports = _build_ports(node_info, stub=True, corr=corr)

    # --- Timers ---
    timers = _build_timers(node_info.timers)

    # --- Physical actions for blocking service clients ---
    physical_actions: list[LFPhysicalAction] = []
    physical_actions = _build_physical_actions(node_info)

    # --- Build lookup maps for reaction wiring ---
    timer_trigger_map: dict[str, str] = {}
    for t in node_info.timers:
        timer_trigger_map[t.callback] = callback_to_timer_name(t.callback)

    sub_trigger_map: dict[str, str] = {}
    for s in node_info.subscribers:
        port_name = _sub_port_name(node_info, s.topic)
        sub_trigger_map[s.callback] = port_name
    sub_trigger_map.update(_corr_sub_triggers(corr))

    # subscriber callback → topic name (for ros2_lf_invoke_sub)
    sub_topic_map: dict[str, str] = {}
    for s in node_info.subscribers:
        sub_topic_map[s.callback] = s.topic

    pub_port_map = _build_pub_port_map(node_info.publishers, corr)

    srv_info = _build_srv_maps(node_info)

    # --- Reactions ---
    reactions = _build_reactions(
        node_info.callbacks,
        timer_trigger_map,
        sub_trigger_map,
        sub_topic_map,
        pub_port_map,
        key,
        srv_info=srv_info,
        corr=corr,
        node_info=node_info,
    )

    reactions += [
        r for _, r in _build_srv_reactions(
            node_info, srv_info, binding_key=key,
        )
    ]
    reactions += [
        r for _, r in _build_action_reactions(
            node_info, binding_key=key, external=corr.external_actions,
        )
    ]

    startup_body: str | None = None
    startup_body = f'ros2_lf_bind_node(self, "{key}");'

    return LFReactor(
        name=reactor_name,
        ports=ports,
        timers=timers,
        physical_actions=physical_actions,
        reactions=reactions,
        logical_actions=_corr_logical_actions(reactions),
        startup_body=startup_body,
        ingress_key=key,
        external_inputs=_external_inputs(
            [cb.name for cb in node_info.callbacks], corr, key,
        ),
    )


def _build_sub_ports_and_timers(
    cb_names: list[str],
    node_info: NodeInfo,
    timer_trigger_map: dict[str, str],
    sub_trigger_map: dict[str, str],
    pub_port_map: dict[str, str],
    port_type_map: dict[str, str],
    port_lf_type: str,
    srv_info: "_SrvMaps | None" = None,
    corr: _NodeCorr = _NO_CORR,
) -> tuple[list[LFPort], list[LFTimer]]:
    """Build ports and timers for a sub-reactor containing the given callbacks."""
    needed_inputs: set[str] = set()
    needed_outputs: set[str] = set()
    sub_timers: list[LFTimer] = []

    cb_map_local = {cb.name: cb for cb in node_info.callbacks}
    for cb_name in cb_names:
        if cb_name in timer_trigger_map:
            for t in node_info.timers:
                if t.callback == cb_name:
                    sub_timers.append(_make_timer(t))
                    break
        if (
            cb_name in sub_trigger_map and cb_name not in corr.external_subs
            and not corr.is_local_sub(cb_name)
        ):
            needed_inputs.add(sub_trigger_map[cb_name])
        # Every member port of an admitted fan-in group.
        needed_inputs.update(corr.sub_ports(cb_name))
        cb_info = cb_map_local.get(cb_name)
        if cb_info:
            for pub_topic in cb_info.publishers_used:
                port_name = pub_port_map.get(pub_topic)
                if port_name:
                    needed_outputs.add(port_name)
            for edge_id in corr.pub_edges.get(cb_name, []):
                if edge_id not in corr.local:
                    needed_outputs.add(corr.port(edge_id))
            # Add service request output ports for async service client sends
            if srv_info:
                for svc_name in cb_info.service_clients_used:
                    if svc_name in srv_info.async_client_services:
                        needed_outputs.add(service_to_request_port_name(svc_name))
            # The action channels a timer or subscription
            # callback may send on (requests of its clients, feedback and
            # results of its node's servers).
            if cb_name in timer_trigger_map or cb_name in sub_trigger_map:
                for port, _, _ in _callback_action_claims(cb_info, node_info):
                    needed_outputs.add(port)

    sub_ports: list[LFPort] = []
    for inp in sorted(needed_inputs):
        lf_type = port_type_map.get(inp, port_lf_type)
        sub_ports.append(LFPort(name=inp, type=lf_type, direction="input"))
    for out in sorted(needed_outputs):
        lf_type = port_type_map.get(out, port_lf_type)
        sub_ports.append(LFPort(name=out, type=lf_type, direction="output"))
    return sub_ports, sub_timers


def _add_forwarding_connections(
    node_connections: list[LFConnection],
    sub_ports: list[LFPort],
    inst_name: str,
) -> None:
    """Append outer↔sub port-forwarding connections for every port on the sub."""
    for port in sub_ports:
        if port.direction == "input":
            node_connections.append(
                LFConnection(source=port.name, destination=f"{inst_name}.{port.name}")
            )
        else:
            node_connections.append(
                LFConnection(source=f"{inst_name}.{port.name}", destination=port.name)
            )


def _drop_unwritten_outputs(
    instances: list[LFReactorInstance],
    sub_reactors: list[LFReactor],
    node_connections: list[LFConnection],
) -> None:
    """Remove sub-reactor output ports that no reaction of the sub-reactor writes.

    A port comes from the callbacks placed in a group, but a callback without a
    trigger yields no reaction: a helper that lambda timers call (``on_b``) is
    analyzed as a callback with the lambdas' publishers and lands in the default
    group. lfc leaves the container's pointer to such an output NULL in a
    federate, so a merge reaction that reads it segfaults. Its forwarding
    connection goes too, so the port is never counted as a writer.
    """
    for inst, sub in zip(instances, sub_reactors):
        written = {e for r in sub.reactions for e in r.effects}
        dead = {
            p.name for p in sub.ports if p.direction == "output" and p.name not in written
        }
        if not dead:
            continue
        sub.ports[:] = [p for p in sub.ports if p.name not in dead]
        node_connections[:] = [
            c for c in node_connections
            if c.source not in {f"{inst.name}.{name}" for name in dead}
        ]


def _stub_add(port: str, n: str = "1") -> str:
    """Add *n* writes to stub port *port* at this tag.

    Reads back what earlier reactions of this reactor set at this tag, so
    several writers of one port in one callback group are all counted. A
    checked add exits the program instead of letting the count wrap, as the
    merge does.
    """
    return (
        f"{{ stub_t rcllf_c = {port}->is_present ? {port}->value : 0; "
        f"if (__builtin_add_overflow(rcllf_c, {n}, &rcllf_c)) {{ lf_print_error_and_exit("
        f'"rcllf: write count of port \'{port}\' overflows int"); }} '
        f"lf_set({port}, rcllf_c); }}"
    )


def _if_ran(invoke: str, writes: list[str]) -> list[str]:
    """Count *writes* only when the *invoke* statement ran a callback.

    The legacy invokes return 1 for a callback that ran and 0 for an empty
    take, an inactive lifecycle node or a shutdown in progress; a write
    counted for a callback that did not run would make the subscriber take a
    message of a later tag.
    """
    if not writes:
        return [invoke]
    return [f"if ({invoke.rstrip(';')} > 0) {{"] + [f"    {w}" for w in writes] + ["}"]


def _count_publishes() -> bool:
    """Count publishes, not callback runs, on legacy stub ports (P17e, default since P17f).

    A timer or subscription callback raises each publisher stub port by the
    publishes the binding saw on that topic during the invoke
    (``ros2_lf_topic_sent``), so a callback that publishes nothing triggers
    nothing downstream, and the subscription invoke learns how many writes of
    its trigger remain (``ros2_lf_invoke_sub_n``), which the take debt rule
    needs. ``RCLLF_COUNT_PUBLISHES=0`` at generation time restores one write
    per callback run, for nodes that publish from threads the binding does not
    see. Their subscriptions then call ``ros2_lf_invoke_sub``, whose misses owe
    no debt, because an empty trigger looks like a late payload (P17f).
    """
    return os.environ.get("RCLLF_COUNT_PUBLISHES", "") != "0"


def _sent_add(port: str, rid_macro: str, topic: str) -> str:
    """Raise stub *port* by the publishes on *topic* of the last invoke (P17e)."""
    esc = topic.replace('"', '\\"')
    return (
        f'{{ int n = ros2_lf_topic_sent({rid_macro}, "{esc}"); '
        f"if (n > 0) {{ {_stub_add(port, 'n')} }} }}"
    )


def _service_sent_add(port: str, rid_macro: str, service: str) -> str:
    """Raise request *port* by the requests sent on *service* (review round 5, O1-2).

    A callback that sends no request (its client found no server, a branch
    skipped the send) writes nothing, so the server runs no handler for a
    request that does not exist, and two sends write twice.
    """
    esc = service.replace('"', '\\"')
    return (
        f'{{ int n = ros2_lf_service_sent({rid_macro}, "{esc}"); '
        f"if (n > 0) {{ {_stub_add(port, 'n')} }} }}"
    )


def _per_write(port: str, lines: list[str]) -> list[str]:
    """Run *lines* once per write the present stub *port* carries."""
    return (
        [f"for (stub_t rcllf_k = {port}->is_present ? {port}->value : 0; rcllf_k > 0; "
         "rcllf_k--) {"]
        + [f"    {line}" for line in lines]
        + ["}"]
    )


def _merge_shared_outputs(node_connections: list[LFConnection]) -> list[LFReaction]:
    """Merge outer output ports that more than one sub-reactor writes.

    lfc allows one connection into a port. When callbacks of several
    sub-reactors publish one topic (two groups, or a Reentrant group with
    two callbacks), their forwarding connections are replaced by a reaction
    of the node reactor that writes the outer port once per tag with the sum
    of the writers' counts, so a stub subscriber takes one
    message per write. One event per tag also survives an ``after d`` link,
    which maps every microstep of a time to one tag. Each sub-reactor keeps
    its own logical order. Writes of different sub-reactors at the same tag
    have no order between them, as in ROS, because their reactions run in
    parallel and publish to DDS directly.

    The sum is an int, and a checked add exits the program instead of
    letting it wrap.
    """
    writers: dict[str, list[str]] = {}
    for conn in node_connections:
        if "." in conn.source and "." not in conn.destination:
            writers.setdefault(conn.destination, []).append(conn.source)
    shared = {port: srcs for port, srcs in writers.items() if len(srcs) > 1}
    if not shared:
        return []
    node_connections[:] = [
        c for c in node_connections if c.destination not in shared or "." not in c.source
    ]
    return [
        LFReaction(triggers=list(srcs), effects=[port], body=_sum_body(port, srcs, [port]),
                   kind=PORT_MERGE_KIND)
        for port, srcs in shared.items()
    ]


def _sum_body(port: str, srcs: list[str], effects: list[str]) -> str:
    """Sum the write counts of the present *srcs* and set every effect to the sum.

    The sum is an int, and a checked add exits the program instead of
    letting it wrap. *port* names the merged port in the error.
    """
    adds = "\n".join(
        f"if ({src}->is_present && __builtin_add_overflow(n, {src}->value, &n)) {{\n"
        f"    lf_print_error_and_exit(\n"
        f'        "rcllf: write count of port \'{port}\' overflows int");\n'
        f"}}"
        for src in srcs
    )
    sets = " ".join(f"lf_set({e}, n);" for e in effects)
    return f"int n = 0;\n{adds}\nif (n > 0) {{ {sets} }}"


def fan_in_port_name(port: str, k: int) -> str:
    """Input port of subscriber port *port* for its *k*-th publisher node, k >= 1."""
    return port if k == 0 else f"{port}_fanin_{k}"


def topic_fan_in(connections: list[LFConnection]) -> dict[str, list[str]]:
    """Stub topic destinations linked from several publisher ports.

    Maps ``inst.port`` to its sources, sorted, so every federate of one
    system computes the same order from the whole-system links. lfc takes
    one connection into a port, so source *k* is routed to
    ``fan_in_port_name(port, k)`` (``route_fan_in``) and the subscriber node
    reactor merges those inputs (``_merge_fan_in``).
    """
    sources: dict[str, set[str]] = {}
    for conn in connections:
        if conn.topic is not None:
            sources.setdefault(conn.destination, set()).add(conn.source)
    return {dst: sorted(srcs) for dst, srcs in sources.items() if len(srcs) > 1}


def route_fan_in(connections: list[LFConnection], fan_in: dict[str, list[str]]) -> None:
    """Point every stub topic link of *fan_in* at its publisher's own input port."""
    for conn in connections:
        srcs = fan_in.get(conn.destination) if conn.topic is not None else None
        if srcs:
            inst, port = conn.destination.split(".", 1)
            conn.user_destination = conn.destination
            conn.destination = f"{inst}.{fan_in_port_name(port, srcs.index(conn.source))}"


def _landing_delay_ns(conn: LFConnection) -> int:
    """After delay that moves a write of *conn*'s source to the subscriber's tag."""
    if conn.is_cross_federate and conn.federate_after_ns is not None:
        return conn.federate_after_ns
    return conn.after_delay_ns or 0


def fan_in_tag_errors(connections: list[LFConnection], federate_of, *,
                      federated: bool = True) -> list[str]:
    """Stub fan-ins whose writers can land on different tags.

    Probe P17h. The merged input carries only a write count and the
    subscription takes its samples from one DDS queue in arrival order. The
    samples handled at a tag are the ones written for that tag only if every
    writer publishes all its samples of one tag before any writer publishes a
    later one: all writers in one federate (it advances tag by tag) with one
    after delay. With unequal delays, or writers in federates that advance
    independently (also under centralized coordination), a sample can be
    handled at another writer's tag. *federate_of* maps an instance to its
    federate (``""`` for an unfederated program). Unequal delays split an
    unfederated fan-in as well: an earlier version tried narrowing the refusal to
    federated programs, and the run swapped 287 of 500 samples per writer. *federated*
    picks the fix the message suggests: an unfederated program has no
    federates to move publishers into.
    """
    groups: dict[str, list[LFConnection]] = {}
    for conn in connections:
        if conn.user_destination is not None:
            groups.setdefault(conn.user_destination, []).append(conn)
    errors = []
    for dst, conns in sorted(groups.items()):
        conns = sorted(conns, key=lambda c: c.source)
        feds = {federate_of(c.source.split(".", 1)[0]) for c in conns}
        delays = {_landing_delay_ns(c) for c in conns}
        if len(feds) <= 1 and len(delays) <= 1:
            continue
        writers = ", ".join(
            f"{c.source} (federate {federate_of(c.source.split('.', 1)[0]) or '-'}, "
            f"after {_landing_delay_ns(c)} ns)" for c in conns)
        topic = conns[0].topic
        fix = (f"put every publisher node of {topic} into one federate with one after "
               f"delay to {dst}" if federated else
               f"give every publisher node of {topic} one equal after delay to {dst}")
        errors.append(
            f"legacy fan-in into {dst} on topic {topic}: writers {writers} can land on "
            "different tags, and the merged port carries only a write count, so the sample "
            "handled at one tag can be another writer's sample of another tag (probe P17h). "
            f"Fix: {fix}, or publish {topic} from one node")
    return errors


def xtag_publisher_errors(connections: list[LFConnection], federate_of) -> list[str]:
    """Topics with a cross-federate tag link and publisher nodes in several federates.

    A cross-federate link carries the writer federate's tag log
    events of its topic, numbered in the order that federate journaled them.
    Publisher nodes of one federate share that numbering, and each of their
    links carries the events since its own last message, so the receiver's log
    equals the writer's. Publishers in two federates number their events
    apart, and the receiver cannot interleave the two numberings in publish
    order, so the samples could not be matched to their tags.
    """
    publishers: dict[str, set[str]] = {}
    crossing: set[str] = set()
    for conn in connections:
        if conn.topic is None:
            continue
        src_inst = conn.source.split(".", 1)[0]
        publishers.setdefault(conn.topic, set()).add(src_inst)
        if federate_of(src_inst) != federate_of(conn.destination.split(".", 1)[0]):
            crossing.add(conn.topic)
    errors = []
    for topic in sorted(crossing):
        nodes = sorted(publishers[topic])
        if len({federate_of(n) for n in nodes}) < 2:
            continue
        names = ", ".join(f"{n} (federate {federate_of(n) or '-'})" for n in nodes)
        errors.append(
            f"cross-federate take link on topic {topic}: publisher nodes {names} publish "
            "from different federates, whose tag log events are numbered apart, so the "
            "receiver cannot tell which publisher's sample belongs to which tag. "
            f"Fix: publish {topic} from nodes of one federate, or generate with "
            "RCLLF_COUNT_PUBLISHES=0, whose links carry only a callback count and drop no "
            "late sample by tag")
    return errors


def xtag_publisher_count(
    info: NodeInfo | None, scope: NodeIdentity | None, topic: str,
) -> tuple[int, str | None]:
    """Gated publishers of resolved *topic* in one writer node, or why the count is not exact.

    Fourth review. The receiver waits for this many
    announced publishers before it reads an unannounced writer as plain, so
    an undercount reads a late-created gated publisher's sample as plain and
    an overcount holds a plain writer's samples for good. The count is used
    only where the analysis proves it: every publisher of the node has a
    static name, resolved with the namespace, remaps and node name the
    crossing graph resolved the topic with (``resolve_name``); without a
    launch identity that resolution must agree with the generator's endpoint
    match; every publisher of the topic is created in the constructor
    (``created_in``) on every path through it (``unconditional``, fifth
    review: a create call under a branch, loop, lambda or try block, or after
    a return, may never run).
    Otherwise the second item names the reason and the link keeps the count
    rule.
    """
    if info is None:
        return 0, "the publisher node has no analysis"
    pubs = list(getattr(info, "publishers", None) or [])
    if any(p.topic_dynamic or not (p.topic or "").strip() for p in pubs):
        return 0, f"node {info.name} has a publisher with a dynamic or unresolved topic name"
    ns = (scope.namespace or None) if scope is not None else None
    remaps = list(scope.remappings) if scope is not None else []
    node_name = scope.node_name if scope is not None else info.name
    mine = [p for p in pubs if resolve_name(p.topic, ns, remaps, node_name) == topic]
    if not (scope is not None and scope.from_launch):
        matched = _match_endpoint_infos(topic, pubs, scope)
        if len(matched) != len(mine) or any(m not in mine for m in matched):
            return 0, (f"node {info.name} has no launch namespace, and its publisher names "
                       f"match {topic} ambiguously")
    if not mine:
        return 0, f"no publisher of {topic} found in the analysis of node {info.name}"
    late = [p for p in mine if p.created_in not in (None, "constructor")]
    if late:
        return 0, (f"node {info.name} creates a publisher of {topic} in {late[0].created_in}, "
                   "which may never run")
    if is_lifecycle_base(info.base_class, info.lifecycle_callbacks) and any(
            p.created_in != "constructor" for p in mine):
        return 0, (f"lifecycle node {info.name} may create its publisher of {topic} outside "
                   "its constructor")
    # Fifth review: a publisher counts only when the analysis saw its create
    # call in the constructor, on every path through it.
    if any(p.created_in is None for p in mine):
        return 0, (f"creation site unknown: the analysis of node {info.name} does not say "
                   f"where it creates its publisher of {topic}")
    if any(p.unconditional is not True for p in mine):
        undecided = all(p.unconditional is None for p in mine if p.unconditional is not True)
        return 0, (f"conditional creation: node {info.name} may skip creating a publisher "
                   f"of {topic} (a branch, loop, lambda, try block or earlier return "
                   "encloses or precedes the call)" if not undecided else
                   f"conditional creation: the analysis of node {info.name} did not decide "
                   f"whether its constructor always creates its publisher of {topic}")
    return len(mine), None


def _merge_fan_in(reactor: LFReactor, port: str, n: int) -> None:
    """Merge the *n* fan-in inputs of stub input *port* of a node reactor.

    Several publisher nodes on one topic each get their own input port, and
    one reaction of the node reactor sums their write counts into one event
    on every group sub-reactor that the port forwarded to, consistent with
    the write count: the subscription runs once per write of
    either publisher. Writes of different publisher nodes at one tag have no
    order between them, as in ROS.
    """
    extra = [fan_in_port_name(port, k) for k in range(1, n)]
    inputs = [p for p in reactor.ports if p.name == port and p.direction == "input"]
    if len(inputs) != 1 or inputs[0].type != STUB_TYPE:
        raise ValueError(
            f"Reactor {reactor.name!r}: {n} publisher nodes link into {port!r}, "
            "which is not one stub input port, so they cannot be merged")
    taken = (
        {p.name for p in reactor.ports} | {i.name for i in reactor.instances}
        | {t.name for t in reactor.timers} | {a.name for a in reactor.physical_actions}
        | set(reactor.logical_actions)
    )
    clash = [name for name in extra if name in taken]
    if clash:
        raise ValueError(
            f"Reactor {reactor.name!r}: fan-in input {clash[0]!r} of {port!r} is already "
            "declared, rename the topic or the declared name")
    if any(port in r.triggers or port in r.sources for r in reactor.reactions):
        raise ValueError(
            f"Reactor {reactor.name!r}: a reaction of the node reactor reads {port!r}, "
            "so its publisher nodes cannot be merged")
    reactor.ports += [LFPort(name=name, type=STUB_TYPE, direction="input") for name in extra]
    forwards = [c.destination for c in reactor.connections if c.source == port]
    if not forwards:
        return
    reactor.connections[:] = [c for c in reactor.connections if c.source != port]
    reactor.reactions.append(LFReaction(
        triggers=[port, *extra], effects=forwards, body=_sum_body(port, [port, *extra], forwards),
        kind=PORT_MERGE_KIND,
    ))


def _build_reactor_with_groups(
    node_info: NodeInfo,
    *,
    binding_key: str | None = None,
    corr: _NodeCorr = _NO_CORR,
    timers_first: bool = False,
) -> tuple[LFReactor, list[LFReactor]]:
    """Build a node-shell reactor whose callbacks live in per-group sub-reactors.

    Mapping:
    - MutuallyExclusive group  → one sub-reactor per group (LF serializes
      reactions within a reactor, preserving within-group mutex).
    - Reentrant group          → one sub-reactor per *callback* in the group
      (sibling reactors run in parallel under reactor-c's multi-threaded
      scheduler, preserving cross-callback reentrant concurrency).
    - default / implicit group → treated as MutuallyExclusive.

    The outer reactor holds only external ports, a startup reaction, and
    port-forwarding connections.  All user reactions live in sub-reactors,
    including service server/client and action server/client reactions which
    are routed into the group sub-reactor that matches their callback_group.

    Physical actions (for blocking service clients) are kept on the outer
    reactor as node-level resources; revisit in Phase 6 (service semantics).
    """
    key = binding_key if binding_key is not None else node_info.class_name
    reactor_name = class_name_to_reactor_name(key)

    # External ports visible to the main reactor (unchanged)
    ports = _build_ports(node_info, stub=True, corr=corr)
    port_type_map = {p.name: p.type for p in ports}
    port_lf_type = STUB_TYPE

    group_map: dict[str, CallbackGroupInfo] = {
        g.name: g for g in node_info.callback_groups
    }

    # Map each callback to its group name.
    # None (no explicit group) is normalized to "default_group" so it merges
    # with the synthesized implicit-default group from the analyzer.
    _DEFAULT = "default_group"
    callback_to_group: dict[str, str] = {}
    for t in node_info.timers:
        callback_to_group[t.callback] = t.callback_group or _DEFAULT
    for s in node_info.subscribers:
        callback_to_group[s.callback] = s.callback_group or _DEFAULT

    # Bucket callbacks by group name, preserving node_info.callbacks order
    group_callbacks: dict[str, list[str]] = {}
    for cb in node_info.callbacks:
        grp = callback_to_group.get(cb.name, _DEFAULT)
        group_callbacks.setdefault(grp, []).append(cb.name)

    timer_trigger_map: dict[str, str] = {
        t.callback: callback_to_timer_name(t.callback) for t in node_info.timers
    }
    sub_trigger_map: dict[str, str] = {
        s.callback: _sub_port_name(node_info, s.topic) for s in node_info.subscribers
    }
    sub_trigger_map.update(_corr_sub_triggers(corr))
    sub_topic_map: dict[str, str] = {
        s.callback: s.topic for s in node_info.subscribers
    }
    pub_port_map = _build_pub_port_map(node_info.publishers, corr)
    cb_map: dict[str, CallbackInfo] = {cb.name: cb for cb in node_info.callbacks}
    srv_info_grp = _build_srv_maps(node_info)

    # Build tagged (group_name, reaction) pairs for service/action reactions.
    # These are bucketed into the same sub-reactors as timer/sub reactions.
    srv_action_tagged = (
        _build_srv_reactions(
            node_info, srv_info_grp, binding_key=key,
        )
        + _build_action_reactions(
            node_info, binding_key=key, external=corr.external_actions,
        )
    )
    # Bucket by group name; reactions with no group join "default_group".
    srv_action_by_group: dict[str, list[LFReaction]] = {}
    for grp, reaction in srv_action_tagged:
        srv_action_by_group.setdefault(grp or _DEFAULT, []).append(reaction)

    # Ensure every group that has srv/action reactions appears in ordered_group_names
    # (even if no timer/sub callback lives in that group).
    all_groups_needed: set[str] = (
        set(group_callbacks.keys()) | set(srv_action_by_group.keys())
    )

    instances: list[LFReactorInstance] = []
    sub_reactors: list[LFReactor] = []
    node_connections: list[LFConnection] = []
    default_group_reactor: LFReactor | None = None

    # Groups are emitted in node_info.callback_groups order; any group needed
    # but not explicitly declared (e.g. "default_group" from hand-built fixtures
    # that omit the CallbackGroupInfo) is appended at the end.
    ordered_group_names: list[str] = [
        g.name for g in node_info.callback_groups
    ]
    for grp in all_groups_needed:
        if grp not in ordered_group_names:
            ordered_group_names.append(grp)

    for group_name in ordered_group_names:
        cb_names = group_callbacks.get(group_name, [])
        extra_reactions = srv_action_by_group.get(group_name, [])
        if not cb_names and not extra_reactions:
            continue

        group_info = group_map.get(group_name)
        is_reentrant = group_info is not None and group_info.type == "Reentrant"

        if is_reentrant:
            # One sub-reactor per callback within the Reentrant group.
            # Service/action reactions for this group share the per-group sub-reactor
            # (they cannot self-concur anyway, so no per-reaction split needed).
            for cb_name in cb_names:
                sub_class = f"{reactor_name}_{_to_pascal(cb_name)}"
                inst_name = sanitize_lf_identifier(cb_name)
                cb_info = cb_map.get(cb_name)

                sub_ports, sub_timers = _build_sub_ports_and_timers(
                    [cb_name], node_info, timer_trigger_map, sub_trigger_map,
                    pub_port_map, port_type_map, port_lf_type, srv_info=srv_info_grp,
                    corr=corr,
                )
                sub_reactions = _build_reactions(
                    [cb_info] if cb_info else [],
                    {cb_name: timer_trigger_map[cb_name]} if cb_name in timer_trigger_map else {},
                    {cb_name: sub_trigger_map[cb_name]} if cb_name in sub_trigger_map else {},
                    {cb_name: sub_topic_map[cb_name]} if cb_name in sub_topic_map else {},
                    pub_port_map,
                    key,
                    srv_info=srv_info_grp,
                    corr=corr,
                    node_info=node_info,
                    timers_first=timers_first,
                )

                warnings.warn(
                    f"Node {node_info.class_name!r}: reentrant callback "
                    f"{cb_name!r} cannot be invoked concurrently with itself "
                    f"under LF (reactor instances are singletons). If parallel "
                    f"self-invocation is required, declare a reactor bank in "
                    f"your .lf source. "
                    f"See docs/callback-groups.md#reactor-bank-escape-hatch.",
                    UserWarning,
                    stacklevel=2,
                )

                sub_reactors.append(LFReactor(
                    name=sub_class,
                    ports=sub_ports,
                    timers=sub_timers,
                    reactions=sub_reactions,
                    logical_actions=_corr_logical_actions(sub_reactions),
                    external_inputs=_external_inputs([cb_name], corr, key),
                ))
                instances.append(LFReactorInstance(name=inst_name, reactor_class=sub_class))
                _add_forwarding_connections(
                    node_connections, sub_ports, inst_name,
                )

            # Service/action reactions for this reentrant group go into a shared
            # MutuallyExclusive sub-reactor (they cannot be self-concurrent).
            if extra_reactions:
                sa_sub_name = f"{sanitize_lf_identifier(group_name.rstrip('_'))}_sa"
                sa_sub_class = f"{reactor_name}_{_to_pascal(sa_sub_name)}"
                sa_ports = _ports_for_reactions(
                    extra_reactions, port_type_map, port_lf_type,
                )
                sub_reactors.append(LFReactor(
                    name=sa_sub_class,
                    ports=sa_ports,
                    reactions=extra_reactions,
                ))
                instances.append(LFReactorInstance(name=sa_sub_name, reactor_class=sa_sub_class))
                _add_forwarding_connections(node_connections, sa_ports, sa_sub_name)
        else:
            # One sub-reactor for the whole MutuallyExclusive group
            sub_name = sanitize_lf_identifier(group_name.rstrip("_"))
            sub_class = f"{reactor_name}_{_to_pascal(sub_name)}"

            sub_ports, sub_timers = _build_sub_ports_and_timers(
                cb_names, node_info, timer_trigger_map, sub_trigger_map,
                pub_port_map, port_type_map, port_lf_type, srv_info=srv_info_grp,
                corr=corr,
            )
            sub_reactions = _build_reactions(
                [cb_map[n] for n in cb_names if n in cb_map],
                {cb: timer_trigger_map[cb] for cb in cb_names if cb in timer_trigger_map},
                {cb: sub_trigger_map[cb] for cb in cb_names if cb in sub_trigger_map},
                {cb: sub_topic_map[cb] for cb in cb_names if cb in sub_topic_map},
                pub_port_map,
                key,
                srv_info=srv_info_grp,
                corr=corr,
                node_info=node_info,
                timers_first=timers_first,
            )

            # Merge in service/action reactions and their ports for this group.
            if extra_reactions:
                sub_reactions += extra_reactions
                extra_ports = _ports_for_reactions(
                    extra_reactions, port_type_map, port_lf_type,
                )
                existing_port_names = {p.name for p in sub_ports}
                for p in extra_ports:
                    if p.name not in existing_port_names:
                        sub_ports.append(p)

            sub_reactors.append(LFReactor(
                name=sub_class,
                ports=sub_ports,
                timers=sub_timers,
                reactions=sub_reactions,
                logical_actions=_corr_logical_actions(sub_reactions),
                external_inputs=_external_inputs(cb_names, corr, key),
            ))
            if group_name == _DEFAULT:
                # Drain host: the MutuallyExclusive default group, so ingress
                # requests keep ROS exclusion with its callbacks (R11).
                default_group_reactor = sub_reactors[-1]
                default_group_reactor.ingress_key = key
            instances.append(LFReactorInstance(name=sub_name, reactor_class=sub_class))
            _add_forwarding_connections(
                node_connections, sub_ports, sub_name,
            )

    # Physical actions stay on the outer reactor (node-level resource)
    physical_actions: list[LFPhysicalAction] = _build_physical_actions(node_info)

    _drop_unwritten_outputs(instances, sub_reactors, node_connections)
    merge_reactions = _merge_shared_outputs(node_connections)
    for reaction in merge_reactions:
        if any(
            port_type_map.get(port) not in (STUB_TYPE, ACTION_PORT_TYPE)
            for port in reaction.effects[:1]
        ):
            raise ValueError(
                f"Node {node_info.class_name!r}: port {reaction.effects[0]!r} is written by "
                "several sub-reactors but is not a stub port, so it cannot be merged"
            )

    # Order bind before every group's first dispatch. Added after
    # the forwarding connections, BIND_PORT is written by the outer startup
    # reaction directly (never forwarded from an outer port).
    bind_lines = [f'ros2_lf_bind_node(self, "{key}");']
    startup_effects: list[str] = []
    for inst, sub in zip(instances, sub_reactors):
        sub.ports.append(LFPort(name=BIND_PORT, type=STUB_TYPE, direction="input"))
        for reaction in sub.reactions:
            # The corr and BNA startup and shutdown reactions run no user code.
            if reaction.kind in _UNBOUND_KINDS:
                continue
            if BIND_PORT not in reaction.sources:
                reaction.sources.append(BIND_PORT)
        if sub.ingress_key is not None:
            sub.ingress_bound = True
        startup_effects.append(f"{inst.name}.{BIND_PORT}")
        bind_lines.append(f"lf_set({inst.name}.{BIND_PORT}, 1);")

    startup_body: str | None = "\n".join(bind_lines)

    # Reentrant or empty default group: the outer node reactor hosts the
    # drain (R11). It has no user reactions, so it serializes only the drain.
    outer_ingress_key = key if default_group_reactor is None else None
    return (
        LFReactor(
            name=reactor_name,
            ports=ports,
            physical_actions=physical_actions,
            reactions=merge_reactions,
            instances=instances,
            connections=node_connections,
            startup_body=startup_body,
            startup_effects=startup_effects,
            ingress_key=outer_ingress_key,
        ),
        sub_reactors,
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _build_ports(
    node_info: NodeInfo, *, stub: bool = False, corr: _NodeCorr = _NO_CORR,
) -> list[LFPort]:
    """Build input and output LFPorts from a NodeInfo.

    Stub ports of topics whose every edge is admitted are
    dropped, and the admitted edges' ``corr_e<ID>`` ports are added. Names of
    the remaining ports are deduplicated over the full list, so they do not
    change when a sibling is dropped.
    """
    ports: list[LFPort] = []
    lf_type_fn = (lambda _: STUB_TYPE) if stub else message_type_to_lf_type

    out_names = [topic_to_port_name(p.topic) for p in node_info.publishers]
    out_names = deduplicate_names(out_names)
    # One input port per topic: two subscriptions on one topic in one node
    # share the port and each gets its own reaction on it.
    in_subs = []
    for sub in node_info.subscribers:
        if all(sub.topic != seen.topic for seen in in_subs):
            in_subs.append(sub)
    in_names = deduplicate_names([_sub_port_name(node_info, s.topic) for s in in_subs])

    for i, pub in enumerate(node_info.publishers):
        if pub.topic in corr.dropped_pub_topics:
            continue
        ports.append(LFPort(
            name=out_names[i],
            type=lf_type_fn(pub.message_type),
            direction="output",
        ))

    for i, sub in enumerate(in_subs):
        if sub.topic in corr.dropped_sub_topics:
            continue
        ports.append(LFPort(
            name=in_names[i],
            type=lf_type_fn(sub.message_type),
            direction="input",
        ))

    ports += corr.ports()

    # Service and action ports are named from the full declared name, which
    # maps "/a/b" and "/a_b" both to "a_b_req". Another declared name on a
    # port, or one port in both directions (a node serving and calling one
    # service), is refused, since lfc rejects the duplicate; the same
    # declared name twice shares its ports.
    origin: dict[str, tuple[str, str]] = {}

    def add(declared: str, kind: str, name: str, direction: str,
            port_type: str = STUB_TYPE) -> None:
        other, other_dir = origin.setdefault(name, (declared, direction))
        if other != declared:
            raise ValueError(
                f"Node {node_info.name!r}: {kind} name {declared!r} and {other!r} "
                f"both give the LF port {name!r}; rename one of them"
            )
        if other_dir != direction:
            raise ValueError(
                f"Node {node_info.name!r}: {kind} {declared!r} needs the LF port "
                f"{name!r} as both input and output (the node serves and calls it)"
            )
        if name not in {p.name for p in ports}:
            ports.append(LFPort(name=name, type=port_type, direction=direction))

    # Service server ports: input request, output response
    for ss in node_info.service_servers:
        add(ss.service_name, "service", service_to_request_port_name(ss.service_name), "input")
        add(ss.service_name, "service", service_to_response_port_name(ss.service_name), "output")

    # Async service client ports: output request, input response
    for sc in node_info.service_clients:
        if sc.call_style in ("async_callback", None):
            add(sc.service_name, "service", service_to_request_port_name(sc.service_name), "output")
            add(sc.service_name, "service", service_to_response_port_name(sc.service_name), "input")

    # Action ports: counted, see ACTION_PORT_TYPE. Requests flow
    # client to server, responses, feedback and results back.
    for asrv in node_info.action_servers:
        name = asrv.action_name
        for channel in _CLIENT_SENDS:
            add(name, "action", _ACTION_PORT_NAME[channel](name), "input", ACTION_PORT_TYPE)
        for channel in _SERVER_SENDS:
            add(name, "action", _ACTION_PORT_NAME[channel](name), "output", ACTION_PORT_TYPE)
    for acli in node_info.action_clients:
        name = acli.action_name
        for channel in _CLIENT_SENDS:
            add(name, "action", _ACTION_PORT_NAME[channel](name), "output", ACTION_PORT_TYPE)
        for channel in _SERVER_SENDS:
            add(name, "action", _ACTION_PORT_NAME[channel](name), "input", ACTION_PORT_TYPE)

    return ports


def _build_timers(timer_infos: list[TimerInfo]) -> list[LFTimer]:
    """Build LFTimers from TimerInfo list."""
    return [_make_timer(t) for t in timer_infos]


def _ms_to_lf_time(ms: float) -> str:
    """A millisecond value as an LF time literal lfc accepts.

    LF time values are integers, so a fractional millisecond (10.5) is written
    in nanoseconds from the exact decimal (period_ms_to_ns), the same value the
    RID table carries, which the binding matches exactly.
    """
    ns = period_ms_to_ns(ms)
    if ns is None:
        raise ValueError(
            f"time {ms!r} ms is not a nanosecond count from 1 ns to the int64 range")
    if ms == int(ms):
        return f"{int(ms)} msec"
    return f"{ns} nsec"


def _make_timer(t: TimerInfo) -> LFTimer:
    """Convert a single TimerInfo to an LFTimer."""
    if t.period_ms is None:
        # No placeholder period. A made-up period would size the
        # correspondence bounds and the LF timer from a value nobody measured,
        # and the binding, which matches a timer handle by its exact period,
        # would refuse it at bind. The resolved-model gate refuses RCLLF-R001
        # before this point; this guards every other caller.
        raise ValueError(
            f"timer callback '{t.callback}' has no period (period_ms is null in "
            "system.json, RCLLF-R001): set it in execution.yaml "
            "(nodes.<node>.timers.<callback>.period) or run the system.json "
            "patch step before lf-generate"
        )
    period_str = _ms_to_lf_time(t.period_ms)
    # Analyzed offset (always 0 today) or the execution.yaml offset.
    offset_ms = getattr(t, "offset_ms", 0.0) or 0.0
    if offset_ms == 0.0:
        offset_str = "0"
    else:
        offset_str = _ms_to_lf_time(offset_ms)
    return LFTimer(
        name=callback_to_timer_name(t.callback),
        offset=offset_str,
        period=period_str,
    )


def _build_physical_actions(node_info: NodeInfo) -> list[LFPhysicalAction]:
    """Build physical actions for blocking service clients."""
    actions: list[LFPhysicalAction] = []
    for sc in node_info.service_clients:
        if sc.call_style in ("blocking_spin", "blocking_future", "blocking_wait"):
            action_name = f"pa_{sanitize_lf_identifier(sc.service_name.lstrip('/'))}_resp"
            actions.append(LFPhysicalAction(name=action_name, min_delay_ns=0))
    return actions


def _build_pub_port_map(
    publishers: list[PublisherInfo], corr: _NodeCorr = _NO_CORR,
) -> dict[str, str]:
    """Map publisher topic names to output port names (stub ports only)."""
    raw_names = [topic_to_port_name(p.topic) for p in publishers]
    deduped = deduplicate_names(raw_names)
    return {
        p.topic: deduped[i] for i, p in enumerate(publishers)
        if p.topic not in corr.dropped_pub_topics
    }


def _sub_port_name(node_info: NodeInfo | None, topic: str) -> str:
    """Input port name of *node_info*'s subscription on node-local *topic*.

    A node that also publishes *topic* would declare its input
    under its output port's name, which lfc rejects as a duplicate variable,
    so that input gets an ``_in`` suffix.
    """
    name = topic_to_port_name(topic)
    if node_info is not None and any(
        topic_to_port_name(p.topic) == name for p in node_info.publishers
    ):
        return f"{name}_in"
    return name


def _corr_sub_triggers(corr: _NodeCorr) -> dict[str, str]:
    """Admitted subscription callbacks are triggered by their ``corr_e`` port
    (a self edge by its drain action), external ones
    by their physical action."""
    # Not emit_effect: for a remote edge that names the egress flush action,
    # and an ingress subscriber is triggered by its port.
    out = {cb: corr.sub_trigger(e) for cb, e in corr.sub_edges.items()}
    out.update(corr.external_subs)
    return out


def external_action_name(callback: str) -> str:
    """Physical action that triggers an external subscription."""
    return f"ext_{sanitize_lf_identifier(callback)}"


def external_action_server_name(short: str) -> str:
    """Physical action that triggers an external action server."""
    return f"ext_action_{sanitize_lf_identifier(short)}"


def _rid_macro(binding_key: str, callback: str) -> str:
    return f"RID_{sanitize_lf_identifier(binding_key)}_{sanitize_lf_identifier(callback)}"


def _external_inputs(
    cb_names: list[str], corr: _NodeCorr, binding_key: str,
) -> list[tuple[str, str]]:
    """``(action, RID macro)`` of the external subscriptions among *cb_names*."""
    return [
        (corr.external_subs[cb], _rid_macro(binding_key, cb))
        for cb in cb_names if cb in corr.external_subs
    ]


def _add_external_subs(
    graph: SystemGraph,
    external_topics: set[str] | None,
    runtime_to_key: dict[str, str],
    node_corr: dict[str, _NodeCorr],
    scopes: dict[str, NodeIdentity] | None = None,
) -> None:
    """Mark every subscription on a declared external topic.

    Its callback gets a physical action trigger and its node loses the
    unconnected input port of that topic.
    """
    for tc in graph.communication.topics:
        if not external_topics or tc.topic not in external_topics:
            continue
        for sub_node in tc.subscribers:
            key = runtime_to_key.get(sub_node, sub_node)
            node = graph.nodes.get(key)
            if node is None:
                continue
            nc = node_corr.get(key)
            if nc is None:
                nc = node_corr[key] = _NodeCorr()
            for info in _match_endpoint_infos(
                tc.topic, node.subscribers, (scopes or {}).get(key),
            ):
                nc.external_subs[info.callback] = external_action_name(info.callback)
                nc.dropped_sub_topics.add(info.topic)


def _add_external_actions(
    graph: SystemGraph,
    runtime_to_key: dict[str, str],
    node_corr: dict[str, _NodeCorr],
) -> None:
    """Mark every action server whose action has no client in this graph.

    Its requests come from outside the LF system (``ros2 action send_goal``,
    say), so a physical action the binding schedules at arrival triggers its
    goal, result and cancel reactions. A federate slice keeps such an action
    only when it has no client anywhere, so a cross-federate server is not
    mistaken for an external one.
    """
    for ac in graph.communication.actions:
        if not ac.server or ac.clients:
            continue
        key = runtime_to_key.get(ac.server, ac.server)
        if key not in graph.nodes:
            continue
        nc = node_corr.get(key)
        if nc is None:
            nc = node_corr[key] = _NodeCorr()
        for asrv in graph.nodes[key].action_servers:
            if _names_action(ac.action, asrv.action_name):
                nc.external_actions.add(asrv.action_name)


def _corr_logical_actions(reactions: list[LFReaction]) -> list[str]:
    """The drain and flush actions of the admitted edges handled here."""
    return [
        corr_drain_action_name(r.corr_drain_edge)
        for r in reactions if r.corr_drain_edge is not None
    ] + [r.triggers[0] for r in reactions if r.kind == XCORR_FLUSH_KIND] + (
        [BNA_WAKE_ACTION] if any(r.kind == BNA_WAKE_KIND for r in reactions) else []
    ) + ([TF2_WAKE_ACTION] if any(r.kind == TF2_WAKE_KIND for r in reactions) else [])


def _corr_emit_line(
    edge_id: int, remote: bool = False, local: bool = False, zero: bool = False,
) -> str:
    """Publisher side: forward this invoke's records of *edge_id* (D4).

    A cross-federate edge sends the records themselves (D1). By
    default (*zero*) the reaction sends the records of its
    own tag at that tag over `after d`: d = 0 lands on the publication tag
    (t, m) over a plain zero-delay connection (centralized; a
    decentralized `after 0` lands on (t, m + 1)), d > 0 on
    (t + d, k) for the k-th message of time t. A flush edge
    (``xfed_flush``) sends all records of one time in one flush at t + 1 ns,
    scheduled once per time. A self edge
     arrives them here, at the emit's tag, as its zero-delay link
    would, and schedules the drain wake at t+d itself.
    """
    if remote and zero:
        port = xcorr_port_name(edge_id)
        return (
            f"{{ rcllf_xcorr_t x; if (ros2_lf_corr_send_remote(self, {edge_id}, &x) > 0) "
            f"lf_set({port}, x); }}"
        )
    if local:
        return (
            f"{{ uint32_t n = ros2_lf_corr_emit(self, {edge_id}); "
            f"if (n > 0) {{ int64_t w = ros2_lf_corr_arrive(self, {edge_id}, n); "
            f"{_corr_resched_line(edge_id)} }} }}"
        )
    if remote:
        action = xcorr_flush_action_name(edge_id)
        return (
            f"if (ros2_lf_corr_flush_due({edge_id}) && lf_schedule({action}, NSEC(1)) <= 0) "
            f"{{ ros2_lf_corr_flush_refused({edge_id}); }}"
        )
    port = corr_port_name(edge_id)
    return (
        f"{{ uint32_t n = ros2_lf_corr_emit(self, {edge_id}); "
        f"if (n > 0) {{ rcllf_corr_t c = {{{edge_id}, n}}; lf_set({port}, c); }} }}"
    )


def _xcorr_flush_reaction(edge_id: int) -> LFReaction:
    """Send egress edge *edge_id*'s records of time t at (t + 1 ns, 0).

    `after d` maps every microstep of t to (t + d, 0), so the records of all
    the reactions at t go in one message. The link is `after d - 1 ns`.
    """
    return LFReaction(
        triggers=[xcorr_flush_action_name(edge_id)],
        effects=[xcorr_port_name(edge_id)],
        body=(
            f"{{ rcllf_xcorr_t x; if (ros2_lf_corr_flush_remote(self, {edge_id}, &x) > 0) "
            f"lf_set({xcorr_port_name(edge_id)}, x); }}"
        ),
        kind=XCORR_FLUSH_KIND,
    )


def _corr_resched_lines(edge_id: int) -> list[str]:
    """Schedule the drain wake ``w`` of *edge_id*, claimed first.

    The record wake and the logical deadline window's wake schedule one drain
    action; reactor-c refuses a second event at a tag that already holds one,
    and a refusal terminalizes the carrier. ros2_lf_corr_claim_wake returns -1
    for a wake whose tag is already queued.
    """
    action = corr_drain_action_name(edge_id)
    return [
        f"w = ros2_lf_corr_claim_wake({edge_id}, w);",
        f"if (w >= 0 && lf_schedule({action}, w) <= 0) "
        f"{{ ros2_lf_corr_schedule_refused({edge_id}); }}",
    ]


def _corr_resched_line(edge_id: int) -> str:
    """:func:`_corr_resched_lines` as one line, for an inline body."""
    return " ".join(_corr_resched_lines(edge_id))


def _build_corr_sub_reactions(
    cb: CallbackInfo,
    edge_id: int,
    effects: list[str],
    stub_effects: list[str],
    emit_edges: list[int],
    topic: str,
    binding_key: str,
    corr: _NodeCorr = _NO_CORR,
    published: dict[str, str] | None = None,
) -> list[LFReaction]:
    """Arrive, drain and close reactions replacing a legacy invoke_sub (D4).

    Only the drain dispatches user code: it keeps the subscription's RID
    (kind ``sub``, ``rid_name`` = the callback) and its outgoing effects. An
    ingress edge arrives with the publisher's records. A stub
    effect in *published* (port to topic) is raised by the publishes the
    binding saw during the drain, as a timer's or sub's is (P17e), so a drain
    that publishes nothing owes the legacy reader no sample.

    *edge_id* is the lead of the subscription's fan-in group. Each
    member gets its own arrive and close reaction, and all of them schedule
    the lead's drain action. A group of one is the reactions above.

    The arrive reactions are declared after every other reaction
    of their reactor (``_build_reactions``), the drain included. An arrive at
    tag g files records due at (t, m + 1) or (t + d, k), never at g, so a
    drain at g pops the same records whether it runs before or after the
    arrives of g. The group's one pending wake (``wake_pending``: an arrive
    returns -1 while a wake is pending, and the drain re-arms the wake for
    the earliest head when it ends) keeps it at one ``lf_schedule`` of the
    drain action per tag, in either order. Members that come at one tag
    file release tags that rise with arrival (a group shares d and T), so
    the pending wake is never later than a record it must drain.

    A cross-federate member has no
    arrive reaction of its own. Its records come at their release tag ((t + d,
    0), or for d = 0 the publication tag (t, m) itself, (t, m + 1)
    when decentralized), so the drain takes its port as a trigger,
    files the records and drains at that tag, as a one-process drain woken by
    its arrive's lf_schedule(drain, d) does. A separate arrive could only wake
    the drain one microstep later, and reactor-c refuses that wake when a
    timeout stops the program at the arrival tag. Members that come at one
    tag are all filed before the one drain pops rank 0, and the re-arm takes
    the rest one microstep each, as in one process. ``RCLLF_XFED_SPLIT_DRAIN=1``
    (``xfed_split_drain``) restores the separate arrive for one release.

    An in-process d = 0 member (``corr.same_tag``) takes the same
    merged shape. Its port comes at the publication tag (t, m), so the first
    record runs there, not one microstep later; a fan-in group's drain is
    triggered by every member port, so LF runs it after all member publishers
    of the tag and every record is filed before the first pop.
    ``RCLLF_LOCAL_SPLIT_DRAIN=1`` (``local_split_drain``) restores the arrive.
    """
    members = corr.sub_members.get(cb.name) or [edge_id]
    action = corr_drain_action_name(edge_id)
    rid_macro = f"RID_{sanitize_lf_identifier(binding_key)}_{sanitize_lf_identifier(cb.name)}"
    arrives = []
    merged_ports = []
    split = xfed_split_drain()
    for m in members:
        port = corr.port(m)
        if m in corr.local:
            continue
        if (m in corr.remote and not split) or m in corr.same_tag:
            merged_ports.append((m, port))
            continue
        arrive_line = (
            f"int64_t w = ros2_lf_corr_arrive_remote(self, {m}, &{port}->value);\n"
            if m in corr.remote else
            f"int64_t w = ros2_lf_corr_arrive(self, {m}, {port}->value.count);\n"
        )
        arrives.append(LFReaction(
            triggers=[port],
            effects=[action],
            body=arrive_line + "\n".join(_corr_resched_lines(edge_id)),
            kind="corr_arrive",
        ))
    drain_lines = [f'int64_t w = ros2_lf_corr_drain(self, {edge_id}, {rid_macro}, "{topic}");']
    drain_lines += [
        _corr_emit_line(e, e in corr.remote, e in corr.local, e in corr.zero)
        for e in emit_edges
    ]
    counted = _count_publishes()
    published = published or {}
    drain_lines += [
        _sent_add(p, rid_macro, published[p]) if counted and p in published else _stub_add(p)
        for p in stub_effects
    ]
    if merged_ports:
        # An arrive returns 0 when its records are due now and it set the
        # group's wake, a wait when they release later, -1 when a wake is
        # already pending; that pending wake drains them at its own tag. It is
        # never later than a record due now: a group's members share d and T
        # (fan_in_release_mismatch), so their release tags rise with arrival.
        drain_lines = (
            ["int64_t w = -1;"]
            + [
                f"if ({port}->is_present) {{ int64_t a = "
                + (f"ros2_lf_corr_arrive(self, {m}, {port}->value.count); "
                   if m in corr.same_tag else
                   f"ros2_lf_corr_arrive_remote(self, {m}, &{port}->value); ")
                + "if (a >= 0) { w = a; } }"
                for m, port in merged_ports
            ]
            + [f"if (w == 0 || {action}->is_present) {{"]
            + [f"    {line}" for line in [drain_lines[0].replace("int64_t w", "w", 1)]
               + drain_lines[1:]]
            + ["}"]
        )
    drain_lines += _corr_resched_lines(edge_id)
    drain_effects = [action] + [e for e in effects if e != action]
    drain = LFReaction(
        triggers=[action] + [port for _, port in merged_ports],
        effects=drain_effects,
        body="\n".join(drain_lines),
        kind="sub",
        rid_name=cb.name,
        corr_emit_edges=list(emit_edges),
        corr_drain_edge=edge_id,
        corr_drain_members=list(members),
    )
    closes = [
        LFReaction(
            triggers=["shutdown"],
            effects=[],
            body=f"ros2_lf_corr_close({m});",
            kind="corr_close",
        )
        for m in members
    ]
    # One wake at (start, 1), after every startup reaction (node
    # binding included), arms the subscription's logical deadline window in
    # the binding (silence_deadline.hpp), so a producer that never publishes
    # is reported at t0 + D. That drain finds no record; it returns the
    # window's end, or -1 for a subscription without a QoS deadline, which is
    # then never woken by it again.
    arm = LFReaction(
        triggers=["startup"],
        effects=[action],
        body="\n".join(["int64_t w = 0;"] + _corr_resched_lines(edge_id)),
        kind=CORR_SILENCE_ARM_KIND,
    )
    return [drain] + closes + [arm] + arrives


def _corr_close_reaction(edge_id: int) -> LFReaction:
    """Shutdown close of an egress edge, in its publisher's reactor."""
    return LFReaction(
        triggers=["shutdown"], effects=[], body=f"ros2_lf_corr_close({edge_id});",
        kind="corr_close",
    )


def _build_external_sub_reaction(
    cb: CallbackInfo,
    action: str,
    effects: list[str],
    stub_effects: list[str],
    emit_edges: list[int],
    binding_key: str,
    topic: str,
    corr: _NodeCorr = _NO_CORR,
) -> LFReaction:
    """The reaction of a subscription on an external topic.

    Triggered by the physical action the ingress thread schedules at arrival;
    the payload is the newest record id, and the binding runs the callback
    once per record up to it. The trailing comment carries the topic for the
    RID table (``binding_header_emitter._infer_topic``), which the binding
    needs to match the subscription handle at bind time.
    """
    # The binding returns how many callbacks ran, each a write.
    ran = "int rcllf_ran = " if stub_effects else ""
    lines = [
        f"{ran}ros2_lf_invoke_external_sub(self, {_rid_macro(binding_key, cb.name)}, "
        f'{action}->value);  /* external topic "{topic}" */'
    ]
    lines += [
        _corr_emit_line(e, e in corr.remote, e in corr.local, e in corr.zero)
        for e in emit_edges
    ]
    if stub_effects:
        lines.append(
            "if (rcllf_ran > 0) { "
            + " ".join(_stub_add(p, "rcllf_ran") for p in stub_effects) + " }"
        )
    return LFReaction(
        triggers=[action],
        effects=effects,
        body="\n".join(lines),
        kind="sub",
        corr_emit_edges=list(emit_edges),
    )


_INVOKE_RID = re.compile(r"ros2_lf_invoke_\w+\(self,\s*(RID_\w+)")


def output_deadline_arm_line(rid_macro: str, topic: str, bound_ns: int) -> str:
    """The arm call of one publisher's actuation lateness check."""
    esc = topic.replace("\\", "\\\\").replace('"', '\\"')
    return (f'ros2_lf_output_deadline_arm({rid_macro}, "{esc}", {int(bound_ns)}LL);'
            f"  /* publishers: output_deadline_ns */")


def _arm_output_deadlines(
    node_key: str,
    node_info: NodeInfo,
    binding_key: str,
    reactors: list[LFReactor],
    deadlines: dict[str, int],
) -> None:
    """Arm the actuation lateness check in every reaction that publishes a bounded topic.

    The publish itself is the
    user's ``publish()`` inside the callback that the reaction's
    ``ros2_lf_invoke_*`` call runs, so the arm line goes right before that
    call: it tells the binding the bound of each topic this reaction may
    publish, and the binding compares ``lf_time_physical() - lf_time_logical()``
    with it in the publication gate, immediately before the real
    ``rcl_publish``, and publishes anyway. No LF ``deadline`` is emitted: a
    missed LF deadline runs its handler instead of the body and would drop
    the message. A bounded topic that no reaction of the node publishes is
    refused, since its check would silently never run.
    """
    if not deadlines:
        return
    cbs = {sanitize_lf_identifier(cb.name): cb for cb in node_info.callbacks}
    prefix = f"RID_{sanitize_lf_identifier(binding_key)}_"
    armed: set[str] = set()
    for reactor in reactors:
        for reaction in reactor.reactions:
            out: list[str] = []
            for line in reaction.body.split("\n"):
                m = _INVOKE_RID.search(line)
                if m:
                    macro = m.group(1)
                    cb = cbs.get(macro[len(prefix):]) if macro.startswith(prefix) else None
                    topics = [t for t in (cb.publishers_used if cb else []) if t in deadlines]
                    out += [output_deadline_arm_line(macro, t, deadlines[t]) for t in topics]
                    armed.update(topics)
                out.append(line)
            reaction.body = "\n".join(out)
    missing = sorted(set(deadlines) - armed)
    if missing:
        raise ValueError(
            f"execution.yaml publishers: node {node_key!r} sets output_deadline_ns on "
            f"{missing}, but no reaction of the node publishes that topic in the analysis "
            f"(no callback lists it in publishers_used), so the lateness check would never "
            f"run. Remove the entry, or re-run colcon lf-analyze if the code changed."
        )


def _timers_first(reaction_order: dict[str, str] | None, node_key: str) -> bool:
    """Whether *node_key*'s ``reaction_order`` is ``timers_first``.

    A node the map does not name takes the registry default."""
    from colcon_lf.exec_config.policies import (
        KEY_REACTION_ORDER, REACTION_ORDER_TIMERS_FIRST, REGISTRY,
    )

    order = (reaction_order or {}).get(node_key, REGISTRY[KEY_REACTION_ORDER].default)
    return order == REACTION_ORDER_TIMERS_FIRST


def _build_reactions(
    callbacks: list[CallbackInfo],
    timer_trigger_map: dict[str, str],
    sub_trigger_map: dict[str, str],
    sub_topic_map: dict[str, str],
    pub_port_map: dict[str, str],
    binding_key: str,
    *,
    srv_info: "_SrvMaps | None" = None,
    corr: _NodeCorr = _NO_CORR,
    node_info: NodeInfo | None = None,
    timers_first: bool = False,
) -> list[LFReaction]:
    """Build LFReactions from CallbackInfos and trigger/effect maps.

    *binding_key* names the ``RID_*`` macros (see ``binding_key.py``); it equals
    the C++ class name only when that class has a single instance.
    *timers_first* declares the ``timer`` reactions before all
    others instead of after them.
    """
    reactions: list[LFReaction] = []

    for cb in callbacks:
        triggers: list[str] = []
        effects: list[str] = []
        kind: str = "unknown"

        # Determine trigger and kind
        if cb.name in timer_trigger_map:
            triggers.append(timer_trigger_map[cb.name])
            kind = "timer"
        if cb.name in sub_trigger_map:
            triggers.append(sub_trigger_map[cb.name])
            kind = "sub"

        if not triggers:
            continue

        # Determine effects (output ports written)
        for pub_topic in cb.publishers_used:
            port_name = pub_port_map.get(pub_topic)
            if port_name:
                effects.append(port_name)

        # A callback that calls an async service client writes its request
        # port. A timer callback gets the srv_send kind (bound as a timer); a
        # subscription callback keeps kind sub, so its RID is bound by topic
        # and its request write sits in the per-write block (review round 5,
        # O1-1: a send from a subscription callback failed at launch with "no
        # unbound timer handle").
        if srv_info and cb.service_clients_used and kind in ("timer", "sub"):
            for svc_name in cb.service_clients_used:
                if svc_name in srv_info.async_client_services:
                    req_port = service_to_request_port_name(svc_name)
                    effects.append(req_port)
            if kind == "timer" and any(
                s in srv_info.async_client_services for s in cb.service_clients_used
            ):
                kind = "srv_send"

        # The action channels it may send on are counted claims,
        # never stub writes. A callback that uses an action client keeps the
        # action_goal_send kind its RID is bound by.
        claims: list[tuple[str, str, str]] = []
        if kind in ("timer", "sub") and node_info is not None:
            claims = _callback_action_claims(cb, node_info)
        if cb.action_clients_used and kind == "timer":
            kind = "action_goal_send"

        # Admitted edges this callback publishes on (timer or sub
        # kind only, enforced by admission) get a corr port and an emit line.
        emit_edges = list(corr.pub_edges.get(cb.name, []))
        stub_effects = list(effects)
        effects += [p for p, _, _ in claims if p not in effects]
        effects += [corr.emit_effect(e) for e in emit_edges]
        claim_lines = [_action_claim_line(*c) for c in claims]

        if cb.name in corr.external_subs:
            # A callback that also sends a request or goal keeps the
            # subscription kind. The binding runs it on the arrival, and its
            # request or goal port (in the effects) triggers the server
            # reaction, as a timer sender's does.
            reactions.append(_build_external_sub_reaction(
                cb, corr.external_subs[cb.name], effects, stub_effects,
                emit_edges, binding_key, sub_topic_map.get(cb.name, ""), corr,
            ))
            reactions[-1].body += "".join("\n" + line for line in claim_lines)
            continue

        if cb.name in corr.sub_edges:
            corr_reactions = _build_corr_sub_reactions(
                cb, corr.sub_edges[cb.name], effects, stub_effects, emit_edges,
                sub_topic_map.get(cb.name, ""), binding_key, corr,
                published={
                    pub_port_map[t]: t for t in cb.publishers_used if t in pub_port_map
                },
            )
            # A claim in a reaction that ran no callback finds nothing sent.
            for r in corr_reactions:
                if claims and any(p in r.effects for p, _, _ in claims):
                    r.body += "".join("\n" + line for line in claim_lines)
            reactions += corr_reactions
            continue

        # Build reaction body
        body = _build_lfc_body(
            cb, kind, stub_effects, sub_topic_map, binding_key, srv_info=srv_info,
            emit_edges=emit_edges, remote_edges=corr.remote, local_edges=corr.local,
            zero_edges=corr.zero,
            sub_port=sub_trigger_map.get(cb.name),
            published={
                pub_port_map[t]: t for t in cb.publishers_used if t in pub_port_map
            },
        )
        body += "".join("\n" + line for line in claim_lines)
        reactions.append(LFReaction(
            triggers=triggers,
            effects=effects,
            body=body,
            kind=kind,
            corr_emit_edges=emit_edges,
        ))
        reactions += [_corr_close_reaction(e) for e in corr.egress_close.get(cb.name, [])]

    # LF runs the reactions of one reactor at one tag in declaration
    # order when no connection orders them. By default (timers_first) timer
    # reactions come before subscription reactions, so a timer acts on the
    # inputs of earlier tags only. subscriptions_first puts the subscription
    # reactions first, so a timer sees the subscription state of its own tag.
    # A stable sort, so source order holds within each group.
    timer_rank = 0 if timers_first else 1
    reactions.sort(key=lambda r: timer_rank if r.kind == "timer" else 1 - timer_rank)
    # Every corr_arrive reaction is declared after every other
    # reaction of the reactor. LF makes a reaction depend on the ones declared
    # before it in its reactor, so an arrive declared first would put the
    # reactor's publishing reactions after the port that triggers it, and a
    # loop of admitted zero-delay links would read as a causality cycle to
    # lfc. Declared last, an arrive depends on everything and nothing depends
    # on it: its only effect is the drain action, whose wake lands at a later
    # tag ((t, m + 1) for d = 0). The drain, not the arrive, runs user code,
    # so the output of a tag does not change with the arrive's position, and
    # a feedback loop of admitted edges is no causality loop (LF semantics:
    # every delivery is at least one microstep later than its publication).
    arrives = [r for r in reactions if r.kind == "corr_arrive"]
    reactions = [r for r in reactions if r.kind != "corr_arrive"] + arrives
    # Each egress flush comes first, so at t + 1 ns it sends the
    # records of t before a reaction of t + 1 ns commits new ones. A d = 0
    # edge has no flush.
    flushed = dict.fromkeys(
        e for r in reactions for e in r.corr_emit_edges
        if e in corr.remote and e not in corr.zero
    )
    return _add_tf2_reactions(_add_bna_reactions(
        [_xcorr_flush_reaction(e) for e in flushed] + reactions, corr, binding_key,
    ), node_info, binding_key)


def _check_bna_stamp_provenance(
    graph: SystemGraph, bounds: CorrBoundsConfig | None,
) -> None:
    """Refuse a BNA input topic without a declared stamp provenance.

    The engine pairs by header stamp, so its determinism holds only when the
    stamps themselves are deterministic. rcllf cannot check that, so the user
    declares it per input in execution.yaml.
    """
    declared = bounds.stamp_provenance if bounds is not None else {}
    for node_key, node_info in graph.nodes.items():
        mf = node_info.message_filters
        for sync in mf.synchronizers if mf else []:
            if sync.bna_params is None:
                continue
            for member in sync.inputs:
                cb = bna_input_callback(sync.callback, member)
                for sub in node_info.subscribers:
                    if sub.callback == cb and (node_key, sub.topic) not in declared:
                        raise ValueError(
                            f"Node {node_key!r}: BoundedNearestToAnchor input topic "
                            f"{sub.topic!r} has no stamp_provenance in execution.yaml "
                            "(subscriptions scope, 'lf_clock' or 'declared:<text>')"
                        )


def _mark_bna_stamp_lag(
    graph: SystemGraph, bounds: CorrBoundsConfig | None, node_corr: dict[str, _NodeCorr],
) -> None:
    """Note the BNA input callbacks with a stamp bound for a lost message.

    An input declared ``stamp_provenance: lf_clock`` and ``max_stamp_lag:
    <time>`` has every header stamp at least its publish tag minus the lag, so
    a lost message may raise the engine's watermark to that bound. ``lf_clock``
    alone is no bound (a stamp taken in an earlier reaction or forwarded by a
    relay lies below the publish tag), and a lag on a ``declared:`` input is
    refused because the publish tag says nothing about such stamps.
    """
    provenance = bounds.stamp_provenance if bounds is not None else {}
    lags = bounds.max_stamp_lag_ns if bounds is not None else {}
    matched: set[tuple[str, str]] = set()
    for node_key, node_info in graph.nodes.items():
        mf = node_info.message_filters
        for sync in mf.synchronizers if mf else []:
            if sync.bna_params is None:
                continue
            for member in sync.inputs:
                cb = bna_input_callback(sync.callback, member)
                for sub in node_info.subscribers:
                    key = (node_key, sub.topic)
                    if sub.callback != cb or key not in lags:
                        continue
                    if provenance.get(key) != STAMP_PROVENANCE_LF_CLOCK:
                        raise ValueError(
                            f"Node {node_key!r}: max_stamp_lag on BoundedNearestToAnchor "
                            f"input topic {sub.topic!r} needs stamp_provenance 'lf_clock'"
                        )
                    matched.add(key)
                    if node_key in node_corr:
                        node_corr[node_key].bna_stamp_lag[cb] = lags[key]
    # A lag on any other subscription of a node in this graph would change
    # nothing; refuse it like a misspelled key.
    for node_key, topic in sorted(set(lags) - matched):
        if node_key in graph.nodes:
            raise ValueError(
                f"Node {node_key!r}: max_stamp_lag on topic {topic!r}, which is no "
                "BoundedNearestToAnchor input; the key only applies to BNA inputs"
            )


def bna_input_callback(sync_callback: str, member: str) -> str:
    """The synthetic callback of one sync input (``message_filters_scan``)."""
    return f"{sync_callback}_via_{member.strip('_')}"


def _with_bna_inputs(node_info: NodeInfo, corr: _NodeCorr) -> _NodeCorr:
    """*corr* with the node's BNA sync inputs, refusing unadmitted ones.

    Every input must be an admitted correspondence edge: the engine needs the drain's tag
    and key, and the missing report of its join.
    """
    mf = node_info.message_filters
    syncs = [s for s in (mf.synchronizers if mf else []) if s.bna_params is not None]
    if not syncs:
        return corr
    inputs: dict[str, int] = {}
    for i, member in enumerate(syncs[0].inputs):
        cb = bna_input_callback(syncs[0].callback, member)
        if cb not in corr.sub_edges:
            reason = ("external" if cb in corr.external_subs
                      else corr.legacy_reasons.get(cb, "no_edge"))
            raise ValueError(
                f"Node {node_info.class_name!r}: RCLLF-H003 bna_input_legacy:{reason}: "
                f"BoundedNearestToAnchor input {member} is not an admitted "
                "correspondence edge, so its admissions have no logical key"
            )
        inputs[cb] = i
    return dataclasses.replace(corr, bna_inputs=inputs)


def _check_sync_inputs(node_info: NodeInfo, corr: _NodeCorr) -> None:
    """Refuse an ExactTime or ApproximateTime input outside mf-sync/1.

    The policy pairs in arrival order, so the converted run is deterministic
    only when every input arrives in logical order: each input an admitted
    correspondence edge (a legacy edge arrives in wall-clock order) and one common
    link delay, so a pair published at one tag lands at one tag. The BNA rule
    (``bna_input_legacy``) applied to the two stock policies.
    """
    mf = node_info.message_filters
    for sync in mf.synchronizers if mf else []:
        if sync.bna_params is not None:
            continue
        delays: dict[str, set[int]] = {}
        for member in sync.inputs:
            cb = bna_input_callback(sync.callback, member)
            topic = next((s.topic for s in node_info.subscribers if s.callback == cb), "")
            # A legacy edge into the input refuses even next to an admitted one
            # into the same subscription: its samples arrive in wall-clock order.
            # The per-topic reason (an edge refused before its subscription
            # was matched) only names an input with no admitted edge: a fan-in
            # group is admitted or refused as a unit, so next to an
            # admitted input it belongs to a twin subscription on the topic.
            legacy = corr.legacy_reasons.get(cb)
            if cb not in corr.sub_edges or legacy is not None:
                reason = ("external" if cb in corr.external_subs
                          else legacy or corr.legacy_topic_reasons.get(topic, "no_edge"))
                raise ValueError(
                    f"Node {node_info.class_name!r}: RCLLF-H003 sync_input_legacy:{reason}: "
                    f"{sync.policy} input {member} is not an admitted correspondence "
                    "edge, so its arrivals have no logical order (domain mf-sync/1)"
                )
            delays[member] = corr.sub_delays.get(cb) or {0}
        if len(set().union(*delays.values())) > 1:
            listed = ", ".join(f"{m} {'/'.join(str(d) for d in sorted(ds))} ns"
                               for m, ds in delays.items())
            raise ValueError(
                f"Node {node_info.class_name!r}: RCLLF-H003 sync_input_delay_unequal: "
                f"{sync.policy} inputs have different link delays ({listed}); give "
                "both input links one after delay (domain mf-sync/1)"
            )


def _order_sync_drains(node_info: NodeInfo) -> NodeInfo:
    """Declare an ExactTime/ApproximateTime synchronizer's input drains in input
    order, input 0 first (the mf-sync/1 arrival order).

    Reactions follow ``node_info.callbacks`` order, which is the source order of
    the subscriptions, not the synchronizer's input order. The input callbacks
    keep the positions they hold and are reassigned to them input 0 first, so
    at one tag input 0's samples reach the policy before input 1's.

    Test hook ``RCLLF_TEST_SYNC_REVERSE_DRAINS=1`` (plan S5) assigns them input
    1 first instead: the converted-side falsification of the oracle's
    ``arrival_order_mismatch`` check. Never set in production.
    """
    mf = node_info.message_filters
    if not mf:
        return node_info
    reverse = os.environ.get("RCLLF_TEST_SYNC_REVERSE_DRAINS") == "1"
    callbacks = list(node_info.callbacks)
    names = [c.name for c in callbacks]
    for sync in mf.synchronizers:
        cbs = [bna_input_callback(sync.callback, m) for m in sync.inputs]
        if sync.bna_params is not None or len(cbs) != 2 or not set(cbs) <= set(names):
            continue
        slots = sorted(names.index(cb) for cb in cbs)
        by_name = {c.name: c for c in callbacks}
        for pos, cb in zip(slots, reversed(cbs) if reverse else cbs):
            callbacks[pos], names[pos] = by_name[cb], cb
    return dataclasses.replace(node_info, callbacks=callbacks)


def _add_bna_reactions(
    reactions: list[LFReaction], corr: _NodeCorr, binding_key: str,
) -> list[LFReaction]:
    """The BNA declare, wake and close reactions of the reactor that holds the
    sync input drains, and the wake loop in each drain.

    The declaration runs at startup before any drain, the wake reaction comes
    after the drains (an admission at a deadline tag is seen before the wake at
    it) and the close after the corr closes.
    """
    drains = {
        corr.bna_inputs[r.rid_name]: r for r in reactions
        if r.corr_drain_edge is not None and r.rid_name in corr.bna_inputs
    }
    if not drains:
        return reactions
    if len(drains) != len(corr.bna_inputs):
        raise ValueError(
            f"Node {binding_key!r}: RCLLF-H003 bna_split: the BoundedNearestToAnchor "
            "inputs land in different reactors (a Reentrant callback group), so "
            "their drains are not mutually exclusive"
        )
    if reactions.index(drains[0]) > reactions.index(drains[1]):
        raise ValueError(
            f"Node {binding_key!r}: BNA anchor drain is ordered after the candidate drain"
        )
    loop = (
        f'{{ int64_t bw; while ((bw = ros2_lf_bna_take_wake("{binding_key}")) >= 0) '
        f"{{ lf_schedule({BNA_WAKE_ACTION}, bw); }} }}"
    )
    for drain in drains.values():
        lines = drain.body.split("\n")
        drain.body = "\n".join(lines[:-1] + [loop, lines[-1]])
        drain.effects.append(BNA_WAKE_ACTION)
    declare = LFReaction(
        triggers=["startup"], effects=[],
        body="\n".join(
            f'ros2_lf_bna_declare_input("{binding_key}", '
            f"{_rid_macro(binding_key, drains[i].rid_name)}, {i});"
            for i in sorted(drains)
        ) + "".join(
            f"\nros2_lf_bna_input_stamp_lag({_rid_macro(binding_key, drains[i].rid_name)}, "
            f"{corr.bna_stamp_lag[drains[i].rid_name]}LL);"
            for i in sorted(drains) if drains[i].rid_name in corr.bna_stamp_lag
        ),
        kind=BNA_DECLARE_KIND,
    )
    wake = LFReaction(
        triggers=[BNA_WAKE_ACTION], effects=[BNA_WAKE_ACTION],
        body=f'ros2_lf_bna_wake("{binding_key}");\n{loop}',
        kind=BNA_WAKE_KIND,
    )
    close = LFReaction(
        triggers=["shutdown"], effects=[],
        body=f'ros2_lf_bna_close("{binding_key}");',
        kind=BNA_CLOSE_KIND,
    )
    last_drain = max(reactions.index(d) for d in drains.values())
    return ([declare] + reactions[:last_drain + 1] + [wake]
            + reactions[last_drain + 1:] + [close])


def _add_tf2_reactions(
    reactions: list[LFReaction], node_info: NodeInfo | None, binding_key: str,
) -> list[LFReaction]:
    """The tf2 wake and close reactions of a node with an
    rcllf_tf2::AcquisitionLookup, the ingest notice after its
    /tf and /tf_static reactions and the wake loop after every user reaction.

    A request made in any user reaction asks for a wake at its logical
    deadline; the loop turns it into an lf_schedule of ``tf2_wake``. An ingest
    re-evaluates the pending requests in the ingest reaction itself, so a
    transform that arrives at tag t yields ``ok`` at t. The wake reaction is
    declared after the user reactions, so an ingest at a deadline tag is seen
    before the deadline decides, and before the corr arrives.
    """
    tf2 = getattr(node_info, "tf2", None) if node_info is not None else None
    if tf2 is None or not tf2.adapters or tf2.refusals:
        return reactions
    from colcon_lf.generation.binding_header_emitter import _cb_name_from_body
    from colcon_lf.node_analysis.tf2_scan import ingest_callback, TF_STATIC_TOPIC, TF_TOPIC

    ingest = {ingest_callback(lst.member, t) for lst in tf2.listeners
              for t in (TF_TOPIC, TF_STATIC_TOPIC)}
    user = [r for r in reactions if r.kind in ("timer", "sub")]
    if not user:
        return reactions
    names = {id(r): _cb_name_from_body(r, binding_key) for r in user}
    if not any(names[id(r)] in ingest for r in user):
        raise ValueError(
            f"Node {binding_key!r}: RCLLF-H003 tf2_split: an rcllf_tf2::AcquisitionLookup "
            "request can run in a reactor that does not hold the TransformListener's "
            "/tf and /tf_static reactions (a callback group other than the default one), "
            "so an ingest and a deadline at one tag would have no order"
        )
    loop = (
        f'{{ int64_t tw; while ((tw = ros2_lf_tf2_take_wake("{binding_key}")) >= 0) '
        f"{{ lf_schedule({TF2_WAKE_ACTION}, tw); }} }}"
    )
    for r in user:
        lines = r.body.split("\n")
        extra = [loop]
        if names[id(r)] in ingest:
            extra = [f'ros2_lf_tf2_ingested("{binding_key}");', loop]
        r.body = "\n".join(lines[:-1] + extra + [lines[-1]])
        r.effects.append(TF2_WAKE_ACTION)
    wake = LFReaction(
        triggers=[TF2_WAKE_ACTION], effects=[TF2_WAKE_ACTION],
        body=f'ros2_lf_tf2_wake("{binding_key}");\n{loop}',
        kind=TF2_WAKE_KIND,
    )
    close = LFReaction(
        triggers=["shutdown"], effects=[],
        body=f'ros2_lf_tf2_close("{binding_key}");',
        kind=TF2_CLOSE_KIND,
    )
    last_user = max(reactions.index(r) for r in user)
    return reactions[:last_user + 1] + [wake] + reactions[last_user + 1:] + [close]


def _build_lfc_body(
    cb: CallbackInfo,
    kind: str,
    effects: list[str],
    sub_topic_map: dict[str, str],
    binding_key: str,
    *,
    srv_info: "_SrvMaps | None" = None,
    emit_edges: list[int] | None = None,
    remote_edges: set[int] | frozenset[int] = frozenset(),
    local_edges: set[int] | frozenset[int] = frozenset(),
    zero_edges: set[int] | frozenset[int] = frozenset(),
    sub_port: str | None = None,
    published: dict[str, str] | None = None,
) -> str:
    """Build the lfc-compilable reaction body using ros2_lf_invoke_*.

    A subscription (*kind* ``sub``) runs once per write its stub input
    *sub_port* carries at this tag, and every run adds a write to each stub
    effect.
    """
    rid_macro = f"RID_{sanitize_lf_identifier(binding_key)}_{sanitize_lf_identifier(cb.name)}"
    lines: list[str] = []
    # P17e: count publishes per topic instead of one write per callback run.
    counted = _count_publishes() and kind in ("timer", "sub")
    published = published or {}

    if kind == "timer":
        lines.append(f"ros2_lf_invoke_timer(self, {rid_macro});")
    elif kind == "sub":
        topic = sub_topic_map.get(cb.name, "")
        if counted and sub_port:
            lines.append(f'ros2_lf_invoke_sub_n(self, {rid_macro}, "{topic}", rcllf_k);')
        else:
            lines.append(f'ros2_lf_invoke_sub(self, {rid_macro}, "{topic}");')
    elif kind == "srv_send":
        # The user's timer/sub callback also calls async_send_request.
        # ros2_lf_invoke_srv_send re-dispatches to the same user callback.
        svc_name = ""
        if srv_info and cb.service_clients_used:
            for s in cb.service_clients_used:
                if s in srv_info.async_client_services:
                    svc_name = s
                    break
        lines.append(f'ros2_lf_invoke_srv_send(self, {rid_macro}, "{svc_name}");')
    elif kind == "action_goal_send":
        # The user's timer/sub callback calls async_send_goal on an action client.
        # ros2_lf_invoke_action_goal_send re-dispatches to the same user callback.
        action_name = cb.action_clients_used[0] if cb.action_clients_used else ""
        lines.append(f'ros2_lf_invoke_action_goal_send(self, {rid_macro}, "{action_name}");')
    else:
        lines.append(f"/* unknown kind: {kind} */")

    # Forward the invoke's records of each admitted edge.
    for edge_id in emit_edges or []:
        lines.append(_corr_emit_line(
            edge_id, edge_id in remote_edges, edge_id in local_edges, edge_id in zero_edges,
        ))

    # A request port is raised once per request really sent (O1-2).
    requested: dict[str, str] = {}
    if srv_info:
        for s in cb.service_clients_used:
            if s in srv_info.async_client_services:
                requested.setdefault(service_to_request_port_name(s), s)

    # Count the publishes seen on each stub effect, or one write per run (P17e).
    writes = [
        _service_sent_add(port_name, rid_macro, requested[port_name])
        if port_name in requested
        else _sent_add(port_name, rid_macro, published[port_name])
        if counted and port_name in published else _stub_add(port_name)
        for port_name in effects
    ]
    if kind == "sub":
        run = _if_ran(lines[0], writes)
        if sub_port:
            # The corr emit forwards every record of this tag at once,
            # so it runs after the loop.
            return "\n".join(_per_write(sub_port, run) + lines[1:])
        return "\n".join(run + lines[1:])
    return "\n".join(lines + writes)


class _SrvMaps:
    """Lookup tables for service wiring within a single NodeInfo."""

    def __init__(
        self,
        async_client_services: set[str],
        response_cb_map: dict[str, str],
        server_cb_map: dict[str, str],
    ) -> None:
        self.async_client_services = async_client_services
        # service_name → response callback name (from ServiceClientInfo.response_callback)
        self.response_cb_map = response_cb_map
        # service_name → server callback name (from ServiceServerInfo.callback)
        self.server_cb_map = server_cb_map


def _build_srv_maps(node_info: NodeInfo) -> _SrvMaps:
    """Build service lookup maps for a NodeInfo."""
    async_client_services: set[str] = set()
    response_cb_map: dict[str, str] = {}
    server_cb_map: dict[str, str] = {}

    for sc in node_info.service_clients:
        if sc.call_style in ("async_callback", None):
            async_client_services.add(sc.service_name)
            if sc.response_callback:
                response_cb_map[sc.service_name] = sc.response_callback

    for ss in node_info.service_servers:
        server_cb_map[ss.service_name] = ss.callback

    return _SrvMaps(
        async_client_services=async_client_services,
        response_cb_map=response_cb_map,
        server_cb_map=server_cb_map,
    )


def _build_srv_reactions(
    node_info: NodeInfo,
    srv_info: _SrvMaps,
    *,
    binding_key: str | None = None,
) -> list[tuple[str | None, LFReaction]]:
    """Build service-specific reactions not driven by timer/sub triggers.

    Returns a list of (callback_group_name_or_None, LFReaction) pairs so the
    caller can route each reaction into the correct group sub-reactor.

    Emits:
    - srv_server reaction (request port trigger → response port effect)
    - srv_client_cb reaction (response port trigger, no pub effects)

    Both claim every action channel of ``_node_action_claims``,
    so a service callback that finishes a goal or sends one raises the port.
    """
    tagged: list[tuple[str | None, LFReaction]] = []
    claims = _node_action_claims(node_info)
    claim_body = "".join("\n" + _action_claim_line(*c) for c in claims)
    claim_effects = [c[0] for c in claims]

    class_san = sanitize_lf_identifier(
        binding_key if binding_key is not None else node_info.class_name
    )

    # Server reactions: one per ServiceServerInfo
    for ss in node_info.service_servers:
        req_port = service_to_request_port_name(ss.service_name)
        resp_port = service_to_response_port_name(ss.service_name)
        cb_san = sanitize_lf_identifier(ss.callback)
        rid_macro = f"RID_{class_san}_{cb_san}"
        svc_esc = ss.service_name.replace('"', '\\"')
        body = "\n".join(_per_write(req_port, _if_ran(
            f'ros2_lf_invoke_srv_server(self, {rid_macro}, "{svc_esc}");',
            [_stub_add(resp_port)],
        ))) + claim_body
        tagged.append((ss.callback_group, LFReaction(
            triggers=[req_port],
            effects=[resp_port, *claim_effects],
            body=body,
            kind="srv_server",
        )))

    # Client response reactions: one per async ServiceClientInfo
    for sc in node_info.service_clients:
        if sc.call_style not in ("async_callback", None):
            continue
        resp_port = service_to_response_port_name(sc.service_name)
        cb_name = sc.response_callback or f"response_callback_{sanitize_lf_identifier(sc.service_name)}"
        cb_san = sanitize_lf_identifier(cb_name)
        rid_macro = f"RID_{class_san}_{cb_san}"
        svc_esc = sc.service_name.replace('"', '\\"')
        body = "\n".join(_per_write(
            resp_port, [f'ros2_lf_invoke_srv_client_cb(self, {rid_macro}, "{svc_esc}");'],
        )) + claim_body
        tagged.append((sc.callback_group, LFReaction(
            triggers=[resp_port],
            effects=list(claim_effects),
            body=body,
            kind="srv_client_cb",
        )))

    return tagged


def _names_action(full: str, wanted: str) -> bool:
    """*full* names the action *wanted*: equal but for a leading slash, or
    *wanted* is its last path segments. The binding's ``names_action`` matches
    a claim to the sent samples the same way."""
    f, w = full.lstrip("/"), wanted.lstrip("/")
    return f == w or f.endswith("/" + w)


def _declared_action(action_name: str, infos) -> str:
    """The declared name of the action in *infos* that *action_name* names.

    Ports, claims and the binding carry the declared name, so
    ``/rebind/fib`` binds the server declared as ``/rebind/fib``.
    ``_check_action_short_names`` keeps the match unique.
    """
    for info in infos:
        if (_names_action(info.action_name, action_name)
                or _names_action(action_name, info.action_name)):
            return info.action_name
    return action_name


_ACTION_PORT_NAME = {
    "goal_req": action_to_goal_req_port_name,
    "result_req": action_to_result_req_port_name,
    "cancel_req": action_to_cancel_req_port_name,
    "goal_resp": action_to_goal_resp_port_name,
    "result": action_to_result_port_name,
    "cancel_resp": action_to_cancel_resp_port_name,
    "feedback": action_to_feedback_port_name,
}
# What a client callback or a server reaction can send.
_CLIENT_SENDS = ("goal_req", "result_req", "cancel_req")
_SERVER_SENDS = ("goal_resp", "result", "cancel_resp", "feedback")


def _action_claims(short: str, channels: tuple[str, ...]) -> list[tuple[str, str, str]]:
    """``(port, action, channel)`` per channel of action *short*."""
    return [(_ACTION_PORT_NAME[c](short), short, c) for c in channels]


def _action_claim_line(port: str, short: str, channel: str) -> str:
    """Raise *port* by the samples this invocation sent on *channel*.

    Two claims of one reaction add up, and a reaction that sent nothing leaves
    the port absent, so a port event always stands for real samples.
    """
    esc = short.replace('"', '\\"')
    return (
        f'{{ int n = ros2_lf_action_sent("{esc}", {_ACTION_CHANNEL[channel]}); '
        f"if (n > 0) {{ {_stub_add(port, 'n')} }} }}"
    )


def _callback_action_claims(
    cb: CallbackInfo, node_info: NodeInfo,
) -> list[tuple[str, str, str]]:
    """Action channels a timer or subscription callback may send on.

    Every request channel of each action client it uses (a cancel or a result
    request may come from any client callback), and feedback and result of
    each action server it publishes on. ``?`` (the analysis could not name the
    server) claims every server of the node. A claim that sees no send costs
    nothing, so the over-approximation never fires a port.
    """
    claims: list[tuple[str, str, str]] = []
    for action_name in cb.action_clients_used:
        claims += _action_claims(
            _declared_action(action_name, node_info.action_clients), _CLIENT_SENDS)
    servers = [a.action_name for a in node_info.action_servers]
    published = dict.fromkeys([*cb.feedback_published, *cb.result_published])
    for action_name in published:
        targets = servers if action_name == "?" else [
            _declared_action(action_name, node_info.action_servers)]
        for short in targets:
            if short in servers:
                claims += _action_claims(short, ("feedback", "result"))
    return list(dict.fromkeys(claims))


def _node_action_claims(node_info: NodeInfo) -> list[tuple[str, str, str]]:
    """Every action channel a callback of *node_info* can send on.

    The request channels of each action client and feedback and result of each
    action server. A reaction that runs a service or action callback claims all
    of them: a result callback of action A may send a goal on action B, and a
    service callback may finish a goal. A claim that sees no send costs
    nothing, so the over-approximation never fires a port.
    """
    claims: list[tuple[str, str, str]] = []
    for acli in node_info.action_clients:
        claims += _action_claims(acli.action_name, _CLIENT_SENDS)
    for asrv in node_info.action_servers:
        claims += _action_claims(asrv.action_name, ("feedback", "result"))
    return list(dict.fromkeys(claims))


def _check_action_short_names(node_info: NodeInfo) -> None:
    """Refuse two actions of one node whose names end in the same segment.

    The binding matches a claim to the sent samples by name, a bare ``move``
    also naming ``/x/move``, so two such actions would share counts.
    ``/left/move`` and ``/right/move`` name neither one the other, and
    their ports carry the declared names, so they pass.
    """
    names = list(dict.fromkeys(
        a.action_name.strip("/")
        for a in (*node_info.action_servers, *node_info.action_clients)))
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if _names_action(a, b) or _names_action(b, a):
                raise ValueError(
                    f"Node {node_info.class_name!r}: action {a!r} ends in the whole name "
                    f"{b!r} or the other way round. Their send counts would be shared, so "
                    "rcllf refuses the node. Give one of them a name that is "
                    "not a path suffix of the other."
                )


def _build_action_reactions(
    node_info: NodeInfo,
    *,
    binding_key: str | None = None,
    external: set[str] | frozenset[str] = frozenset(),
) -> list[tuple[str | None, LFReaction]]:
    """Build action-specific reactions not driven by timer/sub triggers.

    Returns a list of (callback_group_name_or_None, LFReaction) pairs so the
    caller can route each reaction into the correct group sub-reactor.

    Every channel is a counted port. A receiving reaction takes
    exactly the port's count of samples, waiting a bounded time for each, and
    then claims what its callbacks sent in turn:

    - action_goal_server: goal_req, takes goal requests (handle_goal,
      handle_accepted), claims goal_resp, feedback and result
    - action_result_server: result_req, takes result requests, claims result
      (an answer for a goal that is already terminal)
    - action_cancel_server: cancel_req, takes cancel requests (handle_cancel),
      claims cancel_resp, feedback and result
    - action_goal_cb, action_feedback, action_result_cb, action_cancel_cb: the
      client's goal response, feedback, result and cancel response

    Each of them but the result server also claims every channel of
    ``_node_action_claims``, so a result callback of one action that sends a
    goal on another raises that action's port. Feedback and results a server
    sends later come from the user's own timer or subscription callbacks, which
    claim them (``_callback_action_claims``).

    A node with action servers also gets one reaction(shutdown) in its first
    server's group (item 6). It runs the pre-shutdown callbacks the binding kept
    for the node under the first goal server's RID and claims the same node-wide
    channels, so a goal those callbacks abort sends its result in a reaction.

    Two actions of the node whose names end in the same segment are refused
    (``_check_action_short_names``).

    A server whose short name is in *external* has no client inside the LF
    system. One physical action, scheduled by the binding
    when a request arrives, triggers its three reactions, and each takes the
    count of its own entity that arrived since it last ran.
    """
    tagged: list[tuple[str | None, LFReaction]] = []
    class_san = sanitize_lf_identifier(
        binding_key if binding_key is not None else node_info.class_name
    )

    def reaction(trigger: str, invoke: str, rid_cb: str, short: str,
                 sends: tuple[str, ...], kind: str,
                 ext: tuple[str, str, int] | None = None) -> LFReaction:
        rid_macro = f"RID_{class_san}_{sanitize_lf_identifier(rid_cb)}"
        esc = short.replace('"', '\\"')
        # GetResult runs no user callback, so it sends only the result.
        wide = node_claims if kind != "action_result_server" else []
        claims = list(dict.fromkeys([*_action_claims(short, sends), *wide]))
        count = f"{trigger}->value"
        ext_action = ext_rid = None
        if ext is not None:
            # ext = (physical action, goal server RID macro, entity)
            trigger, ext_rid, entity = ext
            ext_action = trigger
            count = f"ros2_lf_action_external_pending(self, {ext_rid}, {entity})"
        lines = [f'{invoke}(self, {rid_macro}, "{esc}", {count});']
        lines += [_action_claim_line(*c) for c in claims]
        return LFReaction(
            triggers=[trigger], effects=[c[0] for c in claims],
            body="\n".join(lines), kind=kind,
            external_action=ext_action, external_register_rid=ext_rid,
        )

    # A RID without a user callback gets a synthesized name. With several
    # servers (or clients) on one node it names the action too, so two RIDs
    # never share a macro.
    def synth(name: str, short: str, several: bool) -> str:
        return f"{name}_{sanitize_lf_identifier(short)}" if several else name

    _check_action_short_names(node_info)
    node_claims = _node_action_claims(node_info)
    several_srv = len(node_info.action_servers) > 1
    for asrv in node_info.action_servers:
        # Ports, claims, RIDs and the binding take the declared name.
        short = name = asrv.action_name
        grp = asrv.callback_group
        goal_cb = asrv.goal_callback or synth("handle_goal", short, several_srv)

        def ext(entity: int) -> tuple[str, str, int] | None:
            if short not in external:
                return None
            goal_rid = f"RID_{class_san}_{sanitize_lf_identifier(goal_cb)}"
            return (external_action_server_name(short), goal_rid, entity)

        tagged.append((grp, reaction(
            action_to_goal_req_port_name(name), "ros2_lf_invoke_action_goal_server",
            goal_cb, name, ("goal_resp", "feedback", "result"), "action_goal_server",
            ext(0),
        )))
        # GetResult has no user callback.
        tagged.append((grp, reaction(
            action_to_result_req_port_name(name), "ros2_lf_invoke_action_result_server",
            synth("get_result_callback", short, several_srv), name, ("result",),
            "action_result_server", ext(1),
        )))
        tagged.append((grp, reaction(
            action_to_cancel_req_port_name(name), "ros2_lf_invoke_action_cancel_server",
            asrv.cancel_callback or synth("handle_cancel", short, several_srv), name,
            ("cancel_resp", "feedback", "result"), "action_cancel_server", ext(2),
        )))

    if node_info.action_servers:
        first = node_info.action_servers[0]
        first_short = first.action_name
        first_goal = first.goal_callback or synth("handle_goal", first_short, several_srv)
        claims = node_claims
        lines = [
            "ros2_lf_action_server_shutdown(self, "
            f"RID_{class_san}_{sanitize_lf_identifier(first_goal)});"
        ]
        lines += [_action_claim_line(*c) for c in claims]
        tagged.append((first.callback_group, LFReaction(
            triggers=["shutdown"], effects=[c[0] for c in claims],
            body="\n".join(lines), kind=ACTION_SHUTDOWN_KIND,
        )))

    several_cli = len(node_info.action_clients) > 1
    for acli in node_info.action_clients:
        short = name = acli.action_name
        grp = acli.callback_group
        # async_cancel_goal returns a future, not a callback, so the cancel
        # response RID gets a synthesized name per action.
        for port, cb_name, kind in (
            (action_to_goal_resp_port_name(name),
             acli.goal_response_callback
             or synth("goal_response_callback", short, several_cli), "action_goal_cb"),
            (action_to_feedback_port_name(name),
             acli.feedback_callback or synth("feedback_callback", short, several_cli),
             "action_feedback"),
            (action_to_result_port_name(name),
             acli.result_callback or synth("result_callback", short, several_cli),
             "action_result_cb"),
            (action_to_cancel_resp_port_name(name),
             f"cancel_response_callback_{sanitize_lf_identifier(short)}", "action_cancel_cb"),
        ):
            tagged.append((grp, reaction(
                port, "ros2_lf_invoke_action_client_cb", cb_name, name, _CLIENT_SENDS, kind,
            )))

    return tagged


def _ports_for_reactions(
    reactions: list[LFReaction],
    port_type_map: dict[str, str],
    port_lf_type: str,
    *,
    exclude_triggers: set[str] | None = None,
) -> list[LFPort]:
    """Return the LFPort list (inputs + outputs) needed by a set of reactions.

    Used to add service/action ports to group sub-reactors that own those
    reactions. *exclude_triggers* names triggers that are actually timers.
    Without it, a reaction triggered by a timer would get a spurious ``input``
    port declared for the timer's name.
    """
    exclude = set(exclude_triggers or ())
    # External action servers are triggered by a physical action,
    # and an action server node's shutdown abort (item 6) by shutdown.
    exclude |= {r.external_action for r in reactions if r.external_action}
    exclude |= {"startup", "shutdown"}
    needed_inputs: set[str] = set()
    needed_outputs: set[str] = set()
    for r in reactions:
        for t in r.triggers:
            if t not in exclude:
                needed_inputs.add(t)
        for e in r.effects:
            needed_outputs.add(e)
    ports: list[LFPort] = []
    for inp in sorted(needed_inputs):
        lf_type = port_type_map.get(inp, port_lf_type)
        ports.append(LFPort(name=inp, type=lf_type, direction="input"))
    for out in sorted(needed_outputs):
        lf_type = port_type_map.get(out, port_lf_type)
        ports.append(LFPort(name=out, type=lf_type, direction="output"))
    return ports


def _topic_path_segments(topic: str) -> list[str]:
    """Split a topic string into its non-empty '/'-separated path segments."""
    return [seg for seg in topic.strip("/").split("/") if seg]


def _endpoint_port_names(
    resolved_topic: str,
    infos: list,
    sub_node: NodeInfo | None = None,
    scope: NodeIdentity | None = None,
) -> list[str]:
    """Declared port names of the endpoints in *infos* on *resolved_topic*.

    Every endpoint ``_match_endpoint_infos`` finds counts, so two declared
    subscriptions that a launch remap joins on one topic give two ports.
    Names follow the identical ``topic_to_port_name`` call
    ``_build_ports`` used for the declaration, in declaration order, without
    repeats (two subscriptions on one declared topic share one port). Pass
    *sub_node* when *infos* are its subscribers, so the name follows
    ``_sub_port_name``. Empty when nothing matches, the caller
    then falls back.

    Several names count only when resolved names confirm every match, that
    is with a launch identity or when every match's absolute name equals
    *resolved_topic*. Unconfirmed suffix matches with several names are
    ambiguous and give an empty list, the old fallback.
    """
    matches = _match_endpoint_infos(resolved_topic, infos, scope)
    names: list[str] = []
    for info in matches:
        name = (_sub_port_name(sub_node, info.topic) if sub_node is not None
                else topic_to_port_name(info.topic))
        if name not in names:
            names.append(name)
    confirmed = (scope is not None and scope.from_launch) or all(
        resolve_name(m.topic, None, []) == resolved_topic for m in matches)
    return names if len(names) == 1 or confirmed else []


def _match_endpoint_infos(
    resolved_topic: str, infos: list, scope: NodeIdentity | None = None,
    attr: str = "topic",
) -> list:
    """Every info of one node whose topic matches *resolved_topic*.

    *attr* names the info field holding the declared name: ``topic`` for
    publishers and subscriptions, ``service_name`` for service endpoints.

    With a launch identity (*scope* from a launch action) an endpoint matches
    when its name, resolved with the node's namespace and remaps by the
    analyzer's own ``resolve_name``, equals *resolved_topic*. Without one the node-local topic must be a path suffix of
    *resolved_topic* or vice versa, and when several
    endpoints match that way the ones whose absolute name equals
    *resolved_topic* win. A private ``~/x`` name expands with the scope's node
    name and namespace either way.
    """
    if scope is not None and scope.from_launch:
        ns = scope.namespace or None
        remaps = list(scope.remappings)
        return [i for i in infos if getattr(i, attr) and resolve_name(
            getattr(i, attr), ns, remaps, scope.node_name) == resolved_topic]

    def local_name(info) -> str:
        name = getattr(info, attr)
        if scope is not None and name and name.startswith("~"):
            return resolve_name(name, scope.namespace or None, [], scope.node_name)
        return name

    resolved_segs = _topic_path_segments(resolved_topic)
    matches = []
    for info in infos:
        local_segs = _topic_path_segments(local_name(info))
        if not local_segs:
            continue
        if (
            local_segs == resolved_segs[-len(local_segs):]
            or resolved_segs == local_segs[-len(resolved_segs):]
        ):
            matches.append(info)
    exact = [m for m in matches if resolve_name(local_name(m), None, []) == resolved_topic]
    if exact and len({getattr(m, attr) for m in matches}) > 1:
        return exact
    return matches


def _declared_stem(
    resolved: str, infos: list, scope: NodeIdentity | None, *, attr: str,
    node_key: str, kind: str,
) -> str:
    """The one declared name ``_build_ports`` named this endpoint's ports from.

    Matched like a topic endpoint (``_endpoint_port_names``), so ``/rebind/add``
    keeps ``rebind_add_req`` and a node-local ``add`` keeps ``add_req``. No match, or several declared names on *resolved*, is a generation
    error: any other stem names a port the node does not declare, which lfc
    rejects.
    """
    names = sorted({getattr(i, attr) for i in _match_endpoint_infos(
        resolved, infos, scope, attr=attr)})
    if len(names) == 1:
        return names[0]
    if names:
        raise ValueError(
            f"Node {node_key!r}: {kind} names {', '.join(map(repr, names))} all resolve "
            f"to {resolved!r}; one LF connection cannot pick between their ports, "
            "rename or remap them apart"
        )
    raise ValueError(
        f"Node {node_key!r}: no declared {kind} resolves to {resolved!r}, so there is "
        "no port to connect"
    )


def _link_delay_ns(tc: TopicConnection, after_delay_ns: int | None) -> int:
    """The after-delay ``_build_connections`` puts on a topic link."""
    if tc.after_delay_ns is not None:
        return tc.after_delay_ns
    if after_delay_ns is not None:
        return after_delay_ns
    return DEFAULT_AFTER_DELAY_NS


def _callback_reaction_kind(node: NodeInfo, cb: CallbackInfo) -> str | None:
    """Kind ``_build_reactions`` gives *cb*'s reaction, None if it gets none."""
    kind = None
    if any(t.callback == cb.name for t in node.timers):
        kind = "timer"
    if any(s.callback == cb.name for s in node.subscribers):
        kind = "sub"
    if kind is None:
        return None
    if kind != "timer":
        return kind  # a subscription sender keeps kind sub (O1-1)
    async_srvs = _build_srv_maps(node).async_client_services
    if any(s in async_srvs for s in cb.service_clients_used):
        kind = "srv_send"
    if cb.action_clients_used:
        kind = "action_goal_send"
    return kind


def _runs_inside_reactions(
    node: NodeInfo, cb: CallbackInfo, topic: str, _seen: tuple[str, ...] = (),
) -> bool:
    """True when helper *cb* runs only inside timer or subscription reactions.

    Every caller the analysis lists (``CallbackInfo.callers``,
    None when the set is not closed) is a timer or subscription callback that
    also lists *topic*, or another such helper. An unknown or empty caller
    set, a caller that is not a callback of the node, a cycle, or a helper
    that sends a request or a goal is refused.
    """
    if not cb.callers or cb.service_clients_used or cb.action_clients_used:
        return False
    by_name = {c.name: c for c in node.callbacks}
    seen = (*_seen, cb.name)
    for name in cb.callers:
        caller = by_name.get(name)
        if caller is None or name in seen or topic not in caller.publishers_used:
            return False
        kind = _callback_reaction_kind(node, caller)
        if kind is None:
            if not _runs_inside_reactions(node, caller, topic, seen):
                return False
        elif kind not in ("timer", "sub"):
            return False
    return True


def _tf2_static_edge(edge: CorrEdge, pub: NodeInfo, pub_info: PublisherInfo,
                     sub_info: SubscriberInfo) -> bool:
    """A /tf_static edge from a tf2 StaticTransformBroadcaster to the admitted
    TransformListener's input.

    Both endpoints are transient_local by tf2_ros's own QoS. Durability
    matters only for a reader that joins after a write: then DDS replays
    history that no publish at a tag produced. Here both nodes are
    constructed before tag 0 and the admission below still requires the
    broadcaster to publish only inside reactions, so every sample the
    listener takes is one publish at one tag, as on a volatile edge.
    """
    if edge.topic != "/tf_static" or not getattr(sub_info, "tf2_listener", None):
        return False
    tf2 = getattr(pub, "tf2", None)
    return tf2 is not None and any(
        b.static and b.member == pub_info.member_variable for b in tf2.broadcasters)


def _corr_refusal(
    edge: CorrEdge,
    pub: NodeInfo | None,
    sub: NodeInfo | None,
    *,
    federated: bool,
    admitted_subs: set[tuple[str, str]],
    sub_info: SubscriberInfo | None = None,
    pub_scope: NodeIdentity | None = None,
    sub_scope: NodeIdentity | None = None,
) -> str | None:
    """Generation-stage admission of one topic edge (D1). None = admitted.

    Fills the edge's node-local topics and callbacks when they resolve.
    *sub_info* picks one of twin subscriptions (``_twin_subscriptions``).
    """
    if pub is None or sub is None:
        return "unknown_node"
    # Resolve the node-local endpoints first, also for edges refused below:
    # a legacy edge must keep its topic's stub port alive.
    pub_infos = _match_endpoint_infos(edge.topic, pub.publishers, pub_scope)
    sub_infos = [sub_info] if sub_info is not None else _match_endpoint_infos(
        edge.topic, sub.subscribers, sub_scope)
    if len(pub_infos) == 1:
        edge.pub_topic = pub_infos[0].topic
    if len(sub_infos) == 1:
        edge.sub_topic = sub_infos[0].topic
        edge.listed_sub_callback = sub_infos[0].callback
        edge.sub_reliability = sub_infos[0].qos_reliability or ""
        edge.sub_depth = sub_infos[0].qos_depth
    elif sub_infos:
        # Twins left ambiguous. The binding cannot tell
        # them apart at startup, so the row carries the weakest QoS the
        # analysis saw on any of them (a hazard it saw is not repeated). Every
        # twin matched the graph topic, so twins spelled differently ("data",
        # "/data") leave sub_topic empty and the binding resolves the graph topic.
        if len({i.topic for i in sub_infos}) == 1:
            edge.sub_topic = sub_infos[0].topic
        rels = [i.qos_reliability or "" for i in sub_infos]
        edge.sub_reliability = "best_effort" if "best_effort" in rels else rels[0]
        depths = [i.qos_depth for i in sub_infos]
        edge.sub_depth = 1 if 1 in depths else depths[0]
    if federated:
        return "federated"
    if len(pub_infos) != 1 or len(sub_infos) != 1:
        return "ambiguous_endpoint"
    pub_info, sub_info = pub_infos[0], sub_infos[0]
    # A lifecycle node's edges are admitted. The binding binds a
    # lifecycle publisher whenever the node creates it (``pub_rebind``).
    # A lifecycle subscription made or reset in a hook is a
    # deferred binding (``sub_rebind``): the binding admits every reader the
    # node creates for it as a new generation, or refuses it with a typed
    # error when it cannot prove which twin the reader is. Every twin of such
    # a subscription is deferred too, so the run-time handle count of a twin
    # never meets a reader the node has not made yet.
    topic_subs = _match_endpoint_infos(edge.topic, sub.subscribers, sub_scope) or [sub_info]
    rebind = bool(sub.is_lifecycle and any(
        i.created_in != "constructor" or i.destroyed_in is not None for i in topic_subs
    ))
    if rebind and len(topic_subs) > 1:
        # No mechanism proves which twin a re-created
        # reader is, so the binding refuses it (rebind_identity). Twins the
        # analysis sees, one listing each, stay legacy here
        # instead of being admitted for a refusal at run time.
        return "ambiguous_endpoint"
    edge.sub_rebind = rebind
    edge.pub_rebind = bool(pub.is_lifecycle or getattr(pub_info, "is_lifecycle", False))
    # W3 slice S3: the binding classifies a serialized publish as
    # UNSUPPORTED_API, so such an edge would never reconcile.
    if getattr(pub_info, "serialized_publish", False):
        return "unsupported_api"
    if ("transient_local" in (pub_info.qos_durability, sub_info.qos_durability)
            and not _tf2_static_edge(edge, pub, pub_info, sub_info)):
        return "transient_local"
    pub_cbs = [cb for cb in pub.callbacks if pub_info.topic in cb.publishers_used]
    # A member helper publishes inside the reactions that call it,
    # and the analysis folds its topics into theirs, so it is left out when
    # every call into it comes from a timer or subscription callback.
    pub_cbs = [
        cb for cb in pub_cbs
        if _callback_reaction_kind(pub, cb) is not None
        or not _runs_inside_reactions(pub, cb, pub_info.topic)
    ]
    # A message_filters synchronizer callback
    # runs only synchronously inside its inputs' rclcpp callbacks, which the
    # analysis models as the <cb>_via_<member> subscription callbacks
    # (message_filters_scan.py). Its publications belong to the dispatched
    # input activation, so it is not a publishing callback of its own.
    folded = {s.sync_callback for s in pub.subscribers if s.sync_callback}
    pub_cbs = [cb for cb in pub_cbs if cb.name not in folded]
    if not pub_cbs:
        return "publisher_callback"
    if any(_callback_reaction_kind(pub, cb) not in ("timer", "sub") for cb in pub_cbs):
        return "publisher_callback_kind"
    # Several publishing callbacks share the edge's one port only
    # when they run in one LF reactor, whose reactions LF serializes in
    # priority order; that order is the records' (and the ordinals') order.
    if len({_callback_reactor(pub, cb.name) for cb in pub_cbs}) != 1:
        return "publisher_callback_split"
    edge.pub_callbacks = [cb.name for cb in pub_cbs]
    sub_cb = next((cb for cb in sub.callbacks if cb.name == sub_info.callback), None)
    if sub_cb is None or _callback_reaction_kind(sub, sub_cb) != "sub":
        return "subscriber_callback"
    # A subscription callback that sends a request or goal keeps kind sub
    # (O1-1) but stays legacy: the drain writes its stub ports once per drain,
    # not once per request sent, so the server took a request that did not
    # exist on every message without one (review round 5, range_guard run:
    # request_shortfalls=43 for 14 requests in 56 messages).
    if sub_cb.service_clients_used or sub_cb.action_clients_used:
        return "subscriber_callback"
    edge.sub_callback = sub_cb.name
    if (edge.sub_node, sub_cb.name) in admitted_subs:
        return "duplicate_subscription"
    if edge.pub_node == edge.sub_node and {
        _callback_reactor(pub, cb.name) for cb in pub_cbs
    } != {_callback_reactor(sub, sub_cb.name)}:
        # A self edge has no corr_e link (it would be a zero-delay
        # cycle), its publishing reactions schedule the drain action instead,
        # and a reaction schedules only actions of its own reactor. So the
        # subscription must share the publishers' one reactor (the rule).
        return "self_loop_split"
    return None


def _twin_subscriptions(
    topic: str, sub: NodeInfo | None, listed: int, scope: NodeIdentity | None = None,
) -> list[SubscriberInfo | None]:
    """The subscription each of *listed* edges into *sub* on *topic* belongs to.

    ``map_communication`` lists a node once per subscription on a topic, so a
    node with two subscriptions on it (twins) is *listed* twice.
    The i-th listing gets the i-th subscription when provenance tells them
    apart, that is when each has its own callback and so its own RID, by
    which the binding picks the subscription handle (W3 S3). Otherwise
    every entry is ``None`` and ``_corr_refusal`` refuses ``ambiguous_endpoint``.
    """
    if sub is None or listed < 2:
        return [None] * listed
    infos = _match_endpoint_infos(topic, sub.subscribers, scope)
    callbacks = [i.callback for i in infos]
    if len(infos) != listed or "" in callbacks or len(set(callbacks)) != len(callbacks):
        return [None] * listed
    return list(infos)


def _callback_reactor(node: NodeInfo, cb_name: str) -> tuple[str, ...]:
    """The sub-reactor ``_build_reactor_with_groups`` places *cb_name* in.

    One per MutuallyExclusive group (the default group included), one per
    callback of a Reentrant group.
    """
    group = "default_group"
    for trig in (*node.timers, *node.subscribers):
        if trig.callback == cb_name:
            group = trig.callback_group or "default_group"
            break
    kinds = {g.name: g.type for g in node.callback_groups}
    if kinds.get(group) == "Reentrant":
        return (group, cb_name)
    return (group,)


def _build_corr_edges(
    communication: SystemCommunication,
    nodes: dict[str, NodeInfo],
    *,
    after_delay_ns: int | None,
    runtime_to_key: dict[str, str],
    binding_keys: dict[str, str],
    link_delays: LinkDelays | None = None,
    federated: bool = False,
    bounds: CorrBoundsConfig | None = None,
    timer_period_overrides: dict[str, dict[str, float]] | None = None,
    xfed: XfedContext | None = None,
    scopes: dict[str, NodeIdentity] | None = None,
    reaction_order: dict[str, str] | None = None,
) -> tuple[list[CorrEdge], dict[str, _NodeCorr]]:
    """Topic edge table in ``_build_connections`` order (D1).

    With *xfed* the table is the whole federated system's: an
    edge between two federates gets role ``cross``, its federate link's
    after as its delay, and the cross-federate refusals of ``_xfed_refusal``.

    Admitted edges get dense ids from 0 in that order. A topic's stub port is
    dropped only where no legacy edge still needs it. An edge whose retention
    and carrier bounds cannot be computed from the schedule is refused
    (``_size_corr_edges``).

    The edges of a topic into one subscription (one subscriber
    listing, so one twin) form a fan-in group, judged as a unit at the
    position of its first edge: a member refused for any reason keeps it and
    the others get ``fan_in_member_refused``, and a group of two or more
    can also be refused as a whole (``_fan_in_refusal``). A node listed once
    per publisher is a member on its first listing only; a repeat
    listing is judged on its own, as before, and never admitted alone next
    to its refused group.
    """
    scopes = scopes or {}
    edges: list[CorrEdge] = []
    admitted_subs: set[tuple[str, str]] = set()
    next_id = 0
    groups = 0

    def judge(edge: CorrEdge, sub_info: SubscriberInfo | None, cross: bool) -> str | None:
        reason = _corr_refusal(
            edge, nodes.get(edge.pub_node), nodes.get(edge.sub_node),
            federated=federated, admitted_subs=admitted_subs, sub_info=sub_info,
            pub_scope=scopes.get(edge.pub_node), sub_scope=scopes.get(edge.sub_node),
        )
        if reason is None and cross:
            reason = _xfed_refusal(edge, nodes[edge.pub_node], xfed)
        # LF allows zero-delay feedback loops as long as there is no causality
        # loop, so an edge is never refused for closing a loop. An
        # in-process d = 0 edge deliver at its publication tag, its port
        # triggering the reaction that runs the callback, so a loop of such
        # edges on which every subscription publishes onward into the loop at
        # that tag is a causality loop. ``_refuse_causality_loops`` makes that
        # a generation error, as lfc does, and adds no implicit microstep.
        return reason

    for tc in communication.topics:
        delay = _link_delay_ns(tc, after_delay_ns)
        pub_keys = [runtime_to_key.get(p, p) for p in tc.publishers]
        listed = Counter(runtime_to_key.get(s, s) for s in tc.subscribers)
        # The twin assignment is one per subscriber listing, built once and
        # reused for every publisher listing. A node with two publishers on the
        # topic is listed twice in pub_keys, and each of its passes walks the
        # same subscriber listings (its repeat edges then refuse as
        # duplicate_subscription).
        twins = {
            k: _twin_subscriptions(tc.topic, nodes.get(k), n, scopes.get(k))
            for k, n in listed.items()
        }
        # (edge, slot, subscription, cross, first listing of its publisher in the slot)
        rows: list[tuple[CorrEdge, tuple[str, int], SubscriberInfo | None, bool, CorrEdge | None]] = []
        members: dict[tuple[str, int], list[int]] = {}  # slot -> member rows
        for pub_key in pub_keys:
            if pub_key not in nodes:
                continue
            seen: Counter[str] = Counter()
            for sub_node in tc.subscribers:
                sub_key = runtime_to_key.get(sub_node, sub_node)
                if sub_key not in nodes:
                    continue
                slot = (sub_key, seen[sub_key])
                sub_info = twins[sub_key][seen[sub_key]]
                seen[sub_key] += 1
                cfg = link_delays.get(tc.topic, pub_key, sub_key) if link_delays else None
                edge = CorrEdge(
                    topic=tc.topic, pub_node=pub_key, sub_node=sub_key,
                    pub_key=binding_keys.get(pub_key, pub_key),
                    sub_key=binding_keys.get(sub_key, sub_key),
                    delay_ns=cfg.ns if cfg else delay,
                )
                cross = xfed is not None and xfed.federate(pub_key) != xfed.federate(sub_key)
                if cross:
                    edge.role = "cross"
                    edge.link_ports = xfed.link_ports(edge, sub_info, scopes)
                    # The in-process delay unless an override names the link.
                    edge.delay_ns = xfed.link_after_ns(*edge.link_ports, edge.delay_ns)
                slot_rows = members.setdefault(slot, [])
                first = next((rows[i][0] for i in slot_rows if rows[i][0].pub_node == pub_key), None)
                if first is None:
                    slot_rows.append(len(rows))
                rows.append((edge, slot, sub_info, cross, first))
        judged: set[tuple[str, int]] = set()
        for edge, slot, sub_info, cross, first in rows:
            if first is not None:
                reason = judge(edge, sub_info, cross)
                if reason is None and first.status != "admitted":
                    # The reader is shared with the refused group's members.
                    reason, edge.note = first.reason, first.note
                if reason is not None:
                    edge.reason = reason
            elif slot not in judged:
                judged.add(slot)
                group = [(rows[i][0], rows[i][2], rows[i][3]) for i in members[slot]]
                # A member node with two publisher objects
                # on the topic would put two writers under one rank.
                shared = [e.pub_node for e, _, _ in group if len(_match_endpoint_infos(
                    e.topic, nodes[e.pub_node].publishers, scopes.get(e.pub_node))) > 1]
                if _judge_fan_in_group(group, judge, nodes, groups, shared=shared):
                    groups += 1
                    for member, _, _ in group:
                        member.status, member.edge_id = "admitted", next_id
                        next_id += 1
                        admitted_subs.add((member.sub_node, member.sub_callback))
            edges.append(edge)

    _size_corr_edges(
        edges, nodes, communication,
        bounds=bounds or CorrBoundsConfig(),
        runtime_to_key=runtime_to_key,
        timer_period_overrides=timer_period_overrides or {},
        scopes=scopes,
    )
    _refuse_causality_loops(edges, nodes, reaction_order,
                            zero_cross=xfed is not None and not xfed.decentralized)
    if xfed is not None:
        _warn_cross_federate_join_spill(edges, xfed)
    refused = [e for e in edges if e.status != "admitted"]
    # The whole-system pass of a federated build (federated, no xfed) only
    # names the system and refuses every edge as "federated". The system
    # admission of plan_federated_corr (xfed set) is the one that counts.
    naming_pass = federated and xfed is None
    if bounds is not None and bounds.require_admitted and refused and not naming_pass:
        # The system's determinism claim needs every edge.
        raise ValueError(
            "require_admitted: refused topic edge(s) "
            + "; ".join(f"{e.topic} {e.pub_node} -> {e.sub_node}: {e.reason}" for e in refused)
        )
    if bounds is not None and bounds.require_admitted and not naming_pass:
        _refuse_unused_input_periods(edges, nodes, bounds)
    return edges, _node_corr_from_edges(edges)


def _refuse_unused_input_periods(
    edges: list[CorrEdge], nodes: dict[str, NodeInfo], bounds: CorrBoundsConfig,
) -> None:
    """Fail on a ``min_input_period_ms`` that sizes no bound.

    The binding checks only the declarations a bound was sized from (they are
    what ``register_min_input_period`` emits), so any other declaration reads
    like a contract nobody enforces. Under ``require_admitted`` that is a
    generation error naming the node, the topic and why it sizes nothing.
    """
    used = {
        key for e in edges if e.status == "admitted"
        for key in e.bound_inputs.get("min_input_periods_ns", {})
    }
    problems = []
    for (node_key, topic), period in sorted(bounds.min_input_period_ms.items()):
        if f"{node_key}:{topic}" in used:
            continue
        node = nodes.get(node_key)
        if node is None:
            why = "no node of that key is in the system"
        elif not any(sub.topic == topic for sub in node.subscribers):
            why = "the node has no subscription on that topic"
        else:
            why = ("no admitted correspondence edge is sized from this input, so the "
                   "binding would never check it")
        problems.append(f"{node_key}:{topic} (min_input_period_ms={period}): {why}")
    if problems:
        raise ValueError(
            "require_admitted: min_input_period_ms declaration(s) size no bound; "
            + "; ".join(problems)
        )


def _judge_fan_in_group(
    group: list[tuple[CorrEdge, SubscriberInfo | None, bool]],
    judge: Callable[[CorrEdge, SubscriberInfo | None, bool], str | None],
    nodes: dict[str, NodeInfo],
    index: int,
    *,
    shared: list[str] | None = None,
) -> bool:
    """Admit or refuse one fan-in group as a unit. True when admitted.

    Each member ``(edge, subscription, cross)`` is judged on its own first,
    which also resolves its endpoints for a legacy stub. A group of two or
    more whose *shared* member nodes have two publisher objects on the topic
    is refused as a whole (``fan_in_shared_publisher``):
    their writers would share one rank, so (release tag, rank, ordinal)
    would not order the group. Otherwise the first refused member names
    itself in the ``fan_in_member_refused`` note of every member that
    passed. An admitted group gets ``fan_in_group`` *index*, and each member
    the rank of its publisher's LF instance name among the members' (Q1
    ruling), which survives the renumbering after refusals and the federate
    split.
    """
    edges = [e for e, _, _ in group]
    reasons = [judge(e, info, cross) for e, info, cross in group]
    for e, reason in zip(edges, reasons):
        e.reason = reason
    if len(edges) > 1 and shared:
        note = _fan_in_note(edges, f"{', '.join(shared)} publish(es) on {edges[0].topic} "
                                   f"from two publisher objects, two writers under one rank")
        for e in edges:
            e.reason, e.note = "fan_in_shared_publisher", note
        return False
    cause = next((e for e in edges if e.reason is not None), None)
    if cause is not None:
        note = _fan_in_note(edges, f"member {cause.label()} is legacy ({cause.reason})")
        for e in edges:
            if e.reason is None:
                e.reason, e.note = "fan_in_member_refused", note
        return False
    if len(edges) > 1:
        reason, why = _fan_in_refusal(edges, nodes)
        if reason is not None:
            note = _fan_in_note(edges, why)
            for e in edges:
                e.reason, e.note = reason, note
            return False
    order = sorted(edges, key=lambda e: (sanitize_lf_identifier(e.pub_node), e.pub_node))
    for e in edges:
        e.fan_in_group, e.fan_in_rank = index, order.index(e)
    return True


def _fan_in_refusal(group: list[CorrEdge], nodes: dict[str, NodeInfo]) -> tuple[str | None, str]:
    """Group-only refusal of a fan-in group of two or more.

    ``(reason, why)``, or ``(None, "")`` when the group may be admitted. The
    members share one drain, one wake and one order cursor, so they must
    release at one offset after their publication (``fan_in_release_mismatch``:
    ``d`` here, a deferred group's ``T_e`` after chaining in
    ``_size_corr_edges``). A self member's inline arrive, a
    lifecycle member that rebinds and a BoundedNearestToAnchor
    input are not supported in a group yet, nor members of which some
    cross federates and some do not (``fan_in_mixed_locality``, P5): a local
    record arrives at its publication tag, a remote one only at its release.
    """
    for e in group:
        if e.pub_node == e.sub_node:  # Known limitation: a self-edge member is refused
            return "fan_in_self_loop", f"member {e.label()} is a self edge"
    for e in group:
        if e.pub_rebind or e.sub_rebind:  # Known limitation: a rebinding member is refused
            return "fan_in_rebind", f"member {e.label()} rebinds with its lifecycle node"
    if len({e.role == "cross" for e in group}) > 1:  # Known limitation: mixed locality is refused
        crossing = ", ".join(e.pub_node for e in group if e.role == "cross")
        local = ", ".join(e.pub_node for e in group if e.role != "cross")
        return "fan_in_mixed_locality", (
            f"{crossing} publish from another federate while {local} share the "
            f"subscriber's federate, so a remote record could be dispatched at a "
            f"later local release")
    sub = group[0]
    if sub.sub_callback in _bna_input_callbacks(nodes.get(sub.sub_node)):  # Known limitation: BNA input
        return "fan_in_bna_input", f"{sub.sub_node}.{sub.sub_callback} is a BoundedNearestToAnchor input"
    if len({e.delay_ns for e in group}) > 1:  # Known limitation: unequal delays are refused
        delays = ", ".join(f"{e.pub_node} {e.delay_ns} ns" for e in group)
        return "fan_in_release_mismatch", f"the members' delays differ ({delays})"
    return None, ""


def _fan_in_note(group: list[CorrEdge], why: str) -> str:
    """The ``note`` of a refused fan-in group's members."""
    pubs = ", ".join(e.pub_node for e in group)
    return (f"the fan-in group of {pubs} into {group[0].sub_node} on {group[0].topic} is "
            f"admitted or refused as a unit, and {why}")


def _node_corr_from_edges(edges: list[CorrEdge]) -> dict[str, _NodeCorr]:
    """Per-node ports and reactions of the admitted edges of one process.

    An egress edge has only its publisher here, an ingress edge
    only its subscriber. A topic's stub port goes only where no legacy edge,
    local or cross-federate, still needs it.
    """
    node_corr: dict[str, _NodeCorr] = {}
    legacy_pub = {(e.pub_node, e.pub_topic) for e in edges if e.edge_id is None}
    legacy_sub = {(e.sub_node, e.sub_topic) for e in edges if e.edge_id is None}
    for e in edges:
        if e.edge_id is None:
            # An edge refused before subscriber matching has no sub_callback
            # yet; its listed subscription still carries the real reason.
            cb = e.sub_callback or e.listed_sub_callback
            if e.role != "egress" and cb:
                sc = node_corr.setdefault(e.sub_node, _NodeCorr())
                sc.legacy_reasons.setdefault(cb, e.reason or "legacy")
            elif e.role != "egress" and e.sub_topic:
                sc = node_corr.setdefault(e.sub_node, _NodeCorr())
                sc.legacy_topic_reasons.setdefault(e.sub_topic, e.reason or "legacy")
            continue
        if e.role != "ingress":
            pc = node_corr.setdefault(e.pub_node, _NodeCorr())
            for cb_name in e.pub_callbacks:
                pc.pub_edges.setdefault(cb_name, []).append(e.edge_id)
            if (e.pub_node, e.pub_topic) not in legacy_pub:
                pc.dropped_pub_topics.add(e.pub_topic)
            if e.role == "egress":
                pc.remote.add(e.edge_id)
                if not e.xfed_flush:
                    pc.zero.add(e.edge_id)
                pc.egress_close.setdefault(e.pub_callbacks[0], []).append(e.edge_id)
        if e.role != "egress":
            sc = node_corr.setdefault(e.sub_node, _NodeCorr())
            members = sc.sub_members.setdefault(e.sub_callback, [])
            members.append(e.edge_id)
            members.sort()
            sc.sub_edges[e.sub_callback] = members[0]
            sc.sub_delays.setdefault(e.sub_callback, set()).add(e.delay_ns or 0)
            if (e.sub_node, e.sub_topic) not in legacy_sub:
                sc.dropped_sub_topics.add(e.sub_topic)
            if e.role == "ingress":
                sc.remote.add(e.edge_id)
            if e.pub_node == e.sub_node and e.role == "local":
                sc.local.add(e.edge_id)
            e.same_tag = corr_same_tag(e)
            if e.same_tag:
                sc.same_tag.add(e.edge_id)
    return node_corr


def local_split_drain() -> bool:
    """Escape to the earlier in-process receiver, kept for one release.

    ``RCLLF_LOCAL_SPLIT_DRAIN=1`` at generation time gives every in-process
    d = 0 member its own arrive reaction that wakes the drain one microstep
    later, (t, m + 1).
    """
    return os.environ.get("RCLLF_LOCAL_SPLIT_DRAIN", "") == "1"


def corr_same_tag(e: CorrEdge) -> bool:
    """The first record of *e* is dispatched at its publication tag.

    An admitted edge with both ends in this process on two nodes, d = 0 and an
    immediate join. A self edge has no port, a deferred join
    releases at t + T and d > 0 at (t + d, 0): those keep the
    drain wake. A cross-federate edge has no arrive either way: its drain
    takes the link's port as a trigger, at (t, m) over a plain
    centralized d = 0 link or (t, m + 1) over a decentralized
    `after 0`.
    Fan-in admission forces one delay, one locality and no self
    member, so a group is same-tag as a unit.
    """
    return (
        e.edge_id is not None and e.role == "local" and e.pub_node != e.sub_node
        and (e.delay_ns or 0) == 0 and not e.deferred and not local_split_drain()
    )


def _feeds_topic(
    node: NodeInfo | None, sub_callback: str | None, topic: str, timers_first: bool,
) -> bool:
    """Whether a delivery to *sub_callback* reaches *node*'s *topic* output at its tag.

    LF makes a reaction depend on the reactions declared
    before it in its reactor (``_build_reactions``, ``reaction_order``), so an
    input that triggers the subscription reaction reaches an output port at
    the same tag exactly when that reaction, or one declared after it in the
    same reactor, writes the port. That is lfc's causality graph. A callback
    that only stores (a latch) declared after every reaction that publishes
    *topic* does not reach it: the stored message is read by a later tag.

    Callback groups split a node over sub-reactors (``_callback_reactor``) and
    lfc adds no dependency between two reactors, so only the subscription's
    own sub-reactor counts. A node with message_filters inputs gets extra BNA
    and sync reactions and is treated as feeding through, as is an unknown
    node or callback.
    """
    if node is None or not sub_callback:
        return True
    mf = node.message_filters
    if mf is not None and (mf.synchronizers or mf.unsupported):
        return True
    reacting = [cb for cb in node.callbacks if _callback_reaction_kind(node, cb) is not None]
    timer_rank = 0 if timers_first else 1

    def rank(cb: CallbackInfo) -> int:
        return timer_rank if _callback_reaction_kind(node, cb) == "timer" else 1 - timer_rank

    ordered = sorted(reacting, key=rank)  # stable, as _build_reactions
    names = [cb.name for cb in ordered]
    if sub_callback not in names:
        return True
    home = _callback_reactor(node, sub_callback)
    return any(
        topic in cb.publishers_used and _callback_reactor(node, cb.name) == home
        for cb in ordered[names.index(sub_callback):]
    )


def _causality_loops(
    edges: list[CorrEdge], nodes: dict[str, NodeInfo],
    reaction_order: dict[str, str] | None, *, zero_cross: bool = False,
) -> list[list[CorrEdge]]:
    """The causality loops among the same-tag edges, each a sorted edge list.

    A same-tag edge (``corr_same_tag``) delivers at its
    publication tag: its port triggers the reaction that runs the subscriber's
    callback. Edge *a* leads into edge *b* when *b* leaves *a*'s subscriber
    and *a*'s delivery reaches *b*'s topic at the same tag (``_feeds_topic``).
    A cycle of that relation is a causality loop, a reaction cycle lfc rejects
    ("Cyclic dependency"). Every other edge breaks a loop: d > 0, a deferred
    join, a legacy link or a decentralized cross-federate link (an LF
    ``after`` connection) and an edge with a split arrive (declared last, its
    only effect is an action). With *zero_cross* (centralized coordination)
    an admitted d = 0 cross-federate edge with an immediate join
    is a plain zero-delay connection whose port triggers the subscriber's
    drain at the publication tag, so it is a candidate too: a loop is then
    refused on every split exactly when it is refused in one process.
    The result is the strongly connected components of more than one edge.
    """
    cand = [e for e in edges if corr_same_tag(e) or (
        zero_cross and e.role == "cross" and e.edge_id is not None
        and (e.delay_ns or 0) == 0 and not e.deferred)]
    succ: dict[int, list[int]] = {}
    for i, a in enumerate(cand):
        node = nodes.get(a.sub_node)
        tf = _timers_first(reaction_order, a.sub_node)
        # ``publishers_used`` holds node-local names: compare *b*'s publisher
        # name as the node declared it, not the namespace- and remap-resolved
        # ``b.topic``, or a relative or remapped publisher never matches.
        succ[i] = [j for j, b in enumerate(cand)
                   if b.pub_node == a.sub_node
                   and _feeds_topic(node, a.sub_callback, b.pub_topic or b.topic, tf)]
    # Tarjan's strongly connected components, iterative.
    index: dict[int, int] = {}
    low: dict[int, int] = {}
    on_stack: set[int] = set()
    stack: list[int] = []
    comps: list[list[int]] = []
    counter = 0
    for root in range(len(cand)):
        if root in index:
            continue
        work = [(root, 0)]
        while work:
            v, k = work.pop()
            if k == 0:
                index[v] = low[v] = counter
                counter += 1
                stack.append(v)
                on_stack.add(v)
            if k < len(succ[v]):
                work.append((v, k + 1))
                w = succ[v][k]
                if w not in index:
                    work.append((w, 0))
                elif w in on_stack:
                    low[v] = min(low[v], index[w])
                continue
            if low[v] == index[v]:
                comp = []
                while True:
                    w = stack.pop()
                    on_stack.discard(w)
                    comp.append(w)
                    if w == v:
                        break
                comps.append(comp)
            if work:
                parent = work[-1][0]
                low[parent] = min(low[parent], low[v])
    loops = [sorted((cand[i] for i in comp), key=lambda e: (e.topic, e.pub_node, e.sub_node))
             for comp in comps if len(comp) > 1]
    return sorted(loops, key=lambda loop: [(e.topic, e.pub_node, e.sub_node) for e in loop])


def _ms(ns: int) -> str:
    return f"{ns / 1_000_000:g} ms"


def _warn_cross_federate_join_spill(edges: list[CorrEdge], xfed: XfedContext) -> None:
    """Warn when a blocking join's stall can push the other direction past its cutoffs.

    A blocking join on an admitted cross-federate edge x waits for an absent
    sample until its cutoff, and while it waits it holds every reaction of
    its subscriber's federate F. A publication of F at the release tag t of x
    then leaves F physically at t + T_x or later, while the cutoff of an
    admitted edge y that F publishes on counts from the tag: t + d_y + T_y.
    So a loss on x turns y's samples ``NOT_READY`` and
    ``publisher_late`` whenever T_x > T_y + d_y. This extends the
    loop-slack rule (``cycle_slack_below_timeout``) to cross-federate loops:
    the check walks the admitted cross-federate edges as a federate graph and
    looks at every y leaving F whose subscriber federate reaches x's publisher
    federate again, so x and y lie on one cycle through the federation link.
    A deferred join never holds its federate and is skipped. A warning, not a
    refusal: a config may accept the spill.
    """
    cross = [e for e in edges if e.status == "admitted" and e.role == "cross"]
    fed = xfed.federate
    succ: dict[str | None, set[str | None]] = {}
    for e in cross:
        succ.setdefault(fed(e.pub_node), set()).add(fed(e.sub_node))

    def reaches(src: str | None, dst: str | None) -> bool:
        seen, todo = {src}, [src]
        while todo:
            f = todo.pop()
            if f == dst:
                return True
            for g in succ.get(f, ()):
                if g not in seen:
                    seen.add(g)
                    todo.append(g)
        return False

    for x in sorted(cross, key=lambda e: (e.topic, e.pub_node, e.sub_node)):
        if x.deferred:
            continue
        held, back_to = fed(x.sub_node), fed(x.pub_node)
        late = sorted(
            (y for y in cross
             if fed(y.pub_node) == held and reaches(fed(y.sub_node), back_to)
             and x.timeout_ns > y.timeout_ns + max(0, y.delay_ns or 0)),
            key=lambda e: (e.timeout_ns + max(0, e.delay_ns or 0), e.topic, e.pub_node, e.sub_node))
        if not late:
            continue
        ys = ", ".join(f"{y.label()} T = {_ms(y.timeout_ns)} + d = {_ms(max(0, y.delay_ns or 0))}"
                       for y in late)
        warnings.warn(
            f"federation: a loss on blocking edge {x.label()} holds federate {held} for its "
            f"T = {_ms(x.timeout_ns)}, longer than the slack T + d of the admitted edge(s) "
            f"{held} publishes on in the other direction of the cycle through federate "
            f"{back_to}: {ys}. Their samples published during the stall leave {held} after "
            f"their cutoffs (NOT_READY, publisher_late). Rule: on an admitted cycle through a "
            f"federation link, the T of a blocking edge is at most the smallest T + d of the "
            f"admitted edges leaving its subscriber's federate; raise those maxwait_ms "
            f"to at least {_ms(x.timeout_ns)} or lower this one",
            UserWarning, stacklevel=2,
        )


def _refuse_causality_loops(
    edges: list[CorrEdge], nodes: dict[str, NodeInfo],
    reaction_order: dict[str, str] | None, *, zero_cross: bool = False,
) -> None:
    """Fail generation on a zero-delay causality loop, as lfc does.

    Owner ruling 2026-11-05, after Edward Lee: "LF allows zero-delay feedback
    loops as long as there is no causality loop." A loop on which some
    subscription does not publish onward into the loop at its tag (it only
    stores) is admitted and every edge delivers at the publication tag. A
    loop on which every one does (``_causality_loops``) is refused here, with
    the links named. No implicit microstep: the user breaks the loop with a
    logical delay on one link. The execution config has no per-link knob for
    an explicit microstep delay (LF ``after 0``), so the message names d > 0.
    """
    loops = _causality_loops(edges, nodes, reaction_order, zero_cross=zero_cross)
    if not loops:
        return
    parts = []
    for loop in loops:
        links = ", ".join(f"{e.topic} ({e.pub_node} -> {e.sub_node})" for e in loop)
        parts.append(f"[{links}]")
    first = loops[0][0]
    raise ValueError(
        "zero-delay causality loop: the d = 0 link(s) "
        + "; ".join(parts)
        + " form a loop on which every subscription callback publishes onward into the "
        "loop at the tag it receives, so no reaction of the loop can run first (lfc: "
        "Cyclic dependency). LF allows a zero-delay feedback loop only when it has no "
        "causality loop, and rcllf adds no implicit microstep. Give one link of the loop "
        "a logical delay d > 0 in execution.yaml, for example connections: "
        f"{first.topic}: links: \"{first.pub_node} -> {first.sub_node}\": after_delay: "
        "\"1 msec\" (with reason and source)."
    )


@dataclass
class XfedContext:
    """The federation a system-wide admission runs against (D3).

    ``node_federate`` maps a node key to its federate, ``link_after_ns`` gives
    the after of a cross-federate stub link from its ``inst.port`` ends
    (``FederationMap.after_ns_for``), ``decentralized`` is the coordination.
    """

    node_federate: dict[str, str]
    link_after_ns: Callable[[str, str, int | None], int]
    decentralized: bool
    nodes: dict[str, NodeInfo]
    key_to_instance: dict[str, str]

    def federate(self, node_key: str) -> str | None:
        return self.node_federate.get(node_key)

    def after_ns(
        self, edge: CorrEdge, sub_info: SubscriberInfo | None,
        scopes: dict[str, NodeIdentity] | None = None,
    ) -> int:
        """The after of *edge*'s federate link, keyed like ``_build_connections``."""
        return self.link_after_ns(*self.link_ports(edge, sub_info, scopes), edge.delay_ns)

    def link_ports(
        self, edge: CorrEdge, sub_info: SubscriberInfo | None,
        scopes: dict[str, NodeIdentity] | None = None,
    ) -> tuple[str, str]:
        """The ``inst.port`` ends *edge*'s after is looked up by."""
        scopes = scopes or {}
        short_topic = edge.topic.rstrip("/").rsplit("/", 1)[-1]
        pub, sub = self.nodes.get(edge.pub_node), self.nodes.get(edge.sub_node)
        pub_ports = _endpoint_port_names(
            edge.topic, pub.publishers if pub else [], scope=scopes.get(edge.pub_node),
        )
        pub_port = pub_ports[0] if pub_ports else topic_to_port_name(short_topic)
        sub_ports = _endpoint_port_names(
            edge.topic, sub.subscribers if sub else [], sub, scope=scopes.get(edge.sub_node),
        )
        sub_port = sub_ports[0] if sub_ports else _sub_port_name(sub, short_topic)
        return (
            f"{self.key_to_instance.get(edge.pub_node, edge.pub_node)}.{pub_port}",
            f"{self.key_to_instance.get(edge.sub_node, edge.sub_node)}.{sub_port}",
        )


def _bna_input_callbacks(node_info: NodeInfo | None) -> set[str]:
    """The synthetic callbacks of *node_info*'s BNA sync inputs."""
    mf = node_info.message_filters if node_info is not None else None
    return {
        bna_input_callback(sync.callback, member)
        for sync in (mf.synchronizers if mf else []) if sync.bna_params is not None
        for member in sync.inputs
    }


def _xfed_refusal(edge: CorrEdge, pub: NodeInfo, xfed: XfedContext) -> str | None:
    """Cross-federate admission on top of the in-process rules (D10).

    A decentralized edge is admitted like a centralized one, its
    late records are reported lost by the binding. A BNA input over it is admitted too; a lost record decides
    through the engine's missing report, and with ``max_stamp_lag`` declared it
    raises the input's watermark to its publish tag minus the lag.
    """
    # The flush sender sends the records of t at t + 1 ns over a
    # link of after d - 1 ns, which must stay positive (`after 0` would keep
    # the microstep), so it needs d >= 2 ns. The default sender sends at
    # the publishing tag over `after d` and admits every d >= 0 (d = 0 is a
    # plain zero-delay link landing on (t, m), or (t, m + 1) when
    # decentralized; d >= 1 ns on (t + d, k)).
    if 0 < edge.delay_ns < 2 and xfed_flush(edge.delay_ns, xfed.decentralized):
        return "xfed_flush_min"
    # The flush message carries a COMPLETE number like the default
    # send (one per time), so a lost flush message is a CONTROL_FAILURE at the
    # receiver and flush edges are admitted again (they were refused in an earlier version).
    return None


def xfed_flush(delay_ns: int, decentralized: bool) -> bool:
    """Whether a cross-federate edge sends through the t + 1 ns flush.

    Default no: the publishing reaction sends each
    microstep's records itself over ``after d``. With d = 0 they land on
    (t, m) over a plain zero-delay connection when centralized,
    on (t, m + 1) over ``after 0`` when decentralized; with d > 0 the k-th message of one time t lands
    on (t + d, k), because the receiver defers a colliding network event past
    the queued ones (reactor-c patch 0002) and, under centralized
    coordination, the RTI grants (t + d, 0) only after the sender has left
    time t, so all of them are queued first.

    The flush stays where that grant argument fails: under decentralized
    coordination a later message of time t can come after the receiver ran
    (t + d, 0) and be tardy, so the sender waits until time t is over (t + 1
    ns, the smallest later time) and sends one message per time. Escape:
    ``RCLLF_XFED_FLUSH=1`` at generation time uses the flush for every d > 0
    edge (the earlier sender), kept for one release.
    """
    if delay_ns <= 0:
        return False
    return decentralized or os.environ.get("RCLLF_XFED_FLUSH", "") == "1"


def xfed_split_drain() -> bool:
    """Escape to the earlier receiver, kept for one release.

    ``RCLLF_XFED_SPLIT_DRAIN=1`` at generation time gives every cross-federate
    member of a subscription its own arrive reaction that wakes a separate
    drain reaction one microstep later. Off by default: that path delivers a
    cross-federate record one microstep later than one process, and refuses a
    record that arrives at a timeout stop tag.
    """
    return os.environ.get("RCLLF_XFED_SPLIT_DRAIN", "") == "1"


def plan_federated_corr(
    graph: SystemGraph,
    *,
    federate_names: list[str],
    xfed: XfedContext,
    after_delay_ns: int | None,
    binding_keys: dict[str, str],
    peer_fqn: dict[str, str],
    bounds: CorrBoundsConfig | None = None,
    timer_period_overrides: dict[str, dict[str, float]] | None = None,
    link_delays: LinkDelays | None = None,
    reaction_order: dict[str, str] | None = None,
) -> tuple[list[CorrEdge], dict[str, list[CorrEdge]]]:
    """System-wide admission of a federated build, split per federate (D3).

    Returns the system table (admitted ids are the global ids) and, per
    federate, its rows in system order: ``local`` rows for edges inside it,
    ``egress``/``ingress`` rows for cross-federate edges, with the federate's
    own dense ids. *peer_fqn* maps a node key to its fully qualified name.
    """
    runtime_to_key = endpoint_key_map(graph)
    system_edges, _ = _build_corr_edges(
        graph.communication, graph.nodes,
        after_delay_ns=after_delay_ns, runtime_to_key=runtime_to_key,
        binding_keys=binding_keys, bounds=bounds, link_delays=link_delays,
        timer_period_overrides=timer_period_overrides, xfed=xfed,
        scopes=build_node_identities(graph), reaction_order=reaction_order,
    )
    per_fed: dict[str, list[CorrEdge]] = {}
    for fed in federate_names:
        rows: list[CorrEdge] = []
        next_id = 0
        for e in system_edges:
            pub_fed, sub_fed = xfed.federate(e.pub_node), xfed.federate(e.sub_node)
            if fed not in (pub_fed, sub_fed):
                continue
            if e.role != "cross":
                role, peer_fed, peer = "local", None, ""
            elif fed == pub_fed:
                role, peer_fed, peer = "egress", sub_fed, peer_fqn.get(e.sub_node, "")
            else:
                role, peer_fed, peer = "ingress", pub_fed, peer_fqn.get(e.pub_node, "")
            # The deferred join lives on the subscriber's ingress
            # row, the publisher's egress row has no join.
            row = dataclasses.replace(
                e, role=role, federate=fed, peer_federate=peer_fed, peer_node=peer,
                deferred=e.deferred and role != "egress",
                tardy_drop=xfed.decentralized and role == "ingress",
                control_stop=e.control_stop and role == "ingress",
                xfed_flush=role == "egress" and xfed_flush(e.delay_ns, xfed.decentralized),
                global_id=e.edge_id, edge_id=None, pub_rids=[], sub_rid=-1,
                pub_callbacks=list(e.pub_callbacks), bound_inputs=dict(e.bound_inputs),
            )
            if e.edge_id is not None:
                row.edge_id, next_id = next_id, next_id + 1
            rows.append(row)
        # A fan-in group lives where its subscriber is (all its
        # members are local or all ingress there, P5). Its rows name the lead
        # by this federate's id and keep the system-wide rank; an egress row
        # is no member, the publisher side has no shared drain or reader.
        local_id = {r.global_id: r.edge_id for r in rows if r.edge_id is not None}
        for r in rows:
            if r.group_lead is None:
                continue
            if r.role == "egress":
                r.group_lead, r.fan_in_rank, r.fan_in_group = None, 0, None
            else:
                r.group_lead = local_id[r.group_lead]
        per_fed[fed] = rows
    return system_edges, per_fed


class _Unsized(Exception):
    """An edge's publication count in a window has no finite bound."""

    def __init__(self, reason: str, detail: str):
        super().__init__(detail)
        self.reason = reason


# C++ field widths of CorrEdgeSpec (corr_table_api.h): the bounds are emitted as
# ``u`` literals into uint32_t, the times as ``LL`` literals into int64_t.
_UINT32_MAX = 2**32 - 1
_INT64_MAX = 2**63 - 1


def _sync_calls_per_input(node: NodeInfo, cb_name: str) -> tuple[int | None, str]:
    """``(n, why)``: synchronizer callbacks one dispatch of *cb_name* can run.

    1 for an ordinary callback. For a message_filters input callback
    ``<cb>_via_<member>`` it is the policy's bound on
    sync callbacks per ``Synchronizer::add()``, read in the deployed Jazzy
    message_filters 4.11.17 headers
    (/opt/ros/jazzy/include/message_filters/message_filters/sync_policies/):

    * ExactTime (exact_time.h:103-115, 151-173): ``add()`` stores the event
      in one timestamp tuple and calls ``checkTuple`` once, which signals at
      most that tuple: 1.
    * ApproximateTime (approximate_time.h:209-255, 505-527, 701): every
      ``publishCandidate`` signal pops one message from every input deque
      (``recoverAndDelete``), and the input not being added holds at most Q
      messages (deque plus past, kept at Q by ``add``'s overflow branch,
      :232) and gains none during the serialized call, so one ``add()``
      signals at most Q times, Q the resolved queue size.

    BoundedNearestToAnchor and an unresolved Q give None (no proven bound).
    """
    sub = next((s for s in node.subscribers if s.callback == cb_name), None)
    if sub is None or not sub.sync_callback:
        return 1, ""
    mf = node.message_filters
    sync = next((s for s in (mf.synchronizers if mf else []) if s.callback == sub.sync_callback),
                None)
    if sync is None:
        return None, f"synchronizer of {sub.sync_callback} not found"
    if sync.policy == "ExactTime":
        return 1, "ExactTime 1 sync callback per add()"
    if sync.policy == "ApproximateTime":
        if sync.queue_size is None or sync.queue_size <= 0:
            return None, (f"ApproximateTime queue size of {sync.member_variable} is not resolved "
                          f"(parameter {sync.queue_param!r})")
        return sync.queue_size, f"ApproximateTime Q={sync.queue_size} sync callbacks per add()"
    return None, f"no proven sync callback bound for policy {sync.policy}"


def _reader_depth(bounds: CorrBoundsConfig, e: CorrEdge, burst: int | None) -> int:
    """KEEP_LAST depth of *e*'s reader.

    A declared ``reader_depth`` is kept as written, so a burst above it still
    refuses the edge (``reader_depth_below_burst``). Undeclared, it is the
    registry default (8) raised to the edge group's proven one-invocation
    burst: the reader must hold one whole invocation, or
    the invocation overwrites itself before ingress takes it, and the fusion
    edge's ApproximateTime burst is Q = 9 > 8. A larger depth costs only
    reader memory.
    """
    declared = bounds.reader_depth.get((e.sub_node, e.sub_topic))
    if declared is not None:
        return declared
    return max(bounds.depth(e.sub_node, e.sub_topic), burst or 0)


class _CorrRates:
    """``N_e(W)`` of W3 design 3.4: publications on edge *e* in any window of length W.

    Timer callback with period P: ``floor(W / P) + 1`` invocations. Subscription
    callback: the sum of ``N_i(W)`` over the admitted edges feeding it, or
    ``floor(W / min_input_period) + 1`` when exec-config declares that period.
    ``N_e(W) = k_e * sum of the invocations of e's publishing callbacks``.
    """

    def __init__(self, edges, nodes, communication, bounds, runtime_to_key, overrides, scopes=None):
        self.nodes = nodes
        self.scopes = scopes or {}
        self.bounds = bounds
        self.overrides = overrides
        # Twin subscriptions give one (topic, pub, sub) several
        # edges, told apart by their subscription callback.
        self.by_key: dict[tuple[str, str, str], list[CorrEdge]] = {}
        for e in edges:
            self.by_key.setdefault((e.topic, e.pub_node, e.sub_node), []).append(e)
        self.topics = list(communication.topics)
        self.runtime_to_key = runtime_to_key
        self.inputs: dict = {}  # bound_inputs of the edge being sized
        self.record = False  # fill self.inputs only on the pass that sizes retain

    def k(self, e: CorrEdge, cb_name: str) -> tuple[int, str]:
        """``(k, source)`` of one publishing callback of *e* (W3 slice S3).

        A declared ``max_publishes_per_invocation`` wins, otherwise the
        analyzer's count of publish call sites outside any loop proves it.
        """
        declared = self.bounds.burst.get(e.pub_node)
        if declared is not None:
            return declared, "declared"
        node = self.nodes[e.pub_node]
        cb = next((c for c in node.callbacks if c.name == cb_name), None)
        proven = cb.publish_bounds.get(e.pub_topic) if cb is not None else None
        if proven is None:
            why = (cb.publish_unbounded.get(e.pub_topic) if cb is not None else None) or "not analyzed"
            raise _Unsized(
                "burst_unbounded",
                f"{e.pub_node}.{cb_name} has no proven publish bound on {e.pub_topic} ({why}) "
                f"and no max_publishes_per_invocation is declared for node '{e.pub_node}'",
            )
        factor, why = _sync_calls_per_input(node, cb_name)
        if factor is None:
            raise _Unsized(
                "burst_unbounded",
                f"{e.pub_node}.{cb_name}: {why}, and no max_publishes_per_invocation is "
                f"declared for node '{e.pub_node}'",
            )
        return proven * factor, "proven" if factor == 1 else f"proven x {why}"

    def edge(self, e: CorrEdge, window_ns: int, stack: tuple = ()) -> int:
        total = 0
        for cb in e.pub_callbacks:
            k, _ = self.k(e, cb)
            total += k * self.callback(e.pub_node, cb, window_ns, stack)
        return total

    def callback(self, node_key: str, cb: str, window_ns: int, stack: tuple) -> int:
        if (node_key, cb) in stack:
            raise _Unsized("rate_unbounded", f"callback cycle through {node_key}.{cb}")
        stack = (*stack, (node_key, cb))
        node = self.nodes[node_key]
        total = 0
        triggers = 0
        periods = []
        for t in node.timers:
            if t.callback != cb:
                continue
            triggers += 1
            period_ns = period_ms_to_ns(self.overrides.get(node_key, {}).get(cb, t.period_ms))
            if period_ns is None:
                raise _Unsized(
                    "period_unknown",
                    f"timer of {node_key}.{cb} has no known period from 1 ns to the int64 "
                    "nanosecond range",
                )
            periods.append(period_ns)
            total += window_ns // period_ns + 1
        if periods and self.record:
            # One entry per timer: two timers on one callback both count.
            self.inputs.setdefault("periods_ns", {})[f"{node_key}.{cb}"] = periods
        for sub in node.subscribers:
            if sub.callback != cb:
                continue
            triggers += 1
            total += self.subscription(node_key, sub, window_ns, stack)
        if triggers == 0:
            raise _Unsized("rate_unbounded", f"{node_key}.{cb} has no timer or subscription")
        return total

    def subscription(self, node_key: str, sub, window_ns: int, stack: tuple) -> int:
        declared = self.bounds.min_input_period_ms.get((node_key, sub.topic))
        if declared is not None:
            period_ns = period_ms_to_ns(declared)
            if period_ns is None:
                raise ValueError(
                    f"min_input_period_ms={declared!r} of {node_key}:{sub.topic} is not a "
                    f"finite period from 1 ns to the int64 nanosecond range"
                )
            if self.record:
                self.inputs.setdefault("min_input_periods_ns", {})[f"{node_key}:{sub.topic}"] = period_ns
            return window_ns // period_ns + 1
        total = 0
        fed = False
        for tc in self.topics:
            subs = {self.runtime_to_key.get(s, s) for s in tc.subscribers}
            if node_key not in subs or not _match_endpoint_infos(
                tc.topic, [sub], self.scopes.get(node_key),
            ):
                continue
            for p in tc.publishers:
                pub_key = self.runtime_to_key.get(p, p)
                candidates = self.by_key.get((tc.topic, pub_key, node_key), [])
                upstream = next(
                    (e for e in candidates if e.sub_callback == sub.callback),
                    candidates[0] if candidates else None,
                )
                if upstream is None or upstream.edge_id is None:
                    raise _Unsized(
                        "rate_unbounded",
                        f"input {tc.topic} of {node_key} is fed from outside the "
                        f"admitted graph (publisher '{pub_key}'), and no "
                        f"min_input_period_ms is declared",
                    )
                try:
                    total += self.edge(upstream, window_ns, stack)
                except _Unsized as exc:
                    raise _Unsized(
                        "rate_unbounded", f"input edge {upstream.label()} is unbounded: {exc}",
                    ) from exc
                fed = True
        if not fed:
            raise _Unsized(
                "rate_unbounded",
                f"input {sub.topic} of {node_key} has no publisher in the system, and no "
                f"min_input_period_ms is declared",
            )
        return total


def _size_corr_edges(
    edges: list[CorrEdge],
    nodes: dict[str, NodeInfo],
    communication: SystemCommunication,
    *,
    bounds: CorrBoundsConfig,
    runtime_to_key: dict[str, str],
    timer_period_overrides: dict[str, dict[str, float]],
    scopes: dict[str, NodeIdentity] | None = None,
) -> None:
    """Refuse unsizable edges, renumber, and fill the bounds (W3 design 3.3 to 3.6).

    ``burst = k_e`` (the largest k of the edge's publishing callbacks, declared
    or proven by the analyzer), ``carrier = N_e(d_e + D_e)``, ``retain = N_e(d_e + D_e + L + eps) + 1``
    with D_e = T_e on a deferred edge (released at its cutoff) and 0
    otherwise, eps = 0 on a
    local edge and T_e on a cross-federate one (the binding refuses eps >= T_e). An
    edge whose burst bound, timer period or input rate is unknown is refused
    (``burst_unbounded``, ``period_unknown``, ``rate_unbounded``), which can
    make edges downstream of it unbounded in turn, so refusal runs to a fixed
    point before anything is sized. ``T_e`` is the subscription's own
    ``take_timeout_ms``: the binding counts it from the publication's commit
    (causal windows), so a publish delayed by an upstream join keeps its
    full ``d + T_e`` and nothing is chained over the publisher's inputs. ``L``
    is ``max_worker_lag_ms`` or twice the largest timeout of an admitted
    edge, and a declared ``L`` below that timeout is a configuration error.

    ``join: deferred`` marks an edge deferred. On a self edge it
    is a configuration error, since the sample is present at the drain and the
    opt-in would do nothing. On any admitted local edge it is a
    configuration error too. A local edge is decided when its publish returns
    (the publishing thread takes the sample, ``local_now``), so its join
    never waits and has no cutoff tag to defer to. T_e of such an edge only
    sizes L and so retention (a warning says so when it is declared), and
    under ``require_admitted`` a local ``sub_rebind`` edge, which cannot be
    decided at publish, is refused (``local_sub_rebind``).
    """
    for e in edges:
        e.deferred = bounds.deferred(e.sub_node, e.sub_topic)
        e.timeout_ns = bounds.timeout_ms(e.sub_node, e.sub_topic) * 1_000_000
        e.control_stop = bounds.control_failure_stop
        if e.deferred and e.pub_node == e.sub_node:
            raise ValueError(
                f"execution config: 'join: deferred' on {e.sub_node} '{e.sub_topic}' is not "
                f"supported, edge {e.label()} is a self edge whose sample is present at the "
                f"drain, so it never waits; remove the key")
        if e.deferred and e.role != "cross" and e.edge_id is not None:
            raise ValueError(
                f"execution config: 'join: deferred' on {e.sub_node} '{e.sub_topic}' is not "
                f"supported, local edge {e.label()} is decided at publish: its sample is taken "
                f"when the publish returns, so its join never waits and has no cutoff to "
                f"defer to; remove the key, it applies to cross-federate edges only")
    rates = _CorrRates(
        edges, nodes, communication, bounds, runtime_to_key, timer_period_overrides, scopes,
    )
    # A fan-in group is admitted as a unit, so a member refused
    # here takes the others along (``fan_in_member_refused``), and a group
    # rule refuses every member with its own reason.
    groups: dict[int, list[CorrEdge]] = {}
    for e in edges:
        if e.edge_id is not None and e.fan_in_group is not None:
            groups.setdefault(e.fan_in_group, []).append(e)

    def refuse(e: CorrEdge, reason: str) -> None:
        e.status, e.reason, e.edge_id = "legacy", reason, None
        group = groups.get(e.fan_in_group, [])
        rest = [m for m in group if m.edge_id is not None]
        if rest:
            note = _fan_in_note(group, f"member {e.label()} is legacy ({reason})")
        for m in rest:
            m.status, m.reason, m.edge_id, m.note = "legacy", "fan_in_member_refused", None, note

    def refuse_group(e: CorrEdge, reason: str, why: str) -> None:
        group = groups.get(e.fan_in_group) or [e]
        for m in group:
            m.status, m.reason, m.edge_id = "legacy", reason, None
            if len(group) > 1:
                m.note = _fan_in_note(group, why)

    def group_burst(e: CorrEdge) -> int | None:
        """Sum of the members' bursts, None while a member is unsized (it is
        refused on its own turn and takes the group along)."""
        try:
            return sum(max(rates.k(m, cb)[0] for cb in m.pub_callbacks)
                       for m in groups.get(e.fan_in_group) or [e])
        except _Unsized:
            return None

    if bounds.require_admitted:
        # A rebind reader's samples wait in DDS until the
        # binding admits it, so a local sub_rebind edge cannot be decided when
        # its publish returns and keeps a cutoff. The determinism claim of
        # require_admitted needs every local edge decided at publish.
        for e in edges:
            if e.edge_id is not None and e.sub_rebind and e.role != "cross":
                refuse(e, "local_sub_rebind")

    changed = True
    while changed:
        changed = False
        for e in edges:
            if e.edge_id is None:
                continue
            try:
                n = rates.edge(e, e.delay_ns)
                reason = None if n > 0 else "carrier_unsized"
            except _Unsized as exc:
                reason = exc.reason
            # One invocation's burst must fit the reader, or it
            # overwrites itself in DDS before ingress can take it.
            # The reader of a group keeps one history for all its writers, so
            # one invocation of every member must fit together.
            burst = group_burst(e) if reason is None else None
            depth = _reader_depth(bounds, e, burst)
            if burst is not None and burst > depth:
                refuse_group(e, "reader_depth_below_burst",
                             f"the members' bursts sum to {burst} > reader_depth {depth}")
                changed = True
                continue
            # D2: every record of one time crosses in one message
            # of at most XFED_RECORD_CAP, so the bound is over all coincident
            # timers of the edge, not one reaction's burst.
            if reason is None and e.role == "cross" and rates.edge(e, 0) > XFED_RECORD_CAP:
                reason = "xfed_record_cap"
            if reason is not None:
                refuse(e, reason)
                changed = True
        if not changed:
            # A deferred drain releases a record at its cutoff,
            # d + T_e after its publication, so a deferred group's members
            # must share one T_e (known limitation: release times are not equalized).
            for group in groups.values():
                live = [m for m in group if m.edge_id is not None]
                if len(live) > 1 and live[0].deferred and len({m.timeout_ns for m in live}) > 1:
                    timeouts = ", ".join(f"{m.pub_node} {m.timeout_ns} ns" for m in live)
                    refuse_group(live[0], "fan_in_release_mismatch",
                                 f"the deferred members' take timeouts differ ({timeouts})")
                    changed = True
    admitted = [e for e in edges if e.status == "admitted"]
    for new_id, e in enumerate(admitted):
        e.edge_id = new_id
    # The lead, the group's lowest edge id, names its drain, its
    # logical action and its wake.
    for group in groups.values():
        for m in group:
            if m.edge_id is not None:
                m.group_lead = min(x.edge_id for x in group if x.edge_id is not None)
    if not admitted:
        return
    _warn_local_only_timeouts(admitted, bounds)

    max_timeout_ns = max(e.timeout_ns for e in admitted)
    if bounds.max_worker_lag_ms is None:
        lag_ns = 2 * max_timeout_ns
    else:
        lag_ns = bounds.max_worker_lag_ms * 1_000_000
        if lag_ns < max_timeout_ns:
            raise ValueError(
                f"max_worker_lag_ms={bounds.max_worker_lag_ms} is below the largest "
                f"maxwait_ms of an admitted correspondence edge "
                f"({max_timeout_ns // 1_000_000}); "
                f"a drain may wait that long past its tag"
            )
    for e in admitted:
        e.max_lag_ns = lag_ns
        ks = {f"{e.pub_node}.{cb}": rates.k(e, cb) for cb in e.pub_callbacks}
        e.burst = max(k for k, _ in ks.values())
        e.reader_depth = _reader_depth(bounds, e, group_burst(e))
        # A deferred edge releases its record at the cutoff tag
        # t + d + T, not at t + d (correspondence.cpp release_micro_locked), so
        # its carrier holds every record of the last d + T and its drain runs
        # T later. Sized from d alone, a 25 ms brake edge (d = 5 ms, T =
        # 100 ms) got carrier 1 and lost 4 of 5 samples to push_overflow.
        defer_ns = max(0, e.timeout_ns) if e.deferred else 0
        e.carrier = rates.edge(e, e.delay_ns + defer_ns)
        # A retained sample lives from its arrival to its drain. The drain of
        # t + d (t + d + T when deferred) ends L later on the receiver's clock
        # (worker_lag_exceeded fails the run past L, checked after the join
        # wait too), and the arrival is no earlier than t - eps (the sender
        # commits at S(a) >= t on its own clock, at most eps apart). eps is 0
        # in one process and on one host. On a cross-federate edge the binding
        # refuses eps >= T at startup (xfed_bound_problem), so T stands in for
        # the runtime eps.
        eps_cover_ns = max(0, e.timeout_ns) if e.role == "cross" else 0
        window_ns = e.delay_ns + defer_ns + lag_ns + eps_cover_ns
        rates.inputs, rates.record = {}, True
        e.retain = rates.edge(e, window_ns) + 1
        rates.record = False
        e.bound_inputs = {
            "k": e.burst, "d_ns": e.delay_ns, "T_ns": e.timeout_ns, "L_ns": lag_ns,
            "W_ns": window_ns, **rates.inputs,
            "k_per_callback": {cb: {"k": k, "source": src} for cb, (k, src) in ks.items()},
        }
        if e.role == "cross":
            e.bound_inputs["records_per_time"] = rates.edge(e, 0)
            e.bound_inputs["eps_cover_ns"] = eps_cover_ns
        _check_corr_widths(e)


def local_now(e: CorrEdge) -> bool:
    """Whether an admitted edge is decided when its publish returns.

    A local edge (both ends in one process, ``role`` local, never a
    whole-system ``cross`` edge or an egress/ingress row) whose reader the
    binding holds from startup. A ``sub_rebind`` edge keeps its cutoff,
    and a deferred one cannot exist (generation error). The binding derives
    the same set (``Table::set_local_now``).
    """
    return (e.status == "admitted" and e.role == "local" and not e.sub_rebind
            and not e.deferred)


def _warn_local_only_timeouts(admitted: list[CorrEdge], bounds: CorrBoundsConfig) -> None:
    """Warn on a declared ``take_timeout_ms`` that never bounds a wait.

    Every admitted edge into the subscription is local and decided at
    publish, so its join never waits and nothing has a cutoff. T_e then only
    enters the worker lag bound L (twice the largest T_e unless
    ``max_worker_lag_ms`` is declared) and through L the retention bound.
    """
    subs: dict[tuple[str, str], list[CorrEdge]] = {}
    for e in admitted:
        subs.setdefault((e.sub_node, e.sub_topic), []).append(e)
    for (node, topic), members in sorted(subs.items()):
        if (node, topic) not in bounds.take_timeout_ms or not all(map(local_now, members)):
            continue
        lag = ("enters the max_worker_lag_ms check" if bounds.max_worker_lag_ms is not None
               else "sizes the worker lag bound L = 2 x the largest maxwait_ms")
        warnings.warn(
            f"execution config: 'maxwait_ms' on {node} '{topic}' only sizes L: its "
            f"edge(s) {', '.join(m.label() for m in members)} are local and decided when the "
            f"publish returns, so no join waits for it and no cutoff counts from it; it only "
            f"{lag} and through L the retention bound",
            UserWarning, stacklevel=2,
        )


def _check_corr_widths(e: CorrEdge) -> None:
    """Refuse a bound that does not fit its CorrEdgeSpec field.

    The emitter writes the counts into uint32_t and the times into int64_t, so
    a larger value would narrow or fail to compile in the generated C++.
    """
    fields = (("delay_ns", e.delay_ns, _INT64_MAX), ("timeout_ns", e.timeout_ns, _INT64_MAX),
              ("max_lag_ns", e.max_lag_ns, _INT64_MAX), ("burst", e.burst, _UINT32_MAX),
              ("retain", e.retain, _UINT32_MAX), ("carrier", e.carrier, _UINT32_MAX),
              ("reader_depth", e.reader_depth, _UINT32_MAX))
    for name, value, limit in fields:
        if value > limit:
            raise ValueError(
                f"correspondence edge {e.label()}: {name} = {value} exceeds the C++ field "
                f"limit {limit} ({'uint32_t' if limit == _UINT32_MAX else 'int64_t'}); "
                f"lower max_publishes_per_invocation, max_worker_lag_ms or maxwait_ms, "
                f"or raise min_input_period_ms"
            )


def _build_connections(
    communication: SystemCommunication,
    node_names: set[str],
    after_delay_ns: int | None = None,
    runtime_to_key: dict[str, str] | None = None,
    key_to_instance: dict[str, str] | None = None,
    nodes: dict[str, NodeInfo] | None = None,
    topic_edges: list[CorrEdge] | None = None,
    scopes: dict[str, NodeIdentity] | None = None,
    link_delays: LinkDelays | None = None,
) -> list[LFConnection]:
    """Build main-reactor LFConnections from TopicConnections and ServiceConnections.

    *topic_edges* is ``_build_corr_edges``'s table, in this function's topic
    loop order: an admitted edge becomes a zero-delay ``corr_e<ID>`` link
    (D4) instead of its stub link. *scopes* are the nodes' launch
    identities, so a launch-remapped endpoint keeps its declared port name.
    *link_delays* gives a configured link its own
    delay and a comment line stating why.
    """
    connections: list[LFConnection] = []
    ld = link_delays or LinkDelays()
    edge_iter = iter(topic_edges) if topic_edges is not None else None
    r2k = runtime_to_key or {}
    k2i = key_to_instance or {}
    nodes = nodes or {}
    scopes = scopes or {}

    for tc in communication.topics:
        # Fallback: last path segment of the resolved topic, matching what
        # _build_ports emits when it can't be resolved from node info either
        # (see _endpoint_port_names for the real per-endpoint match).
        short_topic = tc.topic.rstrip("/").rsplit("/", 1)[-1]
        fallback_port_name = topic_to_port_name(short_topic)

        # Resolve per-link declared logical-time offset: prefer
        # TopicConnection.after_delay_ns, then the caller-supplied default,
        # then the global default. None of these are latency measurements;
        # a measured value may have informed the choice upstream.
        link_delay_ns = tc.after_delay_ns
        if link_delay_ns is None:
            link_delay_ns = after_delay_ns
        if link_delay_ns is None:
            link_delay_ns = DEFAULT_AFTER_DELAY_NS
            log.debug(
                "No declared after_delay_ns for topic %s; using default logical "
                "offset %d ns",
                tc.topic, DEFAULT_AFTER_DELAY_NS,
            )

        for pub_node in tc.publishers:
            pub_key = r2k.get(pub_node, pub_node)
            if pub_key not in node_names:
                continue
            pub_inst = k2i.get(pub_key, pub_key)
            pub_info = nodes.get(pub_key)
            pub_ports = _endpoint_port_names(
                tc.topic, pub_info.publishers if pub_info else [], scope=scopes.get(pub_key),
            )
            if len(pub_ports) > 1:
                # Several outputs into one stub input are not representable,
                # lfc takes one connection per input.
                raise ValueError(
                    f"Node {pub_key!r}: publishers {', '.join(pub_ports)} all resolve to "
                    f"{tc.topic!r}, and one stub input port cannot take several outputs"
                )
            pub_port_name = pub_ports[0] if pub_ports else fallback_port_name
            for sub_node in tc.subscribers:
                sub_key = r2k.get(sub_node, sub_node)
                if sub_key not in node_names:
                    continue
                sub_inst = k2i.get(sub_key, sub_key)
                cfg = ld.get(tc.topic, pub_key, sub_key)
                comment = cfg.comment() if cfg else None
                edge = next(edge_iter) if edge_iter is not None else None
                if edge is not None:
                    assert (edge.pub_node, edge.sub_node) == (pub_key, sub_key), edge.label()
                if edge is not None and edge.edge_id is not None:
                    if pub_key == sub_key:
                        continue  # a self edge has no link
                    port = corr_port_name(edge.edge_id)
                    connections.append(LFConnection(
                        source=f"{pub_inst}.{port}",
                        destination=f"{sub_inst}.{port}",
                        after_delay_ns=None,
                        comment=comment,
                    ))
                    continue
                sub_info = nodes.get(sub_key)
                sub_ports = _endpoint_port_names(
                    tc.topic, sub_info.subscribers if sub_info else [], sub_info,
                    scope=scopes.get(sub_key),
                ) or [_sub_port_name(sub_info, short_topic)]
                for sub_port_name in sub_ports:
                    stub_connection = LFConnection(
                        source=f"{pub_inst}.{pub_port_name}",
                        destination=f"{sub_inst}.{sub_port_name}",
                        after_delay_ns=cfg.ns if cfg else link_delay_ns,
                        topic=tc.topic,
                        link=("topic", pub_key, sub_key, tc.topic),
                        comment=comment,
                    )
                    # A node with two subscriptions on one topic is listed
                    # once per subscription, each port is linked once.
                    if stub_connection not in connections:
                        connections.append(stub_connection)

    # Service connections: client.req → server.req after delay;
    #                      server.resp → client.resp after delay.
    link_delay_ns = after_delay_ns if after_delay_ns is not None else DEFAULT_AFTER_DELAY_NS
    for sc in communication.services:
        server_key = r2k.get(sc.server, sc.server)
        if server_key not in node_names:
            continue
        server_inst = k2i.get(server_key, server_key)
        # Each side links the ports _build_ports emitted from its own declared
        # name: "process_data" for the resolved "/service_example/process_data",
        # "rebind_add" for a declared "/rebind/add".
        server_info = nodes.get(server_key)
        server_stem = _declared_stem(
            sc.service, server_info.service_servers if server_info else [],
            scopes.get(server_key), attr="service_name", node_key=server_key,
            kind="service server")
        for client_node in sc.clients:
            client_key = r2k.get(client_node, client_node)
            if client_key not in node_names:
                continue
            client_inst = k2i.get(client_key, client_key)
            client_info = nodes.get(client_key)
            # Only async clients have ports; a blocking client gets none and
            # stops generation as RCLLF-H009, so it gets no link either.
            client_infos = [c for c in (client_info.service_clients if client_info else [])
                            if c.call_style in ("async_callback", None)]
            if client_info and client_info.service_clients and not client_infos:
                continue
            client_stem = _declared_stem(
                sc.service, client_infos, scopes.get(client_key),
                attr="service_name", node_key=client_key, kind="service client")
            cfg = ld.get(sc.service, client_key, server_key)
            delay_ns = cfg.ns if cfg else link_delay_ns
            comment = cfg.comment() if cfg else None
            connections.append(LFConnection(
                source=f"{client_inst}.{service_to_request_port_name(client_stem)}",
                destination=f"{server_inst}.{service_to_request_port_name(server_stem)}",
                after_delay_ns=delay_ns,
                link=("service request", client_key, server_key, sc.service),
                comment=comment,
            ))
            connections.append(LFConnection(
                source=f"{server_inst}.{service_to_response_port_name(server_stem)}",
                destination=f"{client_inst}.{service_to_response_port_name(client_stem)}",
                after_delay_ns=delay_ns,
                link=("service response", server_key, client_key, sc.service),
                comment=comment,
            ))

    # Action connections: every channel is a counted port, see
    # ACTION_PORT_TYPE. goal_req and goal_resp keep the link delay, the other
    # channels use ACTION_ASYNC_AFTER_DELAY_NS.
    async_delay_ns = ACTION_ASYNC_AFTER_DELAY_NS
    for ac in communication.actions:
        if not ac.server:
            continue
        server_key = r2k.get(ac.server, ac.server)
        if server_key not in node_names:
            continue
        server_inst = k2i.get(server_key, server_key)
        # Each side links the ports named from its own declared action name,
        # as for services.
        server_info = nodes.get(server_key)
        srv = _declared_stem(
            ac.action, server_info.action_servers if server_info else [],
            scopes.get(server_key), attr="action_name", node_key=server_key,
            kind="action server")
        client_keys = list(dict.fromkeys(r2k.get(c, c) for c in ac.clients))
        if len(client_keys) > 1:
            # A goal, result or cancel response goes to the client that asked,
            # but the counted response ports fan out to every client, so each
            # client would wait for the others' responses and trace LOST.
            raise ValueError(
                f"action {ac.action!r} has several client nodes "
                f"({', '.join(client_keys)}). Its responses go only to the client that "
                "asked, and rcllf cannot yet route a response count to one client, so it "
                "refuses the system. Give each client node its own action."
            )
        for client_node in ac.clients:
            client_key = r2k.get(client_node, client_node)
            if client_key not in node_names:
                continue
            client_inst = k2i.get(client_key, client_key)
            client_info = nodes.get(client_key)
            cli = _declared_stem(
                ac.action, client_info.action_clients if client_info else [],
                scopes.get(client_key), attr="action_name", node_key=client_key,
                kind="action client")
            cfg = ld.get(ac.action, client_key, server_key)
            goal_delay_ns = cfg.ns if cfg else link_delay_ns
            for channel in (*_CLIENT_SENDS, *_SERVER_SENDS):
                cli_port = _ACTION_PORT_NAME[channel](cli)
                srv_port = _ACTION_PORT_NAME[channel](srv)
                src, dst = (
                    (f"{client_inst}.{cli_port}", f"{server_inst}.{srv_port}")
                    if channel in _CLIENT_SENDS
                    else (f"{server_inst}.{srv_port}", f"{client_inst}.{cli_port}")
                )
                ends = (client_key, server_key) if channel in _CLIENT_SENDS else (server_key, client_key)
                connections.append(LFConnection(
                    source=src,
                    destination=dst,
                    after_delay_ns=(
                        goal_delay_ns if channel in ("goal_req", "goal_resp")
                        else async_delay_ns
                    ),
                    link=(f"action {channel}", *ends, ac.action),
                    comment=(cfg.comment() if cfg and channel in ("goal_req", "goal_resp")
                             else None),
                ))

    return connections


def _to_pascal(snake: str) -> str:
    """Convert ``snake_case`` to ``PascalCase``."""
    return "".join(p.capitalize() for p in snake.split("_") if p)
