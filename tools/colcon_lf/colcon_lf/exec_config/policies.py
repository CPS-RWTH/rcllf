# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The execution-policy registry — the extension axis for exec-config.

Every policy key the schema understands is one ``PolicySpec`` in ``REGISTRY``.
Loader validation, reference-file emission, resolution, and ``--show`` are all
driven off this registry;
a future policy is one more ``PolicySpec`` plus runtime support, not a change
to the loader/resolver/scaffold code.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction
from typing import Callable, Literal

Scope = Literal["defaults", "nodes", "subscriptions"]

SCOPE_DEFAULTS: Scope = "defaults"
SCOPE_NODES: Scope = "nodes"
SCOPE_SUBSCRIPTIONS: Scope = "subscriptions"

ALL_SCOPES: frozenset[Scope] = frozenset({SCOPE_DEFAULTS, SCOPE_NODES, SCOPE_SUBSCRIPTIONS})

KEY_TAKE_MODE = "take_mode"
KEY_TAKE_TIMEOUT_MS = "take_timeout_ms"
# The user writes the take/join wait bound as `maxwait_ms`. The internal key, the resolved
# model field, the generated policy table and the run records keep the name
# `take_timeout_ms`, which execution.yaml still accepts as an alias of
# `maxwait_ms`. The loader folds the written name into the internal one
# (model.py, _fold_input_names), per scope and per file, so a value written
# under either name overrides one written under the other.
KEY_MAXWAIT_MS = "maxwait_ms"
# Internal registry key -> the name a user writes in execution.yaml.
INPUT_NAMES: dict[str, str] = {KEY_TAKE_TIMEOUT_MS: KEY_MAXWAIT_MS}
# Every name execution.yaml accepts for a registry key whose input name
# differs from it -> that registry key.
INPUT_ALIASES: dict[str, str] = {name: key for key, name in INPUT_NAMES.items()}


def input_name(key: str) -> str:
    """The execution.yaml name of registry key *key*."""
    return INPUT_NAMES.get(key, key)
KEY_AFTER_DELAY = "after_delay"
KEY_AUTO_TRANSITION = "auto_transition"
# Correspondence bounds (W3 design 3.3 to 3.6), generation-time only.
KEY_MAX_WORKER_LAG_MS = "max_worker_lag_ms"
KEY_MAX_PUBLISHES_PER_INVOCATION = "max_publishes_per_invocation"
KEY_MIN_INPUT_PERIOD_MS = "min_input_period_ms"
KEY_READER_DEPTH = "reader_depth"
# Deferred join of an admitted correspondence edge, generation-time only.
KEY_JOIN = "join"
# One-scheduler ingress, generation-time only.
KEY_INGRESS_CAPACITY = "ingress_capacity"
# BNA synchronizer input stamps, generation-time only.
KEY_STAMP_PROVENANCE = "stamp_provenance"
STAMP_PROVENANCE_LF_CLOCK = "lf_clock"
STAMP_PROVENANCE_DECLARED_PREFIX = "declared:"
# Declared bound of (publish tag - header stamp) of an lf_clock BNA
# input; a lost message raises the watermark to its publish tag minus it.
KEY_MAX_STAMP_LAG = "max_stamp_lag"
# Every topic edge must be admitted, else generation fails,
# generation-time only.
KEY_REQUIRE_ADMITTED = "require_admitted"
# What a missing COMPLETE record on a cross-federate edge does,
# generation-time only.
KEY_CONTROL_FAILURE_POLICY = "control_failure_policy"
# The generated program's LF worker count, generation-time only.
KEY_WORKERS = "workers"
CONTROL_FAILURE_REPORT_CONTINUE = "report_continue"
CONTROL_FAILURE_STOP = "stop"
# Declaration order of a reactor's timer and subscription reactions,
# generation-time only.
KEY_REACTION_ORDER = "reaction_order"
# Where the absence cutoff of an admitted cross-federate edge counts from,
# generation-time only. The publication tag is the only anchor; the key stays so configs that say `tag` remain valid.
KEY_CUTOFF_ANCHOR = "cutoff_anchor"
CUTOFF_ANCHOR_TAG = "tag"
CUTOFF_ANCHORS = (CUTOFF_ANCHOR_TAG,)
# The removed physical-start anchor, named only to reject it.
_REMOVED_CUTOFF_ANCHOR = "sender_start"
REACTION_ORDER_SUBSCRIPTIONS_FIRST = "subscriptions_first"
REACTION_ORDER_TIMERS_FIRST = "timers_first"
REACTION_ORDERS = (REACTION_ORDER_SUBSCRIPTIONS_FIRST, REACTION_ORDER_TIMERS_FIRST)

TAKE_MODE_SKIP = "skip"
TAKE_MODE_BLOCK = "block"

JOIN_BLOCKING = "blocking"
JOIN_DEFERRED = "deferred"

# Scope set for generation-time settings: only meaningful system-wide, so
# `defaults` is the only place they may be set (unlike take policy, which is
# per-node/per-subscription).
DEFAULTS_ONLY: frozenset[Scope] = frozenset({SCOPE_DEFAULTS})

# Scope set for generation-time settings that are a property of a node rather
# than of the system or of one subscription. `subscriptions` is excluded
# because a node with no subscriptions (the lifecycle example publishes only)
# would have nowhere to carry the setting at all.
DEFAULTS_AND_NODES: frozenset[Scope] = frozenset({SCOPE_DEFAULTS, SCOPE_NODES})


@dataclass(frozen=True)
class PolicySpec:
    """One execution-policy key: how to parse it, where it may be set, its default."""

    key: str
    scopes: frozenset[Scope]
    parse: Callable[[object], object]  # raises ValueError on bad input
    default: object
    choices: tuple[str, ...] | None  # None = not an enumerated/choice value
    doc: str  # one-line help; shown in the reference file and --show
    # False for keys consumed at generation time rather than by the C++ runtime.
    # Runtime keys are resolved into the frozen (binding_key, topic) table that
    # becomes exec_config.json; generation-time keys must stay out of it so that
    # file's shape is unchanged by their presence.
    runtime: bool = True


def _parse_require_admitted(value: object) -> bool:
    """Parse the require_admitted flag, a real YAML boolean only (as auto_transition)."""
    if isinstance(value, bool):
        return value
    raise ValueError(f"'{KEY_REQUIRE_ADMITTED}' must be true or false, got {value!r}")


def _parse_auto_transition(value: object) -> bool:
    """Parse the auto_transition flag, accepting only a real YAML boolean.

    Deliberately not truthiness: `auto_transition: "false"` is a string and
    would otherwise silently mean True, which is the wrong way round for a key
    whose whole purpose is to stop the wrapper acting on a node.
    """
    if isinstance(value, bool):
        return value
    raise ValueError(
        f"'{KEY_AUTO_TRANSITION}' must be true or false, got {value!r}"
    )


def _parse_stamp_provenance(value: object) -> str:
    if isinstance(value, str) and (
        value == STAMP_PROVENANCE_LF_CLOCK
        or (value.startswith(STAMP_PROVENANCE_DECLARED_PREFIX)
            and value[len(STAMP_PROVENANCE_DECLARED_PREFIX):].strip())
    ):
        return value
    raise ValueError(
        f"'{KEY_STAMP_PROVENANCE}' must be '{STAMP_PROVENANCE_LF_CLOCK}' or "
        f"'{STAMP_PROVENANCE_DECLARED_PREFIX}<text>', got {value!r}"
    )


def _parse_max_stamp_lag(value: object) -> str:
    """An LF time literal >= 0, kept as written."""
    from colcon_lf.generation.lf_time import parse_lf_time_ns

    if isinstance(value, bool) or value is None:
        raise ValueError(f"'{KEY_MAX_STAMP_LAG}' must be a time literal, got {value!r}")
    if isinstance(value, int):
        value = str(value) if value == 0 else f"{value} nsec"
    if not isinstance(value, str):
        raise ValueError(f"'{KEY_MAX_STAMP_LAG}' must be a time literal, got {value!r}")
    ns = parse_lf_time_ns(value)
    if ns is None or ns < 0:
        raise ValueError(f"'{KEY_MAX_STAMP_LAG}' must be a time literal >= 0, got {value!r}")
    return value.strip()


def _parse_take_mode(value: object) -> str:
    if not isinstance(value, str) or value not in (TAKE_MODE_SKIP, TAKE_MODE_BLOCK):
        raise ValueError(
            f"'{KEY_TAKE_MODE}' must be one of "
            f"{(TAKE_MODE_SKIP, TAKE_MODE_BLOCK)!r}, got {value!r}"
        )
    return value


def _parse_control_failure_policy(value: object) -> str:
    choices = (CONTROL_FAILURE_REPORT_CONTINUE, CONTROL_FAILURE_STOP)
    if not isinstance(value, str) or value not in choices:
        raise ValueError(
            f"'{KEY_CONTROL_FAILURE_POLICY}' must be one of {choices!r}, got {value!r}"
        )
    return value


def _parse_cutoff_anchor(value: object) -> str:
    if value == _REMOVED_CUTOFF_ANCHOR:
        raise ValueError(
            f"'{KEY_CUTOFF_ANCHOR}: {_REMOVED_CUTOFF_ANCHOR}' was removed: the "
            "cutoff of a cross-federate edge always counts from the publication tag, "
            "t + max(0, d) + maxwait_ms + eps (eps the declared clock bound); "
            f"delete the key or set it to '{CUTOFF_ANCHOR_TAG}'"
        )
    if not isinstance(value, str) or value not in CUTOFF_ANCHORS:
        raise ValueError(
            f"'{KEY_CUTOFF_ANCHOR}' must be one of {CUTOFF_ANCHORS!r}, got {value!r}"
        )
    return value


def _parse_join(value: object) -> str:
    if not isinstance(value, str) or value not in (JOIN_BLOCKING, JOIN_DEFERRED):
        raise ValueError(
            f"'{KEY_JOIN}' must be one of {(JOIN_BLOCKING, JOIN_DEFERRED)!r}, got {value!r}"
        )
    return value


def parse_reaction_order(value: object) -> str:
    if not isinstance(value, str) or value not in REACTION_ORDERS:
        raise ValueError(
            f"'{KEY_REACTION_ORDER}' must be one of {REACTION_ORDERS!r}, got {value!r}"
        )
    return value


def _parse_after_delay(value: object) -> str:
    """Validate an LF time literal, keeping the string form for the reference file.

    Parsed eagerly so a typo fails at config-load time rather than silently
    reaching generation as a zero delay. The canonical string is
    what round-trips into `execution.reference.yaml`; generation converts it to
    nanoseconds via `parse_lf_time_ns`.
    """
    from colcon_lf.generation.lf_time import parse_lf_time_ns

    if isinstance(value, bool) or value is None:
        raise ValueError(f"'{KEY_AFTER_DELAY}' must be a time literal, got {value!r}")
    if isinstance(value, int):
        value = str(value) if value == 0 else f"{value} nsec"
    if not isinstance(value, str):
        raise ValueError(f"'{KEY_AFTER_DELAY}' must be a time literal, got {value!r}")
    ns = parse_lf_time_ns(value)
    if ns is None:
        raise ValueError(f"'{KEY_AFTER_DELAY}' must not be empty")
    return value.strip()


def _parse_timer_time(key: str, value: object) -> tuple[str, int]:
    """Validate one per-timer LF time literal, returning (literal, ns)."""
    from colcon_lf.generation.lf_time import parse_lf_time_ns

    if isinstance(value, bool) or value is None:
        raise ValueError(f"'{key}' must be a time literal, got {value!r}")
    if isinstance(value, int):
        if value < 0:
            raise ValueError(f"'{key}' must not be negative, got {value!r}")
        value = str(value) if value == 0 else f"{value} nsec"
    if not isinstance(value, str):
        raise ValueError(f"'{key}' must be a time literal, got {value!r}")
    if value.strip().startswith("-"):
        raise ValueError(f"'{key}' must not be negative, got {value!r}")
    ns = parse_lf_time_ns(value)
    if ns is None:
        raise ValueError(f"'{key}' must not be empty")
    return value.strip(), ns


def parse_timer_period(value: object) -> str:
    """Validate a timer-period override's LF time literal (P3).

    Same grammar and eager-validation style as ``_parse_after_delay``: fails at
    config-load time on a typo rather than silently reaching generation. Used
    for ``nodes.<node>.timers.<timer>.period`` in ``execution.yaml``, which is
    not itself a ``PolicySpec`` (it nests one level deeper, per timer, not
    per node), so it is validated directly by ``exec_config/model.py`` rather
    than through ``REGISTRY``. A zero period is refused here, not later in the
    correspondence bounds.
    """
    literal, ns = _parse_timer_time("period", value)
    if ns == 0:
        raise ValueError(f"'period' must be > 0, got {value!r}")
    return literal


def parse_timer_offset(value: object) -> str:
    """Validate a timer start offset, ``nodes.<node>.timers.<timer>.offset``.
    Any value >= 0 is valid, also one longer than the period: LF
    fires the timer first at the offset, then every period."""
    return _parse_timer_time("offset", value)[0]


def _parse_take_timeout_ms(value: object) -> int:
    # The loader renames the key in this message to the one the file wrote.
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"'{KEY_MAXWAIT_MS}' must be an int > 0, got {value!r}")
    if value <= 0:
        raise ValueError(f"'{KEY_MAXWAIT_MS}' must be an int > 0, got {value!r}")
    return value


def _parse_positive_int(key: str) -> Callable[[object], int]:
    """Parser for an int > 0 that names *key* in its error."""
    def parse(value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"'{key}' must be an int > 0, got {value!r}")
        return value
    return parse


_INT64_MAX_NS = 2**63 - 1


def period_ms_to_ns(value: object) -> int | None:
    """A period in milliseconds as whole nanoseconds, rounded down, or None.

    None for a bool, a non-number, a non-finite value, a value below 1 ns or one
    above the int64 nanosecond range. The check runs on the exact decimal value
    before any conversion, and the result is floored, never rounded up: a longer
    period would undercount publications and undersize the bounds.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return None
    # repr() is the decimal the user wrote, so 0.000001 is exactly 1 ns.
    ns = Fraction(repr(value)) * 1_000_000
    if ns < 1 or ns > _INT64_MAX_NS:
        return None
    return math.floor(ns)


def _parse_min_input_period_ms(value: object) -> float:
    # The generator divides by the period in whole nanoseconds.
    if period_ms_to_ns(value) is None:
        raise ValueError(
            f"'{KEY_MIN_INPUT_PERIOD_MS}' must be a finite number from 0.000001 (1 ns) "
            f"to {_INT64_MAX_NS // 1_000_000} ms, got {value!r}"
        )
    return value


# ---------------------------------------------------------------------------
# The v1 registry: take_mode, take_timeout_ms (written maxwait_ms)
# (00_design.md section 2.3).
# ---------------------------------------------------------------------------

REGISTRY: dict[str, PolicySpec] = {
    KEY_TAKE_MODE: PolicySpec(
        key=KEY_TAKE_MODE,
        scopes=ALL_SCOPES,
        parse=_parse_take_mode,
        default=TAKE_MODE_SKIP,
        choices=(TAKE_MODE_SKIP, TAKE_MODE_BLOCK),
        doc=(
            "How a reaction handles a subscription with no new message at the "
            "current tag: both modes poll up to maxwait_ms for a sample the "
            "trigger counted; when the write count is unknown "
            "'skip' drops at once (default) and 'block' still polls up to "
            "maxwait_ms before giving up. Orthogonal to tag semantics — "
            "block never fabricates or re-delivers data, it only delays the "
            "skip decision. Per-subscription block CAN stall other reactions "
            "at the same tag for up to maxwait_ms; a footgun when set "
            "non-uniformly across subscriptions."
        ),
    ),
    KEY_TAKE_TIMEOUT_MS: PolicySpec(
        key=KEY_TAKE_TIMEOUT_MS,
        scopes=ALL_SCOPES,
        parse=_parse_take_timeout_ms,
        default=100,
        choices=None,
        doc=(
            "Milliseconds a take polls for a message its trigger counted, in "
            "'skip' and 'block' mode alike, and for any message in 'block' mode "
            "(still drops on timeout — no data is fabricated). An admitted "
            "correspondence edge's join waits up to it as well. Inert only on an "
            "external_topics subscription, whose message is already taken when "
            "the reaction runs. The take gives up at publish tag + after delay + "
            "timeout, so the timeout is a budget of its own beside the after "
            "delay, not bounded by it. Written 'maxwait_ms'; "
            "'take_timeout_ms' is accepted as its alias."
        ),
    ),
    KEY_AFTER_DELAY: PolicySpec(
        key=KEY_AFTER_DELAY,
        scopes=DEFAULTS_ONLY,
        parse=_parse_after_delay,
        # None = "not configured here". Leaving this unset must not override the
        # --after-delay flag, so the resolver can distinguish absent from set.
        default=None,
        choices=None,
        doc=(
            "Default declared logical-time offset (LF 'after') on inter-node "
            "connections, as an LF time literal such as '1 msec'. This is a "
            "LET-style contract, not a latency measurement. Generation-time "
            "only, so it may be set under 'defaults' but not per-node or "
            "per-subscription; a per-link value in system.json and an explicit "
            "--after-delay flag both take precedence over it."
        ),
        runtime=False,
    ),
    KEY_AUTO_TRANSITION: PolicySpec(
        key=KEY_AUTO_TRANSITION,
        scopes=DEFAULTS_AND_NODES,
        parse=_parse_auto_transition,
        default=True,
        choices=None,
        doc=(
            "Whether the wrapper drives a managed lifecycle node to 'active' "
            "at startup, before binding. True (default) suits a node the "
            "wrapper owns. Set false for a node whose activation belongs to a "
            "supervisor, state machine or operator: no transition is driven "
            "and binding sees only the entities that exist when the startup "
            "reaction runs, so a node that activates later binds nothing. "
            "Inert on a plain rclcpp::Node, which has no managed "
            "lifecycle. Generation-time only, so it may be set under "
            "'defaults' or per node, but not per subscription."
        ),
        runtime=False,
    ),
    KEY_MAX_WORKER_LAG_MS: PolicySpec(
        key=KEY_MAX_WORKER_LAG_MS,
        scopes=DEFAULTS_ONLY,
        parse=_parse_positive_int(KEY_MAX_WORKER_LAG_MS),
        # None = 2 * the largest take_timeout_ms over the admitted edges.
        default=None,
        choices=None,
        doc=(
            "Declared bound L on how far (physical minus logical time) a "
            "correspondence drain may run behind its tag. The binding fails "
            "the run when a drain exceeds it. It sizes each admitted edge's "
            "retention, so it must be at least the largest maxwait_ms of "
            "an admitted edge. Unset means twice that largest timeout. "
            "Generation-time only, 'defaults' scope only."
        ),
        runtime=False,
    ),
    KEY_MAX_PUBLISHES_PER_INVOCATION: PolicySpec(
        key=KEY_MAX_PUBLISHES_PER_INVOCATION,
        scopes=DEFAULTS_AND_NODES,
        parse=_parse_positive_int(KEY_MAX_PUBLISHES_PER_INVOCATION),
        # None = undeclared: the generator uses the publish count the analyzer
        # proves, and the edge stays legacy (burst_unbounded) only when there
        # is no proof either.
        default=None,
        choices=None,
        doc=(
            "Declared burst bound k: the most publishes one invocation of a "
            "node's publishing callback makes on one topic. It overrides the "
            "bound the analyzer proves; a topic edge is refused only when "
            "neither this key nor the analyzer gives k, and the binding "
            "fails the run when a callback publishes more. Generation-time "
            "only, 'defaults' or per node."
        ),
        runtime=False,
    ),
    KEY_MIN_INPUT_PERIOD_MS: PolicySpec(
        key=KEY_MIN_INPUT_PERIOD_MS,
        scopes=ALL_SCOPES,
        parse=_parse_min_input_period_ms,
        # None = undeclared: an input fed from outside the admitted graph
        # leaves the publishing edges downstream of it legacy (rate_unbounded).
        default=None,
        choices=None,
        doc=(
            "Declared smallest spacing, in milliseconds, between two messages "
            "on a subscription. Needed only when the subscription is fed by a "
            "legacy edge or an external publisher and its callback publishes "
            "on an admitted edge, whose rate it then bounds. Generation-time "
            "only."
        ),
        runtime=False,
    ),
    KEY_READER_DEPTH: PolicySpec(
        key=KEY_READER_DEPTH,
        scopes=ALL_SCOPES,
        parse=_parse_positive_int(KEY_READER_DEPTH),
        default=8,
        choices=None,
        doc=(
            "KEEP_LAST depth the binding gives the reader of an admitted "
            "correspondence edge when the user asked for less. An edge whose "
            "max_publishes_per_invocation exceeds it stays legacy "
            "(reader_depth_below_burst). Size it to at least k times the "
            "invocations expected between two ingress takes: a sample "
            "overwritten in the reader before ingress takes it is detected "
            "and delivered missing, never prevented. Generation-time only."
        ),
        runtime=False,
    ),
    KEY_INGRESS_CAPACITY: PolicySpec(
        key=KEY_INGRESS_CAPACITY,
        scopes=DEFAULTS_AND_NODES,
        parse=_parse_positive_int(KEY_INGRESS_CAPACITY),
        default=8,
        choices=None,
        doc=(
            "How many infrastructure service requests (parameters, lifecycle, "
            "type description) the binding's ingress thread may hold STORED "
            "for one node while they wait for an LF worker. A request that "
            "arrives when the node already holds this many is refused at once "
            "(REJECTED overflow, logged as a warning, and a mutating request "
            "gets a failure reply). Generation-time only, 'defaults' or per "
            "node."
        ),
        runtime=False,
    ),
    KEY_STAMP_PROVENANCE: PolicySpec(
        key=KEY_STAMP_PROVENANCE,
        scopes=frozenset({SCOPE_SUBSCRIPTIONS}),
        parse=_parse_stamp_provenance,
        default=None,
        choices=None,
        doc=(
            "Where the header stamps of a BoundedNearestToAnchor synchronizer "
            "input come from: 'lf_clock' (the publisher stamps with the logical "
            "clock) or 'declared:<text>' (a source the user vouches for, such as "
            "a sensor driver). Required on every BNA input, generation refuses "
            "without it. rcllf does not check the stamps, the declaration is "
            "recorded in the resolved model. Generation-time only."
        ),
        runtime=False,
    ),
    KEY_MAX_STAMP_LAG: PolicySpec(
        key=KEY_MAX_STAMP_LAG,
        scopes=frozenset({SCOPE_SUBSCRIPTIONS}),
        parse=_parse_max_stamp_lag,
        default=None,
        choices=None,
        doc=(
            "Largest gap between the publish tag of a message on a "
            "BoundedNearestToAnchor input and its header stamp, for an input "
            "with stamp_provenance 'lf_clock'. '0 ms' when the publisher stamps "
            "with now() in the reaction that publishes, more for a stamp taken "
            "in an earlier reaction or forwarded by a relay. With it a lost "
            "message raises the synchronizer's watermark to its publish tag "
            "minus this lag, so pending anchors decide at the loss report. "
            "Undeclared, a lost message does not move the watermark. rcllf "
            "does not check the stamps. Generation-time only."
        ),
        runtime=False,
    ),
    KEY_JOIN: PolicySpec(
        key=KEY_JOIN,
        scopes=ALL_SCOPES,
        parse=_parse_join,
        default=JOIN_BLOCKING,
        choices=(JOIN_BLOCKING, JOIN_DEFERRED),
        doc=(
            "How the drain of an admitted correspondence edge joins its "
            "sample. 'blocking' (default) dispatches at the release tag, "
            "publish tag + d, and the LF worker waits there for an absent "
            "sample up to its cutoff, which holds every later reaction of the "
            "LF program. 'deferred' dispatches at the cutoff tag, publish "
            "tag + d + maxwait_ms (chained), so the worker never waits for "
            "the sample. The dispatch tag is fixed, the latency grows by the "
            "timeout, and only a sample taken in strictly before the cutoff is "
            "delivered. A self edge stays blocking. Generation-time only."
        ),
        runtime=False,
    ),
    KEY_CUTOFF_ANCHOR: PolicySpec(
        key=KEY_CUTOFF_ANCHOR,
        scopes=ALL_SCOPES,
        parse=_parse_cutoff_anchor,
        default=CUTOFF_ANCHOR_TAG,
        choices=CUTOFF_ANCHORS,
        doc=(
            "Where the absence cutoff of an admitted cross-federate edge counts "
            "from. 'tag' (default and only value) counts from the publication's "
            "logical tag t: the cutoff is t + max(0, d) + maxwait_ms + eps, "
            "eps the declared clock bound across hosts, the maxwait rule of "
            "Paladino et al. (Eq. 1b). One late input then delays the receiver "
            "to its cutoff, so the next hop's timeout must cover it (timeouts "
            "grow per hop). d is the link's existing after_delay. An in-process "
            "edge has no such cutoff. 'sender_start' (the physical start of the "
            "publishing activation) was removed and is an error. "
            "Generation-time only."
        ),
        runtime=False,
    ),
    KEY_REQUIRE_ADMITTED: PolicySpec(
        key=KEY_REQUIRE_ADMITTED,
        scopes=DEFAULTS_ONLY,
        parse=_parse_require_admitted,
        default=False,
        choices=None,
        doc=(
            "Whether every topic edge of the system must be admitted to "
            "publication correspondence. False (default) keeps a refused edge "
            "on the legacy path with its reason. True makes any refused edge a "
            "generation error that names each edge and reason, for a system "
            "whose determinism claim needs every edge. "
            "Generation-time only, 'defaults' only."
        ),
        runtime=False,
    ),
    KEY_WORKERS: PolicySpec(
        key=KEY_WORKERS,
        scopes=DEFAULTS_ONLY,
        parse=_parse_positive_int(KEY_WORKERS),
        # None = the reactor-c default, min(host cores, reaction graph breadth).
        default=None,
        choices=None,
        doc=(
            "Number of LF worker threads of the generated program (the LF "
            "'workers' target property). Unset keeps the reactor-c default, "
            "the smaller of the host's core count and the reaction graph's "
            "breadth, so the worker count, and with it the cost of handing a "
            "tag's reactions from worker to worker, depends on the host. Set it "
            "for a fast tag loop whose reactions are short, where those handoffs "
            "can put the program behind physical time. "
            "Generation-time only, 'defaults' only."
        ),
        runtime=False,
    ),
    KEY_CONTROL_FAILURE_POLICY: PolicySpec(
        key=KEY_CONTROL_FAILURE_POLICY,
        scopes=DEFAULTS_ONLY,
        parse=_parse_control_failure_policy,
        default=CONTROL_FAILURE_REPORT_CONTINUE,
        choices=(CONTROL_FAILURE_REPORT_CONTINUE, CONTROL_FAILURE_STOP),
        doc=(
            "What the subscriber of a cross-federate edge does when a sender "
            "activation's COMPLETE record never arrives (CONTROL_FAILURE: the "
            "receiver cannot tell what that activation published). "
            "'report_continue' (default) logs and traces it and the run "
            "reconcile fails, the program keeps running. 'stop' also requests "
            "an LF stop at once. Generation-time only, 'defaults' only."
        ),
        runtime=False,
    ),
    KEY_REACTION_ORDER: PolicySpec(
        key=KEY_REACTION_ORDER,
        scopes=DEFAULTS_AND_NODES,
        parse=parse_reaction_order,
        default=REACTION_ORDER_TIMERS_FIRST,
        choices=REACTION_ORDERS,
        doc=(
            "Declaration order of a node's timer and subscription reactions in "
            "its reactor. LF runs reactions of one reactor at one tag in "
            "declaration order only when no connection orders them, so this "
            "decides which callback runs first when a timer and a subscription "
            "of the node fire at the same tag. 'timers_first' (default) "
            "declares timer reactions first, 'subscriptions_first' "
            "subscription reactions first. Source order holds within each "
            "group. A timer "
            "callback that sends a service request or action goal stays with "
            "the subscriptions. Generation-time only, 'defaults' or per node."
        ),
        runtime=False,
    ),
}

# Keys the C++ runtime consumes, in registry order. The resolver and
# exec_config.json emission are driven off this, not off REGISTRY directly.
RUNTIME_REGISTRY: dict[str, PolicySpec] = {
    key: spec for key, spec in REGISTRY.items() if spec.runtime
}
