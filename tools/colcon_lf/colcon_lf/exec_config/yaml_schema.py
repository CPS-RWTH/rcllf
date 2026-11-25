# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Constants for the execution-config YAML schema (``execution.yaml``).

Schema (v1)::

    schema_version: 1
    defaults:
      take_mode: skip
      maxwait_ms: 100            # alias: take_timeout_ms
    nodes:
      /robot_1/subscriber_node:
        maxwait_ms: 250
        timers:
          timer_callback:
            period: "5 msec"        # optional, the period the code creates it with
            offset: "2 msec"        # optional, first fire
            offset_runtime: true    # optional, offset settable at start
    subscriptions:
      /robot_1/subscriber_node:
        sensor_data:
          take_mode: block
    acknowledged_hazards: []
    external_topics:
      - /robot_1/cmd_in
    connections:                    # see exec_config/connections.py
      /robot_1/sensor_data:
        after_delay: "2 msec"
        reason: measured
        source: "run <dir>: publisher callback max 1.6 ms"
    publishers:                     # see KEY_PUBLISHERS below
      /robot_1/controller_node:
        /robot_1/cmd:
          output_deadline_ns: 20000000
          basis: "where the bound comes from"
"""

# Top-level keys
KEY_SCHEMA_VERSION = "schema_version"
KEY_DEFAULTS = "defaults"
KEY_NODES = "nodes"
KEY_SUBSCRIPTIONS = "subscriptions"
KEY_ACKNOWLEDGED_HAZARDS = "acknowledged_hazards"
# Topics fed from outside the generated system: each one's
# subscriptions are driven by an LF physical action tagged at arrival.
KEY_EXTERNAL_TOPICS = "external_topics"
# An `external_topics` entry may be a mapping
# `{topic, anchor: stamp, delay_ns, ingress_reserve_ns, grid_ns, basis}`.
# The message's tag is then
# `start + ceil(header.stamp + delay_ns + ingress_reserve_ns - start, grid_ns)`
# instead of its arrival time, and a message arriving after
# `header.stamp + delay_ns` is late. `basis` is the written justification of
# the constants (house rule on constants, plan §8 Q5): no silent default.
KEY_EXT_TOPIC = "topic"
KEY_EXT_ANCHOR = "anchor"
KEY_EXT_DELAY_NS = "delay_ns"
KEY_EXT_RESERVE_NS = "ingress_reserve_ns"
KEY_EXT_GRID_NS = "grid_ns"
KEY_EXT_BASIS = "basis"
EXT_ANCHOR_ARRIVAL = "arrival"
EXT_ANCHOR_STAMP = "stamp"
EXT_ANCHORS = (EXT_ANCHOR_ARRIVAL, EXT_ANCHOR_STAMP)
# Largest delay_ns, ingress_reserve_ns or grid_ns (2**60 ns, about 36 years):
# with header.stamp below 2**31 s and the start tag below 2**61 ns, every step
# of the binding's int64 tag arithmetic then fits. The binding refuses the
# same bound at startup (kAnchorMaxNs, external_input.hpp).
EXT_ANCHOR_MAX_NS = 2**60
EXT_ENTRY_KEYS = frozenset({
    KEY_EXT_TOPIC, KEY_EXT_ANCHOR, KEY_EXT_DELAY_NS, KEY_EXT_RESERVE_NS,
    KEY_EXT_GRID_NS, KEY_EXT_BASIS,
})
# Per-connection after-delays with a stated reason.
KEY_CONNECTIONS = "connections"
# Per publisher
# `publishers: {<node>: {<topic>: {output_deadline_ns, basis}}}`. Every
# publish of <topic> from a reaction of <node> is checked just before the
# real publish: lateness = lf_time_physical() - lf_time_logical(); above
# output_deadline_ns it is logged and traced as `actuation_late` and the
# message is published anyway, unchanged. Not an LF `deadline` (a missed LF
# deadline runs the handler instead of the body and would drop the message).
# There is no default bound: `basis` states where the number comes from.
KEY_PUBLISHERS = "publishers"
KEY_PUB_OUTPUT_DEADLINE_NS = "output_deadline_ns"
KEY_PUB_BASIS = "basis"
PUB_ENTRY_KEYS = frozenset({KEY_PUB_OUTPUT_DEADLINE_NS, KEY_PUB_BASIS})
# Largest output_deadline_ns, the same 2**60 ns bound as the stamp anchor's
# constants (about 36 years): lateness and bound then compare in int64.
PUB_DEADLINE_MAX_NS = 2**60

# Reserved sub-key inside a `nodes.<node>` body:
# `{timer_name: {period: <literal>, offset: <literal>}}`, at least one key,
# (period and offset). Not a
# REGISTRY policy key — it nests one level deeper (per-timer, not per-node), so it
# is validated separately from `_validate_scope`.
KEY_TIMERS = "timers"
KEY_TIMER_PERIOD = "period"
KEY_TIMER_OFFSET = "offset"
# The offset (default ``offset`` or 0) becomes a program option,
# set per run through RCLLF_TIMER_OFFSETS in the generated launch file.
KEY_TIMER_OFFSET_RUNTIME = "offset_runtime"
TIMER_KEYS = (KEY_TIMER_PERIOD, KEY_TIMER_OFFSET, KEY_TIMER_OFFSET_RUNTIME)

# `nodes.<node>.tf2: {wait_bound: <LF time>, on_timeout:
# reject | failure | fallback:<name>}` sets the wait bound and the timeout
# outcome of every rcllf_tf2::AcquisitionLookup the node constructs, over the
# constructor literals (the yaml wins; the literal stays the stock-ROS value).
# Per node, not per member: the adapter registers without knowing its member
# name. Nested like `timers`, so validated outside `_validate_scope`.
KEY_TF2 = "tf2"
KEY_TF2_WAIT_BOUND = "wait_bound"
KEY_TF2_ON_TIMEOUT = "on_timeout"
TF2_KEYS = (KEY_TF2_WAIT_BOUND, KEY_TF2_ON_TIMEOUT)

# The only schema version this slice understands.
SCHEMA_VERSION = 1

# Valid top-level keys (unknown top-level keys are a hard ValueError).
TOP_LEVEL_KEYS = frozenset({
    KEY_SCHEMA_VERSION,
    KEY_DEFAULTS,
    KEY_NODES,
    KEY_SUBSCRIPTIONS,
    KEY_ACKNOWLEDGED_HAZARDS,
    KEY_EXTERNAL_TOPICS,
    KEY_CONNECTIONS,
    KEY_PUBLISHERS,
})
