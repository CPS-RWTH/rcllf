# Resolved Configuration Model

`colcon lf-generate` writes `resolved_model.json` next to `system.json`, alongside
`exec_config.json`. It is a versioned, typed, provenance-tagged snapshot of every
temporal-contract property a generated system actually runs with (topic names, QoS,
timer periods, parameter values and the execution-policy overrides), plus a content
hash and a fail-closed gate for the properties that would otherwise silently default.

Source: `tools/colcon_lf/colcon_lf/resolved_model/` (`model.py` for the schema and
hashing, `resolve.py` for the builder, `__main__.py` for the standalone CLI).

## Shape

```json
{
  "schema": "rcllf.resolved_model",
  "schema_version": 1,
  "system": {"name": "bringup", "system_json_schema_version": 3},
  "claim_status": "supported",
  "records": [
    {
      "contract_id": "timer:publisher_node/timer_callback#period",
      "property": "period",
      "subject": {"kind": "timer", "node": "publisher_node", "callback": "timer_callback"},
      "value": 500.0,
      "unit": "ms",
      "domain": "(0,inf)",
      "provenance": "extracted",
      "evidence": "literal period_ms in system.json",
      "compatibility_mode": "preserved"
    }
  ],
  "mutation_channels": {
    "launch_overrides": "resolved",
    "ros_parameter_events": "observable_not_monitored",
    "environment": "not_detectable",
    "direct_field_mutation": "not_detectable"
  },
  "model_hash": "sha256:...",
  "inputs": [{"role": "system_json", "path": "lf-analysis/pubsub_example/bringup/system.json", "sha256": "..."}]
}
```

The example is a record of `pubsub_example/bringup`, as `colcon lf-generate` writes it.

`records` is sorted by `contract_id` (`<kind>:<subject path>#<property>`, stable
across configurations). `inputs` and `model_hash` sit outside the hash itself, so a
comment-only edit to an input file, or a path/timestamp change, never moves it.
Only a value, a provenance change, or an `acknowledged_hazards` change does.

## Records

A record's `contract_id` is `<kind>:<subject>#<property>`. These are the records
`pubsub_example/bringup` gets, which cover the kinds you meet in a plain system:

| Record | Value | Source |
|---|---|---|
| `node:<node>#fqn` | the node's fully qualified name | `extracted` from the launch file |
| `node:<node>#reaction_order` | `subscriptions_first` or `timers_first` | policy key, `registry-default:reaction_order` unless set |
| `timer:<node>/<callback>#period`, `#offset` | milliseconds | the analysis, or `execution.yaml` |
| `pub:<node>/<topic>#topic`, `#qos_depth`, `#qos_reliability` | resolved topic name and QoS the analysis read | `extracted` |
| `sub:<node>/<topic>#topic`, `#qos_depth`, `#qos_reliability` | the same for subscriptions | `extracted` |
| `sub:<node>/<topic>#take_mode`, `#take_timeout_ms` | `skip` or `block`, and the wait bound in milliseconds | policy keys, `explicitly_configured` |
| `link:<src>-><dst>/<topic>#after_delay` | nanoseconds | `lf-generate` default, `--after-delay` or `execution.yaml` |
| `node:<fqn>#mechanism.take`, `#mechanism.tf2`, `#tf2.*`, `topic:/tf#provenance` | mechanisms the gate checks | see below |

## Provenance

| Value | Meaning |
|---|---|
| `extracted` | Read straight from `system.json` (a literal in the C++ source, or the resolver's own re-derivation). |
| `explicitly_configured` | Set by a launch parameter file, an inline `parameters=` dict, a generate-time `--params-file`, an `execution.yaml` override, or an exec-config policy key (including a registry default, see below). |
| `unresolved` | No source supplied a value. Carries a `reason` (a hazard/refusal code) when one applies. |
| `measured_profiled` | Reserved for a future runtime-measurement source. No resolver path produces it today, and it can never satisfy a gated record. |

An exec-config subscription policy row (`sub:<fqn>/<topic>#<policy key>`) is always
`explicitly_configured`, even when the value is a bare registry default. The evidence
string tells the two apart: `registry-default:<key>` vs. `execution.yaml:<scope>`.

## The fail-closed gate

The gate raises [RCLLF-R001, RCLLF-R002 and RCLLF-R003](diagnostic-codes.md#rcllf-r001).

Generation reads `resolved_model.json`'s records, not the persisted `system.json`
diagnostics, because a value can become resolved *after* analysis (an
`execution.yaml` override, or a generate-time `--params-file`).

- **RCLLF-R001** (timer period unresolved): refuses unless a
  `nodes.<node>.timers.<timer>.period` override in `execution.yaml` supplies the value
  (see below). Listing the code in `acknowledged_hazards` does not proceed.
  The binding matches each timer handle by its exact period in nanoseconds, so
  a timer without a known period could not be bound, and no placeholder period is
  generated.
- **RCLLF-R002** (a node depends on `tf2` or `take()`): refuses unless acknowledged.
  There is no override value for this one. For tf2 the refusal carries a subcode that
  names the conversion ([tf2 lookups](../features/tf2.md)):
  `tf2_direct_lookup` and `tf2_blocking_lookup` (use `rcllf_tf2::AcquisitionLookup`),
  `tf2_listener_unadmitted` (one `TransformListener(*buffer, this)` member) and
  `tf2_clock_mismatch` (a Buffer on the node clock, `/tf` from a node of the system),
  `tf2_stamp_mismatch` and `tf2_stamp_unknown` (a request stamp on the node clock: `now()`
  or the `header.stamp` of an in-system or stamp-anchored input).
  `take()` stays unsupported. `message_filters` stays on the hazard gate of
  [RCLLF-H003](diagnostic-codes.md#rcllf-h003).
- **RCLLF-R003** (`check` subcommand and the run gate): refuses when `resolved_model.json`
  is missing or an `inputs[*].sha256` no longer matches the file on disk.

## The timer-period override

```yaml
# execution.yaml
schema_version: 1
nodes:
  imu_node:
    timers:
      timer_callback:
        period: "5 msec"       # any LF time literal, same grammar as defaults.after_delay
acknowledged_hazards: []       # not needed for a timer this override covers
```

The value reaches the *generated* LF code (`generation.lf_generator.generate_lf_system`'s
`timer_period_overrides`), not only the report. The emitted `timer` statement uses the
real period, and the timer's record is `explicitly_configured`, not `unresolved`. No
acknowledgement needed for that one timer. An unknown timer name is a hard, immediate
validation error naming the node and timer and listing the node's timers.

The same entry takes a start offset, `offset: "2 msec"` (rules in
[Execution Configuration](../features/execution-config.md#timer-start-offsets-and-periods)).
Every timer has a `timer:<node>/<callback>#offset` record in milliseconds: provenance
`explicitly_configured` with evidence `execution.yaml:nodes.<node>.timers.<callback>.offset`
when the entry sets it, else `extracted` with the analyzed `offset_ms` (always 0, rclcpp
timers start at creation). The emitted `timer t_<callback>(<offset>, <period>)` uses it.

## CLI

```bash
python3 -m colcon_lf.resolved_model resolve system.json \
  [--params-file p1.yaml [--params-file p2.yaml ...]] [--exec-config execution.yaml] \
  [-o resolved_model.json] [--workspace path/to/packages]

python3 -m colcon_lf.resolved_model check resolved_model.json [--workspace path/to/packages]
```

`colcon lf-generate` accepts the same `--params-file` (repeatable, later files win,
standing for `--ros-args --params-file` appended at run time) and `--exec-config` flags.

## Limitations

- Parameter-derived timer resolution (e.g. `1 / rate_hz`) is not attempted. A null
  `period_ms` is `unresolved` unless overridden.
- `take()` is detected, not supported: a node with an explicit `take()`,
  `take_serialized()` or `take_loaned()` call gets an unresolved
  `node:<fqn>#mechanism.take` record with reason RCLLF-R002, and
  `colcon lf-generate` exits 1 for it.
- tf2 is supported in one form only ([tf2 lookups](../features/tf2.md)): a
  `tf2_ros::TransformListener(*buffer, this)` member whose `/tf` and `/tf_static` the
  binding feeds into the Buffer inside LF reactions, and lookups through
  `rcllf_tf2::AcquisitionLookup`. The resolved model records it as
  `node:<fqn>#mechanism.tf2` and `node:<fqn>#tf2.lookup.<member>`; the lookup record is
  `explicitly_configured` when `execution.yaml` `nodes.<node>.tf2` sets the wait bound
  or `on_timeout` (the constructor literals stay under `literal`). Each adapter also gets
  `node:<fqn>#tf2.sample_stamp.<member>`: per `request` call the stamp expression and its
  source (`node_clock`, or a message topic with `source: in_system` and its publishers,
  or `external_stamp` for an `anchor: stamp` declaration); unresolved RCLLF-R002 with
  `tf2_stamp_mismatch` or `tf2_stamp_unknown` otherwise. Each listener topic
  gets a `topic:/tf#provenance` (or `topic:/tf_static#provenance`) record: `extracted`
  for an in-system publisher, `explicitly_configured` for an `external_topics` entry
  with `anchor: stamp`, unresolved RCLLF-R002 otherwise. A node that only
  broadcasts is supported without a record of its own. Every other use is refused with
  an RCLLF-R002 subcode: `colcon lf-generate --package tf_listener_example` refuses
  `tf_listener` with `tf2_direct_lookup`, since it calls `lookupTransform` on the Buffer.
- Acknowledging RCLLF-R002 lets generation go on without making tf2 deterministic: the
  refused lookup answers with whatever the Buffer holds when the reaction runs.
  `message_filters` is detected and gated separately ([RCLLF-H003](diagnostic-codes.md#rcllf-h003)).
