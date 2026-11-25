# Soundness Checker

The soundness checker verifies that `colcon lf-analyze`'s statically extracted
pub/sub graph matches what a real run actually does, by diffing it against an
execution traced with [LTTng](tracing.md). Read this page to check whether your
system's extracted model is trustworthy, or to investigate a suspected extraction
gap.

## What it checks

The extraction pipeline (`colcon lf-analyze`) resolves publisher/subscriber
connections from a C++ AST walk plus launch-file remapping resolution
(`comm_analysis/topic_mapper.py`). That resolution can silently under-report a real
connection: a topic name built via string concatenation, a helper method the
analyzer doesn't traverse, or a macro-wrapped call site.

The soundness checker closes that loop: it runs the real system under an LTTng trace
and diffs the **runtime-observed** pub/sub graph against the **statically extracted**
one. A runtime edge with no matching entry in the static model is the dangerous
class, a connection the analyzer never told you about.

## What it compares

| Side | Source | Extracted from |
|---|---|---|
| Static | `system.json` (`colcon lf-analyze` output) | Each node's declared `publishers[]`/`subscribers[]` (`SystemModel`, `tools/lf_eval/lf_eval/model/system.py`) |
| Runtime | LTTng trace of a real `ros2 launch` run | `rcl_publisher_init` / `rcl_subscription_init` tracepoints (fire once per publisher/subscription **at construction time**, regardless of whether a message ever flowed) |

Node identity is reconciled between the two sides via `SystemModel`'s existing
`canonical_id`/`aliases` scheme (case/separator-insensitive matching between a
trace's `rcl_node_init.node_name` spelling and `system.json`'s `key`/`name`/
`class_name` fields). Topic matching is slash-tolerant (`"a/b"` and `"/a/b"` compare
equal).

This is topics only, v1. The comparison never looks at service or action
*endpoints* themselves (backlog). It does, however, understand the **topics that
actions and lifecycle nodes create**, because those show up on the runtime side
whether or not the model mentions them. See
[Derived infrastructure edges](#derived-infrastructure-edges) below.

## Running it

### One command: `scripts/soundness_check.sh`

```bash
scripts/soundness_check.sh <package> [launch-stem] [--strict] [--duration N]

# Examples
scripts/soundness_check.sh pubsub_example
scripts/soundness_check.sh pubsub_example bringup --strict
scripts/soundness_check.sh pubsub_example bringup --duration 15
```

`launch-stem` defaults to `bringup`. Flow:

1. `colcon lf-analyze --package <pkg>` writes `packages/lf-analysis/<pkg>/<launch>/system.json`.
2. A per-run LTTng session (unique session name, `ros2:*` events), output under
   `output/experiments/soundness/<pkg>-<launch>/<run-id>/tracing/` (repo convention:
   all traces land under `output/experiments/`).
3. `timeout --foreground <duration> ros2 launch <pkg> <launch>.launch.py`, a bounded
   run, default 20 seconds.
4. `lf-eval analyze <run-dir> --system <system.json> --metrics topology` (adds
   `--fail-on-missing-edge` under `--strict`).
5. A PASS/FINDINGS summary printed to the terminal, with counts and a pointer to the
   full `metrics.json`.

### Via Docker Compose

```bash
docker compose --profile lf run --rm soundness-check
docker compose --profile lf run --rm -e SOUNDNESS_PACKAGE=pubsub_example soundness-check
docker compose --profile lf run --rm -e SOUNDNESS_STRICT=1 soundness-check
```

Overridable environment variables: `SOUNDNESS_PACKAGE` (default `pubsub_example`),
`SOUNDNESS_LAUNCH` (default `bringup`), `SOUNDNESS_DURATION` (default `20`),
`SOUNDNESS_STRICT` (default `0`).

### Directly via `lf-eval`

If you already have a traced run directory and a `system.json`, skip the script:

```bash
lf-eval analyze output/experiments/<run-dir> \
  --system packages/lf-analysis/pubsub_example/bringup/system.json \
  --metrics topology
```

Add `--fail-on-missing-edge` to make the CLI exit non-zero when a
`missing_from_model` finding is present. Every other `lf-eval` metric stays purely
advisory (never affects the exit code), and this flag is the one opt-in exception,
scoped to the `topology` metric only.

## Finding classes

| Finding | Meaning | Severity |
|---|---|---|
| `missing_from_model` | A runtime pub/sub edge with no matching entry in the static model. The dangerous class: a real connection the analyzer never extracted. | This is what `--strict`/`--fail-on-missing-edge` gates on. |
| `unobserved_in_runtime` | A static edge that was never traced at runtime (the node/topic exists in `system.json` but no matching init event fired during the run). | Informational, can mean the launch never exercised that code path, not necessarily a bug. |
| `unmatched_nodes` | A runtime node name that could not be resolved against the static model at all (via `canonical_id`/`aliases`). | Informational, usually a naming/remap mismatch worth investigating, not proof of a missing edge. |

Infrastructure topics (`/parameter_events`, `/rosout`, `/tf`, `/tf_static`) are
excluded from `missing_from_model` by default, because every real node publishes/subscribes
to some of these and they would otherwise swamp the dangerous-class findings with
noise the static extractor never claimed to capture. Pass
`--include-infra-topics` to `lf-eval analyze`/`report` to report them anyway.

## Derived infrastructure edges

ROS creates some topics that no `create_publisher`/`create_subscription` call in
your source ever mentions, so the static extractor cannot list them. But they
*are* implied by other things `system.json` already records. Rather than adding
them to the exclusion list above (which would suppress genuine findings on the
same topics), the model **derives** them and compares them as ordinary declared
edges:

| Class | Derived from | Edges |
|---|---|---|
| Action | `communication.actions[*]` (`action`, `server`, `clients`) | server **publishes**, each client **subscribes** to `<action>/_action/status` and `<action>/_action/feedback` |
| Lifecycle | a node whose `base_class` is a lifecycle base | node **publishes** `/<node>/transition_event` |

Because these are derived per node rather than globally excluded, a
`transition_event` published by a *non*-lifecycle node, or an `_action/status`
edge on a node that is not that action's server or client, still surfaces as a
`missing_from_model` finding.

Two limits:

- **An action's goal, result and cancel endpoints are services, not topics**, so
  they never appear as publisher/subscription init events and are not derived.
  If one ever shows up on the topic side, it stays a reportable finding.
- **Derived edges never produce `unobserved_in_runtime` findings.** A run that
  simply never exercised an action is not evidence of an extraction gap, and
  reporting it would trade one noise class for another.

`ros2 launch` spawns its own lifecycle-manager node (`launch_ros_<pid>`) which
subscribes to the transition-event topics of the nodes it manages. It is not in
`system.json` because it was never meant to be, so it is excluded by node name.
Both its edges and its unresolvable identity are tooling noise, not findings.

!!! note "Refused `message_filters` subscriptions stay findings on purpose"
    The analyzer records the two inputs of an admitted
    two-input ExactTime or ApproximateTime synchronizer as ordinary
    subscriptions, so `MessageFiltersSubscriberNode`'s `/sync_topic_a` and
    `/sync_topic_b` are part of the static model. A form `RCLLF-H003` refuses,
    such as `MessageFiltersCacheNode`'s `Subscriber` feeding a `Cache`, is not
    extracted. A `missing_from_model` finding for its topic is a **true
    positive, not noise**, because that runtime edge genuinely is absent from
    the static model. Suppressing it would hide a real, already-known gap.

## Caveats

!!! warning "Trace must capture startup"
    `rcl_publisher_init`/`rcl_subscription_init` fire once, at node construction. A
    trace started *after* the node is already running (or a run that exits before
    the LTTng session starts) will show that node's edges as `unmatched_nodes` or
    simply absent, not a real soundness finding. `scripts/soundness_check.sh`
    creates the session before launching the node for exactly this reason. If you
    assemble your own run directory, make sure the same ordering holds.

!!! warning "Namespaced systems: static side not fully trustworthy"
    The static extractor has a known internal-consistency bug for namespaced
    systems (`_build_connections` derives LF port names from the fully-resolved
    topic string while `_build_ports` derives them from the per-node relative
    string, a known open bug). Until
    that's fixed, treat `missing_from_model`/`unobserved_in_runtime` findings on a
    namespaced system with extra skepticism, because a mismatch there can be a static-side
    resolution bug rather than a genuine runtime discovery. The playground examples
    this checker is exercised against today are all non-namespaced.

!!! note "Advisory by default"
    Like every `lf_eval` metric, `topology` never fails a run on its own.
    `data["failed"]` and the CLI's non-zero exit only happen with
    `--fail-on-missing-edge`/`--strict`.

!!! note "Runtime runs are not in GitHub CI (yet)"
    No `docker compose` profile runs inside GitHub Actions today. LTTng-traced
    example runs are real new infrastructure with real flake risk, deliberately
    deferred. `scripts/soundness_check.sh` and the `soundness-check` compose
    service are verified locally / in the dev container across the playground
    example set, and a CI runtime job is a follow-up if that verification holds up.

## `topics.dynamic_name` (RCLLF-H004): the extraction-side half of this story

The soundness checker's static side is only as good as the extractor's topic-name
resolution. `RCLLF-H004` (`topics.dynamic_name`, WARN) flags publishers/subscribers
whose topic argument is not a lone resolvable string literal, including the
concatenation case (`"prefix_" + std::to_string(id)`) that used to silently resolve
to a *partial*, wrong topic name with no warning at all. See the [Support
Matrix](support-matrix.md) for every module including this one.

## QoS hazards: the startup half

The checker compares topology, not QoS. QoS hazards are found statically by
`colcon lf-analyze` and checked again by the binding at startup on the effective
QoS. `RCLLF-H001` (best effort) and `RCLLF-H002` (depth 1) run on the reader of every
admitted and legacy correspondence edge, so a scenario overlay
or a ROS parameter that changes a reader's QoS is still reported. See
[RCLLF-H001](execution-config.md#rcllf-h001) and
[RCLLF-H002](execution-config.md#rcllf-h002) for what stays static.

## Related

- [Support Matrix](support-matrix.md), every registered feature module, including
  `topics.dynamic_name` (RCLLF-H004), which is the static-extraction half of this
  same "don't silently under-report" story.
- [Tracing](tracing.md), the general `lf-eval` trace → metrics → plots pipeline this
  checker's `topology` metric is one member of.
