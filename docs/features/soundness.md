# Soundness Checker

The soundness checker tests whether the model `colcon lf-analyze` extracted from your
sources matches what the system really does. It runs your launch file under an
[LTTng](tracing.md) trace and compares the publishers and subscriptions that appeared
at run time with the ones in the extracted model. Use it before you trust a
conversion, or when you suspect the extractor missed a connection.

## Run it

```bash
docker compose --profile lf run --rm soundness-check
```

This checks `pubsub_example` with its `bringup` launch file for 20 seconds. The
terminal shows four steps and ends with a verdict:

```text
==> [1/4] colcon lf-analyze --package pubsub_example
...
==> [2/4] LTTng session ros2_trace_soundness_pubsub_example_bringup_20261127_100705 -> /workspace/output/experiments/soundness/pubsub_example-bringup/<run>/tracing
==> [3/4] bounded run: timeout --foreground 10s ros2 launch pubsub_example bringup.launch.py
...
==> [4/4] lf-eval analyze --metrics topology
System: bringup (2 nodes) [/workspace/packages/lf-analysis/pubsub_example/bringup/system.json]
...
== topology ==
  run                                               mode     missing_from_model  unobserved_in_runtime  unmatched_nodes
  ------------------------------------------------  -------  ------------------  ---------------------  ---------------
  soundness_pubsub_example_bringup_20261127_100705  unknown  0                   0                      0

Written: /workspace/output/experiments/soundness/pubsub_example-bringup/<run>/analysis/metrics.json

===================================================================
PASS: pubsub_example/bringup ... no runtime edges missing from the static model
  unobserved_in_runtime:  0  (declared edge never traced; informational)
  unmatched_nodes:        0
===================================================================
```

Those lines come from a run with `-e SOUNDNESS_DURATION=10`; the full output also
contains the launch log. `PASS` means that every publisher and subscription seen at
run time is in the model. The container exits with 0.

Run the check on the **stock launch** of your package, as the script does, and not on
the generated LF program. The generated program runs as one process named after the
system (`Bringup`), so its nodes do not match the model's node names and the report
shows false `missing_from_model` and `unmatched_nodes` counts.

### Options

Compose reads these environment variables (set them with `-e NAME=value`):

| Variable | Default | Meaning |
|---|---|---|
| `SOUNDNESS_PACKAGE` | `pubsub_example` | package to check |
| `SOUNDNESS_LAUNCH` | `bringup` | launch file stem |
| `SOUNDNESS_DURATION` | `20` | seconds the launch runs |
| `SOUNDNESS_STRICT` | `0` | `1` makes a `missing_from_model` finding fail the run |

To check another package that is not in the first-start build, name it in
`ENTRYPOINT_PACKAGES` too (see the [Docker reference](../reference/docker.md)):

```bash
docker compose --profile lf run --rm -e ENTRYPOINT_PACKAGES="dynamic_topic_example" \
  -e SOUNDNESS_PACKAGE=dynamic_topic_example soundness-check
```

Inside the dev container you can call the script directly:

```bash
scripts/soundness_check.sh <package> [launch-stem] [--strict] [--duration N]
```

It does four things. It writes `system.json` with `colcon lf-analyze --package <pkg>
--format json`, opens an LTTng session, runs `timeout --foreground <duration> ros2
launch <pkg> <launch>.launch.py`, and analyzes the trace with `lf-eval analyze
--metrics topology` (adding `--fail-on-missing-edge` under `--strict`). The trace
lands in `output/experiments/soundness/<pkg>-<launch>/<run-id>/tracing/`.

### Checking a trace you already have

If you already have a traced run directory and a `system.json`, skip the script:

```bash
lf-eval analyze output/experiments/<run-dir> \
  --system packages/lf-analysis/pubsub_example/bringup/system.json \
  --metrics topology
```

Add `--fail-on-missing-edge` to exit non-zero when a `missing_from_model` finding is
present. Every other `lf-eval` metric is advisory and never changes the exit code.
`<run-dir>` may be a directory made by `docker/trace.sh` (it holds `ust/`) or by the
script above (it holds `tracing/`); see [Tracing](tracing.md#where-the-trace-lands).

## What it checks

The extractor resolves publisher and subscriber connections from a C++ syntax-tree
walk plus launch-file remapping. That can silently miss a real connection: a topic
name built by string concatenation, a helper method the analyzer does not follow, or
a macro-wrapped call. A runtime edge with no match in the model is the dangerous
case, a connection the analyzer never told you about.

| Side | Source | Taken from |
|---|---|---|
| Static | `system.json` | each node's `publishers[]` and `subscribers[]` |
| Runtime | LTTng trace of a real `ros2 launch` run | the `rcl_publisher_init` and `rcl_subscription_init` events, which fire once per publisher or subscription when it is created, whether or not a message ever flows |

Node names are matched without regard to case and separators, and topic names
without regard to a leading slash. The comparison covers topics only. Service and
action endpoints are not compared, but the topics that actions and lifecycle nodes
create are understood (see [Derived infrastructure edges](#derived-infrastructure-edges)).

## Findings

| Finding | Meaning | Severity |
|---|---|---|
| `missing_from_model` | A runtime publisher or subscription with no match in the model. The dangerous class: a real connection the analyzer never extracted. | `--strict` and `--fail-on-missing-edge` fail on this. |
| `unobserved_in_runtime` | A modeled edge that never appeared at run time. | Informational. The launch may not have exercised that code path. |
| `unmatched_nodes` | A runtime node name that could not be matched to the model. | Informational. Usually a naming or remapping mismatch. |

Infrastructure topics (`/parameter_events`, `/rosout`, `/tf`, `/tf_static`) are left
out of `missing_from_model` by default, because every node touches some of them and
they would drown the real findings. Pass `--include-infra-topics` to `lf-eval
analyze` to report them anyway.

## Derived infrastructure edges

ROS creates some topics that no `create_publisher` call in your source mentions, but
`system.json` already implies them. The checker derives them and compares them like
declared edges:

| Class | Derived from | Edges |
|---|---|---|
| Action | `communication.actions[*]` (`action`, `server`, `clients`) | the server publishes, and each client subscribes to, `<action>/_action/status` and `<action>/_action/feedback` |
| Lifecycle | a node whose `base_class` is a lifecycle base | the node publishes `/<node>/transition_event` |

Because they are derived per node and not excluded globally, a `transition_event`
published by a node that is not a lifecycle node, or an `_action/status` edge on a node
that is not that action's server or client, still shows up as `missing_from_model`.

- An action's goal, result and cancel endpoints are services, not topics, so they are
  neither derived nor compared.
- Derived edges never produce `unobserved_in_runtime`: a run that never used an action
  is not evidence of an extraction gap.
- `ros2 launch` starts its own lifecycle-manager node (`launch_ros_<pid>`). It is not in
  `system.json`, so the checker ignores it by name.

!!! note "A refused `message_filters` form stays a finding on purpose"
    The analyzer records the two inputs of an admitted two-input ExactTime or
    ApproximateTime synchronizer as ordinary subscriptions. A form that
    [RCLLF-H003](../reference/diagnostic-codes.md#rcllf-h003) refuses, for example a
    `Cache` fed by a `Subscriber`, is not extracted. A `missing_from_model` finding for
    its topic is a true positive, because that edge really is absent from the model.

## Caveats

!!! warning "The trace must capture startup"
    The init events fire once, when a node is constructed. A trace that starts after
    the node is already running shows that node's edges as `unmatched_nodes` or not
    at all, which is not a real finding. The script opens the session before it
    launches the node for this reason. If you assemble your own run, keep that order.

!!! warning "Namespaced systems"
    For systems that use namespaces, the static side can name ports by two different
    spellings of a topic, so a `missing_from_model` or `unobserved_in_runtime` finding
    can be an extraction artifact and not a real discovery. Treat such findings with
    skepticism. The examples this checker is exercised on are not namespaced.

!!! note "Advisory by default"
    `topology` never fails a run on its own. Only `--fail-on-missing-edge` or
    `--strict` makes a finding change the exit code.

## Dynamic topic names (RCLLF-H004)

The static side is only as good as the extractor's topic-name resolution.
[RCLLF-H004](../reference/diagnostic-codes.md#rcllf-h004) warns about a publisher or
subscription whose topic is not one plain string literal, for example
`"prefix_" + std::to_string(id)`. `dynamic_topic_example` (under
`packages/src/fixtures`) has such a publisher, and the checker reports one
`missing_from_model` finding for it (the verdict line reads `FINDINGS: dynamic_topic_example/bringup ... topology mismatch detected`). See the [Support Matrix](support-matrix.md) for
every registered feature.

## See also

- [Support Matrix](support-matrix.md): the diagnostic codes, the static half of the
  soundness story.
- [Tracing](tracing.md): the `lf-eval` pipeline that the `topology` metric belongs to.
- [QoS hazards](../reference/diagnostic-codes.md#rcllf-h001): checked statically and again at
  startup. This checker compares topology, not QoS.
