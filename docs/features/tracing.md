# Tracing

rcllf's tracing pipeline records an LTTng trace of a running system and turns it
into typed events, causal chains, and metrics with plots, in one command. Read
this page to record a trace, run the analysis, or look up what a metric measures and
its limits.

## What it does

- A **recording path** (`docker/trace.sh` + LTTng) that captures every `ros2:*`
  tracepoint across every container in a run, with zero manual instrumentation.
- An **analysis path** (`tools/lf_eval`, the `lf-eval` CLI) that turns raw CTF trace
  data into typed events, per-callback timelines, causal chains derived from
  `system.json`, and the registered metrics, with plots, in a single invocation.

The same flow works for any package,
so reproducing a result (or comparing `lfc` against stock ROS 2 executors) is a
one-command operation instead of a bespoke analysis script per experiment.

## Recording a trace

Tracing runs as its own Docker Compose service, `tracer`, which every other service
can be traced alongside because all core services share the host PID and IPC
namespaces (`pid: host`, `ipc: host` in `docker-compose.yml`), so one `lttng-sessiond`
sees every container's processes.

```bash
# Terminal 1: start the shared LTTng session
docker compose up tracer

# Terminal 2: run the traced workload
docker compose run --rm dev ros2 launch pubsub_example bringup.launch.py

# Back in terminal 1: stop tracing (Ctrl+C, or from another shell)
docker compose stop tracer
```


Under the hood, `docker/trace.sh`:

1. Creates an LTTng session (`lttng create ros2_trace --output=...`).
2. Enables every ROS 2 userspace tracepoint: `lttng enable-event -u 'ros2:*'`.
3. Starts the session and blocks until it receives `SIGTERM`/`SIGINT`.
4. On shutdown, stops and destroys the session, flushing CTF trace data to disk and
   `chmod`-ing it world-readable.

### Where runs land

Each run gets a timestamped directory under `output/experiments/`
(`$EXPERIMENT_TRACE_DIR`, defaulting to `trace_<YYYYMMDD_HHMMSS>`). Real runs used for
evaluation are typically renamed to `<timestamp>_<mode>_<label>_run<n>`, e.g.
`20260424_002456_stock_pubsub_run3`):

```
output/experiments/<run-dir>/
├── metadata.yaml     # experiment parameters — mode, duration, system, label, hostname
├── tracing/          # LTTng CTF trace files (what lf-eval reads)
├── bags/             # ros2 bag recording (mcap format), if one was taken
└── logs/             # captured stdout/stderr, if any
```

`metadata.yaml` is plain key/value YAML, e.g.:

```yaml
timestamp: "20260424_001126"
mode: "stock"
duration: 60
system: "pubsub_example"
label: "pubsub_run3"
hostname: "host01"
```

`lf-eval` doesn't require `metadata.yaml` to run, but the `mode` field is what lets
the `determinism` metric group runs into `lfc` vs `stock` cohorts for cross-run
comparison. See [Extending](#extending-adding-a-metric-via-the-registry) if you want to key off other
fields.


## Installing `lf-eval`

`lf-eval` lives at `tools/lf_eval` and is not part of the ROS 2 colcon workspace. It's
a plain Python package, installable with pip:

```bash
pip install -e "tools/lf_eval[test]"
```

Extras, from `tools/lf_eval/pyproject.toml`:

| Extra | Adds | Needed for |
|---|---|---|
| `test` | `pytest`, `pytest-cov` | running `tools/lf_eval`'s own test suite |
| `stats` | `scipy` | the KS-test consistency check inside the `determinism` metric (falls back to a pure-numpy statistic with `NaN` p-value if omitted) |

Runtime dependencies (`matplotlib`, `numpy`, `pyyaml`) install unconditionally. The
CTF backend is **not** a pip dependency:

- Preferred: the `bt2` Python bindings, a system package (`python3-bt2`), present in
  the tracing-capable Docker images.
- Fallback: the `babeltrace2` or `babeltrace` (v1) CLI, parsed as text.

`lf-eval` picks whichever is available automatically. Force one with
`--backend {bt2,cli}`.

## Analyzing a trace

```bash
lf-eval analyze <run-dir>... [--system system.json] [--chain A->B->C]
                             [--metrics e2e_latency,cpu,...] [--out DIR]
                             [--backend {bt2,cli}] [--max-gap-ms N]
                             [--keep-raw] [--no-plots]
```

`analyze` reads one or more run directories, derives (or accepts an explicit)
causal chain, computes the requested metrics, writes `<out>/metrics.json` +
`<out>/plots/*.png`, and prints one summary table per metric to the terminal. Exit
codes: `0` success, `1` no runs readable, `2` usage error, `3` no CTF backend
available.

### Single run

```bash
lf-eval analyze output/experiments/latest \
  --system packages/lf-analysis/pubsub_example/bringup/system.json
```

### Multi-run determinism comparison

Because `compute()` receives every run directory at once, comparing `lfc` against
stock ROS 2, or checking run-to-run reproducibility within one mode, is the same
command with more paths:

```bash
lf-eval analyze output/experiments/2026*_lfc_*_run{1,2} \
                output/experiments/2026*_stock_*_run{1,2} \
  --system packages/lf-analysis/pubsub_example/bringup/system.json \
  --metrics e2e_latency,determinism \
  --out output/analysis/lfc_vs_stock
```

For a full multi-variant sweep (stock single/multi/events-threaded vs. `lfc`, across
repeatable scenarios, with a rendered report) rather than hand-picking run directories,
the project uses a benchmark harness that is not part of this release; it drives
this exact `analyze`/`report` path underneath.

### Listing available metrics

```bash
lf-eval list-metrics
```

```text
Available metrics:

  cpu              CPU utilisation/overhead (/proc sampling beside the trace estimate)
  determinism      Ordering invariant, timer jitter, and cross-run reproducibility (cross-run)
  drops            Message drops and queue overflows (publish-vs-delivered, failed takes)
  e2e_latency      Per-chain end-to-end latency distributions and per-hop latency
  output_count     Published values per output topic (outputs.jsonl), group mean
  reproducibility  cross-run per-node callback sequence identity, per mode (cross-run)
  topology         Static vs runtime pub/sub topology diff (soundness checker)

Add your own: subclass lf_eval.metrics.registry.Metric, decorate with @register("name"),
and import it from lf_eval/metrics/__init__.py.
```

### Cross-checking CPU with `sample-cpu`

Trace-derived CPU (see [CPU utilisation](#cpu-utilisation) below) is callback wall
time, not CPU time, so `lf-eval` supports an independent `/proc`-based measurement taken
*while the traced workload runs*:

```bash
# In a third terminal, alongside `docker compose up tracer` and the workload:
lf-eval sample-cpu --patterns pubsub_publisher,pubsub_subscriber \
  --out output/experiments/latest/cpu_proc.json \
  --warmup 5 --window 30
```

`sample-cpu` runs concurrently with the system under test, not after the fact. It
reads fields 14/15 of `/proc/<pid>/stat` for each matching process, skips a warmup
window, samples over a steady-state window, and writes `cpu_proc.json` into the run
directory. Once that file exists, `lf-eval analyze`'s `cpu` metric picks it up
automatically and reports both numbers side by side.

## Metrics

`lf-eval list-metrics` prints every registered metric. The four trace metrics, end-to-end latency, determinism, CPU utilisation and drops, have a section each below. Three more read other inputs: `output_count` counts the published values per output topic in `outputs.jsonl`, `reproducibility` compares each node's callback sequence across runs, and `topology` diffs the static pub/sub graph of `system.json` against the runtime one (the [Soundness Checker](soundness.md)).

Every metric follows the same contract. Given one or more `Run` objects (metadata +
timeline + optional `cpu_proc.json` + drop counts), it either returns a
`MetricResult` with a summary table and plot data, or, if its required inputs are
missing, a result with `available=False` and an explanatory note. `analyze` reports
the latter as "skipped," not a crash.

### End-to-end latency

**What it measures.** Per-chain latency from the first node's `callback_start` to the
last node's `callback_end`, plus a per-hop breakdown, across activations of a derived
or explicit chain.

**What it needs.** `callback_start`/`callback_end` events and (for automatic chain
derivation) a `system.json`. Where the trace also carries `rmw_publish`, `rmw_take`
and the `vpid`/`vtid` contexts that `docker/trace.sh` adds, each hop pairs by
**message identity**. The source timestamp in `rmw_publish.timestamp` equals
`rmw_take.source_timestamp` for the same sample, a take belongs to the next
callback of that subscription on its thread, and a publish belongs to the callback
running on its thread. The downstream invocation is paired with the upstream
invocation that published the message it consumed, with no window. The chain
latency follows the same keys through every hop. Takes that cannot be attributed
(two takes before one callback, a take for another subscription) and keys
published twice, also by a publish outside any traced callback, are left out and
counted as `unresolved`, never guessed.

The pairing basis is decided once per run, not per hop. A run whose ROS trace
carries the identity events above, or an lfc run, pairs every hop of every chain
by message identity. A hop of such a run whose message was never traced is
`unresolved`, it is never paired by timestamps beside identity-paired hops.

**lfc runs.** The rcllf runtime delivers messages by dispatch, not by an rclcpp
take, so the ROS trace of an lfc run cannot tie a message to a callback. A run
directory with `rcllf_trace_manifest.json` is therefore paired from the `rcllf`
dispatch trace: a `CORR_SUB` dispatch names the message it consumes by (gid,
ordinal), and a `publication` event names the dispatch that produced it. The hop
records carry `identity_source: lfc_dispatch` (`ros_trace` for a stock run). The
pairing also needs the run's `system.json` to map runtime reactions to nodes. A
manifest whose dispatch trace cannot be read gives no identity evidence, with the
reason in the result note, and never falls back to the ROS trace.

**Analysis window.** With `--warmup-s`/`--window-s`, pairs are formed on the
whole trace and only the judged downstream (or chain sink) activations must start
inside the window. A message published just before the window and consumed inside
it is therefore still paired, not `unresolved`.

Every downstream activation of an identity-paired hop ends in exactly one counted
outcome, and the hop record lists each count even when it has no sample at all:

| Outcome | Meaning |
|---|---|
| `paired` | the trace names the upstream invocation whose message was used |
| `unresolved` | the data may be the upstream's, but the trace does not name it (no attributed take, a key published twice, an untraced message, or two candidate inputs that overlap in time) |
| `startup` | a timer tick before the node took any input: the run start, not a gap |
| `other_input` | the activation used another node's message |
| `not_chain_data` | the activation is a callback of a subscription that `system.json` does not declare for its node (`communication.topics`), such as `/parameter_events`. A node whose inputs the model cannot name has none of these |

A channel is never treated as outside the chain just because the trace shows nobody publishing on it. A declared input whose publications the trace lost is chain data of unknown origin, so its activations are `unresolved`.

`coverage` is `paired / (paired + unresolved)`, and `complete` is true only when no
activation is unresolved and at least one is paired. The chain carries `complete`
too, true only when the chain and every hop are complete. `lf-eval` prints
`unresolved` and `startup` next to each chain and hop, so a run's start-up samples
are visible in the tool output rather than silently missing. Both the hop pairing
and the chain walk use one rule (`lf_eval.chain_latency.upstream_of`), so an
activation is never unresolved in one and paired with an older message in the other.

A run with no identity evidence at all (intra-process delivery has no
`rmw_publish`, older traces lack the thread contexts) falls back to **timestamp
pairing** for every hop: an upstream `callback_end` with the first downstream
`callback_start` after it. Its `complete` is `null` (unknown, the trace cannot
tell a pair from a coincidence), so the claim gate refuses it and the ordering
verdict is `INCOMPLETE`. This
fallback has no default window (an earlier 200 ms default had no
recorded basis). `--max-gap-ms N` drops pairs further apart than N ms and the
result reports how many it dropped as `excluded_by_window`. Each hop reports the
method that ran in its `pairing` field (`message_identity` or `timestamp`), and
the chain in `e2e_pairing`.

!!! warning "Documented caveat: the timestamp fallback has no error direction"
    Under timestamp pairing a hop can pair with an invocation triggered by a
    *different* message. An unrelated downstream callback that starts before the
    causal one makes the reported latency too small, and a lost or late message
    can make it too large. Such a hop is a heuristic profile with no guaranteed
    error direction, not a bound. Results keep `evidence:
    heuristic_timestamp_pairing` and a clock label in both modes, because the
    physical profile is not a correspondence claim, and the identity-aware
    `semantic_*` metrics below replace it wherever the `rcllf` provider traced
    the edge.

**Gated hops.** A node that owns both a timer and subscriptions re-times incoming
data on its own schedule (only its timer callback publishes). `lf-eval` detects this
structurally from `system.json` and marks hops leaving such a node as "gated,"
pairing from the node's *timer* invocations instead of its subscription callbacks.
Under message identity a timer invocation is paired with the producer of the
newest message the node took before it. The sample is `unresolved` when that
newest input has no traced producer, when an earlier input was still running at
the tick, or when two candidate inputs overlap in time (the newer data cannot be
told apart). A tick before any input is `startup`.

**What the plot shows.** A CDF/histogram of per-chain latency, clipped to the p99.5
percentile with the clipping disclosed in the plot title (e.g. "Distribution
(≤p99.5; 6 outlier(s) clipped)") so a handful of extreme outliers can't visually
collapse the rest of the distribution into one bar. Use `--keep-raw` to retain full
per-sample data in `metrics.json` for further analysis (off by default, since raw samples
can be large). The plot itself always gets full-resolution data regardless of this
flag.

### Determinism

**What it measures.** Two things: (1) an **ordering invariant** (for each *ungated*
hop, the paired downstream `callback_start` must not precede the upstream
`callback_end`), reported as a violation rate, and (2) **cross-run reproducibility**
(for the same chain/hop, a pairwise Kolmogorov-Smirnov test and coefficient of
variation across multiple runs of the same mode). The KS test is defined for any
sample size, so every run with at least one paired sample takes part, and each
comparison reports both sizes (`n_a`, `n_b`) so a reader can judge its power. A run
without a sample on the hop is listed in `excluded_runs`. A hop with fewer than
two sampled runs has no comparison and its runs are listed under
`hops_without_comparison` (an earlier minimum of 5 samples
had no recorded basis).

The ordering verdict is `FAIL` on any violation, `INCOMPLETE` when the evidence
does not cover the chain (an identity-paired hop with an unresolved activation, or
an ungated hop with no pair, or a timestamp-paired run), `PASS` only with neither,
and `N/A` only when no hop could be paired and no activation was left unresolved. `INCOMPLETE` never counts as a pass. Every hop with at least one sample
in two runs is compared by KS, since the test is valid at any size. Each comparison
lists both sample sizes (`n_a`, `n_b`) next to its p-value, so a weak comparison is
visible.

**What it needs.** The same event/chain inputs as end-to-end latency, plus (for the
cross-run comparison) more than one run directory passed to the same `analyze`
invocation. `compute()` receives all runs at once, so no separate "compare" command
is needed. Timer jitter additionally needs each timer's configured period, taken from
`system.json` or falling back to the trace's `rcl_timer_init.period` event.

**What the plot shows.** Per-hop violation rates as a bar chart, and (when multiple
runs of the same mode are present) an overlay of per-run hop-latency distributions so
visibly overlapping vs. diverging distributions are easy to eyeball alongside the KS
p-value.

**Honesty note.** This is the metric most directly aimed at demonstrating rcllf's
determinism claim, so treat its numbers as what they are, an ordering-invariant
violation rate and a statistical reproducibility test, not a proof of determinism in
the formal sense. A low violation rate and a high KS p-value are strong evidence, not
certainty.

### CPU utilisation

**What it measures.** CPU cores consumed by the traced system, as two independent
estimates reported side by side with their relative gap `trace_vs_proc` (the trace
estimate minus the `/proc` one, over the `/proc` one). The metric issues no verdict on
that gap. The two figures measure different quantities (see the caveat below), so no
tolerance separates a broken measurement from a normal difference. An earlier 20%
tolerance had no recorded basis and was removed.

**What it needs.** For the trace-derived estimate: `callback_start`/`callback_end`
pairs (sum of in-callback time / trace wall-clock span). For the `/proc`-derived
estimate: a `cpu_proc.json` produced by `lf-eval sample-cpu` *during* the same run,
placed in the run directory.

!!! warning "Documented caveat: trace-derived CPU is callback wall time"
    Summing in-callback time only counts work that happens inside a traced callback.
    It **excludes** executor spin time, DDS/RMW threads, and any unattributed work,
    and it **includes** time a callback spent preempted or blocked. The number is
    therefore callback wall time, not a lower or upper bound on actual CPU usage. `/proc` sampling (when available) is treated as
    authoritative, and "no `cpu_proc.json` for this run" is a common and expected
    case for historical traces.

**What the plot shows.** Estimated cores used, trace-derived vs. `/proc`-derived
side by side, against the machine's core count (`os.cpu_count()`, override with
`--cores`).

### Drops

**What it measures.** Message loss and queue-overflow indicators. There is no ROS 2
tracepoint for "message dropped," so this metric infers drops from three separate
counting signals rather than one authoritative number:

1. **Failed takes**: `rmw_take` events where `taken == 0` (executor woke up but found
   no message waiting).
2. **Publish-vs-delivered shortfall**: publishes per topic (`rcl_publish` →
   `rcl_publisher_init.topic_name`) times expected subscriber count, compared against
   observed downstream callback invocations on that topic. A **shortfall** is counted
   as a drop, but a **surplus is explicitly not treated as a drop** (it can mean
   pre-trace in-flight messages, or intra-process delivery bypassing `rcl_publish`).
   Both directions are reported so you can judge for yourself.
3. **Startup/shutdown skew**: messages in flight at trace start or stop make edge
   counts approximate. Raw counts are reported alongside the derived deltas, and there is
   no warmup trimming applied automatically.

**What it needs.** `rmw_take`, `rcl_publish`, `rcl_publisher_init`, and subscription
callback events, plus (optionally) `system.json` for QoS depth context.
Infrastructure topics (`/rosout`, `/parameter_events`, `/events/*`, `/clock`) are
excluded by default.

**What the plot shows.** Per-topic published vs. delivered counts and failed-take
rate, so any topic with an outsized shortfall stands out visually.

!!! warning "Documented caveat: inference, not ground truth"
    None of the three signals is a direct "dropped message" tracepoint. Treat the
    numbers as strong circumstantial evidence of drops, not an exact count, as
    stated explicitly in the metric's own module docstring.

## Identity-aware semantic tracing (`rcllf`)

The binding ships an LTTng UST provider `rcllf`. It records the
identity of every tracked publication, the expectation it creates per admitted
edge, the sample ingress retained for it and the LF dispatch that consumed that
exact sample. The analyzer walks a chain by these keys and never compares
timestamps to decide which events belong together. The schema is in the
[semantic trace schema reference](../reference/trace-schema.md).

**Recording.** A benchmark run (the harness is not part of this release) records `rcllf:*` and `ros2:*` on two channels,
`rcllf` and `ros2`, both with the `vpid`, `vtid` and `procname` contexts, and
writes a run manifest `trace_manifest.json` next to `tracing/`. The manifest
holds the enabled events with their filters, the discard count per channel
(`unknown` when lttng printed no counter), the host's boot id, the measured
clock sync at run start and end (chrony, PTP or timesyncd, else `unavailable`),
the launcher exit status and each node process's exit status as `ros2 launch`
reported it (its output is kept as `launch.log`). Federated systems get one
static manifest for the whole federation, so every federate is an expected
producer. A stock run has no provider, so its claims read
`unsupported(provider_absent)`. `lf-eval analyze` needs `--system
<run>/system.json` to resolve chains for the heuristic profiles, `lf-eval
report` reads each run directory's `system.json` by itself.
`docker/trace.sh` records the same channels but writes no run manifest, so its
semantic claims are always `incomplete(no_run_manifest)`.

**Three statuses per claim.** Every correspondence, chain and hop latency claim
gets an event-stream integrity status (`complete` or `incomplete`), a coverage
status (`covered`, `uncovered` or `unsupported`) and a reconciliation status
(`reconciled`, `unreconciled`, or `not_evaluated` when either of the first two
fails). A claim is `supported` only when all three pass. The codes are listed in
the schema reference. A drop on the `ros2` channel fails only the claims that
read `ros2`, a drop on `rcllf` only those that read `rcllf`.

**Heuristic profiles.** `e2e_latency` and `determinism` pair by message identity
where the stock trace allows it and by timestamp otherwise (see
[End-to-end latency](#end-to-end-latency)). Their results carry `evidence: heuristic_timestamp_pairing`
and a clock label (`same_host`, or `cross_host` with the measured clock-sync
bound, or `uncertainty: unknown`). The report shows them under "Physical
profiles (heuristic pairing)". Assigning such a result to a correspondence,
chain or logical reproducibility claim raises `EvidenceRejected`.

**Identity metrics.** `semantic_correspondence` (outcome table per edge),
`semantic_chain` (per source dispatch the key walk to every sink with logical
tag deltas), `semantic_hop_latency` (publication event to the dispatch of the
same key, a same-host physical profile) and `semantic_arrival_lag` (per key the
sample's arrival on the receiver's LF physical clock minus the release tag of
its dispatch, which answers whether an after-delay held without cross-host
pairing). They land in the `semantic` block of `metrics.json`
(`rcllf.semantic_report/1`) and in the report's "Claims" table.

## Extending: adding a metric via the registry

New metrics register themselves with `lf_eval.metrics.registry` and are picked up by
both `--metrics` and `list-metrics` automatically, so no CLI changes are required.

```python
from lf_eval.metrics.registry import Metric, MetricResult, register


@register("retries")
class RetryMetric(Metric):
    summary = "Message retry counts per topic"

    def compute(self, runs, system=None, chains=(), **options):
        # runs: all Run objects passed to `analyze` in this invocation
        # return MetricResult(available=False, note="...") if inputs are missing
        return MetricResult(name=self.name, data={...}, summary_rows=[...])

    def plot(self, result, outdir):
        return []  # list of written Paths
```

Then import the module from `lf_eval/metrics/__init__.py` so the `@register`
decorator runs at import time.

Contract points worth knowing before adding one:

- `compute` receives **all** runs passed to the invocation at once, so cross-run
  metrics (like `determinism`'s cross-run KS test) need no special-casing in the CLI.
- If a metric's required inputs are missing, return
  `MetricResult(available=False, note=...)` rather than raising. `analyze` then
  reports that metric as "skipped" instead of aborting the whole run.
- Raw per-sample data can be attached to the `MetricResult` as an ad-hoc attribute
  (see `raw_samples_by_mode` in `e2e_latency.py`) so plots can use full-resolution
  distributions while `metrics.json` itself stays small.

This is the extension point future policy-linked metrics (e.g. retries, deadline
misses) are expected to use.

## Known constraints

- **No process attribution.** `docker/trace.sh` enables `ros2:*` events with no
  `--context` options, so traces carry no `procname`/`vpid`/`vtid`. All node
  attribution goes through the handle chain (`rcl_node_init` → node name, etc.) or a
  symbol-regex fallback, never by OS process.
- **`bt2` backend needs the dev/tracing image.** The preferred CTF backend (`bt2`
  Python bindings) is a system package present in the tracing-capable Docker images,
  not something `pip install` provides. On a bare host without it, `lf-eval` falls
  back to the `babeltrace2`/`babeltrace` CLI automatically.
- **`scipy` is optional.** Without the `stats` extra, the `determinism` metric's KS
  test degrades to a pure-numpy statistic with a `NaN` p-value instead of failing.
