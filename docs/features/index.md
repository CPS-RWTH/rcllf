# Features

One row per capability that has its own page. The **Status** column uses the same words
as the [Support Matrix](support-matrix.md): *supported* means the generator accepts the
feature, *validated* means an evaluation run backs it, and *experimental* means it is
generated and smoke-tested but has no evaluation behind it. The matrix also lists
every diagnostic code and the limits that bound each feature. For the commands that
produce the converted program, see the [CLI reference](../reference/cli.md).

| Capability | What it does | Status | Docs |
|---|---|---|---|
| **Publication correspondence** | Pairs every publish on a topic edge with the dispatch of its subscriber, so a sample that never arrives is reported as lost and not silently skipped. | Supported, validated | [Publication Correspondence](correspondence.md) |
| **Missing messages** | What your node sees when an expected sample does not arrive in time, and how the run reports it. | Supported, validated | [Missing Messages](missing-messages.md) |
| **Execution configuration** | Sets the take policy (`skip` or `block`, and the timeout) per node or per subscription, and explains the QoS hazard warnings. | Supported | [Execution Configuration](execution-config.md) |
| **Callback groups** | Maps `MutuallyExclusive` and `Reentrant` groups onto LF sub-reactors. A reactor is a singleton, so same-callback self-concurrency is not reproduced automatically. | Supported, experimental | [Callback Groups](../design/callback-groups.md) |
| **message_filters** | Converts two-input `ExactTime` and `ApproximateTime` synchronizers and refuses other forms. | Supported, validated | [message_filters](message-filters.md) |
| **BNA synchronizer** | `BoundedNearestToAnchor`, a synchronizer policy that pairs an anchor with the nearest sample in a hard window. | Supported, experimental | [BNA Synchronizer](bna-synchronizer.md) |
| **tf2** | A tf2 lookup is supported through `rcllf_tf2::AcquisitionLookup`. A direct `Buffer` lookup is refused. | Supported, experimental | [tf2](tf2.md) |
| **Federation** | Splits a generated program across OS processes under the LF RTI. DDS still carries all data. Opt-in with `--federate`. | Supported per node, experimental otherwise | [Federation](federation.md) |
| **Fast mode** (`--fast`, `--timeout`) | Runs a generated program faster than physical time and ends it at a logical time. Also settable through `RCLLF_LF_ARGS`. | Supported | [Fast Mode](fast-mode.md) |
| **Plants** (`rcllf_plant`) | Runs a simulated plant such as MuJoCo behind one checked request per step. | Supported, experimental | [Plants](plants.md) |
| **Soundness checker** | Compares the extracted model with the publishers and subscriptions seen in a real run. | Supported | [Soundness Checker](soundness.md) |
| **Visualizations** | Draws the extracted system as Graphviz graphs (SVG by default, PDF with `--format pdf`). | Supported | [Visualizations](visualizations.md) |
| **Tracing** | LTTng tracing of a run and the `lf-eval` command, which turns a trace into seven metrics, four of them computed from the trace events. | Supported | [Tracing](tracing.md) |
