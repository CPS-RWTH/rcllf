# How rcllf Works

rcllf takes an existing ROS 2 package and runs it under a Lingua Franca (LF)
scheduler instead of a ROS 2 executor. Your node code stays as it is. What
changes is who decides when each callback runs. This page walks through the
pipeline once, with `pubsub_example`, and says what changes at run time.

## The pipeline

One command runs the whole pipeline inside the dev container (see
[First LF Run](first-lf-run.md)):

```bash
colcon lf --package pubsub_example
```

It runs four steps. Each step can also be run on its own; the options are in
the [CLI reference](../reference/cli.md).

**1. Analyze** (`colcon lf-analyze`). Reads the launch files and the C++
source without running anything, and writes a model of the nodes, callbacks
and connections for each launch file:

```text
--- pubsub_example/bringup.launch.py ---
  2 nodes, 1 topics, 0 services, 0 actions
  -> /workspace/packages/lf-analysis/pubsub_example/bringup/
```

That folder holds `system.json` and a drawing of the launch graph. See
[Extraction](../design/extraction.md).

**2. Configure** (`colcon lf-configure`). Creates `config/execution.yaml`
next to the model, the file you edit, and writes
`config/execution.reference.yaml` beside it: every setting you could put into
`execution.yaml`, with its default. See
[Execution Configuration](../features/execution-config.md).

**3. Generate** (`colcon lf-generate`). Turns the model into an LF program and
a colcon package that builds it:

```text
  -> /workspace/packages/lf-analysis/pubsub_example/bringup/lf/Bringup.lf
  -> /workspace/packages/lf-generated/lf_generated_pubsub_example_bringup/ (lfc package)
```

Each node becomes a reactor, each callback a reaction, and each topic a
connection between reactors. The package name comes from the package and the
launch file: `bringup.launch.py` gives `lf_generated_pubsub_example_bringup`
(the full rule is under
[Generated package names](../reference/cli.md#generated-package-names)). See
[LF Code Generation](../design/lf-generation.md).

**4. Build** (`colcon lf-build`). Builds the generated package with colcon.
The LF compiler `lfc` turns `Bringup.lf` into C and C++ sources, which are
compiled together with a copy of your node sources into one executable:

```text
System                          Generation  Built  Reason
pubsub_example/bringup          generated   yes
```

Then list and start the system:

```bash
ros2 lf list
ros2 lf launch pubsub_example bringup
```

```text
Package                  Launch Stem                  Nodes  Type         Launch Ready
pubsub_example           bringup                          2  nodes        ready
```

```text
Launching LF-controlled system: pubsub_example/bringup
[Bringup-1] [INFO] [...] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[Bringup-1] [INFO] [...] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
```

Both nodes run in one process, `Bringup`. The launch file is the same one
`ros2 launch` would use, and it is not changed.

## What changes at run time

- **LF decides when callbacks run.** Every callback runs as an LF reaction at a
  logical time, called a [tag](../design/glossary.md#tag). The 500 ms timer of
  the publisher fires at tags 0 ms, 500 ms, 1000 ms and so on. Each callback
  is named by a [reaction ID (RID)](../design/glossary.md#rid-reaction-id).
- **The order is fixed by the program.** Callbacks at the same tag run in an
  order the generated program fixes, not in the order DDS delivers messages
  or threads get scheduled. With the same inputs, every run gives the same
  order and the same results.
- **One publish, one callback run.** On a topic like `/sensor_data`, each
  publish runs the subscriber's callback exactly once, with exactly that
  sample. A missing sample is reported, not silently skipped. See
  [Publication Correspondence](../features/correspondence.md).
- **Time is logical.** `now()` on a node's ROS clock returns the current tag,
  so two callbacks at one tag see the same time. Wall-clock latency does not
  change what a callback sees.
- **Programs can span processes.** `--federate` splits a program into several
  processes that keep one logical timeline. How time stays consistent there
  depends on the [coordination regime](../design/glossary.md#coordination-regime).
  See [Federation](../features/federation.md).

## What stays the same

- Your node classes, callbacks, messages and launch files. rcllf builds a
  copy of the node sources and never edits the originals.
- DDS still carries every message. rcllf only decides when the callback that
  takes the message runs.

One difference shows in the first lines of output: a timer fires first at the
program's start, one period earlier than a stock ROS 2 wall timer. Set its
start offset to get the stock first tick; see
[Timer start offsets](../features/execution-config.md#timer-start-offsets-and-periods).

Not every ROS 2 pattern converts. The analyzer refuses patterns it cannot
model with a diagnostic code; the
[Support Matrix](../features/support-matrix.md) lists what is covered.

## Where to go next

- [First LF Run](first-lf-run.md): run the pipeline yourself.
- [Architecture](../design/architecture.md): what each step writes and how the
  program starts.
- [Time & Coordination](../design/time-and-coordination.md): the exact
  determinism claim.
- [Glossary](../design/glossary.md): LF and rcllf terms.
