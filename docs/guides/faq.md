# FAQ

Short answers to the questions a ROS 2 user asks first. Each answer ends with the page that
has the details.

## Do I have to change my node code?

No. rcllf reads your C++ node sources and your Python launch file and generates a Lingua
Franca program that calls your existing callbacks. A few habits make a package convert
cleanly: set up timers, publishers and subscriptions in the constructor, write each callback
as an inline lambda or `std::bind(&Class::method, this)`, and let the node class take
`rclcpp::NodeOptions`. A node that does none of this may still run with a warning, or be
refused with a message that says what to change. See
[What rcllf needs from your package](bring-your-own-code.md#what-rcllf-needs-from-your-package).

## Do I need to know Lingua Franca?

No. The Lingua Franca compiler and the runtime are in the container, and `colcon lf` runs the
whole pipeline. The generated program is a file you can open
(`lf-analysis/<package>/<launch>/lf/*.lf`), and reading it helps to see what rcllf did, but you
never write it. [Your First LF Run](../getting-started/first-lf-run.md) opens each generated
file; the [glossary](../design/glossary.md) explains the terms.

## What is supported, and what is not?

rcllf converts ROS 2 Jazzy C++ nodes (`rclcpp`) that a Python launch file starts, on Fast DDS.
Timers, publishers, subscriptions, services, actions, components, lifecycle nodes and
two-input `message_filters` synchronizers are supported. Python nodes (`ament_python`) are not
converted, and a blocking service client, an action server that sends from its own thread and
a few other patterns are refused. The [Support Matrix](../features/support-matrix.md) lists
every feature with its status. `ros2 lf scan` tells you per launch file whether your package
converts, before you build anything (see the [CLI reference](../reference/cli.md)).

## Why does the first start take minutes?

The first `docker compose run --rm dev` builds the runtime and the learning-path examples. A
package outside that set is not built: name it with
`-e ENTRYPOINT_PACKAGES="<package>"` or the Run command fails with `Package not found`. Later
starts build again only when sources changed. `colcon lf-analyze` can also be silent for about
half a minute on its first run while it parses your sources, and `colcon lf-build` runs the
Lingua Franca compiler once per generated package. See
[What the first start builds](../getting-started/installation.md#what-the-first-start-builds).

## Can I run it without Docker?

Not as a supported setup. The container holds ROS 2 Jazzy, a patched Lingua Franca runtime
and the tools together, and rcllf is tested only in it, on x86_64 Linux. arm64 boards such
as the Jetson Orin are experimental. See
[Installation and Platforms](../getting-started/installation.md) and
[arm64 and NVIDIA Jetson](../getting-started/arm64-jetson.md).

## How do I run the converted system? Why not `ros2 launch`?

Run `ros2 lf launch <package> <launch-stem>`, for example
`ros2 lf launch pubsub_example bringup`. The stem is your launch file's name without
`.launch.py`, as `ros2 lf list` prints it. The command starts the generated launch file of the
generated package, which is named `lf_generated_<package>_<system>`; the naming rule is in the
[CLI reference](../reference/cli.md). Your original launch file starts the stock system and
stays unchanged. See [Porting Your ROS 2 System](porting-guide.md#launch-files).

## How do I see what Lingua Franca did?

Read the run log: every callback that rcllf drives has a `Bound timer` or
`Bound subscription` line, and a callback without one never fires. Each connection that gets
exact sample matching logs an `rcllf corr: admitted edge` line. For the generated program,
open `lf-analysis/<package>/<launch>/lf/*.lf`, and run `colcon lf-analyze` on its own to get
the graphs. [Your First LF Run](../getting-started/first-lf-run.md) walks through all of it.

## What does `RCLLF-H007` (or another code) mean?

Most warnings and refusals that `colcon lf-analyze`, `colcon lf-generate` or the runtime print
start with `RCLLF-` and a code. A refusal without a code, such as a misspelled name in
`execution.yaml`, names the problem in its own text. [Diagnostic Codes](../reference/diagnostic-codes.md) gives
each code its severity, its meaning and what to do. An error stops generation. You can list
its code under `acknowledged_hazards` in `execution.yaml` to continue, but that skips the check
only: the problem stays in the program, and a callback the analyzer could not read still never
fires. See [Configure Execution](configure-execution.md).

## What exactly does "deterministic" mean here?

Callbacks run in logical-time order, and their order does not depend on thread scheduling or on
when DDS delivers a message. On connections that meet the correspondence criteria, each
callback gets exactly the sample its publish produced. Wall-clock time is not made repeatable:
when a callback runs and how long it takes still depend on the OS and the CPU, as in stock
ROS 2. See
[Three kinds of determinism](../design/time-and-coordination.md#three-kinds-of-determinism).

## Does it make my system faster or real-time?

No. rcllf makes the program's meaning independent of wall-clock accidents; it does not shorten
latencies or make them repeatable. Choosing real-time ROS 2 techniques for the wall-clock side
stays your job. See
[Three kinds of determinism](../design/time-and-coordination.md#three-kinds-of-determinism).

## Real time or fast mode?

By default a program runs in real time: the runtime waits until the physical clock reaches
each tag. Fast mode (`--fast`, with a `--timeout`) does not wait, so a simulation or a
regression check covers its logical time in far less wall time, with the same reactions,
order and values. Fast mode is not supported for federations in v0.1.2. See
[Fast Mode](../features/fast-mode.md).

## Does it work across several machines?

One program in one process is the supported setup. Splitting it into one process per node
(`--federate per-node`) is supported on one host, and running the federates on several hosts
is experimental. A federation stops cleanly on Ctrl-C, but one that stops at a logical
`--timeout` currently fails. See [Federation](../features/federation.md).

## My package does not convert. What now?

Run `ros2 lf scan` on the workspace: it says per launch file whether it converts and why not.
The usual causes are a node target created inside your own CMake function, a callback stored in
a local variable (`RCLLF-H007`), and a node class without a `NodeOptions` constructor.
[Troubleshooting](../reference/troubleshooting.md) has an entry for each message, and
[Bring Your Own ROS 2 Code](bring-your-own-code.md#three-outcomes-and-what-to-do-next) shows
the three outcomes on real packages.
