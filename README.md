# rcllf

Deterministic ROS 2 nodes, coordinated by [Lingua Franca](https://www.lf-lang.org/).

rcllf analyzes a supported ROS 2 Jazzy C++ application (its node sources and its
launch file) and generates a Lingua Franca program that invokes the existing
callbacks in logical time. Your node code stays unchanged.

What that makes repeatable, and what it does not:

- **Firing order.** Callbacks run in logical-time order. At one logical time, the
  callbacks of one node run one after another in an order fixed by the generated
  program, and a callback that receives another callback's message runs after it,
  not by thread scheduling or DDS arrival. Callbacks of nodes with no connection
  between them may run at the same time on different threads, in either order;
  nothing a topic carries can tell the difference, but state they share outside
  ROS 2 (a global, a file) can: run those with one worker (`workers: 1` in the
  [execution config](docs/features/execution-config.md), or `RCLLF_LF_ARGS="-w 1"` at launch).
  This holds in one process and across federates under centralized coordination,
  the mode `--federate` generates.
- **Sample selection** is a separate guarantee. ROS 2 still carries the payloads over
  DDS. On topic connections that meet the
  [publication correspondence](docs/design/time-and-coordination.md#publication-correspondence-topic-edges)
  criteria, each callback gets exactly the sample its publish produced, and a sample
  that is missing or too late is reported. Other connections take whatever sample
  is in the DDS reader when the callback fires.
- **Wall-clock time** is not made repeatable: when a callback runs and how long it
  takes still depend on the OS, the CPU and DDS, as in stock ROS 2.

The exact guarantees and their conditions:
[Three kinds of determinism](docs/design/time-and-coordination.md#three-kinds-of-determinism)
and the [Support Matrix](docs/features/support-matrix.md).

This is a prototype release (v0.1.1).

## What is supported

- **ROS 2 Jazzy, C++ nodes (`rclcpp`)**, analyzed from source and a Python launch file.
- **Fast DDS (`rmw_fastrtps_cpp`)**, the RMW the dev container sets. A generated program with
  correspondence connections refuses to start under any other RMW.
- **One process**: every node of a launch file in one generated program.
- **One process per node** (`--federate per-node`): the same program split into
  federates that a Lingua Franca runtime coordinates.
- **One install path: Docker.** The dev container has ROS 2, the Lingua Franca
  compiler and every tool preinstalled. Tested on x86_64 Linux.

## Quick start

You need Git, [Docker](https://docs.docker.com/get-docker/) 20.10+ and
[Docker Compose](https://docs.docker.com/compose/install/) v2 or later.

```bash
git clone --branch v0.1.1 https://github.com/CPS-RWTH/rcllf.git
cd rcllf
git submodule update --init third_party/reactor-c
docker compose build dev
docker compose run --rm dev
```

The last command opens a shell in the dev container; on its first start it builds the
runtime and the example packages. In that shell, convert and run `pubsub_example`:

```bash
cd packages && source install/setup.bash
colcon lf --package pubsub_example && source install/setup.bash
ros2 lf launch pubsub_example bringup
```

It worked when both nodes log from one process, `Bringup-1`, and every `Publishing:`
line is followed by an `I heard:` line with the same values.
[Quickstart](docs/getting-started/index.md) shows the expected output, a prebuilt image
of the release, and what to do if the output differs.
[Your First LF Run](docs/getting-started/first-lf-run.md) opens what rcllf generated at
each step.

## Learning path

| Step | Example | What it shows |
|---|---|---|
| 1 | [`pubsub_example`](docs/examples/pubsub-example.md) | The whole loop: analyze, generate, build, run ([Quickstart](docs/getting-started/index.md), [Your First LF Run](docs/getting-started/first-lf-run.md)). Then the same system as one process per node ([Federation quick start](docs/features/federation.md#quick-start)). |
| 2 | [`timer_example`](docs/examples/timer-example.md), [`service_example`](docs/examples/service-example.md) | Timers become Lingua Franca timers; a request and its response keep their order. |
| 3 | [`lifecycle_example`](docs/examples/lifecycle-example.md) | Managed (lifecycle) nodes. |
| 4 | [`fusion_example`](docs/examples/fusion-example.md) | Stock `message_filters` synchronizers; message stamps become logical time. |
| 5 | [`brake_fusion`](docs/examples/brake-fusion.md) | The showcase: the same node code under stock ROS 2 and under rcllf, braking a simulated vehicle (MuJoCo) to a repeatable stop. |

To run your own package, see [Bring Your Own Code](docs/guides/bring-your-own-code.md).

## Experimental

These parts ship but are not covered by the supported scope above. Expect rough edges.

- Per-process federation, a custom federation YAML, and federation across hosts.
- Decentralized coordination (`--coordination decentralized`).
- rcllf variants of `message_filters` (BNA synchronizer) and `tf2`.
- Tracing and trace analysis (`docker/trace.sh`, `lf-eval`, `ros2 lf eval`, `ros2 lf report`).
- The soundness check (the `soundness-check` Compose service) and the resolved-model reports.
- The plant replay (`RCLLF_PLANT_REPLAY`). MuJoCo is the only plant that ships.
- arm64 / NVIDIA Jetson boards.
- The example packages outside the learning path.

## Documentation

The documentation site is built with MkDocs:

```bash
docker compose up docs   # http://127.0.0.1:8000
```

Without Docker, on a host with Python 3 (a virtual environment, since Ubuntu 24.04 and
other current distributions refuse a system-wide `pip install`):

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements-docs.txt && mkdocs serve
```

mkdocs-material prints a boxed notice about MkDocs 2.0 at startup. It does not apply here:
the pinned mkdocs-material requires `mkdocs<2`. `NO_MKDOCS_2_WARNING=1` hides the notice.

## Contributing, citing, license

See [CONTRIBUTING.md](CONTRIBUTING.md). If you use rcllf in academic work, see
[CITATION.cff](CITATION.cff). Apache-2.0, see [LICENSE](LICENSE).
