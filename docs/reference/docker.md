# Docker Reference

Every image target, compose service, and the multi-container tracing setup.
You don't need this page to get started. The
[Quickstart](../getting-started/index.md) covers that. Come
here when a page tells you to run a specific service, or to understand the
container layout.

## Dockerfile Architecture

The Dockerfile uses a multi-stage build. Four stages build on each other in a line, and
one independent stage branches off `base`:

```text
base -> tracing -> dev -> deps
base -> lf-rti
```


### Stage 1: `base`

- **From:** `mcr.microsoft.com/devcontainers/base:ubuntu-24.04`
- **Adds:** ROS 2 Jazzy (ros-base), colcon, rosdep, pip
- **Use case:** Minimal ROS 2 environment

### Stage 2: `tracing`

- **From:** `base`
- **Adds:** LTTng tools, `ros-jazzy-tracetools`, `ros-jazzy-tracetools-trace`, `stress-ng`
- **Adds `vscode` user to the `tracing` group** for non-root LTTng operation
- **Use case:** ROS 2 with tracing capabilities

### Stage 3: `dev`

- **From:** `tracing`
- **Adds:** All ament linting/formatting tools, clang-format, cppcheck, uncrustify, flake8,
  pytest, `graphviz`, the `libclang` Python wheel, MkDocs, OpenJDK 21 with `lfc` v0.12.1,
  `actionlint` and the RTI binary
- **Use case:** Full development environment for the core services and the `lf` and
  `federation` profiles

### Stage 4: `deps`

- **From:** `dev`
- **Adds:** Pre-installed rosdep dependencies for all packages in `packages/src/`
- **Sets `USER vscode`**. All containers run as the non-root `vscode` user (uid 1000)
- **Use case:** Development with all dependencies cached in the image
- **Caching:** Only rebuilds when `package.xml` files change


### Stage 5: `lf-rti`

- **From:** `base` (independent branch, not `deps`)
- **Adds:** The Lingua Franca Run-Time Infrastructure (RTI) daemon, built from
  `third_party/reactor-c`, mediates
  [centralized coordination](../design/time-and-coordination.md#regime-2-federated-centralized)
  for a federation
- **Use case:** Standalone RTI container for [Federation](../features/federation.md)
  (the `rti` compose service). The `dev`/`deps` stages also bundle the RTI binary so
  self-managed federation runs can spawn it as a child process

## Docker Compose Services

Four everyday services need no flag: `dev`, `lint`, `docs` and `tracer`. [What each service does](#what-each-service-does) lists every service.

The `lf` profile holds `colcon-lf-test`, which runs the `tools/colcon_lf` pytest suite in the dev container and tees its output to `output/colcon-lf-test.log`:

```bash
docker compose --profile lf run --rm colcon-lf-test
```

Everything else is grouped into profiles, one per workflow. Each profile is
introduced by the page that uses it. The table below is only the map:

| Profile | Use it when you want to... | Before you run it | Introduced in |
|---|---|---|---|
| `lf` | Build and smoke-test the examples under the LF runtime | `docker compose build dev`, then start `dev` once so the workspace is built (`test-binding`, `analyzer`, `reviz` and `soundness-check` read `packages/build` and `packages/install`) | [Your First LF Run](../getting-started/first-lf-run.md) |
| `federation` | Run a multi-process federation with an RTI | `lf-build-federated` (or `lf-build-federated-decentralized`) before `lf-pubsub-federated`, and the `rti` service running before `lf-fed-publisher` and `lf-fed-subscriber` | [Federation](../features/federation.md) |

```bash
docker compose --profile <name> run --rm <service>
```

For the full service list of a profile: `docker compose --profile <name> config --services`.

### What each service does

| Service | Profile | What it does | Usage |
|---|---|---|---|
| `dev` | none | Interactive shell with the workspace mounted; builds the runtime and the learning-path examples on entry | `docker compose run --rm dev` |
| `lint` | none | Runs `colcon test` (the lint checks) and exits | `docker compose run --rm lint` |
| `docs` | none | Serves this documentation with `mkdocs serve` | `docker compose up docs` |
| `tracer` | none | Starts an LTTng session that captures the events of every container, see [Multi-Container Tracing](#multi-container-tracing) | `docker compose up tracer` |
| `lf-build` | `lf` | Runs `colcon lf` for each package you name (default `pubsub_example`) and smoke-tests the result | `docker compose --profile lf run --rm lf-build [PACKAGE...]` |
| `analyzer` | `lf` | `colcon lf-analyze --format all` over every package | `docker compose --profile lf run --rm analyzer` |
| `reviz` | `lf` | `colcon lf-analyze --viz-only --format all`: redraws the visualizations from existing `system.json` files | `docker compose --profile lf run --rm reviz` |
| `soundness-check` | `lf` | Analyzes, runs the system for a bounded time under tracing and compares the traced topology with the model. Advisory (exit 0) unless `SOUNDNESS_STRICT=1`. See [Soundness](../features/soundness.md) | `docker compose --profile lf run --rm soundness-check` |
| `test-binding` | `lf` | Builds `ros2_lf_binding` with `BUILD_TESTING=ON` and runs its gtests | `docker compose --profile lf run --rm test-binding` |
| `colcon-lf-test` | `lf` | Runs the `tools/colcon_lf` pytest suite | `docker compose --profile lf run --rm colcon-lf-test` |
| `rti` | `federation` | The LF run-time infrastructure daemon that coordinates logical time between federates; start it before any federate | `docker compose --profile federation up -d rti` |
| `lf-build-federated` | `federation` | Builds `pubsub_example` as a 2-federate system with centralized coordination | `docker compose --profile federation run --rm lf-build-federated` |
| `lf-pubsub-federated` | `federation` | Runs that system; the launch file starts its own RTI. Needs `lf-build-federated` first | `docker compose --profile federation run --rm lf-pubsub-federated` |
| `lf-build-federated-decentralized` | `federation` | The same build with decentralized coordination from `federation.yaml` | `docker compose --profile federation run --rm lf-build-federated-decentralized` |
| `lf-pubsub-federated-decentralized` | `federation` | Runs the decentralized build for up to 30 s | `docker compose --profile federation run --rm lf-pubsub-federated-decentralized` |
| `lf-fed-publisher`, `lf-fed-subscriber` | `federation` | One federate each, for separate terminals; they connect to the `rti` service and run for `FED_TIMEOUT` seconds (default 60), then get SIGINT | `docker compose --profile federation run --rm lf-fed-publisher` |

### Shared Configuration

All services share a common base (`x-common` YAML anchor) that provides:

- Same Dockerfile build target (`deps`)
- Workspace mount (`.:/workspace`)
- **Shared host PID and IPC namespaces** (`pid: host`, `ipc: host`). This enables
  cross-container tracing and shared UST ring buffers
- Host networking (for ROS 2 DDS discovery)
- Shared `LTTNG_HOME` volume (`lttng-run`) for a single LTTng session daemon
- Runs as `vscode` user (uid 1000). Output files are owned by uid 1000, which is your
  host user on most single-user Linux machines. With another host uid, take them back
  with `sudo chown -R "$(id -u):$(id -g)" packages output`

## Entrypoint

The entrypoint (`docker/entrypoint.sh`) runs on every container start. The image's
`/entrypoint.sh` is a shim that runs the mounted copy when the workspace is bind-mounted,
so entrypoint edits take effect without rebuilding the image.

1. Stops with exit code 1 when `third_party/reactor-c` is empty (a clone without the
   submodule) and prints the fix, `git submodule update --init third_party/reactor-c`
2. Sources `/opt/ros/jazzy/setup.bash`
3. Builds ROS 2 packages with `colcon build --symlink-install` **only if** a file under
   `packages/src`, `packages/lf-frame` or `packages/lf-generated` is newer than the stamp
   left by the previous successful build. Which packages: `ENTRYPOINT_PACKAGES` unset or
   `examples` builds the runtime (`ros2_lf_binding`), the learning-path examples
   (`pubsub_example`, `timer_example`, `service_example`, `lifecycle_example`,
   `fusion_example`, `brake_fusion`) and their dependencies, plus any generated
   `lf_generated_*` package; `all` builds every package; a list such as
   `ENTRYPOINT_PACKAGES="pubsub_example my_robot"` builds those and their dependencies.
   `ENTRYPOINT_FORCE_BUILD=1` always builds.
   `ENTRYPOINT_SKIP_BUILD=1` never builds. Compose does not forward these three from the
   host shell; pass them with `-e`, for example
   `docker compose run --rm -e ENTRYPOINT_PACKAGES=all dev`. The build runs `--parallel-workers 2` with
   `MAKEFLAGS=-j3` (at most 2 packages x 3 compiler jobs) and prints both values when it
   starts. `ENTRYPOINT_PARALLEL_WORKERS=N` changes the worker count; a `MAKEFLAGS` set by
   the caller is kept. `docker-compose.yml` sets `ENTRYPOINT_PARALLEL_WORKERS=2` and
   `MAKEFLAGS=-j3` for every service, so the service commands that build on their own
   (`lint` and the `lf-build*` scripts) use the same limit; override both with `-e` as for the entrypoint.
4. Sources the workspace overlay
5. Installs the repo's Python CLIs (`colcon-lf`, `ros2 lf`, `lf-eval`) as
   editable packages, skipped when they already import. `~/.local` is a named compose volume
   (`vscode-local`), so this is a one-time cost per volume. `ENTRYPOINT_FORCE_PIP=1` reinstalls.
6. Runs the container command (`exec "$@"`)

### Building more packages

The default selection builds the runtime, the learning-path examples and any generated
`lf_generated_*` package only. Launching
a package outside it, here `action_example`, fails with the default selection:

```text
$ docker compose run --rm dev ros2 launch action_example bringup.launch.py
Package 'action_example' not found: "package 'action_example' not found, searching: ['/workspace/packages/install/timer_example', '/workspace/packages/install/service_example', ...
```

Name the package in `ENTRYPOINT_PACKAGES` and the entrypoint builds it and its dependencies first:

```text
$ docker compose run --rm -e ENTRYPOINT_PACKAGES="action_example" dev ros2 launch action_example bringup.launch.py
[INFO] [action_server_node-1]: process started with pid [2723444]
[INFO] [action_client_node-2]: process started with pid [2723445]
[action_server_node-1] [INFO] [1795775671.664429146] [action_example.action_server]: Action server ready on 'analyze'
...
[action_client_node-2] [INFO] [1795775674.764830680] [action_example.action_client]: Result: success=true, results=[0, 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9]
```

Use `ENTRYPOINT_PACKAGES=all` to build every package.

## Building Images Locally

```bash
# Build the default (deps) target
docker compose build dev

# Build a specific target
docker build -f docker/Dockerfile --target tracing -t rcllf:tracing .
```

## Pulling Pre-built Images

Each release tag publishes one image per target, tagged `<version>-<target>`:

```bash
docker pull ghcr.io/cps-rwth/rcllf:0.1.2-deps      # what the Compose services run
docker pull ghcr.io/cps-rwth/rcllf:0.1.2-dev       # dev target, without the rosdep layer
docker pull ghcr.io/cps-rwth/rcllf:0.1.2-tracing   # tracing target
docker pull ghcr.io/cps-rwth/rcllf:0.1.2-base      # base target
docker pull ghcr.io/cps-rwth/rcllf:0.1.2-lf-rti    # standalone RTI (the rti service)
```

The Compose services expect the **deps** target under the local name
`ghcr.io/cps-rwth/rcllf:dev`. Re-tag the pulled image to that name so Compose uses
it instead of building:

```bash
docker tag ghcr.io/cps-rwth/rcllf:0.1.2-deps ghcr.io/cps-rwth/rcllf:dev
```

Do not `docker compose pull dev`: the registry's `:dev` tag is the dev target, which
lacks the rosdep layer the Compose services rely on.

## Layer Caching Strategy

The Dockerfile orders its layers from rarely to often changed:

1. **System packages** (rarely change) are installed first
2. **rosdep dependencies** (change when `package.xml` changes) are in a separate stage
3. **Source code** (changes frequently) is mounted at runtime, not copied into the image

This means a source code change never rebuilds the image. The entrypoint runs `colcon build` when sources changed.

## Multi-Container Tracing

### How It Works

All services use `pid: host`, sharing the host's PID namespace. A single `lttng-sessiond` can see every process across all containers.

### Usage

```bash
# Terminal 1: Start tracing
docker compose up tracer

# Terminal 2: Run nodes (traced automatically)
docker compose run --rm dev ros2 launch pubsub_example bringup.launch.py

# Stop tracing (Ctrl+C or):
docker compose stop tracer
# -> Trace saved to output/experiments/trace_<timestamp>/
```

The `tracer` log confirms the session before you start a node:

```text
tracer-1  | Session ros2_trace created.
tracer-1  | Traces will be output to /workspace/output/experiments/trace_20261127_102917
tracer-1  | ust event rcllf:* created in channel rcllf
tracer-1  | ust event ros2:* created in channel ros2
tracer-1  | Tracing started for session ros2_trace
tracer-1  | ==> Tracing active. All ROS2 events across containers are being recorded.
```

The trace directory holds the CTF streams under `ust/uid/1000/64-bit/`. A run of the
stock `pubsub_example` (plain ROS 2 nodes, no generated LF system) produced 974
events from `babeltrace2`, all of them `ros2:*` (for example `ros2:rcl_init`
with `procname = "publisher_node"`). `rcllf:*` events appear only for a process
built by the LF pipeline, see [Trace Schema](trace-schema.md).

### Analyzing Traces

```bash
docker compose run --rm dev bash -c \
  "babeltrace2 /workspace/output/experiments/trace_<timestamp>/"
```

For metrics and plots instead of raw event dumps, see
[Tracing & Evaluation](../features/tracing.md) (`lf-eval analyze`).

### Under the Hood

1. All services set `pid: host`, so containers share the host PID namespace
2. All services share `LTTNG_HOME=/tmp/lttng-home` via a named Docker volume (`lttng-run`)
3. The `tracer` service runs `docker/trace.sh`, which starts `lttng-sessiond`, creates the session `ros2_trace` with two channels, `rcllf` and `ros2`, and enables `rcllf:*` events in the first and `ros2:*` events in the second
4. Any ROS 2 process in any container automatically registers its tracepoints
5. On stop, trace data is flushed to `output/experiments/trace_<timestamp>/`
6. All services run as `vscode` (uid 1000), so output files belong to uid 1000 (your
   host user when your host uid is 1000)
