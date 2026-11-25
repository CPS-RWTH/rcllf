# Docker Reference

Every image target, compose service, and the multi-container tracing setup.
You don't need this page to get started. The
[Quickstart](../getting-started/index.md) covers that in two commands. Come
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

Four everyday services need no flag:

| Service | Description | Command |
|---------|-------------|---------|
| `dev` | Interactive development shell | `docker compose run --rm dev` |
| `lint` | Run colcon test (linting) | `docker compose run --rm lint` |
| `docs` | Serve this documentation at localhost:8000 | `docker compose up docs` |
| `tracer` | LTTng tracing session | `docker compose up tracer` |

The `lf` profile also holds `colcon-lf-test`, which runs the `tools/colcon_lf` pytest suite in the dev container and tees its output to `output/colcon-lf-test.log`:

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
   `ENTRYPOINT_SKIP_BUILD=1` never builds. The build runs `--parallel-workers 2` with
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
docker pull ghcr.io/cps-rwth/rcllf:0.1.0-deps      # what the Compose services run
docker pull ghcr.io/cps-rwth/rcllf:0.1.0-dev       # dev target, without the rosdep layer
docker pull ghcr.io/cps-rwth/rcllf:0.1.0-tracing   # tracing target
docker pull ghcr.io/cps-rwth/rcllf:0.1.0-base      # base target
docker pull ghcr.io/cps-rwth/rcllf:0.1.0-lf-rti    # standalone RTI (the rti service)
```

The Compose services expect the **deps** target under the local name
`ghcr.io/cps-rwth/rcllf:dev`. Re-tag the pulled image to that name so Compose uses
it instead of building:

```bash
docker tag ghcr.io/cps-rwth/rcllf:0.1.0-deps ghcr.io/cps-rwth/rcllf:dev
```

Do not `docker compose pull dev`: the registry's `:dev` tag is the dev target, which
lacks the rosdep layer the Compose services rely on.

## Layer Caching Strategy

The Dockerfile is structured for optimal Docker layer caching:

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
3. The `tracer` service runs `docker/trace.sh`, which starts `lttng-sessiond` and enables `ros2:*` events
4. Any ROS 2 process in any container automatically registers its tracepoints
5. On stop, trace data is flushed to `output/experiments/trace_<timestamp>/`
6. All services run as `vscode` (uid 1000), so output files belong to uid 1000 (your
   host user when your host uid is 1000)
