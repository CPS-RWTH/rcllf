# Code Structure

This page maps the repository's directories to the concepts they implement.
Use it to find your way around the source before you extend a pipeline stage.

## Runtime packages (`packages/`)

`packages/` is the colcon workspace root. `packages/src/` holds user
application packages (`pubsub_example`, `component_example`, `system_bringup`,
and similar), ordinary ROS 2 packages that the extraction tool reads and the
generator targets.

`packages/lf-frame/ros2_lf_binding/` is the LF runtime binding, the C++ layer
that lets a Lingua Franca federate host ROS 2 node code. Key files:

| File | Role |
|---|---|
| `include/ros2_lf_binding/ros2_lf_binding.h` | Public API |
| `include/ros2_lf_binding/node_registry.h` | Node registration interface |
| `src/runtime.cpp` | Runtime lifecycle |
| `src/node_binding.cpp` | Node-to-reactor binding |
| `src/reaction_dispatch.cpp` | Reaction dispatch into ROS 2 callbacks |

Read [Architecture](../design/architecture.md) for how the binding fits the
pipeline, and [Binding Internals](ros2-lf-binding.md) for the API itself.
`packages/lf-frame/ros2_lf_msgs/` carries the small set of interfaces the
binding uses internally. `rcllf_plant` holds the simulation plants
([Plants](../features/plants.md)), among them the MuJoCo vehicle that `brake_fusion`
drives. `rcllf_message_filters` ([BNA synchronizer](../features/bna-synchronizer.md))
and `rcllf_tf2` ([tf2](../features/tf2.md)) are experimental.

`packages/lf-analysis/` and `packages/lf-generated/` are gitignored outputs of
`colcon lf-analyze` and `colcon lf-generate`, extracted `system.json` graphs
and emitted `.lf`/launch files. They do not ship in the repo. Running the
pipeline recreates them.

## Tooling (`tools/`)

| Directory | What it does | Docs |
|---|---|---|
| `tools/colcon_lf/` | Extraction and generation CLI (`colcon lf-analyze / lf-generate / lf-build / lf`). Its [README](https://github.com/CPS-RWTH/rcllf/blob/main/tools/colcon_lf/README.md) maps the modules and the tests. | [CLI](cli.md), [Extraction](../design/extraction.md), [LF Code Generation](../design/lf-generation.md) |
| `tools/lf_eval/` | Trace evaluation: reads LTTng/CTF traces and computes timing metrics. | [Tracing](../features/tracing.md) |
| `tools/ros2lf/` | The `ros2 lf` command extension (`list`, `launch`, `scan`, `eval` and `report` verbs). | [CLI](cli.md) |

The benchmark orchestrator is not part of this repository. The method and the results are in the rcllf paper,
[arXiv:2606.09203](https://doi.org/10.48550/arXiv.2606.09203), and the trace metrics it uses are computed by `tools/lf_eval/`.

## Runtime dependency

`third_party/reactor-c` is a git submodule, the Lingua Franca C runtime that
generated code links against, pinned to one commit. The image build applies the
patches in `third_party/patches/` to it; the submodule itself stays unmodified.

## Infrastructure

`docker/` holds the layered Dockerfile (`base` / `tracing` / `dev` / `deps` /
`lf-rti` targets), the container entrypoint, and `trace.sh` for LTTng session
setup.
`scripts/` holds the lint, format and visualization helpers, the documentation
checks, and the fresh-clone gate (`fresh_clone_gate.sh`); `scripts/ci/` holds the steps
the CI jobs run. `.github/workflows/` defines the CI, docs, and Docker-image jobs
described in `CONTRIBUTING.md`.

