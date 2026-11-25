# colcon-lf

The rcllf command line: the colcon verbs `colcon lf-analyze`, `lf-configure`,
`lf-generate`, `lf-build` and `colcon lf` (all steps). It reads a ROS 2 workspace
(launch files, C++ node sources, CMake), writes the extracted system model, and
generates the Lingua Franca program and its colcon package. The `ros2 lf` verbs live
next door in `tools/ros2lf/`.

User documentation is on the docs site: [CLI Reference](../../docs/reference/cli.md),
[Extraction](../../docs/design/extraction.md) and
[LF Code Generation](../../docs/design/lf-generation.md). This file is the entry point
for changing the code.

## Where things are

| Path | What it does |
|---|---|
| `colcon_lf/verb/` | One module per verb (`analyze.py`, `configure.py`, `generate.py`, `build.py`, `lf.py`); the lfc package emitters are in `generate.py` |
| `colcon_lf/launch_analysis/` | Python launch files to a launch graph (nodes, remappings, parameters) |
| `colcon_lf/node_analysis/` | CMake and C++ (libclang) to per-node models: timers, publishers, subscriptions, services, actions, callback groups |
| `colcon_lf/comm_analysis/`, `colcon_lf/graph/` | Connections between nodes and the assembled system graph |
| `colcon_lf/output/` | `system.json` and the other analysis files |
| `colcon_lf/exec_config/`, `colcon_lf/resolved_model/` | Execution configuration and the resolved model with its claim status |
| `colcon_lf/generation/` | The LF program, reaction IDs, binding headers and trace manifest |
| `colcon_lf/federation/` | `--federate` partitioning and the per-federate packages |
| `colcon_lf/features/` | The feature registry behind the generated [Support Matrix](../../docs/features/support-matrix.md) |
| `colcon_lf/visualization/` | Graphviz views of the analyzed system |
| `docs/extractor-status.md` | Which C++ constructs the extractor reads |
| `tests/` | The pytest suite, including runtime tests under `tests/runtime/` |

## Tests

Inside the dev container (`docker compose run --rm dev`), from the repository root:

```bash
cd tools/colcon_lf
pytest tests/ -q                                  # the whole suite, serially
pytest tests/test_topic_mapper.py -q              # one file
pytest tests/ -q -k cmake_parser                 # tests whose name matches
```

Tests that need ROS headers, `compile_commands.json` or a built example skip with a
reason when those are missing; the dev container's first start provides them. The whole
suite takes a while serially. CI runs it in four shards
(`scripts/ci/colcon_lf.sh`, `RCLLF_TEST_SHARD=k/4`); the `colcon-lf-test` Compose
service (profile `lf`) runs it once and keeps the log in `output/`.

When you change what the generator emits, also run the pipeline on an example and read
its output, not only the tests: `colcon lf --package pubsub_example`, then
`ros2 lf launch pubsub_example bringup` (expected output in the
[Quickstart](../../docs/getting-started/index.md#what-success-looks-like)).

When you change the feature registry, regenerate the Support Matrix with
`python3 -m colcon_lf.features.docs_table` from this directory; `--check` says whether
the committed page is current, and CI runs it.
