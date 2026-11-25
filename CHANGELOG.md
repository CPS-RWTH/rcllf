# Changelog

This file lists what each rcllf release contains. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/). Before 1.0.0, a minor release may change the
command line, the generated code and the configuration files.

## [0.1.0] - 2026-11-25

The first public prototype of rcllf.

### Added

- `colcon lf` and its steps `lf-analyze`, `lf-generate` and `lf-build`: analyze a ROS 2
  Jazzy package (C++ `rclcpp` nodes and a Python launch file), generate a Lingua Franca
  program that runs the same callbacks in a fixed, repeatable order, and build it.
- `ros2 lf list` and `ros2 lf launch` to find and run the generated systems.
- Two supported deployments: every node in one process, and one process per node
  (`--federate per-node`) coordinated by the Lingua Franca runtime.
- The runtime binding (`ros2_lf_binding`) that drives unchanged node code from the
  generated program, built on a pinned `reactor-c` with the patches in
  `third_party/patches/`.
- Execution configuration files that set the take policy at the ROS to Lingua Franca
  boundary per node or per subscription.
- A Docker dev container with ROS 2 Jazzy, the Lingua Franca compiler and every tool
  preinstalled. This is the one supported install path, tested on x86_64 Linux.
- A five-step learning path: `pubsub_example`, `timer_example` and `service_example`,
  `lifecycle_example`, `fusion_example` (stock `message_filters` synchronizers), and the
  `brake_fusion` showcase, which runs the same node code under stock ROS 2 and under
  rcllf on a MuJoCo vehicle plant.
- Container images on `ghcr.io/cps-rwth/rcllf`, tagged per release: the
  `<version>-deps` image is the dev container of that release, with every dependency
  preinstalled.
- `scripts/fresh_clone_gate.sh`, which clones a release into an empty directory and
  checks that it builds, passes its tests, runs the quickstart with the documented
  output, and ships a current Support Matrix.
- The documentation site: getting started, examples, a guide for running your own
  workspace, features, design and reference pages, built with MkDocs.

### Experimental

These parts ship but are outside the supported scope. Expect rough edges.

- Per-process federation, a custom federation YAML, federation across hosts, and
  decentralized coordination.
- rcllf variants of `message_filters` (BNA synchronizer) and `tf2`.
- Tracing and trace analysis (`docker/trace.sh`, `lf-eval`, `ros2 lf eval`,
  `ros2 lf report`), the soundness check and the resolved-model reports.
- The plant replay (`RCLLF_PLANT_REPLAY`). MuJoCo is the only plant that ships.
- arm64 and NVIDIA Jetson boards.
- The example packages outside the learning path.
