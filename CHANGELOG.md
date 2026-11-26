# Changelog

This file lists what each rcllf release contains. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/). Before 1.0.0, a minor release may change the
command line, the generated code and the configuration files.

## [0.1.1] - 2026-11-26

Documentation fixes from a first-time-user test of 0.1.0: two testers followed the
README and the Getting Started pages from `git clone` to the first example output and
recorded where the pages disagreed with what the tools printed.

### Fixed

- `lf-fed-publisher` and `lf-fed-subscriber` (Compose): at `FED_TIMEOUT` the federate
  gets SIGINT instead of SIGTERM, so a working multi-container run stops the federation
  at one logical time and both federates resign; before, the peer reported
  `left the federation without resigning` and exited with an error. The federation page
  shows the expected end of such a run.
- Federation page: `ros2 lf launch` runs what the last `colcon lf` generated; running it
  without `--federate` brings back the one-process program.
- Quickstart: `ENTRYPOINT_PACKAGES=all` is passed with `docker compose run -e`; Compose
  does not forward it from the host shell.
- Quickstart: the first container start after `colcon lf` rebuilds the generated
  packages once; the page said later starts skip the build.
- Quickstart: `colcon lf` has four steps (analyze, configure, generate, build); the
  page shows both summary lines it prints, says why `pubsub_example` gives four
  generated packages, and that the "stderr output" is `lfc` progress output.
- Quickstart: the two `Bound` lines and the two stock `started.` lines may come in
  either order; the output checker's summary line and exit code are described.
- Quickstart: the timing paragraph names the cached base image and the host's cores.
- Tutorial and troubleshooting: the configure step, the analyzer's silent first half
  minute, the per-build trace manifest hash, the renamed publisher node and the
  compiler-warning count of the first start.
- `pubsub_example` page: its run command is labeled as the stock ROS 2 run.
- Docker reference: the `ENTRYPOINT_*` settings need `-e`; the `:dev` image tag is one
  per Docker host.

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
