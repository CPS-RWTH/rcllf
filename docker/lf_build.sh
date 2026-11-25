#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# LF pipeline for example packages: analyze -> generate -> build -> smoke test.
#
# Usage: lf_build.sh [PACKAGE...]        (default: pubsub_example)
#   FEDERATE=per-node|per-process|<yaml>  generate federated packages instead
#                                         (no smoke test; run them via the
#                                         federation compose services)
#   COORDINATION=centralized|decentralized  coordination mode for FEDERATE
#                                         output (default: centralized; a
#                                         custom <yaml>'s own `coordination:`
#                                         key is used when this is unset)
#
# Runs after the container entrypoint, so the workspace is already built and
# `colcon lf-*` is installed. Every package gets its own lf-analysis/ tree, and
#  its own generated package name
# (lf_generated_<package>_<launch>), so two packages sharing a launch stem
# (e.g. pubsub_example and service_example, both "bringup") no longer collide.
# Stale output from a previous run of the SAME package/launch pair is still
# removed before regenerating.

# pipefail: `colcon lf-generate ... | tee` must fail the script when generation
# is refused, not carry on to build a package that was never generated.
set -eo pipefail

source /opt/ros/jazzy/setup.bash
WORKSPACE="${WORKSPACE:-/workspace}"
# Build limit as in the entrypoint: docker-compose.yml sets
# MAKEFLAGS=-j3; colcon builds this many packages at once.
BUILD_WORKERS="${ENTRYPOINT_PARALLEL_WORKERS:-2}"
cd "$WORKSPACE/packages"
[ -f install/setup.bash ] && source install/setup.bash

PACKAGES=("$@")
[ ${#PACKAGES[@]} -eq 0 ] && PACKAGES=(pubsub_example)

# basic_components has no launch file: its combo_pipeline is a component graph
# run with `ros2 run`. lifecycle_example and parameter_example have no
# bringup.launch.py, only their own
# node launch file. Everything else uses the
# package's bringup launch file.
launch_for() {
  case "$1" in
    basic_components) echo combo_pipeline ;;
    lifecycle_example) echo lifecycle_node ;;
    parameter_example) echo parameter_node ;;
    callback_group_example) echo callback_group_node ;;
    correspondence_fixture) echo corr_bringup ;;
    sentry_pipeline) echo sentry_pipeline ;;
    *) echo bringup ;;
  esac
}

# The smoke run's exit status counts. Pass on 124 (still running
# at the 30 s bound, stopped by `timeout`) or 0 (stopped by its own LF
# timeout); anything else (a startup refusal, a crash, an at-once exit) fails
# the script. 124 counts only after the full 30 s: `timeout` passes a 124 of
# the program's own through, and the runtime's stop bound exits 124,
# which `ros2 run` would otherwise hand over as a pass. A package that exits
# non-zero on purpose needs an entry here, with its reason:
#   <package>) echo "<rc> <reason>" ;;
# None does today.
smoke_rc_exception() {
  case "$1" in
    *) echo "" ;;
  esac
}

smoke_tail_lines=30

for pkg in "${PACKAGES[@]}"; do
  # LF_LAUNCH: build another launch file of the package (correspondence_fixture
  # also carries corr_loop, a feedback loop).
  launch=${LF_LAUNCH:-$(launch_for "$pkg")}
  system_json="$WORKSPACE/packages/lf-analysis/$pkg/$launch/system.json"

  echo "==> [$pkg] analyze"
  colcon lf-analyze --package "$pkg"

  # Package-qualified guess: the actual name may still differ
  # (e.g. the generator appends "System" to de-conflict with a node class
  # name), which is why every branch below cross-checks against the
  # generator's own "(lfc package)" log line rather than trusting this guess.
  gen_pkg_guess="lf_generated_${pkg}_$(echo "$launch" | tr -d '_')"

  if [ -n "${FEDERATE:-}" ]; then
    echo "==> [$pkg] generate federated packages (--federate $FEDERATE)"
    rm -rf "lf-generated/${gen_pkg_guess}_federated"* \
           "build/${gen_pkg_guess}_federated"* \
           "install/${gen_pkg_guess}_federated"*
    coord_args=()
    [ -n "${COORDINATION:-}" ] && coord_args=(--coordination "$COORDINATION")
    colcon lf-generate --system-json "$system_json" --federate "$FEDERATE" "${coord_args[@]}"
    fed_pkgs=$(ls lf-generated/ | grep "lf_generated_${pkg}_.*_federated" | tr '\n' ' ')
    echo "==> [$pkg] build: $fed_pkgs"
    # shellcheck disable=SC2086
    colcon build --symlink-install --parallel-workers "$BUILD_WORKERS" --packages-select $fed_pkgs
    echo "==> [$pkg] federated build complete"
    continue
  fi

  gen_pkg="$gen_pkg_guess"
  echo "==> [$pkg] generate $gen_pkg"
  rm -rf "lf-generated/$gen_pkg" "build/$gen_pkg" "install/$gen_pkg"
  gen_log=$(mktemp)
  # LF_EXEC_CONFIG: optional execution.yaml for this run (runtime smoke uses
  # it to generate lifecycle_example with auto_transition: false).
  exec_args=()
  [ -n "${LF_EXEC_CONFIG:-}" ] && exec_args=(--exec-config "$LF_EXEC_CONFIG")
  if [ "$pkg" = basic_components ]; then
    colcon lf-generate --system-json "$system_json" --no-launch "${exec_args[@]}" | tee "$gen_log"
  else
    colcon lf-generate --system-json "$system_json" "${exec_args[@]}" | tee "$gen_log"
  fi
  # The generator appends "system" when the launch stem equals a node class
  # name (parameter_node -> lf_generated_parameternodesystem), so take the
  # package name from its own "(lfc package)" line instead of guessing.
  reported=$(sed -n 's#.*/lf-generated/\(lf_generated_[A-Za-z0-9_]*\)/ (lfc package).*#\1#p' "$gen_log" | head -1)
  rm -f "$gen_log"
  if [ -n "$reported" ] && [ "$reported" != "$gen_pkg" ]; then
    echo "==> [$pkg] generated package is $reported"
    gen_pkg="$reported"
  fi

  echo "==> [$pkg] build $gen_pkg"
  colcon build --parallel-workers "$BUILD_WORKERS" --packages-select "$gen_pkg"
  source install/setup.bash

  echo "==> [$pkg] smoke test (30 s)"
  if [ "$pkg" = basic_components ]; then
    smoke_cmd=(ros2 run "$gen_pkg" ComboPipeline)
  else
    # The generated launch file is named after the *reactor* stem only
    # (lf_<stem>.launch.py) — never package-qualified, since it lives inside
    # the already-unique gen_pkg — not always "lf_bringup.launch.py"
    # (a stale wrapper-layout launch file could be picked otherwise,
    # and lifecycle_example's stem ("lifecyclenode") is a concrete
    # case where the old hardcoded "lf_bringup.launch.py" does not exist in
    # the package). Strip "lf_generated_<pkg>_", not just "lf_generated_", so
    # a package-qualified gen_pkg (e.g. lf_generated_pubsub_example_bringup)
    # yields "bringup", not "pubsub_example_bringup".
    launch_stem="${gen_pkg#lf_generated_"${pkg}"_}"
    smoke_cmd=(ros2 launch "$gen_pkg" "lf_${launch_stem}.launch.py")
  fi
  smoke_log=$(mktemp)
  smoke_rc=0
  smoke_start=$SECONDS
  # pipefail: the pipeline's status is the smoke run's, not tee's.
  SYSTEM_JSON="$system_json" timeout 30 "${smoke_cmd[@]}" 2>&1 | tee "$smoke_log" || smoke_rc=$?
  exception=$(smoke_rc_exception "$pkg")
  smoke_secs=$((SECONDS - smoke_start))
  if [ "$smoke_rc" -eq 124 ] && [ "$smoke_secs" -ge 30 ]; then
    echo "==> [$pkg] smoke passed: ran ${smoke_secs} s without error (stopped by the 30 s timeout)"
  elif [ "$smoke_rc" -eq 0 ]; then
    echo "==> [$pkg] smoke passed: stopped on its own after ${smoke_secs} s (exit 0)"
  elif [ -n "$exception" ] && [ "$smoke_rc" -eq "${exception%% *}" ]; then
    echo "==> [$pkg] smoke passed (rc $smoke_rc, exception: ${exception#* })"
  else
    echo "==> [$pkg] smoke FAILED: rc $smoke_rc after ${smoke_secs} s (pass: 124 at the 30 s bound, 0 own LF timeout); last $smoke_tail_lines log lines:" >&2
    tail -n "$smoke_tail_lines" "$smoke_log" >&2
    rm -f "$smoke_log"
    exit 1
  fi
  rm -f "$smoke_log"
done
