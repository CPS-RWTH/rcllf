#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
# The sourced setup files exist only in the container (shellcheck cannot follow them).
# shellcheck disable=SC1090,SC1091
#
# One group of the runtime smoke tests, the test step of the CI
# `runtime-smoke` job. CI and the local gate run this same command.
# Run from the repository root inside the dev image, after
# scripts/ci/build_workspace.sh build and tools.
#
# Environment: ENTRIES, one entry per line,
#   "<source packages>:<pytest -k expression>[:<execution.yaml>[:<launch stem>[:<federate>]]]"
# built (docker/lf_build.sh) and tested one after another. WORKSPACE is the
# repository root as docker/lf_build.sh sees it.
set -eo pipefail

: "${ENTRIES:?set ENTRIES (see the runtime-smoke matrix in .github/workflows/ci.yml)}"
source /opt/ros/jazzy/setup.bash
cd packages
source install/setup.bash
# Entries come in on fd 3: the builds and launches below read stdin.
while IFS= read -r entry <&3; do
  [ -z "$entry" ] && continue
  IFS=: read -r pkgs kexpr exec_yaml launch federate <<< "$entry"
  echo "::group::runtime smoke: $pkgs ($kexpr)"
  # $pkgs is a space-separated package list, split on purpose.
  # shellcheck disable=SC2086
  LF_EXEC_CONFIG="${exec_yaml:+$(realpath "$exec_yaml")}" LF_LAUNCH="$launch" \
    FEDERATE="$federate" bash ../docker/lf_build.sh $pkgs
  source install/setup.bash
  (
    cd ../tools/colcon_lf
    RCLLF_REQUIRE_RUNTIME_SMOKE=1 RCLLF_TAKE_SKIP_FEDERATED="${federate:+1}" \
      python3 -m pytest tests/runtime -v -m runtime -k "$kexpr"
  )
  echo "::endgroup::"
done 3<<< "$ENTRIES"
