#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
# The sourced setup files exist only in the container (shellcheck cannot follow them).
# shellcheck disable=SC1090,SC1091
#
# The steps of the CI `binding-release` job, one subcommand per
# step. CI and the local gate run these same commands. Run from the
# repository root, as root, inside the dev image. The dependency step is
# `scripts/ci/build_workspace.sh deps packages/lf-frame`.
#
#   reactor-c  fetch only the reactor-c submodule (not navigation2 and the other large ones)
#   build      build ros2_lf_binding in Release with ccache (CCACHE_DIR) and check
#              compile_commands.json for -O3 -DNDEBUG
#   test       colcon test of ros2_lf_binding
set -eo pipefail

case "${1:-}" in
  reactor-c)
    git config --global --add safe.directory "$PWD"
    git submodule update --init --depth 1 third_party/reactor-c
    ;;
  build)
    source /opt/ros/jazzy/setup.bash
    ccache --zero-stats
    cd packages
    colcon build --packages-up-to ros2_lf_binding --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
    ccache --show-stats
    # The build must really be Release: optimized and with NDEBUG.
    cc=build/ros2_lf_binding/compile_commands.json
    grep -qE -- '-O3 -DNDEBUG .*src/runtime\.cpp' "$cc" \
      || { echo "::error::runtime.cpp not built with -O3 -DNDEBUG"; exit 1; }
    ;;
  test)
    source /opt/ros/jazzy/setup.bash
    cd packages
    source install/setup.bash
    colcon test --packages-select ros2_lf_binding --event-handlers console_direct+
    colcon test-result --verbose
    ;;
  *)
    echo "usage: $0 reactor-c | build | test" >&2
    exit 2
    ;;
esac
