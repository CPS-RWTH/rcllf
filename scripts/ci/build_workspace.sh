#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
# The sourced setup files exist only in the container (shellcheck cannot follow them).
# shellcheck disable=SC1090,SC1091
#
# The run steps of .github/actions/build-workspace and the dependency step of the
# binding-release job, one subcommand per step. CI and the local gate run these
# same commands. Run from the repository root, as root, inside the dev image.
#
#   deps [PATH]  apt-get update, ccache if missing, scripts/rosdep_install.sh [PATH]
#   build        colcon build of packages/ with ccache (CCACHE_DIR); CORR_TEST_HOOKS=true
#                builds ros2_lf_binding with the correspondence test hooks
#   tools        editable install of colcon_lf[test] and every other tool with a
#                pyproject.toml (no dependencies; they are in the image)
set -eo pipefail

case "${1:-}" in
  deps)
    # scripts/rosdep_install.sh retries `rosdep update`, falls back to a cached
    # index and fails on any key `rosdep check` cannot resolve.
    apt-get update
    # The dev image carries ccache; install it when an older image lacks it.
    command -v ccache >/dev/null || apt-get install -y --no-install-recommends ccache
    source /opt/ros/jazzy/setup.bash
    ./scripts/rosdep_install.sh ${2:+"$2"}
    ;;
  build)
    source /opt/ros/jazzy/setup.bash
    ccache --zero-stats
    cd packages
    # Every correspondence fault hook (RCLLF_TEST_CORR_*) is compiled
    # only on request; an ordinary build logs that a set hook does nothing.
    hooks=OFF
    [ "${CORR_TEST_HOOKS:-false}" = true ] && hooks=ON
    colcon build --symlink-install --cmake-args \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      -DRCLLF_CORR_TEST_HOOKS="$hooks"
    ccache --show-stats
    ;;
  tools)
    # GitHub skips the image entrypoint, so install the tools the way
    # docker/entrypoint.sh does (their dependencies are in the image). Some
    # benchmark tests import colcon_lf; lf_build.sh needs the colcon lf verbs.
    pip install --break-system-packages -e "tools/colcon_lf[test]"
    others=()
    for d in tools/*/; do
      case "$d" in tools/colcon_lf/|tools/ros2lf/) continue ;; esac
      if [ -f "$d/pyproject.toml" ]; then others+=(-e "${d%/}"); fi
    done
    if [ "${#others[@]}" -gt 0 ]; then
      pip install --break-system-packages --no-deps "${others[@]}"
    fi
    mkdir -p output
    ;;
  *)
    echo "usage: $0 deps [PATH] | build | tools" >&2
    exit 2
    ;;
esac
