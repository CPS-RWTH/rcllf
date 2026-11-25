#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
# The sourced setup files exist only in the container (shellcheck cannot follow them).
# shellcheck disable=SC1090,SC1091
#
# The test steps of the CI `lint` job, one subcommand per step. CI and
# the local gate run these same commands. Run from the repository
# root inside the dev image, after scripts/ci/build_workspace.sh build and tools.
#
#   colcon-test  colcon test of packages/ (includes ament linting; as scripts/lint.sh)
#   shards       the colcon-lf shards partition the suite (collection only)
#   ros2lf       the ros2lf tool tests
#   lf-eval      the trace metrics suite; it needs no ROS build
#   matrices     the support matrix is current
set -eo pipefail

ros_env() {
  source /opt/ros/jazzy/setup.bash
  source packages/install/setup.bash
}

case "${1:-}" in
  colcon-test)
    ros_env
    cd packages
    colcon test --event-handlers console_direct+ --python-testing pytest
    colcon test-result --verbose
    ;;
  shards)
    # Every collected colcon_lf test in exactly one of the 4 shards.
    ros_env
    cd tools/colcon_lf
    ../../scripts/check_test_shards.py 4
    ;;
  ros2lf)
    source /opt/ros/jazzy/setup.bash
    cd tools/ros2lf
    python3 -m pytest test/ -v
    ;;
  lf-eval)
    mkdir -p output
    cd tools/lf_eval
    python3 -m pytest tests/ -v 2>&1 | tee ../../output/lf-eval-test.log
    ;;
  matrices)
    ros_env
    cd tools/colcon_lf
    python3 -m colcon_lf.features.docs_table --check
    ;;
  *)
    echo "usage: $0 colcon-test | shards | ros2lf | lf-eval | matrices" >&2
    exit 2
    ;;
esac
