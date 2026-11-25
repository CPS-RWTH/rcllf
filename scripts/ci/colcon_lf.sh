#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
# The sourced setup files exist only in the container (shellcheck cannot follow them).
# shellcheck disable=SC1090,SC1091
#
# One shard of the colcon_lf suite, the test step of the CI `colcon-lf` job. CI
# and the local gate run this same command. Run from the repository
# root inside the dev image, after scripts/ci/build_workspace.sh build and tools.
#
# Environment: RCLLF_TEST_SHARD=k/N (tests/conftest.py assigns whole xdist groups
# to shards by tests/ci_test_durations.json) and RCLLF_REQUIRE_ROS_TESTS=1 (a
# clang parser test that skips without ROS headers becomes an error).
set -eo pipefail

: "${RCLLF_TEST_SHARD:?set RCLLF_TEST_SHARD=k/N}"
shard=${RCLLF_TEST_SHARD%%/*}
source /opt/ros/jazzy/setup.bash
source packages/install/setup.bash
mkdir -p output
cd tools/colcon_lf
# Parallel; grouping rules live in tests/conftest.py.
pytest tests/ -v -n logical --dist loadgroup --durations=0 2>&1 \
  | tee "../../output/colcon-lf-shard-$shard.log"
