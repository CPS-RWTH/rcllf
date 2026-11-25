#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Run the full colcon test suite: unit tests (gtest) plus all ROS2 ament
# linting checks, which are registered as CTest tests. There is no lint-only
# path; the label below says what actually runs.
# --python-testing pytest guards against colcon's silent bare-unittest
# fallback (collects zero tests, stays green) if an ament_python package is
# ever added.
# Usage: ./scripts/lint.sh [package_name]

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/.."

source /opt/ros/jazzy/setup.bash

cd "$WS_DIR/packages"

if [ -d install ]; then
    source install/setup.bash
fi

if [ -n "$1" ]; then
    echo "==> Testing package (unit tests + ament linters): $1"
    colcon test --packages-select "$1" --event-handlers console_direct+ --python-testing pytest
    colcon test-result --verbose --test-result-base build/"$1"
else
    echo "==> Testing all packages (unit tests + ament linters)"
    colcon test --event-handlers console_direct+ --python-testing pytest
    colcon test-result --verbose
fi
