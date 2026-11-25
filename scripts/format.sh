#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Auto-format C++ files using ament_uncrustify — the formatter the lint gate
# actually enforces. ament_clang_format was dropped 2026-08-05: it is not
# lint-enforced and its style fights uncrustify/cpplint (diverged on ~40
# files that pass lint).
# Each step's failure (missing tool, crash) is reported and makes the script
# exit non-zero — "Formatting complete." is only printed when every step ran.
# Usage: ./scripts/format.sh

set -e

source /opt/ros/jazzy/setup.bash

SRC_DIR="$(dirname "$0")/../packages/src"
FRAME_DIR="$(dirname "$0")/../packages/lf-frame"

FAILED=()

run_step() {
    local desc="$1"
    shift
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "!!  FAILED (command not found): ${desc}"
        FAILED+=("$desc — command not found")
        return
    fi
    echo "==> ${desc}..."
    local rc=0
    "$@" || rc=$?
    # ament_* --reformat exits 1 after successfully reformatting divergent
    # files — that is a healthy outcome. Only rc > 1 (crash, signal, usage
    # error) is a real failure.
    if [ "$rc" -gt 1 ]; then
        echo "!!  FAILED (exit ${rc}): ${desc}"
        FAILED+=("$desc — exit $rc")
    fi
}

run_step "ament_uncrustify --reformat on packages/src" \
    ament_uncrustify --reformat "$SRC_DIR"
run_step "ament_uncrustify --reformat on packages/lf-frame" \
    ament_uncrustify --reformat "$FRAME_DIR"

if [ ${#FAILED[@]} -ne 0 ]; then
    echo "==> Formatting finished with ${#FAILED[@]} FAILED step(s):"
    printf '    - %s\n' "${FAILED[@]}"
    exit 1
fi
echo "==> Formatting complete."
