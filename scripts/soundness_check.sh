#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Soundness/topology-diff checker (Phase 7, design decision 4 & 7):
# statically extract a package's system.json, run its launch file under a
# bounded LTTng trace, then diff the runtime rcl_publisher_init/
# rcl_subscription_init graph against the static model via
# `lf-eval analyze --metrics topology`.
#
# Flow: colcon lf-analyze --package <pkg> -> per-run LTTng session (unique
# session name, ros2:* events, explicit output dir) -> bounded `ros2 launch` -> stop/destroy the
# session -> `lf-eval analyze ... --metrics topology`. Advisory by default
# (always exits 0 unless something in the pipeline itself breaks); pass
# --strict to also fail non-zero on a topology "missing_from_model" finding
# (a runtime edge the static analyzer never extracted — the dangerous class,
# see docs/features/soundness.md).
#
# Usage:
#   scripts/soundness_check.sh <package> [launch-stem] [--strict] [--duration N]
#
#   scripts/soundness_check.sh pubsub_example
#   scripts/soundness_check.sh pubsub_example bringup --strict
#   scripts/soundness_check.sh pubsub_example bringup --duration 15
#
# [launch-stem] defaults to "bringup" (most example packages' combined
# launch file); pass it explicitly for packages that only ship a
# single-node launch file (e.g. timer_example -> timer_node).
#
# Traces land under output/experiments/soundness/<pkg>-<launch>/ (repo
# convention: all traces live under output/experiments/).

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

PACKAGE=""
LAUNCH_STEM="bringup"
STRICT=0
DURATION=20

usage() {
    echo "Usage: $0 <package> [launch-stem] [--strict] [--duration N]" >&2
    exit 2
}

if [ $# -lt 1 ]; then
    usage
fi

PACKAGE="$1"
shift

if [ $# -gt 0 ] && [[ "$1" != --* ]]; then
    LAUNCH_STEM="$1"
    shift
fi

while [ $# -gt 0 ]; do
    case "$1" in
        --strict)
            STRICT=1
            shift
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage
            ;;
    esac
done

echo "==> Soundness check: package=${PACKAGE} launch=${LAUNCH_STEM} duration=${DURATION}s strict=${STRICT}"

source /opt/ros/jazzy/setup.bash

cd "$REPO_ROOT/packages"
if [ -d install ]; then
    source install/setup.bash
fi

echo "==> [1/4] colcon lf-analyze --package ${PACKAGE}"
colcon lf-analyze --package "$PACKAGE" --format json

SYSTEM_JSON="$REPO_ROOT/packages/lf-analysis/${PACKAGE}/${LAUNCH_STEM}/system.json"
if [ ! -f "$SYSTEM_JSON" ]; then
    echo "error: expected system.json not found: ${SYSTEM_JSON}" >&2
    echo "       (check that ${LAUNCH_STEM}.launch.py exists under packages/src/${PACKAGE}/launch/)" >&2
    exit 1
fi

RUN_ID="soundness_${PACKAGE}_${LAUNCH_STEM}_$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$REPO_ROOT/output/experiments/soundness/${PACKAGE}-${LAUNCH_STEM}/${RUN_ID}"
TRACE_DIR="$RUN_DIR/tracing"
mkdir -p "$TRACE_DIR"

SESSION_NAME="ros2_trace_${RUN_ID}"
CHANNEL_NAME="soundness_channel"
export LTTNG_HOME="${LTTNG_HOME:-/tmp/lttng-home}"
mkdir -p "$LTTNG_HOME"

cleanup_session() {
    lttng stop "$SESSION_NAME" >/dev/null 2>&1 || true
    lttng destroy "$SESSION_NAME" >/dev/null 2>&1 || true
}
trap cleanup_session EXIT

echo "==> [2/4] LTTng session ${SESSION_NAME} -> ${TRACE_DIR}"
lttng destroy "$SESSION_NAME" >/dev/null 2>&1 || true
lttng create "$SESSION_NAME" --output="$TRACE_DIR"
lttng enable-channel -u "$CHANNEL_NAME" --subbuf-size=16M --num-subbuf=8
lttng enable-event -u 'ros2:*' -c "$CHANNEL_NAME"
lttng start "$SESSION_NAME"

echo "==> [3/4] bounded run: timeout --foreground ${DURATION}s ros2 launch ${PACKAGE} ${LAUNCH_STEM}.launch.py"
timeout --foreground "$DURATION" ros2 launch "$PACKAGE" "${LAUNCH_STEM}.launch.py" || true

lttng stop "$SESSION_NAME" >/dev/null 2>&1 || true
lttng destroy "$SESSION_NAME" >/dev/null 2>&1 || true
trap - EXIT

if [ ! -d "$TRACE_DIR/ust" ] && [ -z "$(find "$TRACE_DIR" -mindepth 1 -print -quit 2>/dev/null)" ]; then
    echo "warning: trace directory ${TRACE_DIR} looks empty — the launch may have exited before any node started" >&2
fi

echo "==> [4/4] lf-eval analyze --metrics topology"
EVAL_OUT="$RUN_DIR/analysis"
EVAL_ARGS=(analyze "$RUN_DIR" --system "$SYSTEM_JSON" --metrics topology --out "$EVAL_OUT" --no-plots)
if [ "$STRICT" -eq 1 ]; then
    EVAL_ARGS+=(--fail-on-missing-edge)
fi

set +e
lf-eval "${EVAL_ARGS[@]}"
EVAL_RC=$?
set -e

METRICS_JSON="$EVAL_OUT/metrics.json"
MISSING_COUNT="n/a"
UNOBSERVED_COUNT="n/a"
UNMATCHED_COUNT="n/a"
if [ -f "$METRICS_JSON" ]; then
    read -r MISSING_COUNT UNOBSERVED_COUNT UNMATCHED_COUNT <<EOF
$(python3 - "$METRICS_JSON" <<'PYEOF'
import json
import sys

with open(sys.argv[1]) as handle:
    payload = json.load(handle)

topology = payload.get("metrics", {}).get("topology", {})
per_run = topology.get("data", {}).get("per_run", {})
missing = unobserved = unmatched = 0
for entry in per_run.values():
    counts = entry.get("counts", {})
    missing += counts.get("missing_from_model", 0)
    unobserved += counts.get("unobserved_in_runtime", 0)
    unmatched += counts.get("unmatched_nodes", 0)
print(missing, unobserved, unmatched)
PYEOF
)
EOF
fi

echo
echo "==================================================================="
if [ "$MISSING_COUNT" != "n/a" ] && [ "$MISSING_COUNT" -gt 0 ]; then
    echo "FINDINGS: ${PACKAGE}/${LAUNCH_STEM} — topology mismatch detected"
    echo "  missing_from_model:     ${MISSING_COUNT}  (runtime edge the static model never declared)"
    echo "  unobserved_in_runtime:  ${UNOBSERVED_COUNT}  (declared edge never traced)"
    echo "  unmatched_nodes:        ${UNMATCHED_COUNT}"
    echo "  See ${METRICS_JSON} (metrics.topology.data.per_run) for details."
elif [ "$MISSING_COUNT" = "n/a" ]; then
    echo "FINDINGS: ${PACKAGE}/${LAUNCH_STEM} — could not read topology results (${METRICS_JSON} missing)"
else
    echo "PASS: ${PACKAGE}/${LAUNCH_STEM} — no runtime edges missing from the static model"
    echo "  unobserved_in_runtime:  ${UNOBSERVED_COUNT}  (declared edge never traced — informational)"
    echo "  unmatched_nodes:        ${UNMATCHED_COUNT}"
fi
echo "==================================================================="

if [ "$STRICT" -eq 1 ]; then
    exit $EVAL_RC
fi
exit 0
