#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

# Tracer script — starts an LTTng session, records all ROS2 UST events,
# and flushes traces on shutdown (SIGTERM/SIGINT).
#

source /opt/ros/jazzy/setup.bash

WORKSPACE="${WORKSPACE:-/workspace}"
TRACE_DIR="${EXPERIMENT_TRACE_DIR:-$WORKSPACE/output/experiments/trace_$(date +%Y%m%d_%H%M%S)}"
mkdir -p /tmp/lttng-home "${TRACE_DIR}"
export LTTNG_HOME=/tmp/lttng-home
# A unique name tells parallel sessions apart in `lttng list` and the logs.
SESSION="${LTTNG_SESSION_NAME:-ros2_trace}"
umask 0000

cleanup() {
    trap - SIGTERM SIGINT EXIT
    echo "Stopping trace..."
    lttng stop "${SESSION}" || true
    lttng destroy "${SESSION}" || true
    chmod -R a+rwX "${TRACE_DIR}" 2>/dev/null || true
    echo "Trace saved to ${TRACE_DIR}"
    exit 0
}

trap cleanup SIGTERM SIGINT EXIT

echo "==> Starting LTTng tracing session (output: ${TRACE_DIR})"
lttng create "${SESSION}" --output="${TRACE_DIR}"
# Separate rcllf and ros2 channels with the contexts the
# lf_eval reader expects. This script writes no run manifest, so semantic
# claims over its sessions are incomplete(no_run_manifest) by design.
for channel in rcllf ros2; do
    lttng enable-channel -u "${channel}"
    lttng add-context -u -t vpid -t vtid -t procname -c "${channel}"
done
lttng enable-event -u 'rcllf:*' -c rcllf
lttng enable-event -u 'ros2:*' -c ros2
lttng start

echo "==> Tracing active. All ROS2 events across containers are being recorded."
echo "==> Press Ctrl+C or run: docker compose stop tracer"

sleep infinity &
wait $!
