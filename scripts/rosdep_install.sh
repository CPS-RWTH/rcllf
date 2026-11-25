#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Resolve and install the workspace's rosdep dependencies, tolerating a
# transient failure of any single rosdep source.
#
# `rosdep update` exits non-zero if *any* source in the index fails to fetch,
# including sources this workspace cannot need (osx-homebrew.yaml is macOS
# package data and the CI container is Linux). It still writes a usable cache
# for the sources that did succeed, so a partial index is not a reason to fail
# the job.
#
# Three tiers:
#   1. a fresh `rosdep update`, retried a few times to absorb a reset;
#   2. the index baked into the image at build time (docker/Dockerfile), which
#      GitHub Actions otherwise hides by overriding HOME to /github/home. It is
#      complete by construction, since the image build fails if it is not;
#   3. whatever partial cache the failed update left in $HOME. Preferred last
#      precisely because it is partial. A reset on base.yaml leaves a cache
#      that exists, is fresh, and cannot resolve anything.
#
# The `rosdep install` call is left exactly as CI ran it before, `-r` included.
# Note what `-r` means for the signal here: an unresolvable key is a warning and
# `rosdep install` still exits 0 ("Continuing to install resolvable
# dependencies"), so this step never was the gate on a bad index. `colcon build`
# is, by failing on the missing dependency. That is why every fallback path
# below announces itself, so a later build failure is attributable to the index
# rather than looking like a code problem.
#
# Usage: ./scripts/rosdep_install.sh [from-paths-dir...]
#   (default: packages/src and packages/lf-frame, so a user package that
#   depends on an lf-frame package such as rcllf_message_filters resolves it
#   as source under --ignore-src, as the image build in docker/Dockerfile does)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ROSDISTRO="${ROS_DISTRO:-jazzy}"
if [ "$#" -gt 0 ]; then
    FROM_PATHS=("$@")
else
    FROM_PATHS=("$WS_DIR/packages/src" "$WS_DIR/packages/lf-frame")
fi
ATTEMPTS="${ROSDEP_UPDATE_ATTEMPTS:-3}"

# Index baked into the image by `rosdep update` in the Dockerfile's base stage.
# That runs as root with HOME=/root, so the cache lands outside the HOME a CI
# job sees.
BAKED_ROS_HOME=/root/.ros

# An index counts only if the cache directory holds at least one non-empty
# file. `rosdep update` creates the directory before it fetches anything, so
# testing for the directory alone would accept an empty index.
has_cache() {
    local cache="$1/rosdep/sources.cache"
    [ -d "$cache" ] && [ -r "$cache" ] || return 1
    [ -n "$(find "$cache" -type f -size +0 -print -quit 2>/dev/null)" ]
}

run_install() {
    echo "==> rosdep install --from-paths ${FROM_PATHS[*]}"
    rosdep install --from-paths "${FROM_PATHS[@]}" --ignore-src -r -y
    # `-r` keeps install tolerant, which also turns an unresolvable key into a
    # warning. `rosdep check` restores the gate: it exits non-zero when a
    # key cannot be resolved or a resolved dependency is not installed. It runs
    # in the same environment (ROS_HOME) as the install, so it checks against
    # the index that was actually used.
    echo "==> rosdep check --from-paths ${FROM_PATHS[*]}"
    rosdep check --from-paths "${FROM_PATHS[@]}" --ignore-src
}

update_succeeded=false
for attempt in $(seq 1 "$ATTEMPTS"); do
    echo "==> rosdep update (attempt $attempt/$ATTEMPTS)"
    if rosdep update --rosdistro="$ROSDISTRO"; then
        update_succeeded=true
        break
    fi
    if [ "$attempt" -lt "$ATTEMPTS" ]; then
        backoff=$((attempt * 5))
        echo "==> rosdep update failed, retrying in ${backoff}s"
        sleep "$backoff"
    fi
done

if [ "$update_succeeded" = true ]; then
    run_install
    exit 0
fi

echo "==> WARNING: rosdep update failed $ATTEMPTS times, falling back to a cached index."
echo "==> A partial index is not a reason to fail this job."

own_ros_home="${ROS_HOME:-$HOME/.ros}"
has_cache "$BAKED_ROS_HOME" && baked=true || baked=false
has_cache "$own_ros_home" && partial=true || partial=false

if [ "$baked" = false ] && [ "$partial" = false ]; then
    echo "==> ERROR: rosdep update failed and no usable cached index exists." >&2
    echo "==> Checked $own_ros_home and $BAKED_ROS_HOME." >&2
    exit 1
fi

note_fallback() {
    echo "==> NOTE: dependencies were resolved from a fallback index, not a fresh"
    echo "==> rosdep update. If the build now fails on a missing dependency, this"
    echo "==> is the first thing to suspect."
}

if [ "$baked" = true ]; then
    echo "==> Using the image's baked index in $BAKED_ROS_HOME (complete, built with the image)."
    note_fallback
    # Subshell so ROS_HOME does not leak into the partial-index retry below.
    if (export ROS_HOME="$BAKED_ROS_HOME"; run_install); then
        exit 0
    fi
    if [ "$partial" = false ]; then
        echo "==> ERROR: rosdep install failed against the baked index." >&2
        exit 1
    fi
    echo "==> rosdep install failed against the baked index, which may predate a new"
    echo "==> dependency. Retrying with the partial index in $own_ros_home."
fi

echo "==> Using the partial index in $own_ros_home."
note_fallback
run_install
