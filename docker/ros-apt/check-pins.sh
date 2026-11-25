#!/usr/bin/env bash
# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
# Print the installed versions of the publication-identity packages and fail if
# any differs from docker/ros-apt/pins.txt, using the pin lines
# of this machine's architecture.
set -euo pipefail
pins="$(cd "$(dirname "$0")" && pwd)/pins.txt"
arch="$(dpkg --print-architecture)"
fail=0
n=0
while read -r pkg pin_arch want; do
    [ "$pin_arch" = "$arch" ] || continue
    n=$((n + 1))
    got="$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null || echo '<not installed>')"
    if [ "$got" = "$want" ]; then
        echo "pinned  $pkg $got"
    else
        echo "DRIFT   $pkg installed=$got pinned=$want" >&2
        fail=1
    fi
done < <(grep -E '^ros-' "$pins")
[ "$n" -gt 0 ] || { echo "check-pins: no pins for architecture $arch in $pins" >&2; exit 1; }
[ "$fail" -eq 0 ] || { echo "check-pins: installed ROS binaries differ from $pins" >&2; exit 1; }
