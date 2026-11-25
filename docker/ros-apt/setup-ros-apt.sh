#!/usr/bin/env bash
# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
# Configure apt to install ROS 2 Jazzy from the dated snapshot in pins.txt.
# Refuses a signing key whose fingerprint does not match.
set -euo pipefail
dir="$(cd "$(dirname "$0")" && pwd)"
pins="$dir/pins.txt"
snapshot="$(sed -n 's/^snapshot=//p' "$pins")"
fpr="$(sed -n 's/^key_fingerprint=//p' "$pins")"
[ -n "$snapshot" ] && [ -n "$fpr" ] || { echo "setup-ros-apt: snapshot/key_fingerprint missing in $pins" >&2; exit 1; }
got="$(gpg --show-keys --with-colons "$dir/ros-snapshots.asc" | awk -F: '/^fpr:/{print $10; exit}')"
if [ "$got" != "$fpr" ]; then
    echo "setup-ros-apt: key fingerprint $got does not match pinned $fpr" >&2
    exit 1
fi
gpg --dearmor < "$dir/ros-snapshots.asc" > /usr/share/keyrings/ros-snapshots.gpg
codename="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-snapshots.gpg] http://snapshots.ros.org/jazzy/$snapshot/ubuntu $codename main" \
    > /etc/apt/sources.list.d/ros2.list
echo "setup-ros-apt: ROS apt source = snapshots.ros.org/jazzy/$snapshot"
