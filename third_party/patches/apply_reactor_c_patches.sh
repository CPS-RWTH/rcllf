#!/bin/sh
# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Apply the local reactor-c patches (third_party/patches/reactor-c/*.patch, in
# name order) to a COPY of the pinned third_party/reactor-c submodule. The
# submodule checkout itself stays pristine. Used by docker/Dockerfile before
# the RTI build and by tools/colcon_lf/tests/test_rti_accept_timeout.py.
#
# A patch that is already in the tree (a forward dry run reports it as
# "previously applied", as patch_lfc_runtime.py checks) is skipped, so a
# runtime patched by an older image gets only the patches it lacks.
#
# Usage: apply_reactor_c_patches.sh <reactor-c-copy-dir>
set -eu
target=${1:?usage: apply_reactor_c_patches.sh <reactor-c-copy-dir>}
patch_dir=$(dirname "$0")/reactor-c
for p in "$patch_dir"/*.patch; do
  [ -e "$p" ] || continue
  if out=$(patch -p1 --forward --dry-run --batch -d "$target" < "$p" 2>&1); then
    echo "reactor-c: applying $(basename "$p")"
    patch -p1 --forward --batch -d "$target" < "$p"
  elif printf '%s\n' "$out" | grep -q "previously applied"; then
    echo "reactor-c: $(basename "$p") already applied"
  else
    printf '%s\n' "$out"
    echo "reactor-c: $(basename "$p") does not apply" >&2
    exit 1
  fi
done
