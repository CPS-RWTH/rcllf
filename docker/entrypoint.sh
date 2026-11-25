#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

set -e

# WORKSPACE is set by docker-compose.yml (x-common-env); default matches the
# Dockerfile's WORKDIR in case the entrypoint is invoked outside compose.
export WORKSPACE="${WORKSPACE:-/workspace}"

# A clone without submodules leaves third_party/reactor-c empty, and the
# runtime binding then fails deep inside CMake ("does not contain a
# CMakeLists.txt file"). Stop here with the one command that fixes it. Checked
# before anything else so it also holds for services that skip the build.
if [ -f "$WORKSPACE/.gitmodules" ] \
    && [ ! -f "$WORKSPACE/third_party/reactor-c/CMakeLists.txt" ]; then
    {
        echo "!!> third_party/reactor-c is empty: the repository was cloned without"
        echo "!!> its reactor-c submodule, which the LF runtime is built from."
        echo "!!> Fix it on the host, in the repository root, then start again:"
        echo "!!>     git submodule update --init third_party/reactor-c"
    } >&2
    exit 1
fi

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Ensure LTTNG_HOME exists (shared across containers via pid: host)
export LTTNG_HOME="${LTTNG_HOME:-/tmp/lttng-home}"
mkdir -p "$LTTNG_HOME"

# pip installs the colcon-lf/ros2lf/lf-eval console scripts below to
# the user site's bin dir (this container runs as non-root `vscode`, so pip
# always installs "--user" even without the flag). That directory is only on
# PATH in a login shell (~/.profile), not in a plain `bash -c "..."` command
# — which is exactly how docker-compose service `command:` blocks invoke this
# entrypoint's "$@". Export it here so every downstream command (interactive
# or compose-driven, login shell or not) can resolve these CLIs.
export PATH="$HOME/.local/bin:$PATH"

# Build workspace packages only when something changed since the last build.
# The stamp file is written after every successful colcon build and records
# the package selection; if the selection is the same and no file under the
# source trees is newer than it, the build is skipped and the existing overlay
# is sourced. Overrides:
#   ENTRYPOINT_PACKAGES=all   build every package in the workspace. The
#                             default ("examples") builds the runtime, the
#                             learning-path examples and their dependencies,
#                             plus any generated lf_generated_* package.
#   ENTRYPOINT_PACKAGES="a b" build these packages and their dependencies.
#   ENTRYPOINT_SKIP_BUILD=1   never build (even without a stamp)
#   ENTRYPOINT_FORCE_BUILD=1  always build
#   ENTRYPOINT_PARALLEL_WORKERS=N  colcon packages built at once (default 2)
#   MAKEFLAGS                 make jobs per package (default -j3; a caller's
#                             value is kept)
# The defaults cap a fresh build at 2 x 3 = 6 compiler processes: unbounded,
# colcon ran ~31 cc1plus at once and took 18 GiB of host memory.
# docker-compose.yml sets both in every service's environment, and the service
# commands that build pass the same worker count to colcon.
# A build failure must not prevent the container command from running: stale
# gitignored generator output (packages/lf-generated) would otherwise break
# every service start. --continue-on-error keeps unaffected packages building;
# the overlay is sourced with whatever succeeded.
STAMP="$WORKSPACE/packages/install/.entrypoint-stamp"
EXAMPLE_PACKAGES="ros2_lf_binding pubsub_example timer_example service_example"
EXAMPLE_PACKAGES="$EXAMPLE_PACKAGES lifecycle_example fusion_example brake_fusion"
build_selection="${ENTRYPOINT_PACKAGES:-examples}"
select_args=()
case "$build_selection" in
    all) ;;
    examples)
        # Only the packages that exist: a trimmed checkout may lack one.
        regex=""
        for pkg in $EXAMPLE_PACKAGES; do
            if find "$WORKSPACE/packages/src" "$WORKSPACE/packages/lf-frame" \
                    -name package.xml -path "*/$pkg/package.xml" -print -quit \
                    2>/dev/null | grep -q .; then
                regex="${regex:+$regex|}$pkg"
            fi
        done
        select_args=(--packages-up-to-regex "^(${regex}|lf_generated_.*)\$") ;;
    *)
        # shellcheck disable=SC2206  # word splitting is the point here
        select_args=(--packages-up-to $build_selection) ;;
esac
needs_build() {
    [ "${ENTRYPOINT_FORCE_BUILD:-0}" = "1" ] && return 0
    [ -f "$STAMP" ] || return 0
    [ "$(cat "$STAMP" 2>/dev/null)" = "$build_selection" ] || return 0
    [ -f "$WORKSPACE/packages/install/setup.bash" ] || return 0
    local changed
    changed=$(find "$WORKSPACE/packages/src" "$WORKSPACE/packages/lf-frame" \
                   "$WORKSPACE/packages/lf-generated" -newer "$STAMP" -print -quit \
                   2>/dev/null)
    [ -n "$changed" ]
}

if [ "${ENTRYPOINT_SKIP_BUILD:-0}" = "1" ]; then
    echo "==> Skipping colcon build (ENTRYPOINT_SKIP_BUILD=1)."
    if [ -f "$WORKSPACE/packages/install/setup.bash" ]; then
        source "$WORKSPACE/packages/install/setup.bash"
    fi
elif [ -d "$WORKSPACE/packages/src" ]; then
    cd "$WORKSPACE/packages"

    if needs_build; then
        build_workers="${ENTRYPOINT_PARALLEL_WORKERS:-2}"
        build_makeflags="${MAKEFLAGS:--j3}"
        echo "==> Building packages with colcon: $build_selection" \
             "(--parallel-workers $build_workers, MAKEFLAGS=$build_makeflags)..."
        if [ "$build_selection" = "examples" ]; then
            echo "==> (the runtime and the example packages; ENTRYPOINT_PACKAGES=all builds everything)"
        fi
        if MAKEFLAGS="$build_makeflags" colcon build --symlink-install \
            --continue-on-error --parallel-workers "$build_workers" \
            "${select_args[@]}" \
            --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON; then
            echo "==> Build complete."
            mkdir -p install && printf '%s\n' "$build_selection" > "$STAMP"
        else
            echo "!!> colcon build FAILED for some packages (see above)." >&2
            echo "!!> Continuing with the packages that did build." >&2
            echo "!!> If the failures are in packages/lf-generated, that output is" >&2
            echo "!!> stale. Regenerate it FROM INSIDE THIS CONTAINER:" >&2
            echo "!!>     rm -rf packages/lf-generated && colcon lf" >&2
            echo "!!> Re-running 'colcon lf' on the host does not help: system.json" >&2
            echo "!!> records absolute source paths, so generating where they do not" >&2
            echo "!!> resolve produces packages that fail to build." >&2
        fi
    else
        echo "==> Sources unchanged since last build; skipping colcon build" \
             "(ENTRYPOINT_FORCE_BUILD=1 to override)."
    fi

    if [ -f install/setup.bash ]; then
        echo "==> Sourcing workspace overlay..."
        source install/setup.bash
    fi
    cd "$WORKSPACE"
fi

# Install the repo's Python CLIs as editable packages. Skipped when the module
# already imports (e.g. ~/.local persisted via the compose `vscode-local`
# volume); ENTRYPOINT_FORCE_PIP=1 reinstalls.
install_tool() {
    local dir="$1" module="$2" extras="$3"
    [ -f "$WORKSPACE/tools/$dir/pyproject.toml" ] || return 0
    if [ "${ENTRYPOINT_FORCE_PIP:-0}" != "1" ] && python3 -c "import $module" 2>/dev/null; then
        return 0
    fi
    echo "==> Installing $dir..."
    pip install --break-system-packages --no-cache-dir --no-deps -e "$WORKSPACE/tools/$dir$extras"
}
install_tool colcon_lf colcon_lf "[test]"
install_tool ros2lf    ros2lf    ""
install_tool lf_eval   lf_eval   "[test,stats]"
# Any further tool package (development checkouts carry more), with its tests.
for tool_toml in "$WORKSPACE"/tools/*/pyproject.toml; do
    tool_dir=$(basename "$(dirname "$tool_toml")")
    case "$tool_dir" in colcon_lf|ros2lf|lf_eval) continue ;; esac
    install_tool "$tool_dir" "$tool_dir" "[test]"
done

# Development-only extras (absent from a release checkout, so these lines then
# do nothing): the benchmark tool and in-repo package hooks
# (packages/src/<pkg>/rcllf_hooks/pyproject.toml), which the benchmark tool
# finds only through the installed rcllf.hooks entry point. Installed editable
# like the tools above, and skipped when the distribution is already
# installed; ENTRYPOINT_FORCE_PIP=1 reinstalls.
for hooks_toml in "$WORKSPACE"/packages/src/*/rcllf_hooks/pyproject.toml; do
    [ -f "$hooks_toml" ] || continue
    hooks_dist=$(sed -n 's/^name *= *"\([^"]*\)".*/\1/p' "$hooks_toml" | head -n 1)
    if [ -n "$hooks_dist" ] && [ "${ENTRYPOINT_FORCE_PIP:-0}" != "1" ] \
        && pip show -q "$hooks_dist" >/dev/null 2>&1; then
        continue
    fi
    echo "==> Installing ${hooks_dist:-$(dirname "$hooks_toml")}..."
    pip install --break-system-packages --no-cache-dir --no-deps -e "$(dirname "$hooks_toml")"
done

exec "$@"
