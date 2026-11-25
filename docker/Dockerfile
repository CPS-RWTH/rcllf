# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

# =============================================================================
# Stage 1: base — ROS2 Jazzy on Microsoft Ubuntu 24.04
# =============================================================================
FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Add ROS2 apt repository
# ROS 2 comes from ONE dated snapshot of the ROS apt repository, so a rebuild
# installs the same binaries (source of truth docker/ros-apt/pins.txt,
# upgrade procedure in its header). check-pins.sh fails the build if the
# publication-identity packages differ from the pins.
COPY docker/ros-apt/ /opt/rcllf/ros-apt/
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg2 \
        lsb-release \
        software-properties-common \
    && /opt/rcllf/ros-apt/setup-ros-apt.sh \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
    && /opt/rcllf/ros-apt/check-pins.sh \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep. The update is retried because it exits non-zero if any
# single source in the rosdistro index fails to fetch, which turns a transient
# reset on raw.githubusercontent.com into a failed image build. The
# resulting cache in /root/.ros is also the fallback index that
# scripts/rosdep_install.sh reaches for when CI cannot refresh it.
RUN rosdep init || true
RUN for attempt in 1 2 3; do \
        rosdep update --rosdistro=jazzy && break; \
        echo "rosdep update failed (attempt ${attempt}/3)"; \
        [ "${attempt}" = 3 ] && exit 1; \
        sleep $((attempt * 5)); \
    done

# Source ROS2 in all bash sessions
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# =============================================================================
# Stage 2: tracing — adds LTTng and ROS2 tracing tools
# =============================================================================
FROM base AS tracing

RUN apt-get update && apt-get install -y --no-install-recommends \
        lttng-tools \
        liblttng-ust-dev \
        python3-lttng \
        babeltrace2 \
        python3-bt2 \
        ros-jazzy-tracetools \
        ros-jazzy-tracetools-trace \
        stress-ng \
    && rm -rf /var/lib/apt/lists/* \
    && usermod -aG tracing vscode

# =============================================================================
# Stage 3: dev — full development tools, linters, and formatters
# =============================================================================
FROM tracing AS dev

# Install ROS2 linting and formatting tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ament-lint-auto \
        ros-jazzy-ament-lint-common \
        ros-jazzy-ament-cmake-clang-format \
        ros-jazzy-ament-clang-format \
        ros-jazzy-ament-cmake-uncrustify \
        ros-jazzy-ament-uncrustify \
        ros-jazzy-ament-cmake-cpplint \
        ros-jazzy-ament-cpplint \
        ros-jazzy-ament-cmake-flake8 \
        ros-jazzy-ament-flake8 \
        ros-jazzy-ament-cmake-xmllint \
        ros-jazzy-ament-cmake-lint-cmake \
        ros-jazzy-ament-cmake-copyright \
        ros-jazzy-ament-cmake-cppcheck \
        ros-jazzy-ament-cmake-pep257 \
        ros-jazzy-ament-pep257 \
    && rm -rf /var/lib/apt/lists/*

# Install additional development tools. libclang comes from the pip wheel below
# (it bundles its own libclang.so), so the apt libclang-dev is not installed.
# stress-ng is installed by the tracing stage.
# nlohmann-json3-dev is a build dependency of ros2_lf_binding (package.xml).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ccache \
        clang-format \
        cppcheck \
        uncrustify \
        python3-flake8 \
        python3-pytest \
        graphviz \
        cmake \
        build-essential \
        nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# lf-eval runtime/test deps, apt-managed (the package itself is pip-installed
# editable from the workspace by entrypoint.sh; baking the heavy deps here
# keeps container startup fast). These MUST come from apt, not pip: the
# tracing stage's python3-bt2 pulls in apt's numpy, which pip cannot
# uninstall/upgrade (no RECORD file), so any pip install that resolves a
# numpy-dependent wheel (matplotlib, scipy) fails the build.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-matplotlib \
        python3-scipy \
        python3-yaml \
        python3-pytest \
        python3-pytest-cov \
    && rm -rf /var/lib/apt/lists/*

# colcon-lf's two non-apt runtime deps, baked here so entrypoint.sh can install
# the four workspace tools with --no-deps (sub-second, no network) instead of
# re-resolving them on every container start — libclang alone is a 24.5 MB wheel
# that was being downloaded every single `docker compose run`.
#
# pip is correct for these two (unlike the apt-managed scientific stack above):
# neither has a numpy dependency, so neither can trip the python3-bt2/numpy
# conflict described in that block, and neither is packaged for this base image.
#
# Keep these version constraints in sync with tools/colcon_lf/pyproject.toml.
# The remaining declared deps of the tools/ packages are already
# present: colcon-core (apt), pyyaml/matplotlib/scipy/pytest/pytest-cov (apt,
# above), ros2cli (ROS 2 Jazzy).
RUN pip install --break-system-packages --no-cache-dir \
        "libclang>=16.0" \
        "graphviz>=0.20"

# Install MkDocs for documentation — versions pinned in requirements-docs.txt,
# the single source shared with the CI docs job.
COPY requirements-docs.txt /tmp/requirements-docs.txt
RUN pip install --break-system-packages --no-cache-dir \
        -r /tmp/requirements-docs.txt \
    && rm /tmp/requirements-docs.txt

# Install Java 21 JRE (required by lfc) and download lfc v0.12.1.
# lfc is a core part of the project workflow (colcon lf-generate / lf-build in
# the dev container, the lf and federation profiles and the CI runtime job), so
# it lives in the dev image rather than a separate toolchain image.
RUN apt-get update && apt-get install -y --no-install-recommends \
        openjdk-21-jre-headless \
        wget \
    && rm -rf /var/lib/apt/lists/*

# lfc itself is Java; the per-architecture bundles differ only in the SWT jar of
# the diagram tool. arm64 (Jetson Orin) takes the official aarch64
# bundle; both bundles are checked against the SHA-256 of the GitHub release.
RUN case "$(dpkg --print-architecture)" in \
        amd64) lfc_arch=x86_64; lfc_sha=97b7e5e66ceeedbfef75be17e57fd80bf28a9f206fae936d53004461d7377731 ;; \
        arm64) lfc_arch=aarch64; lfc_sha=965626b48a166d62ef38f36e33af5ffa1a9e0a08783b8b4f9c06a662b122bac8 ;; \
        *) echo "no lfc bundle for $(dpkg --print-architecture)" >&2; exit 1 ;; \
    esac \
    && wget -q \
        "https://github.com/lf-lang/lingua-franca/releases/download/v0.12.1/lf-cli-0.12.1-Linux-${lfc_arch}.tar.gz" \
        -O /tmp/lfc.tar.gz \
    && echo "${lfc_sha}  /tmp/lfc.tar.gz" | sha256sum -c - \
    && tar -xzf /tmp/lfc.tar.gz -C /opt \
    && mv "/opt/lf-cli-0.12.1-Linux-${lfc_arch}" /opt/lfc \
    && rm /tmp/lfc.tar.gz

# actionlint validates .github/workflows/** locally so CI is never the first
# environment to parse a workflow change.
RUN arch="$(dpkg --print-architecture)" \
    && wget -q "https://github.com/rhysd/actionlint/releases/download/v1.7.7/actionlint_1.7.7_linux_${arch}.tar.gz" \
        -O /tmp/actionlint.tar.gz \
    && tar -xzf /tmp/actionlint.tar.gz -C /usr/local/bin actionlint \
    && rm /tmp/actionlint.tar.gz

ENV PATH="/opt/lfc/bin:${PATH}"

# Build the RTI daemon from the reactor-c submodule and install it system-wide
# so federated launches can start their own RTI without an external service.
# Local reactor-c fixes live in third_party/patches/reactor-c/.
# The same patches go into the runtime lfc bundles in its core jar, which is
# what every generated federate compiles against.
# The --check step fails the build unless the jar's runtime has every patch
# applied and both the jar and the RTI carry the patch markers, so
# no image runs with a partly patched runtime.
COPY third_party/reactor-c /opt/reactor-c
COPY third_party/patches /opt/reactor-c-patches
RUN python3 /opt/reactor-c-patches/patch_lfc_runtime.py /opt/lfc \
    && sh /opt/reactor-c-patches/apply_reactor_c_patches.sh /opt/reactor-c \
    && cmake -S /opt/reactor-c/core/federated/RTI -B /tmp/rti-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/rti-build --parallel $(nproc) \
    && cmake --install /tmp/rti-build \
    && python3 /opt/reactor-c-patches/patch_lfc_runtime.py --check --rti /usr/local/bin/RTI /opt/lfc \
    && rm -rf /tmp/rti-build /opt/reactor-c /opt/reactor-c-patches

# =============================================================================
# Stage 4: deps — pre-install rosdep dependencies for workspace packages
# This layer caches package dependencies. Rebuild only when package.xml changes.
# =============================================================================
FROM dev AS deps

# Copy only package.xml files to leverage Docker layer caching.
# Dependencies change rarely; source code changes often.
COPY packages/src/ /tmp/ws_src/
COPY packages/lf-frame/ /tmp/ws_src/
RUN find /tmp/ws_src -not -name "package.xml" -not -type d -delete 2>/dev/null || true

# Install rosdep dependencies for all packages.
# example_interfaces must be installed explicitly: rosdep --ignore-src skips deps of
# packages whose source is in the scan path, but example_interfaces is not in the
# workspace — it is an external ROS2 package only available via apt.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-example-interfaces \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths /tmp/ws_src --ignore-src -r -y 2>/dev/null || true && \
    rm -rf /tmp/ws_src

# WORKSPACE sets both the image's default WORKDIR and the directory created
# here; docker-compose.yml passes the same value as a bind-mount target and
# as the WORKSPACE env var, so the two stay in sync without duplication.
ARG WORKSPACE=/workspace

# Create workspace, LTTng home and the pip --user tree writable by vscode.
# /home/vscode/.local is the compose `vscode-local` named volume; Docker seeds a
# fresh volume from the image's directory, so the ownership must be set here.
RUN mkdir -p "${WORKSPACE}" /tmp/lttng-home /home/vscode/.local/bin /home/vscode/.local/lib \
    && chown -R vscode:vscode "${WORKSPACE}" /tmp/lttng-home /home/vscode/.local
WORKDIR ${WORKSPACE}

# Entrypoint: a shim prefers the bind-mounted copy (so edits to
# docker/entrypoint.sh take effect without an image rebuild) and falls back to
# the copy baked in here for use outside compose.
COPY docker/entrypoint.sh /entrypoint.baked.sh
RUN chmod +x /entrypoint.baked.sh \
    && printf '#!/bin/bash\nm="${WORKSPACE:-/workspace}/docker/entrypoint.sh"\n[ -x "$m" ] && exec "$m" "$@"\nexec /entrypoint.baked.sh "$@"\n' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

USER vscode
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# =============================================================================
# Stage 5: lf-rti — Lingua Franca Run-Time Infrastructure (RTI) daemon
#
# The RTI coordinates logical time across LF federates over TCP.
# Build from source using the reactor-c submodule bundled in the repo.
# Usage: docker compose up rti
# =============================================================================
FROM base AS lf-rti

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake \
        build-essential \
        git \
    && rm -rf /var/lib/apt/lists/*

# Build lf-rti from the reactor-c submodule (already cloned into third_party/)
# Local reactor-c fixes live in third_party/patches/reactor-c/.
COPY third_party/reactor-c /opt/reactor-c
COPY third_party/patches /opt/reactor-c-patches

WORKDIR /opt/reactor-c

RUN sh /opt/reactor-c-patches/apply_reactor_c_patches.sh /opt/reactor-c \
    && cmake -S core/federated/RTI -B /tmp/rti-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/rti-build --parallel $(nproc) \
    && cmake --install /tmp/rti-build \
    && rm -rf /tmp/rti-build

USER vscode

EXPOSE 15045

ENTRYPOINT ["RTI"]
CMD ["-n", "2", "-p", "15045"]
