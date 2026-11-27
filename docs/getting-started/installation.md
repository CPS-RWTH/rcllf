# Installation and Platforms

This page says where rcllf runs, what to install, which images exist, what the first
start does, and how to update. The short version: **rcllf runs in Docker, on x86_64
Linux.** Everything else on the list below is either experimental or not supported.

| Where | Status |
|---|---|
| x86_64 (amd64) Linux with Docker | Supported. Tested. |
| arm64, NVIDIA Jetson | Experimental. See [arm64 and NVIDIA Jetson](arm64-jetson.md). |
| A host with ROS 2 Jazzy and no Docker | Not supported. See [Without Docker](#without-docker). |
| macOS, Windows | Not tested. |

## Requirements

- **Git**, to clone the repository and its one submodule.
- **Docker 20.10 or later** with **Docker Compose v2 or later** (the `docker compose`
  command, not the old `docker-compose`). The docs were checked with Docker 29.10 and
  Compose v5.8.0. Check yours:

    ```bash
    docker --version
    docker compose version
    ```

    ```text
    Docker version 29.10.0, build fdd4b8e
    Docker Compose version v5.8.0
    ```

- **Disk.** The image is about 3.5 GB on disk (a download of about 850 MB when you pull it).
    The build output goes into your checkout under `packages/build` and `packages/install`.
  With most packages built, `packages/build` was 2.3 GB and `packages/install` 190 MB; the
  default package set needs less.
- **CPU and memory.** Plan for about 8 GB of free memory. The container build runs at most 2 packages at once with 3 compiler
  jobs each, so at most 6 compilers. A machine with fewer cores only takes longer.
- **Network on the first run**: to clone, and to pull or build the image.

The container shares the host's process and network namespaces (Compose sets `pid: host`,
`ipc: host` and `network_mode: host`), so other ROS 2 systems on your network can see its
nodes. Pick your own domain: `ROS_DOMAIN_ID=37 docker compose run --rm dev`.

## The images

One release publishes one image per build target, as `ghcr.io/cps-rwth/rcllf:<version>-<target>`,
for amd64 only:

| Tag | What it holds | Used by |
|---|---|---|
| `<version>-deps` | ROS 2 Jazzy, the Lingua Franca compiler `lfc`, the patched runtime, the RTI, tracing tools, linters and the packages' dependencies | `dev` and every Compose service except `rti`. **This is the one to pull.** |
| `<version>-dev` | The same without the dependency layer | Building on top of it |
| `<version>-tracing`, `<version>-base` | The lower layers | Building on top of them |
| `<version>-lf-rti` | The standalone RTI | The `rti` service (federation profile) |

Compose does not use a version tag. Every service that runs your code uses the one name
`ghcr.io/cps-rwth/rcllf:dev`, which locally means the `deps` target. You get that image in one of
two ways (the [Quickstart](index.md#the-source-and-the-image) shows both): build it with
`docker compose build dev`, or pull the release image and give it the name Compose uses:

```bash
docker pull ghcr.io/cps-rwth/rcllf:<version>-deps
docker tag ghcr.io/cps-rwth/rcllf:<version>-deps ghcr.io/cps-rwth/rcllf:dev
```

Use the image of the release you checked out. Do not run `docker compose pull`: it fetches
the published `dev` tag, which is a different target from the one Compose needs. The name
`ghcr.io/cps-rwth/rcllf:dev` is one tag per Docker host, so two checkouts of different
releases overwrite each other's image. The [Docker reference](../reference/docker.md) has the
full list of targets and services.

## Clone and first start

```bash
git clone --branch v0.1.2 https://github.com/CPS-RWTH/rcllf.git
cd rcllf
git submodule update --init third_party/reactor-c
docker compose build dev      # or pull and tag the image as above
docker compose run --rm dev
```

The last command opens a shell in the dev container, in `/workspace`, which is your checkout.
The [Quickstart](index.md) continues from there with `pubsub_example`.

### What the first start builds

The first `docker compose run --rm dev` builds the runtime, the learning-path examples
(`pubsub_example`, `timer_example`, `service_example`, `lifecycle_example`,
`fusion_example`, `brake_fusion`) and what they depend on, and the generated
`lf_generated_*` packages a `colcon lf` left in the workspace. Later starts build again only when a
file under `packages/src`, `packages/lf-frame` or `packages/lf-generated` changed.

A package outside that set is not built, and its Run command fails with `Package '<name>' not
found`. Name the packages you need with `ENTRYPOINT_PACKAGES`, which builds them and their
dependencies. Pass it with `-e`, because Compose does not forward it from your shell:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="action_example" dev ros2 launch action_example bringup.launch.py
docker compose run --rm -e ENTRYPOINT_PACKAGES="system_bringup parameter_example" dev
docker compose run --rm -e ENTRYPOINT_PACKAGES=all dev      # every package
```

Your own packages go in `packages/src/`; see [Bring Your Own ROS 2 Code](../guides/bring-your-own-code.md).
The variables of the entrypoint are in
[Environment variables](../reference/environment-variables.md).

## Without Docker

rcllf does not support an installation on a host. The dev image is more than ROS 2 Jazzy plus
a few tools. It holds:

- ROS 2 Jazzy from one dated snapshot of the ROS apt repository, so that a rebuild installs the
  same binaries.
- `lfc` 0.12.1, whose C runtime is patched with the files under `third_party/patches`, and an RTI
  built from the patched `reactor-c`. `colcon lf-generate` refuses an image older than the
  checkout's patches.
- Java 21 for `lfc`, `libclang` and `graphviz` for the analyzer, and the Python tools `colcon-lf`,
  `ros2lf` and `lf-eval`, installed from your checkout.

Reproducing that on a host is not a documented or tested recipe, so this page gives none.
Two parts do run on a host with Python 3: the documentation site, and the `lf_eval` tests
(see [CONTRIBUTING](https://github.com/CPS-RWTH/rcllf/blob/main/CONTRIBUTING.md)). The
analysis of your code needs the ROS 2 headers, so use the container for that.

## Update to a new release

Updating means a new checkout and a matching image. Your own packages under `packages/src/`
stay where they are.

```bash
git fetch --tags
git checkout v<new-version>
git submodule update --init third_party/reactor-c
docker compose build dev      # or pull <new-version>-deps and tag it as :dev
docker compose run --rm dev
```

The first start after the update rebuilds what changed. Generated packages come from the old
generator, so run `colcon lf` again for each system you converted. Read the
[changelog](https://github.com/CPS-RWTH/rcllf/blob/main/CHANGELOG.md) for what changed.

## Next

- [Quickstart](index.md): run `pubsub_example` and check the output.
- [arm64 and NVIDIA Jetson](arm64-jetson.md): build the image on an arm64 board.
- [Docker reference](../reference/docker.md): targets, services, profiles, the entrypoint.
- [FAQ](../guides/faq.md): the questions newcomers ask first.
