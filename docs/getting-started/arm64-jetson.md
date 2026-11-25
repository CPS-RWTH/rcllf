# arm64 and NVIDIA Jetson

The dev image and the workspace build natively on arm64. They were tested on an
NVIDIA Jetson Orin NX (8 Cortex-A78AE cores in two clusters of 4, JetPack 6,
L4T R36.4.7, Ubuntu 22.04 host, Docker 29). The image is the same Ubuntu 24.04 +
ROS 2 Jazzy image as on x86_64: CPU only, no CUDA, no L4T base image.

## Build the image on the board

Build on the board itself; no emulation and no registry are needed. On an Orin
NX the `deps` target took about 6 minutes.

```bash
docker buildx build --builder default --load --target deps \
  -t rcllf-arm64:deps -f docker/Dockerfile .
```

What differs from x86_64, all chosen from the build machine's architecture:

| Piece | arm64 choice |
|---|---|
| lfc 0.12.1 | the official `lf-cli-0.12.1-Linux-aarch64` bundle, checked against its SHA-256 |
| ROS 2 binaries | the same dated snapshot; `docker/ros-apt/pins.txt` holds one pin line per architecture, because the arm64 builds of a snapshot carry other build timestamps |
| MuJoCo 3.14.0 | `brake_fusion` and every other MuJoCo package download the `linux-aarch64` tarball and check its SHA-256 |
| actionlint | the `linux_arm64` release |

## Build the workspace

Start one long-lived container and build Release inside it:

```bash
docker run -d --name rcllf-bf --network host --pid host --ipc host \
  -e ENTRYPOINT_SKIP_BUILD=1 -v "$PWD":/workspace rcllf-arm64:deps sleep infinity
docker exec rcllf-bf bash -c 'source /opt/ros/jazzy/setup.bash && cd packages && \
  colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

On the Orin NX the full workspace (45 packages) built in about
15 minutes.

## Floating point

GCC on aarch64 contracts `a * b + c` into one fused multiply-add instruction by
default, which rounds once instead of twice. x86_64 builds without `-march` have
no FMA instruction, so the same source can give different last bits on the two
architectures. `brake_fusion` builds with `-ffp-contract=off` on both, so its
nodes round the same way on both.
Without the flag, GCC
put one `fmsub` into the simulator's integration step on aarch64; with it, none
of the seven component libraries contains a fused instruction, and the x86_64
machine code is unchanged.

## Pinning on Orin NX

The two clusters share no L3 cache. The board scripts run the application and
its stress-ng load on cluster 1 (cores 4-7) and leave cluster 0 (cores 0-3) to
the OS, Docker and the LF RTI. They pin per process with `taskset`, not with a
container cpuset, and record the cpusets in every run's `meta.json`.

## Two boards

Two-board runs of the brake_fusion program (the RTI and one federate on each board)
used a runner script that is not part of this release; start the federates by hand
as in [Federation](../features/federation.md).
