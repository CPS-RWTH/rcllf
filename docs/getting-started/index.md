# Quickstart

Run the bundled `pubsub_example` under rcllf and check that it worked. The shortest
path is first; the sections after it explain each step and the plain ROS 2 run to
compare with. [Your First LF Run](first-lf-run.md) then opens every file rcllf
generates along the way.

## Supported setup

- An x86_64 Linux host with Docker and about 8 GB of free memory (the one tested path).
- ROS 2 Jazzy with C++ (`rclcpp`) nodes and a Python launch file.
- Fast DDS (`rmw_fastrtps_cpp`), which the container sets.
- All nodes of a launch file in one process, or one process per node
  (`--federate per-node`) under centralized coordination.

Everything else is experimental or not supported; the
[Support Matrix](../features/support-matrix.md) lists each feature and its evidence.

## Shortest path

You need Git, [Docker](https://docs.docker.com/get-docker/) 20.10+ and
[Docker Compose](https://docs.docker.com/compose/install/) v2. On the host:

```bash
git clone --branch v0.1.0 https://github.com/CPS-RWTH/rcllf.git
cd rcllf
git submodule update --init third_party/reactor-c
docker compose build dev
docker compose run --rm dev
```

The last command opens a shell in the container (the first start builds the runtime
and the examples, a few minutes). In that shell:

```bash
cd packages && source install/setup.bash
colcon lf --package pubsub_example
source install/setup.bash
ros2 lf launch pubsub_example bringup
```

### What success looks like

`colcon lf` ends its build with `Summary: 4 packages finished`. The launch then prints
this (trimmed, from a run of exactly these commands on a fresh checkout; your
timestamps differ):

```text
[Bringup-1] [INFO] [...] [ros2_lf_binding.corr]: rcllf corr: admitted edge 0 PublisherNode -> SubscriberNode on '/sensor_data' (d = 0 ns, ...
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound subscription RID 1 (SubscriberNode::topic_callback, topic 'sensor_data', resolved '/sensor_data', take_mode=skip timeout_ms=100)
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (PublisherNode::timer_callback, period 500000000 ns)
[Bringup-1] [INFO] [...] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[Bringup-1] [INFO] [...] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
[Bringup-1] [INFO] [...] [publisher_node]: Publishing: temp=21.0, humidity=41.0, pressure=1014.0
[Bringup-1] [INFO] [...] [subscriber_node]: I heard: temp=21.0, humidity=41.0, pressure=1014.0
```

Three things say it worked:

- Both nodes run in **one** process, `Bringup-1`: the generated LF program.
- The two `Bound` lines: rcllf attached the generated program to the node's timer and
  subscription callbacks.
- Every `Publishing:` line is followed by an `I heard:` line with the **same** values,
  counting up by one every 500 ms. The `admitted edge` line says this topic carries
  publication correspondence: the subscriber gets exactly the sample of the publish at
  the same logical time, and a missing sample would be reported instead of skipped.

Stop it with Ctrl-C. To check a saved log of this run mechanically, run
`python3 scripts/check_tutorial_output.py <log>` from the checkout: it checks the two
`Bound` lines and that each publication is heard once, with the same values, in order.

### If it does not look like this

- The container stops at start and prints a `git submodule` command: run it on the
  host, it fetches the Lingua Franca C runtime.
- `colcon lf` fails: its last lines name the step (analyze, generate or build) and the
  file. [Troubleshooting](../reference/troubleshooting.md) lists the known messages.
- The launch exits by itself: the last `[ERROR]` or `[WARN]` lines before the exit
  name the reason; look it up in [Troubleshooting](../reference/troubleshooting.md).
- No `I heard:` lines, or values that skip: see
  [`ros2_lf_bind_node: ... Refusing to start.`](../reference/troubleshooting.md#ros2_lf_bind_node-refusing-to-start)
  and [A subscriber callback sees stale data, or none](../reference/troubleshooting.md#a-subscriber-callback-sees-stale-data-or-none).
- Other ROS 2 systems on your network can interfere: start the container with your
  own domain, `ROS_DOMAIN_ID=37 docker compose run --rm dev`.

## The steps in detail

### The source and the image

`--branch v0.1.0` checks out the release; drop it to follow the default branch. The
`git submodule` line fetches the Lingua Franca C runtime (`reactor-c`), which the
container builds from. If you skip it, the container stops at start and prints that
same command.

`docker compose build dev` builds the image with ROS 2 Jazzy, the Lingua Franca
compiler and the rcllf tools. Instead of building it, you can pull the image built
from the same release and give it the name Compose uses:

```bash
docker pull ghcr.io/cps-rwth/rcllf:0.1.0-deps
docker tag ghcr.io/cps-rwth/rcllf:0.1.0-deps ghcr.io/cps-rwth/rcllf:dev
```

Use the image of the release you checked out: the container builds your checkout's
runtime against the patched `reactor-c` in the image, and `colcon lf-generate` refuses
an image older than the checkout's `third_party/patches`. Do not use
`docker compose pull` for this; it fetches the `dev` tag, which is a different build
target (see the [Docker reference](../reference/docker.md)).

### The first start

On its first start, `docker compose run --rm dev` builds the runtime and the example
packages, then drops you into a shell in `/workspace` (your checkout, mounted). Later
starts skip the build when no source changed.

How long this takes, with the ROS 2 base image already pulled and no build cache:
`docker compose build dev` took 2 min 46 s to 3 min 9 s, the first start 3 min 27 s to
3 min 59 s (10 packages), and the example below printed its first output 6 min 19 s to
7 min 44 s after `git clone`. Two runs on x86_64 produced this range (2026-11-16 and
2026-11-17). The slower run used a 24-core AMD Ryzen 9 5900X host and limited the
container to 6 CPUs and 8 GB of memory, with `make -j4` and two colcon workers. A
machine with fewer cores takes longer. Without the base image, add its download.

The first start also installs the rcllf Python tools (`colcon_lf`, `ros2lf`,
`lf_eval`) with `pip install -e`, so about 60 lines of pip output scroll past once;
later starts skip this. If the build summary lists packages that "had stderr output",
those are compiler warnings, not errors.

Two settings you may want:

- `ROS_DOMAIN_ID` is 0 unless you set it on the host. If other ROS 2 systems run on
  your network, pick your own: `ROS_DOMAIN_ID=37 docker compose run --rm dev`.
- `ENTRYPOINT_PACKAGES=all` builds every package in the workspace at start, not only
  the runtime and the examples. See the [Docker reference](../reference/docker.md#entrypoint).

## Compare with plain ROS 2

The same package as plain ROS 2, two processes and no rcllf. Inside the container:

```bash
ros2 launch pubsub_example bringup.launch.py
```

A publisher node sends a sensor reading every 500 ms and a subscriber node logs it.
The output looks like this (timestamps differ):

```text
[subscriber_node-2] [INFO] [1794890217.114799166] [subscriber_node]: Subscriber node started.
[publisher_node-1] [INFO] [1794890217.114888826] [publisher_node]: Publisher node started.
[publisher_node-1] [INFO] [1794890217.614995995] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[subscriber_node-2] [INFO] [1794890217.615327216] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
[publisher_node-1] [INFO] [1794890218.114982784] [publisher_node]: Publishing: temp=21.0, humidity=41.0, pressure=1014.0
[subscriber_node-2] [INFO] [1794890218.115199214] [subscriber_node]: I heard: temp=21.0, humidity=41.0, pressure=1014.0
```

Stop it with Ctrl-C.
[Example Packages](../examples/index.md) lists the other examples.

The values are the same, but nothing ties a subscriber callback to a particular
publish: which sample it reads depends on when DDS delivered it.

## Next

[Your First LF Run](first-lf-run.md) repeats the rcllf run step by step and opens
what rcllf generated at each step. [Bring Your Own Code](../guides/bring-your-own-code.md)
converts your own package.
