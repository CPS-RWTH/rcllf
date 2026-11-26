---
title: rcllf
hide:
  - navigation
  - toc
---

<div class="rlf-hero" markdown>

# rcllf

<p class="rlf-tagline"><strong>Deterministic ROS 2 nodes, coordinated by Lingua Franca.</strong></p>

[Get Started :material-arrow-right:](getting-started/index.md){ .md-button .md-button--primary }

</div>

rcllf reads a ROS 2 Jazzy C++ application, its node sources and its launch file, and
generates a [Lingua Franca](https://www.lf-lang.org/) (LF) program that calls your
existing callbacks in logical time. Your node code stays unchanged, and ROS 2 still
carries the messages over DDS.

## Try it

With Git, Docker and Docker Compose on an x86_64 Linux host:

```bash
git clone --branch v0.1.1 https://github.com/CPS-RWTH/rcllf.git
cd rcllf
git submodule update --init third_party/reactor-c
docker compose build dev
docker compose run --rm dev
# inside the container:
cd packages && source install/setup.bash
colcon lf --package pubsub_example && source install/setup.bash
ros2 lf launch pubsub_example bringup
```

Each `Publishing:` line is followed by an `I heard:` line with the same values.
The [Quickstart](getting-started/index.md) shows the full expected output and what
to do if it differs. What runs, on which setup:
[Supported setup](getting-started/index.md#supported-setup) and the
[Support Matrix](features/support-matrix.md).

## What becomes repeatable

<div class="grid cards" markdown>

-   :material-sort-numeric-ascending:{ .lg .middle } **Firing order**

    ---

    Callbacks run in logical-time order. At one logical time, a node's callbacks
    and every callback that receives another's message run in an order the
    generated program fixes, not thread scheduling or DDS arrival. Unconnected
    nodes may run in parallel. In one process and under centralized federation.

-   :material-email-check-outline:{ .lg .middle } **Sample selection**

    ---

    On topic connections that meet the
    [publication correspondence](design/time-and-coordination.md#publication-correspondence-topic-edges)
    criteria, a callback gets exactly the sample its publish produced, and a
    missing or late sample is reported. Other connections take what DDS delivered.

-   :material-timer-sand:{ .lg .middle } **Not wall-clock time**

    ---

    When a callback runs and how long it takes still depend on the OS, the CPU
    and DDS, as in stock ROS 2.

</div>

The exact guarantees and their conditions:
[Three kinds of determinism](design/time-and-coordination.md#three-kinds-of-determinism).

## Next

- [Your First LF Run](getting-started/first-lf-run.md): the same example step by step,
  with the node source next to the LF program rcllf generated from it.
- [Bring Your Own Code](guides/bring-your-own-code.md): convert your own package, in
  this checkout or in a workspace of your own.
- [Design](design/design-principles.md): why rcllf exists, its tradeoffs, and how it
  compares to other real-time ROS 2 work.
