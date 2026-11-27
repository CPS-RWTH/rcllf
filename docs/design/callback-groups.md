# Callback Groups

ROS 2 callback groups tell the executor which callbacks may run in parallel.
rcllf has no executor: LF runs every callback as a reaction. This page explains
how the generator maps each group type to reactors, and which ROS 2 behavior
LF cannot reproduce.

## Mapping table

| ROS 2 group | LF shape |
|---|---|
| `MutuallyExclusive` | **One sub-reactor per group.** All callbacks of the group are reactions of one reactor. LF never runs two reactions of one reactor at the same time, so the group's mutual exclusion holds without locks. |
| `Reentrant` | **One sub-reactor per callback** of the group, so callbacks of one Reentrant group can run in parallel with each other. Service and action callbacks of the group share one extra sub-reactor (see [below](#service-and-action-callbacks-in-groups)). |
| Default (no group given) | Same as `MutuallyExclusive`. Every timer, subscription, service and action endpoint without an explicit group goes into one sub-reactor, `<Class>_DefaultGroup`, like rclcpp's default callback group. |

Every node becomes an outer **shell reactor**. The shell:

- holds the node's external ports and forwards them to its sub-reactors;
- holds a `startup` reaction that calls `ros2_lf_bind_node` and then sets the
  `rcllf_bound` input of every sub-reactor;
- holds no callback reactions. All callbacks live in the sub-reactors.

## Parallelism

Callbacks in different sub-reactors **may** run in parallel. They do when two
things hold:

- the program has two or more LF worker threads. The number is the
  [`workers`](../features/execution-config.md#workers) setting. Unset, reactor-c
  picks it from the host's core count and the shape of the reaction graph;
- no connection orders the two reactions at that tag. A reaction that depends
  on another one's output always runs after it.

So two sub-reactors behave like two groups under a `MultiThreadedExecutor`, but
what each callback observes stays fixed by the program, not by thread timing.

## The callback group example

`callback_group_example` creates one MutuallyExclusive group with two timers,
two Reentrant groups with three timers between them, and a timer and a
subscription without a group:

```cpp
exclusive_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
reentrant_group_1_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
reentrant_group_2_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
```

Generate it inside the dev container:

```bash
colcon lf-analyze --package callback_group_example
colcon lf-generate --package callback_group_example
```

The generated shell (`CallbackGroupNodeSystem.lf`, trimmed) has one
sub-reactor for the MutuallyExclusive group, one per Reentrant timer, and the
default group:

```lf
reactor CallbackGroupNode {
    exclusive_group = new CallbackGroupNode_ExclusiveGroup()
    timer_reentrant1a_callback = new CallbackGroupNode_TimerReentrant1aCallback()
    timer_reentrant1b_callback = new CallbackGroupNode_TimerReentrant1bCallback()
    timer_reentrant2_callback = new CallbackGroupNode_TimerReentrant2Callback()
    default_group = new CallbackGroupNode_DefaultGroup()
    ...
}

reactor CallbackGroupNode_ExclusiveGroup {
    ...
    timer t_timer_exclusive1_callback(0, 500 msec)
    timer t_timer_exclusive2_callback(0, 500 msec)

    reaction(t_timer_exclusive1_callback) rcllf_bound {=
    ...
    reaction(t_timer_exclusive2_callback) rcllf_bound {=
    ...
}

reactor CallbackGroupNode_TimerReentrant1aCallback {
    ...
    timer t_timer_reentrant1a_callback(0, 500 msec)

    reaction(t_timer_reentrant1a_callback) rcllf_bound {=
    ...
}
```

The two exclusive timers are reactions of one reactor, so they never overlap.
At a shared tag they run in declaration order. Each reentrant timer has its own
reactor and may run in parallel with the others.

## Service and action callbacks in groups

| Group type | Where service and action reactions go |
|------------|--------------------------------------|
| `MutuallyExclusive` or default | Into the group's sub-reactor, next to its timers and subscriptions. |
| `Reentrant` | Into one extra sub-reactor per group, `<Class>_<Group>Sa`, which acts as a MutuallyExclusive group. A service or action callback cannot get a sub-reactor of its own, because its request, response, goal, feedback and result reactions share state. |

A service or action client without an explicit group goes into the default
group, like any other endpoint.

## The self-concurrency restriction

A ROS 2 Reentrant group also lets **one callback run twice at once**, for
example the same subscription callback on two threads with two messages.

**LF does not support this.** A reaction runs at most once per tag, and a
reactor never runs two of its reactions at once. Two parallel runs of one
callback would need two reactor instances.

The generator accepts the restriction and prints a `UserWarning` for every
reentrant callback. From the generate step above:

```text
UserWarning: Node 'CallbackGroupNode': reentrant callback 'timer_reentrant1a_callback' cannot be invoked concurrently with itself under LF (reactor instances are singletons). If parallel self-invocation is required, declare a reactor bank in your .lf source. See docs/callback-groups.md#reactor-bank-escape-hatch.
```

The section the warning means is the next one on this page.

## Reactor bank escape hatch {#reactor-bank-escape-hatch}

If you need *N* parallel instances of the **same** callback, for example a
pool of workers that each serve one queue, LF offers a **reactor bank**: a
declaration such as `workers = new[4] Worker()` creates four instances of one
reactor, and different instances can run at the same tag in parallel.

rcllf does not generate banks. Keep these limits in mind:

- The generator never infers a bank from a ROS 2 callback group.
- A bank means editing the generated `.lf` file by hand, and every
  `colcon lf-generate` run overwrites that file. Keep such an edit outside the
  generated output and apply it again after each generation.
- Your code must tolerate being one of *N* instances. It must not rely on
  shared state such as one counter or member variable for all instances.
