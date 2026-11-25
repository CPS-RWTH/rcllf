# Callback Groups

ROS 2 callback groups tell the executor which callbacks may run in parallel.
Under LF, scheduling is reactor-based, and this page explains how the generator
maps each group type to a reactor structure and what restrictions apply.

---

## Mapping Table

| ROS 2 group | LF emission |
|---|---|
| `MutuallyExclusive` (named or `default`) | **One sub-reactor per group.** All callbacks in the group share the same reactor instance. LF serializes reactions within a reactor, so within-group mutual exclusion is an LF invariant, needing no locks. |
| `Reentrant` | **One sub-reactor per callback** in the group. Each sub-reactor is a singleton instance, and reactor-c's multi-threaded scheduler runs sibling reactor instances in parallel, preserving cross-callback concurrency within the group. |
| default / implicit (no `create_callback_group`) | Same as `MutuallyExclusive`. The extractor synthesizes a `"default_group"` MutuallyExclusive group for any entity (timers, subscribers, service servers, service clients, action servers, action clients) not explicitly assigned to a group, matching rclcpp's `node_base_->get_default_callback_group()` behaviour. |

Every node reactor is a **shell** that:

- Holds all external ports (inputs / outputs for topics this node subscribes to or publishes on).
- Holds a `startup` reaction that calls `ros2_lf_bind_node`.
- Holds port-forwarding connections between its external ports and its group sub-reactors.
- Contains **no user reactions**, since all reactions live inside group sub-reactors.

Sibling sub-reactors are independent reactor instances under reactor-c, so
**callbacks in different groups always run in parallel**, matching the
behaviour of `MultiThreadedExecutor` in ROS 2.

---

## MutuallyExclusive Group

A MutuallyExclusive group serializes its callbacks, so at most one can execute at
a time.  In LF this is automatic when all callbacks belong to the same reactor,
because a reactor executes its reactions one at a time.

**Example: two timers in one Mutex group**

```cpp
exclusive_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
timer_a_ = create_wall_timer(500ms, [this]{ timer_a_callback(); }, exclusive_group_);
timer_b_ = create_wall_timer(1000ms, [this]{ timer_b_callback(); }, exclusive_group_);
```

Generated `.lf` (simplified):

```lf
reactor MyNode {
  exclusive_group = new MyNode_ExclusiveGroup()
  // ...startup and port-forwarding connections...
}

reactor MyNode_ExclusiveGroup {
  timer t_timer_a_callback(0, 500 msec)
  timer t_timer_b_callback(0, 1000 msec)

  reaction(t_timer_a_callback) {= ros2_lf_invoke_timer(self, RID_...); =}
  reaction(t_timer_b_callback) {= ros2_lf_invoke_timer(self, RID_...); =}
}
```

`timer_a_callback` and `timer_b_callback` are serialized because they both
belong to `MyNode_ExclusiveGroup` and LF never runs two reactions of the same
reactor concurrently.

---

## Reentrant Group

A Reentrant group allows its callbacks to run concurrently with each other.
In LF, each callback gets its own dedicated reactor instance, and reactor-c runs
sibling instances in parallel under a multi-threaded schedule.

**Example: timer in a Reentrant group**

```cpp
reentrant_group_ = create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
timer_c_ = create_wall_timer(750ms, [this]{ timer_c_callback(); }, reentrant_group_);
```

Generated `.lf` (simplified):

```lf
reactor MyNode {
  timer_c_callback = new MyNode_TimerCCallback()
  // ...startup and port-forwarding...
}

reactor MyNode_TimerCCallback {
  timer t_timer_c_callback(0, 750 msec)
  reaction(t_timer_c_callback) {= ros2_lf_invoke_timer(self, RID_...); =}
}
```

`timer_c_callback` runs in its own reactor instance alongside the
`ExclusiveGroup` sub-reactor, since they are siblings and reactor-c schedules them
in parallel.

---

## Cross-Group Parallelism

Because each group becomes a distinct reactor instance, **callbacks in different
groups always run in parallel**, regardless of their type.  This matches
`MultiThreadedExecutor` semantics in ROS 2:

```
MyNode (shell)
├── exclusive_group  = MyNode_ExclusiveGroup()   ─── timer_a, timer_b serialize
└── timer_c_callback = MyNode_TimerCCallback()   ─── runs in parallel with exclusive_group
```

A second MutuallyExclusive group would appear as a third sibling sub-reactor
and also run in parallel with the others.

---

## Service and Action Reactions Inside Groups

Service and action callbacks honour callback-group assignment the same way
timers and subscriptions do, with one wrinkle in the Reentrant case.

| Group type | Where service / action reactions land |
|------------|----------------------------------------|
| `MutuallyExclusive` (or synthesized `default_group`) | Merged into the group's sub-reactor alongside any timers and subscriptions in the group. |
| `Reentrant` | Service and action reactions go into a separate `<NodeClass>_<GroupName>_sa` sub-reactor that is itself MutuallyExclusive. Per-callback singletons (the trick used for reentrant timers and subs) cannot be applied to service/action callbacks because they share request/response state across the goal/feedback/result reactions. |
| Group with only service / action callbacks | Still gets its own sub-reactor, since there is always at least one. |

Implicit-default unification covers service clients and action clients too. If a
client has no explicit group, the extractor assigns it to the synthesized
`default_group` along with timers and subscriptions in the same situation. The
`async_send_request` call style is detected by a classifier that checks for the
async-callback pattern *before* the blocking-future pattern, so a lambda
containing `future.get()` inside `async_send_request` is correctly classified
as async.

---

## The Self-Concurrency Restriction

ROS 2 Reentrant groups also permit a **single callback to run twice
concurrently**, the same subscription callback on two threads with two
different messages at the same wall-clock moment.

**LF does not support this.** A single reaction fires at most once per logical
tag.  Two simultaneous invocations of the *same* callback require two distinct
reaction instances.

The generator accepts this restriction and emits a `UserWarning` for every
reentrant callback to make the trade-off explicit:

```
UserWarning: Node 'MyNode': reentrant callback 'timer_c_callback' cannot be
invoked concurrently with itself under LF (reactor instances are singletons).
If parallel self-invocation is required, declare a reactor bank in your .lf
source.  See docs/callback-groups.md#reactor-bank-escape-hatch.
```

The Autoware reference system does not rely on self-concurrency, so this
restriction does not affect its mapping.

---

## Reactor Bank Escape Hatch {#reactor-bank-escape-hatch}

If you genuinely need *N* parallel instances of the **same** callback, for
example, a pool of worker callbacks each processing one of *N* queues, the
LF path is a **reactor bank**:

```lf
reactor Worker {
  input in: stub_t
  reaction(in) {= ros2_lf_invoke_sub(self, RID_WORKER_PROCESS, "/work"); =}
}

main reactor System {
  workers    = new[N] Worker()
  dispatcher = new Dispatcher()
  dispatcher.out -> workers.in
}
```

Each bank element is an independent reactor instance.  At a given logical tag,
different bank elements can fire concurrently.

**Important constraints:**

- User code must tolerate being one of *N* instances. It must not assume
  singleton state (e.g., shared counters or member variables).
- The generator **does not infer banks automatically** from ROS 2 callback
  groups.  Banks are an explicit source-level modelling decision.
- To use a bank, edit the generated `.lf` file directly, or supply an override
  annotation in the launch file (planned for a future phase).

