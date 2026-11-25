# callback_group_example

Demonstrates ROS 2 callback groups (MutuallyExclusive and Reentrant) and how
the LF generator maps them to reactor structure.

## Pattern

Six callbacks across one MutuallyExclusive group, two Reentrant groups, and one
implicit (synthesized) default group. All timers are 500 ms with a CPU-busy
work body so the log interleaving is visible. Every callback prints
`[name] tick N start` / `[name] tick N end` around the busy work.

| Group | Type | Callbacks |
|-------|------|-----------|
| *(none, synthesized as `default_group`)* | MutuallyExclusive | `timer_default_callback`, `chatter_callback` |
| `exclusive_group_` | MutuallyExclusive | `timer_exclusive1_callback`, `timer_exclusive2_callback` |
| `reentrant_group_1_` | Reentrant | `timer_reentrant1a_callback`, `timer_reentrant1b_callback` |
| `reentrant_group_2_` | Reentrant | `timer_reentrant2_callback` |

The example uses `MultiThreadedExecutor` under stock ROS 2 so the parallelism
and serialization are observable in the log output.

## Source (excerpt)

```cpp
CallbackGroupNode::CallbackGroupNode(const rclcpp::NodeOptions & options):
Node("callback_group_node", options)
{
  exclusive_group_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  reentrant_group_1_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  reentrant_group_2_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // No group → falls into the synthesized default_group
  timer_default_   = create_wall_timer(500ms, [this]{ timer_default_callback(); });

  timer_exclusive1_ = create_wall_timer(
      500ms, [this]{ timer_exclusive1_callback(); }, exclusive_group_);
  timer_exclusive2_ = create_wall_timer(
      500ms, [this]{ timer_exclusive2_callback(); }, exclusive_group_);

  timer_reentrant1a_ = create_wall_timer(
      500ms, [this]{ timer_reentrant1a_callback(); }, reentrant_group_1_);
  timer_reentrant1b_ = create_wall_timer(
      500ms, [this]{ timer_reentrant1b_callback(); }, reentrant_group_1_);
  timer_reentrant2_  = create_wall_timer(
      500ms, [this]{ timer_reentrant2_callback(); },  reentrant_group_2_);

  // Subscription has no explicit group → also synthesized default_group
  subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, [this](auto msg){ chatter_callback(msg); });
}
```

## What the Extractor Finds

- `CallbackGroupInfo` entries: 1 `MutuallyExclusive`, 2 `Reentrant`, plus a
  synthesized `default_group` (MutuallyExclusive) for the timer and subscription
  with no explicit group.
- `TimerInfo.callback_group` references the group by member-variable name.

## Generated LF Structure

```
CallbackGroupNode (shell)
├── default_group     = CallbackGroupNode_DefaultGroup        [Mutex (synthesized)]
│   ├── timer t_timer_default_callback(0, 500 msec)
│   ├── input chatter: stub_t
│   ├── reaction(t_timer_default_callback)
│   └── reaction(chatter)
├── exclusive_group   = CallbackGroupNode_ExclusiveGroup      [Mutex]
│   ├── timer t_timer_exclusive1_callback(0, 500 msec)
│   ├── timer t_timer_exclusive2_callback(0, 500 msec)
│   ├── reaction(t_timer_exclusive1_callback)
│   └── reaction(t_timer_exclusive2_callback)
├── timer_reentrant1a_callback = CallbackGroupNode_TimerReentrant1aCallback   [Reentrant → own instance]
│   └── reaction(t_timer_reentrant1a_callback)
├── timer_reentrant1b_callback = CallbackGroupNode_TimerReentrant1bCallback   [Reentrant → own instance]
│   └── reaction(t_timer_reentrant1b_callback)
└── timer_reentrant2_callback  = CallbackGroupNode_TimerReentrant2Callback    [Reentrant → own instance]
    └── reaction(t_timer_reentrant2_callback)
```

**Serialization within Mutex groups.** `default_group` serializes
`timer_default` with `chatter`, and `exclusive_group` serializes `exclusive1` with
`exclusive2`. Their `tick N start` / `tick N end` markers will not interleave.

**Parallelism across groups.** Sibling sub-reactor instances run in parallel
under reactor-c's multi-threaded scheduler. The two reactors in
`reentrant_group_1` (`reentrant1a` and `reentrant1b`) are siblings and can
overlap, and the same applies to `reentrant_group_2`'s single reactor relative to
all the others.

## Self-Concurrency Note

The generator emits a `UserWarning` for each callback that lives in a Reentrant
group. LF cannot invoke the same reaction twice simultaneously, so a Reentrant
group of one callback (e.g. `reentrant_group_2_`) does not gain any
concurrency over a Mutex group with the same single callback. If you genuinely
need N parallel copies of the same callback, see [Callback Groups: Reactor
Bank Escape Hatch](../design/callback-groups.md#reactor-bank-escape-hatch).

## Run

```bash
docker compose run --rm dev ros2 launch callback_group_example callback_group_node.launch.py
```

## Further Reading

- [Callback Groups](../design/callback-groups.md), full mapping rules and escape hatch documentation.
