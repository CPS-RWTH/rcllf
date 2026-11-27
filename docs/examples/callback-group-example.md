# callback_group_example

ROS 2 callback groups (MutuallyExclusive and Reentrant) and how rcllf maps them to
LF reactors: callbacks that must not overlap share one reactor, the others get
reactors of their own that may run in parallel.

## What it shows

Seven callbacks in one MutuallyExclusive group, two Reentrant groups, and the node's
default group. All six timers have a 500 ms period and a CPU-busy body, and every
callback prints `[name] tick N start` and `[name] tick N end` around the busy work, so
the log shows which callbacks overlap.

| Group | Type | Callbacks |
|-------|------|-----------|
| *(none, the node's default group)* | MutuallyExclusive | `timer_default_callback`, `chatter_callback` |
| `exclusive_group_` | MutuallyExclusive | `timer_exclusive1_callback`, `timer_exclusive2_callback` |
| `reentrant_group_1_` | Reentrant | `timer_reentrant1a_callback`, `timer_reentrant1b_callback` |
| `reentrant_group_2_` | Reentrant | `timer_reentrant2_callback` |

Under stock ROS 2 the node runs in a `MultiThreadedExecutor`, so the groups matter.
From `src/callback_group_node.cpp`:

```cpp
exclusive_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
reentrant_group_1_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
reentrant_group_2_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

timer_default_ = this->create_wall_timer(
  500ms, std::bind(&CallbackGroupNode::timer_default_callback, this));
timer_exclusive1_ = this->create_wall_timer(
  500ms, std::bind(&CallbackGroupNode::timer_exclusive1_callback, this), exclusive_group_);
timer_exclusive2_ = this->create_wall_timer(
  500ms, std::bind(&CallbackGroupNode::timer_exclusive2_callback, this), exclusive_group_);
timer_reentrant1a_ = this->create_wall_timer(
  500ms, std::bind(&CallbackGroupNode::timer_reentrant1a_callback, this), reentrant_group_1_);
timer_reentrant1b_ = this->create_wall_timer(
  500ms, std::bind(&CallbackGroupNode::timer_reentrant1b_callback, this), reentrant_group_1_);
timer_reentrant2_ = this->create_wall_timer(
  500ms, std::bind(&CallbackGroupNode::timer_reentrant2_callback, this), reentrant_group_2_);

subscription_ = this->create_subscription<std_msgs::msg::String>(
  "chatter", 10,
  std::bind(&CallbackGroupNode::chatter_callback, this, std::placeholders::_1));
```

## Run with stock ROS 2

The first start of the dev container does not build this package, so select it with
`ENTRYPOINT_PACKAGES`:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="callback_group_example" dev \
  ros2 launch callback_group_example callback_group_node.launch.py
```

Every 500 ms the callbacks start together; only `exclusive_1` waits until
`exclusive_2`, from its own group, has ended (trimmed; your timestamps and order
differ):

```text
[callback_group_node-1] [INFO] [...] [callback_group_node]: [reentrant_1b] tick 0 start
[callback_group_node-1] [INFO] [...] [callback_group_node]: [exclusive_2] tick 0 start
[callback_group_node-1] [INFO] [...] [callback_group_node]: [default] tick 0 start
[callback_group_node-1] [INFO] [...] [callback_group_node]: [reentrant_2] tick 0 start
[callback_group_node-1] [INFO] [...] [callback_group_node]: [reentrant_1a] tick 0 start
[callback_group_node-1] [INFO] [...] [callback_group_node]: [reentrant_1a] tick 0 end
[callback_group_node-1] [INFO] [...] [callback_group_node]: [reentrant_2] tick 0 end
[callback_group_node-1] [INFO] [...] [callback_group_node]: [exclusive_2] tick 0 end
[callback_group_node-1] [INFO] [...] [callback_group_node]: [exclusive_1] tick 0 start
[callback_group_node-1] [INFO] [...] [callback_group_node]: [default] tick 0 end
[callback_group_node-1] [INFO] [...] [callback_group_node]: [reentrant_1b] tick 0 end
[callback_group_node-1] [INFO] [...] [callback_group_node]: [exclusive_1] tick 0 end
```

## Run under rcllf

Start the dev container with the same selection
(`docker compose run --rm -e ENTRYPOINT_PACKAGES="callback_group_example" dev`), then:

```bash
cd packages
colcon lf --package callback_group_example && source install/setup.bash
ros2 lf launch callback_group_example callback_group_node
```

Generation prints one `UserWarning` per callback in a Reentrant group (three in all),
for example:

```text
UserWarning: Node 'CallbackGroupNode': reentrant callback 'timer_reentrant1a_callback' cannot be invoked concurrently with itself under LF (reactor instances are singletons). If parallel self-invocation is required, declare a reactor bank in your .lf source. [...]
```

The program then binds the seven callbacks and runs them at each 500 ms tag. Again
the two `exclusive` callbacks never overlap, the others run in parallel (trimmed):

```text
[CallbackGroupNodeSystem-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (CallbackGroupNode::timer_exclusive1_callback, period 500000000 ns)
...
[CallbackGroupNodeSystem-1] [INFO] [...] [ros2_lf_binding]: Bound subscription RID 6 (CallbackGroupNode::chatter_callback, topic 'chatter', resolved '/chatter', external_topics, dispatch on arrival)
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [default] tick 0 start
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [reentrant_2] tick 0 start
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [exclusive_1] tick 0 start
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [reentrant_1a] tick 0 start
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [reentrant_1b] tick 0 start
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [exclusive_1] tick 0 end
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [reentrant_1a] tick 0 end
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [reentrant_1b] tick 0 end
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [default] tick 0 end
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [reentrant_2] tick 0 end
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [exclusive_2] tick 0 start
[CallbackGroupNodeSystem-1] [INFO] [...] [callback_group_node]: [exclusive_2] tick 0 end
```

Nothing in the program publishes on `chatter`, so the subscription is bound as an
external topic: its callback runs when a message arrives from outside the program.

## What to look at

- **The generated reactors.** In
  `lf-analysis/callback_group_example/callback_group_node/lf/CallbackGroupNodeSystem.lf`
  the node's reactor holds one sub-reactor per MutuallyExclusive group and one per
  Reentrant callback (simplified):

    ```text
    CallbackGroupNode
    ├── default_group              = CallbackGroupNode_DefaultGroup
    │   ├── timer t_timer_default_callback(0, 500 msec)
    │   ├── physical action ext_chatter_callback        (chatter, from outside)
    │   ├── reaction(t_timer_default_callback)
    │   └── reaction(ext_chatter_callback)
    ├── exclusive_group            = CallbackGroupNode_ExclusiveGroup
    │   ├── timer t_timer_exclusive1_callback(0, 500 msec)
    │   ├── timer t_timer_exclusive2_callback(0, 500 msec)
    │   ├── reaction(t_timer_exclusive1_callback)
    │   └── reaction(t_timer_exclusive2_callback)
    ├── timer_reentrant1a_callback = CallbackGroupNode_TimerReentrant1aCallback
    ├── timer_reentrant1b_callback = CallbackGroupNode_TimerReentrant1bCallback
    └── timer_reentrant2_callback  = CallbackGroupNode_TimerReentrant2Callback
    ```

    Reactions of one reactor run one after the other, in declaration order, which
    gives the MutuallyExclusive guarantee. Separate reactors may run on different LF
    worker threads at the same tag, which gives the Reentrant parallelism.
- **Reentrant does not mean self-concurrent.** LF never runs one reaction twice at
  once, so a Reentrant group with a single callback (`reentrant_group_2_`) gains
  nothing over a MutuallyExclusive group. That is what the `UserWarning` says. If you
  need several parallel copies of one callback, see
  [Callback Groups: Reactor Bank Escape Hatch](../design/callback-groups.md#reactor-bank-escape-hatch).
- **What the analyzer read.** In
  `lf-analysis/callback_group_example/callback_group_node/system.json` the node's
  `callback_groups` list has `exclusive_group_` (MutuallyExclusive),
  `reentrant_group_1_` and `reentrant_group_2_` (Reentrant) and `default_group`
  (MutuallyExclusive). Each entry under `timers` and `subscribers` names its group in
  `callback_group`; the timer and the subscription created without a group are in
  `default_group`.

## Further reading

- [Callback Groups](../design/callback-groups.md): the full mapping rules and the
  escape hatch.
