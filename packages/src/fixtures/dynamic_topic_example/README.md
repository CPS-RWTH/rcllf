# dynamic_topic_example

Fixture package for **RCLLF-H004** (`topics.dynamic_name`, WARNING) — the
playground case that proves the analyzer reports topic names it cannot resolve
instead of silently guessing.

## Pattern

**`DynamicPublisherNode`** (`dynamic_publisher_node`) creates two publishers:

| Publisher | Topic argument | Resolvable? |
|---|---|---|
| `dynamic_publisher_` | `"chatter_" + std::to_string(instance_id_)` | **no** — computed from a parameter |
| `status_publisher_` | `"status"` | yes |

**`StaticSubscriberNode`** (`static_subscriber_node`) subscribes to `"status"`
with a plain string literal, so the package always contributes one fully
resolvable edge to the extracted graph.

## Why this package exists

`clang_parser._first_string_literal` walks the whole topic-argument subtree and
returns the *first* string literal it finds. Before Phase 7, the dynamic
publisher above was therefore recorded as publishing on `chatter_` — a
plausible-looking but wrong topic name, emitted with **no warning at all**. The
Phase 7 classifier (`classify_topic_expression`) detects the extra operands,
drops the partial literal, flags the endpoint `topic_dynamic=True`, and
`features/modules/dynamic_topic.py` turns that into a persisted RCLLF-H004
diagnostic in `system.json`.

## Communication

```
DynamicPublisherNode --[status: std_msgs/String]--> StaticSubscriberNode
DynamicPublisherNode --[chatter_<instance_id>]-->  (edge not in the extracted graph, RCLLF-H004)
```

## Run

```bash
ros2 launch dynamic_topic_example bringup.launch.py
colcon lf-analyze --package dynamic_topic_example   # observe RCLLF-H004
```

## What to Expect

```
[INFO] [dynamic_publisher_node]: Dynamic publisher node started (instance_id=1).
[INFO] [static_subscriber_node]: I heard: 'instance 1'
```
