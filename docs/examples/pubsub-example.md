# pubsub_example

A publisher and a subscriber on one topic, with the custom message type
`custom_interfaces/msg/SensorData`. This is the example the
[Quickstart](../getting-started/index.md) converts, and
[Your First LF Run](../getting-started/first-lf-run.md) walks through its generated
program line by line.

## What it shows

- **Publisher node** `PublisherNode`: a 500 ms timer whose `timer_callback()` raises
  a counter and publishes a `SensorData` message on `sensor_data`.
- **Subscriber node** `SubscriberNode`: a subscription on `sensor_data` whose
  `topic_callback(msg)` logs the values.

```text
PublisherNode --[sensor_data: SensorData]--> SubscriberNode
```

## Run with stock ROS 2

The first start of the dev container builds this package:

```bash
docker compose run --rm dev ros2 launch pubsub_example bringup.launch.py
```

Every 500 ms the publisher sends a message and the subscriber logs it (trimmed; your
timestamps differ):

```text
[publisher_node-1] [INFO] [...] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[subscriber_node-2] [INFO] [...] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
[publisher_node-1] [INFO] [...] [publisher_node]: Publishing: temp=21.0, humidity=41.0, pressure=1014.0
[subscriber_node-2] [INFO] [...] [subscriber_node]: I heard: temp=21.0, humidity=41.0, pressure=1014.0
```

## Run under rcllf

Inside the dev container (`docker compose run --rm dev`):

```bash
cd packages
colcon lf --package pubsub_example && source install/setup.bash
ros2 lf launch pubsub_example bringup
```

The package has four launch files, so the analysis prints four lines
(`bringup.launch.py` gives `2 nodes, 1 topics, 0 services, 0 actions`) and
`colcon lf` generates four programs. `ros2 lf launch` starts the one for
`bringup.launch.py`: both nodes in one process, the timer and the subscription bound
as LF reactions, then the same messages (trimmed):

```text
Launching LF-controlled system: pubsub_example/bringup
  Package: lf_generated_pubsub_example_bringup
  Launch file: lf_bringup.launch.py
...
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound subscription RID 1 (SubscriberNode::topic_callback, topic 'sensor_data', resolved '/sensor_data', take_mode=skip timeout_ms=100)
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (PublisherNode::timer_callback, period 500000000 ns)
...
[Bringup-1] [INFO] [...] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[Bringup-1] [INFO] [...] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
[Bringup-1] [INFO] [...] [publisher_node]: Publishing: temp=21.0, humidity=41.0, pressure=1014.0
[Bringup-1] [INFO] [...] [subscriber_node]: I heard: temp=21.0, humidity=41.0, pressure=1014.0
```

The first message goes out at once, at the program's start tag, because an LF timer
starts at offset 0 (see [timer_example](timer-example.md)).

## What to look at

- **The generated LF program.** `lf-analysis/pubsub_example/bringup/lf/Bringup.lf`
  holds one reactor per node and one connection between them,
  `publisher_node.corr_e0 -> subscriber_node.corr_e0`. The message itself still
  travels over DDS; the connection tells the subscriber's reaction at which
  [tag](../design/glossary.md#tag) a message was sent, so it delivers it at that tag.
  [Your First LF Run](../getting-started/first-lf-run.md) explains each line.
- **What the analyzer read.** In `lf-analysis/pubsub_example/bringup/system.json`
  the node `publisher_node` (`class_name` `PublisherNode`) lists one entry under
  `publishers` (`topic` `sensor_data`, `message_type`
  `custom_interfaces::msg::SensorData`) and one under `timers` (`period_ms` 500.0,
  `callback` `timer_callback`). The node `subscriber_node` lists one entry under
  `subscribers` with `callback` `topic_callback`. The top-level
  `communication.topics` entry for `/sensor_data` links the two and lists the message
  fields.
