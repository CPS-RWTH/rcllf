# pubsub_example

Publisher/subscriber pattern with a custom message type (`SensorData`).

## Pattern

**Publisher node** (`PublisherNode`):

- Timer-driven (500ms) publishes `custom_interfaces::msg::SensorData` on `sensor_data`
- `timer_callback()` increments counter and publishes

**Subscriber node** (`SubscriberNode`):

- Subscribes to `sensor_data` topic
- `topic_callback(msg)` receives `SensorData::SharedPtr`

## Communication

```text
PublisherNode --[sensor_data: SensorData]--> SubscriberNode
```

## What the Extractor Finds

- 2 nodes, 1 `TopicConnection` on `sensor_data`
- Publisher: `PublisherInfo(topic="sensor_data")`, `TimerInfo(period_ms=500)`
- Subscriber: `SubscriberInfo(topic="sensor_data", callback="topic_callback")`
- Callback parameter: `CallbackParameter(name="msg", type="...SensorData::SharedPtr")`

## Run with stock ROS 2

```bash
docker compose run --rm dev ros2 launch pubsub_example bringup.launch.py
```

This runs the two nodes as plain ROS 2 processes. To convert and run them under rcllf,
follow the [Quickstart](../getting-started/index.md).
