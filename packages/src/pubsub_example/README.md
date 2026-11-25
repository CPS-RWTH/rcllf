# pubsub_example

Demonstrates the ROS2 publish/subscribe pattern using a custom message type.

## Pattern

**Publisher node:** `PublisherNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Timer | `create_wall_timer` | Period: 500ms |
| Callback | `timer_callback` | Parameters: *(none)* |
| Publisher | `create_publisher` | Topic: `sensor_data`, Type: `custom_interfaces::msg::SensorData` |

**Subscriber node:** `SubscriberNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Subscription | `create_subscription` | Topic: `sensor_data`, Type: `custom_interfaces::msg::SensorData` |
| Callback | `topic_callback` | Parameters: `msg` (`SensorData::SharedPtr`) |

## Callback Signatures

```cpp
// Publisher: timer drives publishing — no input
void PublisherNode::timer_callback()
// parameters: []

// Subscriber: receives the message as input
void SubscriberNode::topic_callback(
    const custom_interfaces::msg::SensorData::SharedPtr msg)
// parameters: [CallbackParameter("msg", "custom_interfaces::msg::SensorData::SharedPtr")]
```

The subscriber callback receives the message as its parameter. The extraction tool captures this as `CallbackInfo.parameters[0] = CallbackParameter(name="msg", type="custom_interfaces::msg::SensorData::SharedPtr")`.

## Communication

```
PublisherNode --[sensor_data: SensorData]--> SubscriberNode
```

## Run

```bash
docker compose run --rm dev ros2 launch pubsub_example bringup.launch.py   # both nodes together
docker compose run --rm dev ros2 launch pubsub_example publisher_node.launch.py    # publisher only
docker compose run --rm dev ros2 launch pubsub_example subscriber_node.launch.py   # subscriber only
```

## What to Expect

```
[INFO] [publisher_node]: Publisher node started.
[INFO] [subscriber_node]: Subscriber node started.
[INFO] [publisher_node]: Publishing: temp=20.0, humidity=40.0, pressure=1013.0
[INFO] [subscriber_node]: I heard: temp=20.0, humidity=40.0, pressure=1013.0
[INFO] [publisher_node]: Publishing: temp=21.0, humidity=41.0, pressure=1014.0
```
