# custom_interfaces

Defines the custom ROS2 message, service, and action types used across the example packages. Contains no nodes — only interface definitions.

## Interfaces

### `msg/SensorData.msg`

Used by: `pubsub_example`

```
std_msgs/Header header
float64 temperature
float64 humidity
float64 pressure
```

### Other messages

| File | Used by | Fields |
|---|---|---|
| `msg/BrakeCmd.msg` | `brake_fusion` (`/brake_cmd`) | `float64 level` (first on purpose), `uint64 seq`, `int64 tick` |
| `msg/SyncSample.msg` | `sync_oracle_showcase` | `Header header`, `uint32 input`, `uint64 row`, `uint64 payload` |
| `msg/Detections.msg` | `sentry_pipeline` | `Header header`, `uint32 count`, `float64[] boxes` |
| `msg/FusedState.msg` | `sentry_pipeline` | `Header header`, `float64[6] state` |
| `msg/Plan.msg` | `sentry_pipeline` | `Header header`, `float64[] waypoints` |

### `srv/ProcessData.srv`

Used by: `service_example`, `multi_service_example`

```
float64[] values
string method
---
float64 result
bool success
string message
```

Service callbacks receive both the **request** and **response** as parameters:

```cpp
// parameters: [
//   CallbackParameter("request",  "custom_interfaces::srv::ProcessData::Request::SharedPtr"),
//   CallbackParameter("response", "custom_interfaces::srv::ProcessData::Response::SharedPtr"),
// ]
```

### `action/Analyze.action`

Used by: `action_example`, `action_full_example`

```
# Goal
int32 num_samples
string analysis_type
---
# Result
float64[] results
bool success
---
# Feedback
int32 samples_processed
float64 progress_percentage
```

Action goal callbacks receive the **GoalUUID** and **goal** as parameters:

```cpp
// parameters: [
//   CallbackParameter("uuid", "rclcpp_action::GoalUUID"),
//   CallbackParameter("goal", "custom_interfaces::action::Analyze::Goal::ConstSharedPtr"),
// ]
```

## Build

This package is built first (other packages depend on it). It produces no executables — only generated C++ header/source files for the six messages, the service and the action.

```bash
docker compose run --rm dev  # builds all packages including custom_interfaces
```
