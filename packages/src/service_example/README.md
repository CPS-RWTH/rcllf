# service_example

Demonstrates the ROS2 service pattern: a server that processes requests and a timer-driven client.

## Pattern

**Service server node:** `ServiceServerNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Service | `create_service` | Name: `process_data`, Type: `custom_interfaces::srv::ProcessData` |
| Callback | `handle_process_data` | Parameters: `request`, `response` |

**Service client node:** `ServiceClientNode` (extends `rclcpp::Node`)

| Element | Type | Detail |
|---------|------|--------|
| Timer | `create_wall_timer` | Period: 1s |
| Callback | `timer_callback` | Parameters: *(none)* |
| Client | `create_client` | Service: `process_data` |

## Callback Signatures

```cpp
// Server: receives request and fills response
void ServiceServerNode::handle_process_data(
    const custom_interfaces::srv::ProcessData::Request::SharedPtr request,
    custom_interfaces::srv::ProcessData::Response::SharedPtr response)
// parameters: [
//   CallbackParameter("request",  "custom_interfaces::srv::ProcessData::Request::SharedPtr"),
//   CallbackParameter("response", "custom_interfaces::srv::ProcessData::Response::SharedPtr"),
// ]

// Client: timer fires to send the next request — no input
void ServiceClientNode::timer_callback()
// parameters: []
```

Service callbacks always take two parameters: request (read-only) and response (to be filled). The extraction tool records both in `CallbackInfo.parameters`.

## Communication

```
ServiceClientNode --[process_data: ProcessData]--> ServiceServerNode
```

## Run

```bash
docker compose run --rm dev ros2 launch service_example bringup.launch.py  # server + client together
```

## What to Expect

Recorded from a real run (process prefixes and timestamps removed). The client alternates
`sum` and `average` over the values 1, 2, 3 and 0.1 times the request count (0, 1, 2, ...):

```
[INFO] [service_example.service_server]: Service server ready on 'process_data'
[INFO] [service_example.service_client]: Sending request: method='sum', 4 values
[INFO] [service_example.service_server]: Request: method='sum', 4 values
[INFO] [service_example.service_client]: Result: 6.00, success=true, message='Computed sum'
[INFO] [service_example.service_client]: Sending request: method='average', 4 values
[INFO] [service_example.service_server]: Request: method='average', 4 values
[INFO] [service_example.service_client]: Result: 1.52, success=true, message='Computed average'
[INFO] [service_example.service_client]: Sending request: method='sum', 4 values
[INFO] [service_example.service_server]: Request: method='sum', 4 values
[INFO] [service_example.service_client]: Result: 6.20, success=true, message='Computed sum'
...
```
