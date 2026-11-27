# service_example

A ROS 2 service (request and response) between two nodes, with the
`custom_interfaces/srv/ProcessData` service type. The example shows how a request and
its response become LF connections, so both are ordered in logical time.

## What it shows

- **Server node** `ServiceServerNode`: a service server on `process_data`; its callback
  `handle_process_data(request, response)` computes a sum or an average.
- **Client node** `ServiceClientNode`: a 1 s timer sends a request to `process_data`,
  alternating `sum` and `average`, and a response callback logs the result.

```text
ServiceClientNode --[process_data: ProcessData]--> ServiceServerNode
```

## Run with stock ROS 2

The first start of the dev container builds this package:

```bash
docker compose run --rm dev ros2 launch service_example bringup.launch.py
```

One request per second, each followed by the server's log line and the client's result
(trimmed; your timestamps differ):

```text
[service_server_node-1] [INFO] [...] [service_example.service_server]: Service server ready on 'process_data'
[service_client_node-2] [INFO] [...] [service_example.service_client]: Sending request: method='sum', 4 values
[service_server_node-1] [INFO] [...] [service_example.service_server]: Request: method='sum', 4 values
[service_client_node-2] [INFO] [...] [service_example.service_client]: Result: 6.00, success=true, message='Computed sum'
[service_client_node-2] [INFO] [...] [service_example.service_client]: Sending request: method='average', 4 values
[service_server_node-1] [INFO] [...] [service_example.service_server]: Request: method='average', 4 values
[service_client_node-2] [INFO] [...] [service_example.service_client]: Result: 1.52, success=true, message='Computed average'
```

The next results are `6.20` (sum), `1.57` (average), `6.40` (sum), and so on.

## Run under rcllf

Inside the dev container (`docker compose run --rm dev`):

```bash
cd packages
colcon lf --package service_example && source install/setup.bash
ros2 lf launch service_example bringup
```

Both nodes run in one generated program. Three callbacks become LF reactions, then the
same requests and results follow (trimmed):

```text
Launching LF-controlled system: service_example/bringup
  Package: lf_generated_service_example_bringup
  Launch file: lf_bringup.launch.py
...
[Bringup-1] [INFO] [...] [service_example.service_server]: Service server ready on 'process_data'
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound srv_send RID 1 (ServiceClientNode::timer_callback, period 1000000000 ns)
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound service server RID 0 (ServiceServerNode::handle_process_data, service 'process_data')
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound service client RID 2 (ServiceClientNode::process_data_response_callback, service 'process_data')
[Bringup-1] [INFO] [...] [service_example.service_client]: Sending request: method='sum', 4 values
[Bringup-1] [INFO] [...] [service_example.service_server]: Request: method='sum', 4 values
[Bringup-1] [INFO] [...] [service_example.service_client]: Result: 6.00, success=true, message='Computed sum'
[Bringup-1] [INFO] [...] [service_example.service_client]: Sending request: method='average', 4 values
```

The first request goes out at once, at the program's start tag, because an LF timer
starts at offset 0 (see [timer_example](timer-example.md)).

## What to look at

- **The timer that sends a request.** The analyzer sees that `timer_callback` calls
  `async_send_request`, so it binds it as `srv_send` (a timer reaction that sends a
  request), not as a plain timer.
- **The generated LF program.** `lf-analysis/service_example/bringup/lf/Bringup.lf`
  holds one reactor per node. Simplified, the request and the response are LF
  connections:

    ```text
    reactor ServiceClientNode_DefaultGroup
      timer t_timer_callback(0, 1000 msec)
      reaction(t_timer_callback) -> process_data_req       // sends the request
      reaction(process_data_resp)                          // runs the response callback

    reactor ServiceServerNode_DefaultGroup
      reaction(process_data_req) -> process_data_resp      // runs handle_process_data

    main reactor Bringup
      client.process_data_req  -> server.process_data_req  after 0
      server.process_data_resp -> client.process_data_resp after 0
    ```

    `after 0` delays by one microstep, not by time: a request sent at
    [tag](../design/glossary.md#tag) `(T, m)` is handled at `(T, m+1)` and the
    response callback runs at `(T, m+2)`, all at the same logical time `T`. See
    [LF Code Generation: Service Reactions](../design/lf-generation.md#service-reactions)
    for the full mapping.
- **What the analyzer read.** In `lf-analysis/service_example/bringup/system.json`
  the server node lists `service_servers` with `service_name` `process_data` and
  `callback` `handle_process_data`; the client node lists `service_clients` with
  `service_name` `process_data` and one timer (`period_ms` 1000.0). The top-level
  `communication.services` entry links the two.
