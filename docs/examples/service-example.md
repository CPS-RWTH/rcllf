# service_example

ROS 2 service (request/response) pattern with `ProcessData` service type.

## Pattern

**Server node** (`ServiceServerNode`):

- Service server on `process_data` using `custom_interfaces::srv::ProcessData`
- `handle_process_data(request, response)` callback

**Client node** (`ServiceClientNode`):

- Timer-driven (1s) sends requests to `process_data`
- Service client created via `create_client`

## Communication

```text
ServiceClientNode --[process_data: ProcessData]--> ServiceServerNode
```

## What the Extractor Finds

- 1 `ServiceConnection` on `process_data`
- Server: `ServiceServerInfo(service_name="process_data", callback="handle_process_data")`
- Client: `ServiceClientInfo(service_name="process_data")`

## Generated LF (lfc path)

The extractor's classifier identifies the timer callback that invokes
`async_send_request` and upgrades it from `kind="timer"` to `kind="srv_send"`.
The generator emits three reactions across two reactors:

```
ServiceClientNode_DefaultGroup
  timer t_timer_callback(0, 1000 msec)
  reaction(t_timer_callback) -> process_data_req {
      ros2_lf_invoke_srv_send(..., "process_data");
      lf_set(process_data_req, 1);           // one write per request sent
  }
  reaction(process_data_resp) {
      ros2_lf_invoke_srv_client_cb(..., "process_data");   // once per counted response
  }

ServiceServerNode_DefaultGroup
  reaction(process_data_req) -> process_data_resp {
      ros2_lf_invoke_srv_server(..., "process_data");     // once per counted request
      lf_set(process_data_resp, 1);          // one write per response sent
  }

main:
  client.process_data_req  -> server.process_data_req  after 0
  server.process_data_resp -> client.process_data_resp after 0
```

The listing is simplified: the generated reactions loop over the write count. A request
issued at [logical tag](../design/time-and-coordination.md#logical-time-and-tags) `(T, m)`
is handled one microstep later, at `(T, m+1)`, and the client response lambda fires at
`(T, m+2)`, all at the same logical time `T`. `colcon lf-generate --after-delay` changes the
default delay. See [LF Code Generation: Service Reactions](
../design/lf-generation.md#service-reactions) for the full mapping.

## Run

```bash
# Stock ROS 2
docker compose run --rm dev ros2 launch service_example bringup.launch.py

# Under lfc
docker compose --profile lf run --rm lf-build service_example      # analyze, generate, build, smoke-run
```
