# blocking_service_client_example

Demonstrates two blocking service client patterns that are recognized by the LF converter
but not yet supported by `ros2_lf_binding`.

## Nodes

- **blocking_spin_client**: a loop in `main` with a 2 s sleep, no executor. Each pass calls
  `async_send_request`, then blocks in `rclcpp::spin_until_future_complete(node, future, 5s)`.
- **blocking_future_client**: a 2 s wall timer. Its callback calls `async_send_request`,
  then blocks in `future.wait_for(5s)` and reads the result directly. No nested spin.

Both clients call the `/process_data` service provided by `service_example`'s server,
which is included in the `bringup.launch.py`.

## Purpose / Converter gap

`clang_parser.py:543` recognises both patterns in C++ source but `ros2_lf_binding.h`
has no `ros2_lf_invoke_srv_send_blocking` counterpart. This package is a runnable fixture
for future binding work.

**Under stock ROS2:** both clients work correctly at ~0.5 Hz.

**Under rcllf:** generation is refused with RCLLF-H009. Runs before the
refusal showed `blocking_future_client` timing out on every call and `blocking_spin_client`
never sending a request, in one process and with one federate per node.

## Usage

```bash
ros2 launch blocking_service_client_example bringup.launch.py
```

## LF pipeline status

This package is the playground case of RCLLF-H009. `colcon lf-analyze` reports both
clients, and `colcon lf-generate` stops with an error naming the node, the service and
the call style.
