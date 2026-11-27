# action_example

A ROS 2 action (goal, feedback, cancel and result) with the `Analyze` action type. Under
rcllf the server's goal, cancel and result callbacks and its step timer run as LF
reactions, so the client sees the same feedback and result at the same logical times
in every run.

## What it shows

**Server node** (`ActionServerNode`):

- Action server on `analyze` using `custom_interfaces::action::Analyze`.
- Goal, cancel and accepted callbacks. `handle_accepted` only queues the goal.
- A 100 ms step timer advances every active goal by one sample, in acceptance order. A
  step publishes feedback `samples_processed = i`, `progress = 100*i/N`, appends
  `i*1.1` to the result, and ends a canceling goal canceled or a finished goal
  succeeded.
- A pre-shutdown callback aborts every active goal (see
  [Shutdown during a goal](#shutdown-during-a-goal)).

**Client node** (`ActionClientNode`):

- A 2 s timer sends one goal to `analyze` and guards against a second one.
- It logs feedback and the result, then calls `rclcpp::shutdown()`.

```text
ActionClientNode --[analyze: Analyze]--> ActionServerNode
```

The server sends feedback and results from its timer callback, never from a thread of
its own. A server that does (an `execute()` thread started from `handle_accepted`) is
refused at generation with
[RCLLF-H011](../reference/diagnostic-codes.md#rcllf-h011), because the tag at which its
sends reach the client would depend on thread scheduling.

## Run with stock ROS 2

The first start of the dev container does not build this package, so select it with
`ENTRYPOINT_PACKAGES`:

```bash
docker compose run --rm -e ENTRYPOINT_PACKAGES="action_example" dev \
  ros2 launch action_example bringup.launch.py
```

Two seconds after start the client sends a goal, the server reports ten steps 100 ms
apart, and the client prints the result (trimmed; your timestamps differ):

```text
[action_server_node-1] [INFO] [...] [action_example.action_server]: Action server ready on 'analyze'
[action_client_node-2] [INFO] [...] [action_example.action_client]: Sending goal: num_samples=10, analysis_type='statistical'
[action_server_node-1] [INFO] [...] [action_example.action_server]: Received goal: num_samples=10, analysis_type='statistical'
[action_client_node-2] [INFO] [...] [action_example.action_client]: Goal accepted by server, will cancel in 3s
[action_client_node-2] [INFO] [...] [action_example.action_client]: Feedback: samples_processed=0, progress=0.0%
...
[action_client_node-2] [INFO] [...] [action_example.action_client]: Feedback: samples_processed=9, progress=90.0%
[action_server_node-1] [INFO] [...] [action_example.action_server]: Goal succeeded
[action_client_node-2] [INFO] [...] [action_example.action_client]: Result: success=true, results=[0, 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9]
[INFO] [action_client_node-2]: process has finished cleanly [pid ...]
```

The goal finishes after 1 s, before the 3 s cancel timer would fire. The client process
exits; the server keeps running until Ctrl+C.

## Run under rcllf

Start the dev container with the same selection
(`docker compose run --rm -e ENTRYPOINT_PACKAGES="action_example" dev`), then:

```bash
cd packages
colcon lf --package action_example && source install/setup.bash
ros2 lf launch action_example bringup
```

The selection matters: `bringup.launch.py` includes the package's other launch files,
so the analyzer needs `action_example` installed. The analysis prints `2 nodes` for
`bringup.launch.py` and this warning:

```text
    ! [WARNING] RCLLF-H006 (1 occurrence)
      node '/action_example/action_client' creates timer 'cancel_timer_' (callback 'cancel_timer_callback') inside 'goal_response_callback', not in the constructor or a managed-lifecycle hook. [...] 'cancel_timer_callback' never runs under lfc.
```

[RCLLF-H006](../reference/diagnostic-codes.md#rcllf-h006) means a timer created inside
a callback is not part of the generated program ("under lfc" means under the generated
LF program; `lfc` is the Lingua Franca compiler). Here that is the client's 3 s cancel
timer, which also does not fire in the stock run, because the goal finishes first.

The program binds both nodes' callbacks as LF reactions, then gives the same feedback
and result as the stock run (trimmed):

```text
Launching LF-controlled system: action_example/bringup
  Package: lf_generated_action_example_bringup
  Launch file: lf_bringup.launch.py
...
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound action_goal_send RID 4 (ActionClientNode::send_goal, period 2000000000 ns)
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound timer RID 0 (ActionServerNode::step_goals, period 100000000 ns)
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound action server RID 1 (ActionServerNode::handle_goal, action 'analyze')
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound action server RID 2 (ActionServerNode::get_result_callback, action 'analyze')
[Bringup-1] [INFO] [...] [ros2_lf_binding]: Bound action server RID 3 (ActionServerNode::handle_cancel, action 'analyze')
...
[Bringup-1] [INFO] [...] [action_example.action_client]: Feedback: samples_processed=0, progress=0.0%
...
[Bringup-1] [INFO] [...] [action_example.action_client]: Feedback: samples_processed=9, progress=90.0%
[Bringup-1] [INFO] [...] [action_example.action_server]: Goal succeeded
[Bringup-1] [INFO] [...] [action_example.action_client]: Result: success=true, results=[0, 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9]
```

Both nodes share one process here, so the client's `rclcpp::shutdown()` after the
result ends the whole program, and `ros2 lf launch` returns on its own.

The `lf-build` Compose service runs the same steps (analyze, generate, build, then a
30 s smoke run) in one command. Give it the same selection:

```bash
docker compose --profile lf run --rm -e ENTRYPOINT_PACKAGES="action_example" \
  lf-build action_example
```

It ends with the same feedback and result, then:

```text
==> [action_example] smoke passed: stopped on its own after 2 s (exit 0)
```

!!! warning "Known limitation"
    Without `-e ENTRYPOINT_PACKAGES="action_example"`, `lf-build action_example` cannot
    load `bringup.launch.py`, analyzes 0 nodes and generates a program with no
    reactions, yet still prints `smoke passed`. Check that the analysis line reads
    `2 nodes` and that the run prints `Bound ...` lines.

## Shutdown during a goal

`rclcpp_action` throws when a goal handle publishes feedback, reaches a final state, or
is destroyed while still active after the context shut down. The server
therefore ends its goals before the context goes down. It registers a pre-shutdown
callback (`Context::add_pre_shutdown_callback`) that aborts every active goal, and its
destructor does the same for an LF stop that releases the node without a signal.

Under rcllf that callback does not run on rclcpp's signal thread. The binding keeps a
pre-shutdown callback that an action server node registers on the default context
during its construction, and the generated `reaction(shutdown)` of the node runs it.
The abort result is therefore sent at the shutdown tag with an LF origin. On Ctrl-C the
binding first requests the LF stop and waits up to 2 s for the shutdown tag, then lets
rclcpp shut the context down. If the shutdown tag cannot run in time, the kept callback
runs before the context goes down with a warning. The code stays the same as under
stock ROS 2.

Checking `rclcpp::ok()` before `succeed()` is not enough, because the goal handle then
dies active and its destructor publishes a cancel result.

## How the action maps to LF {#generated-lf-lfc-path}

Every action channel is a counted LF port. The reaction that ran a sending callback
raises its port by the number of sends, so a port fires only when a send happened, and
the receiving reaction takes exactly that many samples. A sample that does not arrive
is reported lost. The reactions and the ports whose sends each one reports:

| Reaction | Side | Trigger | Claims |
|----------|------|---------|--------|
| `action_goal_send` | client | timer | `goal_req` |
| `action_goal_server` | server | `goal_req` | `goal_resp`, `feedback`, `result` |
| `action_result_server` | server | `result_req` | `result` |
| `action_cancel_server` | server | `cancel_req` | `cancel_resp`, `feedback`, `result` |
| step timer | server | timer | `feedback`, `result` |
| `action_shutdown` | server | `shutdown` | `feedback`, `result` |
| `action_goal_cb` | client | `goal_resp` | `goal_req`, `result_req`, `cancel_req` |
| `action_feedback` | client | `feedback` | `goal_req`, `result_req`, `cancel_req` |
| `action_result_cb` | client | `result` | `goal_req`, `result_req`, `cancel_req` |
| `action_cancel_cb` | client | `cancel_resp` | `goal_req`, `result_req`, `cancel_req` |

The other launch files of the package exercise more cases: `cancel.launch.py` (a
cancel after sample 3), `chain.launch.py` (a result callback that sends a goal on a
second action), `dual.launch.py` (two servers on one node) and `action_server.launch.py`
(a server alone, for goals sent from outside the program with `ros2 action send_goal`).
The full mapping and its limits are in
[LF Code Generation](../design/lf-generation.md).

## What to look at

- **The bound reactions.** The `Bound ...` lines list each callback that became an LF
  reaction, with its [RID](../design/glossary.md#rid-reaction-id).
- **What the analyzer read.** In `lf-analysis/action_example/bringup/system.json` the
  server node lists `action_servers` with `action_name` `analyze` and one timer
  (`period_ms` 100.0, `callback` `step_goals`); the client node lists
  `action_clients` with `action_name` `analyze` and one timer (`period_ms` 2000.0,
  `callback` `send_goal`). The top-level `communication.actions` entry links the two
  and lists the goal, result and feedback fields.

