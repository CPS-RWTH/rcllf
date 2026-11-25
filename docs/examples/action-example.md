# action_example

ROS 2 action pattern with goal, feedback, cancel and result using the `Analyze` action
type. Under lfc every step of the server runs as an LF reaction, so the client sees the
same feedback, result and logical tags in every run.

## Pattern

**Server node** (`ActionServerNode`):

- Action server on `analyze` using `custom_interfaces::action::Analyze`
- Goal, cancel and accepted callbacks. `handle_accepted` only queues the goal.
- A 100 ms step timer advances every active goal by one sample, in acceptance order. A
  step publishes feedback `samples_processed = i`, `progress = 100*i/N`, appends
  `i*1.1` to the result, and ends a canceling goal canceled or a finished goal
  succeeded.
- A pre-shutdown callback aborts every active goal (see below).

**Client node** (`ActionClientNode`):

- Timer-driven (2 s), sends one goal to `analyze` and guards against a second one
- Logs feedback and result

The server sends feedback and results from its timer callback, never from a thread of
its own. A server that does (an `execute()` thread started from `handle_accepted`, as
in `action_full_example`) is refused at generation with
[RCLLF-H011](../reference/troubleshooting.md#generation-refused-with-rcllf-h011-action-server-sending-from-its-own-thread), because the tag at which its
sends reach the client would depend on thread scheduling.

## Shutdown during a goal

`rclcpp_action` throws when a goal handle publishes feedback, reaches a final state, or
is destroyed while still active after the context shut down. The server
therefore ends its goals before the context goes down. It registers a pre-shutdown
callback (`Context::add_pre_shutdown_callback`) that aborts every active goal, and its
destructor does the same for an lfc stop that releases the node without a signal.

Under lfc that callback does not run on rclcpp's signal thread. The binding keeps a
pre-shutdown callback that an action server node registers on the default context
during its construction, and the generated `reaction(shutdown)` of the node runs it.
The abort result is therefore sent at the shutdown tag with an LF origin. On Ctrl-C the
binding first requests the LF stop and waits up to 2 s for the shutdown tag, then lets
rclcpp shut the context down. If the shutdown tag cannot run in time, the kept callback
runs before the context goes down with a warning. The code stays the same as under
stock ROS 2.

Checking `rclcpp::ok()` before `succeed()` is not enough, because the goal handle then
dies active and its destructor publishes a cancel result.

## Communication

```text
ActionClientNode --[analyze: Analyze]--> ActionServerNode
```

## What the Extractor Finds

- 1 `ActionConnection` on `analyze`
- Server: `ActionServerInfo` with `goal_callback`, `cancel_callback`, `accepted_callback`
- Server: the step timer callback publishes feedback and sends results on the action, so
  its reaction claims those sends
- Client: `ActionClientInfo(action_name="analyze")`, `async_cancel_goal` recorded on its
  own for the cancel client

## Generated LF (lfc path)

Every action channel is a counted port. The action gate counts each rcl_action send per
thread, action and channel. The reaction that ran the sending callback claims the count
and raises its port by it, so a port fires only when a send happened. The receiving
reaction takes exactly that many samples, each with a bounded wait. A sample that does
not arrive is traced `LOST` with one error log.

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

The table lists the claims for the reaction's own action. Each reaction of a node
except `action_result_server`, and each service reaction of that node, also claims the
client sends and the server feedback and result of every other action on the node. In
`chain.launch.py` the result callback of the goal on `analyze_a` sends the goal on
`analyze_b`, and that send is raised on the `analyze_b` goal port. A claim that sees no
send fires nothing.

Action RIDs bind to the server or client whose action name matches, so a node with two
servers (`dual.launch.py`) gets separate reactions for each. Ports are named after the
full declared action name, so `/left/move` and `/right/move` on one node
get separate ports. The binding counts the sends of `/right/move` also for a claim on
`move`, so two actions on one node where one name is a path suffix of the other are
refused at generation. An action with
several client nodes is refused as well, because a response count cannot yet be routed
to the one client that asked.

A server with no client inside the LF system, such as `action_server.launch.py` driven
by `ros2 action send_goal`, gets a physical action instead of the request ports. Its
reactions trigger when a request arrives, and their tags depend on arrival time. Mixed
internal and external clients on one action are not supported, and the generator cannot
detect them: a server whose action has a client inside the LF system takes no goal from
a client outside it.

Client feedback is carried by the counted port only. It is not admitted as a
publication correspondence edge, because `rcl_action_take_feedback` returns no message
info (sequence number) on the client.

## Run

```bash
# Stock ROS 2
docker compose run --rm dev ros2 launch action_example bringup.launch.py

# Under lfc
docker compose --profile lf run --rm lf-build action_example       # generate + build the lfc package
```

