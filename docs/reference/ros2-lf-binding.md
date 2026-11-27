# Binding Internals

`ros2_lf_binding` is the shared library that bridges the lfc-compiled Lingua Franca
scheduler and the running ROS 2 component nodes. It is a page for contributors and for
readers who debug a run. It has two parts: the C API contract that generated code calls
([Public C API](#public-c-api)), then the data structures and the design decisions behind
each module ([Internal Data Structures](#internal-data-structures) onward).

For a high-level overview of where it fits in the pipeline, see [Architecture](../design/architecture.md).
For the time-authority design decision, see [ADR 0001](../design/adr/0001-lf-owns-time.md).

---

## Public C API

The entire public interface is declared in `include/ros2_lf_binding/ros2_lf_binding.h`.
All functions have C linkage so they can be called from lfc-generated C++ without
name-mangling issues. The sections below describe the lifecycle, startup and dispatch
entry points in detail; [Other API groups](#other-api-groups) lists the rest in one line each.

### Lifecycle

#### `ros2_lf_runtime_init(argc, argv)`

Called from `ros2_lf_startup.cpp` **before** lfc's own `main()` via a
`__attribute__((constructor))` function. Responsibilities (in order):

1. `rclcpp::init(argc, argv)`
2. `lf_clock_init()` creates a `RCL_ROS_TIME` clock and enables the ROS time override
   (see [Time source](#time-source-time_sourcecpp))
3. Parses `SYSTEM_JSON` env-var (or `--system-json` argv argument) to locate `system.json`
4. Creates `LFComponentManager` and a `SingleThreadedExecutor` (used only for node
   registration, since lfc owns scheduling)

`ros2_lf_runtime_init` does not load nodes. That is `ros2_lf_runtime_load_nodes()`,
described next.

#### `ros2_lf_runtime_load_nodes()`

Called by the generated `ros2_lf_startup.cpp` constructor after `ros2_lf_runtime_init()`
and after `node_factories.cpp` has registered the non-component nodes with
`ros2_lf_register_node()`. It loads the component and non-component nodes found in the
`system.json` that init parsed. `ros2_lf_runtime_wait_ready()` then waits for DDS
discovery, before lfc sets the start time.

**If `SYSTEM_JSON` is not set and `--system-json` is not provided, the process calls
`rclcpp::shutdown()` and `std::exit(EXIT_FAILURE)`.** A misconfigured launch
otherwise produces zero callbacks and zero trace events, indistinguishable
from a correct run at a glance.

#### `ros2_lf_runtime_shutdown()`

Called from `ros2_lf_startup.cpp` **after** lfc's `main()` returns. Resets the
component manager and executor, then calls `rclcpp::shutdown()`.

On Ctrl-C the rclcpp context is shut down first and LF stops after that, so
user nodes are still alive while `rclcpp::ok()` is already false. A node that
ends work in a pre-shutdown callback (an action server aborting its goals, for
example) gets it run before the context goes down, see
[action_example](../examples/action-example.md#shutdown-during-a-goal).

**Action server pre-shutdown callbacks.** The binding overrides
`rclcpp::Context::add_pre_shutdown_callback` and `remove_pre_shutdown_callback` in
the executable (`action_gate.cpp`). A callback that a node serving an action
registers on the default context while the generated factory constructs it is kept
by the binding, not by the context. `ros2_lf_action_server_shutdown` runs it once
from the node's generated `reaction(shutdown)` under an LF origin. On Ctrl-C the
binding's own pre-shutdown hook first requests the LF stop and waits up to 2 s for
the shutdown tag. When no shutdown tag can run the kept callbacks (LF not running,
the hook runs on an LF worker or in a user callback) or the wait times out, they run
in the hook with a warning, before `rcl_shutdown`. Callbacks of other nodes, of
component nodes and on other contexts stay with their context.

**Process end bound after a signal** (`runtime.cpp`: `arm_stop_timeout()`,
`stop_timeout_ms_from_env()`). Once rclcpp has shut down on
a signal, the binding gives the process `RCLLF_STOP_TIMEOUT_MS` (default 3000)
to reach `ros2_lf_runtime_shutdown()`, and the same amount again for the
teardown after it. The bound starts after the pre-shutdown hook above, so a
`reaction(shutdown)` that runs while the hook waits gets those 2 s first, then
the bound. A signal that arrives only during the teardown (the process reached
its stop time on its own, then the teardown blocks, for example on an RTI that
is gone) arms the same bound; the binding's own `rclcpp::shutdown()` in the
teardown does not. A bound armed by rclcpp's shutdown logs
`process end bounded to <ms> ms ... exits with code 124`; one armed from the
late-signal handler during the teardown logs nothing (no logging in a signal
handler), and `signal(s) during shutdown, counted` appears only if the
teardown completes. A process that is still running when
the bound ends, because a federate's stop waits for an RTI that is gone or
because a shutdown reaction or a destructor takes longer, exits with
**code 124** through `_exit`: no destructors, no atexit handlers, and no
further output, since a write to a full stderr pipe could block the exit
itself. A node that needs longer to stop sets a larger value in its launch
environment, for example `RCLLF_STOP_TIMEOUT_MS=10000`; `0` turns the bound
off. `ros2 launch` still sends SIGTERM `sigterm_timeout` (default 5 s) after
its SIGINT and SIGKILL 5 s later, so a bound above that also needs a larger
`sigterm_timeout`. Processes that end normally (LF reaches its stop time) and
get no signal are not bounded. The value is read once at init; an invalid
value logs a warning and keeps the default.

**Startup bound** (`runtime.cpp`: `arm_startup_bound()`,
`startup_timeout_ms_from_env()`). From before `rclcpp::init` until
`ros2_lf_runtime_wait_ready()` reaches its discovery wait, the binding gives the
start `RCLLF_STARTUP_TIMEOUT_MS` (default 30000). The span covers `rclcpp::init`,
every node construction, the component loading and the correspondence startup
stage; the waits after it have their own timeout (`ROS2_LF_DISCOVERY_TIMEOUT_MS`).
A start that is still inside that span when the bound ends, for example a node
construction whose graph-update publish waits for good on a Fast DDS shared-memory
port lock, prints
`rcllf: startup did not reach the discovery wait within <ms> ms (RCLLF_STARTUP_TIMEOUT_MS) ...`
to stderr and exits with **code 124** through `_exit`, without cleanup. A startup
refusal or a shutdown in that span hands over to the stop bound above. The
hand-over and the expiry are one atomic decision: a stop bound armed before the
startup bound commits to its exit governs the process end, one armed after it
does not change that exit. With `RCLLF_STOP_TIMEOUT_MS=0` a shutdown arms no stop
bound and hands nothing over, so a stuck start still ends at the startup bound.
The armed bound logs `rcllf: startup bounded to <ms> ms`. `0` turns it off; an invalid value
logs a warning and keeps the default.

**Controlled shutdown** (`runtime.cpp`: `stop_lf_before_context_shutdown()`,
`stop_join_bound_ms()`, `stop_drain_bound_ms()`). A signal makes rclcpp run the binding's
pre-shutdown hook before it invalidates the context. The hook requests the LF
stop (the stop tag is the current tag plus one microstep, or the tag the RTI
grants in a federation) and waits, bounded by `RCLLF_STOP_MAX_MS`, until the
stop has *settled*: LF has run every tag below the stop tag and its shutdown
tag, and the runtime shutdown has drained the samples still in flight. The
context, the ingress thread and the user callbacks stay alive meanwhile, so a
join below the stop tag can still match. At the stop request every join that
is waiting gets a deadline of half the process end bound: a join whose sample
lands before it matches as usual; one still waiting at the deadline ends with
`EPOCH_CLOSE`, is counted `pending_at_stop` on its edge (listed in the
reconcile line's `pending` array) and is **not** missing. After LF returns,
the runtime shutdown first waits up to a quarter of the bound for this
program's reliable publishers to be acknowledged (an overloaded federate
flushes its last samples after the stop tag and would otherwise exit before
they are delivered), then a quarter of the bound for the newest sample each
writer still owes (an expectation that ended `NOT_READY` or pending whose
sample has not been taken), so that sample is recorded late rather than never;
`owed_at_close` in the reconcile line counts the writers still owing one when
the table closed. Expectations at or after the stop tag are not dispatched and
end `EPOCH_CLOSE` at the close. A sample whose drain LF refuses to
queue because the drain's tag would lie after the stop tag (its release
`t + d` is past the stop, or it needs the next microstep of the stop tag;
`lf_schedule` returns 0 for such an event) ends `schedule_refused` instead:
the same undelivered sample with the other label, decided by whether its
arrive reaction ran before or after the stop request within the stop tag.
Like `EPOCH_CLOSE` it is neither missing nor pending. The stop
request logs the pending
joins (`LF stop requested with N join(s) pending`, with edge, writer, ordinal
and tag), so a run that the process end bound ends without a reconcile line
still records them. The process end bound itself counts from the last progress
(a dispatch that returns re-arms it, at most every 250 ms) within
`RCLLF_STOP_MAX_MS` (default four times the bound, 12 s) from the signal, so a
run winding down a lagging tag under overload is not cut off with code 124
while its dispatches still return, while a federate whose RTI is gone still
ends at the cap (`stop_progress()`, `rearm_stop_timeout()`, `stop_max_ms_from_env()`).

`RCLLF_EXIT_WITH_PARENT=<pid>` (`runtime.cpp`: `exit_with_parent_if_requested()`) makes the kernel send the process SIGTERM when
its parent dies (`PR_SET_PDEATHSIG`). The value is the pid of the launching
process, taken there (`str(os.getpid())` in the launch file), not in the
child: had the parent died before the child's init, the child's `getppid()`
would already name init or a subreaper. At init the binding compares
`getppid()` with that pid before and after the `prctl`; on a mismatch the
launching process is gone, and the process logs a FATAL line and exits with
code 1 before LF starts. A value that is not a pid, including `1`, is
refused the same way. The generated launch files set it for the
single-process lfc node, for every federate and for the RTI guard
(`rti_guard.py` applies the same check and then does not start the RTI),
because `ros2 launch` exits on a SIGTERM to itself without stopping its
processes.

---

### Startup binding

#### `ros2_lf_bind_node(self, node_class)`

Called from every reactor's `reaction(startup)` body during lfc initialisation.
`node_class` is the unqualified C++ class name (e.g. `"TimerPublisherNode"`).

Steps (implemented in `node_binding.cpp`):

1. Locate the loaded node via `LFComponentManager::find_by_class(node_class)`.
   Matching is tried first with namespace prefix (`pkg::ClassName`), then without.
2. Walk all callback groups of the node to collect all `TimerBase`,
   `SubscriptionBase`, `ServiceBase`, `ClientBase`, and `Waitable` handles.
3. For each entry in the RID table (`get_rid_entries()`):
   - **`timer` / `srv_send` / `action_goal_send`:** find the timer whose period
     equals `entry.period_ns` to the nanosecond, and set kind to `TIMER`.
     A timer of another period, even 0.5 ms off, does not fit, and an entry
     with period 0 (unknown) fits none. `create_wall_timer` stores the duration it
     is given in nanoseconds, and the generator converts the analyzed period from its
     exact decimal, so the two agree when they describe the same timer.
     `srv_send` is the timer callback that calls `async_send_request`, and it is
     bound as a TIMER and dispatched the same way.
   - **`sub`:** find the subscription whose fully-resolved topic matches
     `entry.topic` (leading `/` normalised), and set kind to `SUBSCRIPTION`.
   - **`srv_server`:** match by `get_service_name()`, and set kind to `SERVICE_SERVER`.
   - **`srv_client_cb`:** match the client by service name, and set kind to
     `SERVICE_CLIENT`.
   - **`action_goal_server` / `action_cancel_server` / `action_result_server`:**
     the `rclcpp_action::ServerBase` waitable whose action name equals
     `entry.topic`, and set kind to `ACTION_SERVER`.
   - **`action_goal_cb` / `action_feedback` / `action_result_cb` / `action_cancel_cb`:**
     the `rclcpp_action::ClientBase` waitable whose action name equals `entry.topic`,
     set kind to `ACTION_CLIENT` and record `action_entity_id` (0 = GoalClient,
     1 = ResultClient, 2 = CancelClient, 3 = FeedbackSubscription).
4. Leave every matched timer armed. No executor spins a bound node, so lfc fires
   them via `ros2_lf_invoke_timer`, which skips a timer the user cancelled
   (`cancel()` and `reset()` are honoured).
5. Store the handle in `g_handle_table[entry.reaction_id]`.

`self` is currently unused (reserved for future per-reactor state).

---

### Reaction dispatch

#### `ros2_lf_invoke_timer(self, reaction_id)`

Called from every timer reaction body. It returns quietly, without an error, on every
guard below. Steps, from `reaction_dispatch.cpp`:

1. `require_lf_worker` checks the call comes from an LF worker thread.
2. Reset the per-reaction publication count, then `lf_time_advance_ros_clock()` pushes the
   current [LF logical time](../design/time-and-coordination.md#logical-time-and-tags) into
   the RCL time override so `Node::now()` returns logical time during the callback. The
   periodic take-statistics log runs here too.
3. Range-check `reaction_id` against `g_handle_table` (a warning, then return).
4. `lifecycle_admit`: a lifecycle node that is not active at this tag runs no callback (traced).
5. `bind_late_handle`: a lifecycle node's handle may appear on a later transition. An
   `UNBOUND` entry marked late waits for it; any other `UNBOUND` entry is traced and skipped.
6. A handle that is not a timer is skipped with a debug log.
7. The weak timer pointer is locked. An expired entity (a lifecycle node dropped its
   timer) is traced as expired and skipped.
8. A timer the user cancelled (`is_canceled()`) is skipped until `reset()`; `reset()` does
   not shift the LF tick grid.
9. Trace the callback start, then call `timer->execute_callback(call_info)`.

The `rcl_timer_call_info_t` passed to `execute_callback` has zero timestamps.
rclcpp ignores these fields for user-defined timer callbacks.

#### `ros2_lf_invoke_sub(self, reaction_id, topic)`

Called from every subscription reaction body. Steps:

1. `lf_time_advance_ros_clock()`
2. Range-check `reaction_id`; the lifecycle and late-bind guards of the timer path apply too.
3. Skip with a debug log when `g_handle_table[reaction_id].kind` is not `SUBSCRIPTION`.
4. `sub->take_type_erased(message, message_info)` polls the DDS/rmw queue.
5. If a message is available: `sub->handle_message(message, message_info)`.
6. If no message is available: `RCLCPP_DEBUG` log (not an error, since the lfc
   scheduler fires the reaction at the logical tag even when the DDS queue is
   transiently empty under load).

`topic` is passed through for debug logging only, since the subscription handle already
knows its topic.

#### Service dispatch

All three start with `lf_time_advance_ros_clock()` and a kind-check.

- **`ros2_lf_invoke_srv_send(self, rid, service_name)`** delegates to
  `ros2_lf_invoke_timer(nullptr, rid)`, since the underlying handle was bound as a TIMER.
- **`ros2_lf_invoke_srv_server(self, rid, service_name)`** mirrors
  `rclcpp::Executor::execute_service`:
  ```cpp
  auto request = svc->create_request();
  auto header  = svc->create_request_header();
  if (svc->take_type_erased_request(request.get(), *header))
      svc->handle_request(header, request);
  ```
- **`ros2_lf_invoke_srv_client_cb(self, rid, service_name)`** mirrors
  `execute_client`:
  ```cpp
  auto response = cli->create_response();
  auto header   = cli->create_request_header();
  if (cli->take_type_erased_response(response.get(), *header))
      cli->handle_response(header, response);
  ```

#### Action dispatch

Every action channel is a counted port. The executable-side action gate
(`action_gate.cpp`) wraps the `rcl_action` send, feedback publish and take calls and
counts each send per thread, action and channel. A generated reaction calls
`ros2_lf_action_sent(action, channel)` after its invoke and raises the channel's port
by the result, so a port fires only for a send that happened.

- **`ros2_lf_invoke_action_goal_send(self, rid, action_name)`** delegates to
  `ros2_lf_invoke_timer(nullptr, rid)`.
- **`ros2_lf_invoke_action_goal_server` / `_cancel_server` / `_result_server`
  `(self, rid, action_name, count)`** take `count` goal, cancel or result requests
  from the server waitable and execute each.
- **`ros2_lf_invoke_action_client_cb(self, rid, action_name, count)`** uses
  `entry.action_entity_id` so the same function serves the goal response, feedback,
  result and cancel response reactions.

Each take waits a bounded time for its sample. A sample that does not arrive is
traced `LOST` once with one error log. `rclcpp_action` exposes no QoS event handlers
for its internal entities, so the deadline and message_lost handlers that topics get
cannot be called for actions.

While LF runs, the gate refuses an action send from a thread with no LF origin
(`rcllf: refused`, logged once per action and channel, a `refused` trace line). The
call is not forwarded.

- **`ros2_lf_action_external_register(self, goal_rid, wake, action)`** and
  **`ros2_lf_action_external_pending(self, goal_rid, entity)`** serve a server with
  no client inside the LF system. The server's on-ready callback counts each arriving
  request per entity (0 goal, 1 result, 2 cancel) and schedules a physical action. The
  reaction passes the pending count as its invoke's `count`. These tags are physical.
- **`ros2_lf_action_server_shutdown(self, goal_rid)`** runs the kept pre-shutdown
  callbacks of a server node, see `ros2_lf_runtime_shutdown()` above.

---

### Other API groups

The header declares more entry points than the sections above describe. Generated code
calls them; user code never does. The header comments are the authority for each signature.

| Group | Purpose |
|---|---|
| `ros2_lf_runtime_wait_ready`, `_defer_peer_check`, `_peer_check` | Wait for DDS discovery; a federate defers the cross-federate peer check until the RTI start time is known, then runs it or exits 3 |
| `ros2_lf_runtime_declare_external_topic`, `_declare_external_stamp_anchor` | Declare a topic published from outside the system, and its stamp anchor |
| `ros2_lf_runtime_shutdown_before_resign`, `_test_resign_delay` | Controlled shutdown before a federate resigns from the RTI; a test hook that delays the resign |
| `ros2_lf_logical_time_ns` | Current logical time in nanoseconds |
| `ros2_lf_invoke_sub_n` | Subscription dispatch for a trigger that owes several samples (`remaining`); returns the count dispatched |
| `ros2_lf_topic_sent`, `_service_sent`, `_action_sent` | Count the publishes, requests or action sends a reaction made, so its output port is raised by that count |
| `ros2_lf_output_deadline_arm` | Arm an output deadline for a topic's next publishes |
| `ros2_lf_claim_publish` | Count a publish deferred to another thread for the running reaction (fault injection only) |
| `ros2_lf_external_register`, `_invoke_external_sub` | Register and dispatch a subscription fed from outside the system |
| `ros2_lf_invoke_srv_send`, `_srv_server`, `_srv_client_cb` | Service dispatch (see [Service dispatch](#service-dispatch)) |
| `ros2_lf_invoke_action_*`, `ros2_lf_action_*` | Action dispatch, including `_cancel_server`, `_result_server`, external registration and the pre-shutdown call |
| `ros2_lf_ingress_*` | The bounded ingress queue of a node: capacity, register, move, drain, close |
| `ros2_lf_corr_*` | Publication correspondence: emit, arrive, drain and flush records of an admitted edge, local and remote |
| `ros2_lf_xtag_*`, `ros2_lf_xfed_legacy_sent` | Cross-federate tag transport for legacy edges |
| `ros2_lf_message_lost_context` | Inside a deadline or message-lost callback, what the callback was raised for |

Return values: `ros2_lf_invoke_sub` returns `int` (1 if the callback ran, 0 otherwise, so the
reaction counts a write on its stub outputs only for a callback that ran).
`ros2_lf_invoke_timer` returns `void`. The action `invoke_*` functions take a `count` of
samples, each with a bounded wait, and trace LOST for one that does not arrive.

## Internal Data Structures

### RID table (`rid_table.cpp`, `rid_table_api.h`)

A static vector of `RidEntry` records populated by `ros2_lf_reaction_ids_init()`
before `main()`:

```cpp
struct RidEntry {
    int         reaction_id;    // index into g_handle_table
    std::string node_class;     // e.g. "TimerPublisherNode"
    std::string callback_name;  // e.g. "on_timer"
    std::string kind;           // see kinds below
    std::string topic;          // topic / service / action name; "" for plain timers
    int64_t     period_ns;      // timer period; 0 otherwise
};
```

`kind` values: `"timer"`, `"sub"`, `"srv_send"`, `"srv_server"`, `"srv_client_cb"`,
`"action_goal_send"`, `"action_goal_server"`, `"action_cancel_server"`,
`"action_result_server"`, `"action_goal_cb"`, `"action_feedback"`, `"action_result_cb"`,
`"action_cancel_cb"`. The generated `action_shutdown` reaction gets no RID.

The table is written once (at static-init time) and read-only from then on.
Access is safe without locks because all writes complete before `main()` starts.

`resize_handle_table(RID_COUNT)` is also called during static init to size
`g_handle_table` before any `ros2_lf_bind_node` call can index into it.

### Handle table (`handle_table.hpp`, `rid_table.cpp`)

```cpp
enum class HandleKind {
    TIMER, SUBSCRIPTION,
    SERVICE_SERVER, SERVICE_CLIENT,
    ACTION_SERVER, ACTION_CLIENT,
    UNBOUND,
};

struct HandleEntry {
    HandleKind                            kind = HandleKind::UNBOUND;
    rclcpp::TimerBase::WeakPtr            timer;           // entities are held weakly
    rclcpp::SubscriptionBase::WeakPtr     subscription;
    rclcpp::ServiceBase::WeakPtr          service_server;
    rclcpp::ClientBase::WeakPtr           service_client;
    rclcpp::Waitable::WeakPtr             action_waitable;
    size_t                                action_entity_id = 0;  // 0=GoalClient, 1=ResultClient, 2=CancelClient, 3=FeedbackSubscription
    std::string                           topic;
    std::string                           resolved_topic;
    std::string                           node_class;
    std::string                           callback_name;
    TakeMode                              take_mode = TakeMode::Skip;
    int64_t                               take_timeout_ms = 100;
    bool                                  late = false;       // lifecycle node: handle may appear later
    bool                                  lifecycle = false;  // dispatch asks the lifecycle gate first
    uint64_t                              gate_epoch = 0;
    int64_t                               generation = -1;
};

extern std::vector<HandleEntry> g_handle_table;
```

`g_handle_table` is a global vector indexed by RID. It is:

- **Written** only during startup reactions.
- **Read** during reaction dispatch, only after all startup reactions have completed.
- **Thread safety:** the table is read-only after startup (entries are never written at
  run time), so reads need no lock. An entity's liveness lives in its `weak_ptr`
  referent, checked with `lock()` at each dispatch. (`CMakeLists.txt` builds the vendored
  reactor-c static library with `LF_SINGLE_THREADED 1`; the read-only rule, not that
  flag, is what the dispatch path relies on.)

### Component manager (`component_manager.hpp`)

`LFComponentManager` subclasses `rclcpp_components::ComponentManager`. It adds:

- `load_component(package, plugin_name, executor)` wraps `on_load_node` with
  error recovery, and retries without namespace prefix if the qualified name fails.
- `find_by_class(class_name)` returns the `NodeBaseInterface` for the first
  loaded node whose registered class name contains `class_name` as a suffix.
  This is how `ros2_lf_bind_node("TimerPublisherNode")` resolves to the node
  even when the plugin was registered as `basic_components::TimerPublisherNode`.
- `get_all_components()` returns all `NodeInstanceWrapper` objects.

Loaded nodes are created through `create_node_options()`, which forces
`use_sim_time=true` and turns the TimeSource clock thread off. After loading,
`lf_attach_time_source()` registers the node's TimeSource `/clock`
subscription, and `lf_time_advance_ros_clock()` feeds it the current tag in
memory before every reaction, so the node's `RCL_ROS_TIME` clocks equal LF
logical time. The subscription is never spun and is excluded from reaction
dispatch. Non-component nodes are attached through
`ros2_lf_register_node()`, which sets the ROS time override on
`node->get_clock()` directly.

---

## Time source (`time_source.cpp`)

Implements [ADR 0001](../design/adr/0001-lf-owns-time.md) via the RCL time-override API:

```
lf_clock_init()
  └─ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)   → g_lf_clock
  └─ rcl_enable_ros_time_override(g_lf_clock)

lf_time_advance_ros_clock()   (called at top of every invoke_timer / invoke_sub)
  └─ _lf_get_environments(&envs)                     → envs[0].current_tag.time
  └─ rcl_set_ros_time_override(g_lf_clock, logical_ns)
  └─ if the tag changed:
       └─ rcl_set_ros_time_override(clock, logical_ns)    for each registered node clock
       └─ sub->handle_message(Clock{logical_ns})          for each component TimeSource
```

A program with more than one LF environment stops at the first advance, since
node clocks follow one logical timeline.

`_lf_get_environments` is a symbol generated by `lfc` into the final binary. The
shared library itself is built with `-Wl,--allow-shlib-undefined` so the linker
does not complain about the unresolved reference at library build time.

**Why feed the TimeSource of component nodes?**  
A component node is only visible to the binding through `NodeBaseInterface`,
which does not expose its clock. Its TimeSource does own every clock of the
node, so the binding calls the TimeSource subscription's `handle_message()`
synchronously. No DDS sample is taken and the clock thread is off, so the
`/clock` race that ADR 0001 rejects cannot occur. Registered non-component
nodes expose `get_clock()` and use the override directly.

---

## Clock gate (`clock_gate.cpp`, `clock_guard.cpp`)

`clock_gate.cpp` is installed to `share/ros2_lf_binding/src` and compiled into
every generated executable next to `publication_gate.cpp`. It defines
`rclcpp::Clock::now()`, `rclcpp::Clock::sleep_until()`,
`rcl_clock_add_jump_callback()` and `rcl_clock_fini()`, calls a hook in
`clock_guard.cpp` and forwards to the library definition through
`dlsym(RTLD_NEXT)`. `sleep_for()` reaches `sleep_until()` through the PLT, so
both are covered. The same holds for a node loaded from a component library.

`clock_guard.cpp` keeps a registry of the clocks the binding attached by its
own calls, `g_lf_clock` and every clock `ros2_lf_attach_node_clock()`
attaches. `rcl_clock_fini()` removes a clock, so a later clock at the same
address starts unknown. A component's clocks are not in the registry, since the
binding feeds the component's TimeSource and never holds them. The gate
refuses only what it can identify reliably and fails open everywhere else.

- A sleep on ROS time inside a reaction on a clock the binding attached stops
  the program with a FATAL message naming the node and callback.
  A sleep on any other clock is not checked, a component's clock included, so
  a component that sleeps on ROS time in a reaction hangs. A sleep in a
  constructor or on a thread the node starts itself is not checked either.
  Steady and system clocks are untouched.
- A forward jump handler added to a clock the binding attached stops the
  program, whether it is an `rclcpp::JumpHandler` or a raw
  `rcl_clock_add_jump_callback()` callback. Exempt are the handler
  `sleep_until()` adds for itself and the callback `rcl_timer_init()` adds to
  every timer on a ROS clock. A handler added before the clock is attached and
  a handler on a component's clock are not checked.
- A read inside a callback of a `RCL_ROS_TIME` clock whose override is off
  logs an ERROR once per callback. A read in a constructor before the first tag
  logs a WARN once per node with the value returned.

The node and callback names come from `trace_callback_start()`, which every
dispatch site calls before the user callback. The refusal ends the process with
`_Exit` rather than an exception, since dispatch paths catch
`std::runtime_error` from user code. The ADR 0001 amendment of 2026-10-11 lists
every case.

---

## Unit Tests

The gtest suites live in [`packages/lf-frame/ros2_lf_binding/test/`](https://github.com/CPS-RWTH/rcllf/tree/main/packages/lf-frame/ros2_lf_binding/test) (more than fifty files; read the directory for the full list). These suites are the ones this page refers to:

| Suite | What it tests |
|-------|---------------|
| `test_rid_table` | `register_rid_entry`, `resize_handle_table`, `get_rid_entries`, default `HandleEntry` state |
| `test_time_source` | `lf_clock_init` creates a `RCL_ROS_TIME` clock with override enabled, `lf_time_advance_ros_clock` sets the correct nanosecond value, and null-clock and zero-time guard paths |
| `test_clock_gate` | A ROS-time sleep in a reaction on an attached clock and a forward jump handler added to an attached clock (rclcpp or raw rcl), stop the program with the expected message. Without the guard the sleep hangs and the handler fires at every tag. A sleep on a clock the node drives itself (even one that stands still for 3 s), on a component's clock, in a constructor, or on a reused clock address is not refused, as is a thread already sleeping when its clock is attached. Timers on LF clocks, steady sleeps, backward handlers, reads and several environments are covered |
| `test_node_clock` | `now()` of a registered `Node` and `LifecycleNode`, a timer clock and a component node built with the manager's `create_node_options()` equals the synthetic tag to the nanosecond |
| `test_startup_bound` | A start whose node construction blocks for good in the rmw publish (the test defines `__rmw_publish` and stalls its first main-thread call) ends at `RCLLF_STARTUP_TIMEOUT_MS` with code 124 and the bound's message; a healthy start that reached the discovery wait is not cut off; a stop bound armed before the watchdog's exit decision (forced through a test-only expiry hook) governs the stuck start, one armed after it leaves the startup exit, and with `RCLLF_STOP_TIMEOUT_MS=0` a signal leaves the start to the startup bound |
| `test_stop_bound` | (The bound re-arms on progress within `RCLLF_STOP_MAX_MS`). A process signalled while its LF stop cannot complete (a federate whose RTI is gone) still ends after `RCLLF_STOP_TIMEOUT_MS` with code 124, so does a process signalled during a teardown that blocks, a slow teardown without a signal is not cut off, and with `RCLLF_EXIT_WITH_PARENT=<launch pid>` it ends when its parent dies (ros2 launch exits on SIGTERM without stopping its processes), also when the parent died before init; `RCLLF_EXIT_WITH_PARENT=1` is refused |
| `test_reaction_dispatch` | Out-of-range and unbound `reaction_id` guard paths for `ros2_lf_invoke_timer`, `ros2_lf_invoke_sub`, and the service/action `invoke_*` family |

Each suite that calls `_lf_get_environments` provides a local stub definition that
injects a synthetic logical time, so the tests run without the lfc toolchain.

Run inside Docker:

```bash
docker compose --profile lf run --rm test-binding
```

---

## Startup Sequence (full picture)

```
process start
    │
    │  __attribute__((constructor))
    ▼
ros2_lf_reaction_ids_init()         ← generated by lf_generator.py
    register_rid_entry(...)          ← populates g_rid_entries
    resize_handle_table(RID_COUNT)   ← allocates g_handle_table

ros2_lf_runtime_init(argc, argv)    ← called by the generated ros2_lf_startup.cpp
    rclcpp::init()
    lf_clock_init()
    parse SYSTEM_JSON, create LFComponentManager

ros2_lf_runtime_load_nodes()        ← after node_factories.cpp registered the nodes
    LFComponentManager::load_component(pkg, class) × N
    non-component nodes built from their registered factories

ros2_lf_runtime_wait_ready()        ← DDS discovery, before lfc sets the start time

lfc's main()
    │
    │  lfc initialisation phase — fires all startup reactions
    ▼
ros2_lf_bind_node("NodeClassA")
ros2_lf_bind_node("NodeClassB")
    ...
    └─ g_handle_table[rid] = {timer|sub handle}

    │  lfc steady-state — fires timer/sub reactions at logical tags
    ▼
ros2_lf_invoke_timer(self, RID_X)
    lf_time_advance_ros_clock()
    g_handle_table[RID_X].timer->execute_callback()

ros2_lf_invoke_sub(self, RID_Y, "/topic")
    lf_time_advance_ros_clock()
    sub->take_type_erased() → sub->handle_message()

    │  lfc shutdown
    ▼
ros2_lf_runtime_shutdown()
    rclcpp::shutdown()
```
