# Diagnostic Codes

Each entry gives the severity, where the code is raised, what it means, what to do, and whether it can be
listed under `acknowledged_hazards:` in `execution.yaml`.

- **error**: generation refuses unless the code is acknowledged. Not every error can be acknowledged (see the entry).
- **warning**, **info**: printed, never blocking.
- **Raised at**: *analyze* (`colcon lf-analyze`, stored in `system.json`), *generate gate* (`colcon lf-generate`
  reads the findings and enforces severity), *start-up* or *run* (the binding, when the program runs).
- Acknowledging an error skips the check only; the problem stays in the generated program.
- A message line marked *real* was printed by a run of a packaged example; *template* is the message text as
  the code builds it, with `<placeholders>`.

For `acknowledged_hazards`, see [execution.yaml](../features/execution-config.md). The support state of each feature
is in the [support matrix](../features/support-matrix.md).

## RCLLF-H000 {#rcllf-h000}

**Unknown QoS reliability**

- Severity: info
- Raised at: analyze; start-up
- Can be acknowledged: No (info prints only).

**Meaning.** The analysis could not read the QoS reliability of a subscription or publisher (an expression it cannot resolve). The checks for H001 and H002 cannot run on it, so it is a blind spot in the analysis, not a measured risk. At start-up the binding logs the effective value, for example `rcllf corr: RCLLF-H000 subscription RID 5 (DetectorNode::image_callback) on '/sentry/image': the analysis saw no reliability, the subscription runs best effort`.

**What to do.** Write the QoS as a literal or a named profile so the analysis can read it, or check the effective reliability by hand. H001 and H002 are re-checked on the effective value at start-up.

Message (template):

```text
subscription '<topic>' on node '<node>': QoS reliability could not be determined during analysis (unrecognized QoS expression) - treat the effective reliability as unknown, not safe.
```

## RCLLF-H001 {#rcllf-h001}

**Best-effort input subscription**

- Severity: warning
- Raised at: analyze; start-up
- Can be acknowledged: No (warning prints only).

**Meaning.** A subscription uses `best_effort` reliability, so the transport may drop a message. A dropped message is not a late one: under logical time it can leave a reaction waiting with no error. The code becomes info when every edge into the subscription is admitted for correspondence, because a drop is then reported as a missing outcome.

**What to do.** Use `reliable` QoS, set `take_mode: block` for the subscription, or accept the risk knowingly. See [Missing messages](../features/missing-messages.md).

Message (template):

```text
subscription '<topic>' on node '<node>': best_effort QoS reliability - messages may be dropped in transit; LF's deterministic tag semantics do not compensate for transport drops.
```

## RCLLF-H002 {#rcllf-h002}

**Depth-1 subscription queue**

- Severity: warning
- Raised at: analyze; start-up
- Can be acknowledged: No (warning prints only).

**Meaning.** A subscription has queue depth 1 on a topic with a publisher in the same system. A burst of publishes between two reactions keeps only the newest message. For admitted edges the binding raises the reader to `reader_depth` (default 8); start-up logs, for example, `RCLLF-H002 admitted edge 5 PlannerNode -> ControllerNode on '/sentry/plan': ... rcllf keeps 8 sample(s) in the reader`.

**What to do.** Nothing, if you only want the latest value. Otherwise raise the depth in the node. It never refuses a run.

Message (template):

```text
subscription '<topic>' on node '<node>': queue depth 1 with an in-model publisher - a burst of publishes between reactions will drop all but the most recent message.
```

## RCLLF-H003 {#rcllf-h003}

**message_filters outside the admitted two-input form**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: Yes.

**Meaning.** The node uses `message_filters` in a form rcllf does not support. The message lists the reasons, including `sync_input_legacy:<reason>` (an input on a legacy edge) and `sync_input_delay_unequal` (the two inputs carry unequal delays), or asks you to re-run `colcon lf-analyze` when the scan result is missing. The two-input forms are supported, see [message_filters](../features/message-filters.md).

**What to do.** Rewrite the node to the two-input form, or list the code under `acknowledged_hazards`; the unsupported synchronizer is then not modelled.

Message (template):

```text
node '<node>' uses message_filters in a form rcllf does not support: <reasons>. Supported is a ... Generation refuses ...
```

## RCLLF-H004 {#rcllf-h004}

**Dynamically computed topic name**

- Severity: warning
- Raised at: analyze
- Can be acknowledged: No (warning prints only).

**Meaning.** A publisher, subscription or other endpoint builds its topic name at run time (concatenation, a member, a parameter or a computed expression), so the analysis cannot resolve it and the connection may be missing from the generated model.

**What to do.** Use a string literal or a `constexpr`/`const char[]` constant if the edge must appear.

Message (real):

```text
publisher (name unresolved) on node 'dynamic_publisher_node': the topic name is built at runtime (concatenation, member/parameter reference, or computed expression), so it could not be resolved statically - this connection may be missing from the extracted graph. Use a string literal or a `constexpr`/`const char[]` constant if this edge must appear in the generated model.
```

## RCLLF-H005 {#rcllf-h005}

**Lifecycle hook override skips the base call**

- Severity: warning
- Raised at: analyze
- Can be acknowledged: No (warning prints only).

**Meaning.** A managed node overrides `on_activate()` or `on_deactivate()` without calling the `rclcpp_lifecycle::LifecycleNode` base. The base call activates the managed publishers, so every `publish()` is silently dropped.

**What to do.** Chain to the base: `return rclcpp_lifecycle::LifecycleNode::on_activate(state);`.

Message (real):

```text
node 'unchained_activate_node' overrides on_activate() without calling rclcpp_lifecycle::LifecycleNode::on_activate(), so its managed publishers are never activated, so every publish() on 'unchained_output' is silently dropped. ... Add `return rclcpp_lifecycle::LifecycleNode::on_activate(state);` ...
```

## RCLLF-H006 {#rcllf-h006}

**Timer created outside the constructor or a lifecycle hook**

- Severity: warning
- Raised at: analyze
- Can be acknowledged: No (warning prints only).

**Meaning.** The node creates a timer in an ordinary method or callback. The binding binds timers once at start-up and only sees timers that already exist, so the timer is not extracted and its callback never runs in the generated program.

**What to do.** Create the timer in the constructor or a managed-lifecycle hook.

Message (template):

```text
node '<node>' creates timer '<var>' (callback '<cb>') inside '<method>', not in the constructor or a managed-lifecycle hook. The runtime's one-shot startup bind pass (ros2_lf_bind_node) can only observe timers that already exist by then, so this timer is not extracted into the generated system: '<cb>' never runs under lfc.
```

## RCLLF-H007 {#rcllf-h007}

**Callback in a form the analyzer cannot model**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: Yes.

**Meaning.** A timer or subscription is created with a callback argument that is neither `std::bind(&Node::method, this)` nor an inline lambda written as the whole argument (for example a callback stored in a local variable first). The analyzer cannot find the callback, so no reaction is generated for it.

**What to do.** Pass the callback as `std::bind(&Node::method, this)` or an inline lambda. Acknowledging the code only skips the check: the timer or subscription gets no reaction and never fires.

Message (real):

```text
node 'talker' creates timer 'timer_' (period 1000 ms) with callback argument `publish_message`, which is neither form the analyzer can model. ...
```

## RCLLF-H008 {#rcllf-h008}

**Sleep on the node's ROS clock**

- Severity: warning
- Raised at: analyze; run
- Can be acknowledged: No (warning prints only).

**Meaning.** A callback calls `get_clock()->sleep_for()`. The node's clock follows logical time, which does not advance while a reaction runs, so the sleep never returns; rcllf stops the program with a FATAL message.

**What to do.** Use a timer for a delay, or an `RCL_STEADY_TIME` clock for a wall-clock wait.

Message (real):

```text
node 'clock_ros_sleep' calls get_clock()->sleep_for() at clock_ros_sleep_node.cpp:45. The node's clock follows LF logical time, which does not advance while a reaction runs, so inside a callback the sleep never returns and rcllf stops the program with a FATAL message. Use a timer for a delay, or an RCL_STEADY_TIME clock for a wall-clock wait.
```

## RCLLF-H009 {#rcllf-h009}

**Blocking service client**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: Yes.

**Meaning.** A client waits for its response inside the callback (`spin_until_future_complete`, `future.wait_for`, `future.get`, or a loop on `spin_some`/`spin_once`, a condition variable or a flag). Under rcllf the response is a later LF event, so the wait times out on every call.

**What to do.** Use `async_send_request` with a response callback, see [Architecture](../design/architecture.md#communication-primitives). Acknowledging generates the program with the timeouts.

Message (real):

```text
node 'blocking_future_client' calls service 'process_data' and blocks on future.wait()/wait_for()/get() (call style blocking_future). Under rcllf the response is a later LF event, so a wait inside a callback times out on every call, and a wait outside any callback is never triggered. Use async_send_request with a response callback.
```

## RCLLF-H010 {#rcllf-h010}

**BoundedNearestToAnchor synchronizer admitted**

- Severity: info
- Raised at: analyze
- Can be acknowledged: No (info prints only).

**Meaning.** The node uses the rcllf `BoundedNearestToAnchor` synchronizer, not `ApproximateTime`. It is supported; each input topic needs `stamp_provenance` in `execution.yaml`.

**What to do.** Declare `stamp_provenance` for each input topic, see [BoundedNearestToAnchor](../features/bna-synchronizer.md).

Message (template):

```text
node '<node>' synchronizer <member> uses BoundedNearestToAnchor (<params> ns), the rcllf refinement, not ApproximateTime. Each input topic needs stamp_provenance in execution.yaml.
```

## RCLLF-H011 {#rcllf-h011}

**Action server sending from its own thread**

- Severity: error
- Raised at: generate gate; run
- Can be acknowledged: Yes.

**Meaning.** An action server sends feedback or a result from a thread it starts (an `execute()` thread from `handle_accepted`, or one started in the constructor). The tag at which the send reaches the client depends on thread scheduling, and at run time the binding refuses the send (`rcllf: refused` in the log, a `refused` trace line).

**What to do.** Queue the goal in `handle_accepted` and advance it from a timer callback, as [action_example](../examples/action-example.md) does. Acknowledging generates the program, but the thread's sends are still refused at run time.

Message (template):

```text
node '<node>' sends feedback or a result on action '<action>' from thread '<thread>', started in <where>. Under rcllf an action send needs an LF origin, so the runtime refuses a send from the node's own thread and the client never gets it. Advance goals from a timer callback.
```

## RCLLF-H012 {#rcllf-h012}

**Request sent from a service handler or response callback**

- Severity: error
- Raised at: generate gate; run
- Can be acknowledged: Yes.

**Meaning.** A service handler or a client's response callback sends a request (`async_send_request`) or an action goal. A request is carried by the port of the timer or subscription reaction that sends it; a handler or response callback has no port, so its server would take the request in place of a later in-system one.

**What to do.** Send the request from a timer or subscription callback. If acknowledged, the binding drops the send's count at run time with a warning naming RCLLF-H012 and the response callback does not run.

Message (template):

```text
node '<node>': callback '<cb>' (<role>) sends a request or goal on '<target>'. Under rcllf a request is carried by the port of the timer or subscription reaction that sends it; ... Send it from a timer or subscription callback instead.
```

## RCLLF-H013 {#rcllf-h013}

**Endpoint created outside the constructor**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: Yes.

**Meaning.** A subscription, service, client or publisher is created in an ordinary method (for example inside a timer callback). The start-up pass runs once, so the endpoint is never bound and its callback never runs.

**What to do.** Create the endpoint in the constructor. Acknowledging generates the program, and the late endpoint then delivers nothing.

Message (template):

```text
node '<node>' creates <kind> '<topic>' in '<method>' (callback <cb>); under rcllf its callback never runs. Create it in the constructor.
```

## RCLLF-H014 {#rcllf-h014}

**Work in main() outside callbacks**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: Yes.

**Meaning.** rcllf replaces `main()`, so a `spin_some` loop that calls a publishing node method, or a publish written in `main()`, never runs.

**What to do.** Move the work into a timer callback.

Message (real):

```text
main() of blocking_spin_client_node.cpp runs node work outside callbacks (send_request); rcllf replaces main(), so it never runs. Move it into a timer.
```

## RCLLF-H015 {#rcllf-h015}

**Endpoints in a template base class**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: Yes.

**Meaning.** The node's base class is a template that creates endpoints in its constructor. The analyzer cannot walk a template constructor, so the endpoints would be missing silently.

**What to do.** Use a non-template base class in your workspace (those are walked) or create the endpoints in the derived class.

Message (template):

```text
node '<node>' inherits endpoint-creating code from template base <base> (<calls>); ...
```

## RCLLF-H016 {#rcllf-h016}

**Ambiguous compile variant**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: Yes.

**Meaning.** One source file is built into several executables with different compile definitions (an `#ifdef` selecting `main()` or the node class), and rcllf cannot tell which variant belongs to this executable.

**What to do.** Declare the selecting definition with `target_compile_definitions(<exe> PRIVATE NAME)` on that executable's target.

Message (template):

```text
<source>: <reason> (the executable's compile definitions cannot be determined, so the wrong #ifdef variant may be analyzed)
```

## RCLLF-H017 {#rcllf-h017}

**Intra-process communication**

- Severity: error
- Raised at: analyze; generate gate; start-up
- Can be acknowledged: Yes.

**Meaning.** A composable node's launch file sets `use_intra_process_comms` to true, or its source builds options with `.use_intra_process_comms(true)`. rclcpp then hands a message to a subscription in the same process without the middleware, so the message has no publisher identity for rcllf to order. The binding refuses to start a publication edge of such a node (`intra_process` start-up failure).

**What to do.** Remove the option or set it to `False` (the default). Messages between nodes of one process still use the middleware.

Message (real):

```text
composable node 'intra_process_publisher' sets extra_arguments use_intra_process_comms to True in its launch file. Under rcllf a publication must pass the middleware to have an identity, and the binding refuses to start an edge of a node with intra-process communication on. Remove the option, or set it to false.
```

## RCLLF-R001 {#rcllf-r001}

**Timer period unresolved**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: No. Listing it in `acknowledged_hazards` no longer proceeds.

**Meaning.** The resolved configuration model has a timer whose period the analysis could not resolve. See [Resolved model](resolved-model.md).

**What to do.** Supply the period in the launch or source, or set `nodes.<node>.timers.<callback>.period` in `execution.yaml`, and re-run `colcon lf-generate`.

Message (template):

```text
Error: generation refused, RCLLF-R001 timer period unresolved for <node>/<callback> (period_ms is null in system.json; supply it explicitly or set nodes.<node>.timers.<callback>.period in execution.yaml)
```

## RCLLF-R002 {#rcllf-r002}

**tf2 or take() mechanism unresolved**

- Severity: error
- Raised at: generate gate
- Can be acknowledged: Yes; generation then proceeds without making tf2 deterministic.

**Meaning.** A node depends on `tf2` or on `take()` in a way the resolved model cannot make deterministic. Subcodes: `tf2_direct_lookup`, `tf2_blocking_lookup`, `tf2_listener_unadmitted`, `tf2_clock_mismatch`, `tf2_stamp_mismatch`, `tf2_stamp_unknown`. See [tf2](../features/tf2.md).

**What to do.** Fix the cause named by the subcode (see the tf2 page), for example declare the external topic with `anchor: stamp`, or acknowledge to continue as unsupported.

Message (template, one subcode shown):

```text
Error: generation refused, RCLLF-R002 tf2_clock_mismatch: '<topic>' reaches a TransformListener from outside the system with no declared stamp provenance; declare it in execution.yaml external_topics with 'anchor: stamp' (or list RCLLF-R002 in acknowledged_hazards to proceed as unsupported)
```

## RCLLF-R003 {#rcllf-r003}

**Resolved model stale**

- Severity: error
- Raised at: start-up (run gate); `python3 -m colcon_lf.resolved_model check`
- Can be acknowledged: No.

**Meaning.** `resolved_model.json` is missing, or an input file (such as `system.json` or `execution.yaml`) changed after it was written.

**What to do.** Re-run `colcon lf-generate`.

Message (real):

```text
Error: RCLLF-R003 resolved model stale: resolved_model.json missing at <path>; re-run colcon lf-generate
```

## RCLLF-R004 {#rcllf-r004}

**Second server of an in-system service**

- Severity: error
- Raised at: run
- Can be acknowledged: No (a run refusal, not a generation check).

**Meaning.** Not in the feature registry; a run refusal from the binding. A server outside the system serves a service that an in-system server also serves, so it would answer in-system requests too. The binding checks when it binds the server and again at most once a second while serving. Outside clients are fine: they are served at once and counted in `external_requests` of the `ros2_lf_service_stats` line.

**What to do.** Stop the other server or rename the service.

Message (template):

```text
RCLLF-R004: service '...' has N servers on the graph but M in-system ... Refusing to run.
```
