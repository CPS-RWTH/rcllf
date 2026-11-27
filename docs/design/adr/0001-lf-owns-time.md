# ADR 0001, LF Is the Sole Time Authority

**Date:** 2026-04-20
**Status:** Accepted

## Context

The LF executor replaces the ROS 2 executor's `spin()` with the reactor-c
scheduler. reactor-c owns logical time: it fires reactions at tags computed
from timer periods and `after` delays, not from wall-clock polling.

ROS 2 nodes call `Node::now()` internally (e.g. for message stamps) and
occasionally in user code. By default `rclcpp::Clock(RCL_ROS_TIME)` returns
wall time. If we leave that unchanged:

- `Node::now()` returns wall time while LF logical time advances
  independently.
- Any callback that makes a scheduling decision from `now()` reintroduces
  non-determinism LF is supposed to eliminate.
- Under load, wall time and logical time diverge, and the divergence is
  application-visible.

One alternative is to publish LF logical time on the `/clock` topic and set
`use_sim_time=true` on every node so `rclcpp::Clock` subscribes to it.
This was explicitly rejected (see Consequences below).

## Decision

**LF is the sole time authority.** Concretely:

1. `rclcpp::Clock(RCL_ROS_TIME)` for every node in the LF-controlled process
   is rebound to return `lf_time_logical()` directly, without going through a
   `/clock` topic.
2. This is implemented in `packages/lf-frame/ros2_lf_binding/src/time_source.cpp`
   as a custom `LFTimeSource : public rclcpp::TimeSource`. At `ros2_lf_runtime_init`,
   the custom time source is attached to the `rclcpp::Context` and
   `use_sim_time=true` is set on each constructed node so `RCL_ROS_TIME` queries
   route through it.

   *Note (v0.1.2): no class `LFTimeSource` exists; this mechanism was never
   built. The decision stands with the mechanism described under
   [Amendment 2026-10-09](#amendment-2026-10-09-how-node-clocks-are-actually-bound):
   a non-component node's clock gets a ROS time override set to the current tag
   before each reaction, and a component gets `use_sim_time=true` with a `/clock`
   reader that no executor spins and that the binding feeds in memory. Both live
   in `time_source.cpp`.*
3. `std::chrono::steady_clock` and `RCL_STEADY_TIME` are **not** intercepted.
   User contract: callbacks must not use steady time for scheduling decisions.
   Using it for diagnostics (latency measurement, debug prints) is fine.
4. No `/clock` topic is created, published, or subscribed to.
5. All ROS 2 timers are cancelled after node construction. Timer invocation is
   solely reactor-c-driven.
6. DDS deadline QoS is wall-clock and becomes meaningless under LF. Nodes that
   rely on deadline-QoS callbacks for behaviour must be rewritten or run outside
   LF.

## Consequences

**Accepted:**
- `Node::now()` returns logical time. Two reactions at the same tag see
  identical `now()` values, consistent and deterministic.
- Timer co-scheduling (all LF timers sharing a common logical origin T=0)
  is an LF language-level guarantee.
- No extra DDS traffic for time. No round-trip latency between LF scheduler
  and `rclcpp::Clock`.

**Rejected alternative, `/clock` publication:**
- Splits time authority: LF controls logical time but `rclcpp::Clock` reads
  from a DDS topic. Under load, subscribers lag the publisher by at least one
  DDS round-trip. Different nodes read slightly different "current" times.
- Requires every node to have `use_sim_time=true` plumbed via launch args,
  adding configuration surface.
- The ordering guarantee would become conditional on the DDS round-trip
  staying within the clock-topic QoS deadline, not an unconditional guarantee.

**Known limitations:**
- `steady_clock::now()` in user callbacks is wall-clock. If a callback
  computes a timeout using `steady_clock` and that timeout drives a
  `create_wall_timer` call (at runtime, not at init), the resulting timer will
  not be registered with reactor-c. Document that all timers must be created at
  node construction time.

  *Note (v0.1.2): the analyzer reports such a timer as
  [RCLLF-H006](../../reference/diagnostic-codes.md#rcllf-h006).*
- Third-party packages that internally poll `RCL_STEADY_TIME` for their own
  scheduling (e.g. `rcl_timer_is_ready` checks) are unaffected by this ADR,
  since their timers are cancelled anyway.

## Amendment 2026-04-25, Optional /clock publication for observability

The original decision rejected /clock because it would split *time authority*:
in-process consumers reading time via DDS would see stale values vs LF's
in-memory override. That reasoning still holds for in-process consumers.

This amendment adds *observability-only* /clock and /lf_clock publication,
gated by `ROS2_LF_CLOCK_PUBLISH` env var (default `on_change`, also `off` and
`every_reaction`).

Crucially:

- In-process clock authority is unchanged. `g_lf_clock` continues to be
  driven by `rcl_set_ros_time_override` directly from `lf_time_advance_ros_clock`.
  Component nodes loaded by `LFComponentManager` do NOT subscribe to /clock.
- /clock and /lf_clock exist purely for **external observers**: rosbag, rqt,
  plotjuggler, out-of-process diagnostic nodes.
- The publisher is a single runtime-owned node (`"lf_clock_publisher"`) with
  one publisher for each topic. No user node or generated reactor publishes time.

The split-authority risk the original ADR worried about does not arise: no
in-process consumer is ever told to use_sim_time against /clock under LF control.

## Amendment 2026-10-09, How node clocks are actually bound

The mechanism in Decision item 2 was never wired up.
`g_lf_clock` followed LF time, but every node read its own `RCL_ROS_TIME`
clock, so `Node::now()` returned physical time inside reactions. The decision
stands. The mechanism is now as follows.

- **Non-component nodes** pass through `ros2_lf_register_node()`, which hands
  `node->get_clock()` to the binding. The binding enables the ROS time override
  on that clock and sets it to the current tag before every reaction, together
  with `g_lf_clock`. Timers created on the node clock share its handle and
  follow too.
- **Component nodes** are only visible type-erased. `LFComponentManager`
  overrides `create_node_options()` to force `use_sim_time=true` and turn the
  TimeSource clock thread off. The node's TimeSource then owns a `/clock`
  subscription that no executor spins. Before every reaction the binding hands
  that subscription a `rosgraph_msgs/Clock` message in memory, which sets
  every clock attached to the TimeSource. No DDS sample is ever taken from
  it, so the split-authority race described above cannot occur.
- A non-component node whose own parameters set `use_sim_time=true` is taken
  off `/clock` at registration. The binding sets `use_sim_time=false` and
  hands the TimeSource the matching parameter event in memory, so it joins
  its clock thread and drops its `/clock` subscription. If that fails, the
  node is refused.
- A component whose TimeSource cannot be attached is unloaded and its load
  fails.
- The binding takes each clock's own mutex, the one the TimeSource holds,
  whenever it sets the clock.
- The binding sets the start tag in `ros2_lf_bind_node()`, before lifecycle
  callbacks run. Before the first tag, `now()` returns 0.

The 2026-04-25 statement that component nodes do not subscribe to `/clock` is
superseded. They hold an unspun reader, and the binding excludes it from
reaction dispatch.

**Not covered.** `rclcpp::Clock` objects a node creates itself, steady and
system clocks, waits on ROS time inside a reaction (they would block, since
time does not advance within a tag). Only the first LF environment's tag is
used.

## Amendment 2026-10-11, Clock uses rcllf refuses or reports

The decision stands. ROS time follows LF logical time for every node clock the
binding attaches. The cases the 2026-10-09 amendment left open now each have an
explicit answer. A clock gate compiled into every generated executable
(`clock_gate.cpp`, next to the publication gate) sees `rclcpp::Clock::now()`,
`rclcpp::Clock::sleep_until()`, `rcl_clock_add_jump_callback()` and
`rcl_clock_fini()`. A clock counts as attached only when the binding attached
it by its own call (`g_lf_clock` and `ros2_lf_attach_node_clock()`), and
`rcl_clock_fini()` forgets it.

The gate refuses only what it can identify reliably and fails open everywhere
else. Stopping a working program is worse than
missing a rare mistake. Rows marked "not covered, fails open" give the reason.

| Case | Answer |
|---|---|
| Sleep on ROS time inside a reaction (`sleep_for`, `sleep_until`) on a clock the binding attached | Refused. The program stops with a FATAL message naming the node and callback, instead of hanging. The analyzer reports the call as [RCLLF-H008](../../reference/diagnostic-codes.md#rcllf-h008) (sleep on the node's ROS clock). |
| Not covered, fails open: sleep on ROS time inside a reaction on a component's clock (fed through its TimeSource) | Not refused, so it hangs as it would without the gate. The binding feeds the component's TimeSource and never holds its clocks, so it has no sign that a clock is LF driven. A clock that stands still for a while may still move later, so a stall alone does not prove it. The analyzer reports the call as RCLLF-H008. |
| Not covered, fails open: sleep on ROS time in a constructor | The gate cannot tell the constructed node's own clock from another node's clock reliably, and a node built after LF time started would see its clock move. The sleep is allowed. The analyzer still reports it as RCLLF-H008. |
| Sleep on a steady or system clock, or on ROS time from a thread the node starts itself | Allowed. The user thread waits for logical time and returns. |
| A thread already sleeping on ROS time when the binding attaches the clock | Allowed. Nothing is checked at attach time. |
| A clock destroyed, then another built at the same address | The gate forgets a clock in `rcl_clock_fini()`, so the new clock is judged on its own. |
| A `RCL_ROS_TIME` clock the node builds itself, override off | Not driven. The first read inside each callback logs an ERROR that names the node and callback. The value stays the clock's own. A sleep on it is allowed. |
| Not covered, fails open: a `RCL_ROS_TIME` clock whose override the node enables and drives itself | Not driven, and its reads are not reported. The gate cannot tell it from a component's clock, which LF time drives. A sleep on it is allowed. |
| `now()` in a constructor, before the first tag | Not driven, since no start time exists yet. A WARN once per node states the value returned (0 for a component, physical time for a registered node). |
| A forward jump handler added to a clock the binding attached, rclcpp or raw rcl | Refused when it is added. The callback rcl adds for each timer and the handler `sleep_until()` adds for itself are exempt. LF time jumps forward at every tag, so the handler would run on an LF worker outside the node's reactions. Backward and clock-change handlers (tf2_ros::Buffer) stay allowed. |
| Not covered, fails open: a forward jump handler added before the binding attaches the clock | Not refused. Finding it at attach time means reading rcl's handler array, which rcl offers no synchronised reader for. |
| Not covered, fails open: a forward jump handler on a component's clock | Not refused. The binding feeds the component's TimeSource and never holds its clocks, so it cannot name them when the handler is added. |
| The unspun `/clock` reader of each component | Driven. It is the in-memory channel described above, one reader per component. |
| An LF program with several environments (enclaves) | Refused when time first advances. The generator never emits enclaves. |
