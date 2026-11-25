# tf2 Lookups

A node that looks transforms up through tf2 converts when it uses one form: a
`tf2_ros::TransformListener(*buffer, this)` member and lookups through
`rcllf_tf2::AcquisitionLookup`. tf2 stays the lookup and interpolation authority.
rcllf never computes a transform. It decides **when** the Buffer is fed and **when** a
lookup is evaluated, and both happen at logical tags, so a converted run gives the same
outcomes every time.

## The supported form

```cpp
#include "rcllf_tf2/acquisition_lookup.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
lookup_ = std::make_shared<rcllf_tf2::AcquisitionLookup>(
  *buffer_, this, 55ms, rcllf_tf2::OnTimeout::reject());

lookup_->request("world", "sensor_link", msg->header.stamp,
  [this](const rcllf_tf2::Result & r) { /* r.outcome, r.transform */ });
```

Add `rcllf_tf2` to the package's dependencies. Construct all three objects in the
node's constructor.

- **Ingress.** The binding takes the listener's callback group off tf2's spin thread
  before the first tag. The analyzer adds `/tf` and `/tf_static` subscriptions
  (`tf_ingest_via_<member>`), and each message reaches the Buffer through
  `setTransform` inside an LF reaction, one microstep after the broadcaster's tag.
- **Request.** `request(target, source, stamp, callback)` is evaluated at its tag and
  again after every later `/tf` or `/tf_static` ingest of the node.
- **Outcome.** Every request ends in exactly one outcome:

| Outcome | When |
|---|---|
| `ok` | tf2 returns the transform at `stamp` (at the request tag or at an ingest before the deadline) |
| `reject` / `fallback:<name>` | at the logical deadline `request tag + wait bound`, tf2 still extrapolates; `on_timeout` picks the outcome (`OnTimeout::reject()`, `::failure()` or `::fallback(name)`) |
| `failure` | at the deadline, tf2 reports another error, for example an unknown frame |

A lookup never answers with the latest transform in place of the requested stamp. A
stamp of 0 (tf2's "latest") or below ends in `failure` at the request tag. A request
made outside an LF reaction (from a thread that is not an LF worker) throws
`std::logic_error`. The
deadline is a logical wake that a generated `tf2_wake` reaction delivers; it never
blocks an LF worker. The physical wait is traced, never claimed.

The outcome callback runs inside the reaction that decided it: the request's own
reaction, an ingest reaction or the wake reaction. Those reactions declare no publisher
effects, so the callback must not publish. Store the result and publish from a
reaction of your own.

The wait bound and `on_timeout` are constructor arguments. The analyzer reads them as
literals and records them in the resolved model as `node:<fqn>#tf2.lookup.<member>`
(`wait_bound_ns`, `on_timeout`, `buffer`, `buffer_clock`, `tf2_ros_version`), next to
`node:<fqn>#mechanism.tf2`.

The stamp a request asks for has a record of its own,
`node:<fqn>#tf2.sample_stamp.<member>`. The Buffer runs on the node clock (LF logical
time under rcllf), so the stamp must be a time on that clock. The analyzer follows the
stamp argument of every `<member>->request(...)` call (through `rclcpp::Time(...)`,
durations added or subtracted, and locals or members set from one source) and admits
two sources:

- `node_clock`: the node's own `now()` or `get_clock()->now()` (`extracted`);
- `message`: `<msg>->header.stamp` of a message the node receives in a subscription
  callback, whose topic a node of the system publishes (`source: in_system` with its
  publishers, `extracted`) or `execution.yaml` declares under `external_topics` with
  `anchor: stamp` (`source: external_stamp`, `explicitly_configured`).

Any other clock (`RCL_SYSTEM_TIME`, `RCL_STEADY_TIME`, `std::chrono` clocks, a separate
`rclcpp::Clock`) or a message from outside the system without a stamp anchor is
`tf2_stamp_mismatch`. A stamp the scan cannot follow (a constant, a variable set from
two sources, a message variable that is no subscription callback's), and an adapter
with no `request` call the scan sees, is `tf2_stamp_unknown`. The analyzer does not
check how the producer of a message stamps it: an in-system publisher's `header.stamp`
is whatever that node writes.

What the stamp scan proves, and what it assumes: it admits only stamp forms it proves
from the clang AST of the node's own translation unit and refuses everything else.
Every name is the declaration the compiler resolved, never its spelling (a local
constant named `RCL_ROS_TIME` is not rcl's clock enumerator). A message parameter keeps
its subscription's provenance only when the callback reads it and never writes it,
the message it points to included. A callback method counts only when the node passes it to
`create_subscription` as `std::bind(&Node::method, this, _1)` and never calls it itself. The scan assumes that no `const_cast` removes const
to write a stamp, and it does not look inside called functions: a write in a callee the
node's translation unit does not show is out of scope, like a listener hidden inside an
opaque library.

`execution.yaml` can set both per node, and then the yaml wins over the literals:

```yaml
nodes:
  acq_sampler:
    tf2:
      wait_bound: 125 msec        # an LF time literal, > 0
      on_timeout: failure         # reject | failure | fallback:<name>
```

Either key alone is allowed. The keys apply to every `AcquisitionLookup` of the node
(the adapter does not know its member name when it registers). The record becomes
`explicitly_configured`, keeps the constructor values under `literal`, and lists the
keys it took from yaml under `configured`. Under rcllf the adapter logs
`rcllf_tf2: execution.yaml nodes.<node>.tf2 sets wait bound ... (constructor literal
...)` at construction. Under stock ROS there is no binding, so the constructor literals
stay in force. A `tf2` block on a node without an admitted `AcquisitionLookup`, an
unknown key, a bound that does not parse or is 0, and any other `on_timeout` are
refused when the file is loaded. A fallback name is printable ASCII without spaces,
quotes or backslashes, and the whole `on_timeout` value is at most 255 characters
(the runtime hands it over in a 256-byte buffer).

## What generation refuses

Every other tf2 use is refused with RCLLF-R002 and a subcode that names the
conversion. Listing RCLLF-R002 in `acknowledged_hazards` proceeds without a
determinism claim.

| Subcode | Cause | Conversion |
|---|---|---|
| `tf2_direct_lookup` | `buffer->lookupTransform(...)` or `canTransform(...)` without a timeout (or with zero): it answers with whatever the Buffer holds when the reaction runs, the latest transform for `TimePointZero` | `rcllf_tf2::AcquisitionLookup` |
| `tf2_blocking_lookup` | `lookupTransform` or `canTransform` with a non-zero timeout, or any `waitForTransform`: it waits for a `/tf` message that only a later reaction can deliver, so it blocks the LF worker | `rcllf_tf2::AcquisitionLookup` with a wait bound |
| `tf2_listener_unadmitted` | a listener without the node argument, with `spin_thread = false`, custom QoS or options, made outside the constructor, or a second listener; a Buffer or listener the compiler sees but the source scan cannot read (spelled through a macro, alias or deduced type); a `tf2_ros` buffer or listener header included from the package's code with no listener made in the files the compiler reads for that node; an `AcquisitionLookup` whose wait bound is not a positive `std::chrono` literal, without a listener, or on a Buffer the listener does not feed | one `TransformListener(*buffer, this)` member made in the constructor |
| `tf2_stamp_mismatch` | a request stamp read from a clock that is not the node's, or the `header.stamp` of a message from outside the system without `anchor: stamp` | `now()`, or the `header.stamp` of an in-system or stamp-anchored input |
| `tf2_stamp_unknown` | a request stamp whose source the scan cannot follow, or an adapter with no `request` call in the node's files | pass `now()`, `msg->header.stamp` in the subscription callback, or a variable set from one of them |
| `tf2_clock_mismatch` | a Buffer that is not made with the node's own `get_clock()`, or a listener whose `/tf` or `/tf_static` no node of the system publishes and `execution.yaml` does not declare with `anchor: stamp` | a node-clock Buffer; broadcast the transforms from a node of the system, or declare the topic as a stamp-anchored external input (below) |

For example, `colcon lf-generate --package tf_listener_example` refuses `tf_listener`
with `tf2_direct_lookup`. A node that only broadcasts (`TransformBroadcaster`,
`StaticTransformBroadcaster`) converts without a record of its own: its `/tf` is an
ordinary publisher.

## Showcase and checks

`packages/src/tf2_acq_example` runs cases T1 to T7: interpolation, a stamp equal to a
sample, a wait of one `/tf` period, a timeout, a static frame, an unknown frame and a
stamp past the last sample. Its README derives each outcome by hand.

- **Oracle O1** (`test/test_o1_oracle.cpp`): a plain `tf2::BufferCore` replay of the
  same schedule, without the adapter, yields the hand table.
- **Converted run** (`tools/colcon_lf/tests/runtime/test_runtime_tf2_acq.py`): 20 runs
  match the hand table and give byte-identical outcome streams. A mutant that answers
  the latest transform on a timeout (`RCLLF_TF2_TEST_MUTANT=latest_on_timeout`) is named
  at T5. The run's `tf2_ros` must be the pinned 0.36.23, otherwise the check reports
  `version_unpinned:tf2_ros <found>`.
- **Trace.** The adapter writes one `RCLLF_TF2 {json}` line per request (`tf_request`)
  and outcome (`tf_outcome`), and `tf_open_at_shutdown` for a request still open at
  the end. Each line names the node, the adapter (its construction ordinal on the
  node, 0 for the first) and the request id, which counts per adapter, so
  `(node, adapter, id)` is the pairing key. Request and outcome also carry the
  Buffer's available interval for the frame pair at that moment,
  `oldest_common` and `latest_common` in ns (`null` when tf2 cannot connect the
  frames, `"unavailable"` when the path holds no single exact range: a reparented
  child frame, or a dynamic link stamped only at 0). `lf_eval.tf2_acq` reads them and compares them with a hand table. Under
  rcllf each line is also an LTTng event of the `rcllf` provider, `tf2_request` and
  `tf2_outcome` (trace schema 1.8, see the
  [trace schema](../reference/trace-schema.md)), which `lf_eval.semantic.read_ctf`
  reads.

## External `/tf`

`/tf` from outside the system (a bag, a robot driver in another process) is admitted
as a stamp-anchored external input when `execution.yaml` declares it:

```yaml
external_topics:
  - topic: /tf
    anchor: stamp
    delay_ns: 50000000            # late when arrival > stamp + delay_ns
    basis: the driver publishes each transform within 20 ms of its stamp
```

A `tf2_msgs/TFMessage` has no top-level header, so the anchor is the common stamp of
`transforms[*].header.stamp`: the message is then handled like any stamp-anchored topic
([execution.yaml](execution-config.md#anchoring-a-topic-at-headerstamp)). A message whose transforms carry different stamps is
not admitted: it is counted and traced as `tf_mixed_stamps` (the `external_arrival`
disposition 7), logged, and never split or anchored at a guessed stamp. An empty
message is refused the same way. The resolved model records the decision per listener
topic as `topic:/tf#provenance`: `extracted` with `source: in_system` for a topic a
node of the system publishes, `explicitly_configured` with `source: external`, the declared anchor keys
and `tf_rule: common_stamp_or_tf_mixed_stamps` for a declared anchor, and an
unresolved RCLLF-R002 record (`source: external_undeclared`) otherwise.

`/tf_static` follows the same rule when declared. Its publishers are
`transient_local`, and a replayed static transform keeps its old stamp, so one that
is older than `delay_ns` when the system starts is `LATE` and never reaches the
Buffer. Broadcast static frames from a node of the system where that matters.

## Limitations

- `/tf` from outside the system is admitted only as a declared stamp-anchored input
  (above); an arrival-anchored or undeclared external `/tf` is refused
  (`tf2_clock_mismatch`).
- `/tf` follows the closed-world rule every rcllf topic follows. Once a node of the
  system publishes `/tf`, the topic is internal: the external-topic gate
  only looks at topics with no in-system publisher
  (`tools/colcon_lf/colcon_lf/verb/generate.py:1368`), so the listener is admitted.
  A process outside the system that broadcasts on the same `/tf` is not ingested: the
  correspondence layer delivers only samples of writers the system's own records name
  and counts any other sample as `UNRESOLVED`
  (`packages/lf-frame/ros2_lf_binding/src/correspondence.cpp:1884`). Its transforms
  never reach the Buffer, at arrival time or otherwise; move that broadcaster into the
  system.
- `execution.yaml` `nodes.<node>.tf2` applies to all adapters of a node; there is no
  per-member key.
- Under stock ROS (no binding) the adapter runs the same rules with a 5 ms wall-clock
  poll. Stock outcomes of timing-dependent cases are not claimed equal to the rcllf
  ones.
- The LTTng events carry no transform; the JSON line does.
- Detaching waits for a `/tf` callback that tf2's own executor is running. The binding
  starts the LF tags only after both listener subscriptions have left that executor's
  wait set and the group has stayed non-runnable, and refuses to start if that takes
  more than 1 s (`packages/lf-frame/ros2_lf_binding/src/tf2_ingress.cpp`). It also
  refuses to start if tf2's executor does not hold both subscriptions before the
  detach, which the detach's wait-set probe relies on. A transform
  tf2 took before the detach finished, during node construction, has already reached
  the Buffer outside LF. Once the detach returns, tf2 takes nothing. If the group ever
  becomes runnable again, the binding logs a `tf_ingress_reattached` record, which
  fails the showcase check.
