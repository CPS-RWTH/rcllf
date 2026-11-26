# Troubleshooting

Common errors, known limitations, and workarounds for rcllf.

## First run

### `third_party/reactor-c is empty` (the container stops at start)

```text
!!> third_party/reactor-c is empty: the repository was cloned without
!!> its reactor-c submodule, which the LF runtime is built from.
!!> Fix it on the host, in the repository root, then start again:
!!>     git submodule update --init third_party/reactor-c
```

The clone did not fetch the Lingua Franca C runtime. Run the printed command on the host
(not in the container) and start the container again.

### `WARNING: ---- There are 1 unprocessed future events` at Ctrl-C

```text
WARNING: ---- There are 1 unprocessed future events on the event queue.
WARNING: ---- The first future event has timestamp (5250000000, 0) after start tag.
WARNING: ---- Cleared 1 unprocessed events from the event queue.
---- Elapsed logical time (in nsec): 5,000,000,000
---- Elapsed physical time (in nsec): 5,192,066,107
```

Normal. The Lingua Franca runtime reports at shutdown what was still scheduled, usually the
next timer firing, and how much logical and physical time passed. Nothing is wrong.

### `1 package had stderr output: lf_generated_...` after `colcon lf-build`

Normal. The Lingua Franca compiler (`lfc`) prints its progress (`Generating code for ...`,
`Code generation finished.`) on stderr, and colcon reports any stderr output. The build
succeeded if colcon says `Finished <<<` for the package.

The first container start can report packages with stderr output too, for example
`1 package had stderr output: ros2_lf_binding`: those are compiler warnings
(`-Wunused-but-set-variable`, `-Wmissing-field-initializers`). Which packages warn
depends on the compiler run; the build succeeded if the summary says
`packages finished` and no package `failed`.

### Node class does not accept NodeOptions

```text
Node class 'MyNode' does not accept NodeOptions; instance identity not realized
(instances share the constructor-defined node name and namespace). Add a
`const rclcpp::NodeOptions &` constructor parameter to fix.
```

The generated program creates each launch-file node itself and passes the node name,
namespace and remappings from the launch file through `rclcpp::NodeOptions`. A class
whose constructor takes no `NodeOptions` is still created, but with the name and
namespace hard-coded in its constructor, so two instances of it collide and launch-file
renames are lost. Give the constructor a `NodeOptions` parameter with a default:

```cpp
explicit MyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("my_node", options) { ... }
```

### Nodes from another container or machine show up

The dev container uses `ROS_DOMAIN_ID=0` unless you set it on the host. Other ROS 2
systems on the same network and domain see your topics. Pick a domain of your own:

```bash
ROS_DOMAIN_ID=37 docker compose run --rm dev
```

## lfc Path

### `colcon lf-analyze` fails with `Unresolved include` or a libclang error

```
  Unresolved include: node 'aeb_sim' (source .../src/aeb_mujoco_sim_node.cpp): 'mujoco/mujoco.h' not found, included from .../aeb_mujoco_sim_node.hpp:18
  Error: node 'aeb_sim' (source .../src/aeb_mujoco_sim_node.cpp) has unresolved include(s) 'mujoco/mujoco.h', so timers, publishers or subscriptions can be missing from the analysis. ...
```

**Cause:** The analyzer parses each node with libclang. When a header the node
includes cannot be found, clang stops instantiating templates, so
`create_publisher`, `create_subscription` and `create_wall_timer` calls drop out
of the parse. Which ones survive depends on where the include sits, and what is
left cannot tell what is missing. So any unresolved include in a node source
fails the analysis, even when some timers or endpoints are still visible.
When the missing header holds the node class itself (often the node's own
header), the error reads `no ROS2 node class found, because of unresolved
include(s)` and names the header.

Any other error-level libclang diagnostic fails the same way, for example an
unknown type from a missing `-D` flag:

```
  Error: node 'my_node' (source .../src/my_node.cpp): libclang reported error(s) 'unknown type name 'vendor_t'', so timers, publishers or subscriptions can be missing from the analysis. ...
```

**Fix:** Build the package with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` (or set it
in its `CMakeLists.txt`) so the analyzer uses the real compiler flags. Without
`compile_commands.json` the analyzer searches, in order, the package's own
`include/`, every `install/*/include`, `/opt/ros/<distro>/include`, and last
`build/<pkg>/include` and `build/<pkg>/*/include` (a vendored SDK unpacked by the
build). A header outside all of these needs the compile commands or an
`install(DIRECTORY ...)` into the package's include directory. For a libclang
error, fix the source or the flags it is compiled with. The compile commands
carry the package's `-D` definitions, the fallback does not.

---

### `colcon lf-analyze` fails with `Error: node '...'` for a launched node

```
  Error: node 'ghost' (package 'my_pkg', executable 'not_a_target'): the executable does not resolve to a source file in the package's CMakeLists.txt
```

**Cause:** Every node a launch file starts from a C++ package under `src/` must
become a node in `system.json`. The analyzer stops when it cannot get there: the
executable is not an `add_executable` target in the package's `CMakeLists.txt`,
the target is created inside a CMake function or macro of the package (the
analyzer reads `add_executable`, `add_library` and
`rclcpp_components_register_node(s)` calls written in `CMakeLists.txt` and does not
expand functions or macros), the source file is missing, the source holds no ROS 2
node class, or the
package declares an `ament_cmake` or `cmake` build type in its `package.xml` but
has no `CMakeLists.txt`. Earlier
versions skipped such a node with a log line colcon never printed, so it was
missing from the generated system without notice.

**Fix:** Correct the executable name in the launch file or the target in
`CMakeLists.txt`. For a target built by a CMake function, write the
`add_library` and `rclcpp_components_register_node` calls for that node directly in
`CMakeLists.txt`. The analyzer has no option to drop such a node on purpose.

A node the analyzer cannot analyze by design is reported, not failed:

```
  Warning: node 'rviz2' (package 'rviz2', executable 'rviz2') not analyzed: package 'rviz2' is not under .../src
  Warning: node 'py_node' (package 'py_pkg', executable 'py_node') not analyzed: package 'py_pkg' has build type 'ament_python', so it is not a C++ package
```

Such a node (a binary package from `/opt/ros`, or a package without CMakeLists.txt
whose `package.xml` declares a build type other than `ament_cmake` or `cmake`) is
not in `system.json`. If it should be, add the package source to `src/`.

---

### `colcon lf-generate` fails with `the lfc runtime cannot apply these arguments= entries`

The lfc runtime builds every node in one process from `rclcpp::NodeOptions` and
never runs the node's own `main()`. From a launch `Node(arguments=[...])` (or
`ros_arguments=`) it applies the `--ros-args` remap rules (`-r`/`--remap`,
including `__node`, `__name` and `__ns`) and `-p`/`--param` rules, in the order
`ros2 launch` passes them. The analyzer uses the same rules, so `system.json`
names the node as `ros2 node list` does. Anything else is refused at generation
rather than dropped: plain arguments for `main()`, `--params-file`, process-wide
flags such as `--log-level` or `--enclave`, a relative `__ns`, and a topic remap
targeted at one node (`node:topic:=x`).

**Fix:** move the setting into a field the lfc run applies: a parameter file
into `parameters=[...]`, a remap into `remappings=`, and a name or namespace
into `name=` / `namespace=` or an untargeted `-r` rule.

### `colcon lf-generate` or `colcon build` fails with `lfc runtime ... is not the patched runtime`

lfc compiles every generated program against the reactor-c copy inside its
`lib/core-<version>.jar`, not against `third_party/reactor-c`. The image build
applies `third_party/patches/reactor-c/` to that jar, and each fix patch puts a
marker string into the compiled program; the run gate (a read-back of the markers)
marks a run INVALID when a marker is missing. An image built before a patch
existed produces programs whose every run ends
`read-back invalid: reactor_c_patches: runtime built without the configured
patches (missing rcllf-reactor-c-patch:...)` although the outputs are right
(for example a local `:dev` tag built before a patch landed).
`colcon lf-generate` therefore checks the lfc install (`$LFC`, else
`/opt/lfc/bin/lfc`) with `patch_lfc_runtime.py --check` and refuses to
generate from an unpatched runtime. The generated package's `CMakeLists.txt`
runs the same check before it invokes lfc, so rebuilding an already generated
package in a stale image (`colcon build`, `colcon lf-build`) fails the same way
instead of producing a program whose runs are all INVALID. `colcon lf-build`
runs the check itself as well. A package generated before this check existed
carries no CMake check: regenerate it (`colcon lf-generate`) or build it with
`colcon lf-build`, not plain `colcon build`. `$LFC` is an absolute path or a
bare program name found through a non-empty `PATH` whose entries are all
absolute; a relative path with a directory part (`LFC=./bin/lfc`), or a bare
name with an empty `PATH` or a relative `PATH` entry, is refused by both
checks, because the check and the lfc invocation run from different
directories and could resolve it to two different executables. Any other
value is a program name, `OFF` included. The generated `CMakeLists.txt`
invokes the executable the check resolved. Without an lfc executable at that
location nothing is checked (the check says so, with or without a patch
catalog) and the lfc invocation itself reports the missing program.

**Fix:** rebuild or refresh the image: `docker compose build dev`, or pull the
release image `ghcr.io/cps-rwth/rcllf:<version>-deps` and re-tag it as
`ghcr.io/cps-rwth/rcllf:dev` (see [Docker](docker.md#pulling-pre-built-images)). `docker compose run` uses the
local tag as it is and never rebuilds it when `third_party/patches` changes.

### `SYSTEM_JSON not set` (process exits immediately)

```
[FATAL] [lf_component_manager]: SYSTEM_JSON not set and --system-json not provided.
        The lfc runtime cannot load any components without a system.json path.
```

**Cause:** The lfc-path binary requires a `system.json` to know which component
nodes to load. Without it, no nodes are constructed, no callbacks run, and any
LTTng trace will contain essentially zero `callback_start` events.

**Fix:** Pass the path via the environment variable or command-line argument:

```bash
# Environment variable (used by Docker services)
SYSTEM_JSON=/workspace/path/to/system.json ./<binary>

# Command-line argument
./<binary> --system-json /workspace/path/to/system.json
```

If you use `docker compose run`, check that the service definition sets
`SYSTEM_JSON`.

---

### `lfc failed with exit code No such file or directory`

```
CMake Error at CMakeLists.txt:32 (message):
  lfc failed with exit code No such file or directory
```

**Cause:** `lfc` is not on `PATH`. The `dev` Docker image ships `lfc`
at `/opt/lfc/bin/lfc`. If you're running outside Docker make sure that
path is on `PATH`.

**Fix:** Use the correct Docker service:

```bash
docker compose --profile lf run --rm lf-build             # pubsub_example (the default)
```

If building manually, enter the dev container first:

```bash
docker compose run --rm dev
# Now lfc is on PATH
lfc --version
```

---

### `undefined reference to _lf_get_environments`

**Cause:** You are trying to build `ros2_lf_binding` or a test binary without
providing a definition of `_lf_get_environments`. This symbol is generated by
`lfc` into the final application binary. It is intentionally absent from the
shared library.

**For the library itself:** the `CMakeLists.txt` already adds
`-Wl,--allow-shlib-undefined`. If you see this error building the library,
check that the `target_link_options` line is present.

**For a test binary:** each test that touches time-source code must provide a
stub:

```cpp
#include "core/environment.h"
static environment_t g_stub_env;
int _lf_get_environments(environment_t ** envs) { *envs = &g_stub_env; return 1; }
```

**For the application binary:** this means the generated `ros2_lf_reaction_ids_init.cpp`
was not compiled into the target. Check the generated `CMakeLists.txt`.

---

### `ros2_lf_bind_node: ... Refusing to start.`

```
[FATAL] [ros2_lf_binding]: ros2_lf_bind_node: <reason>. Refusing to start.
```

The binding matches every timer, subscription, service and action handle a node
creates to a reaction ID from `system.json`, and stops the program when a handle
does not fit. The `<reason>` names the handle and the node. The common ones:

- **`... has no reaction ID. The analyzer did not extract it ...`**: the node created
  a handle the analyzer did not see, usually a callback shared by two timers or
  subscriptions, or a handle created outside the constructor. Give each handle its own
  callback and create it in the constructor, then run `colcon lf-analyze` again.
- **`... bound twice: to ... and to ...`**: two reaction IDs fit the same handle.
  Check that `system.json` matches the current source (re-run `colcon lf-analyze`
  after changing the node).
- **`... has no reaction ID and sits in ... outside the LF schedule`**: a library
  thread (such as `tf2_ros::TransformListener` with its own executor) owns the
  handle. Acknowledge RCLLF-R002 in `execution.yaml` to run it as unsupported, or
  create it without its own executor.

---

### Timer fires but `Node::now()` returns wall time

**Cause:** The clock being read is not one the binding drives. The binding
drives the node clock of every node it constructs (see
[ADR 0001](../design/adr/0001-lf-owns-time.md), amendment 2026-10-09). It does
not drive an `rclcpp::Clock` the node creates itself, a node created outside
the binding (for example in `main()`), or steady and system clocks.

A component node that has no TimeSource `/clock` subscription logs
`has no TimeSource /clock subscription` at load time and is unloaded. A
non-component node whose parameters set `use_sim_time=true` is taken off
`/clock` at registration and logs that it now follows LF logical time. If that
fails, registration throws an error naming the node.

**Fix:** Read time through `this->now()` or `this->get_clock()`, not a
separately constructed `rclcpp::Clock`. Do not override `use_sim_time` on
nodes that run under LF. Outside a running LF program `now()` returns 0 until
the first tag.

---

### A subscriber callback sees stale data, or none

**Symptom:** A subscriber callback runs with data older than what the upstream
publisher just produced, or does not run at all on a tick where the publisher
clearly published.

**Not the cause: reaction declaration order.** An earlier version of this page
blamed the generator emitting a subscription reaction ahead of its upstream
timer reaction. That cannot happen. LF orders the reactions at a tag by the
reaction *dependency graph*, and a connection is a precedence edge from the
upstream reaction to the downstream one. Declaration order only breaks ties
among reactions the graph leaves unordered inside a single reactor. The tag does
not explain it either. An admitted d = 0 edge inside one process runs the
subscription at the publication tag (t, m), and an `after 0` link at
(t, m+1), both after the publishing reaction.

**The actual cause: the LF port carries no payload.** Look at a generated
connection and the reaction that feeds it:

```lf
reaction(t_timer_callback) -> sensor_data {=
    ros2_lf_invoke_timer(self, RID_PublisherNode_timer_callback);
    lf_set(sensor_data, 0);
=}
...
publisher_node.sensor_data -> subscriber_node.sensor_data after 0
```

The port type is `stub_t` and the value is a literal `0`. The LF port is a pure
trigger. Your real message travels over DDS, published inside the ROS callback
that `ros2_lf_invoke_timer` calls, and the subscriber's reaction *takes* it from
the live DDS subscription when it fires.

So LF guarantees the order in which your callbacks are invoked, and nothing
about whether the message has arrived by then. That is a transport question,
and the `after` delay is exactly the physical-time slack the take is granted
before it runs. `after 0` grants none. When the take finds nothing, the
[take policy](../features/execution-config.md) decides what happens. A trigger
with a known write count polls up to `maxwait_ms` in `skip` (the default) and
`block` mode alike. Only an unknown write count (`RCLLF_COUNT_PUBLISHES=0`, a publisher
outside any reaction) skips at once in `skip` mode. An admitted correspondence edge
waits up to its generated timeout instead.
A sample that misses its take is never delivered later. The
binding keeps, per topic, a record of every write made inside a reaction
(the publisher's GID and the sample's sequence number), in publish order. A
trigger that counts k writes owns the next k records, and its take delivers
the samples of those records. A sample whose record belongs to an earlier
trigger missed its take: it is dropped as soon as it arrives and reported
(`late_dropped` in the take statistics, and the subscription's
`message_lost` handler if it has one). A lost sample is simply never
delivered and costs no other sample its tag. A take whose trigger counted a
write waits for that sample up to `maxwait_ms` of wall time before it
declares the miss, in `skip` mode as in `block` mode (the tag
can be granted while the sample is still in the DDS receive path); the wait
is counted as `waited` in the take statistics. `block` mode also waits when
the trigger's write count is unknown.

A write announced with `ros2_lf_claim_publish` inside a reaction reserves a
record at that tag; the next publish on the topic made outside any reaction
fills the newest open claim. A claim that is never filled (a lost sample)
stays open. The binding keeps the last 4096 records per topic. It still
knows a sample of an older, forgotten record (or one that fills a forgotten
claim) is from an earlier trigger, by the writer's highest forgotten sequence
number. Such a sample is dropped and reported, never delivered late. An
unclaimed publish that a writer made before such a sample, and that has not
been taken yet, is dropped with it. A publish
outside any reaction with no open claim has no record:
it is *untagged* and delivered at the next take, in addition to that tag's
own samples (`untagged` in the take statistics, one WARN line).

Cases and what you see. None of them drops a sample without counting it in
`late_dropped`:

| Case | What the binding does | What you see |
|---|---|---|
| A sample arrives within `maxwait_ms` of its take | The take waits for it and delivers it at its own tag. | `sub_waited`, `waited` |
| A sample arrives after its take's timeout | The take skips; the sample is dropped when it arrives at a later trigger and reported. | `sub_skipped`, `late_dropped`, WARNs `no sample for a trigger` and `dropped a late sample` once |
| A late sample alone at the next take | Dropped, never delivered one tag late; that take skips too. | as above |
| Several publishers on one take port | Only the late publisher's sample is dropped; the others are delivered at their own tags. | `late_dropped` |
| The sample is lost in transit | Its take skips; nothing else is dropped or delayed. | `sub_skipped` |
| A sample published outside any reaction and not claimed | Delivered at the next take in addition to the tag's own samples. | `untagged`, WARN `published outside any reaction` once |
| A publisher whose QoS does not match | It delivers no samples, so it changes nothing. | nothing |

**Across federates** the same rule applies (second part). The
generated link between two federates carries the write count together with
the writer federate's record events for the topic since that link's last
message (the newest 32), and the receiving federate replays them into its own
records before the take. A late sample is then dropped and reported exactly
once, as in one process. A sample that arrives before the events of its
trigger (a claimed sample published late, whose fill comes with the writer's
next trigger) is held until they arrive, then dropped or delivered by its
record. Every link message also names the writer federate's publishers of
the topic (their GIDs), so the receiver tells writers apart by identity, not
by waiting: the first sample of a named publisher whose events have not
arrived yet is held for them, however many triggers that takes. A crossing
topic has exactly one writer federate, and the receiving federate knows from
the generator how many publishers of the topic that federate has (the
analysis count). The generator uses that count only where the analysis
proves it exact: every publisher of the writer node has a static name,
resolved with the node's namespace and remaps as the topic was, and every
publisher of the topic is created in the constructor on every path through
it (a lifecycle node's publisher made at configure, or one made only when a
parameter is set, may never exist). Otherwise the link keeps the count rule (see the
warning below). A sample of a publisher
the writer federate does not name, once it has named that many, is from a
writer outside rcllf (a plain ROS node) and is delivered as an untagged extra
at the first take (WARN `a writer outside rcllf`). Until then it waits, so a
publisher the writer federate creates after its earlier messages (a
lifecycle publisher made at configure) is held for its own events, never
taken for a plain one. A publisher the analysis did not see is still named
only from its next message on: a sample of it taken before goes out
untagged, and that message logs WARN `announced a publisher created after
its earlier messages`. The take of a crossing topic runs only after its
link's message was imported (the import triggers the subscription); a
hand-written reaction that takes earlier logs WARN `taken before the
cross-federate take link's first message` once, holds the samples until an
import, and drops that trigger's own as late. A sample still held at
shutdown is counted as `unsettled=` in the `ros2_lf_take_stats` line. Several publisher
nodes of one federate each leave on their own link,
and every link carries the events since its own last message, so the receiver
gets them all whichever link it reads first. The generator refuses publisher
nodes of one topic in two different federates and names the topic and the
publishers (`cross-federate take link on topic ...`): their events are
numbered apart, so the receiver could not tell which sample belongs to which
tag. Publish the topic from nodes of one federate.

!!! warning "When a cross-federate link falls back to the count rule"
    The subscription uses the earlier *count rule* when the link lost events:
    more than 32 events since its last message that no other link of the
    topic brought, or events the writer forgot before sending (WARN `the writer's tag log events did not all arrive` naming the
    edge), and when the writer process restarted (WARN `the writer's tag
    log session changed` naming the edge), and when the writer federate has
    more than 16 publishers of the topic, too many to name in one message
    (WARN `more than 16 publishers`). It also does so for a program generated with
    `RCLLF_COUNT_PUBLISHES=0`, whose links carry only a callback count, and
    for a topic whose writer publisher count the analysis cannot prove exact
    (a publisher with a dynamic name, one created outside the constructor
    such as a lifecycle publisher made at configure, one whose create call
    sits under an `if`, `switch`, loop, `?:`, `&&`/`||`, lambda or `try`
    block or after a `return` in the constructor (`conditional creation`),
    one whose creation site the analysis does not record (`creation site
    unknown`), or a node without a launch namespace whose names match the
    topic ambiguously). The receiving
    federate then warns at startup once per edge, with the reason
    (`cross-federate take link <edge> on topic '<topic>' carries only a count
    (publisher count not exact: ...)`, or `(generated with
    RCLLF_COUNT_PUBLISHES=0)`).
    Under the count rule a skipped take owes one sample, and a later take
    that holds more samples than the writes still expected at its tag drops
    the oldest excess and reports it, but a late sample alone at the next take
    is delivered one tag late (WARN `cannot be told from an on-time one`).
    The subscription logs `the count rule applies` once for the topic. The
    same fallback applies to a write the binding could not read a GID for
    (that trigger only). A sample taken early and held for a trigger that
    falls back to the count rule is delivered first at that trigger.

To know how many samples a trigger owes, the generated reactions count the
publishes the binding sees inside a callback, not the callback runs. A node
that publishes from a thread of its own is not seen, so no downstream
trigger is counted for that sample; it is untagged and delivered at the next
take. Announce such a publish with `ros2_lf_claim_publish` inside the
callback, or regenerate the system with `RCLLF_COUNT_PUBLISHES=0` in the
environment of `colcon lf-generate`: each callback run then counts one write,
drop by tag is off, and a skipped take owes nothing.

**Fix, in order of preference:**

1. **Give the take more slack.** Raise the after-delay so the declared
   logical-time offset exceeds your transport's delivery time. `colcon
   lf-generate` emits a zero delay (no slack) by default. Set a per-link
   `after_delay_ns` in `system.json`, pass `--after-delay`, or set
   `defaults.after_delay` in `execution.yaml` (see
   [Execution Configuration](../features/execution-config.md#the-after-delay-default)).
   Precedence is the per-link value, then the flag, then `execution.yaml`.
   Check what actually got emitted in
   `lf-analysis/<pkg>/<stem>/lf/<Stem>.lf`.

2. **Make the miss visible before tuning anything.** The runtime already counts
   them. Look for `ros2_lf_take_stats` in the log:

    ```
    ros2_lf_take_stats: reason=periodic mode=skip timeout_ms=100
      sub_first_try=19 sub_skipped=1 ...
    ros2_lf_take_stats_topic: topic=sensor_data skipped=1 ...
    ```

    `sub_skipped` is the number of takes that found nothing. One skip at
    startup is normal (the first tick can precede discovery). A skip rate that
    tracks your publish rate is the real symptom. `late_dropped` (printed
    only when non-zero) counts samples that came after their take and were
    dropped; `untagged` (printed only when non-zero) counts samples published
    outside any reaction and delivered in addition.

    `reader_lost` (printed only when non-zero) counts samples a KeepLast
    reader replaced before a take, or a best-effort reader lost on the wire:
    a writer that publishes more samples at one tag than the reader's depth
    keeps only the newest ones. They are reported once, at the take that
    finds the gap in the writer's publication numbers (one WARN
    `... never reached a take ...` per reader, `message_lost` with the
    count), and the takes of the replaced samples end at once: they are
    counted in `sub_skipped`, but poll nothing and raise no miss WARN.
    `poll_timeouts` therefore stays 0 for them; a non-zero
    `poll_timeouts` names takes that waited the whole `maxwait_ms` for a
    sample that never came. Raise the reader's depth to the burst size to
    keep every sample.

3. **Tolerate jitter with `maxwait_ms`.** A take whose trigger counted
   a write already waits up to the subscription's `maxwait_ms` (100 ms
   by default); raise it per subscription in `execution.yaml`, below the
   connection's after-delay, when the transport's jitter exceeds it.
   `take_mode: block` extends the wait to takes whose write count is
   unknown. Both delay the skip decision; neither fabricates or
   re-delivers data. Read the
   [block-timing footgun](../features/execution-config.md#the-block-timing-footgun)
   first, because a per-subscription block can stall other reactions at the
   same tag.

!!! note "Measured on `pubsub_example`"
    Generated at `after 0` and at `after 1 msec` and run for 20 s each
    (38 publishes), the subscriber received every message, in order, with no
    staleness in either case, and both runs recorded the same single
    startup skip. The publish-to-callback gap was 0.09 to 0.33 ms at `after 0`
    and 0.78 to 1.01 ms at `after 1 msec`.

    So `after 0` did not bite here. Both endpoints live in one process on an
    idle machine, which is the fastest case DDS delivery ever gets, and the
    margin at `after 0` was the whole of it. Read this as "the hazard is real
    and `after 0` leaves no room for it", not as "`after 0` is fine".

---

### Action server aborts on Ctrl-C with `goal that does not exist`

```text
terminate called after throwing an instance of 'std::runtime_error'
  what():  Asked to publish result for goal that does not exist
```

The server's goal thread outlived the rclcpp context. A Ctrl-C shuts the
context down first, while the node and its action server stay alive until
`ros2_lf_runtime_shutdown()`. A goal handle that is then destroyed while still
active publishes a cancel result, which throws inside its destructor. Stock
ROS 2 shows the same abort. Close the window in the node itself: keep the goal
threads, stop and join them in a pre-shutdown callback, and abort the running
goal there. See [action_example](../examples/action-example.md#shutdown-during-a-goal).
Related messages from the same cause are
`Failed to publish feedback: feedback publisher is invalid` and
`context cannot be slept with because it's invalid`.

---

## Tracing

### Trace has only ~8 events (callback_start count)

This is the symptom of the `SYSTEM_JSON not set` failure described above. Zero
components loaded means zero callbacks fired. Check the container logs for startup
errors before assuming the trace itself is broken.

---

### `colcon test` reports 0 tests

The main `colcon build` does not enable `BUILD_TESTING`. To run the
`ros2_lf_binding` unit tests:

```bash
docker compose --profile lf run --rm test-binding
```

---

## Known Limitations

### Generation refused with RCLLF-H009 (blocking service client)

A service client that waits for its response inside the callback
(`spin_until_future_complete`, `future.wait_for`, `future.get`, or a loop on `spin_some`,
`spin_once`, a condition variable or a flag that the response callback sets after
`async_send_request`) is refused at generation.
Under rcllf the response is a later LF event, so the wait timed out on every
call. Use `async_send_request` with a response callback, which is fully supported, see
[Architecture](../design/architecture.md#communication-primitives). Listing RCLLF-H009 in
`execution.yaml`'s `acknowledged_hazards:` generates the program anyway, with the timeouts.

---

### Generation refused with RCLLF-H011 (action server sending from its own thread)

An action server that publishes feedback or sends a result from a thread it starts
(an `execute()` thread from `handle_accepted`, or a thread started in the constructor)
is refused at generation. The tag at which such a send reaches the client
depends on thread scheduling, and while LF runs the binding refuses the send anyway
(`rcllf: refused` in the log, a `refused` trace line). Queue the goal in
`handle_accepted` and advance it from a timer callback, as
[action_example](../examples/action-example.md) does. Listing RCLLF-H011 in
`execution.yaml`'s `acknowledged_hazards:` generates the program, but the thread's sends
are still refused at runtime.

---

### Generation refused with RCLLF-H012 (request from a service handler or response callback)

A service handler or a client's response callback that sends a request
(`async_send_request`) or an action goal is refused at generation. Under rcllf a
request is carried by the port of the timer or subscription reaction that sends it; a
send from a handler or a response callback has no port, so its server would take it in
place of a later in-system request. Send it from a timer or subscription callback (a
request from a subscription callback is supported). If it is acknowledged in
`execution.yaml`, the binding drops the send's count at runtime with a warning
naming RCLLF-H012, and its response callback does not run.

---

### Run refused with RCLLF-R004 (second server of an in-system service)

`RCLLF-R004: service '...' has N servers on the graph but M in-system ... Refusing to
run.` A server outside the system (another process, a second launch of the same
program) serves a service that an in-system server serves, so it would answer
in-system requests too. The binding checks this when it binds the server and again
at most once a second while serving. Stop the other server or rename the service.
Outside *clients* are fine: their requests are served at once and counted in the
`external_requests` field of the `ros2_lf_service_stats` line at shutdown.

---

### Generation refused with RCLLF-H017 (intra-process communication)

A composable node whose launch file sets `extra_arguments=[{'use_intra_process_comms': True}]`,
or a node whose source builds its options with `.use_intra_process_comms(true)`, is refused at
analysis. rclcpp then hands a message to a subscription in the same process without the
middleware, so the message has no publisher identity for rcllf to order, and the binding
refuses to start a publication edge of such a node (`intra_process` startup failure).
Remove the option, or set it to `False`; the default is off. Messages between nodes of one
process still reach each other through the middleware. Listing RCLLF-H017 in
`execution.yaml`'s `acknowledged_hazards:` generates the program anyway.

---

### Generation refused with RCLLF-H013 (endpoint created outside the constructor)

A subscription, service, client or publisher created in an ordinary method (for
example inside a timer callback) is never bound: the startup pass runs once, so its
callback never runs under rcllf. Create the endpoint in the constructor. Listing
RCLLF-H013 in `execution.yaml`'s `acknowledged_hazards:` generates the program, and
the late endpoint then delivers nothing.

---

### Generation refused with RCLLF-H014 (work in main() outside callbacks)

rcllf replaces `main()`, so a `spin_some` loop that calls a publishing node method, or
a publish written directly in `main()`, never runs. Move the work into a timer callback.

---

### Generation refused with RCLLF-H015 (endpoints in a template base class)

A node whose base class is a template that creates endpoints in its constructor is
refused: the analyzer cannot walk a template constructor, so the endpoints would be
missing silently. Use a non-template base class in your workspace (those are walked) or
create the endpoints in the derived class.

---

### Generation refused with RCLLF-H016 (ambiguous compile variant)

One source file is built into several executables with different compile definitions
(an `#ifdef` selecting `main()` or the node class) and rcllf cannot tell which variant
belongs to this executable. Declare the selecting definition with
`target_compile_definitions(<exe> PRIVATE NAME)` on that executable's target.

---

### All timers must be created at node construction time

`ros2_lf_bind_node` walks callback groups once during the `startup` reaction and
binds every timer it finds to an LF timer. A user `cancel()` or `reset()` on a bound timer
is honoured. Timers created after that point, e.g. in a timer
callback or from a service response, will not be registered with reactor-c and
will never fire.

---

### `steady_clock::now()` returns wall time

`RCL_STEADY_TIME` and `std::chrono::steady_clock` are not intercepted. Callbacks
may use them for diagnostics (latency measurement, logging), but must not use
them to drive scheduling decisions (see [ADR 0001](../design/adr/0001-lf-owns-time.md)).

---

### DDS deadline QoS is ignored

ROS 2 deadline QoS callbacks (`set_on_new_message_callback` with deadline
semantics) are wall-clock-based. Under LF, where logical time advances in
discrete steps, these callbacks either never fire or fire at incorrect logical
times. Do not use deadline QoS on topics that are part of the LF-controlled
graph.

---

### `rclcpp_executor_get_next_ready` events in lfc traces

These events appear in lfc traces because `LFComponentManager` (itself a node)
is added to a `SingleThreadedExecutor`. The executor does minimal work (node
registration only) but still emits executor tracepoints. They do not affect
correctness. The analyzer correctly attributes callbacks to their nodes via the
`rclcpp_callback_register` events.

---

### Launch parser: "launch configuration does not exist" warnings

Some launch files are building blocks meant to be included by a parent launch
file, which supplies required arguments via `launch_arguments=`. Analyzing such
a file standalone fails with:

```
OpaqueFunction skipped: launch configuration 'sensor_model' does not exist
```

This is expected for vehicle/sensor-specific configs and undeclared arguments
that only exist once a parent launch file sets them. It is not a parser bug.

**Fix:** Analyze the top-level launch file instead (the one that includes the
building-block file with its arguments). If you must analyze the building-block
file directly, expect this warning. The produced `launch_graph.json` is still
valid, just missing the parent-supplied context.
