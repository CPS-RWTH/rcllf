# Troubleshooting

Common errors, known limitations, and workarounds for rcllf.

## Find your symptom

| What you see | Section |
|---|---|
| Container stops at start, `reactor-c is empty` | [First run](#third_partyreactor-c-is-empty-the-container-stops-at-start) |
| `unprocessed future events` at Ctrl-C, or `stderr output` after a build | [First run](#first-run) |
| `colcon lf-analyze` fails: unresolved include, or `Error: node '...'` | [lfc Path](#colcon-lf-analyze-fails-with-unresolved-include-or-a-libclang-error) |
| `colcon lf-generate` or the build refuses: `lfc runtime`, `lfc failed`, `undefined reference` | [lfc Path](#lfc-path) |
| Process exits at start: `SYSTEM_JSON not set`, `Refusing to start` | [lfc Path](#system_json-not-set-process-exits-immediately) |
| `Node::now()` shows wall time; a callback sees stale or no data | [lfc Path](#timer-fires-but-nodenow-returns-wall-time) |
| `RCLLF-H0nn` or `RCLLF-R00n` in any message | [Hazard codes](#hazard-codes) and [Diagnostic Codes](diagnostic-codes.md) |
| A trace with only a few events, or `colcon test` finds 0 tests | [Tracing](#tracing) |
| A timer made late, `steady_clock::now()`, DDS deadline QoS | [Known Limitations](#known-limitations) |

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
[WARN] [ros2_lf_binding]: Node class 'MyNode' does not accept NodeOptions; instance identity not realized
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
  Error: node 'aeb_sim' (source .../src/aeb_mujoco_sim_node.cpp) has unresolved include(s) 'mujoco/mujoco.h', so timers, publishers or subscriptions can be missing from the analysis. Make the headers reachable (build the package with -DCMAKE_EXPORT_COMPILE_COMMANDS=ON, or install them under install/<pkg>/include) and re-run colcon lf-analyze.
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
[FATAL] [lf_component_manager]: SYSTEM_JSON not set and --system-json not provided. The lfc runtime cannot load any components without a system.json path. Pass it via the SYSTEM_JSON env var or --system-json <path>.
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

A generated launch file sets `SYSTEM_JSON` for you, so this error shows up when you run the
binary by hand, or start it from your own launch file. No Compose service sets it. For a
manual run, pass it with `-e`:

```bash
docker compose run --rm -e SYSTEM_JSON=/workspace/packages/lf-analysis/<package>/<system>/system.json dev ./<binary>
```

---

### `lfc failed with exit code No such file or directory`

```
CMake Error at CMakeLists.txt:<line> (message):
  lfc failed with exit code No such file or directory
```

The message comes from the `CMakeLists.txt` of the generated package, which runs `lfc` with
`execute_process` and prints its result. The line number varies. The text `No such file or
directory` is what CMake reports when it cannot run the `lfc` file at all.

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

**Cause:** The clock being read is not one the binding drives. The binding drives
the node clock of every node it constructs (see
[ADR 0001](../design/adr/0001-lf-owns-time.md)). It does not drive an
`rclcpp::Clock` the node creates itself, a node created outside the binding (for
example in `main()`), or steady and system clocks.

Two related messages from `ros2_lf_binding`:

- `Node '<name>' has no TimeSource /clock subscription; its now() stays on physical
  time instead of LF logical time`: the node has no `use_sim_time` time source to feed.
- `Node '<name>': setting use_sim_time=false was rejected: ...`: a non-component node
  whose parameters set `use_sim_time=true` is taken off `/clock` at registration, and
  that step failed.

**Fix:** Read time through `this->now()` or `this->get_clock()`, not a
separately constructed `rclcpp::Clock`. Do not override `use_sim_time` on
nodes that run under LF.

---

### A subscriber callback sees stale data, or none

**Symptom:** A subscriber callback runs with data older than what the upstream
publisher just produced, or does not run on a tick where the publisher clearly
published.

**Applies to** topic edges that are not admitted for
[publication correspondence](../features/correspondence.md#when-it-applies). On an
admitted edge every run gets the sample its publish produced. The startup log and
`correspondence.json` in the generated package list which edges stayed on the older
path and why.

**Cause:** On such an edge the LF port carries no message. It carries a count of the
publishes the binding saw in the publishing reaction. For `take_skip_fixture` the
generated reactor contains:

```lf
reaction(t_timer_callback) rcllf_bound -> take {=
    ros2_lf_invoke_timer(self, RID_TakeWriterNode_timer_callback);
    { int n = ros2_lf_topic_sent(RID_TakeWriterNode_timer_callback, "take"); if (n > 0) { ... lf_set(take, rcllf_c); } }
=}
...
take_writer.take -> take_reader.take after 0
```

The message itself travels over DDS, and the subscriber's reaction takes it from the
live DDS subscription when it fires. LF fixes the order of your callbacks, not whether
the message has arrived. Reaction declaration order is not the cause: LF orders
reactions by the dependency graph, and the connection puts the publishing reaction
first. The `after` delay is the physical-time slack the take is given before it runs;
`after 0`, the default, gives none.

When the take finds nothing, the [take policy](../features/execution-config.md#take_mode-and-maxwait_ms)
decides. A take whose trigger counted a write waits up to `maxwait_ms` (100 ms by
default) in `skip` and `block` mode alike. A take with an unknown write count
(`RCLLF_COUNT_PUBLISHES=0`, or a publisher outside any reaction) skips at once in `skip`
mode and waits in `block` mode. A sample that misses its take is never delivered later:
it is dropped when it arrives and counted in `late_dropped`.

| Case | What the binding does | What you see |
|---|---|---|
| A sample arrives within `maxwait_ms` of its take | The take waits and delivers it at its own tag. | `sub_waited`, `waited` |
| A sample arrives after its take's timeout | The take skips; the late sample is dropped when it arrives. | `sub_skipped`, `late_dropped`, WARNs `no sample for a trigger` and `dropped a late sample` once |
| The sample is lost in transit | Its take skips; nothing else is dropped or delayed. | `sub_skipped` |
| A sample published outside any reaction and not claimed | Delivered at the next take in addition to that tag's own samples. | `untagged`, WARN `published outside any reaction` once |

To announce a publish made from a thread of your own, call `ros2_lf_claim_publish`
inside the callback, or generate with `RCLLF_COUNT_PUBLISHES=0` (then each callback
run counts one write and drop by tag is off). The binding keeps the last 4096 records
per topic.

**Across federates** the same rule applies. The generated link between two federates
carries the write count together with the writer's record events for the topic (the
newest 32 since the link's last message), and the receiver replays them before the
take, so a late sample is dropped and reported exactly once. A topic must have all its
publisher nodes in one federate: the generator refuses publisher nodes of one topic in
two federates and names the topic and the publishers (`cross-federate take link on
topic ...`). A link falls back to the earlier count rule, with a startup WARN naming the
edge, when it lost events (`the writer's tag log events did not all arrive`), when the
writer process restarted (`the writer's tag log session changed`), when the writer has
more than 16 publishers of the topic, when the program was generated with
`RCLLF_COUNT_PUBLISHES=0`, or when the analysis cannot prove the writer's publisher
count (a dynamic topic name, a publisher created outside the constructor or under a
condition). Under the count rule a late sample alone at the next take is delivered one
tag late (WARN `cannot be told from an on-time one`). A sample still held at shutdown
is counted as `unsettled=` in the `ros2_lf_take_stats` line.

**Fix, in order of preference:**

1. **Give the take more slack.** Raise the declared logical-time offset above your
   transport's delivery time. Per link: `after_delay_ns` in `system.json`; for all
   links: `colcon lf-generate --after-delay "5 msec"` or `defaults.after_delay` in
   `execution.yaml` (see
   [Execution Configuration](../features/execution-config.md#the-after-delay-default)).
   Precedence is the per-link value, then the flag, then `execution.yaml`. Check what
   was emitted: `grep " after " lf-analysis/<pkg>/<stem>/lf/<Stem>.lf` printed
   `take_writer.take -> take_reader.take after 5 msec` with the flag and `after 0`
   without it.
2. **Make the miss visible.** The runtime prints `ros2_lf_take_stats` periodically and
   at shutdown:

    ```text
    ros2_lf_take_stats: reason=shutdown mode=per-sub timeout_ms=per-sub sub_first_try=35 sub_skipped=0 sub_waited=0 wait_total_us=0 wait_max_us=0 wait_timeouts=0
    ```

    `sub_skipped` counts takes that found nothing, `sub_waited` takes that waited and
    got the sample. One skip at startup is normal (the first tick can precede
    discovery); a skip rate that tracks your publish rate is the real symptom.
    `late_dropped`, `untagged` and `reader_lost` are printed only when non-zero.
    `reader_lost` counts samples a KeepLast reader replaced before a take or a
    best-effort reader lost on the wire: raise the reader's depth to the burst size.
    A non-zero `poll_timeouts` names takes that waited the whole `maxwait_ms` for a
    sample that never came.
3. **Tolerate jitter with `maxwait_ms`.** Raise it per subscription in `execution.yaml`,
   below the connection's after-delay. `take_mode: block` extends the wait to takes
   whose write count is unknown. Both delay the skip decision; neither invents or
   re-delivers data. Read the
   [block-timing footgun](../features/execution-config.md#the-block-timing-footgun)
   first.

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

## Hazard codes

A code that starts with `RCLLF-` is a hazard diagnostic. The [Diagnostic Codes](diagnostic-codes.md) page lists every code with its meaning, fix and whether it can be acknowledged. The common refusals, in short:

### Generation refused with RCLLF-H009 (blocking service client)

A service client waits for its response inside the callback. Use `async_send_request` with a response callback. Details: [RCLLF-H009](diagnostic-codes.md#rcllf-h009).

---

### Generation refused with RCLLF-H011 (action server sending from its own thread)

An action server sends feedback or a result from a thread it starts. Queue the goal in `handle_accepted` and advance it from a timer callback, as [action_example](../examples/action-example.md) does. Details: [RCLLF-H011](diagnostic-codes.md#rcllf-h011).

---

### Generation refused with RCLLF-H012 (request from a service handler or response callback)

A service handler or response callback sends a request or action goal. Send it from a timer or subscription callback. Details: [RCLLF-H012](diagnostic-codes.md#rcllf-h012).

---

### Run refused with RCLLF-R004 (second server of an in-system service)

Another server outside the system serves a service an in-system server also serves. Stop the other server or rename the service. Details: [RCLLF-R004](diagnostic-codes.md#rcllf-r004).

---

### Generation refused with RCLLF-H017 (intra-process communication)

A node turns on `use_intra_process_comms`. Remove the option or set it to `False`. Details: [RCLLF-H017](diagnostic-codes.md#rcllf-h017).

---

### Generation refused with RCLLF-H013 (endpoint created outside the constructor)

A subscription, service, client or publisher is created outside the constructor. Create it in the constructor. Details: [RCLLF-H013](diagnostic-codes.md#rcllf-h013).

---

### Generation refused with RCLLF-H014 (work in main() outside callbacks)

`main()` runs node work, which rcllf never runs. Move the work into a timer callback. Details: [RCLLF-H014](diagnostic-codes.md#rcllf-h014).

---

### Generation refused with RCLLF-H015 (endpoints in a template base class)

The node's base class is a template that creates endpoints. Use a non-template base or create the endpoints in the derived class. Details: [RCLLF-H015](diagnostic-codes.md#rcllf-h015).

---

### Generation refused with RCLLF-H016 (ambiguous compile variant)

One source is built into several executables with different `#ifdef` definitions. Declare the selecting definition with `target_compile_definitions(<exe> PRIVATE NAME)`. Details: [RCLLF-H016](diagnostic-codes.md#rcllf-h016).

---

## Tracing

### Trace has only ~8 events (callback_start count)

A trace with almost no `callback_start` events usually comes from a run that loaded no
components, for example the `SYSTEM_JSON not set` failure described above. Check the
container logs for startup errors before assuming the trace itself is broken.

---

### `colcon test` reports 0 tests

The main `colcon build` does not enable `BUILD_TESTING`. To run the
`ros2_lf_binding` unit tests:

```bash
docker compose --profile lf run --rm test-binding
```

---

## Known Limitations

### All timers must be created at node construction time

`ros2_lf_bind_node` (`node_binding.cpp`) walks the node's callback groups once, in the
`startup` reaction, and binds every timer it finds to an LF timer. A user `cancel()` on
a bound timer is honoured: `ros2_lf_invoke_timer` skips the callback until `reset()`.
A timer created after that point, for example in a timer callback or from a service
response, is not bound and never fires. The analyzer reports it as
[RCLLF-H006](diagnostic-codes.md#rcllf-h006). Create every timer in the constructor.

---

### `steady_clock::now()` returns wall time

`RCL_STEADY_TIME` and `std::chrono::steady_clock` are not intercepted. Callbacks
may use them for diagnostics (latency measurement, logging), but must not use
them to drive scheduling decisions (see [ADR 0001](../design/adr/0001-lf-owns-time.md)).

---

### DDS deadline events never fire

DDS deadline, liveliness and lifespan events are wall-clock based and never reach your
callbacks. rcllf reports a missed deadline itself, at a logical tag: a
`deadline_callback` registered on a subscription runs for a missing message and for a
producer that falls silent. See [Missing Messages](../features/missing-messages.md).
Do not rely on the DDS-side deadline for anything else on a topic in the LF graph.

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
