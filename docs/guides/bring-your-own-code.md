# Bring Your Own ROS 2 Code

This guide takes a ROS 2 package of your own through the rcllf pipeline
(**analyze → configure → generate → build → run**). It starts by creating a small
package from scratch, so you can see every step on code you wrote, and then explains
each stage with a pointer to its reference page.

If you have not run a bundled example yet, do
[Your First LF Run](../getting-started/first-lf-run.md) first. If your packages already
live in a workspace of their own, go to
[Use an existing workspace](#use-an-existing-workspace).

## What rcllf needs from your package

- **C++ nodes (`rclcpp`)** that set up their timers, publishers, subscriptions and
  services in the constructor. rcllf reads the source, so the setup must be visible
  there.
- **A Python launch file** that starts the nodes. The analyzer reads the launch file to
  find out which nodes run together, under which names, namespaces and remappings.
  A package without a launch file has nothing to analyze.
- **Targets declared in `CMakeLists.txt` itself**: `add_executable()`, or
  `add_library()` with `rclcpp_components_register_node()` or
  `rclcpp_components_register_nodes()`. The analyzer reads the CMake text and does not
  expand CMake functions or macros of your own, so a node target created inside one
  does not resolve to its source.
- **Callbacks written as the whole argument**: an inline lambda, or
  `std::bind(&Class::method, this)`. A timer or subscription callback first stored in
  a local variable is refused with RCLLF-H007.
- **A build with `compile_commands.json`**, so the analyzer can parse your sources with
  the same flags the compiler used (`-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`).
- **Node classes that take `rclcpp::NodeOptions`** in their constructor (with a default
  value). The generated program creates one instance per launch-file node and passes
  the name and remappings through `NodeOptions`. A class without that constructor still
  runs, with the warning described in
  [Troubleshooting](../reference/troubleshooting.md#node-class-does-not-accept-nodeoptions).

## Create your own package

Everything below runs inside the dev container (`docker compose run --rm dev`). The
output shown is from a real run; your timestamps and paths will differ.

### 1. Create the package in `packages/src`

`packages/` is the colcon workspace and `packages/src/` holds its packages. A package
created there is visible to `colcon build` and to every `colcon lf` command, and it
lives in your checkout (the container mounts it), so it survives the container.

```bash
cd /workspace/packages/src
ros2 pkg create my_robot --build-type ament_cmake --dependencies rclcpp std_msgs
```

```text
going to create a new package
package name: my_robot
destination directory: /workspace/packages/src
...
build type: ament_cmake
dependencies: ['rclcpp', 'std_msgs']
creating folder ./my_robot
creating ./my_robot/package.xml
...
creating ./my_robot/CMakeLists.txt
```

`ros2 pkg create` also warns that the license is `TODO`; set it in `package.xml` when
you like, rcllf does not care.

### 2. Write two nodes

`my_robot/src/talker.cpp` publishes a counter every 250 ms:

```cpp
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
  explicit Talker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("talker", options)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>("counter", 10);
    timer_ = create_wall_timer(250ms, std::bind(&Talker::on_timer, this));
  }

private:
  void on_timer()
  {
    std_msgs::msg::Int32 msg;
    msg.data = count_++;
    RCLCPP_INFO(get_logger(), "send %d", msg.data);
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
```

`my_robot/src/listener.cpp` logs what it receives:

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class Listener : public rclcpp::Node
{
public:
  explicit Listener(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("listener", options)
  {
    sub_ = create_subscription<std_msgs::msg::Int32>(
      "counter", 10, std::bind(&Listener::on_msg, this, std::placeholders::_1));
  }

private:
  void on_msg(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "got %d", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
```

These are ordinary ROS 2 nodes. Nothing in them refers to rcllf or Lingua Franca.

### 3. Add a launch file

`ros2 pkg create` makes no `launch/` directory, so create it first:

```bash
mkdir -p my_robot/launch
```

`my_robot/launch/bringup.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='my_robot', executable='talker', name='talker', output='screen'),
        Node(package='my_robot', executable='listener', name='listener', output='screen'),
    ])
```

The launch file is what rcllf turns into one program: each `Node(...)` becomes one node
instance in the generated system, and the file name (`bringup`) becomes the name you
launch it by.

### 4. Build and install

Add these lines to `my_robot/CMakeLists.txt`, before `if(BUILD_TESTING)`:

```cmake
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS talker listener DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

Then build it with compile commands exported:

```bash
cd /workspace/packages
colcon build --symlink-install --packages-select my_robot \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

```text
Finished <<< my_robot [6.55s]

Summary: 1 package finished [6.65s]
```

At this point `ros2 launch my_robot bringup.launch.py` runs it as plain ROS 2.

### 5. Run the rcllf pipeline

`colcon lf` runs analyze, configure, generate and build in one go:

```bash
colcon lf --package my_robot
```

```text
============================================================
Step 1/4: colcon lf-analyze
============================================================
==> Found 1 launch file(s) to analyze

--- my_robot/bringup.launch.py ---
  2 nodes, 1 topics, 0 services, 0 actions
  -> /workspace/packages/lf-analysis/my_robot/bringup/

==> All 1 launch file(s) analyzed successfully
...
--- /workspace/packages/lf-analysis/my_robot/bringup/system.json ---
  2 nodes, 1 topics
...
  -> /workspace/packages/lf-generated/lf_generated_my_robot_bringup/ (lfc package)

==> All 1 system(s) generated successfully

============================================================
Step 4/4: colcon lf-build
============================================================
==> Building 1 generated package(s): lf_generated_my_robot_bringup

$ colcon build --packages-select lf_generated_my_robot_bringup --continue-on-error
...
Summary: 1 package finished [17.7s]
  1 package had stderr output: lf_generated_my_robot_bringup

==> All 1 generated package(s) built

System            Generation  Built  Reason
my_robot/bringup  generated   yes

============================================================
LF pipeline complete. Use 'ros2 lf list' to see available systems, 'ros2 lf launch <package> <stem>' to run one.
============================================================
```

The build step prints the Lingua Franca compiler's progress on stderr and colcon reports
"1 package had stderr output"; that is normal. The table at the end has one row per system:
whether it generated, whether its generated package built, and why not. `colcon lf` exits
non-zero when a row failed.

### 6. List and launch

```bash
source install/setup.bash
ros2 lf list
```

```text
Package                  Launch Stem                  Nodes  Type         Launch Ready
----------------------------------------------------------------------------------------
my_robot                 bringup                          2  nodes        ready
pubsub_example           bringup                          2  nodes        ready
...
```

```bash
ros2 lf launch my_robot bringup
```

```text
Launching LF-controlled system: my_robot/bringup
  Package: lf_generated_my_robot_bringup
  Launch file: lf_bringup.launch.py
...
[Bringup-1] [INFO] [1794837977.846839969] [lf_component_manager]: Registered non-component node: Talker
[Bringup-1] [INFO] [1794837977.846851219] [lf_component_manager]: Registered non-component node: Listener
...
[Bringup-1] [INFO] [...] [ros2_lf_binding.corr]: rcllf corr: admitted edge 0 Talker -> Listener on '/counter' (d = 0 ns, T = 100000000 ns, L = 200000000 ns, ...)
...
[Bringup-1] [INFO] [1794837977.850152587] [ros2_lf_binding]: Bound subscription RID 1 (Listener::on_msg, topic 'counter', resolved '/counter', take_mode=skip timeout_ms=100)
[Bringup-1] [INFO] [1794837977.850158877] [ros2_lf_binding]: Bound timer RID 0 (Talker::on_timer, period 250000000 ns)
...
[Bringup-1] [INFO] [1794837977.850408547] [talker]: send 0
[Bringup-1] [INFO] [1794837977.851212767] [listener]: got 0
...
[Bringup-1] [INFO] [1794837978.099828033] [talker]: send 1
[Bringup-1] [INFO] [1794837978.100199133] [listener]: got 1
[Bringup-1] [INFO] [1794837978.349846757] [talker]: send 2
[Bringup-1] [INFO] [1794837978.350148147] [listener]: got 2
```

Both nodes now run in one process. The talker's timer became a Lingua Franca timer
(`Bound timer RID 0`, period 250 ms), and every `send N` is followed by its `got N`
before the next timer firing: the listener's callback is ordered after the publish by
the logical clock, not by when DDS delivers the message. The `admitted edge` line says
the `/counter` connection delivers each publish as its own sample
([publication correspondence](../design/time-and-coordination.md#publication-correspondence-topic-edges)). Stop it with Ctrl-C; the
`WARNING: ---- There are 1 unprocessed future events` lines at shutdown are normal (the
pending event is the next timer firing).

## Use an existing workspace

Your packages do not have to move into the rcllf checkout. Mount your colcon workspace
into the dev container and run the pipeline there. rcllf writes its output into your
workspace and leaves the checkout unchanged.

The commands and output below are from a run on a workspace `~/my_ws` with three
source trees under `src/`, none of them part of rcllf:

| Directory | What | Revision |
|---|---|---|
| `src/examples` | [ros2/examples](https://github.com/ros2/examples), branch `jazzy`, unmodified | `0700885` |
| `src/demos` | [ros2/demos](https://github.com/ros2/demos), branch `jazzy`, unmodified | `37e25e0` |
| `src/my_bringup` | a launch-only `ament_cmake` package written for this run | |

`my_bringup` holds `launch/minimal_pubsub.launch.py`, which starts the minimal publisher
and subscriber from ros2/examples:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='examples_rclcpp_minimal_publisher',
             executable='publisher_member_function', name='minimal_publisher',
             output='screen'),
        Node(package='examples_rclcpp_minimal_subscriber',
             executable='subscriber_member_function', name='minimal_subscriber',
             output='screen'),
    ])
```

Its `CMakeLists.txt` only installs `launch/`, and its `package.xml` lists the two
example packages as `exec_depend`.

### 1. Mount the workspace

From the rcllf checkout on the host:

```bash
docker compose run --rm -v ~/my_ws:/my_ws dev bash
```

The container runs as uid 1000, so that user needs write access to `~/my_ws`. The
first start of the container builds rcllf itself under `/workspace/packages`.

### 2. Install dependencies and build with `compile_commands.json`

Inside the container:

```bash
source /workspace/packages/install/setup.bash
cd /my_ws
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-up-to my_bringup
source install/setup.bash
```

The dev image ships without apt package lists and without a rosdep cache, so run both
updates first. Without `rosdep update`, `rosdep install` asks for it. Without
`apt-get update`, a system dependency fails with `Unable to locate package` (ros2/demos'
`image_tools` needs OpenCV, for one). `--packages-up-to my_bringup` builds the bringup
package and the packages it depends on. The analyzer reads
`build/<package>/compile_commands.json`, so build every package whose nodes your launch
file starts with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.

### 3. Run the pipeline

```bash
colcon lf --package my_bringup
source install/setup.bash
ros2 lf list
```

```
Step 1/4: colcon lf-analyze
==> Found 1 launch file(s) to analyze
--- my_bringup/minimal_pubsub.launch.py ---
  2 nodes, 1 topics, 0 services, 0 actions
  -> /my_ws/lf-analysis/my_bringup/minimal_pubsub/
...
Step 4/4: colcon lf-build
==> Building 1 generated package(s): lf_generated_my_bringup_minimalpubsub
...
==> All 1 generated package(s) built
Package                  Launch Stem                  Nodes  Type         Launch Ready
my_bringup               minimal_pubsub                   2  nodes        ready
```

The analyzer finds a package by the `<name>` in its `package.xml`, not by its
directory: ros2/examples keeps `examples_rclcpp_minimal_publisher` in
`rclcpp/topics/minimal_publisher/`.

`colcon lf --package` converts every launch file of the package. To convert one launch
file of a package that has several, run the stages one by one:

```bash
colcon lf-analyze --package my_bringup --launch-file minimal_pubsub.launch.py
colcon lf-generate --system-json lf-analysis/my_bringup/minimal_pubsub/system.json
colcon lf-build --package my_bringup
```

Launch files may sit in subdirectories of `launch/` (ros2/demos keeps its launch files
in `launch/topics/` and `launch/services/`). `--launch-file` takes the path relative to
`launch/` (`./<name>` for a file directly in `launch/`), or a bare file name that occurs
once. Two launch files with the same name in
different subdirectories would share an output directory, so `colcon lf-analyze --package`
refuses them and names both. See the
[CLI Reference](../reference/cli.md#colcon-lf-analyze).

### 4. Launch

```bash
ros2 lf launch my_bringup minimal_pubsub
```

```
[MinimalPubsub-1] [WARN] [ros2_lf_binding]: Node class 'MinimalPublisher' does not accept NodeOptions; instance identity not realized (instances share the constructor-defined node name and namespace).
[MinimalPubsub-1] [WARN] [ros2_lf_binding]: Node class 'MinimalSubscriber' does not accept NodeOptions; instance identity not realized (instances share the constructor-defined node name and namespace).
[MinimalPubsub-1] [INFO] [ros2_lf_binding.corr]: rcllf corr: admitted edge 0 MinimalPublisher -> MinimalSubscriber on '/topic' (d = 0 ns, ...)
[MinimalPubsub-1] [INFO] [ros2_lf_binding]: Bound subscription RID 1 (MinimalSubscriber::topic_callback, topic 'topic', resolved '/topic', take_mode=skip timeout_ms=100)
[MinimalPubsub-1] [INFO] [ros2_lf_binding]: Bound timer RID 0 (MinimalPublisher::timer_callback, period 500000000 ns)
[MinimalPubsub-1] [INFO] [minimal_publisher]: Publishing: 'Hello, world! 0'
[MinimalPubsub-1] [INFO] [minimal_subscriber]: I heard: 'Hello, world! 0'
[MinimalPubsub-1] [INFO] [minimal_publisher]: Publishing: 'Hello, world! 1'
[MinimalPubsub-1] [INFO] [minimal_subscriber]: I heard: 'Hello, world! 1'
...
[MinimalPubsub-1] [INFO] [ros2_lf_binding.corr]: rcllf corr: {"event":"reconcile", ... "outcomes":{"MATCHED":23}, ... "missing_total":0, ... "reconciled":true ...}
```

(Timestamps removed.) Both nodes run in one process. The publisher's 500 ms wall timer
became a Lingua Franca timer, and in this 12 second run each of the 23 `Publishing`
lines (0 to 22) was followed by the `I heard` line with the same value.

### Where the output goes

Everything is written into the mounted workspace:

| Path under `/my_ws` | What |
|---|---|
| `lf-analysis/my_bringup/minimal_pubsub/system.json` | The extracted system: nodes, callbacks, topics, diagnostics |
| `lf-analysis/my_bringup/minimal_pubsub/nodes/`, `launch/` | Per-node models and the launch graph (`launch_graph.json`, `launch_graph.svg`) |
| `lf-analysis/my_bringup/minimal_pubsub/viz/` | `comm_graph.svg` (nodes and topics) and the other graphs, written by `colcon lf-analyze` but not by `colcon lf` |
| `lf-analysis/my_bringup/minimal_pubsub/config/execution.yaml` | Your execution configuration, created once, never overwritten |
| `lf-analysis/my_bringup/minimal_pubsub/lf/MinimalPubsub.lf` | The generated Lingua Franca program |
| `lf-analysis/my_bringup/minimal_pubsub/resolved_model.json` | The resolved model with its claim status |
| `lf-generated/lf_generated_my_bringup_minimalpubsub/` | The generated colcon package |
| `build/`, `install/`, `log/` | colcon's usual output, now including the generated package |

### Which nodes and connections rcllf found

Before you build anything, `ros2 lf scan` says whether each launch file converts and
what it found:

```bash
ros2 lf scan --workspace /my_ws --package my_bringup
```

```
workspace /my_ws: 1 packages, 1 launch files yes=1, warn=0, no=0

my_bringup/minimal_pubsub.launch.py: convertible=yes, 2 nodes, 1 topics
  launched examples_rclcpp_minimal_publisher/publisher_member_function name=minimal_publisher namespace=None
  launched examples_rclcpp_minimal_subscriber/subscriber_member_function name=minimal_subscriber namespace=None
  node minimal_publisher (MinimalPublisher) pub ['/topic'] sub [] timers 1
  node minimal_subscriber (MinimalSubscriber) pub [] sub ['/topic'] timers 0
  topic /topic [std_msgs::msg::String] ['minimal_publisher'] -> ['minimal_subscriber']
```

`--json` prints the same as an `rcllf.scan/1` document. After analysis, the
connections are in `system.json` under `communication`. `colcon lf` analyzes with
`--format json` and draws no graphs. Run
`colcon lf-analyze --package my_bringup` on its own to also get `viz/comm_graph.svg`.

Sample correspondence (each callback gets exactly the sample its publish produced)
applies only to the connections the program admits at start. Each one logs an
`rcllf corr: admitted edge` line, and the `reconcile` record at shutdown counts the
matched and missing samples. A connection without an `admitted edge` line takes
whatever sample is in the DDS reader when the callback fires. The criteria are in
[Publication correspondence](../design/time-and-coordination.md#publication-correspondence-topic-edges).

### Three outcomes and what to do next

**Converts.** The run above: `convertible=yes`, the program builds and runs. Nothing to
do.

**Converts with a warning.** The two `does not accept NodeOptions` lines above. The
ros2/examples node classes have a constructor without `rclcpp::NodeOptions`, so the
launch file's `name=` and remappings cannot reach them. This run is unaffected because
the launch names equal the constructor names and nothing is remapped. The warning
appears only at run time, not in `lf-analyze` or `ros2 lf scan`. Next step: give the
class a constructor that takes `NodeOptions`, as
[Troubleshooting](../reference/troubleshooting.md#node-class-does-not-accept-nodeoptions)
shows. A node whose package source is not under `src/` (a binary package from
`/opt/ros`) is also a warning, `not analyzed: package ... is not under /my_ws/src`, and
that node is left out of the program. Next step: add its source to `src/` if it should
be in.

**Refused.** The talker and listener of an unmodified ros2/demos `demo_nodes_cpp`, started
from a launch file the same way:

```
  Error: node 'talker' (package 'demo_nodes_cpp', executable 'talker'): the executable does not resolve to a source file in the package's CMakeLists.txt
```

`ros2 lf scan` reports `convertible=no` with the same reason. `demo_nodes_cpp` creates
its targets inside its own CMake function, `create_demo_library()`, and the analyzer
does not expand CMake functions. Next step: write the `add_library` and
`rclcpp_components_register_node` calls for these nodes directly in `CMakeLists.txt`.
With that done, analysis passes and generation stops at the next problem:

```
  Error: generation refused — error-severity hazard diagnostics present:
    ! RCLLF-H007 (2 occurrence(s))
      node 'talker' creates timer 'timer_' (period 1000 ms) with callback argument `publish_message`, which is neither form the analyzer can model. ...
```

The demo stores each callback in a local variable before passing it. Next step: pass
the callback as an inline lambda or `std::bind(&Class::method, this)`. The message also
offers to list `RCLLF-H007` under `acknowledged_hazards:` in `execution.yaml`. That
only skips the check: a timer or subscription the analyzer cannot read gets no reaction,
so it never fires under rcllf.

### Where to look when a stage fails

| Stage | Where |
|---|---|
| Analyze | The `Error:` or `Warning:` line names the node, package and executable. [Troubleshooting](../reference/troubleshooting.md#colcon-lf-analyze-fails-with-error-node-for-a-launched-node) explains each message. |
| Configure, generate | Hazard diagnostics (`RCLLF-H...`) are printed and kept in `system.json` under `diagnostics`. The [Support Matrix](../features/support-matrix.md) lists them. |
| Build | `log/latest_build/lf_generated_<package>_<stem>/stdout_stderr.log` in your workspace. |
| Run | The `Bound timer` and `Bound subscription` lines list every callback rcllf drives. A callback without such a line never fires. |

## The stages one by one

`colcon lf` is a shortcut. Each stage also runs on its own, which helps when one fails.

### Analyze

```bash
colcon lf-analyze --package my_robot
```

This writes `lf-analysis/my_robot/bringup/system.json`, the model of your system:
every node, its timers, publishers, subscriptions, services and actions, the topic
connections between nodes, and the launch-file hierarchy. It also records the QoS
hazard checks used in the next stages.

Go deeper: [CLI Reference](../reference/cli.md) (options, output tree).

### Configure

```bash
colcon lf-configure --package my_robot
colcon lf-configure --package my_robot --show   # print the resolved policy, write nothing
```

`colcon lf-configure` creates two files next to `system.json`: a generated
`execution.reference.yaml` that documents every valid key, and a sparse, user-owned
`execution.yaml` (a key that is present is set; a missing key is inherited). You edit
the second one, for example to set how the boundary between ROS and Lingua Franca takes
messages (`skip` or `block`) per node or per subscription.

If the analysis found a QoS hazard (an unreliable input, a shallow queue, a topic name
computed at run time, or a `message_filters` form other than the two-input ExactTime or
ApproximateTime synchronizer), generation enforces it: warnings print and generation goes
on. An error-level hazard (for example **RCLLF-H003**, **H007**, **H009** or **H011**)
refuses generation. Some of them can be acknowledged by adding the code to
`acknowledged_hazards` in `execution.yaml`. The
[support matrix](../features/support-matrix.md) lists every hazard code with its level, and
[Troubleshooting](../reference/troubleshooting.md) says what to do about each.

Go deeper: [Execution Configuration](../features/execution-config.md) (the two-file
model, precedence, `acknowledged_hazards`).

### Generate

```bash
colcon lf-generate --package my_robot
```

This writes the Lingua Franca program under `lf-analysis/my_robot/bringup/lf/` and a
colcon package `lf_generated_my_robot_bringup` under `lf-generated/`. It reads
`execution.yaml` if present (`--exec-config <path>` points at another file).
`--package <name>` selects every analysis of a package; `--system-json <path>` selects
exactly one. `--federate per-node` splits the program into one federate per node, see
[Federation](../features/federation.md).

A two-input ExactTime or ApproximateTime `message_filters` synchronizer runs unmodified.
Other `message_filters` forms have no automatic conversion; rewrite them to that form or
express them in Lingua Franca terms.

### Build and run

```bash
colcon lf-build --package my_robot
ros2 lf list
ros2 lf launch my_robot bringup
```

`colcon lf-build` runs the Lingua Franca compiler and builds the generated package.
`ros2 lf list` shows every generated system in the workspace; `ros2 lf launch` starts
one through its generated launch file.

### Evaluate (experimental)

With LTTng tracing enabled ([Tracing](../features/tracing.md)), `lf-eval` turns a
traced run into metrics:

```bash
lf-eval analyze <run-dir> -o <out-dir>
lf-eval report <run-dir-1> <run-dir-2> ... -o <out-dir>
lf-eval list-metrics
```

Tracing and evaluation are experimental in this release.

## Examples to compare against

The bundled examples under `packages/src/` show known-good results for each stage:

- **`pubsub_example`**: a two-node publisher and subscriber, the same shape as
  `my_robot` above. Walked step by step in
  [Your First LF Run](../getting-started/first-lf-run.md).
- **`timer_example`** and **`service_example`**: timers and request/response pairs.
- **`fusion_example`**: two sensors feeding an ExactTime and an ApproximateTime
  `message_filters` synchronizer, unmodified.

## When something does not convert

Not every ROS 2 pattern has an automatic conversion, and not every value can be read
from source (a timer period computed from a parameter, for instance, is recorded as
unknown). If your package hits a wall:

- [Porting Your ROS 2 System](porting-guide.md) walks timers, subscriptions,
  components and launch files with a before and after, and the hazards each can trigger.
- [Support Matrix](../features/support-matrix.md) lists what is supported, what warns
  and what is refused.
- [Troubleshooting](../reference/troubleshooting.md) covers common failures.
