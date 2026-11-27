# Data Model Reference

The extraction tool uses Python dataclasses organized into three layers: **Launch**, **Node**, and **Communication**. All models are designed for JSON serialization.

The tables list every field of the dataclasses with the type it has in the source. The
top-level model is [`SystemGraph`](#systemgraph), the content of `system.json`. The classes
for message_filters and tf2 use are summarized under [Detail classes](#detail-classes)
instead of one table each.

Source files:

- `tools/colcon_lf/colcon_lf/graph/models.py`
- `tools/colcon_lf/colcon_lf/launch_analysis/models.py`
- `tools/colcon_lf/colcon_lf/node_analysis/models.py`
- `tools/colcon_lf/colcon_lf/comm_analysis/models.py`

## Launch Layer

### LaunchArgument

A declared launch argument (`DeclareLaunchArgument`).

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Argument name |
| `default_value` | `str \| None` | Default value |
| `description` | `str \| None` | Help text |
| `choices` | `list[str] \| None` | Allowed values |

### LaunchNodeAction

A single node resolved from a launch file.

| Field | Type | Description |
|-------|------|-------------|
| `package` | `str` | ROS 2 package name |
| `executable` | `str` | Executable name or plugin class for components |
| `name` | `str \| None` | Node name override |
| `namespace` | `str \| None` | Resolved namespace (includes pushed namespaces) |
| `parameters` | `dict[str, Any]` | Inline parameters |
| `parameter_files` | `list[Path]` | YAML config file paths |
| `remappings` | `list[tuple[str, str]]` | Topic/service remappings |
| `condition` | `str \| None` | IfCondition/UnlessCondition expression |
| `is_component` | `bool` | True if loaded via ComposableNodeContainer |
| `composable_container` | `str \| None` | Container node name if component |
| `parameter_sources` | `list[dict[str, Any]]` | `parameters=[...]` in launch order: `{"file": "<abs path>"}` or `{"params": {...}}` per entry. `parameters` and `parameter_files` lose that order |
| `condition_result` | `bool \| None` | True when launch starts the node, `None` when a condition (its own or an enclosing one) cannot be decided |
| `environment` | `dict[str, str]` | Environment variables set on this node |
| `output` | `str \| None` | `"screen"`, `"log"` or `"both"` |
| `respawn` | `bool` | Respawn policy |
| `respawn_delay` | `float \| None` | Respawn delay in seconds |
| `arguments` | `list[str]` | Command-line arguments of the executable |
| `use_intra_process_comms` | `bool \| None` | `use_intra_process_comms` of a composable node's `extra_arguments`; `None` when launch does not set it |

### LaunchGroup

A `GroupAction` with an optional pushed namespace.

| Field | Type | Description |
|-------|------|-------------|
| `namespace` | `str \| None` | Pushed namespace |
| `nodes` | `list[LaunchNodeAction]` | Direct child nodes |
| `includes` | `list[LaunchFileInfo]` | Included launch files |
| `groups` | `list[LaunchGroup]` | Nested groups |
| `condition` | `str \| None` | Group condition |

### LaunchFileInfo

Parsed content of a single launch file.

| Field | Type | Description |
|-------|------|-------------|
| `file_path` | `Path` | Absolute path to the launch file |
| `package` | `str` | Owning package name |
| `arguments` | `list[LaunchArgument]` | Declared arguments |
| `nodes` | `list[LaunchNodeAction]` | Direct child nodes |
| `includes` | `list[LaunchFileInfo]` | Recursively included launch files |
| `groups` | `list[LaunchGroup]` | Group actions |
| `errors` | `list[str]` | Problems met while reading the launch file |
| `set_parameters` | `list[tuple[str, Any]]` | `SetParameter` actions: name, value |
| `set_environment` | `list[tuple[str, str]]` | `SetEnvironmentVariable` actions: name, value |
| `set_configurations` | `list[tuple[str, str]]` | `SetLaunchConfiguration` actions: name, value |
| `execute_processes` | `list[LaunchProcessAction]` | Non-ROS processes started with `ExecuteProcess` |

### LaunchProcessAction

A non-ROS process started with `ExecuteProcess`.

| Field | Type | Description |
|-------|------|-------------|
| `cmd` | `list[str]` | Command line |
| `name` | `str \| None` | Process name |
| `cwd` | `str \| None` | Working directory |
| `environment` | `dict[str, str]` | Environment variables |
| `condition` | `str \| None` | Condition expression |
| `output` | `str \| None` | Output setting |
| `shell` | `bool` | True when run through a shell |

### LaunchGraph

Fully resolved launch graph rooted at a single launch file.

| Field | Type | Description |
|-------|------|-------------|
| `root` | `LaunchFileInfo` | Root launch file |
| `all_nodes` | `list[LaunchNodeAction]` | Flattened list with resolved namespaces |
| `context` | `LaunchContextState` | State accumulated while walking the launch file, see below |

### LaunchContextState

| Field | Type | Description |
|-------|------|-------------|
| `parameters` | `dict[str, Any]` | Parameters set with `SetParameter` |
| `environment` | `dict[str, str]` | Environment variables set while walking the launch file |
| `launch_configurations` | `dict[str, str]` | Launch configurations (argument values) in effect |

---

## Node Layer

### CallbackParameter

One input parameter of a ROS 2 callback function.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Parameter name (e.g. `"msg"`, `"request"`, `"goal"`) |
| `type` | `str` | Fully-qualified C++ type (e.g. `"custom_interfaces::msg::SensorData::SharedPtr"`) |

### CallbackInfo

A callback function inside a ROS 2 node. Separates what the callback receives from what it does.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Function name (e.g. `"timer_callback"`) |
| `parameters` | `list[CallbackParameter]` | Input parameters (empty for timer callbacks) |
| `publishers_used` | `list[str]` | Topics published to via `publish()` |
| `service_clients_used` | `list[str]` | Service names called via `async_send_request` |
| `action_clients_used` | `list[str]` | Action names called via `async_send_goal` |
| `variables_read` | `list[str]` | Member variables read |
| `variables_written` | `list[str]` | Member variables written |
| `spawns_callbacks` | `list[str]` | Callback names started with `std::thread` or `std::async` |
| `feedback_published` | `list[str]` | Action names where `publish_feedback()` is called |
| `result_published` | `list[str]` | Action names where `succeed()` or `canceled()` is called |
| `publish_bounds` | `dict[str, int \| None]` | Most publishes one invocation makes per topic, proven from the call sites (an upper bound when some are conditional). `None`: no bound, the reason is in `publish_unbounded` |
| `publish_unbounded` | `dict[str, str]` | Topic -> reason no bound could be proven |
| `missing_handler_publishers` | `list[str]` | Topics published by the subscription's requested-deadline-missed handler |
| `callers` | `list[str] \| None` | The node's own functions whose bodies call this callback; an entry in `<...>` marks a use that is not such a call. `None` when the set is not closed |

### CallbackGroupInfo

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | C++ member variable name (e.g. `"exclusive_group_"`) |
| `type` | `str` | `"MutuallyExclusive"` or `"Reentrant"` |

### TimerInfo

| Field | Type | Description |
|-------|------|-------------|
| `period_ms` | `float \| None` | Timer period in milliseconds |
| `callback` | `str` | Reference to `CallbackInfo.name` |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |
| `member_variable` | `str \| None` | C++ member that holds the timer (e.g. `"timer_"`) |
| `created_in` | `str \| None` | Method that creates it: `"constructor"`, `"on_configure"`, `"on_activate"`, ... |
| `destroyed_in` | `str \| None` | Method that destroys it: `"on_cleanup"`, `"on_deactivate"`, ... |
| `offset_ms` | `float` | First fire in milliseconds after the LF start tag. Always 0 from the analyzer, so a timer fires first at the start tag; stock rclcpp fires a wall timer first one period after creation (a later release matches that by default). `execution.yaml` `nodes.<node>.timers.<timer>.offset` overrides it at generation |
| `callback_expr` | `str \| None` | Source text of the callback argument when the analyzer could not model it (so `callback` is `""`); reported as [RCLLF-H007](diagnostic-codes.md#rcllf-h007) |

### SubscriberInfo

| Field | Type | Description |
|-------|------|-------------|
| `topic` | `str` | Topic name |
| `message_type` | `str` | Fully-qualified C++ message type |
| `callback` | `str` | Reference to `CallbackInfo.name` |
| `qos_depth` | `int \| None` | QoS history depth |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |
| `member_variable` | `str \| None` | C++ member that holds the endpoint |
| `created_in` | `str \| None` | Method that creates it |
| `destroyed_in` | `str \| None` | Method that destroys it |
| `qos_reliability` | `str \| None` | `"reliable"` or `"best_effort"` |
| `qos_durability` | `str \| None` | `"volatile"` or `"transient_local"` |
| `qos_history` | `str \| None` | `"keep_last"` or `"keep_all"` |
| `qos_deadline_ms` | `float \| None` | QoS deadline in milliseconds |
| `topic_dynamic` | `bool` | True when the topic name is built at run time (concatenation, parameter, expression), so `topic` is a best-effort value or empty and RCLLF-H004 reports it. Cleared when a later stage resolves the name. Absent in older `system.json` files, read as false |
| `topic_param` | `str \| None` | The declared string parameter whose value is the topic name; the topic is resolved from its launch value. `None` when the topic is not tied to a parameter |
| `callback_expr` | `str \| None` | Same as `TimerInfo.callback_expr` |
| `sync_callback` | `str \| None` | Set when this subscription is a `message_filters::Subscriber` input of a recognized two-input synchronizer: the user callback the synchronizer calls (`callback` is then a synthetic per-input name) |
| `tf2_listener` | `str \| None` | Set (to the `TransformListener` member) when this is one of the two subscriptions a `tf2_ros::TransformListener` creates on `/tf` and `/tf_static` |

### ServiceServerInfo

| Field | Type | Description |
|-------|------|-------------|
| `service_name` | `str` | Service name |
| `service_type` | `str` | Fully-qualified C++ service type |
| `callback` | `str` | Reference to `CallbackInfo.name` |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |
| `member_variable` | `str \| None` | C++ member that holds the endpoint |
| `created_in` | `str \| None` | Method that creates it |
| `destroyed_in` | `str \| None` | Method that destroys it |

### ActionServerInfo

| Field | Type | Description |
|-------|------|-------------|
| `action_name` | `str` | Action name |
| `action_type` | `str` | Fully-qualified C++ action type |
| `goal_callback` | `str \| None` | Goal handler callback |
| `cancel_callback` | `str \| None` | Cancel handler callback |
| `accepted_callback` | `str \| None` | Accepted handler callback |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |
| `member_variable` | `str \| None` | C++ member that holds the endpoint |
| `created_in` | `str \| None` | Method that creates it |
| `destroyed_in` | `str \| None` | Method that destroys it |

### PublisherInfo

| Field | Type | Description |
|-------|------|-------------|
| `topic` | `str` | Topic name |
| `message_type` | `str` | Fully-qualified C++ message type |
| `qos_depth` | `int \| None` | QoS history depth |
| `member_variable` | `str \| None` | C++ member that holds the publisher (e.g. `"publisher_"`) |
| `is_lifecycle` | `bool` | True for `rclcpp_lifecycle::LifecyclePublisher` |
| `created_in` | `str \| None` | Method that creates it |
| `destroyed_in` | `str \| None` | Method that destroys it |
| `qos_reliability` | `str \| None` | `"reliable"` or `"best_effort"` |
| `qos_durability` | `str \| None` | `"volatile"` or `"transient_local"` |
| `qos_history` | `str \| None` | `"keep_last"` or `"keep_all"` |
| `qos_deadline_ms` | `float \| None` | QoS deadline in milliseconds |
| `topic_dynamic` | `bool` | True when the topic name is built at run time (concatenation, parameter, expression), so `topic` is a best-effort value or empty and RCLLF-H004 reports it. Cleared when a later stage resolves the name. Absent in older `system.json` files, read as false |
| `topic_param` | `str \| None` | Same as `SubscriberInfo.topic_param` |
| `serialized_publish` | `bool` | Some call site publishes a serialized message (`rcl_publish_serialized_message`), which the binding refuses at run time |
| `unconditional` | `bool \| None` | True when the create call runs on every path through the method that creates it; False when it may not; `None` when undecided |

### ServiceClientInfo

| Field | Type | Description |
|-------|------|-------------|
| `service_name` | `str` | Service name |
| `service_type` | `str` | Fully-qualified C++ service type |
| `member_variable` | `str \| None` | C++ member that holds the endpoint |
| `created_in` | `str \| None` | Method that creates it |
| `destroyed_in` | `str \| None` | Method that destroys it |
| `response_callback` | `str \| None` | Callback passed as the second argument of `async_send_request()` |
| `call_style` | `str \| None` | `"blocking_future"` (followed by `future.wait()`/`get()`), `"blocking_wait"` (waits for the response in the same method) or `None` (undetermined, treated as blocking) |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |

### ActionClientInfo

| Field | Type | Description |
|-------|------|-------------|
| `action_name` | `str` | Action name |
| `action_type` | `str` | Fully-qualified C++ action type |
| `member_variable` | `str \| None` | C++ member that holds the endpoint |
| `created_in` | `str \| None` | Method that creates it |
| `destroyed_in` | `str \| None` | Method that destroys it |
| `goal_response_callback` | `str \| None` | Goal response handler |
| `feedback_callback` | `str \| None` | Feedback handler |
| `result_callback` | `str \| None` | Result handler |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |

### ParameterInfo

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Parameter name (e.g. `"robot_name"`) |
| `type` | `str` | Type string (`"string"`, `"double"`, `"int"`, `"bool"`, etc.) |
| `default_value` | `Any \| None` | Default value |
| `description` | `str \| None` | Parameter description |

### NodeInfo

The top-level model for a single ROS 2 node. Assembles all extracted information.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Node name |
| `class_name` | `str` | C++ class name |
| `package` | `str` | Package name |
| `source_file` | `Path` | Path to the `.cpp` source file |
| `header_file` | `Path \| None` | Path to the `.hpp` header file |
| `base_class` | `str` | `"rclcpp::Node"` or `"rclcpp_lifecycle::LifecycleNode"` |
| `is_component` | `bool` | True if `RCLCPP_COMPONENTS_REGISTER_NODE` detected |
| `qualified_name` | `str` | Namespace-qualified class name (`ns::Cls`); empty when unknown |
| `callbacks` | `list[CallbackInfo]` | All callbacks |
| `callback_groups` | `list[CallbackGroupInfo]` | All callback groups |
| `timers` | `list[TimerInfo]` | All timers |
| `subscribers` | `list[SubscriberInfo]` | All subscribers |
| `service_servers` | `list[ServiceServerInfo]` | All service servers |
| `action_servers` | `list[ActionServerInfo]` | All action servers |
| `publishers` | `list[PublisherInfo]` | All publishers |
| `service_clients` | `list[ServiceClientInfo]` | All service clients |
| `action_clients` | `list[ActionClientInfo]` | All action clients |
| `parameters` | `list[ParameterInfo]` | All declared parameters |
| `member_variables` | `list[str]` | Member variable names |
| `lifecycle_callbacks` | `list[str]` | Lifecycle methods (e.g. `"on_configure"`) |
| `lifecycle_hooks_not_chained` | `list[str]` | `on_activate` or `on_deactivate` overrides that do not call the base implementation ([RCLLF-H005](diagnostic-codes.md#rcllf-h005)) |
| `has_param_callback` | `bool` | True if `add_on_set_parameters_callback` used |
| `param_callback` | `str \| None` | Callback bound to `add_on_set_parameters_callback` |
| `parse_warnings` | `list[str]` | Missing headers or other libclang problems that may make the extraction incomplete |
| `unresolved_includes` | `list[dict[str, Any]]` | `#include` directives libclang could not resolve: `header`, `included_from`, `line` |
| `detected_includes` | `list[str]` | `message_filters` or `tf2_ros`, when the source includes them |
| `detected_take_calls` | `list[str]` | Pull-style `take` calls on a subscription |
| `detected_ros_sleep_calls` | `list[str]` | `get_clock()->sleep_for(` and `sleep_until(` calls, as `"<call>:<line>"` ([RCLLF-H008](diagnostic-codes.md#rcllf-h008)) |
| `message_filters` | `MessageFiltersInfo \| None` | Synchronizers recognized in the source; `None` when the node does not use `message_filters` |
| `tf2` | `Tf2Info \| None` | tf2 use recognized in the source; `None` when the node does not use `tf2_ros` |
| `unbound_timers` | `list[TimerInfo]` | Timers created outside the constructor and the lifecycle hooks. Not in `timers`, never generated ([RCLLF-H006](diagnostic-codes.md#rcllf-h006)) |
| `thread_action_senders` | `list[dict[str, str]]` | Threads the node starts that send action feedback or a result: `thread`, `started_in`, `action` |
| `unbound_endpoints` | `list[dict[str, str]]` | Endpoints created outside the constructor and the lifecycle hooks: `kind`, `topic`, `method`, `callback` |
| `template_base_endpoints` | `list[dict[str, str]]` | Template base classes that create endpoints in their constructor: `base`, `calls` |
| `main_outside_callbacks` | `list[dict[str, str]]` | Work `main()` does on the node outside executor callbacks: `source`, `call` |
| `compile_variant_issues` | `list[dict[str, str]]` | Compile definitions that select the source variant could not be told apart: `source`, `reason`, `fatal` |
| `base_classes_merged` | `list[str]` | Workspace base classes merged into this node, most base first |
| `compile_defines` | `list[str]` | `-DNAME[=VALUE]` definitions of the executable target the node was analyzed under |

### CMakeInfo

Parsed information from a single `CMakeLists.txt`.

| Field | Type | Description |
|-------|------|-------------|
| `package_name` | `str` | Package name from `project()` |
| `executables` | `dict[str, str]` | Target name -> first source file path |
| `libraries` | `dict[str, LibraryTarget]` | Target name -> library info |
| `dependencies` | `dict[str, list[str]]` | Target name -> dependency list |
| `target_definitions` | `dict[str, list[str]]` | Target name -> `-DNAME[=VALUE]` from `target_compile_definitions` |
| `interfaces` | `list[str]` | `rosidl_generate_interfaces` file paths |

### LibraryTarget

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Library target name |
| `type` | `str` | `"SHARED"`, `"STATIC"`, `"MODULE"`, or `""` |
| `sources` | `list[str]` | Relative source file paths |
| `component_plugins` | `list[str]` | Plugin classes from `rclcpp_components_register_nodes` (empty when there are none) |

---

## Communication Layer

### TopicConnection

| Field | Type | Description |
|-------|------|-------------|
| `topic` | `str` | Fully resolved topic name (e.g. `"/robot_1/sensor_data"`) |
| `message_type` | `str` | Fully-qualified C++ message type |
| `publishers` | `list[str]` | Publishing node keys |
| `subscribers` | `list[str]` | Subscribing node keys |
| `fields` | `list[dict] \| None` | Message fields as `{"type": ..., "name": ...}` |
| `after_delay_ns` | `int \| None` | Declared logical-time offset (`after` delay) for this link in nanoseconds. `None`: use the generator default |

### ServiceConnection

| Field | Type | Description |
|-------|------|-------------|
| `service` | `str` | Fully resolved service name |
| `service_type` | `str` | Fully-qualified C++ service type |
| `server` | `str` | Server node key |
| `clients` | `list[str]` | Client node keys |
| `request_fields` | `list[dict] \| None` | Request fields |
| `response_fields` | `list[dict] \| None` | Response fields |

### ActionConnection

| Field | Type | Description |
|-------|------|-------------|
| `action` | `str` | Fully resolved action name |
| `action_type` | `str` | Fully-qualified C++ action type |
| `server` | `str` | Server node key |
| `clients` | `list[str]` | Client node keys |
| `goal_fields` | `list[dict] \| None` | Goal fields |
| `result_fields` | `list[dict] \| None` | Result fields |
| `feedback_fields` | `list[dict] \| None` | Feedback fields |

### SystemCommunication

All inter-node communication connections in a ROS 2 system.

| Field | Type | Description |
|-------|------|-------------|
| `topics` | `list[TopicConnection]` | All topic connections |
| `services` | `list[ServiceConnection]` | All service connections |
| `actions` | `list[ActionConnection]` | All action connections |
| `endpoint_ids` | `str` | `"node_key"`: endpoints name a key of `SystemGraph.nodes`, one per node instance |

An endpoint is the node key, so an edge between two instances of one node class
names both instances. A `system.json` written before that has no
`endpoint_ids` and lists endpoints by the node name in the code (`"node_name"`).
`colcon lf-generate` still reads such a file, unless one of its endpoint names
belongs to two or more nodes; then it stops and asks for a fresh `colcon lf-analyze`.

---

## Detail classes

`NodeInfo.message_filters` and `NodeInfo.tf2` hold these classes. They are facts for the
generator and the hazard checks; you rarely read them.

| Class | Fields |
|-------|--------|
| `MessageFiltersInfo` | `synchronizers: list[MessageFiltersSyncInfo]`, `unsupported: list[str]` |
| `MessageFiltersSyncInfo` | `member_variable: str`, `policy: str`, `queue_size: int \| None`, `inputs: list[str]`, `callback: str`, `bna_params: dict[str, int] \| None`, `queue_param: str \| None` |
| `Tf2Info` | `buffers: list[Tf2BufferInfo]`, `listeners: list[Tf2ListenerInfo]`, `broadcasters: list[Tf2BroadcasterInfo]`, `lookups: list[Tf2LookupCallInfo]`, `adapters: list[Tf2AdapterInfo]`, `stamps: list[Tf2StampInfo] \| None`, `refusals: list[Tf2Refusal]` |
| `Tf2BufferInfo` | `member: str`, `clock: str`, `file: str`, `line: int` |
| `Tf2ListenerInfo` | `member: str`, `buffer_member: str \| None`, `file: str`, `line: int` |
| `Tf2BroadcasterInfo` | `member: str`, `static: bool`, `file: str`, `line: int` |
| `Tf2LookupCallInfo` | `receiver: str`, `method: str`, `timeout: bool`, `file: str`, `line: int` |
| `Tf2AdapterInfo` | `member: str`, `buffer_member: str \| None`, `wait_bound_ns: int \| None`, `on_timeout: str \| None`, `file: str`, `line: int` |
| `Tf2StampInfo` | `adapter: str`, `kind: str`, `expr: str`, `detail: str`, `callback: str \| None`, `topic: str \| None`, `file: str`, `line: int` |
| `Tf2Refusal` | `subcode: str`, `detail: str` |

---

## SystemGraph

`colcon lf-analyze` writes one `SystemGraph` per launch file to `system.json`. It joins the
three layers above and is what `colcon lf-generate` reads. Source:
`tools/colcon_lf/colcon_lf/graph/models.py`.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Launch file stem (`bringup` for `bringup.launch.py`) |
| `launch_graph` | `LaunchGraph` | The launch hierarchy: `root`, `all_nodes`, `context` |
| `nodes` | `dict[str, NodeInfo]` | Node key (the fully qualified node name) -> node internals |
| `communication` | `SystemCommunication` | Topics, services and actions between the nodes |
| `parameter_files` | `dict[str, dict]` | Parameter file name -> parsed YAML content |
| `schema_version` | `int` | Schema version, `3` in this release. Bumped when the schema changes |
| `diagnostics` | `list[Diagnostic]` | Hazard findings of the analysis, see below |

Each `Diagnostic` holds `code`, `node`, `topic` (both may be `null`) and `message`. It has
no severity: the severity of a code is looked up in [Diagnostic codes](diagnostic-codes.md)
when `colcon lf-generate` reads the file. A `system.json` without a `diagnostics` field
predates hazard detection, and `lf-generate` asks for a fresh `colcon lf-analyze`. An empty
list means "analyzed, none found".

For `pubsub_example` and `bringup.launch.py`, `colcon lf-analyze` writes a `system.json` whose
top-level keys are `name`, `launch_graph`, `nodes`, `communication`, `parameter_files`,
`schema_version` and `diagnostics`. Two excerpts from it:

```json
"timers": [{"period_ms": 500.0, "callback": "timer_callback", "callback_group": "default_group",
            "member_variable": "timer_", "created_in": "constructor", "destroyed_in": null,
            "offset_ms": 0.0, "callback_expr": null}]
```

```json
"communication": {"topics": [{"topic": "/sensor_data",
                              "message_type": "custom_interfaces::msg::SensorData",
                              "publishers": ["publisher_node"], "subscribers": ["subscriber_node"],
                              "fields": [{"type": "std_msgs/Header", "name": "header"}, ...],
                              "after_delay_ns": null}],
                  "services": [], "actions": [], "endpoint_ids": "node_key"}
```
