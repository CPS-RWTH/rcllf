# Data Model Reference

The extraction tool uses Python dataclasses organized into three layers: **Launch**, **Node**, and **Communication**. All models are designed for JSON serialization.

Source files:

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

### LaunchGroup

A `GroupAction` with an optional pushed namespace.

| Field | Type | Description |
|-------|------|-------------|
| `namespace` | `str \| None` | Pushed namespace |
| `nodes` | `list[LaunchNodeAction]` | Direct child nodes |
| `includes` | `list[LaunchFileInfo]` | Included launch files |
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

### LaunchGraph

Fully resolved launch graph rooted at a single launch file.

| Field | Type | Description |
|-------|------|-------------|
| `root` | `LaunchFileInfo` | Root launch file |
| `all_nodes` | `list[LaunchNodeAction]` | Flattened list with resolved namespaces |

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
| `offset_ms` | `float` | First fire in milliseconds after the LF start tag. Always 0 from the analyzer, so a timer fires first at the start tag; stock rclcpp fires a wall timer first one period after creation (a later release matches that by default). `execution.yaml` `nodes.<node>.timers.<timer>.offset` overrides it at generation |

### SubscriberInfo

| Field | Type | Description |
|-------|------|-------------|
| `topic` | `str` | Topic name |
| `message_type` | `str` | Fully-qualified C++ message type |
| `callback` | `str` | Reference to `CallbackInfo.name` |
| `qos_depth` | `int \| None` | QoS history depth |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |
| `topic_dynamic` | `bool` | True when the topic name is built at run time (concatenation, parameter, expression), so `topic` is a best-effort value or empty and RCLLF-H004 reports it. Cleared when a later stage resolves the name. Absent in older `system.json` files, read as false |

### ServiceServerInfo

| Field | Type | Description |
|-------|------|-------------|
| `service_name` | `str` | Service name |
| `service_type` | `str` | Fully-qualified C++ service type |
| `callback` | `str` | Reference to `CallbackInfo.name` |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |

### ActionServerInfo

| Field | Type | Description |
|-------|------|-------------|
| `action_name` | `str` | Action name |
| `action_type` | `str` | Fully-qualified C++ action type |
| `goal_callback` | `str \| None` | Goal handler callback |
| `cancel_callback` | `str \| None` | Cancel handler callback |
| `accepted_callback` | `str \| None` | Accepted handler callback |
| `callback_group` | `str \| None` | Reference to `CallbackGroupInfo.name` |

### PublisherInfo

| Field | Type | Description |
|-------|------|-------------|
| `topic` | `str` | Topic name |
| `message_type` | `str` | Fully-qualified C++ message type |
| `qos_depth` | `int \| None` | QoS history depth |
| `topic_dynamic` | `bool` | True when the topic name is built at run time (concatenation, parameter, expression), so `topic` is a best-effort value or empty and RCLLF-H004 reports it. Cleared when a later stage resolves the name. Absent in older `system.json` files, read as false |

### ServiceClientInfo

| Field | Type | Description |
|-------|------|-------------|
| `service_name` | `str` | Service name |
| `service_type` | `str` | Fully-qualified C++ service type |

### ActionClientInfo

| Field | Type | Description |
|-------|------|-------------|
| `action_name` | `str` | Action name |
| `action_type` | `str` | Fully-qualified C++ action type |

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
| `has_param_callback` | `bool` | True if `add_on_set_parameters_callback` used |

### CMakeInfo

Parsed information from a single `CMakeLists.txt`.

| Field | Type | Description |
|-------|------|-------------|
| `package_name` | `str` | Package name from `project()` |
| `executables` | `dict[str, str]` | Target name -> first source file path |
| `libraries` | `dict[str, LibraryTarget]` | Target name -> library info |
| `dependencies` | `dict[str, list[str]]` | Target name -> dependency list |
| `interfaces` | `list[str]` | `rosidl_generate_interfaces` file paths |

### LibraryTarget

| Field | Type | Description |
|-------|------|-------------|
| `name` | `str` | Library target name |
| `type` | `str` | `"SHARED"`, `"STATIC"`, `"MODULE"`, or `""` |
| `sources` | `list[str]` | Relative source file paths |
| `component_plugin` | `str \| None` | Plugin class from `rclcpp_components_register_nodes` |

---

## Communication Layer

### TopicConnection

| Field | Type | Description |
|-------|------|-------------|
| `topic` | `str` | Fully resolved topic name (e.g. `"/robot_1/sensor_data"`) |
| `message_type` | `str` | Fully-qualified C++ message type |
| `publishers` | `list[str]` | Publishing node keys |
| `subscribers` | `list[str]` | Subscribing node keys |

### ServiceConnection

| Field | Type | Description |
|-------|------|-------------|
| `service` | `str` | Fully resolved service name |
| `service_type` | `str` | Fully-qualified C++ service type |
| `server` | `str` | Server node key |
| `clients` | `list[str]` | Client node keys |

### ActionConnection

| Field | Type | Description |
|-------|------|-------------|
| `action` | `str` | Fully resolved action name |
| `action_type` | `str` | Fully-qualified C++ action type |
| `server` | `str` | Server node key |
| `clients` | `list[str]` | Client node keys |

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
