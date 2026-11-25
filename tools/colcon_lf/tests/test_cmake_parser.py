# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for colcon_lf.node_analysis.cmake_parser.

Phase 1 — TDD assertions (6 required by plan):
1. pubsub_example: publisher_node -> src/publisher_node.cpp
2. pubsub_example: subscriber_node -> src/subscriber_node.cpp
3. timer_example:  timer_node     -> src/timer_node.cpp
4. pubsub_example: publisher_node dependencies == ["rclcpp", "custom_interfaces"]
5. component_example: component_node library exists, type == "SHARED"
6. custom_interfaces: interfaces == the three expected files

Additional sanity checks to cover remaining packages.
"""

from __future__ import annotations


# ---------------------------------------------------------------------------
# Assertions 1-4: pubsub_example
# ---------------------------------------------------------------------------

class TestPubsubExample:
    def test_publisher_node_executable(self, cmake):
        info = cmake("pubsub_example")
        assert info.executables["publisher_node"] == "src/publisher_node.cpp"

    def test_subscriber_node_executable(self, cmake):
        info = cmake("pubsub_example")
        assert info.executables["subscriber_node"] == "src/subscriber_node.cpp"

    def test_publisher_node_dependencies(self, cmake):
        info = cmake("pubsub_example")
        assert info.dependencies["publisher_node"] == ["rclcpp", "custom_interfaces"]

    def test_subscriber_node_dependencies(self, cmake):
        info = cmake("pubsub_example")
        assert info.dependencies["subscriber_node"] == ["rclcpp", "custom_interfaces"]

    def test_package_name(self, cmake):
        info = cmake("pubsub_example")
        assert info.package_name == "pubsub_example"


# ---------------------------------------------------------------------------
# Assertion 3: timer_example
# ---------------------------------------------------------------------------

class TestTimerExample:
    def test_timer_node_executable(self, cmake):
        info = cmake("timer_example")
        assert info.executables["timer_node"] == "src/timer_node.cpp"

    def test_timer_node_dependency(self, cmake):
        info = cmake("timer_example")
        assert info.dependencies["timer_node"] == ["rclcpp"]

    def test_no_libraries(self, cmake):
        info = cmake("timer_example")
        assert info.libraries == {}

    def test_no_interfaces(self, cmake):
        info = cmake("timer_example")
        assert info.interfaces == []


# ---------------------------------------------------------------------------
# Assertion 5: component_example
# ---------------------------------------------------------------------------

class TestComponentExample:
    def test_library_exists(self, cmake):
        info = cmake("component_example")
        assert "component_node" in info.libraries

    def test_library_is_shared(self, cmake):
        info = cmake("component_example")
        assert info.libraries["component_node"].type == "SHARED"

    def test_library_source(self, cmake):
        info = cmake("component_example")
        assert "src/component_node.cpp" in info.libraries["component_node"].sources

    def test_component_plugin_registered(self, cmake):
        info = cmake("component_example")
        assert info.libraries["component_node"].component_plugins == ["component_example::ComponentNode"]

    def test_no_executables(self, cmake):
        info = cmake("component_example")
        assert info.executables == {}


# ---------------------------------------------------------------------------
# Assertion 6: custom_interfaces
# ---------------------------------------------------------------------------

class TestCustomInterfaces:
    def test_interfaces_list(self, cmake):
        info = cmake("custom_interfaces")
        assert info.interfaces == [
            "msg/SensorData.msg",
            "msg/FusedState.msg",
            "msg/Detections.msg",
            "msg/Plan.msg",
            "msg/BrakeCmd.msg",
            "msg/SyncSample.msg",
            "srv/ProcessData.srv",
            "action/Analyze.action",
        ]

    def test_no_executables(self, cmake):
        info = cmake("custom_interfaces")
        assert info.executables == {}

    def test_no_libraries(self, cmake):
        info = cmake("custom_interfaces")
        assert info.libraries == {}


# ---------------------------------------------------------------------------
# Additional packages: service_example, action_example, lifecycle_example
# ---------------------------------------------------------------------------

class TestServiceExample:
    def test_service_server_executable(self, cmake):
        info = cmake("service_example")
        assert info.executables["service_server_node"] == "src/service_server_node.cpp"

    def test_service_client_executable(self, cmake):
        info = cmake("service_example")
        assert info.executables["service_client_node"] == "src/service_client_node.cpp"

    def test_dependencies_include_custom_interfaces(self, cmake):
        info = cmake("service_example")
        assert "custom_interfaces" in info.dependencies["service_server_node"]
        assert "custom_interfaces" in info.dependencies["service_client_node"]


class TestActionExample:
    def test_action_server_executable(self, cmake):
        info = cmake("action_example")
        assert info.executables["action_server_node"] == "src/action_server_node.cpp"

    def test_action_client_executable(self, cmake):
        info = cmake("action_example")
        assert info.executables["action_client_node"] == "src/action_client_node.cpp"

    def test_rclcpp_action_dependency(self, cmake):
        info = cmake("action_example")
        assert "rclcpp_action" in info.dependencies["action_server_node"]


class TestLifecycleExample:
    def test_lifecycle_node_executable(self, cmake):
        info = cmake("lifecycle_example")
        assert info.executables["lifecycle_node"] == "src/lifecycle_node.cpp"

    def test_rclcpp_lifecycle_dependency(self, cmake):
        info = cmake("lifecycle_example")
        assert "rclcpp_lifecycle" in info.dependencies["lifecycle_node"]


class TestParameterExample:
    def test_parameter_node_executable(self, cmake):
        info = cmake("parameter_example")
        assert info.executables["parameter_node"] == "src/parameter_node.cpp"


class TestBasicComponents:
    def test_library_exists(self, cmake):
        info = cmake("basic_components")
        assert "basic_components" in info.libraries

    def test_library_is_shared(self, cmake):
        info = cmake("basic_components")
        assert info.libraries["basic_components"].type == "SHARED"

    def test_all_eight_plugins_registered(self, cmake):
        info = cmake("basic_components")
        plugins = info.libraries["basic_components"].component_plugins
        assert len(plugins) == 8
        expected = [
            "basic_components::TimerPublisherNode",
            "basic_components::RelayNode",
            "basic_components::TimerAndSubscriptionNode",
            "basic_components::ListenerNode",
            "basic_components::TimerOnlyNode",
            "basic_components::DualListenerNode",
            "basic_components::DualTimerPublisherNode",
            "basic_components::DualSubscriptionTimerPublisherNode",
        ]
        assert plugins == expected

    def test_all_eight_sources(self, cmake):
        info = cmake("basic_components")
        sources = info.libraries["basic_components"].sources
        assert len(sources) == 8

    def test_no_executables(self, cmake):
        info = cmake("basic_components")
        assert info.executables == {}


class TestCallbackGroupExample:
    def test_callback_group_node_executable(self, cmake):
        info = cmake("callback_group_example")
        assert info.executables["callback_group_node"] == "src/callback_group_node.cpp"

    def test_std_msgs_dependency(self, cmake):
        info = cmake("callback_group_example")
        assert "std_msgs" in info.dependencies["callback_group_node"]


# ---------------------------------------------------------------------------
# CMake set() variable resolution (Nav2-style patterns)
# ---------------------------------------------------------------------------

class TestCMakeVariableResolution:
    """Test that set() variables are resolved in target names and sources."""

    def test_resolve_set_variables(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(nav2_collision_monitor)

set(monitor_executable_name collision_monitor)
set(monitor_library_name ${monitor_executable_name}_core)

add_executable(${monitor_executable_name} src/main.cpp)

add_library(${monitor_library_name} SHARED
  src/collision_monitor.cpp
  src/polygon.cpp
)

ament_target_dependencies(${monitor_executable_name}
  rclcpp
)

rclcpp_components_register_nodes(${monitor_library_name}
  "nav2_collision_monitor::CollisionMonitor"
)
""")
        info = parse(cmake)

        # Executable name should be resolved
        assert "collision_monitor" in info.executables
        assert info.executables["collision_monitor"] == "src/main.cpp"

        # Library name should be resolved (chained: ${monitor_executable_name}_core -> collision_monitor_core)
        assert "collision_monitor_core" in info.libraries
        lib = info.libraries["collision_monitor_core"]
        assert "src/collision_monitor.cpp" in lib.sources
        assert "src/polygon.cpp" in lib.sources

        # Dependencies should resolve target name
        assert "collision_monitor" in info.dependencies
        assert "rclcpp" in info.dependencies["collision_monitor"]

        # Component plugin should be registered on resolved library name
        assert "nav2_collision_monitor::CollisionMonitor" in lib.component_plugins


# ---------------------------------------------------------------------------
# Autoware-style CMake patterns (ament_auto_*, singular register_node)
# ---------------------------------------------------------------------------

class TestAmentAutoAddExecutable:
    """Test ament_auto_add_executable() parsing."""

    def test_basic(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(autoware_topic_tools)

ament_auto_add_executable(topic_snapshot_saver src/topic_snapshot_saver.cpp)
""")
        info = parse(cmake)
        assert "topic_snapshot_saver" in info.executables
        assert info.executables["topic_snapshot_saver"] == "src/topic_snapshot_saver.cpp"

    def test_with_project_name(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(autoware_ndt_scan_matcher)

ament_auto_add_executable(${PROJECT_NAME}_node src/main.cpp)
""")
        info = parse(cmake)
        assert "autoware_ndt_scan_matcher_node" in info.executables


class TestAmentAutoAddLibrary:
    """Test ament_auto_add_library() parsing."""

    def test_shared_with_sources(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(autoware_ground_segmentation)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ray_ground_filter/node.cpp
  src/scan_ground_filter/node.cpp
)
""")
        info = parse(cmake)
        assert "autoware_ground_segmentation" in info.libraries
        lib = info.libraries["autoware_ground_segmentation"]
        assert lib.type == "SHARED"
        assert "src/ray_ground_filter/node.cpp" in lib.sources
        assert "src/scan_ground_filter/node.cpp" in lib.sources


class TestSingularRegisterNode:
    """Test rclcpp_components_register_node() (singular) parsing."""

    def test_plugin_and_executable(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(autoware_ground_segmentation)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scan_ground_filter/node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ground_segmentation::ScanGroundFilterComponent"
  EXECUTABLE scan_ground_filter_node)
""")
        info = parse(cmake)
        lib = info.libraries["autoware_ground_segmentation"]
        assert "autoware::ground_segmentation::ScanGroundFilterComponent" in lib.component_plugins

    def test_executable_registered(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(autoware_ground_segmentation)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scan_ground_filter/node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ground_segmentation::ScanGroundFilterComponent"
  EXECUTABLE scan_ground_filter_node)
""")
        info = parse(cmake)
        # EXECUTABLE keyword should create an entry in info.executables
        assert "scan_ground_filter_node" in info.executables

    def test_with_executor_keyword(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(autoware_ndt_scan_matcher)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ndt_scan_matcher.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ndt_scan_matcher::NDTScanMatcher"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor)
""")
        info = parse(cmake)
        lib = info.libraries["autoware_ndt_scan_matcher"]
        assert "autoware::ndt_scan_matcher::NDTScanMatcher" in lib.component_plugins
        assert "autoware_ndt_scan_matcher_node" in info.executables

    def test_multiple_register_nodes(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(ros2_socketcan)

ament_auto_add_library(socket_can_receiver_node SHARED
  src/socket_can_receiver_node.cpp
)
ament_auto_add_library(socket_can_sender_node SHARED
  src/socket_can_sender_node.cpp
)

rclcpp_components_register_node(socket_can_receiver_node
  PLUGIN "drivers::socketcan::SocketCanReceiverNode"
  EXECUTABLE socket_can_receiver_node_exe
)

rclcpp_components_register_node(socket_can_sender_node
  PLUGIN "drivers::socketcan::SocketCanSenderNode"
  EXECUTABLE socket_can_sender_node_exe
)
""")
        info = parse(cmake)
        assert "socket_can_receiver_node_exe" in info.executables
        assert "socket_can_sender_node_exe" in info.executables
        assert "drivers::socketcan::SocketCanReceiverNode" in info.libraries["socket_can_receiver_node"].component_plugins
        assert "drivers::socketcan::SocketCanSenderNode" in info.libraries["socket_can_sender_node"].component_plugins


    def test_project_name_still_resolves(self, tmp_path):
        from colcon_lf.node_analysis.cmake_parser import parse

        cmake = tmp_path / "CMakeLists.txt"
        cmake.write_text("""\
project(my_pkg)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Foo.msg"
  DEPENDENCIES std_msgs
)
""")
        info = parse(cmake)
        assert "msg/Foo.msg" in info.interfaces
