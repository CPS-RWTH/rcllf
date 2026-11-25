# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for lf_naming module."""

from __future__ import annotations

import pytest

from colcon_lf.generation.lf_naming import (
    callback_to_timer_name,
    class_name_to_reactor_name,
    deduplicate_names,
    generated_package_stem,
    launch_stem_to_reactor_name,
    message_type_to_lf_type,
    sanitize_lf_identifier,
    topic_to_port_name,
)


class TestSanitizeLfIdentifier:
    def test_simple(self):
        assert sanitize_lf_identifier("sensor_data") == "sensor_data"

    def test_leading_digit(self):
        assert sanitize_lf_identifier("3d_lidar") == "_3d_lidar"

    def test_special_chars(self):
        assert sanitize_lf_identifier("my-topic.v2") == "my_topic_v2"

    def test_empty(self):
        assert sanitize_lf_identifier("") == "unnamed"

    def test_only_slashes(self):
        assert sanitize_lf_identifier("///") == "unnamed"

    def test_underscores_stripped(self):
        # "_private" → strips to "private" which is C++ reserved → escaped to "private_id"
        assert sanitize_lf_identifier("_private") == "private_id"


class TestTopicToPortName:
    def test_simple(self):
        assert topic_to_port_name("sensor_data") == "sensor_data"

    def test_leading_slash(self):
        assert topic_to_port_name("/sensor_data") == "sensor_data"

    def test_namespaced(self):
        assert topic_to_port_name("/robot_1/sensor_data") == "robot_1_sensor_data"

    def test_deeply_namespaced(self):
        assert topic_to_port_name("/ns1/ns2/topic") == "ns1_ns2_topic"

    def test_no_slash(self):
        assert topic_to_port_name("chatter") == "chatter"


class TestMessageTypeToLfType:
    def test_standard(self):
        assert message_type_to_lf_type("custom_interfaces::msg::SensorData") == "SensorData"

    def test_std_msgs(self):
        assert message_type_to_lf_type("std_msgs::msg::String") == "String"

    def test_with_allocator(self):
        assert message_type_to_lf_type(
            "custom_interfaces::msg::SensorData_<std::allocator<void>>"
        ) == "SensorData"

    def test_reference_system(self):
        assert message_type_to_lf_type("reference_interfaces::msg::Message4kb") == "Message4kb"


class TestCallbackToTimerName:
    def test_simple(self):
        assert callback_to_timer_name("timer_callback") == "t_timer_callback"

    def test_another(self):
        assert callback_to_timer_name("on_timer") == "t_on_timer"


class TestLaunchStemToReactorName:
    def test_simple(self):
        assert launch_stem_to_reactor_name("bringup") == "Bringup"

    def test_multi_word(self):
        assert launch_stem_to_reactor_name("pubsub_example_bringup") == "PubsubExampleBringup"

    def test_empty(self):
        assert launch_stem_to_reactor_name("") == "Main"

    def test_leading_digit_gets_prefix(self):
        # lfc rejects LF names that start with a digit.
        assert launch_stem_to_reactor_name("11_load_composable_nodes") == (
            "Launch_11LoadComposableNodes"
        )

    def test_no_valid_name_refused(self):
        with pytest.raises(ValueError, match="not a valid LF identifier"):
            launch_stem_to_reactor_name("my-launch")


class TestGeneratedPackageStem:
    def test_plain_name_lowered(self):
        assert generated_package_stem("PubsubExampleBringup") == "pubsubexamplebringup"

    def test_system_suffix_kept(self):
        assert generated_package_stem("TimerNodeSystem") == "timernodesystem"

    def test_digit_prefix_dropped(self):
        # The lf_generated_<pkg>_<name> package keeps the bare stem.
        assert generated_package_stem("Launch_11LoadComposableNodes") == (
            "11loadcomposablenodes"
        )
        assert generated_package_stem("Launch_16ComposableParamsSystem") == (
            "16composableparamssystem"
        )


class TestClassNameToReactorName:
    def test_already_pascal(self):
        assert class_name_to_reactor_name("PublisherNode") == "PublisherNode"

    def test_with_namespace(self):
        # After sanitization, :: becomes __
        assert class_name_to_reactor_name("my_ns::MyNode") == "my_ns__MyNode"


class TestDeduplicateNames:
    def test_no_dupes(self):
        assert deduplicate_names(["a", "b", "c"]) == ["a", "b", "c"]

    def test_simple_dupe(self):
        assert deduplicate_names(["a", "b", "a"]) == ["a", "b", "a_2"]

    def test_triple_dupe(self):
        assert deduplicate_names(["a", "a", "a"]) == ["a", "a_2", "a_3"]

    def test_empty(self):
        assert deduplicate_names([]) == []
