# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for colcon_lf.node_analysis.interface_parser.

Tests the parser for ROS2 .msg, .srv, and .action definition files,
and the type-string-to-file resolver.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from colcon_lf.node_analysis.interface_parser import (
    ActionDef,
    InterfaceField,
    MessageDef,
    ServiceDef,
    _parse_fields,
    _split_sections,
    fields_to_dicts,
    parse_action,
    parse_interface,
    parse_msg,
    parse_srv,
    resolve_interface_file,
)


# ---------------------------------------------------------------------------
# _parse_fields unit tests
# ---------------------------------------------------------------------------

class TestParseFields:
    def test_basic_fields(self):
        lines = ["float64 temperature", "float64 humidity"]
        fields = _parse_fields(lines)
        assert len(fields) == 2
        assert fields[0] == InterfaceField(type="float64", name="temperature")
        assert fields[1] == InterfaceField(type="float64", name="humidity")

    def test_strips_inline_comments(self):
        lines = ["float64 temperature  # degrees Celsius"]
        fields = _parse_fields(lines)
        assert len(fields) == 1
        assert fields[0].name == "temperature"

    def test_skips_blank_lines(self):
        lines = ["", "float64 x", "", "float64 y", ""]
        fields = _parse_fields(lines)
        assert len(fields) == 2

    def test_skips_separator_lines(self):
        lines = ["float64 x", "---", "float64 y"]
        fields = _parse_fields(lines)
        # --- is treated as a separator, not a field
        assert len(fields) == 2

    def test_skips_constants(self):
        lines = ["uint8 STATUS_OK = 0", "string name"]
        fields = _parse_fields(lines)
        # STATUS_OK is all-caps with =, should be skipped
        assert len(fields) == 1
        assert fields[0].name == "name"

    def test_array_type(self):
        lines = ["float64[] data"]
        fields = _parse_fields(lines)
        assert fields[0].type == "float64[]"

    def test_nested_message_type(self):
        lines = ["std_msgs/Header header"]
        fields = _parse_fields(lines)
        assert fields[0].type == "std_msgs/Header"
        assert fields[0].name == "header"


# ---------------------------------------------------------------------------
# _split_sections unit tests
# ---------------------------------------------------------------------------

class TestSplitSections:
    def test_single_section(self, tmp_path):
        p = tmp_path / "test.msg"
        p.write_text("float64 x\nfloat64 y\n")
        sections = _split_sections(p)
        assert len(sections) == 1
        assert "float64 x" in sections[0]

    def test_two_sections(self, tmp_path):
        p = tmp_path / "test.srv"
        p.write_text("float64 x\n---\nfloat64 y\n")
        sections = _split_sections(p)
        assert len(sections) == 2

    def test_three_sections(self, tmp_path):
        p = tmp_path / "test.action"
        p.write_text("float64 goal\n---\nfloat64 result\n---\nfloat64 feedback\n")
        sections = _split_sections(p)
        assert len(sections) == 3


# ---------------------------------------------------------------------------
# parse_msg
# ---------------------------------------------------------------------------

class TestParseMSG:
    def test_basic_msg(self, tmp_path):
        p = tmp_path / "SensorData.msg"
        p.write_text("float64 temperature\nfloat64 humidity\nfloat64 pressure\n")
        defn = parse_msg(p)
        assert isinstance(defn, MessageDef)
        assert len(defn.fields) == 3
        names = [f.name for f in defn.fields]
        assert names == ["temperature", "humidity", "pressure"]

    def test_msg_with_comments(self, tmp_path):
        p = tmp_path / "SensorData.msg"
        p.write_text("# Sensor readings\nfloat64 temperature  # degrees\nfloat64 humidity\n")
        defn = parse_msg(p)
        assert len(defn.fields) == 2

    def test_empty_msg(self, tmp_path):
        p = tmp_path / "Empty.msg"
        p.write_text("# no fields\n")
        defn = parse_msg(p)
        assert defn.fields == []


# ---------------------------------------------------------------------------
# parse_srv
# ---------------------------------------------------------------------------

class TestParseSRV:
    def test_basic_srv(self, tmp_path):
        p = tmp_path / "ProcessData.srv"
        p.write_text("float64[] data\nstring label\n---\nbool success\nstring message\n")
        defn = parse_srv(p)
        assert isinstance(defn, ServiceDef)
        assert len(defn.request) == 2
        assert len(defn.response) == 2
        assert defn.request[0].name == "data"
        assert defn.response[0].name == "success"

    def test_empty_response(self, tmp_path):
        p = tmp_path / "Trigger.srv"
        p.write_text("---\nbool success\n")
        defn = parse_srv(p)
        assert defn.request == []
        assert len(defn.response) == 1


# ---------------------------------------------------------------------------
# parse_action
# ---------------------------------------------------------------------------

class TestParseAction:
    def test_basic_action(self, tmp_path):
        p = tmp_path / "Analyze.action"
        p.write_text(
            "float64[] data\nstring label\n---\nbool success\nfloat64 score\n---\nfloat64 progress\n"
        )
        defn = parse_action(p)
        assert isinstance(defn, ActionDef)
        assert len(defn.goal) == 2
        assert len(defn.result) == 2
        assert len(defn.feedback) == 1
        assert defn.goal[0].name == "data"
        assert defn.result[0].name == "success"
        assert defn.feedback[0].name == "progress"

    def test_action_with_comments(self, tmp_path):
        p = tmp_path / "Nav.action"
        p.write_text(
            "# Goal\ngeometry_msgs/Point target\n---\n# Result\nbool reached\n---\n# Feedback\nfloat64 distance\n"
        )
        defn = parse_action(p)
        assert len(defn.goal) == 1
        assert len(defn.result) == 1
        assert len(defn.feedback) == 1


# ---------------------------------------------------------------------------
# resolve_interface_file
# ---------------------------------------------------------------------------

class TestResolveInterfaceFile:
    def test_resolves_msg_type(self, tmp_path):
        msg_path = tmp_path / "src" / "custom_interfaces" / "msg" / "SensorData.msg"
        msg_path.parent.mkdir(parents=True)
        msg_path.write_text("float64 temperature\n")
        result = resolve_interface_file("custom_interfaces::msg::SensorData", tmp_path)
        assert result == msg_path

    def test_resolves_srv_type(self, tmp_path):
        srv_path = tmp_path / "src" / "custom_interfaces" / "srv" / "ProcessData.srv"
        srv_path.parent.mkdir(parents=True)
        srv_path.write_text("---\nbool success\n")
        result = resolve_interface_file("custom_interfaces::srv::ProcessData", tmp_path)
        assert result == srv_path

    def test_resolves_action_type(self, tmp_path):
        act_path = tmp_path / "src" / "custom_interfaces" / "action" / "Analyze.action"
        act_path.parent.mkdir(parents=True)
        act_path.write_text("float64 goal\n---\nbool success\n---\nfloat64 progress\n")
        result = resolve_interface_file("custom_interfaces::action::Analyze", tmp_path)
        assert result == act_path

    def test_returns_none_for_missing_file(self, tmp_path):
        result = resolve_interface_file("custom_interfaces::msg::Missing", tmp_path)
        assert result is None

    def test_short_type_searches_workspace_packages(self, tmp_path):
        """Short name like 'Analyze' should find the .action file by searching packages."""
        act_path = tmp_path / "src" / "custom_interfaces" / "action" / "Analyze.action"
        act_path.parent.mkdir(parents=True)
        act_path.write_text("int32 num_samples\n---\nbool success\n---\nfloat64 progress\n")
        result = resolve_interface_file("Analyze", tmp_path)
        assert result == act_path

    def test_short_type_returns_none_when_not_found(self, tmp_path):
        (tmp_path / "src").mkdir(parents=True)
        result = resolve_interface_file("NonExistent", tmp_path)
        assert result is None

    def test_short_type_finds_msg(self, tmp_path):
        """Short name should also find .msg files."""
        msg_path = tmp_path / "src" / "my_msgs" / "msg" / "CustomData.msg"
        msg_path.parent.mkdir(parents=True)
        msg_path.write_text("float64 value\n")
        result = resolve_interface_file("CustomData", tmp_path)
        assert result == msg_path

    def test_short_type_finds_nested_monorepo(self, tmp_path):
        """Short name should find interfaces in nested mono-repo layouts."""
        act_path = tmp_path / "src" / "navigation2" / "nav2_msgs" / "action" / "Navigate.action"
        act_path.parent.mkdir(parents=True)
        act_path.write_text("float64 goal\n---\nbool success\n---\nfloat64 progress\n")
        result = resolve_interface_file("Navigate", tmp_path)
        assert result == act_path

    def test_system_package_fallback(self, tmp_path):
        """Fully-qualified system types should fall back to /opt/ros or ament_index."""
        # When not in the workspace and not on a ROS2 system, returns None
        (tmp_path / "src").mkdir(parents=True)
        result = resolve_interface_file("std_msgs::msg::String", tmp_path)
        # On a system with ROS2 installed, this would find the file;
        # otherwise it returns None. We just verify it doesn't crash.
        if result is not None:
            assert result.name == "String.msg"

    def test_strips_shared_ptr_suffix(self, tmp_path):
        msg_path = tmp_path / "src" / "custom_interfaces" / "msg" / "SensorData.msg"
        msg_path.parent.mkdir(parents=True)
        msg_path.write_text("float64 temperature\n")
        result = resolve_interface_file("custom_interfaces::msg::SensorData::SharedPtr", tmp_path)
        assert result == msg_path

    def test_strips_const_shared_ptr_suffix(self, tmp_path):
        msg_path = tmp_path / "src" / "custom_interfaces" / "msg" / "SensorData.msg"
        msg_path.parent.mkdir(parents=True)
        msg_path.write_text("float64 temperature\n")
        result = resolve_interface_file(
            "custom_interfaces::msg::SensorData::ConstSharedPtr", tmp_path
        )
        assert result == msg_path


# ---------------------------------------------------------------------------
# parse_interface (end-to-end)
# ---------------------------------------------------------------------------

class TestParseInterface:
    def test_parses_msg(self, tmp_path):
        msg_path = tmp_path / "src" / "custom_interfaces" / "msg" / "SensorData.msg"
        msg_path.parent.mkdir(parents=True)
        msg_path.write_text("float64 temperature\nfloat64 humidity\n")
        defn = parse_interface("custom_interfaces::msg::SensorData", tmp_path)
        assert isinstance(defn, MessageDef)
        assert len(defn.fields) == 2

    def test_parses_srv(self, tmp_path):
        srv_path = tmp_path / "src" / "custom_interfaces" / "srv" / "ProcessData.srv"
        srv_path.parent.mkdir(parents=True)
        srv_path.write_text("float64[] data\n---\nbool success\n")
        defn = parse_interface("custom_interfaces::srv::ProcessData", tmp_path)
        assert isinstance(defn, ServiceDef)
        assert len(defn.request) == 1
        assert len(defn.response) == 1

    def test_parses_action(self, tmp_path):
        act_path = tmp_path / "src" / "custom_interfaces" / "action" / "Analyze.action"
        act_path.parent.mkdir(parents=True)
        act_path.write_text("float64[] data\n---\nbool success\n---\nfloat64 progress\n")
        defn = parse_interface("custom_interfaces::action::Analyze", tmp_path)
        assert isinstance(defn, ActionDef)
        assert len(defn.goal) == 1

    def test_parses_idl_msg(self, tmp_path):
        """IDL message files should be parsed when no .msg equivalent exists."""
        idl_path = tmp_path / "src" / "ref_interfaces" / "msg" / "Data.idl"
        idl_path.parent.mkdir(parents=True)
        idl_path.write_text(
            "module ref_interfaces {\n"
            "  module msg {\n"
            "    struct Data {\n"
            "      uint64 timestamp;\n"
            "      float64 value;\n"
            "    };\n"
            "  };\n"
            "};\n"
        )
        defn = parse_interface("ref_interfaces::msg::Data", tmp_path)
        assert isinstance(defn, MessageDef)
        assert len(defn.fields) == 2
        assert defn.fields[0].type == "uint64"
        assert defn.fields[0].name == "timestamp"

    def test_parses_idl_nested_monorepo(self, tmp_path):
        """IDL in nested mono-repo layout (src/parent/pkg/msg/Foo.idl)."""
        idl_path = tmp_path / "src" / "parent_repo" / "my_msgs" / "msg" / "Status.idl"
        idl_path.parent.mkdir(parents=True)
        idl_path.write_text(
            "module my_msgs {\n"
            "  module msg {\n"
            "    struct Status {\n"
            "      uint32 code;\n"
            "    };\n"
            "  };\n"
            "};\n"
        )
        defn = parse_interface("my_msgs::msg::Status", tmp_path)
        assert isinstance(defn, MessageDef)
        assert len(defn.fields) == 1
        assert defn.fields[0].name == "code"

    def test_returns_none_for_unknown_type(self, tmp_path):
        defn = parse_interface("unknown::msg::Foo", tmp_path)
        assert defn is None


# ---------------------------------------------------------------------------
# fields_to_dicts
# ---------------------------------------------------------------------------

class TestFieldsToDicts:
    def test_converts_fields(self):
        fields = [InterfaceField(type="float64", name="temperature"),
                  InterfaceField(type="string", name="label")]
        result = fields_to_dicts(fields)
        assert result == [
            {"type": "float64", "name": "temperature"},
            {"type": "string", "name": "label"},
        ]

    def test_empty_list(self):
        assert fields_to_dicts([]) == []
