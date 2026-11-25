# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""End-to-end regression tests for `--after-delay`.

Before this fix, `--after-delay` was accepted by argparse and threaded into
`generate_lf_system` on every path, but never read: every emitted connection
was `after 0` regardless of the flag. These tests pin the whole chain —
flag > execution.yaml > "0" — at the layer a user actually touches.

Falsification (required before trusting these): re-break the fix by dropping
the `parse_lf_time_ns` call in `lf_generator.generate_lf_system`, and
`test_flag_is_honored` / `test_yaml_default_is_honored` must fail.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from colcon_lf.generation.lf_emitter import _ns_to_lf_time, emit_lf_system
from colcon_lf.generation.lf_generator import generate_lf_system
from ..corr_declared import generate_declared
from colcon_lf.output.serializer import deserialize_system_graph
from colcon_lf.verb.generate import DEFAULT_AFTER_DELAY, _resolve_after_delay

_FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "pubsub_system.json"
_CONNECTION = "publisher_node.corr_e0 -> subscriber_node.corr_e0"
# The pubsub edge is admitted, so d moved from the LF link into
# the correspondence edge (the drain wake, registered via register_corr_edge).
# The helpers render that d as "after <d>" so the chain stays pinned.


def _graph():
    return deserialize_system_graph(_FIXTURE.read_text())


def _connection_line(after_delay=None, after_delay_ns=None) -> str:
    kwargs = {"target_lfc": True}
    if after_delay is not None:
        kwargs["after_delay"] = after_delay
    if after_delay_ns is not None:
        kwargs["after_delay_ns"] = after_delay_ns
    system = generate_declared(_graph(), **kwargs)
    return _delay_line(system)


def _delay_line(system) -> str:
    text = emit_lf_system(system)
    lines = [ln.strip() for ln in text.splitlines() if _CONNECTION in ln]
    assert lines == [_CONNECTION], f"zero-delay corr link not found in emitted LF:\n{text}"
    (edge,) = [e for e in system.corr_edges if e.status == "admitted"]
    return f"{lines[0]} after {_ns_to_lf_time(edge.delay_ns)}"


@pytest.fixture
def staged(tmp_path: Path) -> Path:
    """A system.json laid out as lf-analyze stages it, so config/ is found."""
    sysdir = tmp_path / "bringup"
    (sysdir / "config").mkdir(parents=True)
    shutil.copy(_FIXTURE, sysdir / "system.json")
    return sysdir / "system.json"


def _write_yaml(system_json: Path, body: str) -> None:
    (system_json.parent / "config" / "execution.yaml").write_text(body)


class TestGeneratorHonorsDelay:
    """The core defect: the string form must reach the emitted connection."""

    @pytest.mark.parametrize(
        "delay,expected",
        [
            ("1 msec", "after 1 msec"),
            ("5 msec", "after 5 msec"),
            ("999 sec", "after 999 sec"),
            ("500 usec", "after 500 usec"),
            ("0", "after 0"),
        ],
    )
    def test_flag_is_honored(self, delay, expected):
        assert _connection_line(after_delay=delay).endswith(expected)

    def test_explicit_ns_wins_over_string(self):
        line = _connection_line(after_delay="5 msec", after_delay_ns=2_000_000)
        assert line.endswith("after 2 msec")

    def test_unset_falls_back_to_library_default(self):
        """No value anywhere => DEFAULT_AFTER_DELAY_NS (0)."""
        assert _connection_line().endswith("after 0")

    def test_bad_value_raises_rather_than_silently_zeroing(self):
        with pytest.raises(ValueError):
            _connection_line(after_delay="not-a-time")


class TestPerLinkPrecedence:
    """A per-link after_delay_ns in system.json outranks the global default."""

    def test_per_link_wins(self, tmp_path: Path):
        raw = json.loads(_FIXTURE.read_text())
        topics = raw["communication"]["topics"]
        assert topics, "fixture has no topic connections"
        topics[0]["after_delay_ns"] = 3_000_000
        patched = tmp_path / "system.json"
        patched.write_text(json.dumps(raw))

        graph = deserialize_system_graph(patched.read_text())
        line = _delay_line(
            generate_declared(graph, after_delay="5 msec", target_lfc=True)
        )
        assert line.endswith("after 3 msec")


class TestCliPrecedence:
    """flag > execution.yaml defaults.after_delay > DEFAULT_AFTER_DELAY."""

    def test_default_when_nothing_set(self, staged: Path):
        assert _resolve_after_delay(
            after_delay_flag=None, system_json=staged, graph=_graph(),
            exec_config_arg=None,
        ) == DEFAULT_AFTER_DELAY

    def test_yaml_default_is_honored(self, staged: Path):
        _write_yaml(staged, "schema_version: 1\ndefaults:\n  after_delay: '7 msec'\n")
        assert _resolve_after_delay(
            after_delay_flag=None, system_json=staged, graph=_graph(),
            exec_config_arg=None,
        ) == "7 msec"

    def test_flag_overrides_yaml(self, staged: Path):
        _write_yaml(staged, "schema_version: 1\ndefaults:\n  after_delay: '7 msec'\n")
        assert _resolve_after_delay(
            after_delay_flag="2 sec", system_json=staged, graph=_graph(),
            exec_config_arg=None,
        ) == "2 sec"

    def test_yaml_value_reaches_emitted_connection(self, staged: Path):
        """The whole chain, not just the resolver."""
        _write_yaml(staged, "schema_version: 1\ndefaults:\n  after_delay: '7 msec'\n")
        resolved = _resolve_after_delay(
            after_delay_flag=None, system_json=staged, graph=_graph(),
            exec_config_arg=None,
        )
        assert _connection_line(after_delay=resolved).endswith("after 7 msec")

    def test_cli_default_reaches_the_emitted_connection(self, staged: Path):
        """The CLI default `after 0` reaches the emitted connection.

        Replaced the 1 msec default. Its reason (an
        `after 0` link grants the DDS take no physical slack) no longer holds
        for an admitted edge: the subscriber waits for its sample up to the
        take cutoff, publication tag + d + T, so T is the slack, not d. A link
        that stays legacy still takes at its tag with no slack; that is the
        cost of the default, and execution.yaml or --after-delay sets a d > 0
        where it matters. The resolver test above pins the string; this pins
        that the string survives all the way into the emitted connection.
        """
        resolved = _resolve_after_delay(
            after_delay_flag=None, system_json=staged, graph=_graph(),
            exec_config_arg=None,
        )
        line = _connection_line(after_delay=resolved)
        assert line.endswith("after 0"), line


class TestYamlValidation:
    """A typo must fail at config load, not silently generate `after 0`."""

    def test_bad_time_literal_rejected(self, staged: Path):
        from colcon_lf.exec_config.model import ExecConfig

        _write_yaml(staged, "schema_version: 1\ndefaults:\n  after_delay: 'nonsense'\n")
        with pytest.raises(ValueError):
            ExecConfig.load(staged.parent / "config" / "execution.yaml", _graph())

    def test_after_delay_rejected_outside_defaults(self, staged: Path):
        """Generation-time key: not settable per-node."""
        from colcon_lf.exec_config.model import ExecConfig

        _write_yaml(
            staged,
            "schema_version: 1\nnodes:\n  publisher_node:\n    after_delay: '5 msec'\n",
        )
        with pytest.raises(ValueError):
            ExecConfig.load(staged.parent / "config" / "execution.yaml", _graph())


class TestRuntimeConfigShapeUnchanged:
    """after_delay is generation-time and must stay out of exec_config.json."""

    def test_resolved_runtime_table_excludes_after_delay(self, staged: Path):
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.exec_config.resolve import resolve

        _write_yaml(staged, "schema_version: 1\ndefaults:\n  after_delay: '7 msec'\n")
        config = ExecConfig.load(staged.parent / "config" / "execution.yaml", _graph())
        resolved = resolve(config, _graph())
        assert "after_delay" not in resolved.defaults
        for row in resolved.subscriptions:
            assert "after_delay" not in row.values
