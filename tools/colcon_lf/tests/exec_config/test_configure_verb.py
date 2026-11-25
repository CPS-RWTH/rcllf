# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for colcon_lf.verb.configure (colcon lf-configure).

Unit tests (no fixture files): verb structure, add_arguments.
Integration tests: configure_one against real system.json fixtures, including
the mandatory namespaced round-trip (system_bringup) and the class-conflict
fixture (launch_features_example/13_group_action_features — two FeatureNodes
both subscribing feature_input; system_bringup's duplicate TimerNode has zero
subscriptions and cannot exercise the conflict path).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

colcon_core = pytest.importorskip(
    "colcon_core.verb", reason="colcon_core not available"
)

_REPO_ROOT = Path(__file__).resolve().parents[4]
_SYSTEM_BRINGUP_JSON = (
    _REPO_ROOT / "packages" / "lf-analysis" / "system_bringup" / "system_bringup"
    / "system.json"
)
# 03_conditions starts only primary_node (its secondary_node
# is behind a false condition), so the two-instance fixture is group 13.
_TWO_FEATURE_NODES_JSON = (
    _REPO_ROOT / "packages" / "lf-analysis" / "launch_features_example"
    / "13_group_action_features" / "system.json"
)


class TestConfigureVerbStructure:
    def test_instantiation(self):
        from colcon_lf.verb.configure import ConfigureVerb

        verb = ConfigureVerb()
        assert hasattr(verb, "add_arguments")
        assert hasattr(verb, "main")

    def test_add_arguments_all_optional(self):
        from colcon_lf.verb.configure import ConfigureVerb

        verb = ConfigureVerb()
        parser = argparse.ArgumentParser()
        verb.add_arguments(parser=parser)
        args = parser.parse_args([])
        assert args.show is False
        assert args.interactive is False


class TestConfigureOneNamespacedRoundTrip:
    """The mandatory system_bringup namespaced end-to-end test."""

    def test_scaffold_validate_and_resolve(self, tmp_path):
        if not _SYSTEM_BRINGUP_JSON.is_file():
            pytest.skip(f"system.json not found: {_SYSTEM_BRINGUP_JSON}")

        import shutil

        from colcon_lf.verb.configure import configure_one

        # Work on a copy so we never mutate the tracked fixture directory.
        work_dir = tmp_path / "system_bringup"
        work_dir.mkdir()
        system_json = work_dir / "system.json"
        shutil.copy(_SYSTEM_BRINGUP_JSON, system_json)

        rc = configure_one(system_json=system_json)
        assert rc == 0

        config_dir = work_dir / "config"
        assert (config_dir / "execution.reference.yaml").is_file()
        assert (config_dir / "execution.yaml").is_file()

        # Reference file mentions the namespaced node and relative topic key.
        ref_content = (config_dir / "execution.reference.yaml").read_text()
        assert "/robot_1/subscriber_node" in ref_content
        assert "sensor_data" in ref_content
        # And the non-namespaced bare node key (exercises the M2 key-form fix).
        # sim_timer is not in system.json: use_sim defaults to false.
        assert "service_server" in ref_content

    def test_per_subscription_policy_round_trips_to_exec_config_json(self, tmp_path):
        if not _SYSTEM_BRINGUP_JSON.is_file():
            pytest.skip(f"system.json not found: {_SYSTEM_BRINGUP_JSON}")

        import shutil

        from colcon_lf.output.serializer import deserialize_system_graph
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.exec_config.resolve import resolve

        work_dir = tmp_path / "system_bringup"
        work_dir.mkdir()
        system_json = work_dir / "system.json"
        shutil.copy(_SYSTEM_BRINGUP_JSON, system_json)
        config_dir = work_dir / "config"
        config_dir.mkdir()

        # User sets a per-subscription policy using the namespaced (slashed)
        # node key and a bare (relative) topic key, as the model actually
        # stores it.
        (config_dir / "execution.yaml").write_text(
            "schema_version: 1\n"
            "subscriptions:\n"
            "  /robot_1/subscriber_node:\n"
            "    sensor_data:\n"
            "      take_mode: block\n",
            encoding="utf-8",
        )

        graph = deserialize_system_graph(system_json.read_text())
        config = ExecConfig.load(config_dir / "execution.yaml", graph)
        resolved = resolve(config, graph)

        matching = [
            row for row in resolved.subscriptions
            if row.node_class == "SubscriberNode" and row.topic == "sensor_data"
        ]
        assert len(matching) == 1
        assert matching[0].values["take_mode"] == "block"


class TestConfigureOnePerInstance:
    """launch_features_example/13_group_action_features: two FeatureNodes, both
    subscribing feature_input — the real-model fixture for per-instance policy
    (system_bringup's duplicate class has zero subscriptions).

    Since ``10_per_instance_rids_design.md`` the fold is node key -> binding key,
    so these two instances produce two independently-configurable rows instead of
    the old class-conflict ValueError.
    """

    def test_identical_values_accepted(self, tmp_path):
        if not _TWO_FEATURE_NODES_JSON.is_file():
            pytest.skip(f"system.json not found: {_TWO_FEATURE_NODES_JSON}")

        import shutil

        from colcon_lf.output.serializer import deserialize_system_graph
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.exec_config.resolve import resolve

        work_dir = tmp_path / "13_group_action_features"
        work_dir.mkdir()
        system_json = work_dir / "system.json"
        shutil.copy(_TWO_FEATURE_NODES_JSON, system_json)

        graph = deserialize_system_graph(system_json.read_text())
        config = ExecConfig.empty()
        resolved = resolve(config, graph)
        # Two FeatureNode instances → two per-instance binding keys, no error.
        feature_rows = [
            row for row in resolved.subscriptions
            if row.node_class.startswith("FeatureNode")
        ]
        assert len(feature_rows) == 2
        assert all(row.node_class.startswith("FeatureNode__") for row in feature_rows)

    def test_differing_take_mode_yields_two_rows(self, tmp_path):
        if not _TWO_FEATURE_NODES_JSON.is_file():
            pytest.skip(f"system.json not found: {_TWO_FEATURE_NODES_JSON}")

        import shutil

        from colcon_lf.output.serializer import deserialize_system_graph
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.exec_config.resolve import resolve

        work_dir = tmp_path / "13_group_action_features"
        work_dir.mkdir()
        system_json = work_dir / "system.json"
        shutil.copy(_TWO_FEATURE_NODES_JSON, system_json)
        config_dir = work_dir / "config"
        config_dir.mkdir()

        graph = deserialize_system_graph(system_json.read_text())
        node_names = [
            name for name, info in graph.nodes.items() if info.class_name == "FeatureNode"
        ]
        assert len(node_names) == 2, "fixture assumption: two FeatureNode instances"

        (config_dir / "execution.yaml").write_text(
            "schema_version: 1\n"
            "nodes:\n"
            f"  {node_names[0]}:\n"
            "    take_mode: block\n"
            f"  {node_names[1]}:\n"
            "    take_mode: skip\n",
            encoding="utf-8",
        )
        config = ExecConfig.load(config_dir / "execution.yaml", graph)
        resolved = resolve(config, graph)

        # One row per instance, each carrying its own take_mode — the whole
        # point of the per-instance work (design note section 5).
        feature_rows = {
            row.node_class: row for row in resolved.subscriptions
            if row.node_class.startswith("FeatureNode__")
        }
        assert len(feature_rows) == 2
        modes = {row.node_keys[0]: row.values["take_mode"] for row in feature_rows.values()}
        assert modes == {node_names[0]: "block", node_names[1]: "skip"}


class TestConfigureOneShow:
    def test_show_prints_resolved_table(self, tmp_path, capsys):
        if not _SYSTEM_BRINGUP_JSON.is_file():
            pytest.skip(f"system.json not found: {_SYSTEM_BRINGUP_JSON}")

        import shutil

        from colcon_lf.verb.configure import configure_one

        work_dir = tmp_path / "system_bringup"
        work_dir.mkdir()
        system_json = work_dir / "system.json"
        shutil.copy(_SYSTEM_BRINGUP_JSON, system_json)

        rc = configure_one(system_json=system_json, show=True)
        assert rc == 0
        captured = capsys.readouterr()
        assert "Resolved effective policy" in captured.out
        assert "SubscriberNode" in captured.out


class TestShowIsReadOnly:
    def test_show_writes_no_file(self, tmp_path, capsys):
        # `colcon lf-configure --show` over a workspace once created an
        # execution.yaml "(created)" in every analysis: a show with side effects.
        # Recorded system.json, so the test runs without a prior lf-analyze.
        import shutil

        from colcon_lf.verb.configure import configure_one

        work_dir = tmp_path / "bringup"
        work_dir.mkdir()
        system_json = work_dir / "system.json"
        shutil.copy(Path(__file__).parent / "fixtures" / "pubsub_bringup.system.json",
                    system_json)

        assert configure_one(system_json=system_json, show=True) == 0
        out = capsys.readouterr().out
        assert not (work_dir / "config").exists(), (
            f"--show wrote {sorted(p.name for p in (work_dir / 'config').iterdir())}")
        assert "(created)" not in out
        assert "no execution.yaml yet" in out
        assert "Resolved effective policy" in out

        # Without --show the scaffold still writes both files.
        assert configure_one(system_json=system_json) == 0
        assert (work_dir / "config" / "execution.yaml").is_file()
        assert (work_dir / "config" / "execution.reference.yaml").is_file()

    def test_show_with_scaffold_only_is_rejected_and_writes_nothing(self, tmp_path):
        # --show promises no file; a scaffold-only run only writes files. The
        # combination once silently wrote the config files under --show.
        import shutil

        from colcon_lf.verb.configure import configure_one

        work_dir = tmp_path / "bringup"
        work_dir.mkdir()
        system_json = work_dir / "system.json"
        shutil.copy(Path(__file__).parent / "fixtures" / "pubsub_bringup.system.json",
                    system_json)

        with pytest.raises(ValueError, match="show=True and scaffold_only=True contradict"):
            configure_one(system_json=system_json, show=True, scaffold_only=True)
        assert not (work_dir / "config").exists()

    def test_show_help_names_the_interactive_exception(self):
        from colcon_lf.verb.configure import ConfigureVerb

        parser = argparse.ArgumentParser()
        ConfigureVerb().add_arguments(parser=parser)
        show = next(a for a in parser._actions if "--show" in a.option_strings)
        assert "Read-only: writes no file" in show.help
        assert "With --interactive" in show.help and "execution.yaml" in show.help
