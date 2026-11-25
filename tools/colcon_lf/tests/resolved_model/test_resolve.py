# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for resolved_model.resolve() against real, analyzed system.json files
(T3/T4).

Golden values recorded from a real `colcon lf-analyze` (see
`tests/test_hierarchical_graph.py::TestParameterPrecedenceFixture` and the
plan's Output verification section for the exact commands) and cross-checked
against a live `ros2 node info` / `ros2 param get` on the running system
(2026-09-24, see plan/journal).
"""

from __future__ import annotations

import pytest

from ..ros_env import requires_ros_headers

pytestmark = requires_ros_headers("resolved_model.resolve() against real analyses")


@pytest.fixture(scope="module")
def analyzed(packages_src, package_dir, analyzed_graph, tmp_path_factory):
    """Analyze the fixtures this module needs once, returning system.json paths."""
    from colcon_lf.output.writer import write

    out = tmp_path_factory.mktemp("resolved-model-analysis")
    paths = {}

    launch_01 = package_dir("launch_features_example") / "launch" / "01_node_namespace_remap.launch.py"
    graph_01 = analyzed_graph(launch_01)
    write(graph_01, out / "01")
    paths["01"] = out / "01" / "system.json"

    launch_prec = packages_src / "parameter_example" / "launch" / "precedence.launch.py"
    graph_prec = analyzed_graph(launch_prec)
    write(graph_prec, out / "precedence")
    paths["precedence"] = out / "precedence" / "system.json"

    launch_svc = packages_src / "service_example" / "launch" / "bringup.launch.py"
    graph_svc = analyzed_graph(launch_svc)
    write(graph_svc, out / "service")
    paths["service"] = out / "service" / "system.json"

    launch_sentry = package_dir("sentry_pipeline") / "launch" / "sentry_pipeline.launch.py"
    graph_sentry = analyzed_graph(launch_sentry)
    write(graph_sentry, out / "sentry")
    paths["sentry"] = out / "sentry" / "system.json"

    return paths


def _resolve(system_json, packages_root, exec_config=None, after_delay=None):
    from colcon_lf.exec_config.model import ExecConfig
    from colcon_lf.resolved_model.resolve import resolve

    return resolve(
        system_json, exec_config=exec_config or ExecConfig.empty(), workspace=packages_root,
        after_delay=after_delay,
    )


class TestNamespaceAndRemap:
    """Golden: live `ros2 node info /ns1/ns_remap_node` + `ros2 topic list`
    (2026-09-24): subscriber /alt_input, publisher /ns1/feature_output.
    """

    def test_resolved_node_key_is_launch_fqn(self, analyzed, packages_root):
        model = _resolve(analyzed["01"], packages_root)
        assert any(r.contract_id == "node:/ns1/ns_remap_node#fqn" for r in model.records)

    def test_sub_topic_matches_live_system(self, analyzed, packages_root):
        model = _resolve(analyzed["01"], packages_root)
        rec = next(
            r for r in model.records
            if r.contract_id == "sub:/ns1/ns_remap_node/feature_input#topic"
        )
        assert rec.value == "/alt_input"
        assert rec.provenance == "extracted"

    def test_pub_topic_matches_live_system(self, analyzed, packages_root):
        model = _resolve(analyzed["01"], packages_root)
        rec = next(
            r for r in model.records
            if r.contract_id == "pub:/ns1/ns_remap_node/feature_output#topic"
        )
        assert rec.value == "/ns1/feature_output"


class TestParameterPrecedenceGoldens:
    """T4a goldens (live `ros2 param get`, see test_hierarchical_graph.py)."""

    @pytest.mark.parametrize("node,name,expected", [
        ("node_a", "robot_name", "node_a_from_params_file"),
        ("node_a", "max_speed", 5.0),
        ("node_a", "update_rate", 42),
        ("node_a", "debug_mode", False),
        ("node_b", "robot_name", "node_b_from_params_file"),
        ("node_b", "max_speed", 9.0),
        ("node_b", "update_rate", 13),
        ("node_b", "debug_mode", True),
    ])
    def test_param_value_matches_live_ros2_param_get(
        self, analyzed, packages_root, node, name, expected
    ):
        model = _resolve(analyzed["precedence"], packages_root)
        rec = next(r for r in model.records if r.contract_id == f"param:{node}/{name}#value")
        assert rec.value == expected
        assert rec.provenance == "explicitly_configured"


class TestSentryTimers:
    def test_unpatched_timers_unresolved_with_r001(self, analyzed, packages_root):
        model = _resolve(analyzed["sentry"], packages_root)
        period_records = [r for r in model.records if r.property == "period"]
        assert len(period_records) == 4
        assert all(r.provenance == "unresolved" and r.reason == "RCLLF-R001" for r in period_records)
        assert model.claim_status == "partial"

    def test_p3_override_resolves_one_timer_and_changes_hash(self, analyzed, packages_root):
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.output.serializer import deserialize_system_graph

        graph = deserialize_system_graph(analyzed["sentry"].read_text())
        cfg = ExecConfig.from_dict(
            {"nodes": {"imu_node": {"timers": {"timer_callback": {"period": "5 msec"}}}}},
            graph, source="execution.yaml",
        )
        before = _resolve(analyzed["sentry"], packages_root)
        after = _resolve(analyzed["sentry"], packages_root, exec_config=cfg)
        assert before.model_hash != after.model_hash

        imu = next(
            r for r in after.records
            if r.contract_id == "timer:imu_node/timer_callback#period"
        )
        assert imu.value == 5.0
        assert imu.provenance == "explicitly_configured"
        others_unresolved = [
            r for r in after.records
            if r.property == "period" and r.subject["node"] != "imu_node"
        ]
        assert len(others_unresolved) == 3
        assert all(r.provenance == "unresolved" for r in others_unresolved)


class TestLinkAfterDelays:
    """The after-delay of every link, as ``_build_connections`` puts
    it on the generated LF. The default is
    ``after 0`` (``verb.generate.DEFAULT_AFTER_DELAY``); the recorded d = 0 runs read
    back d = 0 on every link."""

    def test_service_link_default_is_after_zero(self, analyzed, packages_root):
        model = _resolve(analyzed["service"], packages_root)
        links = [r for r in model.records if r.subject.get("kind") == "link"]
        assert [(r.contract_id, r.value, r.provenance, r.evidence) for r in links] == [(
            "link:/service_example/service_client->/service_example/service_server"
            "/service_example/process_data#after_delay",
            0, "extracted", "lf-generate default")]
        assert links[0].subject["link"] == "service"

    def test_flag_beats_execution_yaml_default(self, analyzed, packages_root):
        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.output.serializer import deserialize_system_graph

        graph = deserialize_system_graph(analyzed["sentry"].read_text())
        cfg = ExecConfig.from_dict({"defaults": {"after_delay": "2 msec"}}, graph,
                                   source="execution.yaml")
        base = _resolve(analyzed["sentry"], packages_root)
        yaml_ = _resolve(analyzed["sentry"], packages_root, exec_config=cfg)
        flag = _resolve(analyzed["sentry"], packages_root, exec_config=cfg, after_delay="5 msec")

        def topic_links(model):
            return {r.contract_id: (r.value, r.provenance, r.evidence)
                    for r in model.records if r.subject.get("link") == "topic"}

        n_pairs = sum(len(t.publishers) * len(t.subscribers)
                      for t in graph.communication.topics)
        assert n_pairs and len(topic_links(base)) == n_pairs
        assert set(topic_links(base).values()) == {(0, "extracted", "lf-generate default")}
        assert set(topic_links(yaml_).values()) == {
            (2_000_000, "explicitly_configured", "execution.yaml:defaults.after_delay")}
        assert set(topic_links(flag).values()) == {
            (5_000_000, "explicitly_configured", "lf-generate --after-delay")}
        assert len({base.model_hash, yaml_.model_hash, flag.model_hash}) == 3
