# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Per-module detection tests on synthetic graphs (00_design.md section 5)."""

from __future__ import annotations

from colcon_lf.features.diagnostics import DetectionContext, Outcome
from colcon_lf.features.modules import (
    actions,
    best_effort_input,
    callback_groups,
    components,
    dynamic_topic,
    lifecycle,
    message_filters,
    message_filters_sync,
    parameters,
    pubsub,
    qos_unknown,
    services,
    shallow_queue,
    timers,
)
from colcon_lf.node_analysis.models import MessageFiltersInfo, MessageFiltersSyncInfo
from tests.features.conftest import make_graph, make_node, make_pub, make_sub


class TestQosUnknown:
    def test_fires_on_unparsed_reliability(self):
        graph = make_graph({"/n": make_node("n", subscribers=[make_sub("t")])})
        diags = qos_unknown.detect(DetectionContext(graph=graph))
        assert len(diags) == 1
        assert diags[0].code == "RCLLF-H000"
        assert diags[0].node == "/n"
        assert diags[0].topic == "t"

    def test_silent_when_reliability_materialized(self):
        graph = make_graph(
            {"/n": make_node("n", subscribers=[make_sub("t", qos_reliability="reliable")])}
        )
        diags = qos_unknown.detect(DetectionContext(graph=graph))
        assert diags == []

    def test_fires_on_unparsed_publisher_reliability(self):
        graph = make_graph({"/n": make_node("n", publishers=[make_pub("t")])})
        diags = qos_unknown.detect(DetectionContext(graph=graph))
        assert len(diags) == 1
        assert diags[0].code == "RCLLF-H000"
        assert diags[0].node == "/n"
        assert diags[0].topic == "t"

    def test_silent_when_publisher_reliability_materialized(self):
        graph = make_graph(
            {"/n": make_node("n", publishers=[make_pub("t", qos_reliability="reliable")])}
        )
        diags = qos_unknown.detect(DetectionContext(graph=graph))
        assert diags == []

    def test_playground_case_is_a_clean_pass(self):
        """pubsub_example's bare-int QoS materializes to 'reliable' — no H000 fires."""
        case = qos_unknown.MODULE.playground
        assert case is not None
        assert case.example == "pubsub_example/bringup"
        assert case.expected_outcome == Outcome.PASS
        assert case.expected_codes == ()


class TestBestEffortInput:
    def test_fires_on_best_effort(self):
        graph = make_graph(
            {"/n": make_node("n", subscribers=[make_sub("t", qos_reliability="best_effort")])}
        )
        diags = best_effort_input.detect(DetectionContext(graph=graph))
        assert len(diags) == 1
        assert diags[0].code == "RCLLF-H001"
        assert diags[0].topic == "t"

    def test_silent_on_reliable(self):
        graph = make_graph(
            {"/n": make_node("n", subscribers=[make_sub("t", qos_reliability="reliable")])}
        )
        diags = best_effort_input.detect(DetectionContext(graph=graph))
        assert diags == []

    def test_names_the_remapped_topic(self):
        """The launch file remaps the code-level ``t``: the text names the topic
        the subscription runs on, ``topic`` keeps the edge key (batch BK)."""
        from colcon_lf.features.diagnostics import Severity
        from colcon_lf.launch_analysis.models import LaunchNodeAction

        graph = make_graph(
            {"/ns/n": make_node("n", subscribers=[make_sub("t", qos_reliability="best_effort")])}
        )
        graph.launch_graph.all_nodes.append(LaunchNodeAction(
            package="test_pkg", executable="n", name="n", namespace="/ns",
            remappings=[("t", "/radar/range")]))
        diags = best_effort_input.detect(DetectionContext(graph=graph))
        assert [(d.node, d.topic) for d in diags] == [("/ns/n", "t")]
        assert diags[0].message.startswith(
            "subscription '/radar/range' on node '/ns/n': best_effort QoS reliability"
        ), diags[0].message

        class Edge:  # an admitted correspondence edge into the subscription
            sub_node, sub_topic, status = "/ns/n", "t", "admitted"

            def label(self):
                return "/radar/range (/p -> /ns/n)"

        [(sev, text)] = best_effort_input.h001_by_edge_status(diags, [Edge()], graph=graph)
        assert sev == Severity.INFO
        assert text.startswith("subscription '/radar/range' on node '/ns/n': best_effort"), text

    def test_unremapped_topic_resolves_in_the_node_namespace(self):
        from colcon_lf.launch_analysis.models import LaunchNodeAction

        graph = make_graph(
            {"/ns/n": make_node("n", subscribers=[make_sub("t", qos_reliability="best_effort")])}
        )
        graph.launch_graph.all_nodes.append(LaunchNodeAction(
            package="test_pkg", executable="n", name="n", namespace="/ns"))
        [d] = best_effort_input.detect(DetectionContext(graph=graph))
        assert d.message.startswith("subscription '/ns/t' on node '/ns/n'"), d.message

    def test_silent_on_unknown(self):
        graph = make_graph({"/n": make_node("n", subscribers=[make_sub("t")])})
        diags = best_effort_input.detect(DetectionContext(graph=graph))
        assert diags == []

    def test_playground_case_fires_h001_as_warn(self):
        case = best_effort_input.MODULE.playground
        assert case is not None
        assert case.example == "qos_hazard_example/best_effort_subscriber_node"
        assert case.expected_outcome == Outcome.WARN
        assert case.expected_codes == ("RCLLF-H001",)


class TestShallowQueue:
    def test_fires_on_depth_1_with_in_model_publisher(self):
        graph = make_graph(
            {
                "/pub": make_node("pub", publishers=[make_pub("t")]),
                "/sub": make_node("sub", subscribers=[make_sub("t", qos_depth=1)]),
            }
        )
        diags = shallow_queue.detect(DetectionContext(graph=graph))
        assert len(diags) == 1
        assert diags[0].code == "RCLLF-H002"
        assert diags[0].node == "/sub"

    def test_silent_when_no_publisher_in_model(self):
        graph = make_graph({"/sub": make_node("sub", subscribers=[make_sub("t", qos_depth=1)])})
        diags = shallow_queue.detect(DetectionContext(graph=graph))
        assert diags == []

    def test_silent_when_depth_not_1(self):
        graph = make_graph(
            {
                "/pub": make_node("pub", publishers=[make_pub("t")]),
                "/sub": make_node("sub", subscribers=[make_sub("t", qos_depth=10)]),
            }
        )
        diags = shallow_queue.detect(DetectionContext(graph=graph))
        assert diags == []

    def test_slash_tolerant_topic_join(self):
        """Publisher topic '/t' and subscriber topic 't' must still join (M2 canonicalization)."""
        graph = make_graph(
            {
                "/pub": make_node("pub", publishers=[make_pub("/t")]),
                "/sub": make_node("sub", subscribers=[make_sub("t", qos_depth=1)]),
            }
        )
        diags = shallow_queue.detect(DetectionContext(graph=graph))
        assert len(diags) == 1

    def test_playground_case_uses_the_isolating_combo_launch(self):
        """The playground case uses the isolating combo launch.

        At ``qos_hazard_example/bringup``, message_filters_subscriber_node's
        RCLLF-H003 (ERROR) co-fires and makes the generate hazard-gate refuse the
        whole launch (observed outcome REJECT, masking this module's own WARN
        level). The purpose-built
        ``shallow_queue_with_publisher`` launch starts only
        hazard_publisher_node + shallow_queue_subscriber_node, so H002 fires in
        isolation and WARN is the real end-to-end outcome.
        """
        case = shallow_queue.MODULE.playground
        assert case is not None
        assert case.example == "qos_hazard_example/shallow_queue_with_publisher"
        assert case.expected_outcome == Outcome.WARN
        assert case.expected_codes == ("RCLLF-H002",)


class TestMessageFiltersSync:
    def test_fires_when_message_filters_in_detected_includes(self):
        graph = make_graph(
            {"/n": make_node("n", detected_includes=["message_filters"])}
        )
        diags = message_filters_sync.detect(DetectionContext(graph=graph))
        assert len(diags) == 1
        assert diags[0].code == "RCLLF-H003"
        assert diags[0].node == "/n"
        assert diags[0].topic is None

    def test_silent_without_message_filters(self):
        graph = make_graph({"/n": make_node("n")})
        diags = message_filters_sync.detect(DetectionContext(graph=graph))
        assert diags == []

    def test_admitted_synchronizer_is_silent(self):
        """A two-input sync with no unsupported use raises nothing."""
        node = make_node("n", detected_includes=["message_filters"])
        node.message_filters = MessageFiltersInfo(synchronizers=[MessageFiltersSyncInfo(
            member_variable="sync_", policy="ApproximateTime", queue_size=10,
            inputs=["sub_a_", "sub_b_"], callback="sync_callback")])
        graph = make_graph({"/n": node})
        assert message_filters_sync.detect(DetectionContext(graph=graph)) == []

    def test_unsupported_use_names_the_reason(self):
        node = make_node("n", detected_includes=["message_filters"])
        node.message_filters = MessageFiltersInfo(
            unsupported=["message_filters::Cache is not supported"])
        graph = make_graph({"/n": node})
        diags = message_filters_sync.detect(DetectionContext(graph=graph))
        assert [d.code for d in diags] == ["RCLLF-H003"]
        assert "message_filters::Cache is not supported" in diags[0].message

    def test_playground_case_fires_h003_as_reject(self):
        case = message_filters_sync.MODULE.playground
        assert case is not None
        assert case.example == "qos_hazard_example/message_filters_cache_node"
        assert case.expected_outcome == Outcome.REJECT
        assert case.expected_codes == ("RCLLF-H003",)


class TestDynamicTopic:
    """RCLLF-H004 — one diagnostic per flagged endpoint."""

    def test_fires_on_dynamic_publisher(self):
        graph = make_graph(
            {"/n": make_node("n", publishers=[make_pub("", topic_dynamic=True)])}
        )
        diags = dynamic_topic.detect(DetectionContext(graph=graph))
        assert len(diags) == 1
        assert diags[0].code == "RCLLF-H004"
        assert diags[0].node == "/n"
        # An unresolved name serializes as None, not "" — a consumer must not
        # be able to mistake it for a real topic called "".
        assert diags[0].topic is None
        assert "may be missing from the extracted graph" in diags[0].message

    def test_fires_on_dynamic_subscriber(self):
        graph = make_graph(
            {"/n": make_node("n", subscribers=[make_sub("", topic_dynamic=True)])}
        )
        diags = dynamic_topic.detect(DetectionContext(graph=graph))
        assert len(diags) == 1
        assert diags[0].code == "RCLLF-H004"
        assert "subscription" in diags[0].message

    def test_reports_best_effort_partial_name_when_one_is_present(self):
        """A `${var}` placeholder topic is kept as a hint but still flagged."""
        graph = make_graph(
            {"/n": make_node("n", publishers=[make_pub("${topic_name}", topic_dynamic=True)])}
        )
        diags = dynamic_topic.detect(DetectionContext(graph=graph))
        assert len(diags) == 1
        assert diags[0].topic == "${topic_name}"
        assert "${topic_name}" in diags[0].message

    def test_silent_on_static_topics(self):
        graph = make_graph(
            {
                "/pub": make_node("pub", publishers=[make_pub("t")]),
                "/sub": make_node("sub", subscribers=[make_sub("t")]),
            }
        )
        assert dynamic_topic.detect(DetectionContext(graph=graph)) == []

    def test_silent_when_a_placeholder_was_resolved_downstream(self):
        """`graph.hierarchical._resolve_topic_placeholders` fills `${param}` from a

        launch/YAML overlay without clearing the extraction-time flag. The name
        *is* known in the finished graph, so H004 must not fire — otherwise
        every parameterized-topic node would carry a permanent false warning.
        """
        graph = make_graph(
            {
                "/n": make_node(
                    "n",
                    publishers=[make_pub("cmd_vel", topic_dynamic=True)],
                    subscribers=[make_sub("/scan", topic_dynamic=True)],
                ),
            }
        )
        assert dynamic_topic.detect(DetectionContext(graph=graph)) == []

    def test_one_diagnostic_per_endpoint_not_per_node(self):
        graph = make_graph(
            {
                "/n": make_node(
                    "n",
                    publishers=[
                        make_pub("", topic_dynamic=True),
                        make_pub("status"),
                    ],
                    subscribers=[make_sub("", topic_dynamic=True)],
                ),
            }
        )
        diags = dynamic_topic.detect(DetectionContext(graph=graph))
        assert len(diags) == 2

    def test_playground_case_points_at_the_fixture_package(self):
        case = dynamic_topic.MODULE.playground
        assert case is not None
        assert case.example == "dynamic_topic_example/bringup"
        assert case.expected_outcome == Outcome.WARN
        assert case.expected_codes == ("RCLLF-H004",)


# Phase 7: the SUPPORTED core-feature modules own no diagnostic codes.
# `detect` is trivially `[]` (module presence is a structural fact from the
# extracted graph/launch, not a diagnostic) — these tests confirm that
# invariant and that each module's registry metadata/playground case is
# well-formed, matching the shared rationale documented in timers.py.
_CORE_MODULES = {
    "core.timers": timers,
    "core.pubsub": pubsub,
    "core.message_filters": message_filters,
    "core.services": services,
    "core.actions": actions,
    "core.callback_groups": callback_groups,
    "core.parameters": parameters,
    "core.lifecycle": lifecycle,
    "core.components": components,
}


class TestSupportedCoreModules:
    def test_detect_is_always_empty_on_empty_graph(self):
        graph = make_graph({})
        for module in _CORE_MODULES.values():
            assert module.MODULE.detect(DetectionContext(graph=graph)) == []

    def test_detect_is_always_empty_even_with_hazard_shaped_graph(self):
        """Presence-only modules never emit diagnostics, even alongside real hazards."""
        graph = make_graph(
            {
                "/n": make_node(
                    "n",
                    subscribers=[make_sub("t", qos_reliability="best_effort", qos_depth=1)],
                    publishers=[make_pub("t")],
                    detected_includes=["message_filters"],
                )
            }
        )
        for module in _CORE_MODULES.values():
            assert module.MODULE.detect(DetectionContext(graph=graph)) == []

    # core.actions' playground example (action_example/bringup) co-fires
    # RCLLF-H006 (ActionClientNode.cancel_timer_ is created
    # outside its constructor) on the SAME launch file. core.actions itself
    # still owns no codes and its own detect() is still always [] (covered
    # above), so it stays in _CORE_MODULES; only its playground outcome is
    # no longer PASS, per the harness's own "co-firing modules must not
    # break each other's playground case" contract (test_playground_cases.py).
    _WARN_DUE_TO_COFIRING = {"core.actions"}

    def test_each_core_module_owns_no_codes_and_has_pass_playground_case(self):
        for feature_id, module in _CORE_MODULES.items():
            assert module.MODULE.feature_id == feature_id
            assert module.MODULE.codes == ()
            assert module.MODULE.severity == {}
            case = module.MODULE.playground
            assert case is not None
            expected = (
                Outcome.WARN
                if feature_id in self._WARN_DUE_TO_COFIRING
                else Outcome.PASS
            )
            assert case.expected_outcome == expected
            assert case.expected_codes == ()
            assert "/" in case.example  # "<package>/<launch-stem>" shape
