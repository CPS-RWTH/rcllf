# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the feature-module registry invariants and run_detection wiring."""

from __future__ import annotations

import pytest

from colcon_lf.features.diagnostics import (
    DetectionContext,
    Diagnostic,
    Outcome,
    PlaygroundCase,
    Severity,
    Support,
)
from colcon_lf.features.registry import (
    REGISTRY,
    FeatureModule,
    run_detection,
    severity_for,
)
from tests.features.conftest import make_graph, make_node, make_sub

# Phase 7: 4 QoS-hazard modules (H000-H003) + 8 SUPPORTED no-hazard core
# modules (timers/pubsub/services/actions/callback_groups/parameters/
# lifecycle/components) + 1 extraction-hardening module (H004) +
# 1 resolved-model gate module (R001-R003) + 1 lifecycle-chaining
# module (H005) + 1 out-of-scope-timer module (H006)
# + 1 unmodeled-callback module (H007) + 1 ROS-time sleep module
# (H008) + 1 blocking service client module (H009)
# + 1 BNA synchronizer module (H010 info)
# + 1 thread action server module (H011)
# + 1 intra-process module (H017).
EXPECTED_FEATURE_IDS = {
    "qos.qos_unknown",
    "qos.best_effort_input",
    "qos.shallow_queue",
    "qos.message_filters_sync",
    "topics.dynamic_name",
    "lifecycle.base_call_chaining",
    "timers.created_outside_scope",
    "callbacks.unmodeled_form",
    "clock.ros_time_sleep",
    "services.blocking_client",
    "sync.bna",
    "actions.thread_server",
    "build.compile_variant_ambiguous",
    "endpoints.created_outside_scope",
    "inheritance.template_base_endpoints",
    "main.work_outside_callbacks",
    "ipc.intra_process",
    "services.request_from_service_callback",
    "core.timers",
    "core.pubsub",
    "core.message_filters",
    "core.services",
    "core.actions",
    "core.callback_groups",
    "core.parameters",
    "core.lifecycle",
    "core.components",
    "core.correspondence",
    "resolved_model.gate",
}

# Modules whose codes never fire via the standard analyze-time detect() scan:
# they gate resolved_model.resolve()'s records at generate
# time instead (verb/generate.py::_check_resolved_model_gate), so they
# cannot have a playground case (the harness runs analyze, then checks
# diagnostics — there is no diagnostic to check here).
NO_PLAYGROUND_MODULES = {
    "resolved_model.gate",
    # Fixture-driven (tests/node_analysis/test_r5fix_c.py): no packages/src example triggers them.
    "build.compile_variant_ambiguous",
    "endpoints.created_outside_scope",
    "inheritance.template_base_endpoints",
    "main.work_outside_callbacks",
    # Fixture-driven (tests/features/test_request_from_service_callback.py): no packages/src example sends from a handler.
    "services.request_from_service_callback",
}


class TestRegistryPopulated:
    def test_all_hazard_code_modules_registered(self):
        codes = {code for module in REGISTRY.values() for code in module.codes}
        assert codes == {
            "RCLLF-H000",
            "RCLLF-H001",
            "RCLLF-H002",
            "RCLLF-H003",
            "RCLLF-H004",
            "RCLLF-H005",
            "RCLLF-H006",
            "RCLLF-H007",
            "RCLLF-H008",
            "RCLLF-H009",
            "RCLLF-H010",
            "RCLLF-H011",
            "RCLLF-H012",
            "RCLLF-H013",
            "RCLLF-H014",
            "RCLLF-H015",
            "RCLLF-H016",
            "RCLLF-H017",
            "RCLLF-R001",
            "RCLLF-R002",
            "RCLLF-R003",
        }

    def test_severity_per_design_table(self):
        assert severity_for("RCLLF-H000") == Severity.INFO
        assert severity_for("RCLLF-H001") == Severity.WARNING
        assert severity_for("RCLLF-H002") == Severity.WARNING
        assert severity_for("RCLLF-H003") == Severity.ERROR
        assert severity_for("RCLLF-H004") == Severity.WARNING
        assert severity_for("RCLLF-H005") == Severity.WARNING
        assert severity_for("RCLLF-H006") == Severity.WARNING
        assert severity_for("RCLLF-H007") == Severity.ERROR
        assert severity_for("RCLLF-H008") == Severity.WARNING
        assert severity_for("RCLLF-H009") == Severity.ERROR
        assert severity_for("RCLLF-H010") == Severity.INFO
        assert severity_for("RCLLF-H011") == Severity.ERROR
        assert severity_for("RCLLF-H017") == Severity.ERROR

    def test_severity_for_unknown_code_raises(self):
        with pytest.raises(KeyError):
            severity_for("RCLLF-DOES-NOT-EXIST")

    def test_exactly_twenty_nine_modules_registered(self):
        """4 QoS-hazard + 10 SUPPORTED core + 1 extraction-hardening + 1
        lifecycle-chaining + 1 resolved-model gate + 1 out-of-scope-timer
        + 1 unmodeled-callback + 1 ROS-time sleep (RCLLF-H008)
        + 1 blocking service client (RCLLF-H009)
        + 1 BNA synchronizer (RCLLF-H010 info)
        + 1 thread action server (RCLLF-H011)
        + 1 intra-process (RCLLF-H017).

        The lifecycle-chaining module (RCLLF-H005) was added;
        the resolved-model gate module (RCLLF-R001-R003); the
        out-of-scope-timer module (RCLLF-H006); core.correspondence,
        core.message_filters; the
        unmodeled-callback module (RCLLF-H007).
        """
        assert set(REGISTRY.keys()) == EXPECTED_FEATURE_IDS
        assert len(REGISTRY) == 29

    def test_supported_core_modules_own_no_codes(self):
        core_ids = {fid for fid in REGISTRY if fid.startswith("core.")}
        assert len(core_ids) == 10
        for fid in core_ids:
            module = REGISTRY[fid]
            assert module.support == Support.SUPPORTED
            assert module.codes == ()
            assert module.severity == {}

    def test_every_module_has_a_playground_case(self):
        """Every registered module has `playground=` set.

        Except ``NO_PLAYGROUND_MODULES``: a module whose codes
        never fire via the analyze-time detect() scan has no diagnostic for
        the playground harness to check.
        """
        for fid, module in REGISTRY.items():
            if fid in NO_PLAYGROUND_MODULES:
                assert module.playground is None
                continue
            assert module.playground is not None, f"{fid} missing playground case"
            assert isinstance(module.playground, PlaygroundCase)
            assert isinstance(module.playground.expected_outcome, Outcome)

    def test_playground_expected_codes_are_subset_of_module_codes_or_empty(self):
        """A module's own case should only assert codes it (or a co-firing peer) owns.

        Modules with no owned codes (core.*) must declare an empty
        expected_codes tuple.
        """
        for fid, module in REGISTRY.items():
            if not module.codes:
                assert module.playground.expected_codes == ()


class TestFeatureModuleInvariants:
    def test_severity_map_must_cover_exactly_codes_missing(self):
        with pytest.raises(AssertionError):
            FeatureModule(
                feature_id="test.bad_missing",
                title="bad",
                support=Support.SUPPORTED,
                codes=("RCLLF-TEST1", "RCLLF-TEST2"),
                severity={"RCLLF-TEST1": Severity.INFO},  # missing TEST2
                detect=lambda ctx: [],
                conversion=None,
                playground=None,
                docs_entry="bad",
            )

    def test_severity_map_must_cover_exactly_codes_extra(self):
        with pytest.raises(AssertionError):
            FeatureModule(
                feature_id="test.bad_extra",
                title="bad",
                support=Support.SUPPORTED,
                codes=("RCLLF-TEST1",),
                severity={
                    "RCLLF-TEST1": Severity.INFO,
                    "RCLLF-TEST2": Severity.INFO,
                },  # extra
                detect=lambda ctx: [],
                conversion=None,
                playground=None,
                docs_entry="bad",
            )

    def test_valid_module_constructs(self):
        module = FeatureModule(
            feature_id="test.ok",
            title="ok",
            support=Support.SUPPORTED,
            codes=("RCLLF-TESTOK",),
            severity={"RCLLF-TESTOK": Severity.WARNING},
            detect=lambda ctx: [],
            conversion=None,
            playground=None,
            docs_entry="ok",
        )
        assert module.codes == ("RCLLF-TESTOK",)

    def test_run_detection_rejects_module_emitting_code_outside_codes(self):
        bad_module = FeatureModule(
            feature_id="test.emits_wrong_code",
            title="bad emitter",
            support=Support.SUPPORTED,
            codes=("RCLLF-TESTOK",),
            severity={"RCLLF-TESTOK": Severity.WARNING},
            detect=lambda ctx: [
                Diagnostic(code="RCLLF-NOT-OWNED", node=None, topic=None, message="oops")
            ],
            conversion=None,
            playground=None,
            docs_entry="bad",
        )
        REGISTRY["test.emits_wrong_code"] = bad_module
        try:
            graph = make_graph({})
            with pytest.raises(AssertionError):
                run_detection(DetectionContext(graph=graph))
        finally:
            del REGISTRY["test.emits_wrong_code"]


class TestRunDetectionIntegration:
    def test_run_detection_across_real_modules(self):
        graph = make_graph(
            {
                "/n1": make_node(
                    "n1",
                    subscribers=[
                        make_sub("t_unknown"),
                        make_sub("t_best_effort", qos_reliability="best_effort"),
                    ],
                ),
            }
        )
        diagnostics = run_detection(DetectionContext(graph=graph))
        codes = {d.code for d in diagnostics}
        assert "RCLLF-H000" in codes  # t_unknown: qos_reliability is None
        assert "RCLLF-H001" in codes  # t_best_effort


class TestOutcomeEnum:
    """Outcome enum + PlaygroundCase typing (Phase 7 design decision 1)."""

    def test_outcome_has_exactly_pass_warn_reject(self):
        assert {o.value for o in Outcome} == {"pass", "warn", "reject"}

    def test_playground_case_expected_outcome_is_outcome_member(self):
        case = PlaygroundCase(
            example="test_pkg/bringup", expected_outcome=Outcome.PASS, expected_codes=()
        )
        assert isinstance(case.expected_outcome, Outcome)
        assert case.expected_outcome is Outcome.PASS

    def test_playground_case_is_frozen(self):
        case = PlaygroundCase(example="test_pkg/bringup", expected_outcome=Outcome.WARN)
        with pytest.raises(AttributeError):
            case.expected_outcome = Outcome.REJECT  # type: ignore[misc]

    def test_playground_case_default_expected_codes_is_empty_tuple(self):
        case = PlaygroundCase(example="test_pkg/bringup", expected_outcome=Outcome.PASS)
        assert case.expected_codes == ()

    def test_feature_module_is_frozen(self):
        """FeatureModule stays frozen (Phase 7 must not relax this)."""
        module = REGISTRY["core.timers"]
        with pytest.raises(AttributeError):
            module.title = "changed"  # type: ignore[misc]
