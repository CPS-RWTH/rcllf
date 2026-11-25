# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for canonicalization/hashing (T2)."""

from __future__ import annotations

import copy
import math

import pytest

from colcon_lf.resolved_model.model import (
    CLAIM_SUPPORTED,
    CLAIM_UNSUPPORTED,
    PROVENANCE_EXPLICITLY_CONFIGURED,
    PROVENANCE_EXTRACTED,
    Record,
    ResolvedModel,
    canonical_bytes,
    model_hash,
)


def _record(contract_id="timer:/imu_node/timer_callback#period", value=5.0, provenance=PROVENANCE_EXPLICITLY_CONFIGURED):
    return Record(
        contract_id=contract_id,
        property="period",
        subject={"kind": "timer", "node": "/imu_node", "callback": "timer_callback"},
        value=value,
        unit="ms",
        domain="(0,inf)",
        provenance=provenance,
        evidence="lf_bench_scenario#nodes.imu_node.rate_hz=200.0",
    )


def _model(records=None, inputs=None, claim_status=CLAIM_SUPPORTED):
    return ResolvedModel(
        system={"name": "sentry_pipeline", "system_json_schema_version": 3},
        claim_status=claim_status,
        records=records if records is not None else [_record()],
        mutation_channels={
            "launch_overrides": "resolved",
            "ros_parameter_events": "observable_not_monitored",
            "environment": "not_detectable",
            "direct_field_mutation": "not_detectable",
        },
        inputs=inputs if inputs is not None else [{"role": "system_json", "path": "a.json", "sha256": "abc"}],
    )


class TestRecordOrderIndependence:
    def test_record_list_order_does_not_change_hash(self):
        r1, r2 = _record("timer:/a#period", 1.0), _record("timer:/b#period", 2.0)
        h1 = model_hash(_model(records=[r1, r2]))
        h2 = model_hash(_model(records=[r2, r1]))
        assert h1 == h2


class TestKeyOrderIndependence:
    def test_dict_construction_order_does_not_change_hash(self):
        m1 = _model()
        m2 = _model()
        # Rebuild subject/mutation_channels with reversed insertion order.
        m2.records[0].subject = dict(reversed(list(m2.records[0].subject.items())))
        m2.mutation_channels = dict(reversed(list(m2.mutation_channels.items())))
        assert model_hash(m1) == model_hash(m2)


class TestInputsOutsideHash:
    def test_inputs_change_does_not_change_hash(self):
        m1 = _model(inputs=[{"role": "system_json", "path": "a.json", "sha256": "abc"}])
        m2 = _model(inputs=[{"role": "system_json", "path": "a.json", "sha256": "different"}])
        assert model_hash(m1) == model_hash(m2)

    def test_workspace_relative_paths_give_same_hash_under_different_roots(self):
        # Two "workspace roots" producing identical relative `inputs` paths
        # (and an identical body) must hash identically — the whole point of
        # relativizing paths before constructing inputs.
        m1 = _model(inputs=[{"role": "system_json", "path": "packages/a/system.json", "sha256": "x"}])
        m2 = _model(inputs=[{"role": "system_json", "path": "packages/a/system.json", "sha256": "x"}])
        assert model_hash(m1) == model_hash(m2)


class TestValueProvenanceAckChangeHash:
    def test_value_change_changes_hash(self):
        base = model_hash(_model())
        changed = model_hash(_model(records=[_record(value=6.0)]))
        assert base != changed

    def test_provenance_change_changes_hash(self):
        base = model_hash(_model())
        changed = model_hash(_model(records=[_record(provenance=PROVENANCE_EXTRACTED)]))
        assert base != changed

    def test_claim_status_change_changes_hash(self):
        # Stands in for an acknowledged_hazards change: acknowledging a
        # gated code flips claim_status to unsupported (T7).
        base = model_hash(_model(claim_status=CLAIM_SUPPORTED))
        changed = model_hash(_model(claim_status=CLAIM_UNSUPPORTED))
        assert base != changed


class TestIntFloatDistinct:
    def test_int_and_float_hash_differently(self):
        h_int = model_hash(_model(records=[_record(value=1)]))
        h_float = model_hash(_model(records=[_record(value=1.0)]))
        assert h_int != h_float


class TestNanRejected:
    def test_nan_value_rejected_at_construction(self):
        with pytest.raises(ValueError, match="NaN"):
            _record(value=float("nan"))

    def test_inf_value_rejected_at_construction(self):
        with pytest.raises(ValueError, match="NaN"):
            _record(value=float("inf"))

    def test_nan_rejected_even_if_smuggled_past_construction(self):
        m = _model()
        m.records[0].value = float("nan")  # bypass __post_init__
        with pytest.raises(ValueError):
            canonical_bytes(m)


class TestReproducible:
    def test_hashing_twice_gives_same_result(self):
        m = _model()
        assert model_hash(copy.deepcopy(m)) == model_hash(copy.deepcopy(m))

    def test_canonical_bytes_is_compact_sorted_json(self):
        raw = canonical_bytes(_model()).decode("utf-8")
        assert ", " not in raw and ": " not in raw  # separators=(",", ":")
        # Top-level keys sorted alphabetically: claim_status < mutation_channels
        # < records < schema < schema_version < system.
        assert raw.startswith('{"claim_status":')
