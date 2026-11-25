# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for run metadata loading."""

from __future__ import annotations

import json

import pytest

from lf_eval.model.run import MODES, load_cpu_proc, load_metadata, make_run
from lf_eval.model.timeline import build_timeline

REAL_METADATA = """\
timestamp: "20260424_002456"
mode: "stock"
duration: 60
system: "autoware_reference_system"
label: "phase5_v2_run3"
hostname: "host01"
"""


def test_loads_real_metadata_shape(tmp_path):
    (tmp_path / "metadata.yaml").write_text(REAL_METADATA)
    meta = load_metadata(tmp_path)
    assert meta.mode == "stock"
    assert meta.duration_s == pytest.approx(60.0)
    assert meta.system == "autoware_reference_system"
    assert meta.label == "phase5_v2_run3"
    assert meta.hostname == "host01"
    assert meta.timestamp == "20260424_002456"


def test_unknown_keys_go_to_extra(tmp_path):
    (tmp_path / "metadata.yaml").write_text("mode: lfc\ncustom_key: 7\n")
    meta = load_metadata(tmp_path)
    assert meta.mode == "lfc"
    assert meta.extra == {"custom_key": 7}


def test_missing_metadata_yields_defaults(tmp_path):
    meta = load_metadata(tmp_path)
    assert meta.mode == "unknown"
    assert meta.duration_s == 0.0


def test_empty_metadata_yields_defaults(tmp_path):
    (tmp_path / "metadata.yaml").write_text("")
    assert load_metadata(tmp_path).mode == "unknown"


def test_non_mapping_metadata_yields_defaults(tmp_path):
    (tmp_path / "metadata.yaml").write_text("- just\n- a list\n")
    assert load_metadata(tmp_path).mode == "unknown"


def test_load_cpu_proc_reads_json(tmp_path):
    payload = {"aggregate_cores": 0.55, "n_cores_machine": 12}
    (tmp_path / "cpu_proc.json").write_text(json.dumps(payload))
    assert load_cpu_proc(tmp_path) == payload


def test_load_cpu_proc_missing_returns_none(tmp_path):
    assert load_cpu_proc(tmp_path) is None


def test_load_cpu_proc_invalid_json_returns_none(tmp_path):
    (tmp_path / "cpu_proc.json").write_text("{not json")
    assert load_cpu_proc(tmp_path) is None


def test_make_run_wraps_a_timeline():
    timeline = build_timeline([])
    run = make_run("r1", timeline, mode="lfc", duration_s=60.0)
    assert run.name == "r1"
    assert run.mode == "lfc"
    assert run.metadata.duration_s == 60.0
    assert run.timeline is timeline


# ── mode generalization (Phase 6) ────────────────────────────────────────


def test_modes_includes_phase6_per_executor_vocabulary():
    """The five-variant vocabulary (design §D/§E) must be in MODES."""
    for mode in ("stock-single", "stock-multi", "stock-events", "lfc", "rslcpp"):
        assert mode in MODES


def test_modes_keeps_pre_phase6_values_for_backward_compat():
    assert "lf" in MODES
    assert "stock" in MODES


@pytest.mark.parametrize(
    "mode",
    [
        "stock-single",
        "stock-multi",
        "stock-events",
        "lfc",
        "rslcpp",
        "a-totally-new-mode-string",
    ],
)
def test_load_metadata_passes_any_mode_string_through_unmangled(tmp_path, mode):
    """MODES is documentation, not a gate — load_metadata never rejects/splits."""
    (tmp_path / "metadata.yaml").write_text(f'mode: "{mode}"\nduration: 10\n')
    meta = load_metadata(tmp_path)
    assert meta.mode == mode


def test_load_run_discard_stats_defaults_to_none_via_make_run():
    """make_run (the synthetic-test seam) leaves discard_stats unset by default."""
    timeline = build_timeline([])
    run = make_run("r1", timeline, mode="lfc")
    assert run.discard_stats is None


# ── measurement window ──────────────────────────────────────────

def _ten_fires():
    from tests.conftest import MS, EventBuilder

    b = EventBuilder()
    b.add_node("A")
    b.add_timer("A", period_ms=100.0)
    for i in range(10):
        b.timer_fire("A", start_ns=i * 100 * MS, duration_ns=5 * MS)
    return b.build(), MS


def test_window_keeps_steady_runtime_events_and_all_registration():
    from lf_eval.model.run import Window, apply_window

    events, MS = _ten_fires()
    kept = apply_window(events, Window(warmup_s=0.2, steady_s=0.5))
    starts = [e.timestamp_ns for e in kept if e.name == "callback_start"]
    assert starts == [i * 100 * MS for i in range(2, 7)], starts
    # registration happened at t=0, before the window, and must survive
    assert {e.name for e in kept} >= {"rcl_node_init", "rcl_timer_init", "rclcpp_timer_callback_added"}
    from lf_eval.model.run import windowed_timeline

    timeline = windowed_timeline(events, Window(warmup_s=0.2, steady_s=0.5))
    assert timeline.total_callback_starts == 5
    assert timeline.duration_ns == 500 * MS, "the span must be the window, not startup plus window"


def test_window_without_steady_length_only_drops_warmup():
    from lf_eval.model.run import Window, apply_window

    events, MS = _ten_fires()
    kept = apply_window(events, Window(warmup_s=0.5))
    assert [e.timestamp_ns for e in kept if e.name == "callback_start"] == [i * 100 * MS for i in range(5, 10)]


def test_window_that_outlasts_the_run_is_refused():
    from lf_eval.model.run import Window, WindowError, apply_window

    events, _ = _ten_fires()
    with pytest.raises(WindowError, match="ends after the last"):
        apply_window(events, Window(warmup_s=0.5, steady_s=5.0))


def test_simulated_time_runs_ignore_the_wall_window():
    from lf_eval.model.run import SIMULATED_MODES, Window, window_for_mode

    assert "rslcpp" in SIMULATED_MODES
    w = Window(warmup_s=5.0, steady_s=20.0)
    assert window_for_mode(w, "rslcpp") is None
    assert window_for_mode(w, "lfc") is w
