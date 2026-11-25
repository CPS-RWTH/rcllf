# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Reader integration tests against a real CTF trace.

These SKIP cleanly when the host has no ``output/experiments/`` run directory or
no babeltrace backend — no CTF binaries are committed to the repo. Everything
else in the suite runs on synthetic events and is always executed.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from lf_eval.reader import (
    BackendUnavailableError,
    available_backends,
    find_tracing_dir,
    read_trace,
)

pytestmark = pytest.mark.skipif(
    not available_backends(),
    reason="no CTF backend (bt2 module or babeltrace2/babeltrace CLI) available",
)


@pytest.fixture(scope="module")
def trace_dir(real_run_dir):
    if real_run_dir is None:
        pytest.skip("no real trace directory under output/experiments/")
    return real_run_dir


def test_find_tracing_dir_accepts_run_dir(trace_dir: Path):
    assert find_tracing_dir(trace_dir) == trace_dir / "tracing"


def test_find_tracing_dir_accepts_ctf_dir_directly(trace_dir: Path):
    ctf = trace_dir / "tracing"
    assert find_tracing_dir(ctf) == ctf


def test_find_tracing_dir_rejects_missing(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        find_tracing_dir(tmp_path / "nope")


def test_reads_real_callback_events(trace_dir: Path):
    """A real trace must yield paired callback events with pointer handles."""
    seen = {"callback_start": 0, "callback_end": 0}
    for event in read_trace(
        trace_dir, event_names={"callback_start", "callback_end"}
    ):
        seen[event.name] += 1
        assert isinstance(event.fields.get("callback"), int)
        assert event.timestamp_ns > 0
        if sum(seen.values()) >= 400:
            break
    assert seen["callback_start"] > 0
    assert seen["callback_end"] > 0


def test_reads_node_names_from_real_trace(trace_dir: Path):
    names = set()
    for event in read_trace(trace_dir, event_names={"rcl_node_init"}):
        node_name = event.fields.get("node_name")
        if isinstance(node_name, str):
            names.add(node_name)
        if len(names) >= 5:
            break
    assert names, "expected at least one rcl_node_init with a node_name"


def test_event_filter_excludes_other_events(trace_dir: Path):
    count = 0
    for event in read_trace(trace_dir, event_names={"rcl_timer_init"}):
        assert event.name == "rcl_timer_init"
        count += 1
        if count >= 5:
            break


def test_timestamps_are_monotonic(trace_dir: Path):
    previous = 0
    for i, event in enumerate(
        read_trace(trace_dir, event_names={"callback_start"})
    ):
        assert event.timestamp_ns >= previous
        previous = event.timestamp_ns
        if i >= 500:
            break


def test_unavailable_backend_raises_clear_error(trace_dir: Path, monkeypatch):
    """Forcing an absent backend must fail with actionable guidance."""
    from lf_eval.reader import reader as reader_module

    monkeypatch.setattr(reader_module.bt2_backend, "is_available", lambda: False)
    monkeypatch.setattr(reader_module.babeltrace_cli, "is_available", lambda: False)
    with pytest.raises(BackendUnavailableError) as excinfo:
        list(read_trace(trace_dir))
    assert "babeltrace" in str(excinfo.value)


def test_load_run_builds_a_usable_timeline(trace_dir: Path):
    """End-to-end: a real run dir yields attributed callbacks."""
    from lf_eval.model import load_run

    run = load_run(trace_dir)
    assert run.timeline.total_callback_starts > 0
    assert run.timeline.duration_ns > 0
    # Real traces must resolve at least some callbacks to node names.
    assert run.timeline.nodes, "no callbacks could be attributed to nodes"
