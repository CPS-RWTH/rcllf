# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for backend selection and the ``discards`` plumbing in ``reader.py``.

Both backend modules are monkeypatched so this exercises the selection/plumbing
logic itself without needing a real CTF trace or babeltrace installation — the
CTF-reading backends themselves are covered by
:mod:`tests.reader.test_reader_integration` (skipif-gated) and
:mod:`tests.reader.test_bt2_backend`/``test_babeltrace_cli``.
"""

from __future__ import annotations

import pytest

from lf_eval.reader import reader as reader_module
from lf_eval.reader.events import DiscardStats, TraceEvent
from lf_eval.reader.reader import BackendUnavailableError, read_trace


@pytest.fixture
def fake_backends(monkeypatch):
    """Force both backends "available" and record the kwargs each receives."""
    calls = {}

    def make_reader(name):
        def read_events(tracing_dir, discards=None):
            calls[name] = discards
            yield TraceEvent(name="callback_start", timestamp_ns=1, fields={})

        return read_events

    monkeypatch.setattr(reader_module.bt2_backend, "is_available", lambda: True)
    monkeypatch.setattr(reader_module.babeltrace_cli, "is_available", lambda: True)
    monkeypatch.setattr(reader_module.bt2_backend, "read_events", make_reader("bt2"))
    monkeypatch.setattr(reader_module.babeltrace_cli, "read_events", make_reader("cli"))
    return calls


def test_read_trace_prefers_bt2_by_default(fake_backends, tmp_path):
    (tmp_path / "tracing").mkdir()
    list(read_trace(tmp_path))
    assert "bt2" in fake_backends
    assert "cli" not in fake_backends


def test_read_trace_forwards_discards_sink_to_backend(fake_backends, tmp_path):
    (tmp_path / "tracing").mkdir()
    sink = DiscardStats()
    list(read_trace(tmp_path, discards=sink))
    assert fake_backends["bt2"] is sink


def test_read_trace_without_discards_passes_none(fake_backends, tmp_path):
    (tmp_path / "tracing").mkdir()
    list(read_trace(tmp_path))
    assert fake_backends["bt2"] is None


def test_read_trace_forced_cli_backend_forwards_discards(fake_backends, tmp_path):
    (tmp_path / "tracing").mkdir()
    sink = DiscardStats()
    list(read_trace(tmp_path, backend="cli", discards=sink))
    assert fake_backends["cli"] is sink
    assert "bt2" not in fake_backends


def test_read_trace_raises_when_no_backend_available(monkeypatch, tmp_path):
    (tmp_path / "tracing").mkdir()
    monkeypatch.setattr(reader_module.bt2_backend, "is_available", lambda: False)
    monkeypatch.setattr(reader_module.babeltrace_cli, "is_available", lambda: False)
    with pytest.raises(BackendUnavailableError):
        list(read_trace(tmp_path))
