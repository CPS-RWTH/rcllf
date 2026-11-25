# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the parts of the bt2 backend that don't need ``bt2`` importable.

``bt2`` is a system-only package (not a pip dependency, see the module
docstring), so it is genuinely absent on a bare host — the full
``read_events`` path is only exercised by the ``skipif``-gated integration
suite (:mod:`tests.reader.test_reader_integration`), same as the rest of the
repo's convention. ``_message_count`` is a pure helper that never imports
``bt2`` itself, so it is safe (and worth) testing directly here.
"""

from __future__ import annotations

from lf_eval.reader.bt2_backend import _message_count


class _FakeMessage:
    def __init__(self, count):
        self.count = count


def test_message_count_uses_exact_count_when_present():
    assert _message_count(_FakeMessage(7)) == 7


def test_message_count_falls_back_to_one_when_count_is_none():
    """bt2 reports `count=None` when the clock can't derive an exact figure —
    treated as a lower-bound 1, not fabricated as an exact number."""
    assert _message_count(_FakeMessage(None)) == 1


def test_message_count_falls_back_to_one_when_attribute_missing():
    class _NoCount:
        pass

    assert _message_count(_NoCount()) == 1


def test_message_count_rejects_non_int_count():
    assert _message_count(_FakeMessage("not-a-number")) == 1
