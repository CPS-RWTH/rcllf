# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Release check of the synchronizer and tf2 claims.

An earlier audit found a support-matrix row that called message_filters
synchronizers validated with no limits, a reference page that said tf2
detection does not exist, and R002 texts that cited a closed tracker issue
that never covered tf2.
"""

from __future__ import annotations

from pathlib import Path

from colcon_lf.features import evidence
from colcon_lf.features.modules import message_filters, resolved_model_gate

REPO = Path(__file__).resolve().parents[4]


def test_message_filters_row_states_evaluation_reach():
    entry = message_filters.MODULE.docs_entry
    assert "Python port" in entry and "#90" not in entry, (
        f"core.message_filters row hides that the evaluation checked a Python port: {entry!r}")
    for phrase in ("20 of 20", "not against the library", "admitted by the scan", "legacy edge"):
        assert phrase in entry, f"core.message_filters row lost {phrase!r}: {entry!r}"
    # A legacy sync input is refused (H003), so the row cites the
    # domain limit (L8) instead of the trigger-only legacy edge (L6).
    assert "refused with RCLLF-H003 sync_input_legacy" in entry, (
        f"core.message_filters row hides the legacy refusal: {entry!r}")
    _, limits = evidence.EVIDENCE["core.message_filters"]
    assert "message-filters-sync-domain" in limits, (
        f"core.message_filters cites no domain limit (L8): {limits!r}")


def test_r002_texts_cite_no_tracker_numbers():
    module = resolved_model_gate.MODULE
    for name, text in (("docs_entry", module.docs_entry), ("conversion", module.conversion)):
        assert "#49" not in text, f"resolved_model.gate {name} cites a closed tracker issue: {text!r}"
        assert "#91" not in text, f"resolved_model.gate {name} cites a tracker number: {text!r}"


def test_resolved_model_page_does_not_call_tf2_detection_missing():
    page = (REPO / "docs" / "reference" / "resolved-model.md").read_text()
    assert "detection does not exist" not in page, (
        "docs/reference/resolved-model.md says tf2/take() detection does not exist")
    assert "#49" not in page, "docs/reference/resolved-model.md cites a closed tracker issue"
    limitations = page.split("## Limitations", 1)[1]
    # tf2 converts in one form and every other form is refused with
    # an R002 subcode; take() stays detected and unsupported.
    for phrase in ("`take()` is detected, not supported",
                   "#mechanism.take` record with reason RCLLF-R002",
                   "`colcon lf-generate` exits 1",
                   "tf2 is supported in one form only",
                   "refuses\n  `tf_listener` with `tf2_direct_lookup`",
                   "Acknowledging RCLLF-R002 lets generation go on without making tf2 deterministic",
                   # The user page names no tracker number; it points to the tf2
                   # feature page instead.
                   "tf2 is supported in one form only ([tf2 lookups](../features/tf2.md))"):
        assert phrase in limitations, (
            f"resolved-model.md Limitations no longer states {phrase!r}")
