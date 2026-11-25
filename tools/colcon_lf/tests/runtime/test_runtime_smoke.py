# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Runtime smoke layer: build + run real lfc examples and check output.

This is the skeleton the tracking issue's dependencies will
extend. It only proves the narrow thing the static playground
(``tests/playground/test_playground_cases.py``) cannot: that the generated
binary actually runs, actually publishes/responds, and actually shuts down
cleanly — not just that analyze/generate accept the launch file. See
``README.md`` in this directory for what it deliberately does not check yet.

Needs the lfc packages built first: ``docker/lf_build.sh pubsub_example
lifecycle_example service_example`` (each rebuilds `lf_generated_bringup` or
`lf_generated_lifecyclenode` fresh — see ``runtime_support._missing_reasons``
for why `pubsub_example` and `service_example` cannot both stay built at
once). CI builds+runs one example at a time for exactly that reason.
"""

from __future__ import annotations

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

_RUN_SECS = 6.0


def test_pubsub_publish_and_correspondence():
    case = rts.require_case("pubsub")
    result = rts.launch_and_capture(case, _RUN_SECS)

    published, heard = rts.parse_pubsub_log(result.log)
    errors = rts.find_pubsub_mismatches(published, heard)
    assert not errors, "\n".join(errors) + f"\n--- log tail ---\n{result.log[-1500:]}"

    assert not result.process_died, f"process had to be SIGKILLed\n{result.log[-1500:]}"
    assert result.finished_cleanly, (
        "expected 'process has finished cleanly' after SIGINT, got:\n" + result.log[-1500:]
    )


def test_lifecycle_reaches_active_and_publishes():
    """Regression: LifecycleNode::on_activate not chaining to the
    base left every LifecyclePublisher deactivated, so the node reached
    'active' but never actually published — silently, since a dropped
    publish only logs once. This asserts the node both reaches active *and*
    that publishes are observed in the log, which the bug would fail.
    """
    case = rts.require_case("lifecycle")
    result = rts.launch_and_capture(case, _RUN_SECS)

    reached_active, ticks = rts.parse_lifecycle_log(result.log)
    assert reached_active, "never logged 'LifecycleExampleNode has been activated':\n" + result.log[-1500:]
    assert len(ticks) >= 3, f"only {len(ticks)} publish(es) observed, expected >= 3:\n{result.log[-1500:]}"
    assert ticks == list(range(len(ticks))), (
        f"publish sequence {ticks} is not the contiguous 0..N-1 lifecycle_node.cpp emits"
    )

    assert not result.process_died, f"process had to be SIGKILLed\n{result.log[-1500:]}"
    assert result.finished_cleanly, (
        "expected 'process has finished cleanly' after SIGINT, got:\n" + result.log[-1500:]
    )


def test_service_request_response_values():
    case = rts.require_case("service")
    result = rts.launch_and_capture(case, _RUN_SECS)

    results = rts.parse_service_results(result.log)
    assert len(results) >= 3, f"only {len(results)} service result(s) observed:\n{result.log[-1500:]}"

    for index, (value, success, message) in enumerate(results):
        expected_value, expected_success, expected_message = rts.expected_service_result(index)
        actual = (f"{value:.2f}", success, message)
        expected = (expected_value, expected_success, expected_message)
        assert actual == expected, (
            f"request {index}: got (value={actual[0]}, success={success}, message={message!r}), "
            f"expected (value={expected_value}, success={expected_success}, message={expected_message!r})"
        )

    assert not result.process_died, f"process had to be SIGKILLed\n{result.log[-1500:]}"
    assert result.finished_cleanly, (
        "expected 'process has finished cleanly' after SIGINT, got:\n" + result.log[-1500:]
    )


# ── negative case: prove the checker actually discriminates ───────────────


def test_negative_fixture_catches_value_mismatch():
    """Falsification for `find_pubsub_mismatches`: a real captured log
    (`fixtures/pubsub_bringup_real.log`) passes clean; a copy with one heard
    value altered (`fixtures/pubsub_bringup_mismatch.log`, see
    `fixtures/README.md` for exactly what changed and how it was produced)
    must fail with a message naming that specific mismatch, not just fail.
    """
    real_log = (rts.FIXTURES_DIR / "pubsub_bringup_real.log").read_text(encoding="utf-8")
    published, heard = rts.parse_pubsub_log(real_log)
    assert rts.find_pubsub_mismatches(published, heard) == []

    mismatch_log = (rts.FIXTURES_DIR / "pubsub_bringup_mismatch.log").read_text(encoding="utf-8")
    m_published, m_heard = rts.parse_pubsub_log(mismatch_log)
    errors = rts.find_pubsub_mismatches(m_published, m_heard)
    assert errors, "expected the altered fixture to be flagged, but the checker reported it clean"
    assert any("position 2" in e and "99.0, 42.0, 1015.0" in e for e in errors), (
        f"checker failed, but not with the expected mismatch: {errors}"
    )
