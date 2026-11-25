# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Runtime smoke for the one-scheduler ingress (output checks 1-4).

Parameter and lifecycle services of an lfc binary answer ``ros2 param`` and
``ros2 lifecycle`` while every callback runs on an LF worker. Expected values
come from the example sources, not from a previous run:

* ``pubsub_example``: no parameter overrides, so rclcpp's ``use_sim_time`` is False.
* ``parameter_example`` (``parameter_node.cpp``): the launch file loads
  ``config/params.yaml``, so ``robot_name`` is ``ros2_lf_robot`` and ``max_speed``
  is 2.5, not the C++ default ``default_robot``.
  The on-set handler logs ``Parameter changed: max_speed =
  <v>`` and the 1 s timer logs ``Max speed: <v>``.
* ``parameter_example`` (``headerless_param_node.cpp``, no header): the launch
  file loads ``config/e/headerless.yaml`` and then an inline dict keyed
  ``/hl/headerless``, so ``label`` is ``inline_label``, ``gain`` 3.5 and
  ``count`` 21 (file), as under ``ros2 launch``, and the node
  does not abort with ``UnknownROSArgsError`` on ``--rcllf-params-yaml``.
* ``lifecycle_example`` (``lifecycle_node.cpp``): on_deactivate resets the
  timer, on_cleanup the publisher; states follow lifecycle_msgs/State ids.
"""

from __future__ import annotations

import re
import time

import pytest

from . import runtime_support as rts

pytestmark = pytest.mark.runtime

_TICK = r"\[lifecycle_node\]: Publishing:"


def _ok(run, *args, expect: str):
    rc, out = run.ros2(*args)
    assert rc == 0 and expect in out, f"ros2 {' '.join(args)} -> rc {rc}:\n{out}"
    return out


def _assert_callbacks_on_lf_threads(log: str):
    """Every callback_start is on an LF worker (lf_thread_id >= 0), never ingress."""
    ingress = re.search(r"ingress: thread started \(gettid (\d+)\)", log)
    assert ingress, "ingress thread never started"
    starts = re.findall(r'"lf_thread_id":(-?\d+),"gettid":(\d+)', log)
    assert starts, "no callback_start trace lines"
    bad = [s for s in starts if int(s[0]) < 0 or s[1] == ingress.group(1)]
    assert not bad, f"callbacks off LF workers: {bad[:5]}"


def _assert_clean(result):
    assert not result.process_died, f"process had to be SIGKILLed\n{result.log[-1500:]}"
    assert result.finished_cleanly, "no 'process has finished cleanly':\n" + result.log[-1500:]


def test_pubsub_param_get_answers():
    case = rts.require_case("pubsub")

    def during(run):
        _ok(run, "param", "get", "/publisher_node", "use_sim_time", expect="Boolean value is: False")
        _ok(run, "param", "list", "/publisher_node", expect="use_sim_time")

    result = rts.launch_interactive(case, during, wait_nodes=("/publisher_node",))
    _assert_callbacks_on_lf_threads(result.log)
    _assert_clean(result)


def test_parameter_set_reaches_node_on_lf_worker():
    case = rts.require_case("parameter")

    def during(run):
        _ok(run, "param", "get", "/parameter_node", "robot_name", expect="String value is: ros2_lf_robot")
        _ok(run, "param", "set", "/parameter_node", "max_speed", "3.5", expect="Set parameter successful")
        time.sleep(2.5)  # > one 1 s timer period

    result = rts.launch_interactive(case, during, wait_nodes=("/parameter_node",))
    log = result.log
    changed = log.find("Parameter changed: max_speed = 3.500000")
    assert changed >= 0, "on-set handler never logged the new value:\n" + log[-1500:]
    assert "Max speed: 3.500000" in log[changed:], "timer never read the new value:\n" + log[-1500:]
    assert re.search(r'"callback":"set_parameters","lf_thread_id":\d+,.*"ingress_record_id":\d+', log)
    _assert_callbacks_on_lf_threads(log)
    _assert_clean(result)


def test_headerless_node_gets_launch_file_and_inline_values():
    case = rts.require_case("headerless_params")
    expected = "label=inline_label gain=3.500000 count=21"

    def during(run):
        _ok(run, "param", "get", "/hl/headerless", "label", expect="String value is: inline_label")
        _ok(run, "param", "get", "/hl/headerless", "gain", expect="Double value is: 3.5")
        _ok(run, "param", "get", "/hl/headerless", "count", expect="Integer value is: 21")
        _ok(run, "topic", "echo", "--once", "/hl/value", "std_msgs/msg/String",
            expect=f"data: {expected}")

    result = rts.launch_interactive(case, during, wait_nodes=("/hl/headerless",))
    assert f"headerless params: {expected}" in result.log, result.log[-1500:]
    _assert_clean(result)


def test_lifecycle_deactivate_stops_ticks_and_cleanup_is_safe():
    case = rts.require_case("lifecycle")

    def during(run):
        _ok(run, "lifecycle", "get", "/lifecycle_node", expect="active [3]")
        _ok(run, "lifecycle", "set", "/lifecycle_node", "deactivate", expect="Transitioning successful")
        before = run.count(_TICK)
        time.sleep(3.0)
        assert run.count(_TICK) == before, "timer kept firing after deactivate"
        _ok(run, "lifecycle", "set", "/lifecycle_node", "cleanup", expect="Transitioning successful")
        time.sleep(1.5)
        assert run.count(_TICK) == before, "tick after cleanup released the publisher"
        _ok(run, "lifecycle", "get", "/lifecycle_node", expect="unconfigured [1]")

    result = rts.launch_interactive(case, during, wait_nodes=("/lifecycle_node",))
    _assert_callbacks_on_lf_threads(result.log)
    _assert_clean(result)


def test_lifecycle_manual_transitions_via_ros2_lifecycle():
    """Needs lifecycle_example generated with lifecycle_manual_execution.yaml."""
    case = rts.require_case("lifecycle_manual")

    def during(run):
        rc, out = run.ros2("lifecycle", "get", "/lifecycle_node")
        if "unconfigured [1]" not in out:
            pytest.fail(
                "lifecycle_example was not generated with auto_transition: false "
                f"(LF_EXEC_CONFIG=tests/runtime/lifecycle_manual_execution.yaml); state: {out!r}"
            )
        for transition in ("configure", "activate"):
            _ok(run, "lifecycle", "set", "/lifecycle_node", transition, expect="Transitioning successful")
        # The timer on_activate creates is bound at its activation and
        # publishes under LF: a subscriber outside the binary receives it.
        echoed.append(_ok(run, "topic", "echo", "--once", "/lifecycle_output", "std_msgs/msg/String",
                          expect="Lifecycle tick: "))
        _ok(run, "lifecycle", "set", "/lifecycle_node", "deactivate", expect="Transitioning successful")
        _ok(run, "lifecycle", "get", "/lifecycle_node", expect="inactive [2]")
        time.sleep(2.5)  # > two 1 s periods: the timer on_deactivate dropped must stay silent

    echoed: list[str] = []
    result = rts.launch_interactive(case, during, wait_nodes=("/lifecycle_node",))
    for word in ("configured", "activated", "deactivated"):
        assert f"LifecycleExampleNode has been {word}" in result.log, result.log[-1500:]
    assert len(re.findall(r'"callback":"change_state"', result.log)) == 3
    assert re.search(r"Bound (?:late|lifecycle) timer RID \d+ \(LifecycleExampleNode::timer_callback", result.log), (
        result.log[-1500:]
    )
    # lifecycle_node.cpp counts from 0 and publishes "Lifecycle tick: <n>" once per period.
    ticks = [int(n) for n in re.findall(_TICK + r" 'Lifecycle tick: (\d+)'", result.log)]
    assert ticks and ticks == list(range(len(ticks))), ticks
    received = int(re.search(r"Lifecycle tick: (\d+)", echoed[0]).group(1))
    assert received in ticks, (received, ticks)
    after_deactivate = result.log.split("LifecycleExampleNode has been deactivated", 1)[1]
    assert not re.search(_TICK, after_deactivate), after_deactivate[-1500:]
    assert "unbound at" not in result.log, result.log[-1500:]
    _assert_callbacks_on_lf_threads(result.log)
    _assert_clean(result)
