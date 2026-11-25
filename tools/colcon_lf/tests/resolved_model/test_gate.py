# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for the RCLLF-R001/R002/R003 fail-closed gate (T6/T7).

Runs the real `colcon lf-generate` entry point (`verb.generate._generate_one`)
against a real, analyzed `sentry_pipeline` system.json — this is the
end-to-end path, not just `resolved_model.resolve()` in isolation, so it
also proves the P3 timer-period override reaches the *generated* LF code
(not just the resolved_model.json report).
"""

from __future__ import annotations

import textwrap

import pytest

from ..ros_env import requires_ros_headers

pytestmark = requires_ros_headers("the real lf-generate R001 gate")


@pytest.fixture(scope="module")
def sentry_system_json(package_dir, analyzed_graph, tmp_path_factory):
    from colcon_lf.output.writer import write

    launch = package_dir("sentry_pipeline") / "launch" / "sentry_pipeline.launch.py"
    graph = analyzed_graph(launch)
    out = tmp_path_factory.mktemp("sentry-gate-test")
    write(graph, out)
    return out / "system.json"


def _generate(system_json, tmp_path, exec_config_path=None):
    from colcon_lf.verb.generate import _generate_one

    rc = _generate_one(
        system_json=system_json,
        output_dir=str(tmp_path / "lf"),
        after_delay=None,
        exec_config_arg=str(exec_config_path) if exec_config_path else None,
    )
    return rc


class TestUnpatchedRefusal:
    def test_refuses_with_exact_r001_text(self, sentry_system_json, tmp_path, capsys):
        rc = _generate(sentry_system_json, tmp_path)
        assert rc != 0
        err = capsys.readouterr().err
        assert (
            "Error: generation refused, RCLLF-R001 timer period unresolved "
            "for imu_node/timer_callback (period_ms is null in system.json; "
            "supply it explicitly or set nodes.imu_node.timers.timer_callback."
            "period in execution.yaml)"
        ) in err
        # resolved_model.json is still staged even on refusal (report exists).
        assert (sentry_system_json.parent / "resolved_model.json").is_file()

    def test_falsification_wrong_message_would_be_caught(self, sentry_system_json, tmp_path, capsys):
        """A falsification: the exact wrong-but-plausible message a lazier
        gate might emit must NOT be what we get (guards against a
        regression that keeps refusing, but with a vaguer message).
        """
        _generate(sentry_system_json, tmp_path)
        err = capsys.readouterr().err
        assert "Error: generation refused" in err  # sanity: it did refuse
        assert "timer period unresolved for imu_node/timer_callback" in err
        assert "generic timer error" not in err  # the bug this guards against


class TestAcknowledgementDoesNotProceed:
    def test_acknowledged_r001_still_refuses(self, sentry_system_json, tmp_path, capsys):
        """An acknowledged RCLLF-R001 used to proceed with a
        1000 msec placeholder timer the binding cannot match. It refuses."""
        exec_config = tmp_path / "execution.yaml"
        exec_config.write_text(textwrap.dedent("""\
            schema_version: 1
            acknowledged_hazards: ["RCLLF-R001"]
        """))
        rc = _generate(sentry_system_json, tmp_path, exec_config_path=exec_config)
        err = capsys.readouterr().err
        assert rc != 0, (
            "acknowledged RCLLF-R001 proceeded with a placeholder timer period")
        assert "Error: generation refused, RCLLF-R001 timer period unresolved" in err
        assert "acknowledged_hazards no longer proceeds)" in err
        assert "Warning: acknowledged hazard RCLLF-R001 present" not in err

    def test_make_timer_refuses_a_missing_period(self):
        from colcon_lf.generation.lf_generator import _make_timer
        from colcon_lf.node_analysis.models import TimerInfo

        with pytest.raises(ValueError, match="has no period"):
            _make_timer(TimerInfo(callback="timer_callback", period_ms=None))


_ALL_FOUR_TIMERS = """\
    schema_version: 1
    nodes:
      imu_node:
        timers:
          timer_callback:
            period: "5 msec"
      lidar_node:
        timers:
          timer_callback:
            period: "100 msec"
      camera_node:
        timers:
          timer_callback:
            period: "33 msec"
      planner_node:
        timers:
          timer_callback:
            period: "50 msec"
"""


class TestP3OverrideSatisfiesR001:
    def test_override_clears_one_timer_others_still_gate(self, sentry_system_json, tmp_path, capsys):
        exec_config = tmp_path / "execution.yaml"
        exec_config.write_text(textwrap.dedent("""\
            schema_version: 1
            nodes:
              imu_node:
                timers:
                  timer_callback:
                    period: "5 msec"
            acknowledged_hazards: []
        """))
        rc = _generate(sentry_system_json, tmp_path, exec_config_path=exec_config)
        assert rc != 0  # the other three still refuse
        err = capsys.readouterr().err
        assert "imu_node/timer_callback" not in err
        assert "lidar_node/timer_callback" in err

    def test_overrides_reach_generated_lf_at_the_real_value(
        self, sentry_system_json, tmp_path, capsys
    ):
        exec_config = tmp_path / "execution.yaml"
        exec_config.write_text(textwrap.dedent(_ALL_FOUR_TIMERS))
        rc = _generate(sentry_system_json, tmp_path, exec_config_path=exec_config)
        assert rc == 0

        lf_files = list((tmp_path / "lf").glob("*.lf"))
        assert lf_files
        lf_text = lf_files[0].read_text()
        assert "timer t_timer_callback(0, 5 msec)" in lf_text
        # No 1000 msec placeholder survives anywhere.
        assert "1000 msec" not in lf_text

    def test_override_reaches_the_binding_rid_table(self, sentry_system_json, tmp_path):
        """The RID entry carries the overridden period, so the
        binding finds the 5 ms timer handle. With period 0 it refuses at bind:
        "RID 1 (ImuNode::timer_callback, timer, period 0 ns) has no unbound
        timer handle in node 'ImuNode' that fits it"."""
        import re
        import shutil

        # The lf-generated package lands next to lf-analysis/, so stage the
        # system.json in its own lf-analysis/<pkg>/<stem>/ tree.
        sj = tmp_path / "lf-analysis" / "sentry_pipeline" / "sentry_pipeline" / "system.json"
        sj.parent.mkdir(parents=True)
        shutil.copy(sentry_system_json, sj)
        exec_config = tmp_path / "execution.yaml"
        exec_config.write_text(textwrap.dedent(_ALL_FOUR_TIMERS))
        assert _generate(sj, tmp_path, exec_config_path=exec_config) == 0

        (init,) = (tmp_path / "lf-generated").glob("*/src/ros2_lf_reaction_ids_init.cpp")
        text = init.read_text()
        row = re.search(
            r'register_rid_entry\(\{RID_ImuNode_timer_callback, "ImuNode", "timer_callback", '
            r'"timer", "", (\d+)LL,', text)
        assert row, f"no ImuNode timer RID entry:\n{text}"
        assert int(row.group(1)) == 5_000_000, (
            f"RID 1 (ImuNode::timer_callback, timer, period {row.group(1)} ns) does not "
            f"carry the 5 msec execution.yaml override, so the binding refuses at bind")

    def test_falsify_hash_changes_with_override(self, sentry_system_json, tmp_path):
        import json

        from colcon_lf.exec_config.model import ExecConfig
        from colcon_lf.output.serializer import deserialize_system_graph
        from colcon_lf.resolved_model.resolve import resolve

        graph = deserialize_system_graph(sentry_system_json.read_text())
        before = resolve(sentry_system_json, exec_config=ExecConfig.empty(), workspace=tmp_path)
        cfg = ExecConfig.from_dict(
            {"nodes": {"imu_node": {"timers": {"timer_callback": {"period": "5 msec"}}}}},
            graph, source="execution.yaml",
        )
        after = resolve(sentry_system_json, exec_config=cfg, workspace=tmp_path)
        assert before.model_hash != after.model_hash, (
            "hash unchanged after override"
        )
        diffs = [
            (b.contract_id, b.value, a.value)
            for b, a in zip(before.sorted_records(), after.sorted_records())
            if b.value != a.value
        ]
        assert len(diffs) == 1
        assert diffs[0][0] == "timer:imu_node/timer_callback#period"


@pytest.fixture(scope="module")
def tf_listener_system_json(package_dir, analyzed_graph, tmp_path_factory):
    """tf_listener_example: real tf2_ros::Buffer/TransformListener usage
    — the R002 real-package case, no system.json patching."""
    from colcon_lf.output.writer import write

    launch = package_dir("tf_listener_example") / "launch" / "tf_listener.launch.py"
    graph = analyzed_graph(launch)
    out = tmp_path_factory.mktemp("tf-listener-gate-test")
    write(graph, out)
    return out / "system.json"


@pytest.fixture(scope="module")
def pubsub_system_json(packages_src, analyzed_graph, tmp_path_factory):
    """pubsub_example: no tf2/take mechanism — must be unaffected by T5."""
    from colcon_lf.output.writer import write

    launch = packages_src / "pubsub_example" / "launch" / "bringup.launch.py"
    graph = analyzed_graph(launch)
    out = tmp_path_factory.mktemp("pubsub-gate-test")
    write(graph, out)
    return out / "system.json"


class TestR002Tf2MechanismRefusal:
    """RCLLF-R002: tf2_ros usage detected on a node refuses generation
    (T5) unless acknowledged."""

    def test_refuses_with_r002_text_naming_tf2(self, tf_listener_system_json, tmp_path, capsys):
        """N3: the example's latest lookup is refused with the
        tf2_direct_lookup subcode, verbatim, not with a plain R002
        that names no subcode and no line."""
        rc = _generate(tf_listener_system_json, tmp_path)
        assert rc != 0
        err = capsys.readouterr().err
        assert (
            "  Error: generation refused, RCLLF-R002 tf2_direct_lookup: tf_listener "
            "tf_buffer_->lookupTransform at tf_listener_node.cpp:44 answers with what the "
            "Buffer holds at that instant (the latest transform for TimePointZero) and "
            "bypasses the bounded acquisition-time request; request the transform through "
            "rcllf_tf2::AcquisitionLookup, which answers for the stamp with a bounded wait "
            "(or list RCLLF-R002 in acknowledged_hazards to proceed as unsupported)\n"
        ) in err

    def test_acknowledged_proceeds_unsupported(self, tf_listener_system_json, tmp_path, capsys):
        exec_config = tmp_path / "execution.yaml"
        exec_config.write_text(textwrap.dedent("""\
            schema_version: 1
            acknowledged_hazards: ["RCLLF-R002"]
        """))
        rc = _generate(tf_listener_system_json, tmp_path, exec_config_path=exec_config)
        assert rc == 0
        err = capsys.readouterr().err
        assert "Warning: acknowledged hazard RCLLF-R002 present" in err

        import json
        model = json.loads((tf_listener_system_json.parent / "resolved_model.json").read_text())
        assert model["claim_status"] == "unsupported"

    def test_acknowledged_r002_reaches_the_binding(self, tf_listener_system_json, tmp_path, capsys):
        """The binding exempts a callback group another executor spins only
        for a node whose RCLLF-R002 was acknowledged, so
        the generated init cpp must register that acknowledgement."""
        exec_config = tmp_path / "execution.yaml"
        exec_config.write_text(textwrap.dedent("""\
            schema_version: 1
            acknowledged_hazards: ["RCLLF-R002"]
        """))
        assert _generate(tf_listener_system_json, tmp_path, exec_config_path=exec_config) == 0
        from pathlib import Path

        inits = [
            line.split("-> ", 1)[1].strip()
            for line in capsys.readouterr().out.splitlines()
            if line.strip().endswith("ros2_lf_reaction_ids_init.cpp")
        ]
        assert len(inits) == 1, inits
        text = Path(inits[0]).read_text()
        assert (
            'ros2_lf_binding::register_foreign_executor_ack("TfListenerNode", "RCLLF-R002");'
            in text
        ), "the RCLLF-R002 acknowledgement was not passed to the binding"

    def test_falsify_disabling_detection_removes_refusal(self, tf_listener_system_json, tmp_path, monkeypatch):
        """Falsification (a test must fail with the message the bug would produce): disable the tf2
        detection and show the exact refusal above disappears — proving
        it is caused by the detection, not some other unresolved record."""
        import colcon_lf.resolved_model.resolve as resolve_mod

        monkeypatch.setattr(resolve_mod, "_TF2_INCLUDE", "__disabled_for_falsification__")
        rc = _generate(tf_listener_system_json, tmp_path)
        assert rc == 0, "R002 refusal persisted after disabling tf2 detection"


class TestPubsubExampleUnaffectedByR002:
    """A node with no tf2/take mechanism must not gain an R002 record."""

    def test_pubsub_example_generates_clean(self, pubsub_system_json, tmp_path, capsys):
        rc = _generate(pubsub_system_json, tmp_path)
        assert rc == 0
        err = capsys.readouterr().err
        assert "RCLLF-R002" not in err

    def test_pubsub_example_resolved_model_has_no_mechanism_records(self, pubsub_system_json, tmp_path):
        import json

        _generate(pubsub_system_json, tmp_path)
        model = json.loads((pubsub_system_json.parent / "resolved_model.json").read_text())
        mechanism_records = [
            r for r in model["records"] if r["property"].startswith("mechanism.")
        ]
        assert mechanism_records == []
        assert model["claim_status"] == "supported"
