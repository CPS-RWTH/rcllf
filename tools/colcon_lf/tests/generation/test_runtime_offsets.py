# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Timer offsets set at program start.

``nodes.<node>.timers.<timer>.offset_runtime: true`` turns the timer's offset
into a main-reactor parameter (one program) or a federated-reactor parameter
(federation), threaded down to the timer. reactor-c sets such a parameter from
the program option ``--<param> <n> <unit>`` in ``main()``, before the start tag.
The generated launch files map ``RCLLF_TIMER_OFFSETS`` onto that option through
``rcllf_timer_offsets.py`` and refuse a name that is not a run-time timer of the
build: a federate skips an option it does not know, so the launch file is the
only place that can refuse it.

To falsify: make ``timer_offset_args`` skip unknown names (the refusal tests
fail with "DID NOT RAISE"), or drop ``_apply_runtime_offsets`` from
``generate_lf_system`` (the timer keeps its literal offset).
"""

from __future__ import annotations

import dataclasses
import json
import runpy
from pathlib import Path

import pytest

from colcon_lf.federation.partition import FederationMap
from colcon_lf.generation import rcllf_timer_offsets as rto
from colcon_lf.verb.generate import _generate_federated, _generate_one

from ..corr_declared import declared
from .test_correspondence_emission import _graph, _pub, _sub, _tc
from .test_fast_mode import PKG

FED = "lf_generated_test_pkg_bringup_federated"
PARAM = "test_node__t_timer_callback_offset"


# ---------------------------------------------------------------------------
# The launch-side helper
# ---------------------------------------------------------------------------

TABLE = {"controller/control_callback": "controller__t_control_callback_offset",
         "radar/scan_callback": "radar__t_scan_callback_offset"}


def test_unset_or_empty_value_adds_no_option():
    assert rto.timer_offset_args(TABLE, text="") == []
    assert rto.timer_offset_args({}, text=" , ") == []


@pytest.mark.parametrize("value, option", [
    ("5 msec", ["5", "msec"]), ("12ms", ["12", "msec"]), ("0", ["0", "sec"]),
    ("2 sec", ["2", "sec"]), ("1500 usec", ["1500", "usec"]), ("7 ns", ["7", "nsec"]),
    # 3 s in nsec overflows reactor-c's int parse; the coarsest exact unit fits.
    ("3000000000 nsec", ["3", "sec"]),
])
def test_value_becomes_the_coarsest_exact_option(value, option):
    args = rto.timer_offset_args(TABLE, text=f"controller/control_callback={value}")
    assert args == ["--controller__t_control_callback_offset"] + option


def test_two_entries_and_a_leading_slash():
    args = rto.timer_offset_args(
        TABLE, text="/radar/scan_callback=1 msec; controller/control_callback=12 msec")
    assert args == ["--radar__t_scan_callback_offset", "1", "msec",
                    "--controller__t_control_callback_offset", "12", "msec"]


def test_unknown_timer_is_refused_and_the_known_ones_are_named():
    with pytest.raises(RuntimeError) as exc:
        rto.timer_offset_args(TABLE, text="controler/control_callback=5 msec")
    msg = str(exc.value)
    assert "no run-time timer 'controler/control_callback' in this build" in msg
    assert "controller/control_callback, radar/scan_callback" in msg


def test_any_name_is_refused_by_a_build_without_run_time_timers():
    with pytest.raises(RuntimeError, match="none \\(no timer of this build has offset_runtime"):
        rto.timer_offset_args({}, text="controller/control_callback=5 msec")


@pytest.mark.parametrize("text, match", [
    ("controller/control_callback", "is not '<node>/<timer>=<offset>'"),
    ("controller/control_callback=5", "needs a unit"),
    ("controller/control_callback=5 days", "unknown unit"),
    ("controller/control_callback=-5 msec", "is not '<integer> <unit>'"),
    ("controller/control_callback=1 msec,controller/control_callback=2 msec", "given twice"),
])
def test_malformed_entries_are_refused(text, match):
    with pytest.raises(RuntimeError, match=match):
        rto.timer_offset_args(TABLE, text=text)


def test_a_federate_takes_only_its_own_options_but_checks_all_names():
    own = {"radar__t_scan_callback_offset"}
    text = "controller/control_callback=5 msec, radar/scan_callback=1 msec"
    assert rto.timer_offset_args(TABLE, own, text) == ["--radar__t_scan_callback_offset", "1",
                                                       "msec"]
    assert rto.timer_offset_args(TABLE, set(), text) == []
    with pytest.raises(RuntimeError, match="no run-time timer 'brake/x'"):
        rto.timer_offset_args(TABLE, own, "brake/x=1 msec")


def test_the_environment_variable_is_read_by_default(monkeypatch):
    monkeypatch.setenv(rto.ENV, "controller/control_callback=1 msec")
    assert rto.timer_offset_args(TABLE) == ["--controller__t_control_callback_offset", "1",
                                            "msec"]


# ---------------------------------------------------------------------------
# The one program
# ---------------------------------------------------------------------------

def _one_program(tmp_path: Path, timer: dict | None) -> tuple[str, str, Path]:
    """Generate the one-timer program of test_fast_mode with *timer* as exec-config."""
    from .test_fast_mode import _one_program as gen

    if timer is None:
        lf_text, launch = gen(tmp_path)
    else:
        tmp_path.mkdir(parents=True, exist_ok=True)
        cfg = tmp_path / "w6.execution.yaml"
        cfg.write_text(json.dumps({"nodes": {"test_node": {"timers": {"timer_callback": timer}}}}))
        lf_text, launch = gen(tmp_path, exec_config_arg=str(cfg))
    return lf_text, launch, tmp_path / "packages/lf-generated" / PKG / "launch"


def test_one_program_threads_the_parameter_from_main_to_the_timer(tmp_path):
    lf_text, launch, _ = _one_program(tmp_path, {"offset_runtime": True})
    assert f"main reactor Bringup({PARAM}: time = 0) {{" in lf_text
    assert f"(t_timer_callback_offset = {PARAM})" in lf_text
    assert "reactor TestNode(t_timer_callback_offset: time = 0) {" in lf_text
    assert "timer t_timer_callback(t_timer_callback_offset, 500 msec)" in lf_text
    assert f"{{'test_node/timer_callback': '{PARAM}'}}, None)" in launch
    assert "+ offset_args\n" in launch


def test_the_baked_offset_is_the_parameter_default(tmp_path):
    lf_text, _, _ = _one_program(tmp_path, {"offset_runtime": True, "offset": "12 msec"})
    assert f"main reactor Bringup({PARAM}: time = 12 msec) {{" in lf_text


def test_without_offset_runtime_the_program_is_unchanged(tmp_path):
    plain, _, _ = _one_program(tmp_path / "a", None)
    off, _, _ = _one_program(tmp_path / "b", {"offset_runtime": False})
    assert off == plain
    assert "main reactor Bringup {" in plain
    assert "timer t_timer_callback(0, 500 msec)" in plain
    assert "_offset" not in plain


def test_offset_runtime_must_be_a_bool(tmp_path):
    from colcon_lf.exec_config.model import ExecConfig

    from ..exec_config.conftest import make_graph, make_node

    graph = make_graph({"n": make_node("n", timers=["timer_callback"])})
    ok = ExecConfig.from_dict(
        {"nodes": {"n": {"timers": {"timer_callback": {"offset_runtime": True}}}}},
        graph, source="test")
    assert ok.nodes["n"]["timers"]["timer_callback"] == {"offset_runtime": True}
    with pytest.raises(ValueError, match="'offset_runtime' must be true or false"):
        ExecConfig.from_dict(
            {"nodes": {"n": {"timers": {"timer_callback": {"offset_runtime": "yes"}}}}},
            graph, source="test")


def _launch_env(monkeypatch):
    """Evaluate a generated launch file outside an install tree."""
    pytest.importorskip("launch_ros")
    packages = pytest.importorskip("ament_index_python.packages")
    monkeypatch.setattr(packages, "get_package_share_directory", lambda pkg: "/nonexistent")


def test_one_program_launch_refuses_an_unknown_name(tmp_path, monkeypatch):
    _, _, launch_dir = _one_program(tmp_path, {"offset_runtime": True})
    assert (launch_dir / "rcllf_timer_offsets.py").is_file()
    _launch_env(monkeypatch)
    monkeypatch.setenv(rto.ENV, "test_nod/timer_callback=5 msec")
    with pytest.raises(RuntimeError, match="no run-time timer 'test_nod/timer_callback'"):
        runpy.run_path(str(launch_dir / "lf_bringup.launch.py"))[
            "generate_launch_description"]()


def test_a_build_without_run_time_timers_refuses_every_name(tmp_path, monkeypatch):
    _, launch, launch_dir = _one_program(tmp_path, None)
    assert '"timer_offset_args"]({}, None)' in launch
    _launch_env(monkeypatch)
    monkeypatch.setenv(rto.ENV, "test_node/timer_callback=5 msec")
    with pytest.raises(RuntimeError, match="Run-time timers: none"):
        runpy.run_path(str(launch_dir / "lf_bringup.launch.py"))[
            "generate_launch_description"]()


# ---------------------------------------------------------------------------
# The federation
# ---------------------------------------------------------------------------

def _federation(tmp_path: Path, runtime_offsets) -> dict:
    src = tmp_path / "packages/src/test_pkg/src"
    src.mkdir(parents=True, exist_ok=True)
    nodes = []
    for n in (_pub("pub", "PubNode"), _sub("sub", "SubNode")):
        f = src / f"{n.name}.cpp"
        f.write_text("// generated by the test\n")
        nodes.append(dataclasses.replace(n, source_file=str(f)))
    graph = _graph(nodes, [_tc("/data", ["pub"], ["sub"])])
    key = next(k for k, n in graph.nodes.items() if n.name == "pub")
    sj = tmp_path / "lf-analysis/test_pkg/bringup/system.json"
    sj.parent.mkdir(parents=True, exist_ok=True)
    sj.write_text(json.dumps({"name": "bringup", "nodes": {}, "communication": {"topics": []}}))
    fmap = FederationMap(node_to_federate={"pub": "pub", "sub": "sub"},
                         federate_names=["pub", "sub"], after_overrides=[],
                         coordination="centralized")
    rc = _generate_federated(
        system_json=sj, graph=graph, fmap=fmap, after_delay="0 msec", generate_launch=True,
        corr_bounds=declared(graph, 1),
        runtime_offsets={key: {"tick"}} if runtime_offsets else None)
    assert rc == 0
    gen = tmp_path / "lf-generated"
    return {
        "key": key.lstrip("/"),
        "top": (gen / FED / "lf/Bringup_federated.lf").read_text(),
        "pub_lf": (gen / FED / "lf/pub/Pub.lf").read_text(),
        "pub_launch": (gen / f"{FED}_pub/launch/lf_pub.launch.py").read_text(),
        "sub_launch": (gen / f"{FED}_sub/launch/lf_sub.launch.py").read_text(),
    }


def test_federated_reactor_takes_the_parameter_and_passes_it_to_its_federate(tmp_path):
    out = _federation(tmp_path, True)
    param = f"{out['key']}__t_tick_offset"
    assert f"federated reactor Bringup_federated({param}: time = 0) {{" in out["top"]
    assert f"pub = new Pub({param} = {param})" in out["top"]
    assert "sub = new Sub()" in out["top"]
    assert "timer t_tick(t_tick_offset, 100 msec)" in out["pub_lf"]
    # Every federate checks the whole federation's names; only pub takes the option.
    table = f"{{'{out['key']}/tick': '{param}'}}"
    assert f"\"timer_offset_args\"]({table}, {{'{param}'}})" in out["pub_launch"]
    assert f"\"timer_offset_args\"]({table}, set())" in out["sub_launch"]
    for text in (out["pub_launch"], out["sub_launch"]):
        assert text.index('args = ["-r"') < text.index("args += offset_args") < text.index(
            "RCLLF_LF_ARGS")
        compile(text, "lf.launch.py", "exec")


def test_federation_without_offset_runtime_is_unchanged(tmp_path):
    out = _federation(tmp_path, False)
    assert "federated reactor Bringup_federated {" in out["top"]
    assert "_offset" not in out["top"]
    assert "timer t_tick(0, 100 msec)" in out["pub_lf"]
    assert '"timer_offset_args"]({}, set())' in out["pub_launch"]
