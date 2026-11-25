# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``colcon lf-generate`` warns about an inert ``take_timeout_ms`` only where the
take never waits.

The generator once warned that a set ``take_timeout_ms`` under
``take_mode: skip`` is inert on every legacy edge (the admitted edges, whose
join waits up to it, were already excluded). Now a
skip take waits up to the timeout for a sample its trigger counted, in one
process and across federates alike, so that warning was wrong. The one
subscription the generator knows never waits is an ``external_topics`` one:
its message enters as a physical action the ingress thread has already taken
when the reaction runs.
"""

from __future__ import annotations

import textwrap

import pytest

from colcon_lf.output.serializer import serialize

from ..generation.test_correspondence_emission import _graph, _pub, _sub, _tc

INERT = "the timeout is inert"
OLD_WARNING = "'take_timeout_ms' is set but resolved 'take_mode' is 'skip'"


@pytest.fixture(autouse=True)
def _no_lfc_package_emission(monkeypatch):
    """The .lf text and the warnings come before the lfc package, which stages
    the user's C++ sources this synthetic graph does not have."""
    import colcon_lf.verb.generate as gen

    monkeypatch.setattr(gen, "_emit_lfc_artifacts", lambda **kwargs: "lf_generated_test")


def _generate(tmp_path, capfd, yaml_text: str, federate: str | None = None,
              external: bool = False):
    from colcon_lf.verb.generate import _generate_one

    nodes = [_sub("sub", "SubNode")] if external else [_pub("pub", "PubNode"), _sub("sub", "SubNode")]
    for node in nodes:
        # The federated packages stage each node's source file.
        node.source_file = str(tmp_path / f"{node.name}.cpp")
        (tmp_path / f"{node.name}.cpp").write_text("// test\n")
    graph = _graph(nodes, [_tc("/data", [] if external else ["pub"], ["sub"])])
    system_json = tmp_path / "lf-analysis" / "test_pkg" / "bringup" / "system.json"
    system_json.parent.mkdir(parents=True)
    system_json.write_text(serialize(graph))
    exec_config = tmp_path / "execution.yaml"
    exec_config.write_text(textwrap.dedent(yaml_text))
    fed = None
    if federate is not None:
        fed = tmp_path / "bringup.federation.yaml"
        fed.write_text(textwrap.dedent(federate))
    out = tmp_path / "lf"
    rc = _generate_one(system_json=system_json, output_dir=str(out), after_delay=None,
                       generate_launch=False, exec_config_arg=str(exec_config),
                       federate=str(fed) if fed else None)
    err = capfd.readouterr().err
    assert rc == 0, err
    return err, out


ADMITTED = """\
    nodes:
      pub: {max_publishes_per_invocation: 1}
    subscriptions:
      sub:
        data: {take_timeout_ms: 15}
"""

# No declared burst bound, and the synthetic publisher callback carries no
# analyzer-proven one either: the edge stays legacy (burst_unbounded).
LEGACY = """\
    subscriptions:
      sub:
        data: {take_timeout_ms: 15}
"""

# No publisher in the system: the message comes from outside.
EXTERNAL = """\
    external_topics: [data]
    subscriptions:
      sub:
        data: {take_timeout_ms: 15}
"""

FEDERATION = """\
    system: bringup
    federates:
      front: {nodes: [pub]}
      back: {nodes: [sub]}
    coordination: centralized
"""


def _inert_lines(err: str) -> list[str]:
    return [line.strip() for line in err.splitlines() if INERT in line]


def test_legacy_edge_gets_no_inert_warning(tmp_path, capfd):
    """RED on a generator that warned for legacy edges, with its own warning text."""
    err, out = _generate(tmp_path, capfd, LEGACY)
    (lf_file,) = out.glob("*.lf")
    assert "ros2_lf_corr_emit" not in lf_file.read_text(), "fixture edge is admitted"
    assert not _inert_lines(err), (
        "lf-generate warned that the take_timeout_ms of a LEGACY edge under take_mode skip "
        "is inert, but a skip take waits up to the timeout for a counted sample: "
        f"{_inert_lines(err)}")
    assert OLD_WARNING not in err, err


def test_admitted_edge_gets_no_inert_warning(tmp_path, capfd):
    err, out = _generate(tmp_path, capfd, ADMITTED)
    (lf_file,) = out.glob("*.lf")
    assert "ros2_lf_corr_emit" in lf_file.read_text(), "fixture edge is not admitted"
    assert not _inert_lines(err), (
        "lf-generate warned that the take_timeout_ms of an ADMITTED edge is inert, but its "
        f"join waits up to that timeout: {_inert_lines(err)}")


def test_legacy_cross_federate_edge_gets_no_inert_warning(tmp_path, capfd):
    err, _ = _generate(tmp_path, capfd, LEGACY, federate=FEDERATION)
    assert not _inert_lines(err), (
        "lf-generate --federate warned that the take_timeout_ms of a LEGACY cross-federate "
        f"edge is inert, but a count-link take waits up to it: {_inert_lines(err)}")
    assert OLD_WARNING not in err, err


def test_admitted_cross_federate_edge_gets_no_inert_warning(tmp_path, capfd):
    err, _ = _generate(tmp_path, capfd, ADMITTED, federate=FEDERATION)
    assert not _inert_lines(err), (
        "lf-generate --federate warned that the take_timeout_ms of an ADMITTED cross-federate "
        f"edge is inert: {_inert_lines(err)}")


def test_external_topic_gets_the_inert_warning(tmp_path, capfd):
    err, _ = _generate(tmp_path, capfd, EXTERNAL, external=True)
    assert _inert_lines(err) == [
        "Warning: SubNode.data: 'maxwait_ms' is set for an external topic; an "
        "external_topics message is already taken when the reaction runs, so the take never "
        "waits and the timeout is inert. A take on an edge inside the system waits up to the "
        "timeout for a counted sample in skip and block mode alike."
    ], err


def test_external_topic_with_the_default_timeout_gets_no_warning(tmp_path, capfd):
    err, _ = _generate(tmp_path, capfd, "    external_topics: [data]\n", external=True)
    assert not _inert_lines(err), err
