# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""One topic published from several group sub-reactors.

Before the fix every publishing sub-reactor forwarded into the node's one
outer port, and lfc rejected the program with "Port named 'data' may only
appear once on the right side of a connection". The node reactor now merges
the writers in one reaction that writes the outer port once per tag with the
summed write count.
"""

from __future__ import annotations

import re
import shutil
import subprocess

import pytest

from colcon_lf.generation.binding_header_emitter import collect_rid_table
from colcon_lf.generation.lf_emitter import emit_lf_system
from colcon_lf.generation.lf_generator import PORT_MERGE_KIND, generate_lf_system
from colcon_lf.node_analysis.models import CallbackInfo, PublisherInfo

from .test_correspondence_emission import MSG, _graph, _reactor_block, _sub, _tc, _two_group_pub

# Sub-reactor instances that publish ``data``, per group kind.
WRITERS = {"MutuallyExclusive": ["ga", "gb"], "Reentrant": ["tick", "tock"]}


def _lfc(tmp_path, text: str) -> subprocess.CompletedProcess:
    lfc = shutil.which("lfc")
    if lfc is None:
        pytest.skip("lfc not on PATH (run in the dev container)")
    src = tmp_path / "src" / "Bringup.lf"
    src.parent.mkdir(parents=True)
    src.write_text(text)
    return subprocess.run([lfc, "--no-compile", str(src)], capture_output=True, text=True,
                          timeout=300)


def _system(kind):
    graph = _graph([_two_group_pub(kind), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    return graph, generate_lf_system(graph, after_delay_ns=1_000_000)


@pytest.mark.parametrize("kind", sorted(WRITERS))
def test_lfc_accepts_two_writers_of_one_topic(tmp_path, kind):
    _, system = _system(kind)
    text = emit_lf_system(system)
    proc = _lfc(tmp_path, text)
    errors = [ln for ln in (proc.stdout + proc.stderr).splitlines() if "rror" in ln]
    assert proc.returncode == 0, "lfc rejected the program:\n" + "\n".join(errors) + "\n" + text


@pytest.mark.parametrize("kind", sorted(WRITERS))
def test_one_merge_reaction_writes_the_outer_port(kind):
    _, system = _system(kind)
    text = emit_lf_system(system)
    block = _reactor_block(text, "PubNode")
    into_data = re.findall(r"^\s*\S+ -> data\s*$", block, flags=re.M)
    assert into_data == [], f"{kind}: forwarding into 'data' left:\n{block}"
    (outer,) = [r for r in system.reactors if r.name == "PubNode"]
    (merge,) = outer.reactions
    writers = [f"{w}.data" for w in WRITERS[kind]]
    assert merge.kind == PORT_MERGE_KIND
    assert merge.triggers == writers, merge
    assert merge.effects == ["data"], merge
    # One event per tag carrying the summed count, no logical
    # action spreading the writes over microsteps.
    assert outer.logical_actions == []
    assert "pub.data -> sub.data after 1 msec" in text, text


@pytest.mark.parametrize("kind", sorted(WRITERS))
def test_merge_reaction_takes_no_rid(kind):
    graph, system = _system(kind)
    table = collect_rid_table({r.name: r for r in system.reactors}, graph)
    callbacks = sorted(row[2] for row in table)
    assert callbacks == ["on_data", "tick", "tock"], table


@pytest.mark.parametrize("kind", sorted(WRITERS))
def test_merge_beside_a_port_named_like_the_old_action(tmp_path, kind):
    # A node that also publishes 'data_merge' declared the
    # name the merge action used to take. The merge has no
    # action; the program must still compile.
    pub = _two_group_pub(kind)
    pub.publishers.append(PublisherInfo(topic="data_merge", message_type=MSG))
    pub.callbacks[0].publishers_used.append("data_merge")
    graph = _graph([pub, _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    text = emit_lf_system(system)
    proc = _lfc(tmp_path, text)
    errors = [ln for ln in (proc.stdout + proc.stderr).splitlines() if "rror" in ln]
    assert proc.returncode == 0, "lfc rejected the program:\n" + "\n".join(errors) + "\n" + text
    (outer,) = [r for r in system.reactors if r.name == "PubNode"]
    assert outer.logical_actions == [], outer.logical_actions


@pytest.mark.parametrize("kind", sorted(WRITERS))
def test_untriggered_helper_is_no_merge_writer(tmp_path, kind):
    # A helper that lambda timers call is analyzed as a callback
    # with the lambdas' publishers but has no trigger, so it lands in the
    # default group without a reaction. Its group's 'data' port had no writer,
    # lfc left the merge reaction's pointer to it NULL in a federate, and the
    # publisher segfaulted on the first tag with two writes.
    pub = _two_group_pub(kind)
    pub.callbacks.append(CallbackInfo(name="helper", publishers_used=["data"]))
    graph = _graph([pub, _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    (outer,) = [r for r in system.reactors if r.name == "PubNode"]
    (merge,) = outer.reactions
    assert merge.triggers == [f"{w}.data" for w in WRITERS[kind]], merge
    for sub in system.reactors:
        written = {e for r in sub.reactions for e in r.effects}
        dangling = [p.name for p in sub.ports if p.direction == "output"
                    and p.name not in written and sub.name.startswith("PubNode_")]
        assert dangling == [], f"{sub.name}: output ports nobody writes: {dangling}"
    text = emit_lf_system(system)
    proc = _lfc(tmp_path, text)
    errors = [ln for ln in (proc.stdout + proc.stderr).splitlines() if "rror" in ln]
    assert proc.returncode == 0, "lfc rejected the program:\n" + "\n".join(errors) + "\n" + text


# The merge's sum and its overflow check: tests/generation/test_stub_write_count.py.
