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
"""A legacy stub port counts publishes, not callback runs (probe P17e, fix B).

Before the fix a timer callback that ran but published nothing still set its
stub port, so every subscriber downstream ran a take for a sample that never
existed (a skip-mode miss, or a block-mode timeout on every empty tick), and a
callback that published twice raised the port by one. With
``RCLLF_COUNT_PUBLISHES=1`` the port carries the publishes the binding saw,
and the subscription invoke receives the writes its trigger still owes.

The bodies run in the C harness of test_stub_write_count, which declares the
two binding calls and lets a test script the publish counts. Under the old
generator the same harness compiles and the tests fail with the old counts.
"""

from __future__ import annotations

import pytest

from colcon_lf.generation.lf_generator import generate_lf_system

from .test_correspondence_emission import _graph, _pub, _sub, _tc
from .test_stub_write_count import _scope
from . import test_stub_write_count as stub



@pytest.fixture(autouse=True)
def _count_publishes(monkeypatch, tmp_path):
    # The default since P17f, set explicitly so the tests also state it.
    monkeypatch.setenv("RCLLF_COUNT_PUBLISHES", "1")


def _reaction(system, kind, port):
    found = [r for reactor in system.reactors for r in reactor.reactions
             if r.kind == kind and port in r.effects]
    assert len(found) == 1, found
    return found[0]


def _timer_tag(tmp_path, published: int) -> str:
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    timer = _reaction(system, "timer", "data")
    body = (
        f"  sent_scripted = 1; sent_script[0] = {published};\n  port_t out = {{0, 0}};\n"
        + _scope({"data": "out"}, [timer.body])
        + '  printf("present=%d value=%d ", out.is_present, out.value);\n'
    )
    return stub._run(tmp_path, body), timer.body


def test_timer_that_published_nothing_writes_no_event(tmp_path):
    out, body = _timer_tag(tmp_path, 0)
    assert out == "present=0 value=0 timers=1 subs=0 servers=0", (
        f"an empty tick wrote the stub port, a phantom write: {out}\n{body}")


def test_timer_that_published_twice_counts_two_writes(tmp_path):
    out, body = _timer_tag(tmp_path, 2)
    assert out == "present=1 value=2 timers=1 subs=0 servers=0", (
        f"two publishes counted as one callback run: {out}\n{body}")


def test_relay_passes_the_remaining_writes_and_counts_its_publishes(tmp_path):
    # Two writes in. The relay runs twice, the first invoke owes 2 writes
    # (this one included), the second 1. The first callback published twice,
    # the second not at all: the output carries 2.
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode", publishes="out")],
                   [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    sub = _reaction(system, "sub", "out")
    (trigger,) = [t for t in sub.triggers if t != "startup"]
    body = (
        "  sent_scripted = 1; sent_script[0] = 2; sent_script[1] = 0;\n"
        "  port_t h_in = {1, 2}, h_out = {0, 0};\n"
        + _scope({trigger: "h_in", "out": "h_out"}, [sub.body])
        + '  printf("value=%d remaining=%d,%d ", h_out.value, remaining_log[0], '
          "remaining_log[1]);\n"
    )
    out = stub._run(tmp_path, body)
    assert out == "value=2 remaining=2,1 timers=0 subs=2 servers=0", (
        f"the relay counted callback runs or did not pass the owed writes: {out}\n{sub.body}")
