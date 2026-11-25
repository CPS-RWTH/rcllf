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
"""A legacy stub port counts its writes at one tag.

LF keeps one value per port per tag. Before the fix every writer set the stub
port to 0, so two reactions of one callback group publishing one topic at one
tag gave one event, and the subscriber reaction took one DDS message for it.
The second message was never taken. Now each writer adds one to the port's
value, the merge reaction of several groups sums the values, and every stub
reader runs its invoke once per counted write.

The tests compile the emitted reaction bodies with a C harness that stubs
``lf_set`` and the binding calls, run one tag and count the invokes.
"""

from __future__ import annotations

import re
import shutil
import subprocess

import pytest

from colcon_lf.generation.lf_generator import PORT_MERGE_KIND, generate_lf_system
from colcon_lf.comm_analysis.models import ServiceConnection
from colcon_lf.node_analysis.models import (
    CallbackInfo, NodeInfo, ServiceClientInfo, ServiceServerInfo,
)

from ..corr_declared import generate_declared
from .test_correspondence_emission import (
    _graph, _pub, _sub, _tc, _two_cb_pub, _two_group_pub,
)

HARNESS = r"""
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
typedef int32_t stub_t;
typedef struct { int is_present; stub_t value; } port_t;
#define U __attribute__((unused))
static int timers, subs, servers;
/* Messages and requests DDS holds; an invoke that finds none runs nothing. */
static int avail = 1000;
U static void* self;
#define lf_set(p, v) ((p)->is_present = 1, (p)->value = (v))
#define lf_print_error_and_exit(...) \
  do { fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); exit(3); } while (0)
/* The last invoke ran its callback, which publishes once on each topic it uses
   (P17e: stub ports count the publishes the binding saw, ros2_lf_topic_sent). */
static int ran;
U static void ros2_lf_invoke_timer(void* s, int r) { (void)s; (void)r; timers++; ran = 1; }
U static int ros2_lf_invoke_sub(void* s, int r, const char* t) {
  (void)s; (void)r; (void)t;
  ran = 0;
  if (avail <= 0) return 0;
  avail--; subs++; ran = 1; return 1;
}
/* A test may script the publish counts (sent_scripted) and read the owed writes
   each subscription invoke received (remaining_log). */
U static int sent_script[8], sent_scripted, sent_idx, remaining_log[8], remaining_n;
U static int ros2_lf_invoke_sub_n(void* s, int r, const char* t, int k) {
  if (remaining_n < 8) remaining_log[remaining_n++] = k;
  return ros2_lf_invoke_sub(s, r, t);
}
U static int ros2_lf_topic_sent(int r, const char* t) {
  (void)r; (void)t;
  return sent_scripted ? sent_script[sent_idx++ & 7] : ran;
}
U static void ros2_lf_invoke_srv_send(void* s, int r, const char* t) {
  (void)s; (void)r; (void)t; timers++; ran = 1;
}
/* Requests the last send callback really sent (review round 5, O1-2): one per
   run, or none when a test sets svc_unsent (the callback chose not to send). */
U static int svc_unsent;
U static int ros2_lf_service_sent(int r, const char* t) {
  (void)r; (void)t;
  return svc_unsent ? 0 : ran;
}
U static int ros2_lf_invoke_srv_server(void* s, int r, const char* t) {
  (void)s; (void)r; (void)t;
  if (avail <= 0) return 0;
  avail--; servers++; return 1;
}
U static int ros2_lf_invoke_external_sub(void* s, int r, int64_t rec) {
  (void)s; (void)r; subs += (int)rec; return (int)rec;
}
/* an admitted edge's drain runs the callback, which publishes as
   scripted (ros2_lf_topic_sent); lf_schedule is never reached (w = -1). */
static int drains;
U static void* corr_drain_e0;
U static int64_t ros2_lf_corr_drain(void* s, int e, int r, const char* t) {
  (void)s; (void)e; (void)r; (void)t; drains++; ran = 1; return -1;
}
U static int lf_schedule(void* a, int64_t w) { (void)a; (void)w; return 1; }
U static void ros2_lf_corr_schedule_refused(int e) { (void)e; }
U static int64_t ros2_lf_corr_claim_wake(int e, int64_t w) { (void)e; return w; }
int main(int argc, char** argv) {
  (void)argc; (void)argv;
BODY
  printf("timers=%d subs=%d servers=%d\n", timers, subs, servers);
  return 0;
}
"""


def _cc():
    cc = shutil.which("gcc") or shutil.which("cc")
    if cc is None:
        pytest.skip("no C compiler on PATH")
    return cc


def _exec(tmp_path, body: str) -> subprocess.CompletedProcess:
    """Compile *body* into the harness and run it."""
    body = re.sub(r"\bRID_\w+", "0", body)
    src = tmp_path / "tag.c"
    src.write_text(HARNESS.replace("BODY", body))
    exe = tmp_path / "tag"
    subprocess.run([_cc(), "-Wall", "-Werror", "-o", str(exe), str(src)], check=True,
                   timeout=120)
    return subprocess.run([str(exe)], capture_output=True, text=True, timeout=30)


def _run(tmp_path, body: str) -> str:
    """Compile *body* into the harness, run it and return its stdout."""
    proc = _exec(tmp_path, body)
    assert proc.returncode == 0, proc
    return proc.stdout.strip()


def _scope(ports: dict[str, str], bodies: list[str]) -> str:
    """One block that binds each port name to a harness port, then runs *bodies*."""
    decls = "".join(f"  port_t* {name} = &{var}; (void){name};\n" for name, var in ports.items())
    runs = "".join("  {\n" + body + "\n  }\n" for body in bodies)
    return "  {\n" + decls + runs + "  }\n"


def _reactor(system, name):
    (reactor,) = [r for r in system.reactors if r.name == name]
    return reactor


def _sub_reaction(system):
    (reaction,) = [r for r in _reactor(system, "SubNode_DefaultGroup").reactions
                   if r.kind == "sub"]
    return reaction


def test_one_group_two_writers_reach_the_subscriber_twice(tmp_path):
    # Two timers of the default group publish 'data' at one tag. The
    # subscriber's reaction must run invoke_sub twice, not once.
    graph = _graph([_two_cb_pub(), _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    pub_group = _reactor(system, "PubNode_DefaultGroup")
    writers = [r for r in pub_group.reactions if "data" in r.effects]
    assert [r.kind for r in writers] == ["timer", "timer"], pub_group.reactions
    sub = _sub_reaction(system)
    (trigger,) = [t for t in sub.triggers if t != "startup"]
    body = (
        "  port_t out = {0, 0}, in = {0, 0};\n"
        + _scope({"data": "out"}, [r.body for r in writers])
        # The connection pub.data -> sub.<trigger> carries the value.
        + "  in = out;\n"
        + _scope({trigger: "in"}, [sub.body])
        + '  printf("value=%d ", out.value);\n'
    )
    assert _run(tmp_path, body) == "value=2 timers=2 subs=2 servers=0", sub.body


@pytest.mark.parametrize("kind", ["MutuallyExclusive", "Reentrant"])
def test_merge_sums_the_group_counts_into_one_event(tmp_path, kind):
    # The merge: one group wrote twice (value 2), the other once.
    # The outer port carries one event with the sum, with no logical action
    # left to spread writes over microsteps (an `after d` link maps those to
    # one tag and loses all but one).
    graph = _graph([_two_group_pub(kind), _sub("sub", "SubNode")],
                   [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    outer = _reactor(system, "PubNode")
    (merge,) = outer.reactions
    assert merge.kind == PORT_MERGE_KIND
    assert outer.logical_actions == [], outer.logical_actions
    srcs = merge.triggers
    assert len(srcs) == 2 and merge.effects == ["data"], merge
    a, b = (s.split(".")[0] for s in srcs)
    body = (
        "  port_t pa = {1, 2}, pb = {1, 1}, out = {0, 0};\n"
        f"  struct {{ port_t* data; }} {a} = {{&pa}}, {b} = {{&pb}};\n"
        + _scope({"data": "out"}, [merge.body])
        + '  printf("present=%d value=%d ", out.is_present, out.value);\n'
    )
    assert _run(tmp_path, body) == "present=1 value=3 timers=0 subs=0 servers=0", merge.body


def test_merge_sum_overflow_exits_instead_of_wrapping(tmp_path):
    # Kept for the sum: INT_MAX writes plus one more exit.
    graph = _graph([_two_group_pub(), _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    outer = _reactor(system, "PubNode")
    (merge,) = outer.reactions
    assert outer.logical_actions == [], outer.logical_actions
    a, b = (s.split(".")[0] for s in merge.triggers)
    body = (
        "  port_t pa = {1, 2147483647}, pb = {1, 1}, out = {0, 0};\n"
        f"  struct {{ port_t* data; }} {a} = {{&pa}}, {b} = {{&pb}};\n"
        + _scope({"data": "out"}, [merge.body])
    )
    proc = _exec(tmp_path, body)
    assert proc.returncode == 3, f"sum wrapped silently: {proc.stdout.strip()}"
    assert "overflows" in proc.stderr, proc.stderr


def test_writer_count_overflow_exits_instead_of_wrapping(tmp_path):
    # Orchestrator review: a writer's add is checked like the merge.
    graph = _graph([_two_cb_pub(), _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    writer = next(r for r in _reactor(system, "PubNode_DefaultGroup").reactions
                  if "data" in r.effects)
    body = "  port_t out = {1, 2147483647};\n" + _scope({"data": "out"}, [writer.body])
    proc = _exec(tmp_path, body)
    assert proc.returncode == 3, f"count wrapped silently: {proc.stdout.strip()}"
    assert "rcllf: write count of port 'data' overflows int" in proc.stderr, proc.stderr


def test_relay_subscriber_forwards_each_taken_message(tmp_path):
    # A subscriber that publishes: three writes in, three invokes and a count
    # of three on its own output.
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode", publishes="out")],
                   [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    sub = _sub_reaction(system)
    (trigger,) = [t for t in sub.triggers if t != "startup"]
    (effect,) = sub.effects
    body = (
        "  port_t h_in = {1, 3}, h_out = {0, 0};\n"
        + _scope({trigger: "h_in", effect: "h_out"}, [sub.body])
        + '  printf("value=%d ", h_out.value);\n'
    )
    assert _run(tmp_path, body) == "value=3 timers=0 subs=3 servers=0", sub.body


def test_relay_counts_only_the_callbacks_that_ran(tmp_path):
    # Three writes in, but DDS holds one message: the other two invokes find
    # nothing and run no callback, so the relay counts one write, not three
    # (a write counted for nothing would make the next subscriber take a
    # message of a later tag).
    graph = _graph([_pub("pub", "PubNode"), _sub("sub", "SubNode", publishes="out")],
                   [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    sub = _sub_reaction(system)
    (trigger,) = [t for t in sub.triggers if t != "startup"]
    (effect,) = sub.effects
    body = (
        "  avail = 1;\n  port_t h_in = {1, 3}, h_out = {0, 0};\n"
        + _scope({trigger: "h_in", effect: "h_out"}, [sub.body])
        + '  printf("present=%d value=%d ", h_out.is_present, h_out.value);\n'
    )
    assert _run(tmp_path, body) == "present=1 value=1 timers=0 subs=1 servers=0", sub.body


def test_service_server_answers_each_counted_request(tmp_path):
    # Two timers of one group send a request at one tag: the server runs twice.
    client = _two_cb_pub()
    client.publishers = []
    for cb in client.callbacks:
        cb.publishers_used = []
        cb.service_clients_used = ["svc"]
    client.service_clients = [ServiceClientInfo(service_name="svc", service_type="x::srv::Y",
                                                call_style="async_callback")]
    server = NodeInfo(
        name="srv", class_name="SrvNode", package="test_pkg", source_file="/fake/srv.cpp",
        callbacks=[CallbackInfo(name="handle")],
        service_servers=[ServiceServerInfo(service_name="svc", service_type="x::srv::Y",
                                           callback="handle")],
    )
    graph = _graph([client, server], [],
                   [ServiceConnection(service="/svc", service_type="x::srv::Y",
                                      server="srv", clients=["pub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    senders = [r for r in system.reactors for r in r.reactions if r.kind == "srv_send"]
    assert len(senders) == 2, senders
    (req,) = {e for r in senders for e in r.effects}
    (serve,) = [r for r in system.reactors for r in r.reactions if r.kind == "srv_server"]
    (resp,) = serve.effects
    body = (
        "  port_t r = {0, 0}, in = {0, 0}, out = {0, 0};\n"
        + _scope({req: "r"}, [s.body for s in senders])
        + "  in = r;\n"
        + _scope({serve.triggers[0]: "in", resp: "out"}, [serve.body])
        + '  printf("resp=%d ", out.value);\n'
    )
    assert _run(tmp_path, body) == "resp=2 timers=2 subs=0 servers=2", serve.body
    # One request lost in DDS: one response, not two.
    short = body.replace("  port_t r =", "  avail = 1;\n  port_t r =", 1)
    assert _run(tmp_path, short) == "resp=1 timers=2 subs=0 servers=1", serve.body
    # O1-2: a send callback that ran but sent no request writes no request port,
    # so the server runs no take (the old one write per run made it take an
    # outside or later request here).
    unsent = body.replace("  port_t r =", "  svc_unsent = 1;\n  port_t r =", 1)
    assert _run(tmp_path, unsent) == "resp=0 timers=2 subs=0 servers=0", serve.body


def test_timer_writer_adds_to_the_count():
    # The emitted write itself: read back what earlier reactions of this tag
    # set, add the publishes the callback made. A plain lf_set(data, 0) loses
    # the earlier writes. P17f: the added amount is the binding's publish count
    # n, no longer the literal 1. One write per callback run was wrong: a tick
    # that publishes nothing wrote a phantom event (a skip-mode miss downstream,
    # which the take debt rule then took for a late payload) and a tick that
    # publishes twice counted one write (P17d note 6).
    graph = _graph([_two_cb_pub(), _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    for r in _reactor(system, "PubNode_DefaultGroup").reactions:
        if "data" in r.effects:
            assert re.search(
                r'\{ int n = ros2_lf_topic_sent\(RID_\w+, "[^"]*data"\); if \(n > 0\) \{ ',
                r.body), r.body
            assert ("{ stub_t rcllf_c = data->is_present ? data->value : 0; "
                    "if (__builtin_add_overflow(rcllf_c, n, &rcllf_c)) { lf_print_error_and_exit("
                    "\"rcllf: write count of port 'data' overflows int\"); } "
                    "lf_set(data, rcllf_c); }") in r.body, r.body
            assert "lf_set(data, 0);" not in r.body, r.body


def test_stub_type_holds_a_count():
    graph = _graph([_two_cb_pub(), _sub("sub", "SubNode")], [_tc("/data", ["pub"], ["sub"])])
    system = generate_lf_system(graph, after_delay_ns=1_000_000)
    assert "typedef int32_t stub_t;" in system.preamble, system.preamble


def test_drain_that_publishes_nothing_owes_the_legacy_reader_nothing(tmp_path):
    # A subscriber on an admitted edge (its callback runs in the
    # corr drain reaction) publishes on a legacy edge only on some runs, as a
    # message_filters synchronizer publishes only when it completes a pair.
    # Three drains, one publish: the legacy reader must be owed one write. The
    # drain raised the port by one per run before, so the reader took three
    # times and two takes found nothing, which the binding counts as skipped
    # takes with a sample owed (the warning).
    lc = _sub("lc", "LcNode", topic="out", base_class="rclcpp_lifecycle::LifecycleNode")
    lc.subscribers[0].qos_durability = "transient_local"
    graph = _graph(
        [_pub("pub", "PubNode"), _sub("relay", "Relay", publishes="out"), lc],
        [_tc("/data", ["pub"], ["relay"]), _tc("/out", ["relay"], ["lc"])],
    )
    system = generate_declared(graph, after_delay_ns=1_000_000)
    (drain,) = [r for r in _reactor(system, "Relay_DefaultGroup").reactions
                if r.corr_drain_edge is not None]
    assert "out" in drain.effects, drain
    (reader,) = [r for r in _reactor(system, "LcNode_DefaultGroup").reactions if r.kind == "sub"]
    (trigger,) = [t for t in reader.triggers if t != "startup"]
    body = (
        "  sent_scripted = 1; sent_script[0] = 1; sent_script[1] = 0; sent_script[2] = 0;\n"
        "  avail = 1;\n  port_t h_out = {0, 0}, h_in = {0, 0};\n"
        + _scope({"out": "h_out"}, [drain.body, drain.body, drain.body])
        + "  h_in = h_out;\n"
        + _scope({trigger: "h_in"}, [reader.body])
        + '  printf("drains=%d value=%d takes=%d ", drains, h_out.value, remaining_n);\n'
    )
    out = _run(tmp_path, body)
    m = re.match(r"drains=(\d+) value=(\d+) takes=(\d+) ", out)
    assert m, out
    drains, value, takes = (int(g) for g in m.groups())
    assert drains == 3, out
    assert value == 1 and takes == 1, (
        f"3 drains published 1 sample on '/out', but the drain raised the port "
        f"by {value}, so the legacy reader takes {takes} times and {takes - 1} of them are "
        f"skipped takes with a sample owed (the skipped-take warning); drain body:\n{drain.body}")
