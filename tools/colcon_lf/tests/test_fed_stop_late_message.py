# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""A message from the RTI tagged past the stop tag ends no federate.

Under centralized coordination every message reaches a federate through the RTI
connection. A sender's last message before the stop may carry a tag past the
stop tag (an ``after`` delay), and the RTI may grant the receiver its stop tag
first, because the delay allows it. reactor-c's ``handle_tagged_message``
(``core/federated/federate.c``) then closed the RTI connection, and the RTI
listener thread called ``lf_print_error_and_exit``: exit() while the main
thread still ran its shutdown. A plain LF federate exits 1 after a correct
stop; an rcllf federate, whose atexit list holds the ROS 2 teardown, aborts
(exit -6) or segfaults (exit -11). The fix is the local patch
``third_party/patches/reactor-c/0005-*.patch``.

The program makes the race certain. The sender requests the stop at tag 0
and then holds its environment mutex, so its RTI listener cannot apply the
stop grant: to the sender the grant is still in flight, as in the board runs.
The RTI grants the stop to the receiver, which reaches its stop tag and holds
it in a shutdown reaction. The sender then sends a message 50 ms after tag 0,
past the stop tag, which its own stop check lets through. The federates run
one by one beside their own RTI, so each exit code is read.

The order is forced, not timed. The sender enters its critical section
before ``lf_request_stop()``, which sends the stop request without the
environment mutex: entered after it, a loaded host let the RTI round trip
finish first, the sender applied its stop grant, and its own stop check
dropped the 50 ms message silently (no discard line, test red, fix never
reached). The sender sends only once the receiver's shutdown reaction (at its
stop tag) wrote the ``at_stop`` marker, and the receiver leaves its stop tag
only 1 s after the sender wrote ``sent``, the time the RTI has to forward one
message already sent. ``SEND_DROPPED`` names a send refused at the sender.
"""

from __future__ import annotations

import os
import shutil
import socket
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
PATCH_LFC = REPO / "third_party" / "patches" / "patch_lfc_runtime.py"

PROGRAM = """\
target C { logging: warn, coordination: centralized }
preamble {=
  #include <stdbool.h>
  #include <stdio.h>
  #include <stdlib.h>
  #include <time.h>
  #include <unistd.h>
  static void marker_path(const char* name, char* buf, size_t n) {
    snprintf(buf, n, "%s/%s", getenv("LATE_STOP_DIR"), name);
  }
  static void touch_marker(const char* name) {
    char p[4096];
    marker_path(name, p, sizeof p);
    FILE* f = fopen(p, "w");
    if (f) fclose(f);
  }
  // Up to 30 s; the order, not this bound, is what the test relies on.
  static bool wait_for_marker(const char* name) {
    char p[4096];
    marker_path(name, p, sizeof p);
    struct timespec d = {0, 10000000};
    for (int i = 0; i < 3000; i++) {
      if (access(p, F_OK) == 0) return true;
      nanosleep(&d, NULL);
    }
    return false;
  }
=}
reactor Src {
  preamble {=
    #include "federate.h"
  =}
  output out: int
  reaction(startup) -> out {=
    // Hold the mutex the RTI listener needs to apply the stop grant from
    // before the stop request leaves: lf_request_stop() sends it without
    // the mutex, and a grant applied before this point makes the send below a
    // silent drop at the sender. The mutex is recursive; lf_request_stop() only
    // locks and unlocks it.
    lf_critical_section_enter(self->base.environment);
    lf_request_stop();
    // Send only once the receiver is at its stop tag (an order, not a delay).
    if (!wait_for_marker("at_stop")) {
      printf("NO_AT_STOP\\n");
    }
    // What the network sender for `out` would send (port 0, federate 1).
    int v = 7;
    int rc = lf_send_tagged_message(self->base.environment, MSEC(50),
                                    MSG_TYPE_TAGGED_MESSAGE, 0, 1, "federate 1 via the RTI",
                                    sizeof(int), (unsigned char*)&v);
    touch_marker("sent");
    lf_critical_section_exit(self->base.environment);
    printf(rc == 0 ? "SENT\\n" : "SEND_DROPPED %d\\n", rc);
    fflush(stdout);
  =}
}
reactor Dst {
  input in: int
  reaction(in) {= printf("RECV %d\\n", in->value); =}
  reaction(shutdown) {=
    // Stay at the stop tag while the sender's message arrives: until the
    // sender sent it, then 1 s for the RTI to forward it.
    touch_marker("at_stop");
    if (!wait_for_marker("sent")) {
      printf("NO_SENT\\n");
    }
    struct timespec d = {1, 0};
    nanosleep(&d, NULL);
    printf("SHUTDOWN %lld\\n", (long long)lf_time_logical_elapsed());
    fflush(stdout);
  =}
}
federated reactor {
  s = new Src()
  d = new Dst()
  s.out -> d.in after 50 msec
}
"""


@pytest.fixture(scope="module")
def patched_lfc(tmp_path_factory) -> Path:
    lfc = shutil.which("lfc")
    if lfc is None:
        pytest.skip("lfc not on PATH (run in the dev container)")
    if shutil.which("patch") is None:
        pytest.skip("patch not available")
    lfc_dir = Path(lfc).resolve().parent.parent
    copy = tmp_path_factory.mktemp("lfc") / "lfc"
    shutil.copytree(lfc_dir, copy, symlinks=True)
    subprocess.run([sys.executable, str(PATCH_LFC), str(copy)], check=True, timeout=300)
    return copy / "bin" / "lfc"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_message_past_stop_tag_from_rti_is_discarded(patched_lfc, tmp_path):
    src = tmp_path / "src" / "LateStop.lf"
    src.parent.mkdir()
    src.write_text(PROGRAM)
    build = subprocess.run([str(patched_lfc), str(src)], capture_output=True, text=True,
                           timeout=600)
    assert build.returncode == 0, build.stdout + build.stderr
    # lfc writes the federates and an RTI built from the same runtime here.
    bindir = tmp_path / "fed-gen" / "LateStop" / "bin"
    port = _free_port()
    markers = tmp_path / "markers"
    markers.mkdir()
    env = {**os.environ, "LATE_STOP_DIR": str(markers)}
    rti = subprocess.Popen([str(bindir / "RTI"), "-n", "2", "-p", str(port), "-i", "late"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    feds = {name: subprocess.Popen([str(bindir / f"federate__{name}"), "-r", f"localhost:{port}",
                                    "-i", "late"],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   env=env)
            for name in ("s", "d")}
    out = {}
    try:
        for name, p in feds.items():
            out[name] = p.communicate(timeout=60)[0]
        rti_out = rti.communicate(timeout=30)[0]
    finally:
        for p in (*feds.values(), rti):
            if p.poll() is None:
                p.kill()
    log = "".join(f"--- federate {n} (exit {feds[n].returncode})\n{o}" for n, o in out.items())
    log += f"--- RTI (exit {rti.returncode})\n{rti_out}"
    assert "SENT" in out["s"], log
    # The order the race needs held (the sender saw the receiver at its
    # stop tag, the receiver saw the send).
    assert "NO_AT_STOP" not in out["s"] and "NO_SENT" not in out["d"], log
    assert "Failed to complete the reading of a message from the RTI" not in out["d"], (
        "the receiver closed its RTI connection on a message past its stop tag and "
        "exited from the RTI listener thread\n" + log)
    # The race happened: the receiver was at its stop tag when the message came.
    assert "SHUTDOWN 0" in out["d"], log
    assert "RECV" not in out["d"], log
    assert "Discarding a message from the RTI tagged past the stop tag" in out["d"], log
    assert feds["d"].returncode == 0, log
    assert feds["s"].returncode == 0, log
