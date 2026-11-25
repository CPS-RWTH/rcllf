# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The RTI listener's resend of a skipped NET cannot overtake the main thread.

Under centralized coordination a federate that was granted beyond its next
event skips the NET for that event (``last_skipped_NET``). When the RTI later
sends a DNET below the skipped tag, the RTI listener thread resends the skipped
NET in ``handle_downstream_next_event_tag`` (``core/federated/federate.c``) and
clears the bookkeeping. reactor-c did that without ``env->mutex`` while the
main thread reads and writes the same fields in ``lf_send_next_event_tag``
under that mutex. Preempted between its check and its send, the listener sent
a NET the main thread had already superseded (the old skipped tag, or NEVER
once the main thread had cleared it). The RTI records any NET it receives, so
the federate's next event went on record in its past and every federate
upstream or downstream of it through an ``after`` delay waited for a grant
that never came: the federation stalled until the runner's SIGINT
(brake_fusion fed1host under CPU stress, 3 of 150 runs). The fix is the local
patch ``third_party/patches/reactor-c/0007-*.patch``: the handler runs under
``env->mutex``.

The program makes the preemption certain. The fixture puts two test hooks
into the jar's federate.c in the listener's DNET handler (the first five DNETs,
each bounded in time). Hook 1, right after the handler sets
``received_any_DNET`` and before the fix's lock, holds the listener until the
main thread has skipped a NET, which it does on every granted tick while
``received_any_DNET`` is set and ``last_DNET`` is still NEVER, so the check
sees a pending skipped NET. Hook 2, between the check and the send, holds the
listener until the main thread has sent a NET beyond its last grant and waits
for the RTI. The listener's send then follows the main thread's send on
the same socket. ``a`` and ``b`` have 1 ms timers and are connected both ways
through ``after`` delays of 4999999 ns that carry no message, so there is no
LTC-based grant and the RTI grants each federate up to the other's next event
plus the delay. ``a`` sleeps 5 ms per tick for its first 8 ms, so ``b`` runs
ahead to its bound (``a``'s recorded NET plus 5 ms) and the RTI sends ``a`` its
first DNET; ``b`` sleeps 2 ms per tick throughout, so after ``a``'s new NET
releases ``b`` the RTI has read ``a``'s stale NET long before ``b``'s next NET
could grant ``a``. Without the fix the listener reads NEVER (the main thread's
clear) and sends NET NEVER after the newer NET; the RTI records it, can grant
``a`` nothing and ``b`` only up to the start tag plus the delay, and both wait
until the test kills them. With the fix hook 1 still releases (it runs before the
lock), hook 2 times out under the mutex because the main thread cannot send
while the listener holds it, the resend carries the tag the check saw, and the
run completes.
"""

from __future__ import annotations

import shutil
import socket
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
PATCH_LFC = REPO / "third_party" / "patches" / "patch_lfc_runtime.py"
FEDERATE_C = "lib/c/reactor-c/core/federated/federate.c"
HOOK1_ANCHOR = "  _fed.received_any_DNET = true;\n"
HOOK_ANCHOR = "  if (lf_tag_compare(DNET, _fed.last_skipped_NET) < 0) {\n"
HOOK_BEFORE = """\
    { // rcllf test hook 1: hold the listener (at most 50 ms, the first
      // 5 DNETs) until the main thread has skipped a NET, so that the check
      // below finds a pending skipped NET.
      static int hook1_count = 0;
      if (hook1_count < 5) {
        hook1_count++;
        instant_t deadline = lf_time_physical() + MSEC(50);
        while (lf_time_physical() < deadline &&
               lf_tag_compare(_fed.last_skipped_NET, NEVER_TAG) == 0) {
        }
      }
    }
"""

HOOK_AFTER = """\
    { // rcllf test hook 2: the listener thread is held here, between
      // its check of last_skipped_NET and its send, until the main thread has
      // sent a NET beyond its last TAG and waits for the RTI (at most 200 ms,
      // the first 5 resends). It spins: the stale NET has to reach the RTI
      // before the peer answers the new NET.
      static int hook_count = 0;
      if (hook_count < 5) {
        hook_count++;
        tag_t seen = _fed.last_sent_NET;
        instant_t deadline = lf_time_physical() + MSEC(200);
        bool released = false;
        while (lf_time_physical() < deadline) {
          if (lf_tag_compare(_fed.last_sent_NET, seen) != 0 &&
              lf_tag_compare(_fed.last_sent_NET, _fed.last_TAG) > 0) {
            released = true;
            break;
          }
        }
        printf("HOOK %d %s\\n", hook_count, released ? "released by a NET" : "timed out");
        fflush(stdout);
      }
    }
"""

PROGRAM = """\
target C { logging: log, coordination: centralized, timeout: 2 sec }
reactor A {
  input in: int
  output out: int
  timer t(0, 1 msec)
  reaction(t) {=
    // Slow start: b runs ahead to its bound and the RTI sends a its first DNET.
    if (lf_time_logical_elapsed() < MSEC(8)) lf_sleep(MSEC(5));
  =}
  reaction(in) {= =}
}
reactor B {
  input in: int
  output out: int
  timer t(0, 1 msec)
  reaction(t) {=
    // Slow throughout: b's next NET comes long after a's stale NET.
    lf_sleep(MSEC(2));
  =}
  reaction(in) {= =}
  reaction(shutdown) {=
    printf("SHUTDOWN %lld\\n", (long long)lf_time_logical_elapsed());
    fflush(stdout);
  =}
}
federated reactor {
  a = new A()
  b = new B()
  a.out -> b.in after 4999999 nsec
  b.out -> a.in after 4999999 nsec
}
"""


def _hook_runtime(lfc_dir: Path) -> None:
    """Put the two test hooks into federate.c inside lfc's core jar."""
    jars = sorted((lfc_dir / "lib").glob("core-*.jar"))
    assert len(jars) == 1, jars
    jar = jars[0]
    with zipfile.ZipFile(jar) as zf:
        src = zf.read(FEDERATE_C).decode()
    assert src.count(HOOK1_ANCHOR) == 1, "DNET handler anchor 1 not found in federate.c"
    assert src.count(HOOK_ANCHOR) == 1, "DNET handler anchor 2 not found in federate.c"
    src = src.replace(HOOK1_ANCHOR, HOOK1_ANCHOR + HOOK_BEFORE)
    src = src.replace(HOOK_ANCHOR, HOOK_ANCHOR + HOOK_AFTER)
    out = jar.with_suffix(".jar.tmp")
    with zipfile.ZipFile(jar) as zin, zipfile.ZipFile(out, "w") as zout:
        for info in zin.infolist():
            data = src.encode() if info.filename == FEDERATE_C else zin.read(info.filename)
            zout.writestr(info, data)
    out.replace(jar)


@pytest.fixture(scope="module")
def hooked_lfc(tmp_path_factory) -> Path:
    lfc = shutil.which("lfc")
    if lfc is None:
        pytest.skip("lfc not on PATH (run in the dev container)")
    if shutil.which("patch") is None:
        pytest.skip("patch not available")
    lfc_dir = Path(lfc).resolve().parent.parent
    copy = tmp_path_factory.mktemp("lfc") / "lfc"
    shutil.copytree(lfc_dir, copy, symlinks=True)
    subprocess.run([sys.executable, str(PATCH_LFC), str(copy)], check=True, timeout=300)
    _hook_runtime(copy)
    return copy / "bin" / "lfc"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_dnet_resend_cannot_overtake_main_thread(hooked_lfc, tmp_path):
    src = tmp_path / "src" / "DnetRace.lf"
    src.parent.mkdir()
    src.write_text(PROGRAM)
    build = subprocess.run([str(hooked_lfc), str(src)], capture_output=True, text=True,
                           timeout=600)
    assert build.returncode == 0, build.stdout + build.stderr
    bindir = tmp_path / "fed-gen" / "DnetRace" / "bin"
    port = _free_port()
    # Log files, not pipes: with logging: log a federate writes more than a pipe
    # buffer holds while the test waits on its sibling.
    logs = {name: tmp_path / f"{name}.log" for name in ("rti", "a", "b")}
    handles = {name: path.open("w") for name, path in logs.items()}
    # Line-buffered RTI output: a stalled RTI is killed, and its log must show
    # the NETs it recorded last.
    stdbuf = ["stdbuf", "-oL", "-eL"] if shutil.which("stdbuf") else []
    rti = subprocess.Popen([*stdbuf, str(bindir / "RTI"), "-n", "2", "-p", str(port), "-i", "dnet"],
                           stdout=handles["rti"], stderr=subprocess.STDOUT)
    feds = {name: subprocess.Popen([str(bindir / f"federate__{name}"), "-r", f"localhost:{port}",
                                    "-i", "dnet", "-f", "true"],
                                   stdout=handles[name], stderr=subprocess.STDOUT)
            for name in ("a", "b")}
    stalled = []
    try:
        for name, p in feds.items():
            try:
                p.wait(timeout=30)
            except subprocess.TimeoutExpired:
                stalled.append(name)
                p.kill()
                p.wait()
        if rti.poll() is None:
            rti.kill()
        rti.wait(timeout=30)
    finally:
        for p in (*feds.values(), rti):
            if p.poll() is None:
                p.kill()
        for h in handles.values():
            h.close()
    out = {name: logs[name].read_text(errors="replace") for name in ("a", "b")}
    rti_out = logs["rti"].read_text(errors="replace")

    def tail(text: str, n: int = 12) -> str:
        return "\n".join(text.splitlines()[-n:])

    log = "".join(f"--- federate {n} (exit {feds[n].returncode}, last lines)\n{tail(o)}\n"
                  for n, o in out.items())
    log += f"--- RTI (exit {rti.returncode}, last lines)\n{tail(rti_out)}\n"
    nets = [l for l in rti_out.splitlines() if "the Next Event Tag (NET)" in l]
    log += "--- RTI: last NETs recorded\n" + "\n".join(nets[-6:]) + "\n"
    # The hook fired: a skipped a NET and the RTI's DNET made its listener resend it.
    assert "HOOK 1" in out["a"], "the DNET resend path was not exercised\n" + log
    assert not stalled, (
        f"federation stalled: federate(s) {stalled} did not finish within 30 s after the "
        "RTI listener's resend of a skipped NET overtook the main thread's newer NET:\n"
        + log)
    assert "SHUTDOWN 2000000000" in out["b"], log
    assert feds["a"].returncode == 0, log
    assert feds["b"].returncode == 0, log
