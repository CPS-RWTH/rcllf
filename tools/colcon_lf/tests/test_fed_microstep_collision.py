# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Three messages of one sender time survive a delayed federated link.

An ``after d`` connection maps the tags (t,0), (t,1) and (t,2) of one sender
time to the one tag (t+d,0). In a federate every network message is
scheduled through reactor-c's ``_lf_schedule_at_tag``
(``core/federated/federate.c``), which under the default ``defer`` policy
moved a colliding event one microstep later without checking that tag. The
second message landed on (t+d,1), the third as well, and only one of those
two reached the port. A legacy stub port carries a write
count, so a lost message is a lost count. The fix is the local patch
``third_party/patches/reactor-c/0002-*.patch``. This test applies the patches
to a copy of the lfc install, the way ``docker/Dockerfile`` does, builds a
two-federate program whose sender writes one port at three microsteps of one
time, and checks that all three values arrive.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
PATCH_LFC = REPO / "third_party" / "patches" / "patch_lfc_runtime.py"

PROGRAM = """\
target C { timeout: 10 msec, logging: warn, coordination: centralized }
reactor Src {
  output out: int
  logical action a
  logical action b
  timer t(0)
  reaction(t) -> out, a {= lf_set(out, 1); lf_schedule(a, 0); =}
  reaction(a) -> out, b {= lf_set(out, 2); lf_schedule(b, 0); =}
  reaction(b) -> out {= lf_set(out, 3); =}
}
reactor Dst {
  input in: int
  reaction(in) {=
    printf("RECV %lld %u %d\\n", (long long)lf_time_logical_elapsed(), lf_tag().microstep, in->value);
  =}
}
federated reactor {
  s = new Src()
  d = new Dst()
  s.out -> d.in after 1 msec
}
"""


@pytest.fixture(scope="module")
def patched_lfc(tmp_path_factory) -> Path:
    lfc = shutil.which("lfc")
    if lfc is None:
        pytest.skip("lfc not on PATH (run in the dev container)")
    if shutil.which("RTI") is None or shutil.which("patch") is None:
        pytest.skip("RTI or patch not available")
    lfc_dir = Path(lfc).resolve().parent.parent
    copy = tmp_path_factory.mktemp("lfc") / "lfc"
    shutil.copytree(lfc_dir, copy, symlinks=True)
    subprocess.run([sys.executable, str(PATCH_LFC), str(copy)], check=True, timeout=300)
    return copy / "bin" / "lfc"


def test_three_microsteps_reach_the_receiving_federate(patched_lfc, tmp_path):
    src = tmp_path / "src" / "Collide.lf"
    src.parent.mkdir()
    src.write_text(PROGRAM)
    build = subprocess.run([str(patched_lfc), str(src)], capture_output=True, text=True,
                           timeout=600)
    assert build.returncode == 0, build.stdout + build.stderr
    run = subprocess.run([str(tmp_path / "bin" / "Collide")], capture_output=True, text=True,
                         timeout=120)
    got = [tuple(int(x) for x in m) for m in
           re.findall(r"^RECV (\d+) (\d+) (\d+)$", run.stdout, flags=re.M)]
    values = [v for _, _, v in got]
    assert sorted(values) == [1, 2, 3], (
        f"received values {values}, expected [1, 2, 3]: a message of a colliding "
        f"microstep was lost on the delayed federated link\n{run.stdout}{run.stderr}"
    )
    # Each message got its own microstep of t+d.
    assert sorted((t, m) for t, m, _ in got) == [(1_000_000, 0), (1_000_000, 1),
                                                  (1_000_000, 2)], got
