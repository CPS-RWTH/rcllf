# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The RTI keeps waiting for a federate that connects late.

reactor-c's ``accept_socket`` (``network/impl/src/socket_common.c``) tested
``errno != EAGAIN || errno != EWOULDBLOCK || errno != EINTR``, which is always
true. The RTI's listening socket has a 10 s ``SO_RCVTIMEO``, so once no
federate connected for 10 s the ``accept`` timeout ended the loop, the RTI
logged "RTI failed to accept the federate." and exited without its
federates. The fix is carried as a local patch in
``third_party/patches/reactor-c/``. This test builds the RTI from a copy of
the pinned submodule with those patches applied, the same way
``docker/Dockerfile`` does, and checks that it is still waiting after the
accept timeout has passed.
"""

from __future__ import annotations

import os
import re
import shutil
import signal
import socket
import subprocess
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
REACTOR_C = REPO / "third_party" / "reactor-c"
APPLY = REPO / "third_party" / "patches" / "apply_reactor_c_patches.sh"
BUG_MESSAGE = "RTI failed to accept the federate."


def _accept_timeout_s() -> int:
    header = (REACTOR_C / "network/api/socket_common.h").read_text()
    return int(re.search(r"#define TCP_TIMEOUT_TIME SEC\((\d+)\)", header).group(1))


@pytest.fixture(scope="module")
def patched_rti(tmp_path_factory) -> Path:
    if not (REACTOR_C / "core/federated/RTI/CMakeLists.txt").exists():
        pytest.skip("third_party/reactor-c submodule not checked out")
    for tool in ("cmake", "cc", "patch", "stdbuf"):
        if shutil.which(tool) is None:
            pytest.skip(f"{tool} not available")
    work = tmp_path_factory.mktemp("rti")
    src = work / "reactor-c"
    shutil.copytree(REACTOR_C, src, ignore=shutil.ignore_patterns(".git"))
    subprocess.run(["sh", str(APPLY), str(src)], check=True)
    build = work / "build"
    subprocess.run(
        ["cmake", "-S", str(src / "core/federated/RTI"), "-B", str(build),
         "-DCMAKE_BUILD_TYPE=Release"],
        check=True, capture_output=True,
    )
    subprocess.run(
        ["cmake", "--build", str(build), "--parallel", "4"],
        check=True, capture_output=True,
    )
    return build / "RTI"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("", 0))
        return s.getsockname()[1]


def test_rti_waits_past_the_accept_timeout(patched_rti, tmp_path):
    log = tmp_path / "rti.log"
    with log.open("w") as out:
        rti = subprocess.Popen(
            # Unbuffered, so the log survives the crash that follows the bug.
            ["stdbuf", "-o0", "-e0", str(patched_rti), "-n", "2", "-i", "issue259", "-p", str(_free_port())],
            stdout=out, stderr=subprocess.STDOUT, start_new_session=True,
        )
    try:
        # No federate connects. Wait two seconds past one accept timeout.
        deadline = time.monotonic() + _accept_timeout_s() + 2
        while time.monotonic() < deadline and rti.poll() is None:
            time.sleep(0.2)
        text = log.read_text()
        assert BUG_MESSAGE not in text, f"RTI gave up waiting:\n{text}"
        assert rti.poll() is None, f"RTI exited with {rti.returncode}:\n{text}"
    finally:
        if rti.poll() is None:
            os.killpg(rti.pid, signal.SIGKILL)
        rti.wait(timeout=10)
