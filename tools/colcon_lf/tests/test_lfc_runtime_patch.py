# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""lfc's bundled reactor-c gets the local accept fix too.

Federates are compiled by lfc against the runtime inside
``lib/core-<ver>.jar``, not against ``third_party/reactor-c``. That copy has
the same always-true ``accept`` condition as the RTI had, so a
federate's P2P listener stops after one 10 s accept timeout and logs
"Federate failed to accept the network abstraction.".
``third_party/patches/patch_lfc_runtime.py`` applies the local patches to the
jar. This test runs it on a copy of the installed lfc and reads the patched
``socket_common.c`` back out of the jar.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "third_party" / "patches" / "patch_lfc_runtime.py"
LFC_HOME = Path(os.environ.get("LFC_HOME", "/opt/lfc"))
MEMBER = "lib/c/reactor-c/network/impl/src/socket_common.c"
BUGGY = "errno != EAGAIN || errno != EWOULDBLOCK || errno != EINTR"
FIXED = "errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR"


def _socket_common(lfc: Path) -> str:
    (jar,) = (lfc / "lib").glob("core-*.jar")
    with zipfile.ZipFile(jar) as zf:
        return zf.read(MEMBER).decode()


@pytest.fixture
def lfc_copy(tmp_path) -> Path:
    if not list((LFC_HOME / "lib").glob("core-*.jar")):
        pytest.skip(f"no lfc install at {LFC_HOME}")
    if shutil.which("patch") is None:
        pytest.skip("patch not available")
    copy = tmp_path / "lfc"
    (copy / "lib").mkdir(parents=True)
    for jar in (LFC_HOME / "lib").glob("core-*.jar"):
        shutil.copy2(jar, copy / "lib" / jar.name)
    return copy


def _patch(lfc: Path) -> str:
    r = subprocess.run([sys.executable, str(SCRIPT), str(lfc)],
                       capture_output=True, text=True, check=True)
    return r.stdout


def test_lfc_runtime_accept_retries_on_timeout(lfc_copy):
    src = _socket_common(lfc_copy)
    if FIXED in src:
        pytest.skip(f"{LFC_HOME} already ships a fixed runtime")
    assert BUGGY in src
    _patch(lfc_copy)
    src = _socket_common(lfc_copy)
    assert BUGGY not in src and FIXED in src, (
        "lfc's bundled reactor-c still gives up on the first accept timeout"
    )


def test_lfc_runtime_patch_is_idempotent(lfc_copy):
    _patch(lfc_copy)
    jar = next((lfc_copy / "lib").glob("core-*.jar"))
    first = jar.read_bytes()
    out = _patch(lfc_copy)
    assert "already patched" in out
    assert jar.read_bytes() == first
    with zipfile.ZipFile(jar) as zf:
        assert zf.testzip() is None


def test_runtime_with_older_patches_gets_the_new_ones(tmp_path):
    # The CI image's lfc runtime carries 0001 from an older image. A new patch
    # must still apply: re-applying 0001 used to fail the whole run.
    src = REPO / "third_party" / "reactor-c"
    patches = sorted((REPO / "third_party" / "patches" / "reactor-c").glob("*.patch"))
    if not (src / "core").is_dir() or len(patches) < 2:
        pytest.skip("reactor-c submodule or a second patch missing")
    if shutil.which("patch") is None:
        pytest.skip("patch not available")
    tree = tmp_path / "reactor-c"
    shutil.copytree(src, tree, ignore=shutil.ignore_patterns(".git"))
    with patches[0].open("rb") as f:
        subprocess.run(["patch", "-p1", "--batch", "-s", "-d", str(tree)], stdin=f, check=True)
    r = subprocess.run(["sh", str(REPO / "third_party" / "patches" / "apply_reactor_c_patches.sh"),
                        str(tree)], capture_output=True, text=True)
    assert r.returncode == 0, f"patching a runtime that already has {patches[0].name} failed:\n" \
        f"{r.stdout}{r.stderr}"
    assert f"{patches[0].name} already applied" in r.stdout, r.stdout
    for p in patches:
        with p.open("rb") as f:
            check = subprocess.run(["patch", "-p1", "--forward", "--dry-run", "--batch", "-d",
                                    str(tree)], stdin=f, capture_output=True, text=True)
        assert "previously applied" in check.stdout, f"{p.name} is not in the tree: {check.stdout}"


def _check(lfc: Path, *extra: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(SCRIPT), "--check", *extra, str(lfc)],
                          capture_output=True, text=True)


def test_check_accepts_the_patched_runtime_and_changes_nothing(lfc_copy):
    """--check passes a fully patched jar and leaves it untouched."""
    _patch(lfc_copy)
    jar = next((lfc_copy / "lib").glob("core-*.jar"))
    before = jar.read_bytes()
    r = _check(lfc_copy)
    assert r.returncode == 0, r.stderr
    doc = json.loads(r.stdout)
    assert doc["problems"] == []
    assert set(doc["patches"].values()) == {"applied"}
    assert "rcllf-reactor-c-patch:0007-federate-dnet-handler-env-mutex" in doc["markers_found"]
    assert jar.read_bytes() == before


def test_check_refuses_a_runtime_without_patch_0007(lfc_copy, tmp_path):
    """An earlier image's jar lacked patch 0007. A jar patched with every
    patch except 0007/0008 must fail, naming 0007, with nothing changed."""
    jar = next((lfc_copy / "lib").glob("core-*.jar"))
    src = tmp_path / "src"
    with zipfile.ZipFile(jar) as zf:
        members = [n for n in zf.namelist() if n.startswith("lib/c/reactor-c/")]
        zf.extractall(src, members)
        entries = [(i, zf.read(i.filename)) for i in zf.infolist()]
    tree = src / "lib" / "c" / "reactor-c"
    # From either start (an unpatched jar, or the current image's fully patched
    # one): every patch but 0007/0008 applied, 0008 then 0007 reverted.
    patches = sorted((SCRIPT.parent / "reactor-c").glob("*.patch"))
    for p in patches:
        if not p.name.startswith(("0007", "0008")):
            subprocess.run(["patch", "-p1", "--forward", "--batch", "-d", str(tree)],
                           stdin=p.open("rb"), capture_output=True, check=False)
    for p in reversed(patches):
        if p.name.startswith(("0007", "0008")):
            subprocess.run(["patch", "-p1", "-R", "--force", "--batch", "-d", str(tree)],
                           stdin=p.open("rb"), capture_output=True, check=False)
    with zipfile.ZipFile(jar, "w", zipfile.ZIP_DEFLATED) as zf:
        for info, data in entries:
            f = src / info.filename
            zf.writestr(info, f.read_bytes() if f.is_file() else data)
    before = jar.read_bytes()
    r = _check(lfc_copy)
    assert r.returncode == 1
    assert "patch 0007-federate-dnet-handler-env-mutex missing" in r.stderr
    assert "marker rcllf-reactor-c-patch:0007-federate-dnet-handler-env-mutex absent" in r.stderr
    assert jar.read_bytes() == before


def test_check_rti_requires_the_network_markers(lfc_copy, tmp_path):
    """--rti: a binary without the socket_common marker is refused."""
    _patch(lfc_copy)
    marker = b"rcllf-reactor-c-patch:0001-socket_common-retry-accept-on-timeout"
    rti_marker = b"rcllf-reactor-c-patch:0009-rti-federate-upstream-failed-notice"
    good, bad = tmp_path / "RTI-good", tmp_path / "RTI-bad"
    good.write_bytes(b"\x7fELF..." + marker + b"\0" + rti_marker + b"\0")
    bad.write_bytes(b"\x7fELF..." + rti_marker + b"\0")
    ok = _check(lfc_copy, "--rti", str(good))
    assert ok.returncode == 0, ok.stderr
    assert json.loads(ok.stdout)["rti_sha256"]
    r = _check(lfc_copy, "--rti", str(bad))
    assert r.returncode == 1
    assert f"RTI marker {marker.decode()} absent" in r.stderr


def test_check_refuses_a_patch_catalog_without_0007(lfc_copy, tmp_path):
    """The expected patches and markers come from the patch
    files, so a catalog that lost 0007/0008 would expect less and pass a jar
    without them. The named REQUIRED_PATCHES list makes that a refusal."""
    _patch(lfc_copy)
    gate = tmp_path / "patches"
    (gate / "reactor-c").mkdir(parents=True)
    shutil.copy2(SCRIPT, gate / SCRIPT.name)
    for p in (SCRIPT.parent / "reactor-c").glob("*.patch"):
        if not p.name.startswith(("0007", "0008")):
            shutil.copy2(p, gate / "reactor-c" / p.name)
    r = subprocess.run([sys.executable, str(gate / SCRIPT.name), "--check", str(lfc_copy)],
                       capture_output=True, text=True)
    assert r.returncode == 1, r.stdout
    assert "patch 0007-federate-dnet-handler-env-mutex not in the patch catalog" in r.stderr
    assert "patch 0008-rcllf-marker-0007-federate not in the patch catalog" in r.stderr


def test_check_rti_requires_the_upstream_failed_notice(lfc_copy, tmp_path):
    """Patch 0009 changes the RTI (core/federated/RTI), so an RTI
    built before it, which never tells a federate that its upstream peer left
    with an error, is refused by --rti."""
    _patch(lfc_copy)
    old_rti = tmp_path / "RTI-before-0009"
    old_rti.write_bytes(
        b"\x7fELF...rcllf-reactor-c-patch:0001-socket_common-retry-accept-on-timeout\0")
    r = _check(lfc_copy, "--rti", str(old_rti))
    assert r.returncode == 1, r.stdout
    assert ("RTI marker rcllf-reactor-c-patch:0009-rti-federate-upstream-failed-notice absent"
            in r.stderr)
