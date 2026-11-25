#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Apply the local reactor-c patches to the runtime bundled inside lfc.

lfc copies the C runtime from ``lib/c/reactor-c/`` inside ``lib/core-<ver>.jar``
into every generated program, so federates never see ``third_party/reactor-c``.
This script extracts that tree, applies ``third_party/patches/reactor-c/*.patch``
with ``apply_reactor_c_patches.sh`` and writes the changed files back into the
jar. It is idempotent: a jar that is already patched is left as it is.
Used by docker/Dockerfile after the lfc download and by
tools/colcon_lf/tests/test_rti_accept_timeout.py.

``--check`` changes nothing. It exits 1 unless every patch reads
"previously applied" on the jar's runtime (a patch that is missing or whose
application failed fails) and every marker a marker patch adds is in the
runtime sources. It prints one JSON object: the jar, its sha256, the patch
states and the markers found. The Dockerfile runs it after patching, and the
benchmark campaign preparation runs it on the
container it is about to use, so a stale image is refused before launch.

The required patches are also listed by name in ``REQUIRED_PATCHES``: a patch
catalog that lost one (0007 above all, whose absence was an earlier image's defect) is a
failure, not a smaller set of expectations. Add a new patch to both.

``--rti <binary>`` with ``--check`` also requires, in the RTI binary, the
marker of every marker patch on a source the RTI compiles (network/ and, since
patch 0009, core/federated/RTI/), and adds the RTI's sha256 to the JSON.

Usage: patch_lfc_runtime.py [--check [--rti <binary>]] <lfc-install-dir>
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

PREFIX = "lib/c/reactor-c/"
HERE = Path(__file__).resolve().parent
APPLY = HERE / "apply_reactor_c_patches.sh"
MARKER_RE = re.compile(rb"rcllf-reactor-c-patch:[A-Za-z0-9_.-]+")
FOR_RE = re.compile(r"^Rcllf-Marker-For:\s*(\S+)", re.M)
# Every patch the campaign artifact must carry, independent of
# which files happen to be in reactor-c/.
REQUIRED_PATCHES = (
    "0001-socket_common-retry-accept-on-timeout",
    "0002-reactor_common-defer-colliding-event-past-queued",
    "0003-rcllf-marker-0001-socket_common",
    "0004-rcllf-marker-0002-reactor_common",
    "0005-federate-discard-rti-message-past-stop",
    "0006-rcllf-marker-0005-federate",
    "0007-federate-dnet-handler-env-mutex",
    "0008-rcllf-marker-0007-federate",
    "0009-rti-federate-upstream-failed-notice",
)


def patch_states(src: Path) -> dict[str, str]:
    """Each patch's state on the runtime tree: applied, missing or conflict."""
    out = {}
    for p in sorted((HERE / "reactor-c").glob("*.patch")):
        r = subprocess.run(
            ["patch", "-p1", "--forward", "--batch", "--dry-run", "-d", str(src)],
            stdin=p.open("rb"), capture_output=True, text=True,
        )
        if r.returncode == 0:
            out[p.stem] = "missing"
        elif "previously applied" in r.stdout:
            out[p.stem] = "applied"
        else:
            out[p.stem] = "conflict"
    return out


def expected_markers() -> list[str]:
    """The marker strings the marker patches put into the runtime."""
    return sorted(
        "rcllf-reactor-c-patch:" + m.group(1)
        for p in (HERE / "reactor-c").glob("*.patch")
        for m in [FOR_RE.search(p.read_text(errors="replace"))] if m)


def expected_rti_markers() -> list[str]:
    """Markers of the marker patches on sources the RTI compiles (network/, core/federated/RTI/)."""
    out = []
    for p in (HERE / "reactor-c").glob("*.patch"):
        text = p.read_text(errors="replace")
        m = FOR_RE.search(text)
        if m and re.search(r"^\+\+\+ b/(network|core/federated/RTI)/", text, re.M):
            out.append("rcllf-reactor-c-patch:" + m.group(1))
    return sorted(out)


def check(lfc_dir: Path, rti: Path | None = None) -> int:
    jar = _core_jar(lfc_dir)
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        with zipfile.ZipFile(jar) as zf:
            members = [n for n in zf.namelist() if n.startswith(PREFIX)]
            zf.extractall(root, members)
        src = root / PREFIX
        states = patch_states(src) if members else {}
        found = set()
        for f in src.rglob("*") if members else []:
            if f.is_file():
                found.update(m.decode() for m in MARKER_RE.findall(f.read_bytes()))
    want = expected_markers()
    catalog = {p.stem for p in (HERE / "reactor-c").glob("*.patch")}
    problems = [f"patch {n} not in the patch catalog {HERE / 'reactor-c'}"
                for n in REQUIRED_PATCHES if n not in catalog]
    problems += [f"patch {n} {s}" for n, s in states.items() if s != "applied"]
    problems += [f"marker {m} absent" for m in want if m not in found]
    if not members:
        problems.append(f"{jar.name} has no {PREFIX}")
    doc = {
        "jar": str(jar), "jar_sha256": hashlib.sha256(jar.read_bytes()).hexdigest(),
        "patches": states, "markers_expected": want, "markers_found": sorted(found)}
    if rti is not None:
        data = rti.read_bytes() if rti.is_file() else b""
        rti_found = sorted({m.decode() for m in MARKER_RE.findall(data)})
        doc.update(rti=str(rti), rti_sha256=hashlib.sha256(data).hexdigest() if data else None,
                   rti_markers_expected=expected_rti_markers(), rti_markers_found=rti_found)
        if not data:
            problems.append(f"RTI {rti} not found")
        problems += [f"RTI marker {m} absent" for m in expected_rti_markers()
                     if data and m not in rti_found]
    doc["problems"] = problems
    print(json.dumps(doc))
    if problems:
        print(f"patch_lfc_runtime --check: {jar.name} is not the patched runtime: "
              + "; ".join(problems), file=sys.stderr)
        return 1
    return 0


def _core_jar(lfc_dir: Path) -> Path:
    jars = sorted((lfc_dir / "lib").glob("core-*.jar"))
    if len(jars) != 1:
        sys.exit(f"patch_lfc_runtime: expected one lib/core-*.jar in {lfc_dir}, found {jars}")
    return jars[0]


def _already_patched(src: Path) -> bool:
    # patch --forward --dry-run exits 1 and says "Reversed (or previously
    # applied)" for a patch that is already in the tree.
    for p in sorted((HERE / "reactor-c").glob("*.patch")):
        r = subprocess.run(
            ["patch", "-p1", "--forward", "--batch", "--dry-run", "-d", str(src)],
            stdin=p.open("rb"), capture_output=True, text=True,
        )
        if r.returncode == 0:
            return False
        if "previously applied" not in r.stdout:
            sys.exit(f"patch_lfc_runtime: {p.name} does not apply:\n{r.stdout}{r.stderr}")
    return True


def main() -> None:
    args = sys.argv[1:]
    if args[:1] == ["--check"]:
        rti = None
        if args[1:2] == ["--rti"] and len(args) == 4:
            rti, args = Path(args[2]), args[:1] + args[3:]
        if len(args) == 2:
            sys.exit(check(Path(args[1]), rti))
    if len(sys.argv) != 2 or sys.argv[1].startswith("--"):
        sys.exit("usage: patch_lfc_runtime.py [--check [--rti <binary>]] <lfc-install-dir>")
    jar = _core_jar(Path(sys.argv[1]))
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        with zipfile.ZipFile(jar) as zf:
            members = [n for n in zf.namelist() if n.startswith(PREFIX)]
            if not members:
                sys.exit(f"patch_lfc_runtime: {jar} has no {PREFIX}")
            zf.extractall(root, members)
        src = root / PREFIX
        before = {n: (root / n).read_bytes() for n in members if not n.endswith("/")}
        if _already_patched(src):
            print(f"lfc runtime: {jar.name} already patched")
            return
        subprocess.run(["sh", str(APPLY), str(src)], check=True)
        changed = {n: (root / n).read_bytes() for n in before if (root / n).read_bytes() != before[n]}
        out = jar.with_suffix(".jar.tmp")
        with zipfile.ZipFile(jar) as zin, zipfile.ZipFile(out, "w") as zout:
            for info in zin.infolist():
                data = changed.get(info.filename)
                zout.writestr(info, data if data is not None else zin.read(info.filename))
        os.chmod(out, jar.stat().st_mode)
        os.replace(out, jar)
        for n in sorted(changed):
            print(f"lfc runtime: patched {n} in {jar.name}")


if __name__ == "__main__":
    main()
