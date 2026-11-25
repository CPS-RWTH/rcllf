# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""``colcon lf-generate`` refuses an lfc runtime without the reactor-c patches.

Every live ``ros2 lf run`` on the local ``:dev`` image of 2026-10-12 ended
``read-back invalid: reactor_c_patches: runtime built without the configured
patches (missing rcllf-reactor-c-patch:0002-...)`` although the outputs were
right: the image predated the marker patch, so lfc's jar lacked the marker and
every program generated from it carried an empty marker table. The gate here
catches that runtime before a program is generated from it.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import textwrap
import zipfile
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[3]
LFC_HOME = Path(os.environ.get("LFC_HOME", "/opt/lfc"))
MARKER_0002 = "rcllf-reactor-c-patch:0002-reactor_common-defer-colliding-event-past-queued"
# What the bug produced: the run's error on the stale image.
READBACK_ERROR = ("read-back invalid: reactor_c_patches: runtime built without the "
                  f"configured patches (missing {MARKER_0002})")


def _gate(workspace, env):
    """The gate's verdict, or None where the gate does not exist (the bug)."""
    try:
        from colcon_lf.generation.lfc_runtime import check_lfc_runtime
    except ImportError:
        return None
    return check_lfc_runtime(workspace, env)


def _abs_path(*first: Path) -> str:
    """A PATH of *first* plus the environment's absolute entries (a relative
    entry is refused for a bare $LFC, see round 6)."""
    return ":".join([str(p) for p in first]
                    + [e for e in os.environ.get("PATH", "").split(":") if os.path.isabs(e)])


def _unpatched_lfc(root: Path) -> Path:
    """An lfc install whose runtime jar carries the stock reactor_common.c: no marker."""
    (root / "bin").mkdir(parents=True)
    (root / "lib").mkdir()
    lfc = root / "bin" / "lfc"
    lfc.write_text("#!/bin/sh\nexit 0\n")
    lfc.chmod(0o755)
    with zipfile.ZipFile(root / "lib" / "core-0.12.1.jar", "w") as zf:
        zf.writestr("lib/c/reactor-c/core/reactor_common.c",
                    "/* stock runtime: no rcllf patch, no marker */\n"
                    "trigger_handle_t _lf_schedule(void) { return 0; }\n")
    return lfc


def test_falsification_unpatched_runtime_is_refused(tmp_path):
    """Fails on the old code with the message the stale image produced."""
    lfc = _unpatched_lfc(tmp_path / "lfc")
    doc = _gate(REPO / "packages", {"LFC": str(lfc)})
    assert doc is not None and doc["status"] == "failed", READBACK_ERROR
    assert f"marker {MARKER_0002} absent" in doc["problems"]
    assert doc["jar"].endswith("core-0.12.1.jar")
    assert MARKER_0002 in doc["markers_expected"]
    assert MARKER_0002 not in doc["markers_found"]


def test_verb_refuses_before_generating(tmp_path, monkeypatch, capsys):
    import colcon_lf.verb.generate as gen

    lfc = _unpatched_lfc(tmp_path / "lfc")
    monkeypatch.setenv("LFC", str(lfc))
    sj = tmp_path / "system.json"
    sj.write_text("{}")
    monkeypatch.setattr(gen, "_find_system_jsons", lambda *a: [sj])
    monkeypatch.setattr(gen, "_resolve_workspace", lambda ws: REPO / "packages")
    generated = []
    monkeypatch.setattr(gen, "_generate_one", lambda **kw: generated.append(kw) or 0)
    args = SimpleNamespace(
        target_lfc=False, federate=None, system_json=str(sj), package=None,
        output_dir=None, workspace=str(REPO / "packages"), after_delay=None, launch=True,
        emit_federation_template=False, force=False, exec_config=None, params_file=None,
    )
    assert gen.GenerateVerb().main(context=SimpleNamespace(args=args)) == 1
    err = capsys.readouterr().err
    assert "Error: lfc runtime" in err and f"marker {MARKER_0002} absent" in err
    assert "docker compose build dev" in err and "#769" not in err
    assert generated == []


def test_no_lfc_install_is_reported_not_refused(tmp_path):
    from colcon_lf.generation.lfc_runtime import check_lfc_runtime

    doc = check_lfc_runtime(REPO / "packages", {"LFC": str(tmp_path / "none" / "bin" / "lfc")})
    assert doc["status"] == "skipped" and doc["problems"] == []
    assert "no lfc install" in doc["reason"]


def test_no_patch_catalog_is_reported_not_refused(tmp_path, monkeypatch):
    from colcon_lf.generation import lfc_runtime

    lfc = _unpatched_lfc(tmp_path / "lfc")
    monkeypatch.setattr(lfc_runtime, "patch_checker", lambda ws=None: None)
    doc = lfc_runtime.check_lfc_runtime(tmp_path, {"LFC": str(lfc)})
    assert doc["status"] == "skipped" and "patch_lfc_runtime.py" in doc["reason"]


def test_lfc_resolution_matches_the_emitted_cmake():
    """The gate checks the lfc the generated CMakeLists will run."""
    import inspect

    import colcon_lf.verb.generate as gen
    from colcon_lf.generation.lfc_runtime import DEFAULT_LFC, lfc_binary

    src = inspect.getsource(gen)
    assert 'set(LFC_BIN "$ENV{{LFC}}")' in src and f'set(LFC_BIN "{DEFAULT_LFC}")' in src
    # The three rules of both checks: absolute as given, bare through PATH,
    # relative with a directory part never (refused, see below).
    assert lfc_binary({"LFC": sys.executable}) == Path(sys.executable)
    exe = Path(sys.executable)
    assert lfc_binary({"LFC": exe.name, "PATH": str(exe.parent)}) == exe
    assert lfc_binary({"LFC": f"./{exe.name}", "PATH": str(exe.parent)}) is None
    assert lfc_binary({"LFC": exe.name, "PATH": f"bin:{exe.parent}"}) is None
    assert lfc_binary({"LFC": exe.name, "PATH": ""}) is None
    assert lfc_binary({"LFC": exe.name}) is None


def test_patched_install_passes():
    """The image's lfc (Dockerfile ran patch_lfc_runtime --check on it) passes the gate."""
    from colcon_lf.generation.lfc_runtime import check_lfc_runtime

    lfc = LFC_HOME / "bin" / "lfc"
    if not lfc.is_file():
        pytest.skip(f"no lfc install at {LFC_HOME}")
    doc = check_lfc_runtime(REPO / "packages", {"LFC": str(lfc)})
    assert doc["status"] == "ok", doc["problems"]
    assert MARKER_0002 in doc["markers_found"]


# --------------------------------------------------------------------------
# Configure-time check in the emitted CMake: a
# generated package persists and may be rebuilt in a stale image without
# lf-generate running again, so the CMake re-runs the check before lfc.

def _cmake_script(tmp_path, lfc_bin, checker):
    from colcon_lf.generation.lfc_runtime import cmake_runtime_check
    script = tmp_path / "check.cmake"
    script.write_text(f'set(LFC_BIN "{lfc_bin.as_posix()}")\n' + cmake_runtime_check(checker))
    return script


def _run_cmake(script, cwd=None, env=None):
    """``cmake -P``: in script mode CMAKE_CURRENT_SOURCE_DIR is the working directory.
    *env* replaces the environment cmake sees (its own path is resolved here)."""
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("cmake not installed")
    return subprocess.run([cmake, "-P", str(script)], capture_output=True, text=True,
                          cwd=cwd, env=env)


def _flat(res):
    """CMake word-wraps FATAL_ERROR text at the terminal width (the wrap point moves
    with the temp-dir length; CI shard 2 failed on it): compare on one line."""
    return " ".join((res.stdout + res.stderr).split())


def test_emitted_cmake_refuses_an_unpatched_runtime_at_configure_time(tmp_path):
    from colcon_lf.generation import lfc_runtime as rt
    lfc = _unpatched_lfc(tmp_path / "lfc")
    checker = rt.patch_checker(REPO)
    assert checker is not None
    res = _run_cmake(_cmake_script(tmp_path, lfc, checker))
    out = _flat(res)
    assert res.returncode != 0, out
    assert "is not the patched runtime: " in out and "docker compose build dev" in out, out
    assert "docker compose build dev" in out, out
    assert MARKER_0002 in out, out


def test_emitted_cmake_finds_the_checker_above_a_moved_package(tmp_path):
    """The path baked at generate time is a fallback: a
    package configured in another checkout meets that checkout's catalog."""
    lfc = _unpatched_lfc(tmp_path / "lfc")
    checkout = tmp_path / "checkout"
    (checkout / "third_party").mkdir(parents=True)
    (checkout / "third_party" / "patches").symlink_to(REPO / "third_party" / "patches")
    pkg = checkout / "packages" / "lf-generated" / "lf_generated_moved"
    pkg.mkdir(parents=True)
    script = _cmake_script(pkg, lfc, tmp_path / "gone" / "patch_lfc_runtime.py")
    res = _run_cmake(script, cwd=pkg)
    out = _flat(res)
    assert res.returncode != 0 and "is not the patched runtime" in out, out
    # The new checkout's catalog was used, not the (gone) baked path.
    assert "checkout/third_party/patches/patch_lfc_runtime.py" in out, out
    assert "gone/patch_lfc_runtime.py" not in out


def test_emitted_cmake_warns_without_any_patch_catalog(tmp_path):
    """A package outside any checkout (no catalog above it): a WARNING, not a refusal."""
    lfc = _unpatched_lfc(tmp_path / "lfc")
    res = _run_cmake(_cmake_script(tmp_path, lfc, None), cwd=tmp_path)
    assert res.returncode == 0, _flat(res)
    assert "lfc runtime NOT checked" in _flat(res)


def test_emitted_cmake_resolves_a_bare_lfc_through_path(tmp_path, monkeypatch):
    from colcon_lf.generation import lfc_runtime as rt
    lfc = _unpatched_lfc(tmp_path / "lfc")
    monkeypatch.setenv("PATH", _abs_path(lfc.parent))
    res = _run_cmake(_cmake_script(tmp_path, Path("lfc"), rt.patch_checker(REPO)))
    out = _flat(res)
    assert res.returncode != 0 and "is not the patched runtime" in out, out


def test_gate_resolves_a_bare_lfc_through_path(tmp_path):
    lfc = _unpatched_lfc(tmp_path / "lfc")
    env = {"LFC": "lfc", "PATH": _abs_path(lfc.parent)}
    doc = _gate(REPO / "packages", env)
    assert doc is not None and doc["status"] == "failed", doc
    assert f"marker {MARKER_0002} absent" in doc["problems"]


def test_lf_build_refuses_before_building(tmp_path, monkeypatch, capsys):
    """Packages generated before this change carry no CMake check; lf-build gates them."""
    import colcon_lf.verb.build as build

    lfc = _unpatched_lfc(tmp_path / "lfc")
    monkeypatch.setenv("LFC", str(lfc))
    monkeypatch.setattr(build, "_resolve_workspace", lambda ws: REPO / "packages")
    built = []
    monkeypatch.setattr(build, "_colcon_build", lambda *a: built.append(a) or 0)
    monkeypatch.setattr(build, "find_generated_packages", lambda *a: ["lf_generated_x"])
    rc = build.BuildVerb().main(context=SimpleNamespace(args=SimpleNamespace(workspace=None, package=None)))
    err = capsys.readouterr().err
    assert rc == 1 and "Error: lfc runtime" in err and "docker compose build dev" in err, err
    assert built == []


def test_emitted_cmake_check_passes_on_a_patched_install(tmp_path):
    from colcon_lf.generation import lfc_runtime as rt
    lfc = LFC_HOME / "bin" / "lfc"
    if not lfc.is_file():
        pytest.skip(f"no lfc install at {LFC_HOME} (run in the dev container)")
    res = _run_cmake(_cmake_script(tmp_path, lfc, rt.patch_checker(REPO)))
    assert res.returncode == 0, _flat(res)


def test_both_cmake_templates_run_the_check_before_lfc():
    """Both emitters (coordinator and per-program package) carry the block
    right after LFC_BIN is resolved and before lfc is invoked."""
    import colcon_lf.verb.generate as generate_mod
    src = Path(generate_mod.__file__).read_text()
    assert src.count("{lfc_runtime_check}") == 2, "both CMake templates re-check the runtime"
    for block in src.split('cmake_text = textwrap.dedent(f"""')[1:]:
        if "{lfc_runtime_check}" not in block:
            continue
        resolve = block.index('set(LFC_BIN "/opt/lfc/bin/lfc")')
        check = block.index("{lfc_runtime_check}")
        run = block.index("${{LFC_BIN}}", check)
        assert resolve < check < run


# --------------------------------------------------------------------------
# The two resolutions must not diverge, a
# missing lfc is lfc's own error, and the baked checker path is a CMake string.

RELATIVE_REFUSAL = "Set LFC to an absolute path, or to a bare program name resolved through PATH"


def test_gate_refuses_a_relative_lfc_with_a_directory_part(tmp_path, monkeypatch):
    """Falsification: on fe287786 the gate resolved ``LFC=./lfc/bin/lfc`` against its
    own working directory (checked it from one cwd, skipped it from another) while
    the emitted CMake resolved it against the source dir and lfc ran it from the
    staging dir: two different executables could be checked and used."""
    from colcon_lf.generation.lfc_runtime import lfc_runtime_error

    lfc = _unpatched_lfc(tmp_path / "lfc")
    assert lfc.is_file()
    verdicts = {}
    for cwd in (tmp_path, tmp_path / "elsewhere"):
        cwd.mkdir(exist_ok=True)
        monkeypatch.chdir(cwd)
        doc = _gate(REPO / "packages", {"LFC": "./lfc/bin/lfc"})
        verdicts[cwd.name] = (doc["status"], doc.get("reason") or " ".join(doc["problems"]))
        assert doc["status"] == "failed" and RELATIVE_REFUSAL in " ".join(doc["problems"]), (
            f"the gate's verdict from {cwd} was {verdicts[cwd.name]}: it resolved the "
            "relative $LFC against its own working directory")
        assert lfc_runtime_error(doc).startswith("Error: LFC=./lfc/bin/lfc is a relative path")
        assert "Set LFC to an absolute path" in lfc_runtime_error(doc)
    # The verdict does not depend on where the gate runs.
    assert len(set(verdicts.values())) == 1, verdicts


def test_emitted_cmake_refuses_a_relative_lfc_with_a_directory_part(tmp_path):
    """Falsification: on fe287786 configure resolved ``./lfc/bin/lfc`` against the
    source dir and reported that lfc's runtime verdict ("is not the patched
    runtime"), although lfc itself runs from the staging dir."""
    from colcon_lf.generation import lfc_runtime as rt
    _unpatched_lfc(tmp_path / "lfc")
    res = _run_cmake(_cmake_script(tmp_path, Path("./lfc/bin/lfc"), rt.patch_checker(REPO)),
                     cwd=tmp_path)
    out = _flat(res)
    assert res.returncode != 0, out
    assert "is a relative path with a directory part" in out and "Set LFC to an absolute path" in out, out
    assert RELATIVE_REFUSAL in out, out
    assert "is not the patched runtime" not in out, out


@pytest.mark.parametrize("lfc_bin", ["{tmp}/none/bin/lfc", "lfc-missing-769"],
                         ids=["absolute-missing", "bare-missing"])
def test_emitted_cmake_skips_a_missing_lfc_with_a_status_note(tmp_path, lfc_bin):
    """Falsification: on fe287786 a missing lfc made configure fail with "is not the
    patched runtime ... rebuild the image", while lf-generate only notes a skipped
    check. Now configure notes the skip and lfc's own invocation reports it."""
    from colcon_lf.generation import lfc_runtime as rt
    lfc_bin = Path(lfc_bin.format(tmp=tmp_path.as_posix()))
    assert not lfc_bin.exists() and shutil.which(str(lfc_bin)) is None
    res = _run_cmake(_cmake_script(tmp_path, lfc_bin, rt.patch_checker(REPO)), cwd=tmp_path)
    out = _flat(res)
    assert res.returncode == 0, out
    assert "lfc runtime NOT checked: no lfc executable at" in out, out
    assert "is not the patched runtime" not in out and "rebuild the image" not in out, out


def test_emitted_cmake_quotes_the_checker_hint_path(tmp_path):
    """Falsification: on fe287786 the baked checker path went into the CMake text
    unescaped; a checkout path with a quote broke the parse and one with ``$``
    was expanded as a variable reference (the hint silently went unused)."""
    _unpatched_lfc(tmp_path / "lfc")
    checkout = tmp_path / 'q"uote $var back\\slash'
    (checkout / "third_party").mkdir(parents=True)
    (checkout / "third_party" / "patches").symlink_to(REPO / "third_party" / "patches")
    hint = checkout / "third_party" / "patches" / "patch_lfc_runtime.py"
    assert hint.is_file()
    # The package lives outside any checkout, so the baked hint is the only catalog.
    res = _run_cmake(_cmake_script(tmp_path, tmp_path / "lfc" / "bin" / "lfc", hint), cwd=tmp_path)
    out = _flat(res)
    assert "Parse error" not in out and "Syntax error" not in out, out
    assert res.returncode != 0 and "is not the patched runtime" in out, out
    assert "lfc runtime NOT checked" not in out, out
    assert f"with {hint.as_posix()}" in out, out


# --------------------------------------------------------------------------
# A bare name through a PATH with a relative
# entry diverges too, the invocation must run the checked file, and a
# directory named as $LFC is absent, not a runtime to judge.

def test_gate_refuses_a_bare_lfc_with_a_relative_path_entry(tmp_path, monkeypatch):
    """Falsification: on 8309fab4 the gate found ``lfc`` through the relative PATH
    entry ``lfc/bin`` against its own working directory and judged that file."""
    lfc = _unpatched_lfc(tmp_path / "lfc")
    monkeypatch.chdir(tmp_path)
    for path in (_abs_path(Path("lfc/bin")), f":{_abs_path()}", f"{_abs_path()}:"):
        doc = _gate(REPO / "packages", {"LFC": "lfc", "PATH": path})
        assert doc["status"] == "failed" and "PATH entry" in " ".join(doc["problems"]), (
            f"PATH={path}: the gate judged {doc.get('lfc') or doc.get('jar')}: {doc}")
        assert RELATIVE_REFUSAL in " ".join(doc["problems"])
    assert lfc.is_file()


def test_emitted_cmake_refuses_a_bare_lfc_with_a_relative_path_entry(tmp_path, monkeypatch):
    """Falsification: on 8309fab4 configure found ``lfc`` through the relative PATH
    entry against its working directory and judged it ("is not the patched runtime"),
    while the lfc invocation runs from the staging dir."""
    from colcon_lf.generation import lfc_runtime as rt
    _unpatched_lfc(tmp_path / "lfc")
    for path in (_abs_path(Path("lfc/bin")), f"{_abs_path()}:"):
        monkeypatch.setenv("PATH", path)
        res = _run_cmake(_cmake_script(tmp_path, Path("lfc"), rt.patch_checker(REPO)), cwd=tmp_path)
        out = _flat(res)
        assert res.returncode != 0, out
        assert "is a bare program name and PATH entry" in out and RELATIVE_REFUSAL in out, out
        assert "is not the patched runtime" not in out, out


def test_emitted_cmake_invokes_the_executable_it_checked(tmp_path, monkeypatch):
    """Falsification: on 8309fab4 the check resolved a bare ``lfc`` through PATH but
    the invocation still ran the bare name from the staging directory; now
    ``LFC_BIN`` is the resolved file. Shown on the no-catalog branch, where no
    verdict stops the script."""
    from colcon_lf.generation.lfc_runtime import cmake_runtime_check
    lfc = _unpatched_lfc(tmp_path / "lfc")
    monkeypatch.setenv("PATH", _abs_path(lfc.parent))
    script = tmp_path / "check.cmake"
    script.write_text('set(LFC_BIN "lfc")\n' + cmake_runtime_check(None)
                      + '\nmessage(STATUS "RESOLVED_LFC_BIN=${LFC_BIN}")\n')
    res = _run_cmake(script, cwd=tmp_path)
    out = _flat(res)
    assert res.returncode == 0, out
    assert f"RESOLVED_LFC_BIN={lfc.as_posix()}" in out, out
    # A missing bare name stays as given, so lfc's own invocation reports it.
    script.write_text('set(LFC_BIN "lfc-missing-769")\n' + cmake_runtime_check(None)
                      + '\nmessage(STATUS "RESOLVED_LFC_BIN=${LFC_BIN}")\n')
    out = _flat(_run_cmake(script, cwd=tmp_path))
    assert "RESOLVED_LFC_BIN=lfc-missing-769" in out, out


def test_emitted_cmake_ignores_a_find_root_sysroot(tmp_path, monkeypatch):
    """Falsification: on 18fbb224 the emitted ``find_program`` honoured
    ``CMAKE_FIND_ROOT_PATH`` (a toolchain's sysroot), so with
    ``CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY`` it resolved a bare ``lfc`` to
    ``<sysroot>/<PATH entry>/lfc`` while the Python gate had checked
    ``<PATH entry>/lfc``. The check names the install it judges, so the wrong
    sysroot path showed in its message. Now the lookup walks PATH itself
    (round 8c: first with ``NO_CMAKE_FIND_ROOT_PATH``, which left
    ``CMAKE_IGNORE_PATH`` and the executable bit open) and both resolve the
    host file."""
    from colcon_lf.generation import lfc_runtime as rt
    host = _unpatched_lfc(tmp_path / "host")
    sysroot = tmp_path / "sysroot"
    rerooted = _unpatched_lfc(sysroot / host.parent.parent.relative_to("/"))
    assert rerooted.as_posix() == sysroot.as_posix() + host.as_posix()
    monkeypatch.setenv("PATH", _abs_path(host.parent))
    script = tmp_path / "check.cmake"
    script.write_text(f'set(CMAKE_FIND_ROOT_PATH "{sysroot.as_posix()}")\n'
                      'set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)\n'
                      'set(LFC_BIN "lfc")\n' + rt.cmake_runtime_check(rt.patch_checker(REPO)))
    out = _flat(_run_cmake(script, cwd=tmp_path))
    assert f"checking lfc runtime {host.parent.parent.as_posix()} " in out, out
    assert sysroot.as_posix() not in out, out
    # Same lookup, no catalog: the invocation gets the host file, not the sysroot's.
    script.write_text(f'set(CMAKE_FIND_ROOT_PATH "{sysroot.as_posix()}")\n'
                      'set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)\n'
                      'set(LFC_BIN "lfc")\n' + rt.cmake_runtime_check(None)
                      + '\nmessage(STATUS "RESOLVED_LFC_BIN=${LFC_BIN}")\n')
    out = _flat(_run_cmake(script, cwd=tmp_path))
    assert f"RESOLVED_LFC_BIN={host.as_posix()}" in out, out


def test_emitted_cmake_ignores_cmake_ignore_path(tmp_path, monkeypatch):
    """Falsification: on 4fcb7666 the lookup was a
    ``find_program``, which drops ``CMAKE_IGNORE_PATH`` entries whatever flags
    it carries, so with the PATH entry ignored the check reported "no lfc
    executable at lfc" and the invocation ran the bare name through PATH
    unchecked; ``shutil.which`` had found it. Now the lookup walks PATH."""
    from colcon_lf.generation import lfc_runtime as rt
    lfc = _unpatched_lfc(tmp_path / "host")
    monkeypatch.setenv("PATH", _abs_path(lfc.parent))
    assert rt.lfc_binary({"LFC": "lfc", "PATH": os.environ["PATH"]}) == lfc
    script = tmp_path / "check.cmake"
    script.write_text(f'set(CMAKE_IGNORE_PATH "{lfc.parent.as_posix()}")\n'
                      'set(LFC_BIN "lfc")\n' + rt.cmake_runtime_check(rt.patch_checker(REPO)))
    out = _flat(_run_cmake(script, cwd=tmp_path))
    assert f"checking lfc runtime {lfc.parent.parent.as_posix()} " in out, out
    assert "is not the patched runtime" in out, out


def test_emitted_cmake_skips_a_non_executable_candidate_as_which_does(tmp_path, monkeypatch):
    """Falsification: on 4fcb7666 ``find_program`` took a
    readable but non-executable ``lfc`` in the first PATH entry, so the check
    judged ``<blocked>/bin/lfc`` while ``shutil.which`` had skipped it for
    ``<good>/bin/lfc``. Shown here in script mode (policy CMP0109 unset); the
    generated package's ``cmake_minimum_required(VERSION 3.19)`` sets it NEW,
    where find_program already skipped the file. Now the walk asks ``test -x``
    as which does, whatever the policy."""
    from colcon_lf.generation import lfc_runtime as rt
    blocked = _unpatched_lfc(tmp_path / "blocked")
    blocked.chmod(0o644)
    good = _unpatched_lfc(tmp_path / "good")
    monkeypatch.setenv("PATH", _abs_path(blocked.parent, good.parent))
    assert rt.lfc_binary({"LFC": "lfc", "PATH": os.environ["PATH"]}) == good
    script = tmp_path / "check.cmake"
    script.write_text('set(LFC_BIN "lfc")\n' + rt.cmake_runtime_check(None)
                      + '\nmessage(STATUS "RESOLVED_LFC_BIN=${LFC_BIN}")\n')
    out = _flat(_run_cmake(script, cwd=tmp_path))
    assert f"RESOLVED_LFC_BIN={good.as_posix()}" in out, out


def test_emitted_cmake_permission_test_is_not_taken_from_path(tmp_path, monkeypatch):
    """Falsification: on 45b00eb4 the walk ran ``test -x``
    through the PATH it was searching, so a ``test`` in the first entry that
    always exits 1 made every candidate look non-executable: the check
    reported "no lfc executable at lfc" and the invocation ran the bare name
    through PATH unchecked, while ``shutil.which`` (a syscall) had found it.
    Now the permission test is /usr/bin/test or /bin/test."""
    from colcon_lf.generation import lfc_runtime as rt
    lfc = _unpatched_lfc(tmp_path / "host")
    shadow = lfc.parent / "test"
    shadow.write_text("#!/bin/sh\nexit 1\n")
    shadow.chmod(0o755)
    monkeypatch.setenv("PATH", _abs_path(lfc.parent))
    assert rt.lfc_binary({"LFC": "lfc", "PATH": os.environ["PATH"]}) == lfc
    script = tmp_path / "check.cmake"
    script.write_text('set(LFC_BIN "lfc")\n' + rt.cmake_runtime_check(None)
                      + '\nmessage(STATUS "RESOLVED_LFC_BIN=${LFC_BIN}")\n')
    out = _flat(_run_cmake(script, cwd=tmp_path))
    assert f"RESOLVED_LFC_BIN={lfc.as_posix()}" in out, out


def test_emitted_cmake_takes_an_execute_only_lfc_as_which_does(tmp_path, monkeypatch):
    """Falsification: on 15286122 the walk and the presence
    test used CMake's ``EXISTS``, which asks for read permission (access R_OK),
    while ``shutil.which`` asks X_OK and ``Path.is_file`` only stats: an lfc with
    mode 0111 passed the Python gate, bare or absolute, and was absent for the
    check. Now every file test goes through the pinned ``test`` binary. (Root
    reads anything, so under root the old code passed this test as well.)"""
    from colcon_lf.generation import lfc_runtime as rt
    lfc = _unpatched_lfc(tmp_path / "host")
    lfc.chmod(0o111)
    monkeypatch.setenv("PATH", _abs_path(lfc.parent))
    assert rt.lfc_binary({"LFC": "lfc", "PATH": os.environ["PATH"]}) == lfc
    assert rt.lfc_binary({"LFC": lfc.as_posix()}) == lfc
    script = tmp_path / "check.cmake"
    for lfc_bin in ("lfc", lfc.as_posix()):
        script.write_text(f'set(LFC_BIN "{lfc_bin}")\n' + rt.cmake_runtime_check(None)
                          + '\nmessage(STATUS "RESOLVED_LFC_BIN=${LFC_BIN}")\n')
        out = _flat(_run_cmake(script, cwd=tmp_path))
        assert f"RESOLVED_LFC_BIN={lfc.as_posix()}" in out, out
        assert "NOT checked: no lfc executable" not in out, out


def test_emitted_cmake_treats_a_fifo_named_lfc_as_absent(tmp_path, monkeypatch):
    """Falsification: on 15286122 an executable FIFO named
    lfc passed ``EXISTS`` and ``NOT IS_DIRECTORY``, so the check judged its home
    ("checking lfc runtime <home>") and failed on it, while the Python gate's
    ``Path.is_file`` had reported no lfc. Now presence is ``test -f``, bare or
    absolute."""
    from colcon_lf.generation import lfc_runtime as rt
    fifo = tmp_path / "host" / "bin" / "lfc"
    fifo.parent.mkdir(parents=True)
    os.mkfifo(fifo)
    fifo.chmod(0o755)
    monkeypatch.setenv("PATH", _abs_path(fifo.parent))
    assert rt.lfc_binary({"LFC": "lfc", "PATH": os.environ["PATH"]}) is None
    assert rt.lfc_binary({"LFC": fifo.as_posix()}) is None
    script = tmp_path / "check.cmake"
    for lfc_bin in ("lfc", fifo.as_posix()):
        script.write_text(f'set(LFC_BIN "{lfc_bin}")\n'
                          + rt.cmake_runtime_check(rt.patch_checker(REPO)))
        res = _run_cmake(script, cwd=tmp_path)
        out = _flat(res)
        assert res.returncode == 0 and "lfc runtime NOT checked: no lfc executable" in out, out


def _invocation_blocks():
    """The ``execute_process`` that runs lfc, from each CMake template in
    generate.py, as runnable CMake (f-string braces unescaped)."""
    src = (REPO / "tools" / "colcon_lf" / "colcon_lf" / "verb" / "generate.py").read_text()
    blocks = []
    for block in src.split('cmake_text = textwrap.dedent(f"""')[1:]:
        if "{lfc_runtime_check}" not in block:
            continue
        run = block.index("${{LFC_BIN}}", block.index("{lfc_runtime_check}"))
        start = block.rfind("execute_process(", 0, run)
        end = block.index("\n        )\n", run) + len("\n        )\n")
        blocks.append(textwrap.dedent(block[start:end]).replace("{{", "{").replace("}}", "}"))
    assert len(blocks) == 2, "both CMake templates invoke lfc"
    return blocks


@pytest.mark.parametrize("which", [0, 1], ids=["federated", "per-program"])
def test_emitted_invocation_runs_the_checked_file_with_a_semicolon_in_its_path(tmp_path, which):
    """Falsification: on 6eb1f424 both templates expanded
    ``${LFC_BIN}`` unquoted in ``COMMAND``, so an absolute ``LFC=/x;y/bin/lfc``
    was checked as one file but invoked as ``/x`` with ``y/bin/lfc`` as its
    first argument: ``LFC_RESULT=No such file or directory`` and the checked
    lfc never ran. Now the expansion is quoted."""
    from colcon_lf.generation.lfc_runtime import cmake_runtime_check
    lfc = tmp_path / "te;st" / "bin" / "lfc"
    lfc.parent.mkdir(parents=True)
    ran = tmp_path / "ran.txt"
    lfc.write_text(f'#!/bin/sh\necho "$0 $@" > "{ran}"\nexit 0\n')
    lfc.chmod(0o755)
    gen_dir = tmp_path / "lf-src"
    gen_dir.mkdir()
    (gen_dir / "Sys.lf").write_text("// stub\n")
    script = tmp_path / "invoke.cmake"
    script.write_text(f'set(LFC_BIN "{lfc.as_posix()}")\n'
                      f'set(LF_GEN_DIR "{gen_dir.as_posix()}")\n'
                      f'set(LF_SRC_STAGED "{gen_dir.as_posix()}/Sys.lf")\n'
                      'set(_LF_FULL_PREFIX "")\n'
                      + cmake_runtime_check(None) + "\n"
                      + _invocation_blocks()[which]
                      + '\nmessage(STATUS "LFC_RESULT=${LFC_RESULT}")\n')
    out = _flat(_run_cmake(script, cwd=tmp_path))
    assert "LFC_RESULT=0" in out, out
    assert ran.is_file() and ran.read_text().startswith(lfc.as_posix() + " "), out


def test_emitted_cmake_skips_a_directory_named_as_lfc(tmp_path):
    """Falsification: on 8309fab4 an absolute $LFC naming a directory passed the
    EXISTS test and configure judged that directory's install home ("is not the
    patched runtime"); the Python gate had skipped it (``is_file``)."""
    from colcon_lf.generation import lfc_runtime as rt
    lfc_dir = _unpatched_lfc(tmp_path / "lfc").parent  # .../lfc/bin, a directory
    assert lfc_dir.is_dir()
    res = _run_cmake(_cmake_script(tmp_path, lfc_dir, rt.patch_checker(REPO)), cwd=tmp_path)
    out = _flat(res)
    assert res.returncode == 0, out
    assert "lfc runtime NOT checked: no lfc executable at" in out, out
    assert "is not the patched runtime" not in out, out
    doc = _gate(REPO / "packages", {"LFC": str(lfc_dir)})
    assert doc["status"] == "skipped" and "no lfc install" in doc["reason"], doc


# --------------------------------------------------------------------------
# An empty PATH, a $LFC that CMake reads as a
# false constant, a semicolon in a PATH entry, and the missing-lfc note
# without a catalog.

def _template_lfc_defaults():
    """The LFC_BIN default lines of both CMake templates in generate.py, as the
    CMake text they emit (f-string braces unescaped)."""
    import colcon_lf.verb.generate as generate_mod
    src = Path(generate_mod.__file__).read_text()
    blocks = []
    for block in src.split('cmake_text = textwrap.dedent(f"""')[1:]:
        if "{lfc_runtime_check}" not in block:
            continue
        start = block.index('set(LFC_BIN "$ENV{{LFC}}")')
        end = block.index("{lfc_runtime_check}")
        blocks.append(textwrap.dedent(block[start:end]).replace("{{", "{").replace("}}", "}"))
    assert len(blocks) == 2, "both CMake templates resolve LFC_BIN"
    return blocks


def test_gate_refuses_a_bare_lfc_without_a_path(tmp_path):
    """Falsification: on af855e0b the gate's PATH rule skipped an empty or unset PATH,
    so the bare name resolved through this process's own default search path (the
    image's /opt/lfc/bin was judged, or nothing was found and the check skipped),
    not through the PATH the emitted CMake and the lfc invocation see."""
    _unpatched_lfc(tmp_path / "lfc")
    for env in ({"LFC": "lfc", "PATH": ""}, {"LFC": "lfc"}):
        doc = _gate(REPO / "packages", env)
        assert doc["status"] == "failed", f"PATH={env.get('PATH')!r}: {doc}"
        assert "PATH is empty or unset" in " ".join(doc["problems"]), doc
        assert RELATIVE_REFUSAL in " ".join(doc["problems"])


def test_emitted_cmake_refuses_a_bare_lfc_without_a_path(tmp_path):
    """Falsification: on af855e0b configure let an empty PATH through: find_program
    found nothing and the check was skipped (STATUS), while the lfc invocation and
    the Python gate fall back to their own default search paths."""
    from colcon_lf.generation import lfc_runtime as rt
    _unpatched_lfc(tmp_path / "lfc")
    env = {**os.environ, "PATH": ""}
    res = _run_cmake(_cmake_script(tmp_path, Path("lfc"), rt.patch_checker(REPO)),
                     cwd=tmp_path, env=env)
    out = _flat(res)
    assert res.returncode != 0, out
    assert "is a bare program name and PATH is empty or unset" in out, out
    assert RELATIVE_REFUSAL in out and "lfc runtime NOT checked" not in out, out


def test_cmake_templates_default_lfc_only_when_unset(tmp_path):
    """Falsification: on af855e0b both templates defaulted with ``if(NOT LFC_BIN)``, so
    LFC=OFF (a CMake false constant) silently became /opt/lfc/bin/lfc while the
    Python gate resolved and judged the program named OFF."""
    from colcon_lf.generation.lfc_runtime import cmake_runtime_check, lfc_binary
    off = _unpatched_lfc(tmp_path / "lfc").with_name("OFF")
    (off.parent / "lfc").rename(off)
    env = {**os.environ, "LFC": "OFF", "PATH": _abs_path(off.parent)}
    assert lfc_binary(env) == off
    for i, default in enumerate(_template_lfc_defaults()):
        script = tmp_path / f"default{i}.cmake"
        script.write_text(default + cmake_runtime_check(None)
                          + '\nmessage(STATUS "RESOLVED_LFC_BIN=${LFC_BIN}")\n')
        out = _flat(_run_cmake(script, cwd=tmp_path, env=env))
        assert f"RESOLVED_LFC_BIN={off.as_posix()}" in out, f"template {i}: {out}"


def test_emitted_cmake_judges_an_lfc_path_cmake_reads_as_false(tmp_path):
    """Falsification: on af855e0b presence was tested as a CMake boolean, so an
    executable whose path ends in -NOTFOUND counted as absent: the check was
    skipped and lfc ran unchecked."""
    from colcon_lf.generation import lfc_runtime as rt
    lfc = _unpatched_lfc(tmp_path / "lfc")
    not_found = lfc.with_name("lfc-NOTFOUND")
    lfc.rename(not_found)
    res = _run_cmake(_cmake_script(tmp_path, not_found, rt.patch_checker(REPO)), cwd=tmp_path)
    out = _flat(res)
    assert res.returncode != 0 and "is not the patched runtime" in out, out
    assert "lfc runtime NOT checked" not in out, out
    doc = _gate(REPO / "packages", {"LFC": str(not_found)})
    assert doc["status"] == "failed", doc


def test_emitted_cmake_accepts_a_semicolon_in_an_absolute_path_entry(tmp_path):
    """Falsification: on af855e0b the PATH went through a CMake list, so the absolute
    entry ``.../a;b`` split at the semicolon and ``b`` was refused as relative while
    the Python gate accepted that PATH."""
    from colcon_lf.generation.lfc_runtime import cmake_runtime_check, relative_lfc_error
    lfc = _unpatched_lfc(tmp_path / "lfc")
    odd = tmp_path / "a;b"
    odd.mkdir()
    path = _abs_path(odd, lfc.parent)
    assert relative_lfc_error("lfc", path) is None
    script = tmp_path / "check.cmake"
    script.write_text('set(LFC_BIN "lfc")\n' + cmake_runtime_check(None)
                      + '\nmessage(STATUS "RESOLVED_LFC_BIN=${LFC_BIN}")\n')
    res = _run_cmake(script, cwd=tmp_path, env={**os.environ, "PATH": path})
    out = _flat(res)
    assert res.returncode == 0 and "is relative" not in out, out
    assert f"RESOLVED_LFC_BIN={lfc.as_posix()}" in out, out


def test_emitted_cmake_notes_a_missing_lfc_without_a_catalog(tmp_path):
    """Falsification: on af855e0b the missing-lfc STATUS note lived inside the catalog
    branch; a package configured outside any checkout got only the catalog WARNING."""
    res = _run_cmake(_cmake_script(tmp_path, tmp_path / "nolfc" / "bin" / "lfc", None),
                     cwd=tmp_path)
    out = _flat(res)
    assert res.returncode == 0, out
    assert "lfc runtime NOT checked: no lfc executable at" in out, out
