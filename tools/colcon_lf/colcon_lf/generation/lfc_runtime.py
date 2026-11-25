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
"""The lfc runtime a generated program compiles against must be the patched one.

``lfc`` compiles every generated program against the reactor-c copy inside its
``lib/core-<ver>.jar``, not against ``third_party/reactor-c``. The dev image
patches that jar from ``third_party/patches/reactor-c`` at build time and the
read-back gate of the benchmark harness marks a run INVALID when the
running program lacks a patch marker. An image built before a patch existed
(the local ``:dev`` tag of 2026-10-12 predated the 0004 marker
patch of 2026-10-13) therefore produced programs whose every run ended
``read-back invalid: reactor_c_patches: runtime built without the configured
patches``, although the outputs were right.

``colcon lf-generate`` is the one step every build path runs before that
runtime enters a program, so it checks the lfc install here, with the
checker the Dockerfile uses (``patch_lfc_runtime.py --check``), and refuses
to generate from an unpatched runtime. The lfc install is resolved by the
rule the emitted CMake uses: ``$LFC``, else ``/opt/lfc/bin/lfc``. Without an
lfc install there is nothing to check (the build fails on its own); without
the patch catalog (a workspace outside the repository) there is no
expectation to check against, and the skip is reported.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Optional

DEFAULT_LFC = "/opt/lfc/bin/lfc"
CHECKER = Path("third_party") / "patches" / "patch_lfc_runtime.py"
REMEDY = ("the image predates third_party/patches or lfc is a foreign install; "
          "rebuild the image (docker compose build dev) or pull a current "
          "ghcr.io/cps-rwth/rcllf image")


RESOLUTION_RULE = ("the runtime check and the lfc invocation would resolve it from "
                   "different directories. Set LFC to an absolute path, "
                   "or to a bare program name resolved through PATH")


def relative_lfc_error(name: str, path: Optional[str] = None) -> Optional[str]:
    """Why ``$LFC`` cannot be resolved the same way everywhere, else None.

    The generate-time gate, the configure-time check and the lfc invocation
    run from different working directories, so a relative path with a
    directory part (``./bin/lfc``), or a bare name looked up through a PATH
    with a relative entry (``bin:/usr/bin``, an empty entry is the working
    directory) or through no PATH at all (each process would fall back to its
    own default search path), could have one lfc checked and another one build
    the program. An absolute path is used as given; a
    bare name resolves through a non-empty PATH whose entries are all absolute."""
    if os.path.isabs(name):
        return None
    if "/" in name:
        return (f"LFC={name} is a relative path with a directory part: "
                f"{RESOLUTION_RULE}.")
    if not path:
        return (f"LFC={name} is a bare program name and PATH is empty or unset: "
                f"{RESOLUTION_RULE} whose entries are all absolute.")
    for entry in path.split(os.pathsep):
        if not os.path.isabs(entry):
            return (f"LFC={name} is a bare program name and PATH entry '{entry}' is "
                    f"relative: {RESOLUTION_RULE} whose entries are all absolute.")
    return None


def lfc_binary(env: Optional[dict[str, str]] = None) -> Optional[Path]:
    """The lfc the emitted CMake runs (``$LFC``, else /opt/lfc/bin/lfc); None if absent.

    A relative name with a directory part never resolves (``relative_lfc_error``)."""
    env = os.environ if env is None else env
    name = env.get("LFC") or DEFAULT_LFC
    if relative_lfc_error(name, env.get("PATH")):
        return None
    if "/" not in name:  # a bare name: the executable PATH resolves, as CMake will
        found = shutil.which(name, path=env.get("PATH"))
        if found is None:
            return None
        name = found
    lfc = Path(name)
    return lfc if lfc.is_file() else None


def lfc_home(lfc: Path) -> Path:
    """The install dir of an lfc binary (``<home>/bin/lfc``)."""
    return lfc.resolve().parent.parent


def patch_checker(workspace: Optional[Path] = None) -> Optional[Path]:
    """``third_party/patches/patch_lfc_runtime.py`` of this checkout, if reachable.

    Looked for above *workspace* first (the checkout being generated for), then
    above this module (an editable install, possibly of another checkout)."""
    starts = []
    if workspace is not None:
        starts.append(Path(workspace).resolve())
    starts.append(Path(__file__).resolve())
    for start in starts:
        for parent in [start, *start.parents]:
            candidate = parent / CHECKER
            if candidate.is_file():
                return candidate
    return None


def cmake_quote(text: str) -> str:
    """*text* as the content of a CMake quoted argument: ``\\``, ``"`` and ``$`` escaped."""
    return text.replace("\\", "\\\\").replace('"', '\\"').replace("$", "\\$")


def cmake_runtime_check(checker: Optional[Path]) -> str:
    """CMake text that re-checks ``${LFC_BIN}`` when the generated package configures.

    The generate-time gate (``check_lfc_runtime``) is not enough on its own: a
    generated package persists in the workspace and may be rebuilt later, in a
    different image or with another ``$LFC``, without ``colcon lf-generate``
    running again. The emitted CMake therefore runs the same
    ``patch_lfc_runtime.py --check`` right before it invokes lfc.
    Without a patch catalog above the package the check is a WARNING, not a
    refusal, like the generate-time gate (a checkout always has one); without
    an lfc executable it is a STATUS note and lfc's own invocation reports the
    missing program (a directory counts as absent, as in the Python gate). A
    relative ``$LFC`` with a directory part, or a bare name with an empty PATH
    or a relative PATH entry, is refused here as in the Python gate
    (``relative_lfc_error``),
    and ``LFC_BIN`` is rewritten to the resolved executable so the lfc
    invocation runs the file that was checked. A bare name is resolved by
    walking PATH as ``shutil.which`` does (first entry holding an executable
    non-directory), not with ``find_program``: that would re-root entries into
    a toolchain's ``CMAKE_FIND_ROOT_PATH``, drop ``CMAKE_IGNORE_PATH`` entries
    and accept a non-executable file, so the host gate and the configure-time
    check could judge different files. The candidate and presence tests on
    lfc go through ``/usr/bin/test`` (else ``/bin/test``), not CMake's
    ``EXISTS`` (which wants read permission) nor a PATH-resolved ``test``;
    present means ``test -f``, as ``Path.is_file``. ``EXISTS`` still picks
    that binary and locates the checker, where no parity with the host gate
    is claimed.
    *checker* is embedded as a CMake quoted argument (``cmake_quote``).
    """
    hint = cmake_quote(checker.as_posix()) if checker is not None else ""
    return f"""\
# The generated program compiles against the reactor-c inside
# lfc's jar. Refuse an lfc install without the configured reactor-c patches
# here too, so a rebuild in a stale image fails before lfc runs rather than
# with "read-back invalid: reactor_c_patches" after every run. The checker is
# the checkout's {CHECKER.as_posix()}: the checkout above this package wins, the
# path known at generate time is a fallback (a package moved between checkouts
# must meet the new checkout's catalog).
if(NOT IS_ABSOLUTE "${{LFC_BIN}}" AND "${{LFC_BIN}}" MATCHES "/")
    message(FATAL_ERROR "LFC=${{LFC_BIN}} is a relative path with a directory part: "
        "{RESOLUTION_RULE}.")
endif()
# Resolve the executable once, here: an absolute path as given, a bare name
# through PATH (non-empty, every entry absolute; CMake 3.19 compatible: no
# NO_CACHE, so unset first). LFC_BIN becomes that file, so the lfc invocation
# below runs exactly the executable this check judged; only a bare name that
# PATH does not hold stays as given, and then the invocation's own PATH
# lookup (the shell's, like the host gate's) reports it. Names are compared as
# strings and tested as paths, never as CMake booleans: OFF or x-NOTFOUND is
# a program name like any other.
unset(_LFC_RESOLVED)
unset(_LFC_RESOLVED CACHE)
# The lfc candidate and presence tests go through a pinned system "test"
# binary (access(2) and stat(2), what shutil.which and Path.is_file ask):
# CMake's EXISTS also wants read permission, and a bare "test" could be
# shadowed by the PATH being searched. EXISTS stays for the two reads below
# that need no such parity: choosing that binary and locating the checker.
set(_lfc_test "")
foreach(_lfc_t /usr/bin/test /bin/test)
    if(EXISTS "${{_lfc_t}}" AND NOT IS_DIRECTORY "${{_lfc_t}}")
        set(_lfc_test "${{_lfc_t}}")
        break()
    endif()
endforeach()
if("${{_lfc_test}}" STREQUAL "")
    message(FATAL_ERROR "LFC=${{LFC_BIN}}: neither /usr/bin/test nor /bin/test exists "
        "to judge the lfc executable.")
endif()
if(IS_ABSOLUTE "${{LFC_BIN}}")
    set(_LFC_RESOLVED "${{LFC_BIN}}")
else()
    set(_lfc_path "$ENV{{PATH}}")
    if("${{_lfc_path}}" STREQUAL "")
        message(FATAL_ERROR "LFC=${{LFC_BIN}} is a bare program name and PATH is empty or "
            "unset: {RESOLUTION_RULE} whose entries are all absolute.")
    endif()
    # Every entry starts with "/" (checked on the string: a semicolon in an entry
    # is an ordinary character, not a CMake list separator).
    if(NOT "${{_lfc_path}}" MATCHES "^/[^:]*(:/[^:]*)*$")
        string(REGEX MATCH "(^|:)([^/:][^:]*)?(:|$)" _lfc_bad_entry "${{_lfc_path}}")
        message(FATAL_ERROR "LFC=${{LFC_BIN}} is a bare program name and PATH entry "
            "'${{CMAKE_MATCH_2}}' is relative: {RESOLUTION_RULE} whose entries are "
            "all absolute.")
    endif()
    # Walk PATH here, entry by entry, as shutil.which does. find_program would
    # not: a toolchain's CMAKE_FIND_ROOT_PATH re-roots the entries into its
    # sysroot, CMAKE_IGNORE_PATH hides entries, and under policy CMP0109 OLD a
    # readable but non-executable file is taken. ":" separates entries; a ";"
    # in an entry is an ordinary character (no CMake list is built).
    set(_lfc_rest "${{_lfc_path}}")
    while(NOT "${{_lfc_rest}}" STREQUAL "")
        string(FIND "${{_lfc_rest}}" ":" _lfc_colon)
        if(_lfc_colon EQUAL -1)
            set(_lfc_entry "${{_lfc_rest}}")
            set(_lfc_rest "")
        else()
            string(SUBSTRING "${{_lfc_rest}}" 0 ${{_lfc_colon}} _lfc_entry)
            math(EXPR _lfc_colon "${{_lfc_colon}} + 1")
            string(SUBSTRING "${{_lfc_rest}}" ${{_lfc_colon}} -1 _lfc_rest)
        endif()
        set(_lfc_cand "${{_lfc_entry}}/${{LFC_BIN}}")
        # shutil.which's test: executable for this user (X_OK) and not a directory.
        execute_process(COMMAND "${{_lfc_test}}" -x "${{_lfc_cand}}" -a ! -d "${{_lfc_cand}}"
            RESULT_VARIABLE _lfc_x)
        if(_lfc_x EQUAL 0)
            set(_LFC_RESOLVED "${{_lfc_cand}}")
            break()
        elseif(NOT _lfc_x EQUAL 1)
            message(FATAL_ERROR "LFC=${{LFC_BIN}} is a bare program name and ${{_lfc_test}} "
                "could not judge ${{_lfc_cand}}: ${{_lfc_x}}")
        endif()
    endwhile()
endif()
# Present means a regular file, symlinks followed, as the host gate's
# Path.is_file: a directory, a FIFO or a dangling link named lfc is absent.
set(_LFC_PRESENT FALSE)
if(IS_ABSOLUTE "${{_LFC_RESOLVED}}")
    execute_process(COMMAND "${{_lfc_test}}" -f "${{_LFC_RESOLVED}}" RESULT_VARIABLE _lfc_f)
    if(_lfc_f EQUAL 0)
        set(LFC_BIN "${{_LFC_RESOLVED}}")
        set(_LFC_PRESENT TRUE)
    elseif(NOT _lfc_f EQUAL 1)
        message(FATAL_ERROR "LFC=${{LFC_BIN}}: ${{_lfc_test}} could not judge "
            "${{_LFC_RESOLVED}}: ${{_lfc_f}}")
    endif()
endif()
set(LFC_PATCH_CHECK "LFC_PATCH_CHECK-NOTFOUND")
set(_lfc_dir "${{CMAKE_CURRENT_SOURCE_DIR}}")
while(NOT "${{_lfc_dir}}" STREQUAL "" AND NOT "${{_lfc_dir}}" STREQUAL "/")
    if(EXISTS "${{_lfc_dir}}/{CHECKER.as_posix()}")
        set(LFC_PATCH_CHECK "${{_lfc_dir}}/{CHECKER.as_posix()}")
        break()
    endif()
    get_filename_component(_lfc_parent "${{_lfc_dir}}" DIRECTORY)
    if("${{_lfc_parent}}" STREQUAL "${{_lfc_dir}}")
        break()
    endif()
    set(_lfc_dir "${{_lfc_parent}}")
endwhile()
if("${{LFC_PATCH_CHECK}}" STREQUAL "LFC_PATCH_CHECK-NOTFOUND" AND EXISTS "{hint}")
    set(LFC_PATCH_CHECK "{hint}")
endif()
if(NOT _LFC_PRESENT)
    # Nothing to check, catalog or not: the lfc invocation below reports the
    # missing executable.
    message(STATUS "[lf] lfc runtime NOT checked: no lfc executable at ${{LFC_BIN}} "
        "($LFC, else {DEFAULT_LFC}); the lfc invocation reports it")
elseif(NOT "${{LFC_PATCH_CHECK}}" STREQUAL "LFC_PATCH_CHECK-NOTFOUND")
    get_filename_component(_LFC_REAL "${{_LFC_RESOLVED}}" REALPATH)
    get_filename_component(_LFC_HOME "${{_LFC_REAL}}" DIRECTORY)
    get_filename_component(_LFC_HOME "${{_LFC_HOME}}" DIRECTORY)
    message(STATUS "[lf] checking lfc runtime ${{_LFC_HOME}} with ${{LFC_PATCH_CHECK}}")
    execute_process(
        COMMAND python3 "${{LFC_PATCH_CHECK}}" --check "${{_LFC_HOME}}"
        RESULT_VARIABLE _LFC_CHECK_RESULT
        OUTPUT_VARIABLE _LFC_CHECK_OUTPUT
        ERROR_VARIABLE _LFC_CHECK_OUTPUT
    )
    if(NOT _LFC_CHECK_RESULT EQUAL 0)
        message(FATAL_ERROR "lfc runtime ${{_LFC_HOME}} is not the patched runtime"
            ": {REMEDY}.\\n${{_LFC_CHECK_OUTPUT}}")
    endif()
else()
    message(WARNING "[lf] lfc runtime NOT checked: no {CHECKER.as_posix()} above "
        "${{CMAKE_CURRENT_SOURCE_DIR}}; a program built from an unpatched "
        "lfc fails every run's read-back")
endif()
"""


def check_lfc_runtime(workspace: Optional[Path] = None,
                      env: Optional[dict[str, str]] = None) -> dict[str, Any]:
    """Judge the lfc runtime the generated programs will compile against.

    Returns ``{"status": "ok"|"skipped"|"failed", "problems": [...], ...}`` with
    the checker's report (jar, markers expected and found) when it ran, or a
    ``reason`` when nothing could be checked."""
    env = os.environ if env is None else env
    relative = relative_lfc_error(env.get("LFC") or DEFAULT_LFC, env.get("PATH"))
    if relative:
        return {"status": "failed", "lfc": env.get("LFC"), "problems": [relative],
                "error": f"Error: {relative}"}
    lfc = lfc_binary(env)
    if lfc is None:
        return {"status": "skipped", "problems": [],
                "reason": f"no lfc install ($LFC or {DEFAULT_LFC})"}
    checker = patch_checker(workspace)
    if checker is None:
        return {"status": "skipped", "problems": [], "lfc": str(lfc),
                "reason": f"no {CHECKER} above {workspace or 'this install'}"}
    home = lfc_home(lfc)
    r = subprocess.run([sys.executable, str(checker), "--check", str(home)],
                       capture_output=True, text=True)
    doc: dict[str, Any] = {"status": "failed", "lfc": str(lfc), "checker": str(checker)}
    stdout, stderr = (r.stdout or "").strip(), (r.stderr or "").strip()
    try:
        report = json.loads(stdout.splitlines()[-1]) if stdout else None
    except (json.JSONDecodeError, IndexError):
        report = None
    if not isinstance(report, dict):
        doc["problems"] = [f"patch_lfc_runtime --check gave no report (exit {r.returncode}): "
                           f"{stderr or stdout}"]
        return doc
    doc.update(report)
    problems = list(report.get("problems") or [])
    if r.returncode != 0 and not problems:
        problems = [f"patch_lfc_runtime --check exit {r.returncode}: {stderr}"]
    doc["problems"] = problems
    doc["status"] = "failed" if problems else "ok"
    return doc


def lfc_runtime_error(doc: dict[str, Any]) -> str:
    """The refusal ``colcon lf-generate`` prints for a failed check."""
    if doc.get("error"):  # a usage refusal (relative $LFC), not a runtime verdict
        return doc["error"]
    jar = doc.get("jar") or doc.get("lfc")
    return (f"Error: lfc runtime {jar} is not the patched runtime: "
            + "; ".join(doc.get("problems") or ["unknown"])
            + f". {REMEDY}. A program generated from it would end every run "
            "'read-back invalid: reactor_c_patches: runtime built without the "
            "configured patches'.")
