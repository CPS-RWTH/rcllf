# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared plumbing for the runtime smoke layer.

The static playground (``tests/playground/``) only proves that the analyzer
and generator accept a launch file — it never runs the lfc-generated binary.
This module builds the small pieces every runtime case needs on top of that:

* ``CASES`` — the source package / generated package / launch file triple for
  each example this slice covers.
* ``require_case`` — called as each test's first line; skips (or, under
  ``RCLLF_REQUIRE_RUNTIME_SMOKE=1``, hard-fails) per case, so one case
  missing its build does not hide the others and CI can never silently skip.
  Same intent as ``RCLLF_REQUIRE_ROS_TESTS`` in ``tests/ros_env.py``,
  but checked at test-execution time rather than as a decorator — see
  ``require_case``'s own docstring for why.
* ``launch_and_capture`` — runs ``ros2 launch <gen_pkg> <launch_file>`` in its
  own process group, stops it with SIGINT to the group after ``run_secs``,
  and returns the captured combined stdout/stderr text. Empirically (verified
  in the dev container while writing this slice) a raw SIGTERM to the launch
  process group kills the ``ros2 launch`` parent via the OS default
  disposition before it reaches its asyncio shutdown path, so no
  "process has finished cleanly" line is ever printed; SIGINT reaches the
  same graceful-shutdown code path the binding's shutdown handling exercises
  (the ``rclcpp`` signal handler logs "signal_handler(SIGINT/SIGTERM)" for
  both, then the LF scheduler stops) *and* additionally lets ``ros2 launch``
  itself print the completion line this module asserts on. This is a
  deliberate deviation from a SIGTERM assumption; see the runtime README.
* ``find_pubsub_mismatches`` — the value-correspondence checker for the
  pubsub case, factored out so the negative fixture test
  (``test_negative_fixture_catches_value_mismatch``) can run it directly
  against a captured log without spinning up a real process.

Fixtures are never hand-written: every ``.log`` under ``fixtures/`` is a real
capture, per ``fixtures/README.md``.
"""

from __future__ import annotations

import dataclasses
import os
import re
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

REQUIRE_ENV = "RCLLF_REQUIRE_RUNTIME_SMOKE"

# tests/runtime/runtime_support.py -> repo root is five parents up
# (runtime -> tests -> colcon_lf -> tools -> <root>).
_REPO_ROOT = Path(__file__).resolve().parents[4]
_PACKAGES_ROOT = _REPO_ROOT / "packages"
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"


@dataclasses.dataclass(frozen=True)
class RuntimeCase:
    """One example's source package / generated package / launch file triple."""

    source_pkg: str
    launch_stem: str  # the launch file this was analyzed from, e.g. "bringup"
    gen_pkg: str  # colcon package produced by lf-generate
    launch_file: str  # file under <gen_pkg>/share/<gen_pkg>/launch/
    # Text that must appear (factory_marker) or must not appear
    # (factory_forbidden) in the generated src/node_factories.cpp, for cases
    # that share a gen_pkg but differ in generation options (
    # lifecycle_example with and without auto_transition; the default emits
    # no marker, the opt-out emits "/*auto_transition=*/false").
    factory_marker: str | None = None
    factory_forbidden: str | None = None


CASES = {
    "pubsub": RuntimeCase(
        source_pkg="pubsub_example",
        launch_stem="bringup",
        gen_pkg="lf_generated_pubsub_example_bringup",
        launch_file="lf_bringup.launch.py",
    ),
    "lifecycle": RuntimeCase(
        source_pkg="lifecycle_example",
        launch_stem="lifecycle_node",
        gen_pkg="lf_generated_lifecycle_example_lifecyclenode",
        launch_file="lf_lifecyclenode.launch.py",
        factory_forbidden="/*auto_transition=*/false",
    ),
    "lifecycle_manual": RuntimeCase(
        source_pkg="lifecycle_example",
        launch_stem="lifecycle_node",
        gen_pkg="lf_generated_lifecycle_example_lifecyclenode",
        launch_file="lf_lifecyclenode.launch.py",
        factory_marker="/*auto_transition=*/false",
    ),
    "parameter": RuntimeCase(
        source_pkg="parameter_example",
        launch_stem="parameter_node",
        # The stem equals the node class, so the generator appends "System".
        gen_pkg="lf_generated_parameter_example_parameternodesystem",
        launch_file="lf_parameternodesystem.launch.py",
    ),
    # Headerless node with a params file and an inline dict.
    "headerless_params": RuntimeCase(
        source_pkg="parameter_example",
        launch_stem="headerless_params",
        gen_pkg="lf_generated_parameter_example_headerlessparams",
        launch_file="lf_headerlessparams.launch.py",
    ),
    # Stress: five group sub-reactors, every timer at offset 0.
    "bind_order": RuntimeCase(
        source_pkg="callback_group_example",
        launch_stem="callback_group_node",
        gen_pkg="lf_generated_callback_group_example_callbackgroupnodesystem",
        launch_file="lf_callbackgroupnodesystem.launch.py",
    ),
    # Publication correspondence fixture (scenario via CORR_SCENARIO).
    "corr": RuntimeCase(
        source_pkg="correspondence_fixture",
        launch_stem="corr_bringup",
        gen_pkg="lf_generated_correspondence_fixture_corrbringup",
        launch_file="lf_corrbringup.launch.py",
    ),
    # BoundedNearestToAnchor over two sensors with a fixed stamp table.
    "bna": RuntimeCase(
        source_pkg="bna_example",
        launch_stem="bringup",
        gen_pkg="lf_generated_bna_example_bringup",
        launch_file="lf_bringup.launch.py",
    ),
    # tf2 acquisition-time lookups through rcllf_tf2::AcquisitionLookup
    # with LF-owned /tf and /tf_static ingress (cases T1 to T7).
    "tf2_acq": RuntimeCase(
        source_pkg="tf2_acq_example",
        launch_stem="bringup",
        gen_pkg="lf_generated_tf2_acq_example_bringup",
        launch_file="lf_bringup.launch.py",
    ),
    # fusion_example, example 4 of the learning path: message_filters
    # ExactTime and ApproximateTime over two timers (30 ms and 50 ms). The
    # public CI smokes it.
    "fusion": RuntimeCase(
        source_pkg="fusion_example",
        launch_stem="bringup",
        gen_pkg="lf_generated_fusion_example_bringup",
        launch_file="lf_bringup.launch.py",
    ),
    # brake_fusion, example 5 of the learning path: the same nodes as an rcllf
    # program (t2_approx, built with config/t2/subs.execution.yaml) and as
    # stock ROS 2 (t2_separate, launched straight from the source package).
    "brake_fusion_t2approx": RuntimeCase(
        source_pkg="brake_fusion",
        launch_stem="t2_approx",
        gen_pkg="lf_generated_brake_fusion_t2approx",
        launch_file="lf_t2approx.launch.py",
    ),
    "brake_fusion_t2separate": RuntimeCase(
        source_pkg="brake_fusion",
        launch_stem="t2_separate",
        gen_pkg="brake_fusion",
        launch_file="t2_separate.launch.py",
    ),
    # message_filters synchronizers on the mf-sync/1 domain (corpus
    # via SYNC_CORPUS); the fork/join variant is corpus P3.
    "sync_corpus": RuntimeCase(
        source_pkg="sync_oracle_showcase",
        launch_stem="corpus",
        gen_pkg="lf_generated_sync_oracle_showcase_corpus",
        launch_file="lf_corpus.launch.py",
    ),
    "sync_forkjoin": RuntimeCase(
        source_pkg="sync_oracle_showcase",
        launch_stem="forkjoin",
        gen_pkg="lf_generated_sync_oracle_showcase_forkjoin",
        launch_file="lf_forkjoin.launch.py",
    ),
    # corr_ping <-> corr_pong feedback loop.
    "corr_loop": RuntimeCase(
        source_pkg="correspondence_fixture",
        launch_stem="corr_loop",
        gen_pkg="lf_generated_correspondence_fixture_corrloop",
        launch_file="lf_corrloop.launch.py",
    ),
    # The causality loop corr_loop_echo with /corr/pong at d = 1 ms.
    "corr_loop_echo_delayed": RuntimeCase(
        source_pkg="correspondence_fixture",
        launch_stem="corr_loop_echo_delayed",
        gen_pkg="lf_generated_correspondence_fixture_corrloopechodelayed",
        launch_file="lf_corrloopechodelayed.launch.py",
    ),
    # Relay chain with analyzer-proven k = 1.
    "corr_relay": RuntimeCase(
        source_pkg="correspondence_fixture",
        launch_stem="corr_relay",
        gen_pkg="lf_generated_correspondence_fixture_corrrelaysystem",
        launch_file="lf_corrrelaysystem.launch.py",
    ),
    # /corr/a remapped to /corr/a_remapped on publisher and subscriber.
    "corr_remap": RuntimeCase(
        source_pkg="correspondence_fixture",
        launch_stem="corr_remap",
        gen_pkg="lf_generated_correspondence_fixture_corrremap",
        launch_file="lf_corrremap.launch.py",
    ),
    # One topic published from two group sub-reactors.
    "corr_two_group": RuntimeCase(
        source_pkg="correspondence_fixture",
        launch_stem="corr_two_group",
        gen_pkg="lf_generated_correspondence_fixture_corrtwogroup",
        launch_file="lf_corrtwogroup.launch.py",
    ),
    "corr_reentrant": RuntimeCase(
        source_pkg="correspondence_fixture",
        launch_stem="corr_reentrant",
        gen_pkg="lf_generated_correspondence_fixture_corrreentrant",
        launch_file="lf_corrreentrant.launch.py",
    ),
    # A long join of one reactor at the tag of another's publish.
    "corr_shared_join": RuntimeCase(
        source_pkg="correspondence_fixture",
        launch_stem="corr_shared_join",
        gen_pkg="lf_generated_correspondence_fixture_corrsharedjoin",
        launch_file="lf_corrsharedjoin.launch.py",
    ),
    # Two subscriptions on one topic in one node,
    # one correspondence edge each.
    "twin_sub": RuntimeCase(
        source_pkg="handle_matching_example",
        launch_stem="twin_sub",
        gen_pkg="lf_generated_handle_matching_example_twinsub",
        launch_file="lf_twinsub.launch.py",
    ),
    # Take mode skip on a late or lost sample, one writer and a fan-in.
    "take_one": RuntimeCase(
        source_pkg="take_skip_fixture",
        launch_stem="take_one",
        gen_pkg="lf_generated_take_skip_fixture_takeone",
        launch_file="lf_takeone.launch.py",
    ),
    "take_fan": RuntimeCase(
        source_pkg="take_skip_fixture",
        launch_stem="take_fan",
        gen_pkg="lf_generated_take_skip_fixture_takefan",
        launch_file="lf_takefan.launch.py",
    ),
    # Same-tag delivery on d = 0 links, one stage (ONE) versus a
    # two-node chain (CHAIN) of the same computation.
    "same_tag_one": RuntimeCase(
        source_pkg="same_tag_fixture",
        launch_stem="st_one",
        gen_pkg="lf_generated_same_tag_fixture_stone",
        launch_file="lf_stone.launch.py",
    ),
    "same_tag_chain": RuntimeCase(
        source_pkg="same_tag_fixture",
        launch_stem="st_chain",
        gen_pkg="lf_generated_same_tag_fixture_stchain",
        launch_file="lf_stchain.launch.py",
    ),
    "service": RuntimeCase(
        source_pkg="service_example",
        launch_stem="bringup",
        gen_pkg="lf_generated_service_example_bringup",
        launch_file="lf_bringup.launch.py",
    ),
}


def _missing_reasons(case: RuntimeCase) -> list[str]:
    """Why ``case`` is not runnable right now, or ``[]`` if it is.

    ``gen_pkg`` is package-qualified (``lf_generated_<package>_<stem>``), so
    two packages with a launch file of the same name (``bringup`` in
    ``pubsub_example`` and ``service_example``) never share an install, and
    only "was it built at all" needs checking.
    """
    missing = []
    if shutil.which("ros2") is None:
        missing.append("no `ros2` on PATH — run inside the dev container after sourcing the workspace")
        return missing
    install_dir = _PACKAGES_ROOT / "install" / case.gen_pkg
    if not install_dir.is_dir():
        missing.append(f"{install_dir} not built")
        return missing
    if case.factory_marker is not None or case.factory_forbidden is not None:
        factories = _PACKAGES_ROOT / "lf-generated" / case.gen_pkg / "src" / "node_factories.cpp"
        text = factories.read_text(encoding="utf-8") if factories.is_file() else ""
        if case.factory_marker is not None and case.factory_marker not in text:
            missing.append(
                f"{case.gen_pkg} was generated with other options ({case.factory_marker!r} "
                f"not in {factories}); rebuild with the matching LF_EXEC_CONFIG"
            )
        if case.factory_forbidden is not None and case.factory_forbidden in text:
            missing.append(
                f"{case.gen_pkg} was generated with other options ({case.factory_forbidden!r} "
                f"in {factories}); rebuild without LF_EXEC_CONFIG"
            )
    return missing


def require_case(case_key: str) -> RuntimeCase:
    """Call as the first line of a runtime test: skip or hard-fail per readiness.

    Deliberately evaluated at *test execution* time, not as a decorator —
    ``pytest.mark.skipif``/a raise in a decorator factory runs at module
    import (collection) time for every test in the file regardless of
    ``-k``/``-m`` filtering, which would make ``RCLLF_REQUIRE_RUNTIME_SMOKE=1``
    fail the whole module even when only one case's test was selected (found
    while wiring this up: filtering to `-k service` still hard-failed on the
    unrelated, unbuilt `pubsub` case's decorator). Checking inside the body
    means only the tests actually selected to run are ever evaluated.
    """
    case = CASES[case_key]
    missing = _missing_reasons(case)
    if not missing:
        return case
    reason = f"case {case_key!r} not runnable: {'; '.join(missing)}"
    if os.environ.get(REQUIRE_ENV) == "1":
        pytest.fail(f"{REQUIRE_ENV}=1 but {reason}")
    pytest.skip(reason)


# The correspondence fault hooks act only in a binding built with
# -DRCLLF_CORR_TEST_HOOKS=ON. An ordinary build injects nothing and says so at
# startup with this line; a test that sets a hook must not read that run.
HOOKS_COMPILED_OUT = "is set but the test hooks are compiled out"


def run_failure(log: str, *, returncode: int | None = None, sigkilled: bool = False,
                need_clean: bool = False) -> str | None:
    """What went wrong with a launch run itself, or ``None`` for a healthy run.

    ``sigkilled``: the harness had to escalate to SIGKILL. ``returncode``: the
    launcher's exit code, passed only where 0 is the healthy value (a run that
    ended on its own). ``need_clean``: the run must log ``process has finished
    cleanly``.
    """
    if sigkilled:
        return "the process had to be SIGKILLed"
    died = next((ln.strip() for ln in log.splitlines() if "process has died" in ln), None)
    if died is not None:
        return died
    if returncode is not None and returncode != 0:
        return f"the launcher exited with rc={returncode}"
    if need_clean and "process has finished cleanly" not in log:
        return "no 'process has finished cleanly' in the log"
    return None


def check_fault_hooks(log: str, failure: str | None = None) -> None:
    """Skip (or, under ``RCLLF_REQUIRE_RUNTIME_SMOKE=1``, fail) a run whose hooks did nothing.

    ``failure`` is the run's own outcome from :func:`run_failure`, read before
    this check: a run that died, crashed or hung fails even with the hooks
    compiled out, because no fault was injected and the failure is the
    binary's own. Only an otherwise healthy run is skipped.
    """
    if HOOKS_COMPILED_OUT not in log:
        return
    line = next(ln for ln in log.splitlines() if HOOKS_COMPILED_OUT in ln)
    if failure is not None:
        pytest.fail(f"the run failed ({failure}) although the correspondence fault hooks were "
                    f"compiled out and injected nothing: {line.strip()}\nlog tail:\n"
                    f"{log[-1500:]}")
    reason = ("a correspondence fault hook was set but ros2_lf_binding was built without "
              f"-DRCLLF_CORR_TEST_HOOKS=ON, so no fault was injected: {line.strip()}")
    if os.environ.get(REQUIRE_ENV) == "1":
        pytest.fail(f"{REQUIRE_ENV}=1 but {reason}")
    pytest.skip(reason)


@dataclasses.dataclass
class LaunchResult:
    log: str
    finished_cleanly: bool
    process_died: bool


def read_capture(out_file) -> str:
    """Everything a child has written so far to ``out_file``, its stdout.

    Reads with ``pread`` and leaves the file offset alone. The child shares
    this open file, so a ``seek(0)`` here would move its write position too:
    a line written right after the seek lands over the start of the log
    (a tick-5 line read as the first dispatch).
    """
    fd, chunks, offset = out_file.fileno(), [], 0
    while chunk := os.pread(fd, 1 << 20, offset):
        chunks.append(chunk)
        offset += len(chunk)
    return b"".join(chunks).decode("utf-8", errors="replace")


def launch_and_capture(
    case: RuntimeCase, run_secs: float, *, stop_timeout_s: float = 5.0,
    until=None, until_timeout_s: float = 20.0, settle_s: float = 0.5, allow_exit: bool = False,
    second_signal_after_s: float | None = None,
) -> LaunchResult:
    """Run ``ros2 launch <gen_pkg> <launch_file>`` for ``run_secs``, then stop it.

    Stops with SIGINT to the whole process group (never a bare ``pkill``/
    ``pgrep`` pattern match — this container shares the host PID namespace,
    so a pattern match would hit unrelated host processes). A case that
    cannot even find its own process group must fail loudly, never pass
    silently. ``allow_exit=True`` is for a run that ends itself (for example
    a callback calling ``rclcpp::shutdown``): its full log is returned instead
    of raising, with ``finished_cleanly`` read from the log as usual.

    ``second_signal_after_s`` sends a second SIGINT straight to the launched
    binaries that long after the group SIGINT, as launch's own forwarding does
    about 1 ms later.
    """
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "79")
    # A file, not a pipe: nothing reads a pipe during the run, and a traced
    # run fills its 64 KiB buffer, blocking the binary on write until the
    # SIGINT deadline expires (measured with RCLLF_TRACE_CALLBACKS runs).
    out_file = tempfile.TemporaryFile(mode="w+")
    proc = subprocess.Popen(
        ["ros2", "launch", case.gen_pkg, case.launch_file],
        cwd=str(_PACKAGES_ROOT),
        stdout=out_file,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )

    def read_all() -> str:
        return read_capture(out_file)

    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError as exc:  # pragma: no cover - defensive
        raise AssertionError(f"launch process {proc.pid} vanished immediately: {exc}") from exc

    time.sleep(run_secs)
    if until is not None:
        # Poll the captured output until the expected evidence is there (a
        # loaded host is slower than the fixed budget), then let it settle.
        deadline = time.monotonic() + until_timeout_s
        while time.monotonic() < deadline and proc.poll() is None and not until(read_all()):
            time.sleep(0.2)
        time.sleep(settle_s)

    if proc.poll() is not None and allow_exit:
        out = read_all()
        out_file.close()
        check_fault_hooks(out, run_failure(out, returncode=proc.returncode, need_clean=True))
        return LaunchResult(log=out, finished_cleanly="process has finished cleanly" in out,
                            process_died=False)
    if proc.poll() is not None:
        # It exited on its own before we ever asked it to — that is not the
        # "clean SIGINT stop" this harness is testing, so treat it as an
        # invalid run rather than silently reading whatever log it left.
        out = read_all()
        raise AssertionError(
            f"{case.gen_pkg} exited on its own after {run_secs}s (rc={proc.returncode}) "
            f"before the harness sent SIGINT; log tail:\n{out[-2000:]}"
        )

    children = _group_members(pgid) - {proc.pid}
    os.killpg(pgid, signal.SIGINT)
    if second_signal_after_s is not None:
        time.sleep(second_signal_after_s)
        for pid in children:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    try:
        proc.wait(timeout=stop_timeout_s)
        process_died = False
    except subprocess.TimeoutExpired:
        os.killpg(pgid, signal.SIGKILL)
        proc.wait(timeout=stop_timeout_s)
        process_died = True

    out = read_all()
    out_file.close()
    check_fault_hooks(out, run_failure(out, sigkilled=process_died, need_clean=True))
    finished_cleanly = "process has finished cleanly" in out
    return LaunchResult(log=out, finished_cleanly=finished_cleanly, process_died=process_died)


def _group_members(pgid: int) -> set[int]:
    """PIDs whose process group is ``pgid``, read from /proc (no pattern match)."""
    members = set()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
        except (OSError, IndexError):
            continue
        if int(fields[2]) == pgid:
            members.add(int(entry))
    return members


# ── interactive runs: ros2 CLI calls against a live binary ────


class LiveRun:
    """A launched case whose output is read live by a background thread."""

    def __init__(self, proc: subprocess.Popen, env: dict):
        import threading

        self.proc = proc
        self.env = env
        self._lines: list[str] = []
        self._lock = threading.Lock()
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.proc.stdout:
            with self._lock:
                self._lines.append(line)

    def log(self) -> str:
        with self._lock:
            return "".join(self._lines)

    def count(self, pattern: str) -> int:
        return len(re.findall(pattern, self.log()))

    def ros2(self, *args: str, timeout: float = 15.0, retry_s: float = 20.0) -> tuple[int, str]:
        """Run one ``ros2`` CLI command against the live system.

        Always ``--no-daemon`` (every verb used here accepts it): a ros2
        daemon started before the node exists can answer from a stale graph
        cache ("Node not found", seen in CI).

        Each ``--no-daemon`` call discovers the graph afresh, so on a loaded
        runner it can still miss a node that ``wait_for_node`` already saw
        (measured: rc 1 "Node not found" right after a successful wait). A
        "Node not found" means nothing was sent, so it is retried until
        ``retry_s`` runs out; any other result is returned as is.
        """
        deadline = time.monotonic() + retry_s
        while True:
            try:
                r = subprocess.run(
                    ["ros2", *args, "--no-daemon"], env=self.env, capture_output=True,
                    text=True, timeout=timeout,
                )
                rc, out = r.returncode, r.stdout + r.stderr
            except subprocess.TimeoutExpired as exc:
                out = exc.stdout.decode() if isinstance(exc.stdout, bytes) else (exc.stdout or "")
                rc = 124
            if "Node not found" not in out or time.monotonic() >= deadline:
                return rc, out
            time.sleep(0.5)


    def wait_for_node(self, fqn: str, timeout_s: float = 20.0) -> None:
        """Poll ``ros2 node list --no-daemon`` until ``fqn`` is in the graph.

        DDS discovery on a slow CI runner can take longer than the settle
        time; querying before it completes gives "Node not found".
        """
        deadline = time.monotonic() + timeout_s
        last = ""
        while time.monotonic() < deadline:
            rc, out = self.ros2("node", "list", timeout=10.0, retry_s=0.0)
            last = out
            if rc == 0 and fqn in out.split():
                return
            if self.proc.poll() is not None:
                break
            time.sleep(0.5)
        raise AssertionError(
            f"node {fqn} not in the ROS graph after {timeout_s:.0f} s "
            f"(`ros2 node list --no-daemon` last printed {last.split()!r})"
        )


def launch_interactive(case: RuntimeCase, during, *, settle_s: float = 4.0,
                       stop_timeout_s: float = 5.0, wait_nodes: tuple[str, ...] = ()) -> LaunchResult:
    """Launch ``case``, wait ``settle_s``, call ``during(run)``, then SIGINT the group.

    Same process-group discipline as :func:`launch_and_capture`. ``during``
    runs its assertions while the binary is alive; an exception from it still
    stops the process group before it propagates.
    """
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "79")
    env["RCLLF_TRACE_CALLBACKS"] = "1"
    proc = subprocess.Popen(
        ["ros2", "launch", case.gen_pkg, case.launch_file],
        cwd=str(_PACKAGES_ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True, env=env,
    )
    pgid = os.getpgid(proc.pid)
    run = LiveRun(proc, env)
    process_died = False
    try:
        time.sleep(settle_s)
        if proc.poll() is not None:
            raise AssertionError(
                f"{case.gen_pkg} exited on its own (rc={proc.returncode}) before the "
                f"harness acted; log tail:\n{run.log()[-2000:]}"
            )
        for fqn in wait_nodes:
            run.wait_for_node(fqn)
        during(run)
    finally:
        try:
            os.killpg(pgid, signal.SIGINT)
            proc.wait(timeout=stop_timeout_s)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=stop_timeout_s)
            process_died = True
        except ProcessLookupError:
            pass
        run._reader.join(timeout=2.0)
    out = run.log()
    return LaunchResult(
        log=out, finished_cleanly="process has finished cleanly" in out, process_died=process_died,
    )


# ── pubsub value-correspondence checker ────────────────────────────────────

_PUBLISH_RE = re.compile(
    r"\[publisher_node\]: Publishing: temp=([-\d.]+), humidity=([-\d.]+), pressure=([-\d.]+)"
)
_HEARD_RE = re.compile(
    r"\[subscriber_node\]: I heard: temp=([-\d.]+), humidity=([-\d.]+), pressure=([-\d.]+)"
)


def parse_pubsub_log(log_text: str) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
    """Parse publisher/subscriber log lines into ordered value tuples.

    Matches ``publisher_node.cpp:41`` ("Publishing: temp=%.1f, ...") and
    ``subscriber_node.cpp:30`` ("I heard: temp=%.1f, ...").
    """
    published = [tuple(float(v) for v in m.groups()) for m in _PUBLISH_RE.finditer(log_text)]
    heard = [tuple(float(v) for v in m.groups()) for m in _HEARD_RE.finditer(log_text)]
    return published, heard


def find_pubsub_mismatches(
    published: list[tuple[float, float, float]], heard: list[tuple[float, float, float]]
) -> list[str]:
    """Check ``heard`` against ``published``: identity, order, and loss bounds.

    Returns a list of human-readable problems; empty means the run is sound.
    """
    errors: list[str] = []
    if len(published) < 5:
        errors.append(f"only {len(published)} published message(s), need >= 5")

    idx_map: list[int] = []
    search_from = 0
    for pos, h in enumerate(heard):
        found = None
        for i in range(search_from, len(published)):
            if published[i] == h:
                found = i
                break
        if found is None:
            errors.append(
                f"heard value at position {pos} ({h}) does not match any published "
                f"value at or after published index {search_from} — value "
                "correspondence or order violated"
            )
            # Keep scanning from the same point so one bad value doesn't
            # cascade into a false report for every value after it.
            continue
        idx_map.append(found)
        search_from = found + 1

    if idx_map:
        lost_at_start = idx_map[0]
        lost_at_end = (len(published) - 1) - idx_map[-1]
        mid_gaps = sum(b - a - 1 for a, b in zip(idx_map, idx_map[1:]))
        if mid_gaps > 0:
            errors.append(f"{mid_gaps} publish(es) lost mid-stream (not at start/stop)")
        if lost_at_start + lost_at_end > 1:
            errors.append(
                f"more than one publish lost at start/stop: start={lost_at_start} end={lost_at_end}"
            )
    elif published:
        errors.append("no heard value matched any published value")

    return errors


# ── lifecycle ────────────────────────────────────────────────────────────

_LIFECYCLE_ACTIVE_RE = re.compile(r"LifecycleExampleNode has been activated")
_LIFECYCLE_PUBLISH_RE = re.compile(r"\[lifecycle_node\]: Publishing: 'Lifecycle tick: (\d+)'")


def parse_lifecycle_log(log_text: str) -> tuple[bool, list[int]]:
    """Matches ``lifecycle_node.cpp``'s on_activate log and its publish ticks."""
    reached_active = bool(_LIFECYCLE_ACTIVE_RE.search(log_text))
    ticks = [int(m.group(1)) for m in _LIFECYCLE_PUBLISH_RE.finditer(log_text)]
    return reached_active, ticks


# ── service ──────────────────────────────────────────────────────────────

# The logger name carries the launch namespace ("service_example.service_client")
# since the example nodes take NodeOptions; a bare "service_client" stays valid.
_SERVICE_RESULT_RE = re.compile(
    r"\[(?:[\w.]+\.)?service_client\]: Result: ([-\d.]+), success=(true|false), message='([^']*)'"
)


def parse_service_results(log_text: str) -> list[tuple[float, bool, str]]:
    """Matches ``service_client_node.cpp``'s "Result: ..." log line."""
    out = []
    for m in _SERVICE_RESULT_RE.finditer(log_text):
        out.append((float(m.group(1)), m.group(2) == "true", m.group(3)))
    return out


def expected_service_result(index: int) -> tuple[str, bool, str]:
    """Derive the expected (result_str, success, message) for request ``index``.

    From ``service_client_node.cpp``: ``values = {1.0, 2.0, 3.0, index*0.1}``,
    ``method = "sum" if index % 2 == 0 else "average"``. From
    ``service_server_node.cpp``: sum accumulates all values; average divides
    by the count (always 4 here, since ``values`` is never empty).
    """
    values = [1.0, 2.0, 3.0, index * 0.1]
    total = sum(values)
    if index % 2 == 0:
        return f"{total:.2f}", True, "Computed sum"
    return f"{total / len(values):.2f}", True, "Computed average"
