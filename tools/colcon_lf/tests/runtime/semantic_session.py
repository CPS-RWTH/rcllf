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
"""A private LTTng session around one generated-system run (T10).

The session daemon runs under a private `LTTNG_HOME` and is stopped by the PID
it was started with (the host PID namespace is shared, other users' daemons
exist). Channels `rcllf` and `ros2` carry the vpid, vtid and procname
contexts, as the benchmark harness records them. After the run the `lttng list` text is
captured before `destroy` and a `rcllf.manifest/1` run manifest
(`lf_eval.run_manifest`) is written next to `tracing/`, so the run directory is
what `lf-bench run` produces.
"""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import tempfile
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

from . import runtime_support as rts

CONTEXTS = ("vpid", "vtid", "procname")


@dataclass
class SessionSpec:
    rcllf_events: Sequence[str] = ("rcllf:*",)
    rcllf_filter: Optional[str] = None
    subbuf_size: str = "1M"
    num_subbuf: int = 8
    ros2: bool = True


@dataclass
class TracedRun:
    run_dir: Path
    log: str
    returncode: Optional[int]
    killed: bool
    list_output: str = ""
    extra: dict = field(default_factory=dict)


class Daemon:
    """lttng-sessiond under a private LTTNG_HOME, stopped by its own PID."""

    def __init__(self) -> None:
        self.home = Path(tempfile.mkdtemp(prefix="rcllf_lttng_"))
        self.env = dict(os.environ, LTTNG_HOME=str(self.home))
        self.proc = subprocess.Popen(
            ["lttng-sessiond"], env=self.env, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.lttng("list", check=False).returncode == 0:
                return
            time.sleep(0.2)
        self.stop()
        raise AssertionError("private lttng-sessiond did not come up")

    def lttng(self, *args: str, check: bool = True) -> subprocess.CompletedProcess:
        r = subprocess.run(["lttng", *args], env=self.env, capture_output=True, text=True)
        if check and r.returncode != 0:
            raise AssertionError(f"lttng {' '.join(args)} failed: {r.stdout}{r.stderr}")
        return r

    def stop(self) -> None:
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait(timeout=5)
        shutil.rmtree(self.home, ignore_errors=True)


@contextmanager
def daemon():
    d = Daemon()
    try:
        yield d
    finally:
        d.stop()


def _create(d: Daemon, name: str, out: Path, spec: SessionSpec) -> None:
    d.lttng("create", name, f"--output={out}")
    channels = []
    if spec.rcllf_events:
        channels.append(("rcllf", ",".join(spec.rcllf_events), spec.rcllf_filter))
    if spec.ros2:
        channels.append(("ros2", "ros2:*", None))
    for ch, events, flt in channels:
        d.lttng("enable-channel", "-u", ch, f"--subbuf-size={spec.subbuf_size}",
                f"--num-subbuf={spec.num_subbuf}", "-s", name)
        args = ["enable-event", "-u", events, "-c", ch, "-s", name]
        if flt:
            args += ["--filter", flt]
        d.lttng(*args)
        ctx = ["add-context", "-u"]
        for c in CONTEXTS:
            ctx += ["-t", c]
        d.lttng(*ctx, "-c", ch, "-s", name)
    d.lttng("start", name)


def static_manifest_for(case: rts.RuntimeCase) -> Path:
    name = ("rcllf_trace_manifest_federated.json" if case.gen_pkg.endswith("_federated")
            else "rcllf_trace_manifest.json")
    return rts._PACKAGES_ROOT / "lf-analysis" / case.source_pkg / case.launch_stem / name


def traced_launch(
    d: Daemon, case: rts.RuntimeCase, run_dir: Path, *, secs: float, spec: SessionSpec = SessionSpec(),
    env: Optional[dict] = None, until=None, until_timeout_s: float = 20.0,
    kill: signal.Signals = signal.SIGINT, write_manifest: bool = True, copy_static: bool = True,
    metadata: Optional[dict] = None,
) -> TracedRun:
    """Run *case* for *secs* (or until *until(log)*) inside a fresh session.

    The run manifest comes from lf_eval (shipped), not lf_bench (dev-only);
    a call with ``write_manifest=False`` needs neither.
    """
    if write_manifest:
        from lf_eval.run_manifest import (build_run_manifest, capture_clock_sync, tool_version,
                                          write_run_manifest)

    run_dir.mkdir(parents=True, exist_ok=True)
    # A caller may suffix the session name (for example with its branch) so
    # parallel sessions are told apart in `lttng list` and in the logs.
    name = f"rcllf_{run_dir.name}{os.environ.get('RCLLF_LTTNG_SESSION_SUFFIX', '')}"
    clock_start = capture_clock_sync() if write_manifest else None
    _create(d, name, run_dir / "tracing", spec)
    penv = dict(d.env)
    penv.setdefault("ROS_DOMAIN_ID", "89")
    penv.update(env or {})
    out_file = tempfile.TemporaryFile(mode="w+")
    proc = subprocess.Popen(
        ["ros2", "launch", case.gen_pkg, case.launch_file], cwd=str(rts._PACKAGES_ROOT),
        stdout=out_file, stderr=subprocess.STDOUT, text=True, start_new_session=True, env=penv)

    def log() -> str:
        return rts.read_capture(out_file)  # never seek: the child shares the offset

    time.sleep(secs)
    if until is not None:
        deadline = time.monotonic() + until_timeout_s
        while time.monotonic() < deadline and proc.poll() is None and not until(log()):
            time.sleep(0.2)
        time.sleep(0.5)
    killed = escalated = False
    if proc.poll() is None:
        os.killpg(proc.pid, kill)
        killed = True
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            escalated = True
            proc.wait(timeout=5)
    text = log()
    out_file.close()
    d.lttng("stop", name)
    list_output = d.lttng("list", name).stdout
    d.lttng("destroy", name)
    rc = proc.returncode
    # A hook set on a hooks-OFF build injected nothing, so skip the run,
    # but only once its own outcome is read: a died or hung run still fails.
    # The launcher's rc counts only for a run that ended on its own; one the
    # driver stopped reports the signal it was sent. A run that ended on its
    # own or was stopped with SIGINT must log a clean finish; an intentional
    # SIGKILL (or SIGTERM, which kills launch before it logs) cannot.
    rts.check_fault_hooks(text, rts.run_failure(
        text, returncode=None if killed else rc, sigkilled=escalated,
        need_clean=not killed or kill == signal.SIGINT))
    if write_manifest:
        doc = build_run_manifest(
            list_output=list_output, contexts=CONTEXTS, clock_start=clock_start,
            clock_end=capture_clock_sync(),
            launcher={"role": "launcher", "returncode": rc if rc is not None and rc >= 0 else None,
                      "signal": -rc if rc is not None and rc < 0 else None,
                      "stopped_by_driver": killed},
            launch_log=text,
            static_manifest=static_manifest_for(case),
            lttng_version=tool_version(["lttng", "--version"]),
            babeltrace_version=tool_version(["babeltrace2", "--version"]),
        )
        write_run_manifest(doc, run_dir)
        if copy_static and static_manifest_for(case).is_file():
            shutil.copy2(static_manifest_for(case), run_dir / "rcllf_trace_manifest.json")
    (run_dir / "launch.log").write_text(text)
    if metadata is not None:
        import yaml

        (run_dir / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    return TracedRun(run_dir=run_dir, log=text, returncode=rc, killed=killed, list_output=list_output)
