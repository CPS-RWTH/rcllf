# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Clean SIGINT exit of a federated launch.

reactor-c's RTI installs ``signal(SIGINT, exit)``, so a SIGINT to the launch
process group ended it with ``exit(2)`` before the federates could stop, and
the federates then exited 1 on the reset RTI socket. The generated
orchestrator runs the RTI under ``rti_guard.py``, which keeps the group
SIGINT away from it so the RTI ends normally once every federate resigned.
"""

from __future__ import annotations

import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

from colcon_lf.federation import rti_guard

from .test_xfed_correspondence import FED, _pubsub

GUARD = Path(__file__).resolve().parents[2] / "colcon_lf/federation/rti_guard.py"

# Stands in for the RTI: like reactor-c's RTI main() it exits 2 on SIGINT,
# and otherwise ends with code 0 after the federates "resigned" (here, after
# a fixed time).
FAKE_RTI = (
    "import signal, sys, time\n"
    "signal.signal(signal.SIGINT, lambda *a: sys.exit(2))\n"
    "time.sleep(1.5)\n"
)


def _start_guarded(child_code: str) -> subprocess.Popen:
    # Own process group, so the test can signal the group like Ctrl-C does.
    return subprocess.Popen(
        [sys.executable, str(GUARD), sys.executable, "-c", child_code],
        start_new_session=True,
    )


def test_orchestrator_runs_the_rti_through_the_guard(tmp_path):
    _pubsub(tmp_path)
    launch_dir = tmp_path / "lf-generated" / FED / "launch"
    text = (launch_dir / "lf_bringup_federated.launch.py").read_text()
    assert "rti_guard.py" in text
    assert '"RTI", "-n", "2", "-p", rti_port' in text
    guard = launch_dir / "rti_guard.py"
    assert guard.read_text() == GUARD.read_text()
    # The launch process's pid, evaluated in the launch process.
    assert 'additional_env={"RCLLF_EXIT_WITH_PARENT": str(os.getpid())}' in text
    compile(text, "lf_bringup_federated.launch.py", "exec")


def test_group_sigint_does_not_reach_the_rti():
    """Without the guard the stand-in exits 2, as the RTI once did."""
    bare = subprocess.Popen([sys.executable, "-c", FAKE_RTI], start_new_session=True)
    time.sleep(0.5)
    os.killpg(bare.pid, signal.SIGINT)
    assert bare.wait(timeout=10) == 2

    guarded = _start_guarded(FAKE_RTI)
    time.sleep(0.5)
    os.killpg(guarded.pid, signal.SIGINT)  # Ctrl-C to the launch group
    guarded.send_signal(signal.SIGINT)      # ros2 launch's own SIGINT at shutdown
    assert guarded.wait(timeout=10) == 0


def test_sigterm_is_forwarded_to_the_rti():
    """ros2 launch escalates to SIGTERM, the RTI must still go away.

    After the grace, and before launch's SIGKILL 5 s later.
    """
    guarded = _start_guarded("import time\ntime.sleep(30)\n")
    time.sleep(0.5)
    t0 = time.monotonic()
    guarded.send_signal(signal.SIGTERM)
    assert guarded.wait(timeout=10) == 128 + signal.SIGTERM
    took = time.monotonic() - t0
    assert rti_guard.TERM_GRACE_S - 0.1 <= took < 5


def test_group_sigterm_leaves_the_rti_time_to_grant_the_stop():
    """``timeout`` sends SIGTERM to the whole launch group.

    The federates then request their LF stop, which the RTI must still be
    there to grant. Forwarded at once, the SIGTERM ended the RTI first and the
    federates hung. The stand-in RTI ends on its own 1 s after the signal, as
    the real one does once every federate resigned; it must end with 0, not
    with the forwarded SIGTERM.
    """
    guarded = _start_guarded(
        "import signal, sys, time\n"
        "signal.signal(signal.SIGTERM, lambda *a: sys.exit(143))\n"
        "signal.signal(signal.SIGUSR1, lambda *a: (time.sleep(1), sys.exit(0)))\n"
        "time.sleep(30)\n"
    )
    time.sleep(0.5)
    guarded.send_signal(signal.SIGTERM)  # what timeout sends to the group
    _signal_child(guarded.pid, signal.SIGUSR1)  # the federates resign
    assert guarded.wait(timeout=10) == 0, "the guard forwarded SIGTERM before the RTI was done"


def test_guard_gets_sigterm_when_launch_dies():
    """ros2 launch exits on SIGTERM without stopping its processes.

    The guard must then end the RTI anyway (here after the grace, as the RTI
    stand-in never ends on its own).
    """
    launch = subprocess.Popen(
        [sys.executable, "-c",
         "import os, subprocess, sys, time\n"
         f"g = subprocess.Popen([sys.executable, {str(GUARD)!r}, sys.executable, '-c',"
         " 'import time; time.sleep(300)'],"
         " env={**os.environ, 'RCLLF_EXIT_WITH_PARENT': str(os.getpid())})\n"
         "print(g.pid, flush=True)\n"
         "time.sleep(300)\n"],
        stdout=subprocess.PIPE, start_new_session=True,
    )
    try:
        assert _readable(launch.stdout.fileno(), WAIT_CAP_S)
        guard_fd = os.pidfd_open(int(launch.stdout.readline()))
        time.sleep(0.5)
        launch.kill()  # launch gone, no signal reaches the guard from it
        launch.wait()
        assert _readable(guard_fd, rti_guard.TERM_GRACE_S + WAIT_CAP_S), \
            "the guard outlived its launch"
        os.close(guard_fd)
    finally:
        if launch.poll() is None:
            launch.kill()
            launch.wait()
        launch.stdout.close()


def test_guard_whose_launch_died_first_does_not_start_the_rti(tmp_path):
    """Launch dies before the guard reaches its prctl.

    The guard's parent is then init or a subreaper. A ``getppid()`` taken in
    the guard would accept it, no death signal would come, and the RTI would
    run on. The pid from the launch file shows the mismatch.
    """
    started = tmp_path / "rti_started"
    rti = f"import pathlib, time; pathlib.Path({str(started)!r}).touch(); time.sleep(300)"
    # Waits until its parent, the stand-in launch, is gone, then becomes the guard.
    late_start = (
        'while [ "$(cut -d" " -f4 /proc/$$/stat)" = "$RCLLF_EXIT_WITH_PARENT" ]; '
        'do sleep 0.05; done; '
        'exec "$0" "$@"'
    )
    launch = subprocess.Popen(
        [sys.executable, "-c",
         "import os, subprocess, sys\n"
         f"g = subprocess.Popen(['sh', '-c', {late_start!r}, sys.executable,"
         f" {str(GUARD)!r}, sys.executable, '-c', {rti!r}],"
         " env={**os.environ, 'RCLLF_EXIT_WITH_PARENT': str(os.getpid())})\n"
         "print(g.pid, flush=True)\n"],
        stdout=subprocess.PIPE, start_new_session=True,
    )
    guard_fd = None
    try:
        assert _readable(launch.stdout.fileno(), WAIT_CAP_S)
        guard_fd = os.pidfd_open(int(launch.stdout.readline()))
        launch.wait(timeout=WAIT_CAP_S)  # gone before the guard starts
        assert _readable(guard_fd, WAIT_CAP_S), "the guard outlived its launch"
        assert not started.exists(), "the guard started the RTI after its launch died"
    finally:
        if guard_fd is not None:
            try:
                signal.pidfd_send_signal(guard_fd, signal.SIGKILL)
            except ProcessLookupError:
                pass
            os.close(guard_fd)
        if launch.poll() is None:
            launch.kill()
            launch.wait()
        launch.stdout.close()


def test_guard_refuses_a_value_that_is_not_a_pid():
    """The old ``RCLLF_EXIT_WITH_PARENT=1`` named no pid; it is refused."""
    guarded = subprocess.run(
        [sys.executable, str(GUARD), sys.executable, "-c", "pass"],
        env={**os.environ, "RCLLF_EXIT_WITH_PARENT": "1"},
        capture_output=True, text=True, timeout=WAIT_CAP_S,
    )
    assert guarded.returncode == 2
    assert "not the pid of the launching process" in guarded.stderr


def _signal_child(guard_pid: int, signum: int) -> None:
    """Signal the guard's only child, the stand-in RTI."""
    deadline = time.monotonic() + WAIT_CAP_S
    while time.monotonic() < deadline:
        try:
            children = Path(f"/proc/{guard_pid}/task/{guard_pid}/children").read_text().split()
        except FileNotFoundError:
            children = []
        if children:
            os.kill(int(children[0]), signum)
            return
        time.sleep(0.05)
    raise AssertionError("the guard never started the stand-in RTI")


# Under a saturated CPU, starting the guard and the stand-in RTI (two Python
# interpreters) took 12 to 18 s in the reproduction, far past the 5 s the
# test used to allow. Both waits below wake on the event itself, so the cap
# only bounds a hang and costs nothing when the host is idle.
WAIT_CAP_S = 60.0


def _readable(fd: int, timeout_s: float) -> bool:
    poller = select.poll()
    poller.register(fd, select.POLLIN)
    return bool(poller.poll(timeout_s * 1000))


def test_a_killed_guard_takes_the_rti_with_it():
    """ros2 launch's last resort is SIGKILL, which the guard cannot forward.

    The RTI runs in its own session, so without a parent death signal it
    would outlive the guard and hold its port for the next launch.
    """
    # The stand-in RTI reports its pid on the stdout it inherits through the
    # guard (the guard closes other fds for its child) once it runs, then
    # blocks. A pidfd (taken while it is alive, so immune to pid reuse) becomes
    # readable the moment it exits.
    guarded = subprocess.Popen(
        [sys.executable, str(GUARD), sys.executable, "-c",
         "import os, time\nos.write(1, str(os.getpid()).encode())\ntime.sleep(300)\n"],
        start_new_session=True,
        stdout=subprocess.PIPE,
    )
    read_end = guarded.stdout.fileno()
    rti_fd = None
    try:
        assert _readable(read_end, WAIT_CAP_S), \
            f"the stand-in RTI did not start within {WAIT_CAP_S:.0f} s"
        out = os.read(read_end, 32)
        assert out, f"the guard exited with {guarded.wait()} before the RTI started"
        rti_pid = int(out)
        rti_fd = os.pidfd_open(rti_pid)
        guarded.kill()
        guarded.wait(timeout=WAIT_CAP_S)
        assert _readable(rti_fd, WAIT_CAP_S), "the RTI outlived its SIGKILLed guard"
    finally:
        if guarded.poll() is None:
            guarded.kill()
            guarded.wait()
        if rti_fd is not None:
            try:
                signal.pidfd_send_signal(rti_fd, signal.SIGKILL)
            except ProcessLookupError:
                pass  # it has already ended
            os.close(rti_fd)
        guarded.stdout.close()
