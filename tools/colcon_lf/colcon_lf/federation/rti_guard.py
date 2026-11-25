#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Run the LF RTI so that a signal to the launch does not kill it first.

Usage: rti_guard.py RTI -n <federates> -p <port> ...

reactor-c's RTI main() installs ``signal(SIGINT, exit)``. A Ctrl-C reaches
every process of the launch group at once, so the RTI called ``exit(2)``
before the federates had stopped, sent them MSG_TYPE_FAILED, and the
federates' own LF stop requests then failed on the closed socket (exit 1).

This guard starts the RTI in its own session, out of reach of the group
SIGINT, and ignores SIGINT itself (ros2 launch sends one at shutdown too).
The federates stop through their LF stop request, resign, and the RTI ends
on its own with code 0.

SIGTERM is forwarded after a grace period. A group SIGTERM, as
``timeout`` sends to ``ros2 launch``, reaches the federates and this guard at
once. Forwarded at once, it ended the RTI while the federates still waited
for it to grant their stop, and they hung. Within the grace the RTI grants
the stop and exits on its own; after it, ros2 launch's escalation (SIGTERM
after ``sigterm_timeout``, SIGKILL ``sigkill_timeout`` = 5 s later) still
ends the RTI, so the grace stays below that 5 s. ros2 launch answers a SIGTERM
to itself by exiting without stopping its processes, so the guard also gets
SIGTERM from the kernel when its parent dies (PR_SET_PDEATHSIG). The launch
file passes its own pid in ``RCLLF_EXIT_WITH_PARENT``: a ``getppid()`` taken
here would name init or a subreaper if launch had already died, and no death
signal would come. If launch is gone before the guard got here, the guard
exits 1 without starting the RTI. SIGKILL, the
last resort, cannot be forwarded, so the RTI gets SIGTERM from the kernel
when the guard dies. The guard exits with the RTI's code (128 + N for
signal N).

Lost federates (completeness of the control records). A
cross-federate payload is expected only from the control record that the
publisher's federate sends over its RTI connection (correspondence.cpp
``arrive_remote``). While publisher, RTI and receiver live, that record
cannot be lost: TCP delivers it in order and the RTI forwards it under its
mutex before any grant that would let the receiver pass its tag
(rti_remote.c ``handle_timed_message``). A record goes missing only when a
federate's connection ends without MSG_TYPE_RESIGN. The RTI then marks it
NOT_CONNECTED and skips it in later grants, but sends no new grant on the
close. In a run that SIGKILLed one federate (eebd1c56) the receiver stalled at logical
660 ms until its wall timeout and then printed "process has finished
cleanly". No record and no payload join names the loss. The guard reads the RTI's output, independently of any record, and
reports every federate whose connection closed before it resigned or that
reported an error. A run with such a federate does not pass: the guard
ends with LOST_FEDERATE_EXIT where the RTI itself ended 0.
"""

import ctypes
import os
import signal
import re
import shutil
import subprocess
import sys
import threading
import time

PR_SET_PDEATHSIG = 1
# Seconds between a SIGTERM to the guard and its forward to the RTI.
TERM_GRACE_S = float(os.environ.get("RCLLF_RTI_TERM_GRACE_S", "3"))
# Exit code of a run with a lost federate. Distinct from the
# RTI's own codes (0, 1 when a federate reported an error, 2 from its SIGINT
# exit), from 128 + N for a signal and from the binding's 3 (startup refusal,
# match_warmup), so the launch log names the cause.
LOST_FEDERATE_EXIT = 4
# Seconds the relay may take, after the RTI exited, to pass on what is left
# in its pipe (at most one pipe buffer, 64 KiB on Linux, read in well under a
# second by a live reader). A cap, not a wait: a relay still busy after it
# means our stdout stalled, and the run fails closed.
RELAY_DRAIN_S = 5.0

# The RTI's lines (rti_remote.c, lf_print_info / lf_print_error).
_RESIGNED = re.compile(r"RTI: Federate (\d+) has resigned\.")
_CLOSED = re.compile(r"RTI: Connection to federate (\d+) is closed\.")
_FAILED = re.compile(r"RTI: Federate (\d+) reports an error and has exited\.")


class LostFederates:
    """Federates whose RTI connection ended without a resign, from RTI lines."""

    def __init__(self):
        self.resigned: set[int] = set()
        self.lost: dict[int, str] = {}

    def feed(self, line: str) -> int | None:
        """Note one RTI output line; the federate id it reports lost, or None."""
        m = _RESIGNED.search(line)
        if m:
            self.resigned.add(int(m.group(1)))
            return None
        for rx, why in ((_FAILED, "reported an error and exited"),
                        (_CLOSED, "closed its RTI connection without resigning")):
            m = rx.search(line)
            if m and int(m.group(1)) not in self.resigned and int(m.group(1)) not in self.lost:
                self.lost[int(m.group(1))] = why
                return int(m.group(1))
        return None

    def line(self, fed: int) -> str:
        return (f"rti_guard: federate {fed} {self.lost[fed]}; the control records it "
                f"would have sent after that are missing and not counted by any receiver")

    def report(self) -> list[str]:
        return [self.line(f) for f in sorted(self.lost)]


def _die_with(parent: int) -> bool:
    """SIGTERM this process when ``parent`` dies; False if it already died.

    ``parent`` must come from the parent itself: compared before and after
    prctl, so a parent that died at any point before is noticed.
    """
    if os.getppid() != parent:
        return False
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_PDEATHSIG, signal.SIGTERM) != 0:
        # As the binding does: warn and run on; the parent may then outlive us.
        print(f"rti_guard: prctl(PR_SET_PDEATHSIG) failed: {os.strerror(ctypes.get_errno())}; "
              "the guard may outlive its launch", file=sys.stderr)
    return os.getppid() == parent


def _launch_pid() -> int | None:
    """The pid the launch file put in RCLLF_EXIT_WITH_PARENT, or None."""
    value = os.environ.get("RCLLF_EXIT_WITH_PARENT", "")
    if not value:
        return None
    if not value.isdigit() or int(value) <= 1:
        raise ValueError(
            f"RCLLF_EXIT_WITH_PARENT={value!r} is not the pid of the launching process; "
            "set it to str(os.getpid()) in the launch file"
        )
    return int(value)


def _rti_dies_with(parent: int):
    def hook():
        if not _die_with(parent):  # the guard died before prctl took effect
            os._exit(1)

    return hook


def main(argv: list[str]) -> int:
    if not argv:
        print("usage: rti_guard.py RTI [args...]", file=sys.stderr)
        return 2
    try:
        launch = _launch_pid()
    except ValueError as e:
        print(f"rti_guard: {e}", file=sys.stderr)
        return 2
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    term_at = []

    def on_term(_signum, _frame):
        if not term_at:  # a second SIGTERM (group, then parent death) adds nothing
            term_at.append(time.monotonic())

    signal.signal(signal.SIGTERM, on_term)
    if launch is None:
        # Started by hand: best effort only, the parent may already be gone.
        _die_with(os.getppid())
    elif not _die_with(launch):
        print(f"rti_guard: the launching process {launch} is gone; not starting the RTI",
              file=sys.stderr)
        return 1
    # reactor-c prints with vfprintf and no flush, so into a pipe its lines
    # would come in blocks, or not at all if the RTI dies by a signal.
    # stdbuf -oL makes them line by line where coreutils has it.
    stdbuf = shutil.which("stdbuf")
    if stdbuf:
        argv = [stdbuf, "-oL", *argv]
    rti = subprocess.Popen(argv, start_new_session=True, preexec_fn=_rti_dies_with(os.getpid()),
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    watch = LostFederates()

    def relay():
        # Pass the RTI's output through unchanged as it comes (a partial line
        # is not held back) and report each lost federate at once, from the
        # complete lines.
        out = sys.stdout.buffer
        pending = b""
        while True:
            chunk = os.read(rti.stdout.fileno(), 65536)
            if not chunk:
                break
            # Parse before the write: a reader of our stdout that stalls must
            # not keep a lost-federate line unparsed.
            pending += chunk
            *lines, pending = pending.split(b"\n")
            for raw in lines:
                fed = watch.feed(raw.decode(errors="replace"))
                if fed is not None:
                    # stderr, so it never lands inside a partial RTI line.
                    print(watch.line(fed), file=sys.stderr, flush=True)
            out.write(chunk)
            out.flush()
        if pending:
            watch.feed(pending.decode(errors="replace"))

    reader = threading.Thread(target=relay, daemon=True)
    reader.start()

    forwarded = False
    while True:
        try:
            rc = rti.wait(timeout=0.05)
            break
        except subprocess.TimeoutExpired:
            if term_at and not forwarded and time.monotonic() - term_at[0] >= TERM_GRACE_S:
                rti.send_signal(signal.SIGTERM)
                forwarded = True
    reader.join(timeout=RELAY_DRAIN_S)
    rc = rc if rc >= 0 else 128 - rc
    if reader.is_alive():
        # The relay is still blocked writing to our stdout, so RTI output may
        # be unread and a lost federate unseen. Fail closed, and leave with
        # os._exit: a normal exit would block flushing the stalled stdout.
        for line in watch.report():
            print(line, file=sys.stderr, flush=True)
        print(f"rti_guard: the RTI's output was not fully read within {RELAY_DRAIN_S:g} s "
              "of its exit; a lost federate cannot be ruled out", file=sys.stderr,
              flush=True)
        os._exit(rc if rc != 0 else LOST_FEDERATE_EXIT)
    for line in watch.report():
        print(line, file=sys.stderr, flush=True)
    if watch.lost and rc == 0:
        return LOST_FEDERATE_EXIT
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
