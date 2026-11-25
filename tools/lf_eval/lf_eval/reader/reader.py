# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Backend selection and the public trace-reading entry point."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Iterator, Literal, Optional

from lf_eval.reader import babeltrace_cli, bt2_backend
from lf_eval.reader.events import DiscardStats, TraceEvent

logger = logging.getLogger(__name__)

Backend = Literal["bt2", "cli"]

# Subdirectory that docker/trace.sh writes CTF data into, relative to a run dir.
TRACING_SUBDIR = "tracing"


class BackendUnavailableError(RuntimeError):
    """Raised when no CTF backend is installed."""

    MESSAGE = (
        "No CTF trace backend available. Install either:\n"
        "  * the babeltrace2 Python bindings — `apt install python3-bt2` "
        "(preferred; already present in the ROS2 tracing images), or\n"
        "  * the babeltrace2 CLI — `apt install babeltrace2`.\n"
        "The rcllf dev container ships both; run `lf-eval` inside it, or use\n"
        "`docker compose run --rm dev` and re-run the same command."
    )

    def __init__(self, message: Optional[str] = None) -> None:
        super().__init__(message or self.MESSAGE)


def available_backends() -> list[Backend]:
    """Return the usable backends, most-preferred first."""
    found: list[Backend] = []
    if bt2_backend.is_available():
        found.append("bt2")
    if babeltrace_cli.is_available():
        found.append("cli")
    return found


def find_tracing_dir(path: Path) -> Path:
    """Resolve the CTF directory for ``path``.

    Accepts either an experiment run directory (containing ``tracing/``) or the
    CTF directory itself, so users can pass whichever they have at hand.
    """
    path = Path(path)
    candidate = path / TRACING_SUBDIR
    if candidate.is_dir():
        return candidate
    if not path.is_dir():
        raise FileNotFoundError(f"not a directory: {path}")
    return path


def read_trace(
    path: Path,
    backend: Optional[Backend] = None,
    event_names: Optional[set[str]] = None,
    discards: Optional[DiscardStats] = None,
) -> Iterator[TraceEvent]:
    """Stream :class:`TraceEvent` records from a run dir or CTF dir.

    :param path: experiment run directory or CTF trace directory.
    :param backend: force a backend; default picks ``bt2`` then ``cli``.
    :param event_names: when given, only yield events whose (prefix-stripped)
        name is in this set — a large win on the CLI backend, which otherwise
        materialises every ``rclcpp_executor_get_next_ready`` line.
    :param discards: when given, accumulates the bt2/CTF-side discarded-event
        cross-check (M7) as the trace streams past — a sibling out-param
        rather than a return-type change, so every existing
        ``list(read_trace(...))``/iterator call site is unaffected. Only
        meaningful once the returned iterator has been fully consumed (the
        generator can't know the total discard count until CTF end-of-stream);
        see :mod:`lf_eval.reader.bt2_backend`.
    :raises BackendUnavailableError: when no backend is installed.
    """
    tracing_dir = find_tracing_dir(path)

    if backend is None:
        backends = available_backends()
        if not backends:
            raise BackendUnavailableError()
        backend = backends[0]
    elif backend == "bt2" and not bt2_backend.is_available():
        raise BackendUnavailableError("bt2 bindings requested but not importable")
    elif backend == "cli" and not babeltrace_cli.is_available():
        raise BackendUnavailableError("babeltrace CLI requested but not on PATH")

    logger.debug("reading %s with backend=%s", tracing_dir, backend)
    module = bt2_backend if backend == "bt2" else babeltrace_cli
    stream = module.read_events(tracing_dir, discards=discards)
    if event_names is None:
        yield from stream
    else:
        for event in stream:
            if event.name in event_names:
                yield event
