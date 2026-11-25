# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""CTF trace reading — the only layer that knows about babeltrace.

Everything downstream consumes :class:`~lf_eval.reader.events.TraceEvent`
records, so the model/metrics layers are testable from synthetic in-Python
fixtures without any CTF binaries.
"""

from lf_eval.reader.events import DiscardStats, TraceEvent
from lf_eval.reader.reader import (
    BackendUnavailableError,
    available_backends,
    find_tracing_dir,
    read_trace,
)

__all__ = [
    "BackendUnavailableError",
    "DiscardStats",
    "TraceEvent",
    "available_backends",
    "find_tracing_dir",
    "read_trace",
]
