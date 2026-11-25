# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""lf_eval — LTTng trace analysis for rcllf experiments.

Layered so that everything above ``reader`` is testable without CTF binaries:

``lf_eval.reader``
    CTF trace directory -> typed :class:`~lf_eval.reader.events.TraceEvent`
    records. The only layer that touches babeltrace.
``lf_eval.model``
    Run metadata + ``system.json`` topology + per-callback timelines built from
    event records; chain resolution.
``lf_eval.metrics``
    Extensible registry of metric classes (``compute`` + ``plot``).
``lf_eval.plots``
    Shared matplotlib styling (Agg backend).
``lf_eval.cli``
    ``lf-eval analyze | list-metrics | sample-cpu``.
"""

__version__ = "0.1.0"
