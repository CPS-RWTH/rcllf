# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Metric registry and the v1 metric set.

Importing this package registers every built-in metric. To add one, create a
module here, subclass :class:`~lf_eval.metrics.registry.Metric`, decorate it with
``@register("<name>")``, and add the import below — see
:mod:`lf_eval.metrics.registry` for the full recipe.
"""

from lf_eval.metrics.registry import (
    INPUT_KINDS,
    Metric,
    MetricResult,
    all_metrics,
    has_compare,
    get_metric,
    metric_names,
    register,
    resolve_metrics,
)

# Importing these modules is what populates the registry.
from lf_eval.metrics import cpu as _cpu  # noqa: F401
from lf_eval.metrics import determinism as _determinism  # noqa: F401
from lf_eval.metrics import drops as _drops  # noqa: F401
from lf_eval.metrics import e2e_latency as _e2e_latency  # noqa: F401
from lf_eval.metrics import output_count as _output_count  # noqa: F401
from lf_eval.metrics import reproducibility as _reproducibility  # noqa: F401
from lf_eval.metrics import topology as _topology  # noqa: F401

__all__ = [
    "INPUT_KINDS",
    "Metric",
    "MetricResult",
    "all_metrics",
    "get_metric",
    "has_compare",
    "metric_names",
    "register",
    "resolve_metrics",
]
