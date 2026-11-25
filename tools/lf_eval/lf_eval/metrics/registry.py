# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The metric registry — ``lf_eval``'s extension point.

Adding a metric requires no changes to the CLI, the reader, or any existing
metric. Drop a module in :mod:`lf_eval.metrics`, subclass :class:`Metric`,
decorate it, and import it from ``lf_eval/metrics/__init__.py``::

    from lf_eval.metrics.registry import Metric, MetricResult, register

    @register("retries")
    class RetryMetric(Metric):
        '''Policy-linked retry counts per topic.'''

        name = "retries"
        summary = "Message retry counts per topic"

        def compute(self, runs, system=None, chains=(), **options):
            per_run = {r.name: _count_retries(r) for r in runs}
            return MetricResult(
                name=self.name,
                data={"per_run": per_run},
                summary_rows=[{"run": k, "retries": v} for k, v in per_run.items()],
            )

        def plot(self, result, outdir):
            ...  # return a list of written Paths; [] is fine

``compute`` receives **all** runs at once (not one at a time) so cross-run
metrics such as ``determinism`` work the same way as single-run ones. It must
tolerate an empty/insufficient input by returning a result with
``available=False`` rather than raising — the CLI reports skipped metrics
instead of aborting the whole analysis.

The roadmap's future policy-linked metrics (retries, deadline misses, queue
policy effects) are expected to slot in exactly like the example above.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Sequence, Type

from lf_eval.model.chain import Chain
from lf_eval.model.run import Run
from lf_eval.model.system import SystemModel


@dataclass
class MetricResult:
    """What a metric produces.

    :param name: the metric's registered name.
    :param data: JSON-serialisable payload, written to ``metrics.json``.
    :param summary_rows: flat rows for the CLI's summary table; each dict's keys
        become columns.
    :param available: ``False`` when the metric had insufficient input; ``note``
        should then explain why.
    :param note: human-readable remark shown in the CLI output.
    """

    name: str
    data: dict[str, Any] = field(default_factory=dict)
    summary_rows: list[dict[str, Any]] = field(default_factory=list)
    available: bool = True
    note: str = ""
    plots: list[str] = field(default_factory=list)
    #: evidence class: "identity" or "heuristic_timestamp_pairing"
    evidence: str = ""
    #: clock label, e.g. {"clock_domain": "same_host"} (D6)
    clock: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        doc = {
            "available": self.available,
            "note": self.note,
            "data": self.data,
            "summary": self.summary_rows,
            "plots": list(self.plots),
        }
        if self.evidence:
            doc["evidence"] = self.evidence
        if self.clock:
            doc["clock"] = dict(self.clock)
        return doc


class Metric:
    """Base class for all metrics.

    Subclasses set :attr:`name` / :attr:`summary` and implement
    :meth:`compute`; :meth:`plot` is optional.
    """

    #: Registered metric name (must match the ``@register`` argument).
    name: str = ""
    #: One-line description shown by ``lf-eval list-metrics``.
    summary: str = ""
    #: When True the metric is only meaningful with two or more runs.
    cross_run: bool = False

    # Results-folder interface (``ros2 lf eval``, run spec plan 3c).
    # All four are optional; a metric that sets none of them keeps working
    # unchanged, as a per-group, trace-reading metric without options.

    #: ``"per_group"`` (default): called once per (variant, sweep point) group
    #: with that group's own graph model. ``"cross_group"``: called once with
    #: the runs of every group, ``system=None``.
    scope: str = "per_group"
    #: What the metric reads, from :data:`INPUT_KINDS`. A run or group that
    #: lacks one makes the metric ``available=False`` with the reason; a
    #: missing ``lf_model`` on a stock group is ``not_applicable``.
    inputs: tuple[str, ...] = ("trace",)
    #: Explicit option opt-in: ``{key: type}``. ``None`` means the metric is
    #: always called as ``compute(runs, system, chains)`` and passing it an
    #: option is an eval error, never silently dropped.
    options_schema: Optional[dict[str, type]] = None

    def compare(self, baseline: MetricResult, candidate: MetricResult) -> list[dict[str, Any]]:
        """Delta rows of ``candidate`` against ``baseline`` (optional).

        Convention: ``delta = candidate - baseline``. A metric that does not
        override this has no comparison (see :func:`has_compare`).
        """
        raise NotImplementedError

    def compute(
        self,
        runs: Sequence[Run],
        system: Optional[SystemModel] = None,
        chains: Sequence[Chain] = (),
        **options: Any,
    ) -> MetricResult:
        """Compute the metric over ``runs``."""
        raise NotImplementedError

    def plot(self, result: MetricResult, outdir: Path) -> list[Path]:
        """Render standard plots for ``result`` into ``outdir``.

        Returns the written paths. The default implementation draws nothing.
        """
        return []


#: Input kinds a metric may declare in :attr:`Metric.inputs` (plan 3c).
INPUT_KINDS = ("trace", "outputs", "arrivals", "cpu", "bag", "graph_model", "lf_model")
SCOPES = ("per_group", "cross_group")


def has_compare(metric: Metric) -> bool:
    """``True`` when ``metric`` overrides :meth:`Metric.compare`."""
    return type(metric).compare is not Metric.compare


_REGISTRY: dict[str, Type[Metric]] = {}


def register(name: str) -> Callable[[Type[Metric]], Type[Metric]]:
    """Class decorator registering a :class:`Metric` under ``name``."""

    def decorator(cls: Type[Metric]) -> Type[Metric]:
        if name in _REGISTRY:
            raise ValueError(f"metric {name!r} is already registered")
        if not issubclass(cls, Metric):
            raise TypeError(f"{cls!r} must subclass Metric")
        if cls.scope not in SCOPES:
            raise ValueError(f"metric {name!r}: scope {cls.scope!r} not in {SCOPES}")
        unknown = [k for k in cls.inputs if k not in INPUT_KINDS]
        if unknown:
            raise ValueError(f"metric {name!r}: unknown inputs {unknown}; known {INPUT_KINDS}")
        cls.name = name
        _REGISTRY[name] = cls
        return cls

    return decorator


def get_metric(name: str) -> Metric:
    """Instantiate the metric registered under ``name``."""
    try:
        cls = _REGISTRY[name]
    except KeyError:
        known = ", ".join(sorted(_REGISTRY)) or "(none)"
        raise KeyError(f"unknown metric {name!r}; available: {known}") from None
    return cls()


def metric_names() -> list[str]:
    """All registered metric names, sorted."""
    return sorted(_REGISTRY)


def all_metrics() -> list[Metric]:
    """Instantiate every registered metric."""
    return [get_metric(name) for name in metric_names()]


def resolve_metrics(names: Optional[Iterable[str]] = None) -> list[Metric]:
    """Instantiate the named metrics, or all of them when ``names`` is falsy."""
    if not names:
        return all_metrics()
    return [get_metric(name) for name in names]
