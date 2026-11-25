# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Plot styling shared by every metric.

The ``Agg`` backend is selected on import so plotting works headless (CI, Docker,
SSH) with no display. Colours encode the experiment ``mode`` axis, which is the
durable axis of comparison across the whole evaluation. ``mode`` is an **open
string** (see ``lf_eval.model.run.MODES``), not a closed enum, so nothing here
splits a mode string apart (no ``-`` splitting, no substring mangling) — every
mode is looked up/hashed as one opaque token, exactly as read from
``metadata.yaml``. Five well-known Phase 6 values plus the pre-Phase-6 pair:

* ``stock-single``/``stock-multi``/``stock-events`` — stock ROS2, one rclcpp
  executor class each (SingleThreaded/MultiThreaded/Events)
* ``lfc``    — compiled LF path (lfc + ros2_lf_binding)
* ``rslcpp`` — RSLCPP assessment variant
* ``lf``     — LF executor bridge (pre-Phase-6 data)
* ``stock``  — plain ROS2 executor, unspecified (pre-Phase-6 data)

A mode string outside this known set still gets a **distinct, deterministic**
colour (hashed into a fixed fallback palette) rather than one shared grey, and
``mode_label`` returns the raw string unchanged — both keep an operator-added
sixth variant (or a typo'd mode string) distinguishable instead of invisible.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Any, Iterable, Optional

import matplotlib

matplotlib.use("Agg")  # headless; must precede the pyplot import
import matplotlib.pyplot as plt  # noqa: E402

MODE_COLORS = {
    "stock-single": "#FF5722",  # deep orange — stock, SingleThreadedExecutor
    "stock-multi": "#FF9800",   # orange      — stock, MultiThreadedExecutor
    "stock-events": "#FFC107",  # amber       — stock, EventsExecutor
    "lfc": "#1565C0",           # dark blue   — compiled LF
    "rslcpp": "#8E24AA",        # purple      — RSLCPP
    "lf": "#2196F3",            # blue        — LF executor (bridge, pre-Phase-6)
    "stock": "#E64A19",         # burnt orange — stock, unspecified (pre-Phase-6)
    "unknown": "#9E9E9E",
}

MODE_LABELS = {
    "stock-single": "Stock ROS2 (SingleThreaded)",
    "stock-multi": "Stock ROS2 (MultiThreaded)",
    "stock-events": "Stock ROS2 (Events)",
    "lfc": "lfc (compiled LF)",
    "rslcpp": "RSLCPP (simulated time)",
    "lf": "LF executor (bridge)",
    "stock": "Stock ROS2",
    "unknown": "unknown mode",
}

#: Mode plotting order.
MODE_ORDER = ("stock-single", "stock-multi", "stock-events", "lfc", "rslcpp", "lf", "stock", "unknown")

DPI = 150

#: Deterministic fallback palette for mode strings outside MODE_COLORS —
#: distinct colours, not one shared grey, so an unrecognized/future mode is
#: still visually distinguishable from every other unrecognized mode.
_FALLBACK_PALETTE = (
    "#00897B",  # teal
    "#5D4037",  # brown
    "#3949AB",  # indigo
    "#C0CA33",  # lime
    "#D81B60",  # pink
    "#6D4C41",  # dark brown
    "#00ACC1",  # cyan
    "#7CB342",  # light green
)


def _fallback_color(mode: str) -> str:
    """Deterministic colour for a mode string not in :data:`MODE_COLORS`.

    Hashes the mode string (whole, never split on ``-`` or any other
    separator — mode strings like ``stock-multi`` are opaque tokens) into a
    fixed index of :data:`_FALLBACK_PALETTE`, so the same unknown mode always
    gets the same colour across runs/plots, and distinct unknown modes are
    very likely to get distinct colours.
    """
    digest = hashlib.sha256(mode.encode("utf-8")).digest()
    index = digest[0] % len(_FALLBACK_PALETTE)
    return _FALLBACK_PALETTE[index]


def mode_color(mode: str) -> str:
    """Colour for an experiment mode.

    Known modes (:data:`MODE_COLORS`) get their fixed colour; any other
    non-empty mode string gets a deterministic fallback colour (see
    :func:`_fallback_color`) instead of a single shared grey. Only the empty
    string / literal ``"unknown"`` fall back to the shared grey.
    """
    if mode in MODE_COLORS:
        return MODE_COLORS[mode]
    if not mode or mode == "unknown":
        return MODE_COLORS["unknown"]
    return _fallback_color(mode)


def mode_label(mode: str) -> str:
    """Legend label for an experiment mode."""
    return MODE_LABELS.get(mode, mode)


def present_modes(modes: Iterable[str]) -> list[str]:
    """Filter+order the modes actually present in a dataset."""
    seen = set(modes)
    ordered = [m for m in MODE_ORDER if m in seen]
    ordered.extend(sorted(seen - set(MODE_ORDER)))
    return ordered


def figure(
    title: str = "",
    subtitle: str = "",
    figsize: tuple[float, float] = (10.0, 5.0),
    ncols: int = 1,
) -> tuple[Any, Any]:
    """Create a styled figure. Returns ``(fig, axes)``.

    ``axes`` is a single Axes when ``ncols == 1``, else an array.
    """
    fig, axes = plt.subplots(1, ncols, figsize=figsize)
    if title:
        heading = f"{title}\n{subtitle}" if subtitle else title
        fig.suptitle(heading, fontsize=13, fontweight="bold")
    return fig, axes


def save(fig: Any, outdir: Path, filename: str) -> Path:
    """Save and close ``fig`` as ``outdir/filename``."""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / filename
    fig.tight_layout()
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    return path


def short_label(name: str, max_len: int = 12) -> str:
    """Abbreviate a node/hop label for axis ticks.

    Generic CamelCase abbreviation (initials of each word) rather than the
    predecessor script's hardcoded name->short-name dictionary.
    """
    if "->" in name:
        return "→".join(short_label(p, max_len) for p in name.split("->"))
    if len(name) <= max_len:
        return name
    words = _split_camel(name)
    if len(words) >= 2:
        candidate = "".join(w[0] for w in words)
        if len(candidate) >= 2:
            # Keep the first word readable when it still fits.
            first = words[0]
            if len(first) + len(words) - 1 <= max_len:
                return first + "".join(w[0] for w in words[1:])
            return candidate
    return name[: max_len - 1] + "…"


def _split_camel(name: str) -> list[str]:
    words: list[str] = []
    current = ""
    for ch in name:
        if ch.isupper() and current and not current[-1].isupper():
            words.append(current)
            current = ch
        else:
            current += ch
    if current:
        words.append(current)
    return words


def annotate_empty(ax: Any, message: str = "no data") -> None:
    """Put a placeholder message on an otherwise empty axes."""
    ax.text(
        0.5,
        0.5,
        message,
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=11,
        color="#757575",
    )
    ax.set_xticks([])
    ax.set_yticks([])
