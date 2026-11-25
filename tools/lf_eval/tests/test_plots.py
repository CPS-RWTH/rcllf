# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Tests for shared plot styling."""

from __future__ import annotations

import matplotlib

from lf_eval.plots import style


def test_agg_backend_is_selected():
    """Plots must render headless — no display required."""
    assert matplotlib.get_backend().lower() == "agg"


def test_mode_colors_are_distinct():
    colors = {style.mode_color(m) for m in ("lfc", "lf", "stock")}
    assert len(colors) == 3


def test_unknown_mode_gets_fallback_color():
    """Unrecognized modes get a *deterministic* fallback, not the shared grey."""
    assert style.mode_color("weird") != style.MODE_COLORS["unknown"]
    assert style.mode_color("weird") == style.mode_color("weird")


def test_empty_and_literal_unknown_mode_share_the_grey():
    assert style.mode_color("") == style.MODE_COLORS["unknown"]
    assert style.mode_color("unknown") == style.MODE_COLORS["unknown"]


def test_distinct_unknown_modes_get_distinct_colors():
    assert style.mode_color("rclcpp-events-v2") != style.mode_color("custom-executor")


def test_five_phase6_modes_are_distinct():
    modes = ("stock-single", "stock-multi", "stock-events", "lfc", "rslcpp")
    colors = {style.mode_color(m) for m in modes}
    assert len(colors) == 5


def test_mode_color_never_splits_on_hyphen():
    """A mode string is one opaque token — never mangled into a substring."""
    assert style.mode_color("stock-single") == style.MODE_COLORS["stock-single"]
    assert style.mode_color("stock-single") != style.mode_color("stock")
    assert style.mode_color("stock-single") != style.mode_color("single")


def test_mode_label_falls_back_to_the_name():
    assert style.mode_label("weird") == "weird"
    assert "lfc" in style.mode_label("lfc")


def test_present_modes_orders_lfc_before_stock():
    assert style.present_modes({"stock", "lfc"}) == ["lfc", "stock"]


def test_present_modes_appends_unknown_modes_sorted():
    assert style.present_modes({"zeta", "alpha", "lfc"}) == ["lfc", "alpha", "zeta"]


def test_present_modes_empty():
    assert style.present_modes(set()) == []


def test_short_label_keeps_short_names():
    assert style.short_label("NodeA") == "NodeA"


def test_short_label_abbreviates_camel_case():
    label = style.short_label("PointsTransformerFront")
    assert len(label) < len("PointsTransformerFront")


def test_short_label_handles_hop_arrows():
    label = style.short_label("FrontLidarDriver->PointsTransformerFront")
    assert "→" in label


def test_short_label_truncates_unsplittable_names():
    label = style.short_label("aaaaaaaaaaaaaaaaaaaa", max_len=8)
    assert len(label) <= 8


def test_figure_and_save_roundtrip(tmp_path):
    fig, ax = style.figure("Title", "subtitle")
    ax.plot([0, 1], [0, 1])
    path = style.save(fig, tmp_path / "nested", "out.png")
    assert path.is_file()
    assert path.stat().st_size > 0
    assert path.parent.name == "nested"


def test_figure_with_two_columns():
    fig, axes = style.figure("T", ncols=2)
    assert len(axes) == 2


def test_annotate_empty_clears_ticks():
    fig, ax = style.figure()
    style.annotate_empty(ax, "nothing here")
    assert ax.get_xticks().size == 0
