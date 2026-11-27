# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Drift test: docs/reference/diagnostic-codes.md covers exactly the registry codes.

Every code in the feature registry needs a ``{#rcllf-...}`` heading anchor on the
page, and every anchor on the page must name a code that exists. R004 is a run
refusal of the binding, not a registry code, so it is the one allowed extra.
"""

from __future__ import annotations

import re
from pathlib import Path

from colcon_lf.features.registry import REGISTRY

PAGE = Path(__file__).resolve().parents[4] / "docs" / "reference" / "diagnostic-codes.md"
NOT_IN_REGISTRY = {"RCLLF-R004"}


def _registry_codes() -> set[str]:
    return {code for module in REGISTRY.values() for code in module.codes}


def _page_codes() -> set[str]:
    anchors = re.findall(r"\{#(rcllf-[a-z0-9]+)\}", PAGE.read_text(encoding="utf-8"))
    return {a.upper() for a in anchors}


def test_every_registry_code_has_an_anchor():
    missing = sorted(_registry_codes() - _page_codes())
    assert not missing, f"codes without a {{#rcllf-...}} anchor in {PAGE.name}: {missing}"


def test_every_anchor_names_a_real_code():
    unknown = sorted(_page_codes() - _registry_codes() - NOT_IN_REGISTRY)
    assert not unknown, f"anchors in {PAGE.name} for codes that do not exist: {unknown}"
