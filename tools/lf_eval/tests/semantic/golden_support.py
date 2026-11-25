# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Load recorded goldens and apply named, scripted mutations (T6-T8).

Goldens are recorded from real traced runs by a maintainer recording script
(see golden/README.md); nothing here writes events by hand beyond the named
mutations below, each a small edit of a recorded run.
"""

from __future__ import annotations

import copy
import json
from collections import defaultdict
from pathlib import Path

import pytest

from lf_eval.semantic.claims import status_text
from lf_eval.semantic.reader import load_jsonl
from lf_eval.semantic.reconcile import analyze

from ..dev_only import golden_absent_reason

GOLDEN = Path(__file__).parent / "golden"


class Golden:
    def __init__(self, name: str):
        d = GOLDEN / name
        if not (d / "semantic_events.jsonl").is_file():
            pytest.skip(golden_absent_reason(d))
        self.events, self.stats = load_jsonl(d / "semantic_events.jsonl")
        self.run_manifest = json.loads((d / "trace_manifest.json").read_text())
        self.static = json.loads((d / "rcllf_trace_manifest.json").read_text())
        self.metadata: dict = {}

    def copy(self) -> "Golden":
        return copy.deepcopy(self)

    def analyze(self) -> dict:
        return analyze(self.events, self.stats, run_manifest=self.run_manifest,
                       static_manifest=self.static, metadata=self.metadata)

    def claim(self, edge: str = "0", kind: str = "correspondence") -> dict:
        for c in self.analyze()["claims"]:
            if c["kind"] == kind and c["subject"].split(" ")[1] == edge:
                return c
        raise AssertionError(f"no {kind} claim for edge {edge}")

    # --- named mutations -------------------------------------------------------
    def drop(self, pred, resequence: bool = True) -> "Golden":
        self.events = [e for e in self.events if not pred(e)]
        if resequence:
            self.resequence()
        return self

    def resequence(self) -> "Golden":
        """Renumber seq 1..n per (epoch, category) and make run_end totals agree."""
        n: dict = defaultdict(int)
        for e in self.events:
            if e.name in ("run_end", "run_end_edge", "run_end_publisher"):
                continue
            n[(e.epoch, e.cat)] += 1
            e.seq = n[(e.epoch, e.cat)]
        for e in self.events:
            if e.name == "run_end":
                for cat in range(len(e.fields["totals"])):
                    removed = e.fields["totals"][cat] - n.get((e.epoch, cat), 0)
                    e.fields["totals"][cat] -= removed
                    e.fields["attempts"][cat] -= removed
        return self

    def first(self, name: str):
        return next(e for e in self.events if e.name == name)


def text(claim: dict, part: str) -> str:
    return status_text(claim[part], claim[f"{part}_codes"])
