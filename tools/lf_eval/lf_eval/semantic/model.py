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
"""Per-epoch tables of the semantic trace (D2).

An epoch is one process run (`run_begin` names executable and pid). Tables
are keyed by identity, never by timestamp: publications by (gid, ordinal),
expectations and terminals by (edge, gid, ordinal), samples by retained id,
dispatches by dispatch_id.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any, Iterable, Optional

from lf_eval.semantic import schema
from lf_eval.semantic.reader import SemEvent

Key = tuple[str, int]            # (gid hex, ordinal)
EdgeKey = tuple[int, str, int]   # (edge, gid hex, ordinal)


@dataclass
class Epoch:
    epoch: int
    run_begin: Optional[SemEvent] = None
    run_end: Optional[SemEvent] = None
    lf_start: Optional[SemEvent] = None  # schema 1.1, absent in older traces
    providers: list[SemEvent] = field(default_factory=list)
    backlogs: dict[str, SemEvent] = field(default_factory=dict)            # gid -> backlog
    publications: dict[Key, list[SemEvent]] = field(default_factory=lambda: defaultdict(list))
    expectations: dict[EdgeKey, list[SemEvent]] = field(default_factory=lambda: defaultdict(list))
    terminals: dict[EdgeKey, list[SemEvent]] = field(default_factory=lambda: defaultdict(list))
    samples: list[SemEvent] = field(default_factory=list)
    carriers: list[SemEvent] = field(default_factory=list)
    external_arrivals: list[SemEvent] = field(default_factory=list)
    dispatch_begin: dict[int, SemEvent] = field(default_factory=dict)
    dispatch_end: dict[int, SemEvent] = field(default_factory=dict)
    cleanups: list[SemEvent] = field(default_factory=list)
    end_publishers: dict[str, SemEvent] = field(default_factory=dict)      # gid -> tracked one wins
    end_publisher_records: int = 0  # raw count, several untracked slots may share a gid
    duplicate_tracked_gids: int = 0
    end_edges: dict[int, SemEvent] = field(default_factory=dict)
    seqs: dict[int, list[int]] = field(default_factory=lambda: defaultdict(list))
    duplicate_dispatch_ids: int = 0

    @property
    def executable(self) -> str:
        return str(self.run_begin.fields.get("executable", "")) if self.run_begin else ""

    @property
    def boot_id(self) -> str:
        return str(self.run_begin.fields.get("boot_id", "")) if self.run_begin else ""

    def samples_by_retained(self) -> dict[int, SemEvent]:
        return {int(s.fields["retained_id"]): s for s in self.samples if int(s.fields["retained_id"]) > 0}

    def summary(self) -> dict[str, Any]:
        """Identity-only counts, stable across timestamp permutations."""
        return {
            "epoch": self.epoch,
            "executable": self.executable,
            "publications": sum(len(v) for v in self.publications.values()),
            "expectations": sum(len(v) for v in self.expectations.values()),
            "terminals": sum(len(v) for v in self.terminals.values()),
            "samples": len(self.samples),
            "dispatches": len(self.dispatch_begin),
            "has_run_begin": self.run_begin is not None,
            "has_run_end": self.run_end is not None,
        }


def ekey(e: SemEvent) -> EdgeKey:
    return (int(e.fields["edge"]), e.gid(), int(e.fields["ordinal"]))


def build_epochs(events: Iterable[SemEvent]) -> dict[int, Epoch]:
    """Group rcllf events by epoch into identity-keyed tables."""
    out: dict[int, Epoch] = {}
    for e in events:
        ep = out.setdefault(e.epoch, Epoch(e.epoch))
        if e.name not in schema.CLOSE_GROUP:
            ep.seqs[e.cat].append(e.seq)
        n = e.name
        if n == "run_begin":
            ep.run_begin = e
        elif n == "run_end":
            ep.run_end = e
        elif n == "lf_start":
            ep.lf_start = e
        elif n == "provider":
            ep.providers.append(e)
        elif n == "backlog":
            ep.backlogs[e.gid()] = e
        elif n == "publication":
            ep.publications[(e.gid(), int(e.fields["ordinal"]))].append(e)
        elif n == "expectation":
            ep.expectations[ekey(e)].append(e)
        elif n == "terminal":
            ep.terminals[ekey(e)].append(e)
        elif n == "sample":
            ep.samples.append(e)
        elif n == "carrier":
            ep.carriers.append(e)
        elif n == "external_arrival":
            ep.external_arrivals.append(e)
        elif n == "dispatch_begin":
            did = int(e.fields["dispatch_id"])
            ep.duplicate_dispatch_ids += did in ep.dispatch_begin
            ep.dispatch_begin[did] = e
        elif n == "dispatch_end":
            ep.dispatch_end[int(e.fields["dispatch_id"])] = e
        elif n == "cleanup":
            ep.cleanups.append(e)
        elif n == "run_end_publisher":
            ep.end_publisher_records += 1
            prev = ep.end_publishers.get(e.gid())
            tracked = int(e.fields.get("tracked", 0))
            if prev is not None and tracked and int(prev.fields.get("tracked", 0)):
                ep.duplicate_tracked_gids += 1
            if prev is None or tracked:
                ep.end_publishers[e.gid()] = e
        elif n == "run_end_edge":
            ep.end_edges[int(e.fields["edge"])] = e
    return out
