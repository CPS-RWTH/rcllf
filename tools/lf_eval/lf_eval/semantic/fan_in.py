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
"""Fan-in order check.

A fan-in group is the admitted edges into one subscription callback, listed in
the static manifest by a shared ``group_lead``. Its one drain dispatches the
records of all members by the key ``(delivery tag, rank, ordinal)``, where the
delivery tag is LF's ``lf_delay_tag(publication tag, offset)``: ``(t + offset, 0)``
for an offset > 0 and the publication tag ``(t, m)`` itself for 0. The offset is
the group's delay, plus its cutoff for a deferred join (the binding's
``Table::arrive_locked``). In the order the drain ran them, their dispatch tags,
the keys never decrease. Like the rest of the semantic layer this joins by
identity only: a dispatch finds its publication tag by ``(gid, ordinal)``.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Any, Iterable, Mapping, Optional, Sequence

from lf_eval.semantic.model import Epoch, Key
from lf_eval.semantic.reader import SemEvent

OrderKey = tuple[int, int, int, int]


def fan_in_groups(rows: Iterable[dict[str, Any]]) -> dict[int, list[dict[str, Any]]]:
    """Admitted manifest rows of each fan-in group of two or more, by lead edge id."""
    out: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for r in rows:
        if r.get("status") == "admitted" and r.get("group_lead") is not None:
            out[int(r["group_lead"])].append(r)
    return dict(out)


def order_key(row: dict[str, Any], tag_time: int, tag_microstep: int, ordinal: int) -> OrderKey:
    """Key of the record *ordinal* published at (*tag_time*, *tag_microstep*) on *row*.

    The release tag is reactor-c's ``lf_delay_tag(tag, offset)``: (t, m + 1) for
    offset 0, else (t + offset, 0); then rank, ordinal.
    A ``same_tag`` row (in-process, d = 0) releases at (t, m) itself.
    """
    bounds = row.get("bounds") or {}
    offset = max(0, int(row["delay_ns"]))
    if bounds.get("join") == "deferred":
        offset += max(0, int(bounds.get("timeout_ns", 0)))
    micro = 0 if offset != 0 else tag_microstep + (0 if row.get("same_tag") else 1)
    return (tag_time + offset, micro, int(row["fan_in_rank"]),
            ordinal)


def _record(key: OrderKey, d: SemEvent) -> dict[str, Any]:
    return {"edge": int(d.fields["edge"]), "gid": d.gid(), "ordinal": int(d.fields["ordinal"]),
            "key": list(key), "dispatch_tag": [int(d.fields["tag_time"]), int(d.fields["tag_microstep"])]}


def fan_in_order(ep: Epoch, members: Sequence[dict[str, Any]],
                 publications: Optional[Mapping[Key, list[SemEvent]]] = None) -> dict[str, Any]:
    """Check one group's dispatches in *ep*; *publications* defaults to the epoch's own."""
    pubs = ep.publications if publications is None else publications
    rows = {int(r["id"]): r for r in members}
    seq = sorted((d for d in ep.dispatch_begin.values() if int(d.fields["edge"]) in rows),
                 key=lambda d: (int(d.fields["tag_time"]), int(d.fields["tag_microstep"]),
                                int(d.fields["dispatch_id"])))
    out: dict[str, Any] = {"members": sorted(rows), "dispatches": len(seq), "unkeyed": 0,
                           "violations": 0, "first_violation": None}
    prev: Optional[tuple[OrderKey, SemEvent]] = None
    for d in seq:
        ordinal = int(d.fields["ordinal"])
        p = pubs.get((d.gid(), ordinal))
        if not p:
            out["unkeyed"] += 1  # reconcile_edge reports it as missing_publication
            continue
        key = order_key(rows[int(d.fields["edge"])], int(p[0].fields["tag_time"]),
                        int(p[0].fields["tag_microstep"]), ordinal)
        if prev is not None and key < prev[0]:
            out["violations"] += 1
            if out["first_violation"] is None:
                out["first_violation"] = {"after": _record(*prev), "dispatched": _record(key, d)}
        prev = (key, d)
    return out
