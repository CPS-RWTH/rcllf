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
"""Reader and per-epoch tables on the recorded golden (T6)."""

from __future__ import annotations

from lf_eval.semantic import schema
from lf_eval.semantic.model import build_epochs

from .golden_support import Golden


def test_golden_epoch_tables():
    g = Golden("corr_many")
    epochs = build_epochs(g.events)
    main = max(epochs.values(), key=lambda e: len(e.expectations))
    s = main.summary()
    assert s["has_run_begin"] and s["has_run_end"], s
    admitted = [k for k, v in main.publications.items() if v[0].fields["origin"] == 0]
    assert len(admitted) == 30, len(admitted)
    assert s["expectations"] == 60 and s["terminals"] == 60 and s["samples"] == 60, s
    assert len(main.backlogs) == 2, main.backlogs
    assert main.executable.endswith("CorrBringup"), main.executable
    assert {len(v) for v in main.seqs.values()} and all(
        sorted(v) == list(range(1, len(v) + 1)) for v in main.seqs.values()), "seq 1..n per category"


def test_golden_w3_dispatch_end_carries_cutoff_and_missing_hook():
    """Schema 1.2: the missing dispatch names its cutoff and hook.

    Recorded run: ordinal 2 of `one` dropped, both hooks registered. Expected
    from the fixture source: cutoff 200 + 1 + 100 ms after the start tag, the
    deadline hook first, every other corr dispatch NA.
    """
    g = Golden("corr_w3_missing")
    ep = next(iter(build_epochs(g.events).values()))
    assert int(ep.run_begin.fields["schema_minor"]) == 2
    t0 = int(ep.lf_start.fields["lf_start_time_ns"])
    corr = [d for d, b in ep.dispatch_begin.items()
            if int(b.fields["entry_kind"]) == schema.ENTRY_KINDS["CORR_SUB"]]
    rows = sorted((int(ep.dispatch_begin[d].fields["ordinal"]),
                   int(ep.dispatch_end[d].fields["outcome"]),
                   int(ep.dispatch_end[d].fields["cutoff"]) - t0,
                   int(ep.dispatch_end[d].fields["missing_hook"])) for d in corr)
    missing = [r for r in rows if r[1] != schema.OUTCOMES["MATCHED"]]
    assert missing == [(2, schema.OUTCOMES["TIMED_OUT"], 301_000_000,
                        schema.MISSING_HOOKS["DEADLINE"])], missing
    assert all(r[3] == schema.MISSING_HOOKS["NA"] for r in rows if r not in missing), rows


# lf_start and the dispatch of the dropped ordinal 2 on edge 0 (local /corr/a),
# copied from the corr_w3_missing run of the reproduction:
# test_runtime_semantic_trace.py with RCLLF_RUN_SEMANTIC_TRACE=1 in the
# ci-dev image, --shm-size 1g. The edge is local, so the sample is LOST and
# its dispatch_end.cutoff is the sentinel -1.
_W3_LOCAL_T0 = 1794953809541709634
_W3_LOCAL_BEGIN = {"dispatch_id": 6, "edge": 0, "entry_kind": 2, "ordinal": 2, "rid": 1,
                   "tag_microstep": 0, "tag_time": 1794953809741709634}
_W3_LOCAL_END = {"callback_ran": 1, "cutoff": -1, "dispatch_id": 6, "join_wait_ns": 65840,
                 "missing_hook": 3, "outcome": 4, "retained_id": 0}


def test_no_cutoff_sentinel_is_never_read_as_a_tag():
    """dispatch_end.cutoff -1 means none, not lf_start - 1 elapsed.

    The traced check computed cutoff - lf_start_time_ns and failed on a local
    edge with [(2, -1794949381135231006, 3)] == [(2, 300000000, 3)].
    """
    got = [(_W3_LOCAL_BEGIN["ordinal"], schema.cutoff_elapsed(_W3_LOCAL_END["cutoff"], _W3_LOCAL_T0),
            _W3_LOCAL_END["missing_hook"])]
    assert got == [(2, None, schema.MISSING_HOOKS["DEADLINE"])], got
    assert _W3_LOCAL_END["outcome"] == schema.OUTCOMES["LOST"]
    # A recorded cutoff stays a tag. lf_start and the TIMED_OUT dispatch_end
    # of the schema-1.2 corr_w3_missing golden (recorded 2026-10-11, before
    # local edges lost their cutoff): publish tag 200 ms + d 1 ms + T 100 ms.
    assert schema.cutoff_elapsed(1791679325306523092, 1791679325005523092) == 301_000_000
