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
"""The Python schema mirror agrees with rcllf_trace_schema.h (T1)."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from lf_eval.semantic import schema

REPO = Path(__file__).resolve().parents[4]
BINDING = REPO / "packages" / "lf-frame" / "ros2_lf_binding"
HEADER = BINDING / "include" / "ros2_lf_binding" / "rcllf_trace_schema.h"
CORR_HPP = BINDING / "src" / "correspondence.hpp"
GATE_API = BINDING / "include" / "ros2_lf_binding" / "publication_gate_api.h"


def _defines(text: str) -> dict[str, str]:
    return dict(re.findall(r"^#define\s+(RCLLF_TRACE_\w+)\s+(\S+)", text, re.M))


def _c_enum(text: str, name: str) -> dict[str, int]:
    m = re.search(r"enum\s+" + name + r"\s*\{(.*?)\};", text, re.S)
    assert m, f"enum {name} not found"
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    return {k: int(v, 0) for k, v in re.findall(r"(\w+)\s*=\s*(\w+)", body)}


def _cpp_enum_order(text: str, name: str) -> list[str]:
    m = re.search(r"enum class\s+" + name + r"\s*:\s*\w+\s*\{(.*?)\};", text, re.S)
    assert m, f"enum class {name} not found"
    body = re.sub(r"///?<?.*", "", m.group(1))
    return [t.strip() for t in body.split(",") if t.strip()]


def _macro_body(text: str, name: str) -> str:
    m = re.search(r"#define\s+" + name + r"\((\w)\)\s*\\\n((?:.*\\\n)*.*\n)", text)
    assert m, f"macro {name} not found"
    return m.group(2)


def parse_header(text: str) -> dict:
    events = {}
    for ev, cat in re.findall(r"X\((\w+),\s*RCLLF_CAT_(\w+)\)", _macro_body(text, "RCLLF_TRACE_EVENTS")):
        fields = re.findall(r"F\((\w+),\s*[^,]+,\s*(\w+)\)", _macro_body(text, f"RCLLF_FIELDS_{ev}"))
        events[ev] = (cat, tuple(fields))
    return {"defines": _defines(text), "events": events}


@pytest.fixture(scope="module")
def header() -> dict:
    return parse_header(HEADER.read_text())


def test_versions_match(header):
    d = header["defines"]
    assert d["RCLLF_TRACE_SCHEMA_NAME"].strip('"') == schema.SCHEMA_NAME
    assert int(d["RCLLF_TRACE_SCHEMA_MAJOR"]) == schema.SCHEMA_MAJOR, "schema major drifted"
    assert int(d["RCLLF_TRACE_SCHEMA_MINOR"]) == schema.SCHEMA_MINOR, "schema minor drifted"
    assert int(d["RCLLF_TRACE_PROVIDER_VERSION"]) == schema.PROVIDER_VERSION
    assert int(d["RCLLF_TRACE_GID_LEN"]) == schema.GID_LEN
    assert int(d["RCLLF_TRACE_NO_EDGE"].rstrip("uU"), 0) == schema.NO_EDGE
    assert int(d["RCLLF_TRACE_NO_RID"].strip("()")) == schema.NO_RID


@pytest.mark.parametrize("enum_name", sorted(schema.ENUMS))
def test_enum_codes_match(enum_name):
    prefix, mirror = schema.ENUMS[enum_name]
    c = {k[len(prefix):]: v for k, v in _c_enum(HEADER.read_text(), enum_name).items() if k != f"{prefix}COUNT"}
    for code in sorted(set(c) | set(mirror)):
        assert c.get(code) == mirror.get(code), (
            f"{enum_name}.{code}: header {c.get(code)} != mirror {mirror.get(code)}"
        )
    if enum_name == "rcllf_trace_cat":
        assert _c_enum(HEADER.read_text(), enum_name)["RCLLF_CAT_COUNT"] == schema.CAT_COUNT


def test_events_and_fields_match(header):
    c_events = header["events"]
    assert list(c_events) == list(schema.EVENTS), (
        f"event list drifted: header {list(c_events)} mirror {list(schema.EVENTS)}"
    )
    for ev, (cat, fields) in c_events.items():
        mcat, mfields = schema.EVENTS[ev]
        assert cat == mcat, f"{ev}.category: header {cat} != mirror {mcat}"
        for i, (cf, mf) in enumerate(zip(fields, mfields)):
            assert cf == mf, f"{ev}.field[{i}]: header {cf} != mirror {mf}"
        assert len(fields) == len(mfields), f"{ev}: header {len(fields)} fields, mirror {len(mfields)}"


@pytest.mark.parametrize(
    ("cpp_enum", "wire"),
    [("Outcome", schema.OUTCOMES), ("SampleOutcome", schema.SAMPLE_OUTCOMES), ("Origin", schema.ORIGINS)],
)
def test_wire_codes_follow_correspondence_enums(cpp_enum, wire):
    """The wire codes are the enum ordinals (A1): a reorder there is drift here."""
    order = _cpp_enum_order(CORR_HPP.read_text(), cpp_enum)
    for i, name in enumerate(order):
        assert wire.get(name) == i, f"corr::{cpp_enum}::{name} is {i}, wire code {wire.get(name)}"


def test_api_codes_follow_gate_api():
    gate = _c_enum(GATE_API.read_text(), "rcllf_gate_api")
    for k, v in gate.items():
        assert schema.APIS[k[len("RCLLF_GATE_"):]] == v, f"api {k} drifted"


def test_parity_reports_the_drifting_field():
    text = HEADER.read_text().replace("F(int, uint64_t, ordinal) \\\n  F(int, uint8_t, result)",
                                      "F(int, uint64_t, ordinal) \\\n  F(int, uint8_t, reslt)")
    events = parse_header(text)["events"]
    got = events["publication"][1][2]
    assert got == ("int", "reslt"), "mutation did not apply"
    assert got != schema.EVENTS["publication"][1][2]
