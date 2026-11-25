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
"""Static trace manifest `rcllf_trace_manifest.json` (schema `rcllf.manifest/1`).

Generation facts the semantic analyzer needs and cannot read from a trace:
the executables, the provider marker the binding installed, the edge table
(admitted or legacy with the generator's reason verbatim), the RID table, the
resolved model hash and a hash over the generated sources. The executable
carries the manifest hash, the source hash and the model hash in the exported
`rcllf_trace_build_info` array, which the binding copies into `run_begin`.
"""

from __future__ import annotations

import hashlib
import json
import os
from collections import Counter
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

from colcon_lf.generation.lf_generator import local_now

MANIFEST_SCHEMA = "rcllf.manifest/1"
STATIC_MANIFEST_NAME = "rcllf_trace_manifest.json"
#: Federated generation of the same system writes here, so an lfc and a
#: federated build of one system never overwrite each other's manifest.
FEDERATED_MANIFEST_NAME = "rcllf_trace_manifest_federated.json"
BUILD_INFO_CPP = "rcllf_trace_build_info.cpp"


def canonical_hash(doc: Any) -> str:
    """sha256 of the canonical JSON form (sorted keys, no whitespace)."""
    text = json.dumps(doc, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def generated_source_hash(pkg_dir: Path, subdirs: Sequence[str] = ("src", "lf")) -> str:
    """sha256 over (relative path, content) of the generated sources, sorted.

    The build-info source itself is excluded: it embeds this hash.
    """
    h = hashlib.sha256()
    pkg_dir = Path(pkg_dir)
    files: list[Path] = []
    for sub in subdirs:
        root = pkg_dir / sub
        if root.is_dir():
            files.extend(p for p in root.rglob("*") if p.is_file() and p.name != BUILD_INFO_CPP)
    for p in sorted(files, key=lambda q: q.relative_to(pkg_dir).as_posix()):
        h.update(p.relative_to(pkg_dir).as_posix().encode("utf-8") + b"\0")
        h.update(p.read_bytes() + b"\0")
    return h.hexdigest()


def find_provider_marker(prefixes: Optional[Iterable[str]] = None) -> Optional[str]:
    """Contents of `share/ros2_lf_binding/trace_provider` on the ament prefix path, or None."""
    if prefixes is None:
        prefixes = [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    for prefix in prefixes:
        marker = Path(prefix) / "share" / "ros2_lf_binding" / "trace_provider"
        if marker.is_file():
            return marker.read_text(encoding="utf-8").strip()
    return None


def _edge_row(e: Any, grouped: bool = False) -> dict[str, Any]:
    row = {
        "topic": e.topic,
        # Lists only: a reader expecting one `rid` fails loudly.
        "publisher": {"node": e.pub_node, "key": e.pub_key, "callbacks": list(e.pub_callbacks),
                      "rids": list(e.pub_rids)},
        "subscriber": {"node": e.sub_node, "key": e.sub_key, "callback": e.sub_callback,
                       "rid": e.sub_rid},
        "delay_ns": e.delay_ns,
        "status": e.status,
    }
    if e.status == "admitted":
        row["id"] = e.edge_id
        if grouped:
            # A member of a fan-in group of two or more. The lead
            # names the group's one drain, the rank breaks ties in its order key.
            row["group_lead"] = e.group_lead
            row["fan_in_rank"] = e.fan_in_rank
        if getattr(e, "same_tag", False):
            # The first record of a tag runs at its publication tag.
            row["same_tag"] = True
        # Every bound with the inputs it was computed from, so a
        # reviewer can recheck each number from the manifest alone.
        row["bounds"] = {
            "burst": e.burst, "retain": e.retain, "carrier": e.carrier,
            "reader_depth": e.reader_depth,
            "timeout_ns": e.timeout_ns, "max_lag_ns": e.max_lag_ns,
            "join": "deferred" if e.deferred else "blocking",
            "inputs": dict(e.bound_inputs),
        }
        if local_now(e):
            # Decided when its publish returns, so no join waits
            # and nothing counts from a cutoff. timeout_ns only sizes L. The
            # same keys as the edge's reconcile row.
            row["bounds"]["local_now"] = True
            row["bounds"]["cutoff"] = "none"
    else:
        row["reason"] = e.reason or ""  # the generator's reason string, verbatim (P8)
    if getattr(e, "federate", None) is not None:
        # Where the row lives in the federation.
        row["role"] = e.role
        row["global_id"] = e.global_id
        row["peer_federate"] = e.peer_federate
    return row


def build_static_manifest(
    *,
    executables: Sequence[str],
    corr_edges: Sequence[Any],
    rid_table: Sequence[Sequence[Any]],
    model_hash: Optional[str],
    generated_source_hash: str,
    provider_marker: Optional[str],
) -> dict[str, Any]:
    """Assemble the static manifest. Deterministic: no timestamps, no absolute paths."""
    # The binding header's rule: a group of one registers no fan-in, so its row
    # keeps the single-subscriber shape.
    group_size = Counter(
        e.group_lead for e in corr_edges
        if e.edge_id is not None and getattr(e, "group_lead", None) is not None
    )
    return {
        "schema": MANIFEST_SCHEMA,
        "kind": "static",
        "executables": sorted(executables),
        "provider_marker": provider_marker,
        "edges": [_edge_row(e, group_size[getattr(e, "group_lead", None)] > 1)
                  for e in corr_edges],
        "rids": [
            {"rid": rid, "node": node, "callback": cb, "kind": kind, "topic": topic}
            for rid, node, cb, kind, topic, *_ in rid_table
        ],
        "model_hash": model_hash,
        "generated_source_hash": generated_source_hash,
    }


def write_static_manifest(doc: dict[str, Any], path: Path) -> str:
    """Write *doc* canonically and return the sha256 of the written bytes."""
    data = (json.dumps(doc, indent=2, sort_keys=True) + "\n").encode("utf-8")
    Path(path).write_bytes(data)
    return hashlib.sha256(data).hexdigest()


def emit_build_info_cpp(manifest_hash: str, source_hash: str, model_hash: Optional[str]) -> str:
    """C++ source defining the exported `rcllf_trace_build_info` array and the bounds
    of the reactor-c patch marker section (`rcllf_reactor_c_patch_markers`)."""
    return (
        "// Auto-generated by colcon lf-generate. DO NOT EDIT.\n"
        "// Copyright 2026 rcllf Contributors\n"
        "// SPDX-License-Identifier: Apache-2.0\n"
        "// [static manifest sha256, generated source sha256, resolved model hash]\n"
        'extern "C" __attribute__((visibility("default"), used))\n'
        "const char * const rcllf_trace_build_info[3] = {\n"
        f'  "{manifest_hash}",\n'
        f'  "{source_hash}",\n'
        f'  "{model_hash or ""}",\n'
        "};\n"
        "// Bounds of the reactor-c patch markers compiled into this\n"
        "// program (third_party/patches/reactor-c), read back by the binding.\n"
        'extern "C" __attribute__((weak)) const char __start_rcllf_reactor_c_patches[];\n'
        'extern "C" __attribute__((weak)) const char __stop_rcllf_reactor_c_patches[];\n'
        'extern "C" __attribute__((visibility("default"), used))\n'
        "const char * const rcllf_reactor_c_patch_markers[2] = {\n"
        "  __start_rcllf_reactor_c_patches,\n"
        "  __stop_rcllf_reactor_c_patches,\n"
        "};\n"
    )
