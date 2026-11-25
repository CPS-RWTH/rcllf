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
"""Map communication endpoints to node keys.

The analyzer lists each endpoint by its launch node key, one per node instance.
A system.json from an older analyzer lists it by ``NodeInfo.name``, which
every instance of one class shares: such a file is still read when each listed
name belongs to one node, and refused when it could mean several.
"""

from __future__ import annotations

from colcon_lf.comm_analysis.models import NODE_KEY, SystemCommunication


class AmbiguousEndpointError(ValueError):
    """An older system.json (written before endpoints carried the node instance key) names an endpoint that several node instances share."""


def _endpoint_names(comm: SystemCommunication) -> set[str]:
    names: set[str] = set()
    for tc in comm.topics:
        names.update(tc.publishers)
        names.update(tc.subscribers)
    for conn in (*comm.services, *comm.actions):
        names.add(conn.server)
        names.update(conn.clients)
    names.discard("")
    return names


def endpoint_key_map(graph) -> dict[str, str]:
    """``{endpoint id: node key}`` for *graph*'s communication endpoints.

    *graph* is anything with ``nodes`` (keyed by node key) and
    ``communication``. Raises :class:`AmbiguousEndpointError` for an older
    graph whose endpoint name belongs to two or more node instances, since
    the file does not say which instance sits on which end.
    """
    if getattr(graph.communication, "endpoint_ids", None) == NODE_KEY:
        return {key: key for key in graph.nodes}
    by_name: dict[str, list[str]] = {}
    for key, ni in graph.nodes.items():
        by_name.setdefault(ni.name, []).append(key)
    used = _endpoint_names(graph.communication)
    ambiguous = {name: keys for name, keys in by_name.items() if len(keys) > 1 and name in used}
    if ambiguous:
        detail = "; ".join(
            f"{name!r} is node {', '.join(repr(k) for k in keys)}"
            for name, keys in sorted(ambiguous.items())
        )
        raise AmbiguousEndpointError(
            "system.json lists topic, service and action endpoints by node name "
            f"(written by an older colcon lf-analyze), and {detail}: the file does not say "
            "which instance is on which end of a connection. Re-run "
            "'colcon lf-analyze' to list endpoints per node instance."
        )
    return {name: keys[0] for name, keys in by_name.items()}
