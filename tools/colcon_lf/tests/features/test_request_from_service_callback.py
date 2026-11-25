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
"""RCLLF-H012 — request sent from a service handler or a response callback.

To falsify: make `_handlers()` return an empty dict and
`test_fires_for_request_from_service_handler` fails with `assert [] == [...]`.
"""

from __future__ import annotations

from pathlib import Path

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.features.diagnostics import DetectionContext, Severity
from colcon_lf.features.modules.request_from_service_callback import CODE_H012, detect
from colcon_lf.features.registry import severity_for
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph
from colcon_lf.node_analysis.models import (
    CallbackInfo,
    NodeInfo,
    ServiceClientInfo,
    ServiceServerInfo,
    SubscriberInfo,
)


def _graph(node: NodeInfo) -> SystemGraph:
    root = LaunchFileInfo(file_path=Path("/fake/bringup.launch.py"), package="test_pkg")
    return SystemGraph(
        name="s",
        launch_graph=LaunchGraph(root=root, all_nodes=[]),
        nodes={"n": node},
        communication=SystemCommunication(),
    )


def _node(callbacks, *, servers=(), subscribers=()) -> NodeInfo:
    return NodeInfo(
        name="n",
        class_name="N",
        package="test_pkg",
        source_file=Path("/fake/n.cpp"),
        callbacks=list(callbacks),
        subscribers=list(subscribers),
        service_servers=list(servers),
        service_clients=[ServiceClientInfo(
            service_name="/downstream",
            service_type="std_srvs::srv::Trigger",
            call_style="async_callback",
            response_callback="on_response",
        )],
    )


def test_fires_for_request_from_service_handler():
    node = _node(
        [CallbackInfo(name="handle", service_clients_used=["/downstream"])],
        servers=[ServiceServerInfo(
            service_name="/upstream", service_type="std_srvs::srv::Trigger", callback="handle",
        )],
    )
    diags = detect(DetectionContext(graph=_graph(node)))
    assert [(d.code, d.topic) for d in diags] == [(CODE_H012, "/downstream")]
    assert "handler of service '/upstream'" in diags[0].message


def test_fires_for_request_from_response_callback():
    node = _node([CallbackInfo(name="on_response", service_clients_used=["/downstream"])])
    diags = detect(DetectionContext(graph=_graph(node)))
    assert [d.code for d in diags] == [CODE_H012]


def test_quiet_for_request_from_subscription_callback():
    node = _node(
        [CallbackInfo(name="on_range", service_clients_used=["/downstream"])],
        subscribers=[SubscriberInfo(
            topic="/range", message_type="std_msgs::msg::Float64", callback="on_range",
        )],
    )
    assert detect(DetectionContext(graph=_graph(node))) == []


def test_is_an_error():
    assert severity_for(CODE_H012) is Severity.ERROR
