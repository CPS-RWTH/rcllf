# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""Federate names stay short enough for lfc's C name truncation.

lfc keeps the last 39 characters of a generated C instance name. Inside
federate ``F`` the network receiver ``nr_federate__F`` then gets the same name
as the federate's main reactor once ``F`` has 31 characters or more, lfc
renames one of them with ``_2``, and its own federate preamble still names
``_<tail>_self``, so the C compile fails. Measured with lfc 0.12.1 on a two
federate program: 30 characters compile, 31, 32, 33 and 35 do not.
"""

from __future__ import annotations

import textwrap

import pytest

from colcon_lf.comm_analysis.models import SystemCommunication
from colcon_lf.federation.partition import _per_node, _per_process, resolve_partition
from colcon_lf.graph.models import SystemGraph
from colcon_lf.launch_analysis.models import LaunchFileInfo, LaunchGraph, LaunchNodeAction
from colcon_lf.node_analysis.models import NodeInfo

# The longest federate name lfc 0.12.1 compiles (see the module docstring).
LFC_SAFE = 30

PUB = "publisher_node_with_a_name_of_forty_or_more_chars"
SUB = "subscriber_node_with_a_name_of_forty_or_more_chars"


def _graph(names: list[str], launch_nodes: list | None = None) -> SystemGraph:
    nodes = {
        n: NodeInfo(name=n, class_name="Node", package="test_pkg", source_file=f"/fake/{n}.cpp")
        for n in names
    }
    root = LaunchFileInfo(file_path="/fake/bringup.launch.py", package="test_pkg")
    return SystemGraph(
        name="test_system",
        launch_graph=LaunchGraph(root=root, all_nodes=launch_nodes or []),
        nodes=nodes,
        communication=SystemCommunication(),
    )


def _assert_lfc_safe(names: list[str]) -> None:
    for name in names:
        assert len(name) <= LFC_SAFE, (
            f"federate name '{name}' has {len(name)} characters, lfc's 39-character "
            f"truncation then names its network receiver like the main reactor and "
            f"the federate preamble's '_..._self' is undeclared"
        )


@pytest.mark.parametrize("mode", [_per_node, _per_process])
def test_long_node_names_give_short_distinct_federates(mode):
    fmap = mode(_graph([PUB, SUB]))
    _assert_lfc_safe(fmap.federate_names)
    assert len(set(fmap.federate_names)) == 2
    assert fmap.federate_for(PUB) != fmap.federate_for(SUB)
    # The readable prefix of the node name stays.
    assert fmap.federate_for(PUB).startswith("publisher_node")


def test_shortening_is_deterministic():
    a = _per_node(_graph([PUB, SUB])).federate_names
    b = _per_node(_graph([PUB, SUB])).federate_names
    assert a == b


def test_names_up_to_the_limit_are_kept():
    name = "n" * LFC_SAFE
    assert _per_node(_graph([name, "b"])).federate_names == [name, "b"]


def test_same_prefix_names_stay_distinct():
    # Same first 40 characters, so a plain cut would merge them.
    a = "x" * 40 + "_left"
    b = "x" * 40 + "_right"
    fmap = _per_node(_graph([a, b]))
    _assert_lfc_safe(fmap.federate_names)
    assert fmap.federate_for(a) != fmap.federate_for(b)


def test_long_container_federate_is_shortened():
    container = "a_composable_container_with_a_name_over_forty_chars"
    launch = [
        LaunchNodeAction(package="test_pkg", executable="test_pkg::Node", name=n,
                         is_component=True, composable_container=container)
        for n in ("talker", "listener")
    ]
    fmap = _per_process(_graph(["talker", "listener"], launch))
    _assert_lfc_safe(fmap.federate_names)
    assert fmap.federate_for("talker") == fmap.federate_for("listener")


def test_yaml_federate_name_over_the_limit_is_refused(tmp_path):
    long_fed = "a_user_chosen_federate_name_of_40_chars_"
    yaml_file = tmp_path / "fed.yaml"
    yaml_file.write_text(textwrap.dedent(f"""\
        federates:
          {long_fed}:
            nodes: [{PUB}]
          short:
            nodes: [{SUB}]
    """))
    with pytest.raises(ValueError, match=rf"'{long_fed}' has 40 characters.*at most {LFC_SAFE}"):
        resolve_partition(str(yaml_file), _graph([PUB, SUB]))


def test_yaml_federate_name_at_the_limit_is_kept(tmp_path):
    fed = "f" * LFC_SAFE
    yaml_file = tmp_path / "fed.yaml"
    yaml_file.write_text(textwrap.dedent(f"""\
        federates:
          {fed}:
            nodes: [{PUB}]
          short:
            nodes: [{SUB}]
    """))
    assert resolve_partition(str(yaml_file), _graph([PUB, SUB])).federate_names == [fed, "short"]
