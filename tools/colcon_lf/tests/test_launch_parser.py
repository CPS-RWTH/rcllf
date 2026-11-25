# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Tests for colcon_lf.launch_analysis.parser.

Phase 2 — TDD assertions (7 required by plan). All tests require a built
and sourced ROS2 workspace (run inside Docker).

Mark with ``pytest.mark.skipif`` if launch/launch_ros are not importable so
the test file can be collected without Docker (tests simply skip).
"""

from __future__ import annotations

import pytest

# Skip entire module if launch_ros is not installed (outside Docker)
launch_ros = pytest.importorskip("launch_ros", reason="launch_ros not available — run inside Docker")

from .ros_env import requires_ros_headers  # noqa: E402


# ---------------------------------------------------------------------------
# Assertion 1: timer_example — single Node
# ---------------------------------------------------------------------------

class TestTimerExampleLaunch:
    def test_single_node(self, launch_parser):
        graph = launch_parser("timer_example", "timer_node.launch.py")
        assert len(graph.root.nodes) == 1
        node = graph.root.nodes[0]
        assert node.package == "timer_example"
        assert node.executable == "timer_node"

    def test_all_nodes_count(self, launch_parser):
        graph = launch_parser("timer_example", "timer_node.launch.py")
        assert len(graph.all_nodes) == 1

    def test_no_includes(self, launch_parser):
        graph = launch_parser("timer_example", "timer_node.launch.py")
        assert graph.root.includes == []

    def test_no_arguments(self, launch_parser):
        graph = launch_parser("timer_example", "timer_node.launch.py")
        assert graph.root.arguments == []


# ---------------------------------------------------------------------------
# Assertion 2 + 3: pubsub_example — bringup includes two launch files
# ---------------------------------------------------------------------------

class TestPubsubBringup:
    def test_two_includes(self, launch_parser):
        graph = launch_parser("pubsub_example", "bringup.launch.py")
        assert len(graph.root.includes) == 2

    def test_all_nodes_has_two(self, launch_parser):
        graph = launch_parser("pubsub_example", "bringup.launch.py")
        assert len(graph.all_nodes) == 2

    def test_publisher_node_present(self, launch_parser):
        graph = launch_parser("pubsub_example", "bringup.launch.py")
        executables = [n.executable for n in graph.all_nodes]
        assert "publisher_node" in executables

    def test_subscriber_node_present(self, launch_parser):
        graph = launch_parser("pubsub_example", "bringup.launch.py")
        executables = [n.executable for n in graph.all_nodes]
        assert "subscriber_node" in executables


class TestPubsubPublisherLaunch:
    def test_single_node(self, launch_parser):
        """Assertion 3: publisher_node.launch.py has exactly 1 node."""
        graph = launch_parser("pubsub_example", "publisher_node.launch.py")
        assert len(graph.root.nodes) == 1
        assert graph.root.nodes[0].executable == "publisher_node"


# ---------------------------------------------------------------------------
# Assertion 4: component_example — ComposableNodeContainer
# ---------------------------------------------------------------------------

class TestComponentLaunch:
    def test_composable_node_present(self, launch_parser):
        graph = launch_parser("component_example", "component_node.launch.py")
        components = [n for n in graph.all_nodes if n.is_component]
        assert len(components) == 1

    def test_plugin_name(self, launch_parser):
        graph = launch_parser("component_example", "component_node.launch.py")
        components = [n for n in graph.all_nodes if n.is_component]
        assert components[0].executable == "component_example::ComponentNode"

    def test_container_name(self, launch_parser):
        graph = launch_parser("component_example", "component_node.launch.py")
        components = [n for n in graph.all_nodes if n.is_component]
        assert components[0].composable_container is not None


# ---------------------------------------------------------------------------
# Assertion 5: parameter_example — YAML config file path
# ---------------------------------------------------------------------------

class TestParameterLaunch:
    def test_parameter_files_resolved(self, launch_parser):
        graph = launch_parser("parameter_example", "parameter_node.launch.py")
        assert len(graph.all_nodes) == 1
        node = graph.all_nodes[0]
        assert len(node.parameter_files) >= 1

    def test_params_yaml_path(self, launch_parser):
        graph = launch_parser("parameter_example", "parameter_node.launch.py")
        node = graph.all_nodes[0]
        paths_str = [str(p) for p in node.parameter_files]
        assert any("config/params.yaml" in p or "params.yaml" in p for p in paths_str)


class TestLaunchArgumentParameter:
    """A ``LaunchConfiguration`` parameter value is bound and YAML-typed.

    ``launch_ros`` hands ``choices_node`` the integer 500, so the generated
    system must too: the string ``'$(var timer_period_ms)'`` (or ``'500'``)
    makes ``declare_parameter<int64_t>`` throw InvalidParameterTypeException.
    """

    @staticmethod
    def _choices_node(graph):
        return next(n for n in graph.all_nodes if n.name == "choices_node")

    def test_argument_default_bound_as_int(self, launch_parser):
        node = self._choices_node(
            launch_parser("launch_features_example", "02_arg_with_choices.launch.py"))
        assert node.parameters["timer_period_ms"] == 500
        # Named but not namespaced: launch_ros keys the inline dict '/**'.
        assert node.parameter_sources == [{"params": {"timer_period_ms": 500}, "key": "/**"}]

    def test_included_argument_bound_as_int(self, launch_parser):
        # all_features forwards its own timer_period_ms argument (default '500').
        node = self._choices_node(
            launch_parser("launch_features_example", "all_features.launch.py"))
        assert node.parameters["timer_period_ms"] == 500

    @staticmethod
    def _parse(tmp_path, body, child=None, graphs=None):
        """Parse a launch file whose ``generate_launch_description`` returns *body*.

        With *child*, ``include(**args)`` in *body* includes a second file
        whose description returns *child*, passing *args* as launch arguments.
        The parsed graph is appended to *graphs* when given.
        """
        launch_dir = tmp_path / "src" / "test_pkg" / "launch"
        launch_dir.mkdir(parents=True)
        header = """\
import launch
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration,
    TimerAction)
from launch.conditions import (
    IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals, UnlessCondition)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import (
    ComposableNodeContainer, LoadComposableNodes, Node, PushRosNamespace)
from launch_ros.descriptions import ComposableNode


def node(name, value=None, **kwargs):
    value = LaunchConfiguration('x') if value is None else value
    return Node(package='test_pkg', executable='n', name=name, parameters=[{'p': value}],
                **kwargs)


"""
        child_path = launch_dir / "child.launch.py"
        (launch_dir / "test.launch.py").write_text(header + f"""\
def include(condition=None, **launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource({str(child_path)!r}),
        launch_arguments=launch_arguments.items(), condition=condition)


def generate_launch_description():
    return launch.LaunchDescription({body})
""")
        if child is not None:
            child_path.write_text(header + f"""\
def generate_launch_description():
    return launch.LaunchDescription({child})
""")
        from colcon_lf.launch_analysis.parser import parse_launch, clear_parse_launch_cache
        clear_parse_launch_cache()
        graph = parse_launch(launch_dir / "test.launch.py", workspace_root=tmp_path)
        if graphs is not None:
            graphs.append(graph)
        params = {n.name: n.parameters.get("p") for n in graph.all_nodes}
        errors = list(graph.root.errors)
        for included in graph.root.includes:
            errors.extend(included.errors)
        return params, errors

    def test_set_launch_configuration_overrides_default(self, tmp_path):
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            node('before'),
            SetLaunchConfiguration('x', '2'),
            node('after'),
        ]""")
        assert params == {"before": 1, "after": 2}

    def test_set_before_group_reaches_node_inside(self, tmp_path):
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration('x', '2'),
            GroupAction([node('inside')]),
        ]""")
        assert params == {"inside": 2}

    def test_scoped_group_set_does_not_leak(self, tmp_path):
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            GroupAction([SetLaunchConfiguration('x', '2'), node('inside')]),
            node('after'),
            GroupAction([SetLaunchConfiguration('x', '3')], scoped=False),
            node('after_unscoped'),
        ]""")
        assert params == {"inside": 2, "after": 1, "after_unscoped": 3}

    def test_pushed_namespace_keys_inline_dict_by_full_name(self, tmp_path):
        # Jazzy node.py: prefix_namespace(pushed, None) is the pushed
        # namespace, so a named node under PushRosNamespace is fully
        # specified and its inline dict is keyed /ns/n, not /**.
        launch_dir = tmp_path / "src" / "test_pkg" / "launch"
        launch_dir.mkdir(parents=True)
        (launch_dir / "test.launch.py").write_text("""\
import launch
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    return launch.LaunchDescription([GroupAction([
        PushRosNamespace('ns'),
        Node(package='test_pkg', executable='n', name='n', parameters=[{'p': 1}]),
    ])])
""")
        from colcon_lf.launch_analysis.parser import parse_launch, clear_parse_launch_cache
        clear_parse_launch_cache()
        graph = parse_launch(launch_dir / "test.launch.py", workspace_root=tmp_path)
        (node,) = graph.all_nodes
        assert node.parameter_sources == [{"params": {"p": 1}, "key": "/ns/n"}]

    @pytest.mark.parametrize("text, expected", [
        ("true", True),
        ("1.5", 1.5),
        ("''", ""),
        ("[1, 2]", [1, 2]),
        ("hello", "hello"),
    ])
    def test_value_typed_like_launch_ros(self, tmp_path, text, expected):
        params, errors = self._parse(tmp_path, f"""[
            DeclareLaunchArgument('x', default_value={text!r}),
            node('n'),
        ]""")
        assert params["n"] == expected and type(params["n"]) is type(expected)
        assert errors == []

    def test_empty_value_is_empty_string(self, tmp_path):
        # Concatenated with an empty argument value the text is empty;
        # launch_ros turns that into '' rather than YAML null.
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration('x', ''),
            node('n'),
        ]""")
        assert params["n"] == "" and errors == []

    @pytest.mark.parametrize("text, kind", [
        ("{a: 1}", "dict"), ("[1, a]", "list"), ("~", "NoneType"),
    ])
    def test_value_launch_ros_rejects_is_an_error(self, tmp_path, text, kind):
        # launch_ros evaluate_parameter_dict raises TypeError for these.
        params, errors = self._parse(tmp_path, f"""[
            DeclareLaunchArgument('x', default_value={text!r}),
            node('n'),
        ]""")
        assert params["n"] == text
        assert errors == [
            f"node 'n' parameter 'p': value {text!r} parses as {kind}; "
            f"launch_ros rejects it"]

    def test_substituted_var_not_expanded_again(self, tmp_path):
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('y', default_value='7'),
            DeclareLaunchArgument('x', default_value='$(var y)'),
            node('n'),
        ]""")
        assert params["n"] == "$(var y)"

    def test_argument_without_value_is_an_error(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x'),
            node('n'),
        ]""")
        assert params["n"] == "$(var x)"
        assert errors == ["node 'n' parameter 'p': launch argument 'x' has no value"]

    def test_set_value_evaluated_when_set(self, tmp_path):
        # SetLaunchConfiguration performs its value when it runs
        # (launch/actions/set_launch_configuration.py:71-74).
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('y', default_value='5'),
            SetLaunchConfiguration('x', LaunchConfiguration('y')),
            SetLaunchConfiguration('y', '6'),
            node('n'),
        ]""")
        assert params == {"n": 5}


class TestLaunchConfigurationScope:
    """Include and ``GroupAction(forwarding=False)`` scope like launch.

    Rules from launch in the Jazzy dev image (``/opt/ros/jazzy/lib/python3.12/
    site-packages/launch/actions/``): an include sets its launch arguments as
    configurations and yields the description without pushing a scope
    (``include_launch_description.py:238-243``); a declared argument's default
    applies only when the configuration is unset
    (``declare_launch_argument.py:205-215``); a scoped group with
    ``forwarding=False`` resets the configurations to its evaluated
    ``launch_configurations`` (``group_action.py:127-136``,
    ``reset_launch_configurations.py:84-109``).
    """

    _parse = staticmethod(TestLaunchArgumentParameter._parse)

    def test_set_inside_include_reaches_parent(self, tmp_path):
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            include(),
            node('after'),
        ]""", child="""[
            SetLaunchConfiguration('x', '2'),
            node('inside'),
        ]""")
        assert params == {"inside": 2, "after": 2}

    def test_include_argument_reaches_parent(self, tmp_path):
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            include(x='3'),
            node('after'),
        ]""", child="[node('inside')]")
        assert params == {"inside": 3, "after": 3}

    def test_parent_value_wins_over_child_default(self, tmp_path):
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            include(),
        ]""", child="""[
            DeclareLaunchArgument('x', default_value='2'),
            node('inside'),
        ]""")
        assert params == {"inside": 1}

    def test_child_default_reaches_parent(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            include(),
            node('after'),
        ]""", child="[DeclareLaunchArgument('x', default_value='2')]")
        assert params == {"after": 2} and errors == []

    def test_unforwarded_group_keeps_its_configurations(self, tmp_path):
        # The kept values are evaluated in the parent scope before the reset.
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            DeclareLaunchArgument('y', default_value='5'),
            GroupAction([node('kept')], forwarding=False,
                        launch_configurations={'x': LaunchConfiguration('y')}),
            node('after'),
        ]""")
        assert params == {"kept": 5, "after": 1}

    def test_unforwarded_group_drops_other_configurations(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            GroupAction([node('n')], forwarding=False),
        ]""")
        assert params == {"n": "$(var x)"}
        assert errors == ["node 'n' parameter 'p': launch argument 'x' has no value"]

    def test_unforwarded_group_applies_declared_default(self, tmp_path):
        params, _ = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            GroupAction([DeclareLaunchArgument('x', default_value='4'), node('n')],
                        forwarding=False),
            node('after'),
        ]""")
        assert params == {"n": 4, "after": 1}

    def test_set_name_is_performed(self, tmp_path):
        # SetLaunchConfiguration performs its name as well as its value
        # (set_launch_configuration.py:71-74).
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('key', default_value='x'),
            SetLaunchConfiguration(LaunchConfiguration('key'), '7'),
            node('n'),
        ]""")
        assert params == {"n": 7} and errors == []

    def test_empty_list_default_is_empty_string(self, tmp_path):
        # perform_substitutions over [] is '' (declare_launch_argument.py:214-215).
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value=[]),
            node('n'),
        ]""")
        assert params == {"n": ""} and errors == []

    def test_launch_configuration_default_applies_while_unset(self, tmp_path):
        # LaunchConfiguration.perform returns its default only when the name
        # is not set (launch/substitutions/launch_configuration.py:87-102).
        params, errors = self._parse(tmp_path, """[
            node('unset', LaunchConfiguration('x', default='3')),
            SetLaunchConfiguration('y', LaunchConfiguration('x', default='4')),
            node('from_set', LaunchConfiguration('y')),
            DeclareLaunchArgument('x', default_value='1'),
            node('set', LaunchConfiguration('x', default='3')),
            GroupAction([node('reset', LaunchConfiguration('x', default='')),
                         node('reset_spaced', LaunchConfiguration('x', default='a b'))],
                        forwarding=False),
        ]""")
        assert params == {"unset": 3, "from_set": 4, "set": 1, "reset": "",
                          "reset_spaced": "a b"}
        assert errors == []

    def test_unforwarded_group_resets_namespace(self, tmp_path):
        # forwarding=False clears ros_namespace, which PushRosNamespace and Node
        # read (launch_ros push_ros_namespace.py:67-82, node.py:397).
        graphs = []
        self._parse(tmp_path, """[
            PushRosNamespace('a'),
            GroupAction([node('reset', '1')], forwarding=False),
            GroupAction([node('kept', '1')], forwarding=False,
                        launch_configurations={'ros_namespace': '/b'}),
            GroupAction([PushRosNamespace('c'), node('pushed', '1')], forwarding=False),
            GroupAction([node('forwarded', '1')]),
            node('outside', '1'),
        ]""", graphs=graphs)
        namespaces = {n.name: n.namespace for n in graphs[0].all_nodes}
        assert namespaces == {"reset": None, "kept": "/b", "pushed": "/c",
                              "forwarded": "/a", "outside": "/a"}


# ---------------------------------------------------------------------------
# Assertion 6 + 7: system_bringup — arguments, namespaces, all nodes
# ---------------------------------------------------------------------------

class TestConditionalLaunchConfiguration:
    """A conditional set or declared default applies only when its condition holds.

    launch runs an action only when its condition is None or evaluates true
    (``launch/action.py:105-109``, Jazzy). IfCondition accepts ``true``/``1``/
    ``false``/``0`` after ``strip().lower()`` and raises on anything else
    (``conditions/evaluate_condition_expression_impl.py``). A condition the
    analyzer cannot evaluate is recorded as an error and leaves the
    configuration unknown, never a guessed branch.
    """

    _parse = staticmethod(TestLaunchArgumentParameter._parse)

    def test_false_set_is_skipped(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration('x', '2', condition=IfCondition('false')),
            node('n'),
        ]""")
        assert params == {"n": 1} and errors == []

    def test_false_declared_default_is_skipped(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='2', condition=IfCondition('0')),
            DeclareLaunchArgument('x', default_value='3'),
            node('n'),
        ]""")
        assert params == {"n": 3} and errors == []

    def test_condition_reads_the_current_scope(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('use', default_value='true'),
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration('use', ' False '),
            SetLaunchConfiguration('x', '2', condition=IfCondition(LaunchConfiguration('use'))),
            node('if_false'),
            SetLaunchConfiguration('x', '3', condition=UnlessCondition(LaunchConfiguration('use'))),
            node('unless_false'),
        ]""")
        assert params == {"if_false": 1, "unless_false": 3} and errors == []

    def test_launch_configuration_equals(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('mode', default_value='real'),
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration('x', '2', condition=LaunchConfigurationEquals('mode', 'sim')),
            node('equals'),
            SetLaunchConfiguration(
                'x', '3', condition=LaunchConfigurationNotEquals('mode', 'sim')),
            node('not_equals'),
            SetLaunchConfiguration('x', '4', condition=LaunchConfigurationEquals('unset', None)),
            node('unset_equals_none'),
        ]""")
        assert params == {"equals": 1, "not_equals": 3, "unset_equals_none": 4}
        assert errors == []

    def test_python_expression_condition(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('mode', default_value='sim'),
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration('x', '2', condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('mode'), "' == 'sim'"]))),
            node('n'),
        ]""")
        assert params == {"n": 2} and errors == []

    def test_condition_inside_include_and_group(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            include(use='false'),
            node('after_include'),
            GroupAction([
                SetLaunchConfiguration('x', '5', condition=UnlessCondition('true')),
                node('in_group'),
            ], forwarding=False, launch_configurations={'x': '4'}),
        ]""", child="""[
            DeclareLaunchArgument('use', default_value='true'),
            SetLaunchConfiguration('x', '2', condition=IfCondition(LaunchConfiguration('use'))),
            node('inside'),
        ]""")
        assert params == {"inside": 1, "after_include": 1, "in_group": 4}
        assert errors == []

    def test_undecidable_set_is_reported_not_guessed(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration(
                'x', '2', condition=IfCondition(EnvironmentVariable('USE_X'))),
            node('n'),
        ]""")
        assert params == {"n": "$(var x)"}
        assert errors == [
            "SetLaunchConfiguration 'x': condition IfCondition(...) not evaluated "
            "(it uses EnvironmentVariable, which is not evaluated); 'x' is unknown after it",
            "node 'n' parameter 'p': launch argument 'x' has no known value here",
        ]

    def test_undecidable_declared_default_is_reported(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument(
                'x', default_value='2', condition=IfCondition(LaunchConfiguration('use'))),
            node('n'),
        ]""")
        assert params == {"n": "$(var x)"}
        assert errors == [
            "DeclareLaunchArgument 'x': condition IfCondition($(var use)) not evaluated "
            "(launch configuration 'use' does not exist); 'x' is unknown after it",
            "node 'n' parameter 'p': launch argument 'x' has no known value here",
        ]

    def test_invalid_expression_is_reported(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration('x', '2', condition=IfCondition('yes')),
            node('n'),
        ]""")
        assert params == {"n": "$(var x)"}
        assert errors[0].startswith(
            "SetLaunchConfiguration 'x': condition IfCondition(yes) not evaluated "
            "(invalid condition expression")

    def test_unknown_value_makes_later_condition_undecidable(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('use', default_value='true'),
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration(
                'use', 'false', condition=IfCondition(EnvironmentVariable('E'))),
            SetLaunchConfiguration('x', '2', condition=IfCondition(LaunchConfiguration('use'))),
            node('n'),
        ]""")
        assert params == {"n": "$(var x)"}
        assert "SetLaunchConfiguration 'x': condition IfCondition($(var use)) not " \
            "evaluated (launch configuration 'use' has no known value here); " \
            "'x' is unknown after it" in errors

    # Review: one test per item.

    def test_false_group_or_include_changes_no_scope(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            GroupAction([SetLaunchConfiguration('x', '2'), node('in_false_group')],
                        scoped=False, condition=IfCondition('false')),
            node('after_group'),
            include(x='3', condition=UnlessCondition('true')),
            node('after_include'),
        ]""", child="""[SetLaunchConfiguration('x', '4')]""")
        # The false group's node is not started either.
        assert params == {"after_group": 1, "after_include": 1}
        assert errors == []

    def test_undecidable_group_or_include_makes_its_names_unknown(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            DeclareLaunchArgument('y', default_value='1'),
            GroupAction([SetLaunchConfiguration('x', '2')], scoped=False,
                        condition=IfCondition(EnvironmentVariable('E'))),
            node('after_group'),
            include(y='3', condition=IfCondition(EnvironmentVariable('E'))),
            node('after_include', LaunchConfiguration('y')),
        ]""", child="""[]""")
        assert params == {"after_group": "$(var x)", "after_include": "$(var y)"}
        assert "GroupAction: condition IfCondition(...) not evaluated (it uses " \
            "EnvironmentVariable, which is not evaluated); 'x' is unknown after it" in errors
        assert "IncludeLaunchDescription: condition IfCondition(...) not evaluated (it " \
            "uses EnvironmentVariable, which is not evaluated); 'y' is unknown after it" \
            in errors

    def test_unevaluated_set_value_is_unknown(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            SetLaunchConfiguration('mode', EnvironmentVariable('MODE')),
            SetLaunchConfiguration('x', '2', condition=LaunchConfigurationEquals('mode', 'sim')),
            node('n'),
        ]""")
        assert params == {"n": "$(var x)"}
        assert "SetLaunchConfiguration 'x': condition LaunchConfigurationEquals('mode', " \
            "'sim') not evaluated (launch configuration 'mode' has no known value here); " \
            "'x' is unknown after it" in errors

    def test_python_expression_reading_the_host_is_not_evaluated(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            DeclareLaunchArgument('flag', default_value='True'),
            SetLaunchConfiguration('x', '2', condition=IfCondition(PythonExpression(
                ["__import__('os').getenv('HOME') is not None"]))),
            node('host'),
            SetLaunchConfiguration('x', '3', condition=IfCondition(PythonExpression(
                [LaunchConfiguration('flag'), " and 1 in (1, 2) and not 'a' == 'b'"]))),
            node('pure'),
        ]""")
        assert params == {"host": "$(var x)", "pure": 3}
        assert errors[0] == (
            "SetLaunchConfiguration 'x': condition IfCondition(...) not evaluated (its "
            "PythonExpression is not only literals, comparisons and boolean operators); "
            "'x' is unknown after it")

    def test_declared_condition_is_evaluated_when_name_is_set(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            DeclareLaunchArgument('x', default_value='1'),
            DeclareLaunchArgument('x', default_value='2', condition=IfCondition('yes')),
            node('n'),
        ]""")
        assert params == {"n": 1}
        assert len(errors) == 1 and errors[0].startswith(
            "DeclareLaunchArgument 'x': condition IfCondition(yes) not evaluated "
            "(invalid condition expression") and "unknown" not in errors[0]

    def test_unevaluated_group_and_include_values_are_reported(self, tmp_path):
        params, errors = self._parse(tmp_path, """[
            GroupAction([node('in_group')],
                        launch_configurations={'x': EnvironmentVariable('E')}),
            include(x=EnvironmentVariable('E')),
        ]""", child="""[node('in_child')]""")
        assert params == {"in_group": "$(var x)", "in_child": "$(var x)"}
        assert "GroupAction: launch configuration 'x' not evaluated; " \
            "'x' is unknown in the group" in errors
        assert "IncludeLaunchDescription: launch argument 'x' not evaluated; " \
            "'x' is unknown after it" in errors


class TestLaunchConditionDropsNodes:
    """A node behind a false condition is not started, as in launch.

    launch skips an action whose condition is false (``launch/action.py:105-109``,
    Jazzy): a Node, and everything inside a GroupAction, IncludeLaunchDescription,
    TimerAction or container. A ComposableNode's own condition is evaluated
    when the container loads it (``composable_node_container.py:85``,
    ``load_composable_nodes.py:279-282``). An undecidable condition keeps the
    node, with ``condition_result`` None and an error naming node and condition.
    """

    _parse = staticmethod(TestLaunchArgumentParameter._parse)

    def _graph(self, tmp_path, body, child=None):
        graphs = []
        params, _ = self._parse(tmp_path, body, child=child, graphs=graphs)
        from colcon_lf.verb.analyze import _launch_errors
        return graphs[0], params, [err for _, err in _launch_errors(graphs[0].root)]

    def test_own_condition_reads_the_current_scope(self, tmp_path):
        graph, params, errors = self._graph(tmp_path, """[
            DeclareLaunchArgument('use', default_value='true'),
            node('if_true', 1, condition=IfCondition(LaunchConfiguration('use'))),
            node('unless_true', 1, condition=UnlessCondition(LaunchConfiguration('use'))),
            SetLaunchConfiguration('use', 'false'),
            node('if_false', 1, condition=IfCondition(LaunchConfiguration('use'))),
            node('unless_false', 1, condition=UnlessCondition(LaunchConfiguration('use'))),
            node('equals', 1, condition=LaunchConfigurationEquals('use', 'false')),
            node('not_equals', 1, condition=LaunchConfigurationNotEquals('use', 'false')),
        ]""")
        assert params == {"if_true": 1, "unless_false": 1, "equals": 1}
        assert [n.condition_result for n in graph.all_nodes] == [True, True, True]
        assert errors == []

    def test_false_group_and_include_drop_everything_inside(self, tmp_path):
        grand = tmp_path / "grand.launch.py"
        grand.write_text(
            "import launch\n"
            "from launch_ros.actions import Node\n\n\n"
            "def generate_launch_description():\n"
            "    return launch.LaunchDescription([Node(\n"
            "        package='test_pkg', executable='n', name='in_grand')])\n")
        graph, params, errors = self._graph(tmp_path, """[
            GroupAction([node('in_group', 1), GroupAction([node('nested', 1)]), include()],
                        condition=IfCondition('false')),
            include(condition=UnlessCondition('true')),
            node('after', 1),
            include(),
        ]""", child=f"""[node('in_child', 1), IncludeLaunchDescription(
                PythonLaunchDescriptionSource({str(grand)!r}))]""")
        # The unconditional include at the end still parses the child that
        # the false include named.
        assert sorted(n.name for n in graph.all_nodes) == ["after", "in_child", "in_grand"]
        assert errors == []

    def test_composable_and_timer_conditions(self, tmp_path):
        graph, _, errors = self._graph(tmp_path, """[
            ComposableNodeContainer(
                name='c', namespace='', package='rclcpp_components',
                executable='component_container', composable_node_descriptions=[
                    ComposableNode(package='test_pkg', plugin='P', name='comp_true'),
                    ComposableNode(package='test_pkg', plugin='P', name='comp_false',
                                   condition=IfCondition('false'))]),
            ComposableNodeContainer(
                name='off', namespace='', package='rclcpp_components',
                executable='component_container', condition=IfCondition('false'),
                composable_node_descriptions=[
                    ComposableNode(package='test_pkg', plugin='P', name='in_false_container')]),
            LoadComposableNodes(target_container='c', composable_node_descriptions=[
                ComposableNode(package='test_pkg', plugin='P', name='loaded_false',
                               condition=UnlessCondition('1'))]),
            TimerAction(period=1.0, actions=[node('in_false_timer', 1)],
                        condition=IfCondition('0')),
            TimerAction(period=1.0, actions=[node('in_timer', 1)]),
        ]""")
        assert sorted(n.name for n in graph.all_nodes) == ["comp_true", "in_timer"]
        assert errors == []

    def test_set_inside_a_timer_does_not_reach_later_siblings(self, tmp_path):
        # The timer's actions run on a copy of the configurations and their
        # changes do not persist (timer_action.py handle()), also when the
        # timer's own condition is undecidable.
        graph, params, errors = self._graph(tmp_path, """[
            DeclareLaunchArgument('use', default_value='true'),
            TimerAction(period=1.0, actions=[
                SetLaunchConfiguration('use', 'false'),
                node('in_timer_after_set', 1,
                     condition=IfCondition(LaunchConfiguration('use')))]),
            TimerAction(period=1.0, condition=IfCondition(EnvironmentVariable('E')),
                        actions=[SetLaunchConfiguration('use', 'false')]),
            node('after_timer', 1, condition=IfCondition(LaunchConfiguration('use'))),
        ]""")
        assert params == {"after_timer": 1}
        assert [n.condition_result for n in graph.all_nodes] == [True]
        assert errors == []

    def test_undecidable_condition_keeps_the_node_and_is_reported(self, tmp_path):
        graph, params, errors = self._graph(tmp_path, """[
            node('own', 1, condition=IfCondition(EnvironmentVariable('E'))),
            node('invalid', 1, condition=IfCondition('yes')),
            GroupAction([node('in_group', 1),
                         GroupAction([node('nested', 1)], condition=IfCondition('true')),
                         node('false_in_group', 1, condition=IfCondition('false'))],
                        condition=IfCondition(EnvironmentVariable('E'))),
            include(condition=IfCondition(EnvironmentVariable('E'))),
        ]""", child="""[node('in_child', 1)]""")
        assert params == {"own": 1, "invalid": 1, "in_group": 1, "nested": 1, "in_child": 1}
        assert all(n.condition_result is None for n in graph.all_nodes)
        env = "not evaluated (it uses EnvironmentVariable, which is not evaluated); " \
            "the generated system assumes it starts"
        assert f"Node 'own': condition IfCondition(...) {env}" in errors
        assert f"Node 'in_group': enclosing GroupAction condition IfCondition(...) {env}" \
            in errors
        assert f"Node 'nested': enclosing GroupAction condition IfCondition(...) {env}" \
            in errors
        assert f"Node 'in_child': enclosing IncludeLaunchDescription condition " \
            f"IfCondition(...) {env}" in errors
        invalid = [e for e in errors if e.startswith("Node 'invalid'")]
        assert len(invalid) == 1 and invalid[0].startswith(
            "Node 'invalid': condition IfCondition(yes) not evaluated (invalid condition "
            "expression") and invalid[0].endswith("the generated system assumes it starts")

    def test_conditions_fixture_starts_only_the_primary_node(self, launch_parser):
        # enable_primary defaults to 'true': stock launch starts primary_node only.
        graph = launch_parser("launch_features_example", "03_conditions.launch.py")
        assert [n.name for n in graph.all_nodes] == ["primary_node"]
        assert graph.all_nodes[0].condition_result is True


class TestSystemBringup:
    def test_namespace_argument_declared(self, launch_parser):
        """Assertion 6a: 'namespace' argument with default 'robot_1'."""
        graph = launch_parser("system_bringup", "system_bringup.launch.py")
        arg_names = {a.name: a for a in graph.root.arguments}
        assert "namespace" in arg_names
        assert arg_names["namespace"].default_value == "robot_1"

    def test_use_sim_argument_declared(self, launch_parser):
        """Assertion 6b: 'use_sim' argument with default 'false'."""
        graph = launch_parser("system_bringup", "system_bringup.launch.py")
        arg_names = {a.name: a for a in graph.root.arguments}
        assert "use_sim" in arg_names
        assert arg_names["use_sim"].default_value == "false"

    # The included launch files resolve through the install space's package shares.
    @requires_ros_headers("the system_bringup package-set test")
    def test_all_nodes_has_multiple_packages(self, launch_parser):
        """Assertion 7: all_nodes includes nodes from multiple packages."""
        graph = launch_parser("system_bringup", "system_bringup.launch.py")
        packages = {n.package for n in graph.all_nodes}
        assert "pubsub_example" in packages
        assert "timer_example" in packages
        assert "service_example" in packages

    def test_namespaced_nodes_have_namespace(self, launch_parser):
        """Assertion 7b: nodes inside GroupAction carry the 'robot_1' namespace."""
        graph = launch_parser("system_bringup", "system_bringup.launch.py")
        # Nodes inside the GroupAction+PushRosNamespace should have namespace set
        pubsub_nodes = [n for n in graph.all_nodes if n.package == "pubsub_example"]
        assert len(pubsub_nodes) >= 1
        # At least one pubsub node should have a namespace containing 'robot_1'
        namespaces = [n.namespace for n in pubsub_nodes if n.namespace]
        assert any("robot_1" in ns for ns in namespaces)

    def test_false_conditioned_node_is_not_started(self, launch_parser):
        """``sim_timer`` has IfCondition(use_sim), default 'false': launch skips it."""
        graph = launch_parser("system_bringup", "system_bringup.launch.py")
        assert "sim_timer" not in {n.name for n in graph.all_nodes}
        # The use_* includes default to 'false' as well.
        assert not {"multi_service_example", "action_full_example",
                    "reentrant_fanout_example"} & {n.package for n in graph.all_nodes}

    def test_has_groups(self, launch_parser):
        """system_bringup uses GroupAction — groups list should be non-empty."""
        graph = launch_parser("system_bringup", "system_bringup.launch.py")
        assert len(graph.root.groups) >= 1
        assert graph.root.groups[0].namespace is not None


# ---------------------------------------------------------------------------
# Depth limit and new feature tests (require launch_ros)
# ---------------------------------------------------------------------------

class TestMaxIncludeDepth:
    """Verify that deeply nested includes don't crash with RecursionError."""

    def test_depth_limit_returns_error_stub(self, tmp_path):
        """Create a chain of launch files deeper than _MAX_INCLUDE_DEPTH."""
        from colcon_lf.launch_analysis.parser import _MAX_INCLUDE_DEPTH

        src_dir = tmp_path / "src" / "depth_pkg"
        launch_dir = src_dir / "launch"
        launch_dir.mkdir(parents=True)
        (src_dir / "package.xml").write_text(
            '<package><name>depth_pkg</name></package>'
        )

        # Create a chain: level_0 includes level_1, ..., level_N includes level_N+1
        chain_len = _MAX_INCLUDE_DEPTH + 5
        for i in range(chain_len + 1):
            content = "import launch\n"
            content += "from launch.actions import IncludeLaunchDescription\n"
            content += "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
            content += "def generate_launch_description():\n"
            if i < chain_len:
                next_file = launch_dir / f"level_{i + 1}.launch.py"
                content += f"    return launch.LaunchDescription([\n"
                content += f"        IncludeLaunchDescription(PythonLaunchDescriptionSource('{next_file}')),\n"
                content += f"    ])\n"
            else:
                content += f"    return launch.LaunchDescription([])\n"
            (launch_dir / f"level_{i}.launch.py").write_text(content)

        from colcon_lf.launch_analysis.parser import parse_launch, clear_parse_launch_cache
        clear_parse_launch_cache()

        # This should NOT raise RecursionError
        graph = parse_launch(
            launch_dir / "level_0.launch.py",
            workspace_root=tmp_path,
        )
        assert graph is not None
        # Should have some nodes or at least not crash


class TestLoadComposableNodes:
    """Verify that LoadComposableNodes entities are extracted."""

    def test_load_composable_nodes_extracted(self, tmp_path):
        """LoadComposableNodes with inline ComposableNode descriptions."""
        src_dir = tmp_path / "src" / "comp_pkg"
        launch_dir = src_dir / "launch"
        launch_dir.mkdir(parents=True)
        (src_dir / "package.xml").write_text(
            '<package><name>comp_pkg</name></package>'
        )

        (launch_dir / "test.launch.py").write_text("""\
import launch
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
    )
    loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package='test_pkg',
                plugin='test_pkg::MyNode',
                name='my_node',
            ),
        ],
        target_container='my_container',
    )
    return launch.LaunchDescription([container, loader])
""")

        from colcon_lf.launch_analysis.parser import parse_launch, clear_parse_launch_cache
        clear_parse_launch_cache()

        graph = parse_launch(
            launch_dir / "test.launch.py",
            workspace_root=tmp_path,
        )
        # Should find the node from LoadComposableNodes
        component_nodes = [n for n in graph.all_nodes if n.is_component]
        assert len(component_nodes) >= 1
        assert any(n.package == "test_pkg" for n in component_nodes)


# ---------------------------------------------------------------------------
# Analysis leaves the analysed source tree untouched
# ---------------------------------------------------------------------------

def test_parse_launch_writes_no_bytecode_into_source(tmp_path):
    """Loading a Python launch file must not leave launch/__pycache__ behind.

    The directory broke a later ``colcon build --symlink-install`` of
    nav2_simple_commander, whose setup.py installs every file under launch/.
    """
    launch_dir = tmp_path / "src" / "test_pkg" / "launch"
    launch_dir.mkdir(parents=True)
    (launch_dir / "test.launch.py").write_text("""\
import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(package='test_pkg', executable='test_node', name='test_node'),
    ])
""")

    from colcon_lf.launch_analysis.parser import parse_launch, clear_parse_launch_cache
    clear_parse_launch_cache()

    graph = parse_launch(launch_dir / "test.launch.py", workspace_root=tmp_path)

    assert [n.executable for n in graph.all_nodes] == ["test_node"]
    assert not (launch_dir / "__pycache__").exists(), (
        f"analysis wrote {sorted(p.name for p in (launch_dir / '__pycache__').iterdir())} "
        f"into {launch_dir / '__pycache__'}"
    )


def test_concurrent_launch_loads_restore_bytecode_flag(tmp_path, monkeypatch):
    """Two threads loading launch files leave sys.dont_write_bytecode as found.

    Unguarded, the second load saves the first one's True and restores it
    last, so bytecode writing stays off for the rest of the process.
    """
    import sys
    import threading
    import time

    import launch.launch_description_sources as sources
    from colcon_lf.launch_analysis.parser import parse_launch, clear_parse_launch_cache

    loads = []

    class _SlowSource:
        def __init__(self, path):
            pass

        def try_get_launch_description_without_context(self):
            loads.append(sys.dont_write_bytecode)
            time.sleep(0.2)
            return None

    monkeypatch.setattr(sources, "AnyLaunchDescriptionSource", _SlowSource)
    monkeypatch.setattr(sys, "dont_write_bytecode", False)
    clear_parse_launch_cache()
    launch_dir = tmp_path / "src" / "test_pkg" / "launch"
    launch_dir.mkdir(parents=True)
    files = [launch_dir / f"{name}.launch.py" for name in ("first", "second")]
    threads = []
    for path in files:
        path.write_text("")
        threads.append(threading.Thread(target=parse_launch, args=(path, tmp_path)))
        threads[-1].start()
        time.sleep(0.05)
    for thread in threads:
        thread.join()

    assert loads == [True, True]
    assert sys.dont_write_bytecode is False, (
        "sys.dont_write_bytecode left True after two concurrent launch loads"
    )
