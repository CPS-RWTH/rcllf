# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``maxwait_ms`` with ``take_timeout_ms`` as its alias.

execution.yaml writes the
take/join wait bound as ``maxwait_ms``. ``take_timeout_ms`` stays accepted as
an alias; the internal key, the resolved model field, the generated policy
table and the run records keep the name ``take_timeout_ms``. Both names at
one level with different values are refused naming both; with the same value
they are accepted with one warning. A value under one name overrides one
under the other, across layers and across scopes.
"""

from __future__ import annotations

import textwrap
import warnings

import pytest
import yaml

from colcon_lf.exec_config.interactive import run_interactive
from colcon_lf.exec_config.model import ExecConfig
from colcon_lf.exec_config.resolve import resolve, resolve_corr_bounds
from colcon_lf.exec_config.scaffold import REFERENCE_FILENAME, scaffold

from .conftest import make_graph, make_node


def _graph():
    return make_graph({"n": make_node("n", subscriptions=["t", "u"])})


def _load(text: str, source: str = "x.yaml") -> ExecConfig:
    return ExecConfig.from_dict(yaml.safe_load(textwrap.dedent(text)), _graph(), source=source)


class TestNewKey:
    def test_maxwait_ms_is_accepted_at_every_scope(self):
        config = _load("""\
            defaults: {maxwait_ms: 100}
            nodes: {n: {maxwait_ms: 200}}
            subscriptions: {n: {t: {maxwait_ms: 300}}}
        """)
        assert config.defaults == {"take_timeout_ms": 100}
        assert config.nodes == {"n": {"take_timeout_ms": 200}}
        assert config.subscriptions == {"n": {"t": {"take_timeout_ms": 300}}}

    def test_either_name_yields_the_same_config(self):
        text = """\
            defaults: {take_mode: block, {key}: 40, reader_depth: 3}
            nodes: {n: {{key}: 50}}
            subscriptions: {n: {t: {{key}: 60, take_mode: skip}}}
        """
        old = _load(text.replace("{key}", "take_timeout_ms"))
        new = _load(text.replace("{key}", "maxwait_ms"))
        # Same values in the same key order: the generated output cannot tell.
        assert list(new.defaults.items()) == list(old.defaults.items())
        assert new.nodes == old.nodes
        assert list(new.subscriptions["n"]["t"].items()) == list(
            old.subscriptions["n"]["t"].items())

    def test_resolved_row_and_corr_bounds_read_maxwait_ms(self):
        config = _load("""\
            defaults: {maxwait_ms: 70}
            subscriptions: {n: {t: {maxwait_ms: 90}}}
        """)
        rows = {row.topic: row.values for row in resolve(config, _graph()).subscriptions}
        assert rows["t"]["take_timeout_ms"] == 90
        assert rows["u"]["take_timeout_ms"] == 70
        bounds = resolve_corr_bounds(config, _graph())
        assert bounds.timeout_ms("n", "t") == 90
        assert bounds.timeout_ms("n", "u") == 70

    @pytest.mark.parametrize("key", ["maxwait_ms", "take_timeout_ms"])
    def test_a_bad_value_names_the_key_the_file_wrote(self, key):
        with pytest.raises(ValueError) as exc:
            _load(f"defaults: {{{key}: 0}}\n")
        assert str(exc.value) == f"x.yaml: defaults: '{key}' must be an int > 0, got 0"


class TestBothNames:
    def test_different_values_are_refused_naming_both(self):
        with pytest.raises(ValueError) as exc:
            _load("""\
                subscriptions: {n: {t: {take_timeout_ms: 100, maxwait_ms: 200}}}
            """)
        assert str(exc.value) == (
            "x.yaml: subscriptions.'n'.'t': 'maxwait_ms' and its alias 'take_timeout_ms' are "
            "both set with different values (maxwait_ms: 200, take_timeout_ms: 100); keep "
            "only 'maxwait_ms'"
        )

    @pytest.mark.parametrize("scope", ["defaults: {{{body}}}", "nodes: {{n: {{{body}}}}}"])
    def test_different_values_are_refused_at_defaults_and_nodes(self, scope):
        with pytest.raises(ValueError, match="'maxwait_ms' and its alias 'take_timeout_ms'"):
            _load(scope.format(body="maxwait_ms: 5, take_timeout_ms: 6") + "\n")

    def test_the_same_value_is_accepted_with_one_warning(self):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            config = _load("defaults: {maxwait_ms: 250, take_timeout_ms: 250}\n")
        assert config.defaults == {"take_timeout_ms": 250}
        messages = [str(w.message) for w in caught if issubclass(w.category, UserWarning)]
        assert messages == [
            "x.yaml: defaults: 'take_timeout_ms' is an alias of 'maxwait_ms' and both are set "
            "to 250; keep only 'maxwait_ms'"
        ]


class TestOverridesAcrossNames:
    """A value under one name overrides one under the other (the documented precedence)."""

    @pytest.mark.parametrize("pkg_key,user_key", [
        ("take_timeout_ms", "maxwait_ms"), ("maxwait_ms", "take_timeout_ms"),
    ])
    def test_user_layer_overrides_package_layer(self, tmp_path, pkg_key, user_key):
        graph = _graph()
        package = tmp_path / "package.yaml"
        user = tmp_path / "user.yaml"
        package.write_text(f"defaults: {{{pkg_key}: 250}}\n"
                           f"subscriptions: {{n: {{t: {{{pkg_key}: 300}}}}}}\n")
        user.write_text(f"defaults: {{{user_key}: 40}}\n"
                        f"subscriptions: {{n: {{t: {{{user_key}: 50}}}}}}\n")
        config = ExecConfig.load_layered(user, package, graph)
        assert config.defaults == {"take_timeout_ms": 40}
        assert config.origin(("defaults", "take_timeout_ms")) == f"user {user}"
        rows = {row.topic: row for row in resolve(config, graph).subscriptions}
        assert rows["t"].values["take_timeout_ms"] == 50
        assert rows["t"].origins["take_timeout_ms"] == f"user {user} (subscriptions)"
        assert rows["u"].values["take_timeout_ms"] == 40
        assert resolve_corr_bounds(config, graph).timeout_ms("n", "t") == 50

    @pytest.mark.parametrize("wide,narrow", [
        ("take_timeout_ms", "maxwait_ms"), ("maxwait_ms", "take_timeout_ms"),
    ])
    def test_narrower_scope_overrides_across_names(self, wide, narrow):
        config = _load(f"""\
            defaults: {{{wide}: 250}}
            nodes: {{n: {{{narrow}: 200}}}}
            subscriptions: {{n: {{t: {{{wide}: 30}}}}}}
        """)
        rows = {row.topic: row.values for row in resolve(config, _graph()).subscriptions}
        assert rows["t"]["take_timeout_ms"] == 30
        assert rows["u"]["take_timeout_ms"] == 200


class TestUserFacingNames:
    def test_reference_file_lists_maxwait_ms(self, tmp_path):
        scaffold(tmp_path, _graph())
        content = (tmp_path / REFERENCE_FILENAME).read_text()
        assert "#   maxwait_ms: 100\n" in content
        assert "#       maxwait_ms: 100\n" in content
        assert "take_timeout_ms:" not in content

    def test_interactive_writes_maxwait_ms_over_the_alias(self, tmp_path):
        sparse = tmp_path / "execution.yaml"
        sparse.write_text("schema_version: 1\ndefaults: {take_timeout_ms: 40}\n")
        prompts: list[str] = []
        answers = iter(["", "75", "q"])

        def fake_input(prompt: str) -> str:
            prompts.append(prompt)
            return next(answers)

        run_interactive(sparse, _graph(), input_fn=fake_input, print_fn=lambda *a, **k: None)
        assert prompts[1].startswith("  maxwait_ms [40] — ")
        assert yaml.safe_load(sparse.read_text())["defaults"] == {"maxwait_ms": 75}


class TestConfigureEnvLine:
    """`colcon lf-configure --show` reports the env override the binding applies."""

    _TAIL = (
        " overrides maxwait_ms at runtime for legacy subscriptions only. An admitted "
        "correspondence edge keeps its generated timeout, so set maxwait_ms in "
        "execution.yaml and regenerate to change it"
    )

    def _line(self, environ):
        from colcon_lf.verb.configure import maxwait_env_line

        return maxwait_env_line(environ)

    def test_unset_and_empty_variables_report_nothing(self):
        assert self._line({}) is None
        assert self._line({"ROS2_LF_MAXWAIT_MS": "", "ROS2_LF_TAKE_TIMEOUT_MS": ""}) is None

    def test_each_name_alone(self):
        assert self._line({"ROS2_LF_MAXWAIT_MS": "250"}) == "env ROS2_LF_MAXWAIT_MS=250" + self._TAIL
        assert self._line({"ROS2_LF_TAKE_TIMEOUT_MS": "250"}) == (
            "env ROS2_LF_TAKE_TIMEOUT_MS=250 (alias of ROS2_LF_MAXWAIT_MS)" + self._TAIL
        )
        # An empty primary counts as unset, so the alias applies.
        assert self._line({"ROS2_LF_MAXWAIT_MS": "", "ROS2_LF_TAKE_TIMEOUT_MS": "7"}) == (
            "env ROS2_LF_TAKE_TIMEOUT_MS=7 (alias of ROS2_LF_MAXWAIT_MS)" + self._TAIL
        )

    def test_the_same_value_applies(self):
        assert self._line({"ROS2_LF_MAXWAIT_MS": "250", "ROS2_LF_TAKE_TIMEOUT_MS": "250"}) == (
            "env ROS2_LF_MAXWAIT_MS=250 (and its alias ROS2_LF_TAKE_TIMEOUT_MS=250)" + self._TAIL
        )

    def test_a_disagreeing_pair_is_reported_as_refused(self):
        assert self._line({"ROS2_LF_MAXWAIT_MS": "200", "ROS2_LF_TAKE_TIMEOUT_MS": "100"}) == (
            "env ROS2_LF_MAXWAIT_MS=200 and its alias ROS2_LF_TAKE_TIMEOUT_MS=100 disagree: "
            "the binding refuses the maxwait override, so every subscription keeps its "
            "configured maxwait_ms"
        )
