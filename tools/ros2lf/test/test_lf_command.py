# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""`ros2 lf` lists and accepts only the verbs whose package is installed.

ros2cli's own ``add_subparsers_on_demand`` builds the parser; only the entry-point
lookup is replaced by the verb classes, so the test does not depend on how ros2lf
happens to be installed.
"""

from __future__ import annotations

import argparse

import pytest
import ros2cli.command

from ros2lf.command import lf
from ros2lf.verb.eval import EvalVerb
from ros2lf.verb.launch import LaunchVerb
from ros2lf.verb.list import ListVerb
from ros2lf.verb.report import ReportVerb
from ros2lf.verb.scan import ScanVerb

VERBS = {
    "eval": EvalVerb, "launch": LaunchVerb, "list": ListVerb,
    "report": ReportVerb, "scan": ScanVerb,
}


@pytest.fixture(autouse=True)
def _entry_points(monkeypatch):
    monkeypatch.setattr(ros2cli.command, "get_entry_points",
                        lambda group: {name: None for name in VERBS})
    monkeypatch.setattr(
        ros2cli.command, "get_command_extensions",
        lambda group, *, exclude_names=None: {
            name: cls() for name, cls in VERBS.items() if name not in (exclude_names or ())
        })


def _parser(argv):
    # ros2cli builds every parser with this formatter; the Commands section is a MutableString.
    parser = argparse.ArgumentParser(
        prog="ros2 lf", formatter_class=argparse.RawDescriptionHelpFormatter)
    lf.LfCommand().add_arguments(parser, "ros2 lf", argv=argv)
    return parser


def _listed(parser):
    """The verb names `ros2 lf --help` lists, one per line of the Commands section."""
    help_text = parser.format_help()
    return sorted(
        line.split()[0] for line in help_text.split("Commands:", 1)[1].splitlines()
        if line.strip() and line.split()[0] in VERBS
    )




@pytest.mark.parametrize("verb", ["eval", "report"])
def test_without_lf_eval_its_verbs_are_not_accepted(monkeypatch, capsys, verb):
    monkeypatch.setattr(lf, "_installed", lambda module: module != "lf_eval")
    parser = _parser([verb])
    with pytest.raises(SystemExit) as exit_info:
        parser.parse_args([verb])
    assert exit_info.value.code == 2
    assert f"invalid choice: '{verb}'" in capsys.readouterr().err


def test_without_lf_eval_eval_and_report_are_not_listed(monkeypatch):
    monkeypatch.setattr(lf, "_installed", lambda module: module != "lf_eval")
    assert _listed(_parser([])) == sorted(set(VERBS) - {"eval", "report"})


def test_with_every_package_every_verb_is_listed(monkeypatch):
    monkeypatch.setattr(lf, "_installed", lambda module: True)
    assert _listed(_parser([])) == sorted(VERBS)


def test_eval_and_report_say_experimental(monkeypatch):
    monkeypatch.setattr(lf, "_installed", lambda module: True)
    lines = {line.split()[0]: line for line in _parser([]).format_help().splitlines()
             if line.strip() and line.split()[0] in VERBS}
    assert "Experimental:" in lines["eval"]
    assert "Experimental:" in lines["report"]


def test_unavailable_verbs_checks_the_installed_modules(monkeypatch):
    monkeypatch.setattr(lf, "OPTIONAL_VERBS", {"a": "json", "b": "no_such_module_xyz"})
    assert lf.unavailable_verbs() == ["b"]


def test_a_broken_optional_package_is_an_error_not_a_hidden_verb(monkeypatch):
    # An ImportError while locating an installed package means it is broken; hiding
    # its verbs would turn that into a misleading "invalid choice".
    def broken(module):
        raise ImportError(f"broken {module} install")
    monkeypatch.setattr(lf.importlib.util, "find_spec", broken)
    with pytest.raises(ImportError, match="broken lf_eval install"):
        lf._installed("lf_eval")


def test_a_module_without_spec_counts_as_not_installed(monkeypatch):
    def no_spec(module):
        raise ValueError(f"{module}.__spec__ is None")
    monkeypatch.setattr(lf.importlib.util, "find_spec", no_spec)
    assert lf._installed("lf_eval") is False
