# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""``ros2 lf scan`` (run spec slice R7).

The pubsub_example expectations below were written by reading the package by
hand, not from scan output:

- ``launch/publisher_node.launch.py``: one ``Node(package='pubsub_example',
  executable='publisher_node', name='publisher_node')``.
- ``launch/subscriber_node.launch.py``: one ``Node(... executable=
  'subscriber_node', name='subscriber_node')``.
- ``launch/bringup.launch.py``: no node of its own; includes the two files
  above, in that order.
- ``launch/long_names.launch.py``: the same two executables, named
  ``publisher_node_with_a_name_of_forty_or_more_chars`` and
  ``subscriber_node_with_a_name_of_forty_or_more_chars``.
- ``src/publisher_node.cpp`` calls ``create_publisher<custom_interfaces::msg::
  SensorData>("sensor_data", 10)`` and ``src/subscriber_node.cpp``
  ``create_subscription<...SensorData>("sensor_data", ...)``. With no
  namespace and no remapping in any launch file the topic resolves to
  ``/sensor_data``.
"""

from __future__ import annotations

import argparse
import json
import re
from html.parser import HTMLParser

import pytest

from colcon_lf.scan import add_scan_arguments, render_html, render_text, scan_command

PUB, SUB = "publisher_node", "subscriber_node"
LPUB = "publisher_node_with_a_name_of_forty_or_more_chars"
LSUB = "subscriber_node_with_a_name_of_forty_or_more_chars"
TOPIC = "/sensor_data"
TYPE = "custom_interfaces::msg::SensorData"

# launch file -> (launched (executable, name) in launch order,
#                 {topic: (publishers, subscribers)})
EXPECTED = {
    "bringup.launch.py": ([("publisher_node", PUB), ("subscriber_node", SUB)],
                          {TOPIC: ([PUB], [SUB])}),
    "long_names.launch.py": ([("publisher_node", LPUB), ("subscriber_node", LSUB)],
                             {TOPIC: ([LPUB], [LSUB])}),
    "publisher_node.launch.py": ([("publisher_node", PUB)], {TOPIC: ([PUB], [])}),
    "subscriber_node.launch.py": ([("subscriber_node", SUB)], {TOPIC: ([], [SUB])}),
}


def _container_only():
    pytest.importorskip("launch_ros", reason="launch_ros not available — run inside Docker")
    pytest.importorskip("clang.cindex", reason="libclang not available — run inside Docker")
    from .ros_env import requires_ros_headers
    mark = requires_ros_headers("the pubsub_example scan tests")
    if mark.args[0]:
        pytest.skip(mark.kwargs["reason"])


def scan_mismatches(entries: dict, expected: dict = EXPECTED) -> list[str]:
    """Every difference between the scan entries and the hand-written list."""
    bad = []
    if sorted(entries) != sorted(expected):
        bad.append(f"launch files {sorted(entries)} != hand list {sorted(expected)}")
    for lf, (launched, topics) in expected.items():
        e = entries.get(lf)
        if e is None:
            continue
        got_launched = [(n["executable"], n["name"]) for n in e["launched"]]
        if got_launched != launched:
            bad.append(f"{lf}: launched {got_launched} != hand list {launched}")
        got_nodes = sorted(n["key"] for n in e["nodes"])
        if got_nodes != sorted(name for _, name in launched):
            bad.append(f"{lf}: nodes {got_nodes} != hand list "
                       f"{sorted(name for _, name in launched)}")
        got_topics = {t["topic"]: (t["publishers"], t["subscribers"]) for t in e["topics"]}
        if got_topics != topics:
            bad.append(f"{lf}: topics {got_topics} != hand list {topics}")
        for t in e["topics"]:
            if t["message_type"] != TYPE:
                bad.append(f"{lf}: {t['topic']} type {t['message_type']} != {TYPE}")
    return bad


@pytest.fixture(scope="module")
def pubsub_scan(packages_root):
    _container_only()
    from colcon_lf.scan import scan_workspace

    doc = scan_workspace(packages_root, package="pubsub_example")
    (pkg,) = doc["packages"]
    return doc, {e["launch_file"]: e for e in pkg["launch_files"]}


def test_scan_pubsub_lists_the_hand_read_nodes_and_topics(pubsub_scan) -> None:
    doc, entries = pubsub_scan
    assert doc["schema"] == "rcllf.scan/1"
    assert scan_mismatches(entries) == []
    for lf, e in entries.items():
        # pubsub_example is the example `colcon lf` converts in CI
        assert e["convertible"] == "yes", (lf, e["reasons"])
        assert e["reasons"] == [] and e["not_analyzed"] == [] and e["launch_errors"] == []
    # each node's own view agrees with the topic table
    e = entries["bringup.launch.py"]
    nodes = {n["key"]: n for n in e["nodes"]}
    assert nodes[PUB]["publishes"] == [TOPIC] and nodes[PUB]["subscribes"] == []
    assert nodes[SUB]["subscribes"] == [TOPIC] and nodes[SUB]["publishes"] == []
    # publisher_node.cpp has one create_wall_timer(500ms, ...); subscriber_node.cpp none
    assert (nodes[PUB]["timers"], nodes[SUB]["timers"]) == (1, 0)
    assert doc["summary"] == {"packages": 1, "launch_files": 4,
                              "convertible": {"yes": 4, "warn": 0, "no": 0}}


def test_dropped_launch_node_is_named(pubsub_scan) -> None:
    """Falsification kept in the suite: a scan missing one launch-file node is
    caught, with the node named."""
    _, entries = pubsub_scan
    broken = json.loads(json.dumps(entries))
    e = broken["bringup.launch.py"]
    e["launched"] = [n for n in e["launched"] if n["name"] != SUB]
    e["nodes"] = [n for n in e["nodes"] if n["key"] != SUB]
    found = scan_mismatches(broken)
    assert found == [
        f"bringup.launch.py: launched [('publisher_node', '{PUB}')] != hand list "
        f"[('publisher_node', '{PUB}'), ('subscriber_node', '{SUB}')]",
        f"bringup.launch.py: nodes ['{PUB}'] != hand list ['{PUB}', '{SUB}']",
    ], found


def test_scan_cli_json_and_html(packages_root, tmp_path, capsys) -> None:
    _container_only()
    parser = argparse.ArgumentParser()
    add_scan_arguments(parser)
    html_out = tmp_path / "scan.html"
    args = parser.parse_args(["--workspace", str(packages_root), "--package", "pubsub_example",
                              "--launch-file", "bringup.launch.py", "--json",
                              "--html", str(html_out)])
    assert scan_command(args) == 0
    doc = json.loads(capsys.readouterr().out)
    (pkg,) = doc["packages"]
    assert scan_mismatches({e["launch_file"]: e for e in pkg["launch_files"]},
                           {"bringup.launch.py": EXPECTED["bringup.launch.py"]}) == []
    page = html_out.read_text()
    for name in (PUB, SUB, TOPIC, TYPE):
        assert f"<code>{name}</code>" in page or f"<td>{name}</td>" in page, name


def test_scan_cli_refuses_unknown_package(packages_root, capsys) -> None:
    pytest.importorskip("colcon_core", reason="colcon_core not available — run inside Docker")
    parser = argparse.ArgumentParser()
    add_scan_arguments(parser)
    args = parser.parse_args(["--workspace", str(packages_root), "--package", "no_such_pkg"])
    assert scan_command(args) == 2
    assert "package 'no_such_pkg' not found" in capsys.readouterr().err


# ── HTML: self-contained and escaped (host-runnable) ──────────────────────


class _Tags(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.tags, self.urls, self.text = set(), [], []

    def handle_starttag(self, tag, attrs):
        self.tags.add(tag)
        self.urls += [v for k, v in attrs if k in ("src", "href", "action")]

    def handle_data(self, data):
        self.text.append(data)


def test_scan_html_escapes_every_string() -> None:
    evil = '<script>alert(1)</script>&"\'<img src=x onerror=alert(1)>'
    entry = {"package": evil, "launch_file": evil, "path": evil, "convertible": "no",
             "reasons": [evil], "not_analyzed": [evil], "launch_errors": [],
             "launched": [{"package": evil, "executable": evil, "name": evil,
                           "namespace": evil, "is_component": False,
                           "condition_result": None}],
             "nodes": [{"key": evil, "package": evil, "class_name": evil, "is_component": False,
                        "publishes": [evil], "subscribes": [evil], "timers": 0}],
             "topics": [{"topic": evil, "message_type": evil, "publishers": [evil],
                         "subscribers": [evil]}],
             "services": [{"service": evil, "service_type": evil, "server": evil,
                           "clients": [evil]}],
             "actions": [{"action": evil, "action_type": evil, "server": evil,
                          "clients": [evil]}],
             "diagnostics": [{"code": evil, "severity": evil, "node": evil, "topic": evil,
                              "message": evil}]}
    doc = {"schema": "rcllf.scan/1", "generated_at": evil, "workspace": evil,
           "packages": [{"name": evil, "path": evil, "launch_files": [entry]}],
           "summary": {"packages": 1, "launch_files": 1,
                       "convertible": {"yes": 0, "warn": 0, "no": 1}}}
    page = render_html(doc)
    p = _Tags()
    p.feed(page)
    assert not {"script", "img", "link", "iframe"} & p.tags, p.tags
    assert p.urls == []
    assert "@import" not in page and not re.search(r"url\(", page)
    # escaped, not dropped: the string comes back verbatim as text
    assert sum(1 for t in p.text if evil in t) >= 27
    # every listed kind reaches both views
    text = render_text(doc)
    for word in ("launched ", "service ", "action ", "timers 0", "not analysed: "):
        assert word in text, word
    for th in ("<th>launched</th>", "<th>service</th>", "<th>action</th>", "<th>timers</th>"):
        assert th in page, th
