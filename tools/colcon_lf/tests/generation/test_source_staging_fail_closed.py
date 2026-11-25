# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

"""Regression: source staging must fail closed, not emit a placeholder.

Before this fix, a non-component node whose ``source_file`` did not resolve at
generation time got a 3-line placeholder .cpp:

    #define main lf_unused_main_0
    // PLACEHOLDER: source file '/workspace/.../publisher_node.cpp' not found ...

The staged copy is what supplies the node class's constructor to the generated
wrapper, so the placeholder produced a package that compiled (the class is
declared by the exported header) and then failed to link with
``undefined reference to pubsub_example::PublisherNode::PublisherNode(...)`` —
in a different package, minutes later, naming the constructor rather than the
missing file.

The usual trigger is a ``system.json`` extracted inside the dev container
(absolute ``/workspace/...`` paths) but generated from the host.

These tests must stay hermetic. The unresolvable path is built under
``tmp_path`` rather than written as a literal ``/workspace/...``, because the
guard under test is a plain filesystem check: a literal container path is a real
file inside the dev container (compose mounts the repo at ``$WORKSPACE``), where
staging then succeeds and the ``pytest.raises`` assertions fail. CI runs this
suite in the dev image but with the checkout under ``/__w/...``, so it never saw
that failure. These four tests are build-independent and must pass everywhere;
the clang-dependent tests elsewhere in the suite need a *built* workspace and
fail without one, on a host or in a fresh container alike.
"""

from __future__ import annotations

import pytest

from colcon_lf.node_analysis.models import NodeInfo
from colcon_lf.verb.generate import SourceStagingError, _stage_node_source_or_fail


def _unresolvable_container_path(tmp_path) -> str:
    """Return a container-looking source path that cannot resolve anywhere.

    Mirrors the shape of the real trigger,
    ``/workspace/packages/src/pubsub_example/src/publisher_node.cpp``, but rooted
    in a fresh ``tmp_path`` so it is absent by construction in every environment
    rather than only where ``/workspace`` happens not to be mounted.
    """
    path = (tmp_path / "workspace" / "packages" / "src" / "pubsub_example"
            / "src" / "publisher_node.cpp")
    assert not path.exists(), f"precondition: {path} must not exist"
    return str(path)


def _node(source_file: str | None) -> NodeInfo:
    return NodeInfo(
        name="publisher_node",
        class_name="PublisherNode",
        package="pubsub_example",
        source_file=source_file,
        header_file="pubsub_example/publisher_node.hpp",
    )


class TestStageNodeSourceOrFail:
    def test_reads_real_source(self, tmp_path):
        src = tmp_path / "publisher_node.cpp"
        src.write_text("// real source\nint main() { return 0; }\n", encoding="utf-8")
        assert _stage_node_source_or_fail(str(src), _node(str(src))) == (
            "// real source\nint main() { return 0; }\n")

    def test_unresolvable_container_path_raises(self, tmp_path):
        """The system_bringup failure mode: container path, host generation."""
        missing = _unresolvable_container_path(tmp_path)
        with pytest.raises(SourceStagingError) as exc:
            _stage_node_source_or_fail(missing, _node(missing))
        msg = str(exc.value)
        # The message must name the unresolved file — the placeholder's failure
        # mode was that the eventual error named the constructor instead.
        assert missing in msg
        assert "PublisherNode" in msg
        assert "pubsub_example" in msg

    def test_no_source_file_recorded_raises(self):
        with pytest.raises(SourceStagingError):
            _stage_node_source_or_fail("", _node(None))

    def test_message_explains_the_container_path_cause(self, tmp_path):
        missing = _unresolvable_container_path(tmp_path)
        with pytest.raises(SourceStagingError) as exc:
            _stage_node_source_or_fail(missing, _node(missing))
        msg = str(exc.value)
        assert "container" in msg.lower()
        assert "undefined reference" in msg
