# Copyright 2026 rcllf Contributors
# SPDX-License-Identifier: Apache-2.0
"""The federated name pass keeps a cross-federate deferred join.

``colcon lf-generate --federate per-node --exec-config deferred.execution.yaml``
(the E5 deferred configuration) wrote the whole federation and
then failed: the pass that only computes the package name regenerated the
system as one process, every edge became local, and the local-edge check
refused ``join: deferred`` on a local edge. The pass is federated now, so the
deferred join, valid on every cross-federate edge, no longer fails the build.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import colcon_lf.verb.generate as gen

_FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "pubsub_system.json"
_DEFERRED = "schema_version: 1\ndefaults:\n  join: deferred\n"


def test_per_node_federation_with_join_deferred_generates(tmp_path, monkeypatch, capsys):
    system_dir = tmp_path / "lf-analysis" / "pubsub_example" / "bringup"
    system_dir.mkdir(parents=True)
    system_json = system_dir / "system.json"
    shutil.copy(_FIXTURE, system_json)
    cfg = tmp_path / "deferred.execution.yaml"
    cfg.write_text(_DEFERRED)
    seen = {}

    def federation_written(**kw):
        # Stands in for the federation emission, which succeeded in the E5
        # run; the name pass after it is what failed.
        seen["fmap"] = kw["fmap"]
        return 0

    monkeypatch.setattr(gen, "_generate_federated", federation_written)
    rc = gen._generate_one(system_json=system_json, output_dir=None, after_delay="5 msec",
                           federate="per-node", exec_config_arg=str(cfg))
    err = capsys.readouterr().err
    assert seen["fmap"].federate_names == ["publisher_node", "subscriber_node"], seen
    assert "Error generating federated artifacts" not in err, (
        f"the federated name pass refused the cross-federate deferred join as a local "
        f"edge: {err}")
    assert rc == 0, err
