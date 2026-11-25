# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Resolved configuration model.

A versioned, typed, provenance-tagged snapshot of every temporal-contract
property (topics, QoS, timer periods, parameter values, execution-policy
overrides) that a generated system actually runs with, plus a content hash
and a fail-closed gate for the properties that would otherwise silently
default.

Public surface:

- :mod:`colcon_lf.resolved_model.model` — ``Record``, ``ResolvedModel``,
  ``canonical_bytes``, ``model_hash``, ``to_json``, ``from_json``.
- :mod:`colcon_lf.resolved_model.resolve` — builds a ``ResolvedModel`` from a
  ``system.json``, launch parameter files, and the resolved exec-config.
- ``python3 -m colcon_lf.resolved_model`` — ``resolve``/``check`` CLI.
"""
