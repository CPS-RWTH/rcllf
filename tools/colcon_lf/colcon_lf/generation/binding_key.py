# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Binding-key synthesis — the single home for the per-instance key scheme.

A **binding key** is the string that identifies one node *instance* across the
whole generated artifact set:

* the LF reactor definition name (``lf_generator.py``),
* the ``ros2_lf_bind_node(self, "<key>")`` string baked into the startup body,
* ``RidEntry.node_class`` / the ``RID_<key>_<callback>`` macro
  (``binding_header_emitter.py``),
* the ``ros2_lf_register_node("<key>", ...)`` factory registration and the
  per-instance component-list entry (``verb/generate.py``),
* the ``node_class`` field of ``exec_config.json`` (``exec_config/resolve.py``).

All of those derive from :func:`build_binding_keys` in one generator run, so
they cannot drift.

Rule:

* a class with exactly **one** instance keeps its plain class name (byte-identical
  output for every single-instance system — the regression guard);
* a class with **N >= 2** instances gets ``f"{class_name}__{suffix}"`` where
  *suffix* is the normalized node key;
* candidates that collide (normalization is lossy) get a deterministic
  ``_h<6 hex of SHA-1 of the raw node key>`` disambiguator;
* the final key set is asserted unique, ``ValueError`` naming the raw node keys
  otherwise.

Because the keys are derived from ``system.json`` content only (class names and
node keys), they are stable across regenerations for a fixed topology.  Adding
an instance of a class re-keys *that class's* generated rows; user-facing
``execution.yaml`` keys are node names and are unaffected.
"""

from __future__ import annotations

import hashlib
import re

__all__ = [
    "BINDING_KEY_MARKER",
    "build_binding_keys",
    "normalize_node_key",
]

# Separator between the class name and the per-instance suffix.  Never appears
# in a normalized class name (``_`` runs are collapsed), so its presence is an
# unambiguous "this is a synthesized key" marker for the C++ side.
BINDING_KEY_MARKER = "__"

_INVALID_KEY_CHARS = re.compile(r"[^A-Za-z0-9_]")
_UNDERSCORE_RUN = re.compile(r"_+")


def normalize_node_key(node_key: str) -> str:
    """Normalize a model node key into a binding-key suffix.

    Strips leading slashes, replaces every character outside ``[A-Za-z0-9_]``
    with ``_``, and collapses runs of ``_``.  Lossy by construction (``/a/b_c``
    and ``/a_b/c`` both normalize to ``a_b_c``) — see
    :func:`build_binding_keys` for the collision handling that makes it safe.
    """
    stripped = node_key.lstrip("/")
    sanitized = _INVALID_KEY_CHARS.sub("_", stripped)
    collapsed = _UNDERSCORE_RUN.sub("_", sanitized).strip("_")
    return collapsed or "unnamed"


def _hash_suffix(raw_node_key: str) -> str:
    """Deterministic disambiguator: ``_h`` + first 6 hex of SHA-1 of the raw key.

    Hashes the **raw** node key (not the normalized form) because that is the
    only value guaranteed distinct between two colliding candidates.
    """
    digest = hashlib.sha1(raw_node_key.encode("utf-8")).hexdigest()
    return f"_h{digest[:6]}"


def build_binding_keys(nodes: dict[str, object]) -> dict[str, str]:
    """Map every node key in *nodes* to its binding key.

    Parameters
    ----------
    nodes:
        ``{node_key: node_info}`` — anything with a ``class_name`` attribute
        (in practice ``SystemGraph.nodes``).  Iteration order is preserved.

    Returns
    -------
    dict
        ``{node_key: binding_key}``.

    Raises
    ------
    ValueError
        If two distinct node keys still produce the same binding key after
        hash disambiguation (a synthesis bug), naming the raw node keys.
    """
    # Group node keys by class name, preserving insertion order.
    by_class: dict[str, list[str]] = {}
    for node_key, node_info in nodes.items():
        class_name = getattr(node_info, "class_name", "") or ""
        by_class.setdefault(class_name, []).append(node_key)

    result: dict[str, str] = {}
    for class_name, node_keys in by_class.items():
        if len(node_keys) == 1:
            # Single instance: plain class name (byte-identical to pre-change
            # output for every single-instance system).
            result[node_keys[0]] = class_name
            continue

        # N >= 2: synthesize per-instance keys.
        candidates: dict[str, str] = {
            node_key: f"{class_name}{BINDING_KEY_MARKER}{normalize_node_key(node_key)}"
            for node_key in node_keys
        }
        # Any candidate claimed by more than one node key is disambiguated for
        # *all* of its claimants, so the outcome never depends on ordering.
        counts: dict[str, int] = {}
        for candidate in candidates.values():
            counts[candidate] = counts.get(candidate, 0) + 1
        for node_key, candidate in candidates.items():
            if counts[candidate] > 1:
                candidates[node_key] = candidate + _hash_suffix(node_key)
        result.update(candidates)

    # Final uniqueness assertion across the whole system (a class name could in
    # principle collide with another class's synthesized key).
    owners: dict[str, list[str]] = {}
    for node_key, binding_key in result.items():
        owners.setdefault(binding_key, []).append(node_key)
    duplicates = {key: keys for key, keys in owners.items() if len(keys) > 1}
    if duplicates:
        detail = "; ".join(
            f"{key!r} <- {', '.join(repr(k) for k in keys)}"
            for key, keys in sorted(duplicates.items())
        )
        raise ValueError(
            "binding-key synthesis produced duplicate keys (this is a generator "
            f"bug, not a user error): {detail}"
        )
    return result
