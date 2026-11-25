# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Bounded Nearest-to-Anchor (BNA) synchronizer oracle and comparator.

``oracle``
    The standalone reference of the BNA rules
    (``docs/features/bna-synchronizer.md``).
``corpus``
    The ``.bna`` text corpus format, its JSONL form and seeded random corpora.
``compare``
    First-difference comparison of an implementation's outputs with the oracle's.
``schedule``
    The admit stream an end-to-end run must produce, from its publish schedule only.
"""
