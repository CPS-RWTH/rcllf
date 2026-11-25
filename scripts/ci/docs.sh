#!/bin/bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# The steps of the CI `docs` job, one subcommand per step. CI and
# the local gate run these same commands. Run from the repository
# root with a Python where pip may install (CI: actions/setup-python).
#
#   install       MkDocs and its theme, pinned in requirements-docs.txt (shared
#                 with docker/Dockerfile, so a new release cannot break --strict)
#   build         mkdocs build --strict
#   punctuation   no em or en dashes in prose (CONTRIBUTING.md)
#   scripts       no two packages build a library of the same name, and
#                 the scripts/tests suite
set -eo pipefail
extra_jobs=""

case "${1:-}" in
  install)
    pip install -r requirements-docs.txt
    ;;
  build)
    mkdocs build --strict
    ;;
  punctuation)
    python3 scripts/check_punctuation.py
    ;;
  scripts)
    python3 scripts/check_library_names.py
    python3 -m pytest -q scripts/tests
    ;;
  *)
    echo "usage: $0 install | build | punctuation${extra_jobs} | scripts" >&2
    exit 2
    ;;
esac
