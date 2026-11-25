#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Enforce the docs punctuation rule (CONTRIBUTING.md, "Docs style guide"):
# no em or en dashes in prose. Fenced code blocks and inline code spans are
# skipped. Exit 1 on any violation, listing file:line.
#
# --advisory also lists prose lines containing semicolons. They are not
# failures, since a semicolon check cannot tell prose from syntax reliably.
#
# Usage: scripts/check_punctuation.py [--advisory] [paths...]
#   default paths: docs/ README.md CONTRIBUTING.md

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PATHS = ["docs", "README.md", "CONTRIBUTING.md"]
DASHES = re.compile("[–—]")
INLINE_CODE = re.compile(r"(`+).*?\1")
FENCE = re.compile(r"^\s*(```|~~~)")

def prose_lines(path):
    """Yield (lineno, text) for lines outside fenced code, inline code removed."""
    in_fence = False
    for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if FENCE.match(line):
            in_fence = not in_fence
            continue
        if not in_fence:
            yield n, INLINE_CODE.sub("", line)


def markdown_files(paths):
    for p in paths:
        p = (ROOT / p).resolve()
        files = sorted(p.rglob("*.md")) if p.is_dir() else [p]
        yield from files


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--advisory", action="store_true",
                    help="also list prose lines containing semicolons")
    ap.add_argument("paths", nargs="*", default=DEFAULT_PATHS)
    args = ap.parse_args()

    failures = 0
    for f in markdown_files(args.paths):
        rel = f.relative_to(ROOT).as_posix()
        for n, text in prose_lines(f):
            if DASHES.search(text):
                failures += 1
                print(f"{rel}:{n}: em/en dash in prose: {text.strip()}")
            elif args.advisory and ";" in re.sub(r"&\w+;|&#\d+;", "", text):
                print(f"{rel}:{n}: advisory semicolon: {text.strip()}")
    if failures:
        print(f"{failures} punctuation violation(s); see CONTRIBUTING.md "
              "'Docs style guide' (Punctuation)", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
