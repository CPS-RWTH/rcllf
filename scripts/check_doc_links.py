#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Check that every link in the given Markdown pages resolves.

Checked per page:
  - relative Markdown links ``[text](path#anchor)``: the path exists, and the
    anchor is a heading slug or an explicit ``<a id>`` / ``{#id}`` / ``{: #id }`` of the target,
  - in-page links ``[text](#anchor)``,
  - backticked release citations ``component:path``: some committed table copy
    listed in a release set ends with ``/path`` and exists. A tree without release
    sets has none, so every such citation is reported.

External ``http(s)``/``mailto`` links are not fetched. Exit 1 and one line per
broken link, naming page, line and target.

Usage::

    python3 scripts/check_doc_links.py docs/features/support-matrix.md
"""

from __future__ import annotations

import argparse
import re
import sys
import unicodedata
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
RELEASE_SETS: tuple[Path, ...] = ()

_LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
_HEADING = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")
# An attribute list sets an id as {#id} or, with attr_list, {: #id .class }.
_ATTR_ID = r"\{:?[^{}]*?#([\w-]+)[^{}]*\}"
_EXPLICIT_ID = re.compile(r"<a\s+(?:id|name)=\"([^\"]+)\"|" + _ATTR_ID)
_RELEASE_REF = re.compile(r"`((?:final-\d{4}-\d{2}-\d{2}|rq4-[a-z]+-v\d+)):([^`\s]+)`")
_COMMITTED = re.compile(r"^\s*committed:\s*(\S+)\s*$")
_COMPONENT = re.compile(r"^- component:\s*(\S+)\s*$")


def page_text(page: Path) -> str:
    """The Markdown the site serves for ``page``."""
    return page.read_text()


def slugify(text: str) -> str:
    """Python-Markdown ``toc`` slug of a heading (the mkdocs default)."""
    text = re.sub(r"`([^`]*)`", r"\1", text)
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"<[^>]+>", "", text)
    text = re.sub(_ATTR_ID, "", text)
    text = unicodedata.normalize("NFKD", text).encode("ascii", "ignore").decode("ascii")
    text = re.sub(r"[^\w\s-]", "", text).strip().lower()
    return re.sub(r"[-\s]+", "-", text)


def anchors(page: Path) -> set[str]:
    """Every anchor a link may target in ``page``."""
    found: set[str] = set()
    seen: dict[str, int] = {}
    in_fence = False
    for line in page_text(page).splitlines():
        if line.lstrip().startswith(("```", "~~~")):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        for match in _EXPLICIT_ID.finditer(line):
            found.add(match.group(1) or match.group(2))
        heading = _HEADING.match(line)
        if heading:
            explicit = re.search(_ATTR_ID, heading.group(2))
            slug = explicit.group(1) if explicit else slugify(heading.group(2))
            if slug in seen:
                seen[slug] += 1
                slug = f"{slug}_{seen[slug]}"
            else:
                seen[slug] = 0
            found.add(slug)
    return found


def committed_copies(release_set: Path) -> list[tuple[str, str]]:
    """(component, committed path) for every ``committed:`` copy of the release set."""
    if not release_set.is_file():
        return []
    copies: list[tuple[str, str]] = []
    component = ""
    for line in release_set.read_text().splitlines():
        if m := _COMPONENT.match(line):
            component = m.group(1)
        elif m := _COMMITTED.match(line):
            copies.append((component, m.group(1)))
    return copies


def check_page(page: Path, copies: list[tuple[str, str]]) -> list[str]:
    problems: list[str] = []
    cache: dict[Path, set[str]] = {}
    in_fence = False
    for lineno, line in enumerate(page_text(page).splitlines(), start=1):
        if line.lstrip().startswith(("```", "~~~")):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        for match in _LINK.finditer(line):
            target = match.group(1)
            if re.match(r"^[a-z]+:", target):
                continue
            path_part, _, anchor = target.partition("#")
            dest = page if not path_part else (page.parent / path_part).resolve()
            where = f"{page.relative_to(REPO)}:{lineno}"
            if not dest.exists():
                problems.append(f"{where}: link target '{target}' does not exist ({dest})")
                continue
            if anchor:
                if dest.is_dir() or dest.suffix != ".md":
                    problems.append(f"{where}: anchor '#{anchor}' on a non-Markdown target '{target}'")
                    continue
                if dest not in cache:
                    cache[dest] = anchors(dest)
                if anchor not in cache[dest]:
                    problems.append(f"{where}: anchor '#{anchor}' not found in {dest.relative_to(REPO)}")
        for match in _RELEASE_REF.finditer(line):
            component, rel = match.groups()
            # The component and the path together name one copy: the right path
            # under another component is a broken citation.
            hits = [c for comp, c in copies
                    if comp == component and c.endswith("/" + rel) and (REPO / c).is_file()]
            if not hits:
                problems.append(
                    f"{page.relative_to(REPO)}:{lineno}: release citation '{component}:{rel}' "
                    f"matches no committed copy of component {component} in "
                    f"{', '.join(s.relative_to(REPO).as_posix() for s in RELEASE_SETS)}"
                )
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pages", nargs="+", type=Path)
    args = parser.parse_args(argv)
    copies = [c for s in RELEASE_SETS for c in committed_copies(s)]
    problems: list[str] = []
    links = 0
    for page in args.pages:
        page = page.resolve()
        if not page.is_file():
            problems.append(f"page {page} does not exist")
            continue
        text = page_text(page)
        links += len(_LINK.findall(text)) + len(_RELEASE_REF.findall(text))
        problems += check_page(page, copies)
    for problem in problems:
        print(f"broken link: {problem}")
    print(f"checked {links} links in {len(args.pages)} pages, {len(problems)} broken")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
