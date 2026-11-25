# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Compare a converted sync run with oracle O1, per synchronizer instance.

    python -m lf_eval.sync_oracle --corpus corpora/f1.yaml --log run.log \\
        --o1 exact_q2=o1_exact_q2.jsonl [--o1 NODE=FILE ...] [--player corpus_player]

Prints ``<node>: compatible`` or ``<node>: <first report>`` per instance and
exits 1 when any instance has a report. A ``--o1 NODE=-`` entry checks only
the run side (domain and arrival order) and is meant for out-of-domain corpora
such as N1, whose O1 stream has no meaning. ``--stock`` compares a plain
``ros2 launch`` run (oracle O2): it has no drain or dispatch lines, so only
the config, tuple and classification checks apply.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

from lf_eval.sync_oracle.compare import (
    check_domain, compare, compare_classes, compare_config, compare_order, compare_tuples,
    parse_jsonl)
from lf_eval.sync_oracle.corpus import load_corpus
from lf_eval.sync_oracle.order import parse_run


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--corpus", required=True, type=Path)
    ap.add_argument("--log", required=True, type=Path, help="ros2 launch log of the converted run")
    ap.add_argument("--o1", action="append", required=True, metavar="NODE=FILE",
                    help="O1 stream (JSON lines) of one instance; FILE '-' checks the run only")
    ap.add_argument("--player", default="corpus_player")
    ap.add_argument("--stock", action="store_true",
                    help="stock run: skip the domain and arrival-order checks")
    args = ap.parse_args(argv)

    corpus = load_corpus(args.corpus)
    run = parse_run(args.log.read_text(encoding="utf-8", errors="replace"))
    player = run.sync.get(args.player, [])
    bad = 0
    for spec in args.o1:
        node, _, path = spec.partition("=")
        if path == "-":
            report = (check_domain(run, node, corpus, player)
                      or compare_order(run, node, corpus))
        elif args.stock:
            o1 = parse_jsonl(Path(path).read_text(encoding="utf-8"))
            mine = run.sync.get(node, [])
            policy = next((r.get("policy") for r in o1 if r.get("ev") == "config"), "")
            report = (compare_config(o1, mine, corpus, player)
                      or compare_tuples(o1, mine)
                      or compare_classes(corpus, o1, mine, policy))
        else:
            o1 = parse_jsonl(Path(path).read_text(encoding="utf-8"))
            report = compare(corpus, o1, run, node, player_node=args.player)
        print(f"{node}: {report or 'compatible'}")
        bad += report is not None
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
