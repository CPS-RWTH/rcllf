# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Record one topic's publish and take events from a real run as a test fixture.

Usage: python3 extract.py <run_dir> <topic> <out.json.gz>

Keeps the node, publisher and subscription inits of the topic, every
``rcl_publish`` and ``rmw_publish`` of its publishers, every ``rmw_take`` of
its readers and the match probe's ``writer_matched`` and ``reader_matched``
events of both, so ``collect_drop_counts`` sees the topic exactly as in the run.
"""

import gzip
import json
import sys

from lf_eval.metrics.drops import DROP_EVENTS
from lf_eval.reader import read_trace


def main(run_dir: str, topic: str, out: str) -> None:
    events = list(read_trace(run_dir, event_names=set(DROP_EVENTS) | {"rcl_node_init"}))
    inits = [e for e in events if e.name == "rcl_publisher_init" and e.get("topic_name") == topic]
    pubs = {(e.pid, e.get("rmw_publisher_handle")) for e in inits}
    rcl_pubs = {(e.pid, e.get("publisher_handle")) for e in inits}
    subs = {(e.pid, e.get("rmw_subscription_handle")) for e in events
            if e.name == "rcl_subscription_init" and e.get("topic_name") == topic}
    nodes = {(e.pid, e.get("node_handle")) for e in events
             if e.name == "rcl_subscription_init" and e.get("topic_name") == topic}
    keep = []
    for e in events:
        if (e.name in ("rcl_publisher_init", "rcl_subscription_init") and e.get("topic_name") == topic
                or e.name == "rcl_node_init" and (e.pid, e.get("node_handle")) in nodes
                or e.name == "rmw_publish" and (e.pid, e.get("rmw_publisher_handle")) in pubs
                or e.name == "rcl_publish" and (e.pid, e.get("publisher_handle")) in rcl_pubs
                or e.name in ("rmw_take", "reader_matched")
                and (e.pid, e.get("rmw_subscription_handle")) in subs
                or e.name == "writer_matched" and (e.pid, e.get("rmw_publisher_handle")) in pubs):
            keep.append([e.name, e.timestamp_ns, e.pid, dict(e.fields)])
    with gzip.open(out, "wt") as fh:
        json.dump({"source_run": run_dir.rstrip("/").split("/")[-1], "topic": topic, "events": keep}, fh)
    print(f"{len(keep)} events")


if __name__ == "__main__":
    main(*sys.argv[1:4])
