#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Check that the CI shards of the colcon_lf suite (RCLLF_TEST_SHARD=k/N)
# partition it: every collected test is in exactly one shard, none is
# dropped. Collects the full suite once and each shard once (collection only, no
# test runs) and prints the counts. Exit 1 on any test missing or duplicated.
#
# Usage (from tools/colcon_lf, ROS environment sourced): ../../scripts/check_test_shards.py N

import os
import subprocess
import sys


def _collect(shard=None):
    env = dict(os.environ)
    env.pop("RCLLF_TEST_SHARD", None)
    if shard:
        env["RCLLF_TEST_SHARD"] = shard
    out = subprocess.run(
        [sys.executable, "-m", "pytest", "tests/", "--collect-only", "-q", "-p", "no:cacheprovider"],
        env=env, capture_output=True, text=True,
    )
    if out.returncode != 0:
        sys.stderr.write(out.stdout + out.stderr)
        sys.exit(f"collection failed for shard {shard or 'all'} (rc={out.returncode})")
    return [line for line in out.stdout.splitlines() if "::" in line and not line.startswith(" ")]


def main(count):
    full = _collect()
    seen = {}
    for k in range(1, count + 1):
        ids = _collect(f"{k}/{count}")
        print(f"shard {k}/{count}: {len(ids)} tests")
        for nodeid in ids:
            seen.setdefault(nodeid, []).append(k)
    total = sum(len(v) for v in seen.values())
    print(f"full suite: {len(full)} tests; sum of shards: {total}")
    missing = sorted(set(full) - set(seen))
    extra = sorted(set(seen) - set(full))
    duplicated = sorted(n for n, ks in seen.items() if len(ks) > 1)
    for label, ids in (("missing from every shard", missing), ("not in the full suite", extra),
                       ("in more than one shard", duplicated)):
        for nodeid in ids:
            print(f"ERROR {label}: {nodeid}")
    return 1 if (missing or extra or duplicated or not full) else 0


if __name__ == "__main__":
    sys.exit(main(int(sys.argv[1])))
