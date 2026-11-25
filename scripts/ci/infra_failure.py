#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Decide whether a failed workflow run failed only on infrastructure.
#
# Input: the JSON of `gh run view <id> --json jobs` (a file, or - for stdin).
# Exit 0 and print "infra" when every failed job failed only in an
# infrastructure step: the runner could not set up the job, pull the container
# image, or upload an artifact. Such a run is worth one automatic rerun of its
# failed jobs (.github/workflows/ci-retry.yml). Exit 1 and print "code" with the
# reasons otherwise; a run that failed in a test, build or lint step is never
# rerun automatically.
#
# Census of the CI runs from 2026-11-05 to 2026-11-12: three runs failed at
# "Initialize containers" (ghcr permission_denied), one at "Upload test results"
# (artifact service returned HTML). It also calls two further runs infra: their
# branch pinned an image tag that was never pushed ("not found").
# Their rerun fails the same way; the rerun is attempted once only, so that
# costs one wasted job, not a loop.
#
#   python3 scripts/ci/infra_failure.py jobs.json

import json
import sys

# Steps that run no project code. "How to reproduce locally" only echoes a hint
# and is marked failed when an earlier step failed; on its own it is neutral.
INFRA_STEPS = ("Set up job", "Initialize containers", "Stop containers",
               "Complete job", "How to reproduce locally")
INFRA_PREFIXES = ("Upload ", "Post Upload ", "Set up Docker Buildx",
                  "Log in to GitHub Container Registry")
# Jobs that only aggregate other jobs' results: they fail because another job
# failed, and a rerun of the failed jobs reruns them too.
AGGREGATORS = ("build-and-test",)


def is_infra_step(name):
    return name in INFRA_STEPS or name.startswith(INFRA_PREFIXES)


def classify(jobs):
    """Return (infra, reasons) for the jobs list of `gh run view --json jobs`."""
    reasons = []
    infra_jobs = 0
    for job in jobs:
        if job.get("conclusion") != "failure":
            continue
        name = job.get("name", "?")
        if name in AGGREGATORS:
            continue
        failed = [s["name"] for s in job.get("steps") or []
                  if s.get("conclusion") == "failure"]
        if not failed:
            reasons.append(f"{name}: failed with no failed step (timeout or cancel?)")
            continue
        code = [s for s in failed if not is_infra_step(s)]
        if code:
            reasons.append(f"{name}: failed in {', '.join(code)}")
        else:
            infra_jobs += 1
            reasons.append(f"{name}: infra failure in {', '.join(failed)}")
    infra = infra_jobs > 0 and all("infra failure" in r for r in reasons)
    return infra, reasons


def main(argv):
    if len(argv) != 2:
        sys.exit("usage: infra_failure.py JOBS_JSON|-")
    f = sys.stdin if argv[1] == "-" else open(argv[1], encoding="utf-8")
    with f:
        data = json.load(f)
    infra, reasons = classify(data["jobs"] if isinstance(data, dict) else data)
    print("infra" if infra else "code")
    for r in reasons:
        print(f"  {r}")
    return 0 if infra else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
