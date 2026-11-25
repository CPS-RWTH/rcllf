#!/usr/bin/env bash
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Fresh-clone gate. Clones the repo at one commit into an empty scratch
# directory and proves, from that clone alone, that the minimum slice builds and
# tests, that the getting-started tutorial runs with the documented output, and that
# the support matrix matches the feature registry and every link in it resolves.
# It pushes, tags and releases nothing.
#
# Usage:
#   scripts/fresh_clone_gate.sh [--repo URL|PATH|BUNDLE] [--commit REV] [--scratch DIR]
#                               [--image IMG] [--expect-private] [--steps a,b,...]
#
#   --repo            what to clone (default: https://github.com/CPS-RWTH/rcllf.git)
#   --commit          commit, tag or branch to check out (default: origin/main of --repo)
#   --scratch         empty or absent directory to clone into (default: mktemp -d)
#   --image           the image every container step runs, and whose manifest
#                     reachable-image reads (default: ghcr.io/cps-rwth/rcllf:dev, the
#                     name `docker compose build dev` gives its build). A release check
#                     passes the release's image: --image ghcr.io/cps-rwth/rcllf:0.1.0-deps
#   --expect-private  the repo and image are private before the public release: the
#                     image step passes on a local copy when an anonymous pull is
#                     refused, and says so in its line. Without it that is a FAIL.
#                     The container steps then use only that local copy, never a pull.
#   --steps           run only these steps (names as printed), for falsification runs;
#                     a name that is not a step of the cloned tree exits 2
#
# Output: one "PASS <step>: ..." or "FAIL <step>: ..." line per step, then a summary
# line. Exit 1 when any step fails. Step logs go to <scratch>/gate-logs/.
# Before the first container step an "image: ..." line names the image, its ID and
# its registry digests. Every container step runs that image ID through a Compose
# override (<scratch>/gate-compose.override.yml) that removes the dev service's build
# section and disables pulls, so a missing image fails the step instead of falling
# back to a source build or another tag. Without --expect-private the image is always
# pulled once, by its --image name, even when a copy is local, and a failed pull fails
# the step.
#
# Steps: clone, submodules, reachable-submodules, reachable-image, colcon-build,
# colcon-test, colcon-build-all, pytest-colcon_lf, pytest-lf_eval, tutorial-run,
# tutorial-output, doc-links, support-matrix.
set -uo pipefail

REPO_URL="https://github.com/CPS-RWTH/rcllf.git"
COMMIT=""
SCRATCH=""
IMAGE="ghcr.io/cps-rwth/rcllf:dev"
EXPECT_PRIVATE=0
ONLY=""
while [ $# -gt 0 ]; do
    case "$1" in
        --repo) REPO_URL="$2"; shift 2 ;;
        --commit) COMMIT="$2"; shift 2 ;;
        --scratch) SCRATCH="$2"; shift 2 ;;
        --image) IMAGE="$2"; shift 2 ;;
        --expect-private) EXPECT_PRIVATE=1; shift ;;
        --steps) ONLY=",$2,"; shift 2 ;;
        -h|--help)
            # The comment block from line 5 up to the first line of code.
            awk 'NR < 5 { next } /^# (>>>|<<<) dev-only/ { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"
            exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if [ -z "$SCRATCH" ]; then
    SCRATCH=$(mktemp -d -t rcllf-gate-XXXXXX)
elif [ -e "$SCRATCH" ] && [ -n "$(ls -A "$SCRATCH" 2>/dev/null)" ]; then
    echo "FAIL setup: scratch dir $SCRATCH is not empty" >&2
    exit 2
fi
mkdir -p "$SCRATCH"
SCRATCH=$(cd "$SCRATCH" && pwd)
CLONE="$SCRATCH/repo"
LOGS="$SCRATCH/gate-logs"
mkdir -p "$LOGS"
# A ROS domain of our own (100 to 199) so the tutorial run never talks to another run.
DOMAIN=${ROS_DOMAIN_ID:-$(( 100 + $(printf '%s' "$SCRATCH" | cksum | cut -d' ' -f1) % 100 ))}
PROJECT="rcllf-gate-$(printf '%s' "$SCRATCH" | cksum | cut -d' ' -f1)"
FAILED=()
PASSED=0

want() { [ -z "$ONLY" ] || [[ "$ONLY" == *",$1,"* ]]; }
pass() { echo "PASS $1: $2"; PASSED=$((PASSED + 1)); }
fail() { echo "FAIL $1: $2"; FAILED+=("$1"); }
# Last line of a log that says why a step failed, for the FAIL line. Patterns are
# tried in order, so a specific cause (a missing package) beats a generic "Failed".
why() {
    local log=$1 pat line; shift
    for pat in "$@"; do
        line=$(grep -E "$pat" "$log" 2>/dev/null | tail -1 | cut -c1-240)
        [ -n "$line" ] && { echo "$line"; return; }
    done
}

# The image of every container step, resolved once. Without --expect-private it is
# always pulled by its --image name, so a stale local copy under that tag is replaced
# by the registry's image, and a failed pull fails the gate (no fallback to the local
# copy). With --expect-private only the local copy is used. Its ID goes into a Compose
# override that removes the dev service's build section, so Compose can neither build
# nor pull.
OVERRIDE="$SCRATCH/gate-compose.override.yml"
IMAGE_ID=""
IMAGE_ERROR=""
exec 3>&1
resolve_image() {
    [ -n "$IMAGE_ID" ] && return 0
    [ -n "$IMAGE_ERROR" ] && return 1
    local source
    if [ "$EXPECT_PRIVATE" = 1 ]; then
        source="local copy, not pulled (--expect-private)"
        docker image inspect --format '{{.Id}}' "$IMAGE" >/dev/null 2>&1 \
            || IMAGE_ERROR="$IMAGE is not present locally, and --expect-private runs only a local copy"
    else
        source="pulled from the registry"
        docker pull "$IMAGE" >"$LOGS/image-pull.log" 2>&1 \
            || IMAGE_ERROR="docker pull $IMAGE failed, and a local copy is not used without --expect-private: $(tail -1 "$LOGS/image-pull.log" | cut -c1-200)"
    fi
    if [ -z "$IMAGE_ERROR" ]; then
        IMAGE_ID=$(docker image inspect --format '{{.Id}}' "$IMAGE" 2>/dev/null)
        [ -n "$IMAGE_ID" ] || IMAGE_ERROR="docker image inspect $IMAGE printed no ID"
    fi
    if [ -n "$IMAGE_ERROR" ]; then
        echo "image: $IMAGE unavailable: $IMAGE_ERROR" >&3
        return 1
    fi
    local digests
    digests=$(docker image inspect --format '{{join .RepoDigests " "}}' "$IMAGE" 2>/dev/null)
    echo "image: $IMAGE id=$IMAGE_ID digests=${digests:-none (local build, never pushed or pulled)} source=$source" >&3
    cat >"$OVERRIDE" <<YAML
# Written by fresh_clone_gate.sh: run the selected image, never build or pull it.
services:
  dev:
    image: "$IMAGE_ID"
    build: !reset null
    pull_policy: never
YAML
}

# Run a command in the dev container with the clone mounted at /workspace.
in_dev() {
    if ! resolve_image; then
        echo "error: Error: no container started, image $IMAGE_ERROR"
        return 1
    fi
    docker compose -p "$PROJECT" -f "$CLONE/docker-compose.yml" -f "$OVERRIDE" run --rm -T \
        -e ENTRYPOINT_SKIP_BUILD=1 -e ROS_DOMAIN_ID="$DOMAIN" \
        -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e MAKEFLAGS=-j4 \
        dev bash -lc "$1"
}

# The per-run compose project owns its named volumes (vscode-local holds the pip
# installs). Remove exactly those at exit, nothing of any other project.
cleanup() {
    [ -f "$CLONE/docker-compose.yml" ] && \
        docker compose -p "$PROJECT" -f "$CLONE/docker-compose.yml" down -v >/dev/null 2>&1
}
trap cleanup EXIT

echo "fresh-clone gate: repo=$REPO_URL commit=${COMMIT:-<default branch>} scratch=$SCRATCH domain=$DOMAIN"

# --- clone ------------------------------------------------------------------
if want clone; then
    if GIT_TERMINAL_PROMPT=0 git clone --quiet "$REPO_URL" "$CLONE" >"$LOGS/clone.log" 2>&1 \
        && { [ -z "$COMMIT" ] || git -C "$CLONE" checkout --quiet --detach "$COMMIT" >>"$LOGS/clone.log" 2>&1; }; then
        pass clone "$(git -C "$CLONE" rev-parse HEAD) $(git -C "$CLONE" log -1 --format=%s | cut -c1-80)"
    else
        fail clone "$(why "$LOGS/clone.log" 'fatal|error') (log $LOGS/clone.log)"
        echo "SUMMARY: FAIL (clone failed, nothing else can run)"; exit 1
    fi
fi
[ -d "$CLONE/.git" ] || { echo "FAIL setup: no clone at $CLONE"; exit 1; }

# --- the steps of this tree ----------------------------------------------------
STEPS="clone submodules reachable-submodules reachable-image colcon-build colcon-test \
colcon-build-all pytest-colcon_lf pytest-lf_eval tutorial-run tutorial-output doc-links \
support-matrix"
PYTEST_TOOLS=(colcon_lf lf_eval)
DOC_LINK_PAGES=(docs/features/support-matrix.md)
EXTRA_STEPS=""
run_extra_steps() { :; }
if [ -n "$ONLY" ]; then
    IFS=',' read -r -a asked <<<"${ONLY:1:-1}"
    for s in "${asked[@]}"; do
        [[ " $STEPS $EXTRA_STEPS " == *" $s "* ]] && continue
        echo "FAIL setup: '$s' is not a step of this tree (steps: $STEPS${EXTRA_STEPS:+ $EXTRA_STEPS})"
        exit 2
    done
fi

# --- submodules ---------------------------------------------------------------
if want submodules; then
    if GIT_TERMINAL_PROMPT=0 git -C "$CLONE" submodule update --init --recursive >"$LOGS/submodules.log" 2>&1; then
        pass submodules "$(git -C "$CLONE" submodule status --recursive | grep -vc '^-') initialised"
    else
        fail submodules "$(why "$LOGS/submodules.log" 'fatal|error') (log $LOGS/submodules.log)"
    fi
fi

# --- reachability: every submodule URL, anonymously ---------------------------
if want reachable-submodules; then
    # A missing or unreadable .gitmodules must not read as "0 of 0 reachable".
    urls=$(git -C "$CLONE" config -f .gitmodules --get-regexp '^submodule\..*\.url$' 2>>"$LOGS/reachable.log")
    cfg_rc=$?
    bad=(); n=0
    while read -r _key url; do
        [ -n "$url" ] || continue
        n=$((n + 1))
        if ! GIT_TERMINAL_PROMPT=0 GIT_ASKPASS=/bin/false git ls-remote "$url" HEAD >>"$LOGS/reachable.log" 2>&1; then
            bad+=("$url")
        fi
    done <<<"$urls"
    if [ "$cfg_rc" -ne 0 ] || [ "$n" -eq 0 ]; then
        fail reachable-submodules "no submodule URL read from .gitmodules (git config exit $cfg_rc, $n URLs)"
    elif [ ${#bad[@]} -eq 0 ]; then pass reachable-submodules "$n of $n submodule URLs answer git ls-remote"
    else fail reachable-submodules "unreachable: ${bad[*]}"; fi
fi

# --- reachability: the dev image, anonymously, without pulling -----------------
if want reachable-image; then
    # The exit status decides, not stderr: a silent non-zero exit is a refusal.
    DOCKER_CONFIG=$(mktemp -d) docker manifest inspect "$IMAGE" >/dev/null 2>"$LOGS/reachable-image.log"
    anon_rc=$?
    anon="exit $anon_rc: $(head -1 "$LOGS/reachable-image.log")"
    [ -s "$LOGS/reachable-image.log" ] || anon="exit $anon_rc, no message"
    if [ "$anon_rc" -eq 0 ]; then
        pass reachable-image "$IMAGE manifest readable anonymously"
    elif [ "$EXPECT_PRIVATE" = 1 ] && local_id=$(docker image inspect --format '{{.Id}}' "$IMAGE" 2>/dev/null); then
        pass reachable-image "private: anonymous manifest read refused ($anon), local copy ${local_id#sha256:} used (--expect-private)"
    else
        fail reachable-image "$IMAGE not readable anonymously ($anon)"
    fi
fi

# --- build and test the minimum slice in the dev container --------------------
SLICE="ros2_lf_binding pubsub_example"
if want colcon-build; then
    if in_dev "cd packages && colcon build --packages-up-to $SLICE --parallel-workers 2 \
            --event-handlers console_cohesion- status-" >"$LOGS/colcon-build.log" 2>&1; then
        pass colcon-build "$(why "$LOGS/colcon-build.log" '^Summary:')"
    else
        fail colcon-build "$(why "$LOGS/colcon-build.log" 'provided by "[^"]+"|not found|No such file' 'Failed|failed|Error|error') (log $LOGS/colcon-build.log)"
    fi
fi
if want colcon-test; then
    in_dev "cd packages && source install/setup.bash && colcon test --packages-select $SLICE \
            --event-handlers console_cohesion- status- && colcon test-result --verbose" >"$LOGS/colcon-test.log" 2>&1
    test_rc=$?
    # A run that collected no test result also exits 0, so the summary must show
    # a positive test count with no error and no failure.
    summary=$(grep -E '^Summary: [0-9]+ tests, [0-9]+ errors, [0-9]+ failures, [0-9]+ skipped' "$LOGS/colcon-test.log" | tail -1)
    read -r tests errors failures < <(echo "$summary" | sed -E 's/^Summary: ([0-9]+) tests, ([0-9]+) errors, ([0-9]+) failures.*/\1 \2 \3/')
    if [ "$test_rc" -eq 0 ] && [ -n "$summary" ] && [ "$tests" -gt 0 ] && [ "$errors" -eq 0 ] && [ "$failures" -eq 0 ]; then
        pass colcon-test "$summary"
    elif [ -n "$summary" ]; then
        fail colcon-test "exit $test_rc, $summary, needs more than 0 tests and 0 errors and failures (log $LOGS/colcon-test.log)"
    else
        fail colcon-test "exit $test_rc, no colcon test-result summary line, $(why "$LOGS/colcon-test.log" 'failed|Error') (log $LOGS/colcon-test.log)"
    fi
fi
# The colcon_lf suite parses every example package (launch files, compile commands),
# so it needs the whole workspace built, as CI builds it before that suite. No
# --symlink-install: it cannot rebuild over the plain install of colcon-build.
if want colcon-build-all; then
    if in_dev "cd packages && colcon build --parallel-workers 2 \
            --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --event-handlers console_cohesion- status-" >"$LOGS/colcon-build-all.log" 2>&1; then
        pass colcon-build-all "$(why "$LOGS/colcon-build-all.log" '^Summary:')"
    else
        fail colcon-build-all "$(why "$LOGS/colcon-build-all.log" 'provided by "[^"]+"|not found|No such file' 'Failed|failed|Error|error') (log $LOGS/colcon-build-all.log)"
    fi
fi
for tool in "${PYTEST_TOOLS[@]}"; do
    step="pytest-$tool"
    want "$step" || continue
    if in_dev "source packages/install/setup.bash && cd tools/$tool \
            && python3 -m pytest tests -q -p no:cacheprovider" >"$LOGS/$step.log" 2>&1; then
        pass "$step" "$(why "$LOGS/$step.log" ' passed| skipped')"
    else
        fail "$step" "$(why "$LOGS/$step.log" ' failed| error|Error') (log $LOGS/$step.log)"
    fi
done

# --- the getting-started tutorial, end to end, with its output checked --------
if want tutorial-run; then
    # exec 2>&1 inside the container: one stream, so the log keeps the order of the run.
    if in_dev "exec 2>&1; cd packages && source install/setup.bash && colcon lf --package pubsub_example \
            && source install/setup.bash \
            && { timeout --kill-after=5 15 ros2 lf launch pubsub_example bringup; rc=\$?; \
                 echo \"launch rc=\$rc\"; [ \$rc = 124 ]; }" >"$LOGS/tutorial.log" 2>&1; then
        pass tutorial-run "colcon lf --package pubsub_example, then ros2 lf launch ran 15 s (log $LOGS/tutorial.log)"
    else
        fail tutorial-run "$(why "$LOGS/tutorial.log" 'provided by "[^"]+"|not found|No such file|does not exist' 'launch rc=|Error|error|failed') (log $LOGS/tutorial.log)"
    fi
fi
if want tutorial-output; then
    if out=$(python3 "$CLONE/scripts/check_tutorial_output.py" "$LOGS/tutorial.log" 2>&1); then
        pass tutorial-output "$(echo "$out" | head -1)"
    else
        fail tutorial-output "$(echo "$out" | tr '\n' ' ' | cut -c1-300)"
    fi
fi

# --- every link of the support matrix and of every README resolves -------------
# A missing page is a broken link, so a tree without the support matrix fails here.
# The READMEs are in no MkDocs build, so no other check reads their links.
if want doc-links; then
    mapfile -t readmes < <(git -C "$CLONE" ls-files -- README.md '*/README.md' ':!third_party/**')
    if out=$(cd "$CLONE" && python3 scripts/check_doc_links.py "${DOC_LINK_PAGES[@]}" "${readmes[@]}" 2>&1); then
        pass doc-links "${DOC_LINK_PAGES[*]} and ${#readmes[@]} READMEs: $(echo "$out" | tail -1)"
    else
        fail doc-links "$(echo "$out" | grep -m3 -e 'broken link' -e 'Error' | tr '\n' ' ')"
    fi
fi

# --- the support matrix agrees with the feature registry and its limits --------
if want support-matrix; then
    if in_dev "cd tools/colcon_lf && python3 -m colcon_lf.features.docs_table --check" >"$LOGS/support-matrix.log" 2>&1; then
        pass support-matrix "docs/features/support-matrix.md matches the feature registry, evidence and limits"
    else
        fail support-matrix "$(why "$LOGS/support-matrix.log" '^error') (log $LOGS/support-matrix.log)"
    fi
fi

run_extra_steps

if [ ${#FAILED[@]} -eq 0 ] && [ "$PASSED" -eq 0 ]; then
    echo "SUMMARY: FAIL (no step ran)"
    exit 1
fi
if [ ${#FAILED[@]} -eq 0 ]; then
    echo "SUMMARY: PASS ($PASSED steps)"
    exit 0
fi
echo "SUMMARY: FAIL (${#FAILED[@]} failed: ${FAILED[*]}; $PASSED passed)"
exit 1
