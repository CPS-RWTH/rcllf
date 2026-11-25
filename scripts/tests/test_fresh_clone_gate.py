# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""The fresh-clone gate runs the image it checks (scripts/fresh_clone_gate.sh).

The gate runs with a fake ``docker`` on PATH that logs every call, so these tests need
no Docker daemon. The last test hands the gate's Compose override to the real
``docker compose config`` with the repository's ``docker-compose.yml``, and skips
without a Docker CLI.
"""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
GATE = ROOT / 'scripts' / 'fresh_clone_gate.sh'
IMAGE = 'ghcr.io/cps-rwth/rcllf:0.1.0-deps'
IMAGE_ID = 'sha256:' + 'ab' * 32

# Logs each call. `image inspect` knows only the image named in $FAKE_STATE/local, with
# ID $FAKE_ID, or $FAKE_PULLED_ID after a pull. `pull` succeeds and makes the image
# local when $FAKE_PULL_OK is 1.
FAKE_DOCKER = r'''#!/usr/bin/env bash
echo "docker $*" >>"$FAKE_LOG"
case "$1 $2" in
  "image inspect")
    ref="${@: -1}"
    [ -f "$FAKE_STATE/local" ] && [ "$ref" = "$(cat "$FAKE_STATE/local")" ] || exit 1
    id="$FAKE_ID"; [ -f "$FAKE_STATE/pulled" ] && [ -n "$FAKE_PULLED_ID" ] && id="$FAKE_PULLED_ID"
    case "$*" in *RepoDigests*) echo "$FAKE_DIGEST" ;; *) echo "$id" ;; esac
    exit 0 ;;
  "manifest inspect") echo "denied" >&2; exit 1 ;;
esac
if [ "$1" = pull ]; then
  [ "$FAKE_PULL_OK" = 1 ] || { echo "pull access denied" >&2; exit 1; }
  echo "$2" >"$FAKE_STATE/local"; touch "$FAKE_STATE/pulled"; exit 0
fi
exit 0
'''


def _repo(tmp_path, files=None):
    repo = tmp_path / 'repo'
    repo.mkdir()
    shutil.copy(ROOT / 'docker-compose.yml', repo / 'docker-compose.yml')
    for rel, text in (files or {}).items():
        (repo / rel).parent.mkdir(parents=True, exist_ok=True)
        (repo / rel).write_text(text)
    git = ['git', '-C', str(repo), '-c', 'user.name=t', '-c', 'user.email=t@t']
    subprocess.run(git[:3] + ['init', '-q'], check=True)
    subprocess.run(git + ['add', '.'], check=True)
    subprocess.run(git + ['commit', '-q', '-m', 'fixture'], check=True)
    return repo


def _gate(tmp_path, *extra, local=None, pull_ok=False, digest='', pulled_id='', files=None):
    bin_dir = tmp_path / 'bin'
    state = tmp_path / 'state'
    bin_dir.mkdir()
    state.mkdir()
    (bin_dir / 'docker').write_text(FAKE_DOCKER)
    (bin_dir / 'docker').chmod(0o755)
    if local:
        (state / 'local').write_text(local + '\n')
    log = tmp_path / 'docker.log'
    log.touch()
    env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}', FAKE_LOG=str(log),
               FAKE_STATE=str(state), FAKE_ID=IMAGE_ID, FAKE_DIGEST=digest,
               FAKE_PULL_OK='1' if pull_ok else '0', FAKE_PULLED_ID=pulled_id,
               ROS_DOMAIN_ID='150')
    scratch = tmp_path / 'scratch'
    out = subprocess.run(
        ['bash', str(GATE), '--repo', str(_repo(tmp_path, files)), '--commit', 'HEAD',
         '--scratch', str(scratch), '--image', IMAGE, *extra],
        capture_output=True, text=True, env=env, timeout=120)
    return out, log.read_text().splitlines(), scratch / 'gate-compose.override.yml'


def _runs(calls):
    return [c for c in calls if c.startswith('docker compose') and ' run ' in c]


def test_private_release_image_reaches_every_container_run(tmp_path):
    digest = 'ghcr.io/cps-rwth/rcllf@sha256:' + 'cd' * 32
    out, calls, override = _gate(tmp_path, '--expect-private', '--steps',
                                 'clone,reachable-image,support-matrix',
                                 local=IMAGE, digest=digest)
    assert out.returncode == 0, out.stdout + out.stderr
    assert f'image: {IMAGE} id={IMAGE_ID} digests={digest}' in out.stdout
    assert 'PASS reachable-image: private: anonymous manifest read refused' in out.stdout
    runs = _runs(calls)
    assert len(runs) == 1 and f'-f {override} run --rm -T' in runs[0], calls
    assert not [c for c in calls if c.startswith('docker pull')], calls
    text = override.read_text()
    assert f'image: "{IMAGE_ID}"' in text
    assert 'build: !reset null' in text and 'pull_policy: never' in text


def test_missing_private_image_fails_without_build_or_pull(tmp_path):
    # Another tag is local, the selected one is not: no fallback to it.
    out, calls, override = _gate(tmp_path, '--expect-private', '--steps',
                                 'clone,support-matrix',
                                 local='ghcr.io/cps-rwth/rcllf:dev')
    assert out.returncode == 1
    assert (f'image: {IMAGE} unavailable: {IMAGE} is not present locally, and '
            '--expect-private runs only a local copy') in out.stdout
    assert 'FAIL support-matrix: error: Error: no container started' in out.stdout
    assert not _runs(calls) and not override.exists(), calls
    assert not [c for c in calls if c.startswith(('docker pull', 'docker build'))], calls


def test_public_image_is_pulled_once_by_its_name(tmp_path):
    out, calls, override = _gate(tmp_path, '--steps', 'clone,support-matrix', pull_ok=True)
    assert out.returncode == 0, out.stdout + out.stderr
    assert [c for c in calls if c.startswith('docker pull')] == [f'docker pull {IMAGE}']
    assert f'image: {IMAGE} id={IMAGE_ID} digests=none' in out.stdout
    assert len(_runs(calls)) == 1


def test_public_gate_pulls_over_a_stale_local_copy(tmp_path):
    # A local copy under the tag differs from the registry's image: the pull wins.
    fresh = 'sha256:' + 'ef' * 32
    out, calls, override = _gate(tmp_path, '--steps', 'clone,support-matrix',
                                 local=IMAGE, pull_ok=True, pulled_id=fresh)
    assert out.returncode == 0, out.stdout + out.stderr
    assert [c for c in calls if c.startswith('docker pull')] == [f'docker pull {IMAGE}']
    assert f'image: {IMAGE} id={fresh} ' in out.stdout
    assert 'source=pulled from the registry' in out.stdout
    assert f'image: "{fresh}"' in override.read_text()
    assert IMAGE_ID not in override.read_text()


def test_public_gate_fails_when_the_pull_fails_despite_a_local_copy(tmp_path):
    out, calls, override = _gate(tmp_path, '--steps', 'clone,support-matrix',
                                 local=IMAGE, pull_ok=False)
    assert out.returncode == 1
    assert (f'image: {IMAGE} unavailable: docker pull {IMAGE} failed, and a local copy '
            'is not used without --expect-private') in out.stdout
    assert 'FAIL support-matrix: error: Error: no container started' in out.stdout
    assert not _runs(calls) and not override.exists(), calls


def test_doc_links_reads_every_readme(tmp_path):
    # The support matrix and its target resolve; a package README one level short fails.
    files = {
        'scripts/check_doc_links.py': (ROOT / 'scripts' / 'check_doc_links.py').read_text(),
        'docs/features/support-matrix.md': '# Matrix\n\nSee [tf2](tf2.md).\n',
        'docs/features/tf2.md': '# tf2\n',
        'packages/src/ex/README.md': '# ex\n\nSee [tf2](../../docs/features/tf2.md).\n',
    }
    out, _calls, _override = _gate(tmp_path, '--steps', 'clone,doc-links', files=files)
    assert out.returncode == 1
    assert ("FAIL doc-links: broken link: packages/src/ex/README.md:3: link target "
            "'../../docs/features/tf2.md' does not exist") in out.stdout


@pytest.mark.skipif(shutil.which('docker') is None, reason='no docker CLI')
def test_override_sets_the_dev_service_image_in_the_real_compose_file(tmp_path):
    real_docker = shutil.which('docker')
    if subprocess.run([real_docker, 'compose', 'version'], capture_output=True).returncode:
        pytest.skip('no docker compose plugin')
    out, _calls, override = _gate(tmp_path, '--expect-private', '--steps',
                                  'clone,support-matrix', local=IMAGE)
    assert out.returncode == 0, out.stdout + out.stderr
    cfg = subprocess.run(
        [real_docker, 'compose', '-p', 'gate-test', '-f', str(ROOT / 'docker-compose.yml'),
         '-f', str(override), 'config', '--format', 'json'],
        capture_output=True, text=True, check=True)
    dev = json.loads(cfg.stdout)['services']['dev']
    assert dev['image'] == IMAGE_ID
    assert 'build' not in dev
    assert dev['pull_policy'] == 'never'
