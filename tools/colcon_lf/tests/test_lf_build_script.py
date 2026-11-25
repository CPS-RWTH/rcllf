# Copyright 2026 rcllf Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""docker/lf_build.sh exit status when lf-generate refuses."""

import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / 'docker' / 'lf_build.sh'

# Stands in for colcon: analyze passes, generate refuses the way the
# generator does (rc 1), build only records that it was called.
FAKE_COLCON = """#!/bin/bash
case "$1" in
  lf-analyze) exit 0 ;;
  lf-generate) echo "  Error: generation refused, fake refusal"; exit 1 ;;
  build) echo "build $*" >> "$FAKE_LOG"; exit 0 ;;
esac
exit 0
"""


@pytest.mark.skipif(not Path('/opt/ros/jazzy/setup.bash').is_file(),
                    reason='lf_build.sh sources /opt/ros/jazzy/setup.bash (dev image)')
def test_refused_generate_fails_the_script(tmp_path):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    colcon = bin_dir / 'colcon'
    colcon.write_text(FAKE_COLCON)
    colcon.chmod(0o755)
    (tmp_path / 'packages' / 'install').mkdir(parents=True)
    (tmp_path / 'packages' / 'install' / 'setup.bash').write_text('')
    log = tmp_path / 'build.log'
    env = dict(os.environ, PATH=f"{bin_dir}:{os.environ['PATH']}",
               WORKSPACE=str(tmp_path), FAKE_LOG=str(log))
    proc = subprocess.run(['bash', str(SCRIPT), 'pubsub_example'], env=env,
                          capture_output=True, text=True, timeout=120)
    assert 'generation refused' in proc.stdout
    built = log.read_text() if log.exists() else ''
    assert proc.returncode != 0, (
        f'lf_build.sh exited 0 after a refused lf-generate, and went on to: {built.strip()}')
    assert not built, f'lf_build.sh built after a refused lf-generate: {built.strip()}'
