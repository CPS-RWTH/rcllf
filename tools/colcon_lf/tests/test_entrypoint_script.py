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

"""docker/entrypoint.sh stops on an empty reactor-c checkout with the fix command."""

import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / 'docker' / 'entrypoint.sh'
FIX = 'git submodule update --init third_party/reactor-c'
HAS_ROS = Path('/opt/ros/jazzy/setup.bash').is_file()


def _workspace(tmp_path, *, reactor_c_filled):
    """A checkout as `git clone` leaves it: .gitmodules plus the gitlink dir."""
    (tmp_path / '.gitmodules').write_text(
        '[submodule "third_party/reactor-c"]\n'
        '\tpath = third_party/reactor-c\n'
        '\turl = https://github.com/lf-lang/reactor-c.git\n')
    reactor_c = tmp_path / 'third_party' / 'reactor-c'
    reactor_c.mkdir(parents=True)
    if reactor_c_filled:
        (reactor_c / 'CMakeLists.txt').write_text('# stand-in\n')
    return tmp_path


def _run(workspace, marker):
    env = dict(os.environ, WORKSPACE=str(workspace), ENTRYPOINT_SKIP_BUILD='1',
               LTTNG_HOME=str(workspace / 'lttng'))
    return subprocess.run(['bash', str(SCRIPT), 'touch', str(marker)], env=env,
                          capture_output=True, text=True, timeout=120)


def test_empty_reactor_c_stops_with_the_fix_command(tmp_path):
    workspace = _workspace(tmp_path, reactor_c_filled=False)
    marker = tmp_path / 'command-ran'
    proc = _run(workspace, marker)
    assert proc.returncode != 0, (
        'entrypoint.sh went on with an empty third_party/reactor-c; stderr:\n' + proc.stderr)
    assert FIX in proc.stderr, (
        f'entrypoint.sh did not print the fix command {FIX!r}; stderr:\n{proc.stderr}')
    assert not marker.exists(), 'entrypoint.sh ran the container command anyway'


def test_filled_reactor_c_passes_the_check(tmp_path):
    workspace = _workspace(tmp_path, reactor_c_filled=True)
    marker = tmp_path / 'command-ran'
    proc = _run(workspace, marker)
    assert 'reactor-c is empty' not in proc.stderr, proc.stderr
    if HAS_ROS:
        # In the dev image the entrypoint goes on to run the container command.
        assert proc.returncode == 0, proc.stderr
        assert marker.exists(), 'entrypoint.sh did not run the container command'


@pytest.mark.skipif(not HAS_ROS, reason='the entrypoint sources /opt/ros/jazzy (dev image)')
def test_no_gitmodules_skips_the_check(tmp_path):
    # An image-only workspace (no git checkout) has no .gitmodules: no check.
    marker = tmp_path / 'command-ran'
    proc = _run(tmp_path, marker)
    assert proc.returncode == 0, proc.stderr
    assert marker.exists()
