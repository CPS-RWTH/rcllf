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
"""
A4, the supervised plant timeout from outside the process.

simulator_fault_node is simulator_node linked against rcllf_plant_fault, the
Transactor build that honours RCLLF_PLANT_FAULT at the plant thread's dispatch.
With `hang:<k>` the plant step k never returns, with `delay:<k>:<ms>` it
returns after the timeout. Either way the node must log plant_timeout for
step k within plant_step_timeout_ms plus the margin below, publish no
/sim/state for step k or later, and exit with code 3, and the test ends in
bounded wall time whatever the node does.

Margin: 1000 ms on top of the timeout, for the DDS delivery of the last state
to this process and the exit itself; the 200 ms timeout plus the margin is
still far below the 20 s bound that a hung node would run into.
"""

import json
import os
import signal
import subprocess
import time

from custom_interfaces.msg import BrakeCmd
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

FAULT_STEP = 50
TIMEOUT_MS = 200
MARGIN_MS = 1000
BOUND_S = 20.0


class Watcher(Node):

    def __init__(self):
        super().__init__('plant_timeout_watcher')
        self.received = []
        # The start gate waits for a /brake_cmd publisher; it sends nothing.
        # The simulator requests the controller's offered deadline
        # (loss_events.hpp kBrakeDeadline, 37.5 ms), so the publisher offers it.
        self.pub = self.create_publisher(
            BrakeCmd, '/brake_cmd',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       deadline=Duration(nanoseconds=37_500_000)))
        self.create_subscription(
            Odometry, '/sim/state', self.on_state,
            QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))

    def on_state(self, _msg):
        self.received.append(time.monotonic())


def run_fault(fault, tmp_path, until_states=None):
    node_bin = os.environ.get('SIM_FAULT_NODE')
    if not node_bin:
        pytest.skip('SIM_FAULT_NODE not set (registered by CMake under BUILD_TESTING)')
    env = dict(os.environ, RCLLF_PLANT_FAULT=fault)
    args = [node_bin, '--ros-args',
            '-p', 'expected_state_subscribers:=1',
            '-p', 'max_steps:=1000',
            '-p', f'plant_step_timeout_ms:={TIMEOUT_MS}',
            '-p', f'log_dir:={tmp_path}']
    rclpy.init()
    watcher = Watcher()
    with open(tmp_path / 'simulator.out', 'w') as out:
        child = subprocess.Popen(args, stdout=out, stderr=subprocess.STDOUT, env=env,
                                 start_new_session=True)
    t0 = time.monotonic()
    exited = None
    try:
        while time.monotonic() - t0 < BOUND_S:
            rclpy.spin_once(watcher, timeout_sec=0.01)
            if child.poll() is not None:
                exited = time.monotonic()
                break
            if until_states is not None and len(watcher.received) >= until_states:
                break
    finally:
        hung = child.poll() is None
        if hung and until_states is not None and len(watcher.received) >= until_states:
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=10)
            hung = False
        elif hung:
            # Its own process group only (start_new_session), by its pid.
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
        watcher.destroy_node()
        rclpy.shutdown()
    assert watcher.received, f'{fault}: no /sim/state received, the simulator never started'
    assert not hung, (
        f'{fault}: simulator still running {BOUND_S:.0f} s after start, killed: '
        'the plant step is not supervised')
    log = [json.loads(line) for line in open(tmp_path / 'simulator.jsonl')]
    return child.returncode, log, watcher.received, exited


@pytest.mark.parametrize('fault', [f'hang:{FAULT_STEP}', f'delay:{FAULT_STEP}:600'])
def test_plant_timeout_fails_closed(fault, tmp_path):
    rc, log, received, exited = run_fault(fault, tmp_path)
    plant = [r for r in log if r['ev'] == 'plant']
    assert plant and plant[0]['fault'] == fault, plant
    steps = [r['step'] for r in log if r['ev'] == 'plant_step']
    timeouts = [r for r in log if r['ev'] == 'plant_timeout']
    assert timeouts, f'{fault}: no plant_timeout logged; exit code {rc}'
    assert timeouts[0]['step'] == FAULT_STEP, timeouts
    assert steps == list(range(1, FAULT_STEP)), \
        f'{fault}: plant_step lines {steps[:3]}..{steps[-3:]} beyond step {FAULT_STEP - 1}'
    assert len(received) == FAULT_STEP - 1, \
        f'{fault}: {len(received)} /sim/state received, expected {FAULT_STEP - 1}'
    assert rc == 3, f'{fault}: exit code {rc}, expected 3'
    waited_ms = (exited - received[-1]) * 1000
    assert waited_ms <= TIMEOUT_MS + MARGIN_MS, \
        f'{fault}: exit {waited_ms:.0f} ms after the last state, bound {TIMEOUT_MS + MARGIN_MS}'


def test_without_fault_the_simulator_reaches_the_horizon(tmp_path):
    """The same harness on a healthy plant: no timeout, the horizon is reached."""
    node_bin = os.environ.get('SIM_FAULT_NODE')
    if not node_bin:
        pytest.skip('SIM_FAULT_NODE not set')
    rc, log, received, _ = run_fault('', tmp_path, until_states=1000)
    assert not [r for r in log if r['ev'] in ('plant_timeout', 'plant_fault')]
    assert [r for r in log if r['ev'] == 'horizon'], 'no horizon reached'
    assert len(received) >= 1000
    assert [r['step'] for r in log if r['ev'] == 'plant_step'] == list(range(1, 1001))


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
