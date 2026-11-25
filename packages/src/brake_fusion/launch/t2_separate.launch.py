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
T2 separate: R2 with a radar tracker before the fusion, stock.

r2_separate.launch.py (one process per node) with the T2 change of
t2_container.launch.py: radar_tracker reads both radars on /radar/range and
publishes /radar/track, the fusion is a two-input message_filters
synchronizer on /radar/track (remapped input) and /camera/range. `fusion`
picks approximate (ApproximateFusionNode, the default) or exact
(ExactFusionNode). stamp_mode defaults to `now` (system time in stock): the
default is ApproximateTime with `now` stamps.
Every other argument is as in r2_separate.launch.py, real_time_factor
included.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'stamp_mode': 'now',
    'stamp_origin_ns': '0',
    'queue_size': '9',  # approximate_fusion_node.cpp
    'use_sim_time': 'false',
    'publish_clock': 'false',
    'real_time_factor': '1.0',
    'steps_per_tick': '1',
    'detection_work_us': '2500',  # camera_node.cpp
    'detection_work_mode': 'fixed',
    'start_at_ns': '0',
    'log_inputs': 'true',
    'radar2_bias': '0.5',
    'gain': '0.5',
    'fusion': 'approximate',
    # Stock two-host split: `a` starts the simulator
    # and the four sensors, `b` the tracker, fusion and controller, the host_a /
    # host_b split of config/t2/t2_approx_2host.federation.yaml. `all` (default)
    # starts every node in one launch, as before.
    'side': 'all',
    # Sender-side loss: the controller skips the
    # first brake command at or above this level, so DDS never sees it (the
    # campaign's rq2b drop removes a sample after DDS delivered it). -1 is the
    # node default, off, so every other run is unchanged.
    'drop_first_brake_ge': '-1.0',
    # Stopping margin X: negative selects the
    # controller default kStopMargin of stop_margin.hpp, the one place X
    # lives; stop_margin_m:=0.6 overrides it (write a decimal point).
    'stop_margin_m': '-1.0',
    # false: the full stage alone (t2_approx_fullonly, the worst case the
    # threshold is derived for).
    'partial_stage': 'true',
    # Brake actuation delay d_b in ms: a brake command
    # stamped t acts in the first plant step strictly after t + d_b, and the
    # controller's full-stage
    # threshold uses L + d_b (stop_margin.hpp). One argument for both nodes,
    # so they never disagree. 0 is today's behaviour; t2_approx_db25 and
    # t2_approx_db100 pin the candidates.
    'brake_actuation_delay_ms': '0',
    # The simulator's and the controller's d_b apart (a design choice): stock
    # ROS 2 applies each brake command in the
    # first simulator step after it arrives, with no stamp-keyed queue, so the
    # stock cells run simulator_brake_actuation_delay_ms:=0 while the
    # controller keeps brake_actuation_delay_ms:=100 and its threshold
    # 1.244 s. Negative (default) takes brake_actuation_delay_ms, so every
    # launch line without them is unchanged.
    'simulator_brake_actuation_delay_ms': '-1',
    'controller_brake_actuation_delay_ms': '-1',
    # Fixed simulated horizon: the simulator steps 1 ms
    # and stops at max_steps (simulator_node.cpp), so max_steps:=HORIZON_MS
    # gives stock the horizon rcllf gets from lf_end. 20000 is the node
    # default, so every launch line without it is unchanged.
    'max_steps': '20000',
}


def generate_launch_description():
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': arg('use_sim_time')}
    stamps = {'stamp_mode': arg('stamp_mode'), 'stamp_origin_ns': arg('stamp_origin_ns')}

    def d_b(node_name):
        # the node's own d_b when given (>= 0), else the common one
        own = arg(node_name + '_brake_actuation_delay_ms')
        return ParameterValue(PythonExpression(
            ['int(', own, ') if int(', own, ') >= 0 else int(',
             arg('brake_actuation_delay_ms'), ')']), value_type=int)

    def on_side(side):
        return IfCondition(PythonExpression(
            ["'", arg('side'), "' in ('all', '", side, "')"]))

    def node(executable, name, params, side, **kw):
        return Node(
            package='brake_fusion', executable=executable, name=name, output='screen',
            parameters=[params], condition=kw.pop('condition', on_side(side)), **kw)

    def fusion(policy):
        return node(
            policy + '_fusion_node', 'fusion', {**sim_time, 'queue_size': arg('queue_size')},
            'b', remappings=[('/radar/range', '/radar/track')],
            condition=IfCondition(PythonExpression(
                ["'", arg('side'), "' in ('all', 'b') and '", arg('fusion'), "' == '",
                 policy, "'"])))

    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [
            node('simulator_node', 'simulator', {
                'use_sim_time': False, 'publish_clock': arg('publish_clock'),
                'real_time_factor': ParameterValue(arg('real_time_factor'), value_type=float),
                'steps_per_tick': arg('steps_per_tick'),
                'start_at_ns': arg('start_at_ns'), 'expected_state_subscribers': 4,
                'max_steps': ParameterValue(arg('max_steps'), value_type=int),
                'brake_actuation_delay_ms': d_b('simulator')}, 'a'),
            node('radar_node', 'radar', {**sim_time, **stamps}, 'a'),
            node('radar_node', 'radar2', {**sim_time, **stamps, 'range_bias': arg('radar2_bias')},
                 'a'),
            node('camera_node', 'camera', {
                **sim_time, **stamps, 'detection_work_us': arg('detection_work_us'),
                'detection_work_mode': arg('detection_work_mode')}, 'a'),
            node('wheel_speed_node', 'wheel_speed', {**sim_time, **stamps}, 'a'),
            node('radar_tracker_node', 'radar_tracker', {**sim_time, 'gain': arg('gain')}, 'b'),
            fusion('exact'),
            fusion('approximate'),
            node('controller_node', 'controller', {
                **sim_time, 'log_inputs': arg('log_inputs'), 'fusion': arg('fusion'),
                'drop_first_brake_ge': ParameterValue(
                    arg('drop_first_brake_ge'), value_type=float),
                'stop_margin_m': ParameterValue(arg('stop_margin_m'), value_type=float),
                'partial_stage': ParameterValue(arg('partial_stage'), value_type=bool),
                'brake_actuation_delay_ms': d_b('controller')}, 'b'),
        ])
