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
T2 container: R2 with a radar tracker before the fusion.

r2_container.launch.py with one change: radar and radar2 still publish on
/radar/range, the new radar_tracker uses every message of both radars and
publishes /radar/track, and the fusion is a two-input message_filters
synchronizer on /radar/track (its /radar/range input remapped) and
/camera/range. `fusion` picks the policy: approximate (ApproximateFusionNode,
the default) or exact (ExactFusionNode); t2_exact.launch.py and
t2_approx.launch.py pin it, so `colcon lf-analyze` sees one system per policy.
The federation config/t2/<system>_2host.federation.yaml puts the tracker with
the fusion and the controller on host_b, so /radar/range crosses the federate
boundary as in r2_container.launch.py. stamp_mode defaults to `now` as in
r2_container.launch.py (logical time in rcllf, system time in stock).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ARGS = {
    'container': 'component_container',
    'stamp_mode': 'now',
    'stamp_origin_ns': '0',
    'queue_size': '9',  # approximate_fusion_node.cpp
    'use_sim_time': 'false',
    'publish_clock': 'false',
    'steps_per_tick': '1',
    'detection_work_us': '2500',  # camera_node.cpp
    'detection_work_mode': 'fixed',
    'start_at_ns': '0',
    'log_inputs': 'true',
    'radar2_bias': '0.5',
    'gain': '0.5',
    'fusion': 'approximate',
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
    # Simulated horizon in 1 ms plant steps: the
    # simulator logs its horizon event at this step. 20000 is the node's own
    # default (simulator_node.cpp); a campaign with a fixed
    # horizon H passes H here, so rcllf, stock and RSLCPP end at one step.
    'max_steps': '20000',
}


def generate_launch_description():
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': arg('use_sim_time')}
    stamps = {'stamp_mode': arg('stamp_mode'), 'stamp_origin_ns': arg('stamp_origin_ns')}

    def component(cls, name, params, **kw):
        return ComposableNode(
            package='brake_fusion', plugin='brake_fusion::' + cls, name=name,
            parameters=[params], **kw)

    def fusion(cls, policy):
        return component(
            cls, 'fusion', {**sim_time, 'queue_size': arg('queue_size')},
            remappings=[('/radar/range', '/radar/track')],
            condition=LaunchConfigurationEquals('fusion', policy))

    nodes = [
        component('SimulatorNode', 'simulator', {
            'use_sim_time': False, 'publish_clock': arg('publish_clock'),
            'steps_per_tick': arg('steps_per_tick'),
            'start_at_ns': arg('start_at_ns'), 'expected_state_subscribers': 4,
            'brake_actuation_delay_ms': arg('brake_actuation_delay_ms'),
            'max_steps': arg('max_steps')}),
        component('RadarNode', 'radar', {**sim_time, **stamps}),
        component('RadarNode', 'radar2', {**sim_time, **stamps, 'range_bias': arg('radar2_bias')}),
        component('CameraNode', 'camera', {
            **sim_time, **stamps, 'detection_work_us': arg('detection_work_us'),
            'detection_work_mode': arg('detection_work_mode')}),
        component('WheelSpeedNode', 'wheel_speed', {**sim_time, **stamps}),
        component('RadarTrackerNode', 'radar_tracker', {**sim_time, 'gain': arg('gain')}),
        fusion('ExactFusionNode', 'exact'),
        fusion('ApproximateFusionNode', 'approximate'),
        component('ControllerNode', 'controller', {
            **sim_time, 'log_inputs': arg('log_inputs'), 'fusion': arg('fusion'),
            'stop_margin_m': arg('stop_margin_m'), 'partial_stage': arg('partial_stage'),
            'brake_actuation_delay_ms': arg('brake_actuation_delay_ms')}),
    ]
    container = ComposableNodeContainer(
        name='brake_fusion_container',
        namespace='',
        package='rclcpp_components',
        executable=arg('container'),
        composable_node_descriptions=nodes,
        output='screen',
    )
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [container])
