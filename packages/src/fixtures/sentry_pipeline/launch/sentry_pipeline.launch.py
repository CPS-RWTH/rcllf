#!/usr/bin/env python3
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
Single-process composable-node container for sentry_pipeline.

All 9 nodes are loaded into one rclcpp_components container
so that the stock executors and the lfc variant differ only in
scheduling/runtime, never in process topology. Node names match the
`<node_name>` keys of the SENTRY_SCENARIO overlay so that the ROS params file
and the overlay resolve unambiguously.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    nodes = [
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::ImuNode',
            name='imu_node',
        ),
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::LidarNode',
            name='lidar_node',
        ),
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::CameraNode',
            name='camera_node',
        ),
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::FusionNode',
            name='fusion_node',
        ),
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::DetectorNode',
            name='detector_node',
        ),
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::PlannerNode',
            name='planner_node',
        ),
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::ControllerNode',
            name='controller_node',
        ),
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::ActuatorSinkNode',
            name='actuator_sink_node',
        ),
        ComposableNode(
            package='sentry_pipeline',
            plugin='sentry_pipeline::DiagnosticsSinkNode',
            name='diagnostics_sink_node',
        ),
    ]

    container = ComposableNodeContainer(
        name='sentry_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )

    return LaunchDescription([container])
