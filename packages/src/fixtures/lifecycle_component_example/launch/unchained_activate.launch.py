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
Launch the deliberately broken RCLLF-H005 fixture on its own.

`UnchainedActivateNode` overrides `on_activate` without chaining to
`rclcpp_lifecycle::LifecycleNode::on_activate`, so its publisher is never
activated and every publish is dropped. This launch file is the playground case
for `lifecycle.base_call_chaining`, whose expected outcome is WARN with
RCLLF-H005.

It launches only the broken node, so the diagnostic set for this system is
exactly the one code under test. The correct sibling has its own launch file.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    nodes = [
        ComposableNode(
            package='lifecycle_component_example',
            plugin='lifecycle_component_example::UnchainedActivateNode',
            name='unchained_activate_node',
        ),
    ]

    container = ComposableNodeContainer(
        name='unchained_activate_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )

    return LaunchDescription([container])
