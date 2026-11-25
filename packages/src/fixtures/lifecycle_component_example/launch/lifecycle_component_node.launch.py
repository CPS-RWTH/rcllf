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
Launch the managed lifecycle node as a composed component.

This is the shape this fixture covers. Declaring the node as a
``ComposableNode`` is what puts it on the component path, and the component
path cannot drive a managed lifecycle. Under plain ROS 2 this container loads
the node and leaves it ``unconfigured``, so nothing is published until a
supervisor transitions it. Under the wrapper, generation now re-routes it to
the non-component path so its lifecycle is driven before binding.
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    nodes = [
        ComposableNode(
            package='lifecycle_component_example',
            plugin='lifecycle_component_example::LifecycleComponentNode',
            name='lifecycle_component_node',
        ),
    ]

    container = ComposableNodeContainer(
        name='lifecycle_component_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )

    return LaunchDescription([container])
