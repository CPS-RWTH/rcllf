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

# LoadComposableNodes targeting a pre-existing external container by name.

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='feature_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen',
    )

    loader = LoadComposableNodes(
        target_container='feature_container',
        composable_node_descriptions=[
            ComposableNode(
                package='launch_features_example',
                plugin='launch_features_example::FeatureComponent',
                name='loaded_component',
            ),
        ],
    )

    return LaunchDescription([container, loader])
