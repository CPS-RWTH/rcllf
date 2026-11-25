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

# SetLaunchConfiguration with IfCondition/UnlessCondition switching.
# Selects component_container or component_container_mt based on use_mt argument.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_mt_arg = DeclareLaunchArgument(
        'use_mt',
        default_value='false',
        choices=['true', 'false'],
        description='Use multithreaded component container when true.',
    )

    set_single = SetLaunchConfiguration(
        'container_exec',
        'component_container',
        condition=UnlessCondition(LaunchConfiguration('use_mt')),
    )

    set_mt = SetLaunchConfiguration(
        'container_exec',
        'component_container_mt',
        condition=IfCondition(LaunchConfiguration('use_mt')),
    )

    container = ComposableNodeContainer(
        name='feature_container_dynamic',
        namespace='',
        package='rclcpp_components',
        executable=LaunchConfiguration('container_exec'),
        composable_node_descriptions=[
            ComposableNode(
                package='launch_features_example',
                plugin='launch_features_example::FeatureComponent',
                name='dynamic_component',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([use_mt_arg, set_single, set_mt, container])
