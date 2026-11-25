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

# OpaqueFunction(function=launch_setup) for runtime launch construction.
# The node configuration is derived at launch time from argument values.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    period_ms = int(LaunchConfiguration('timer_period_ms').perform(context))
    node = Node(
        package='launch_features_example',
        executable='feature_node',
        name='opaque_node',
        output='screen',
        parameters=[{
            'timer_period_ms': period_ms,
            'message_prefix': 'opaque',
        }],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'timer_period_ms',
            default_value='500',
            description='Timer period resolved at launch time via OpaqueFunction.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
