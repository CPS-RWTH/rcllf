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

# Nested GroupAction — a GroupAction inside another GroupAction.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def generate_launch_description():
    enable_inner_arg = DeclareLaunchArgument(
        'enable_inner',
        default_value='true',
        description='Enable the inner nested group when true.',
    )

    nested_group = GroupAction(
        actions=[
            PushRosNamespace('outer_ns'),
            GroupAction(
                condition=IfCondition(LaunchConfiguration('enable_inner')),
                actions=[
                    SetParameter(name='use_sim_time', value=False),
                    PushRosNamespace('inner_ns'),
                    Node(
                        package='launch_features_example',
                        executable='feature_node',
                        name='nested_node',
                        output='screen',
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription([enable_inner_arg, nested_group])
