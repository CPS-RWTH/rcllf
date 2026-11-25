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

# GroupAction with condition= on the GroupAction itself.
# GroupAction containing SetParameter (scoped to all child nodes).
# GroupAction with scoped=False (namespace leaks out of the group scope).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def generate_launch_description():
    enable_group_arg = DeclareLaunchArgument(
        'enable_group',
        default_value='true',
        description='Enable the conditional group when true.',
    )

    # GroupAction with condition= and SetParameter inside
    conditional_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_group')),
        actions=[
            SetParameter(name='use_sim_time', value=False),
            Node(
                package='launch_features_example',
                executable='feature_node',
                name='group_node_a',
                output='screen',
            ),
        ],
    )

    # GroupAction with scoped=False so PushRosNamespace leaks outward
    unscoped_group = GroupAction(
        scoped=False,
        actions=[
            PushRosNamespace('unscoped_ns'),
            Node(
                package='launch_features_example',
                executable='feature_node',
                name='group_node_b',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([enable_group_arg, conditional_group, unscoped_group])
