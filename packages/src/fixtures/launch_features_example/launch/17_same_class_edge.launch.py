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

# Two instances of one node class wired to each other. relay_a
# publishes feature_output on /same_class_chat, and /grp/relay_b subscribes
# its feature_input to it, so the edge connects two reactor instances of the
# same class.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='launch_features_example',
            executable='feature_node',
            name='relay_a',
            parameters=[{'message_prefix': 'relay_a'}],
            remappings=[('feature_output', '/same_class_chat')],
            output='screen',
        ),
        Node(
            package='launch_features_example',
            executable='feature_node',
            name='relay_b',
            namespace='grp',
            parameters=[{'message_prefix': 'relay_b', 'timer_period_ms': 300}],
            remappings=[('feature_input', '/same_class_chat')],
            output='screen',
        ),
    ])
