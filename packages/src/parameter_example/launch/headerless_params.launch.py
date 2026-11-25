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
Headerless node with a params file and an inline dict.

``headerless_param_node`` has no header, so the lfc generator builds it
through the headerless factory. The node is named and namespaced, so
launch_ros files the inline dict under ``/hl/headerless`` and, being later
in the list, it wins over ``e/headerless.yaml`` for ``label`` and ``gain``.
Expected values: label=inline_label, gain=3.5, count=21 (file).
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    e_params = PathJoinSubstitution(
        [FindPackageShare('parameter_example'), 'config', 'e', 'headerless.yaml'])
    return LaunchDescription([
        Node(
            package='parameter_example',
            executable='headerless_param_node',
            name='headerless',
            namespace='hl',
            output='screen',
            parameters=[e_params, {'label': 'inline_label', 'gain': 3.5}],
        ),
    ])
