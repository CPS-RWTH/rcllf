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

"""Two writers fan in to one take-mode skip subscription."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='take_skip_fixture',
            executable='take_writer_node',
            name='take_writer_a',
            parameters=[{'id_base': 0}],
            output='screen',
        ),
        Node(
            package='take_skip_fixture',
            executable='take_writer_node',
            name='take_writer_b',
            parameters=[{'id_base': 1000}],
            output='screen',
        ),
        Node(
            package='take_skip_fixture',
            executable='take_reader_node',
            name='take_reader',
            output='screen',
        ),
    ])
