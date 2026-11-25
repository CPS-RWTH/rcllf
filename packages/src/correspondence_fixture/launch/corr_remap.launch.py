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

"""corr_publisher_one -> corr_sub_2 with /corr/a remapped on both nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='correspondence_fixture',
            executable='corr_publisher_one',
            name='corr_publisher_one',
            output='screen',
            remappings=[('/corr/a', '/corr/a_remapped')],
        ),
        Node(
            package='correspondence_fixture',
            executable='corr_subscriber_a',
            name='corr_sub_2',
            output='screen',
            remappings=[('/corr/a', '/corr/a_remapped')],
        ),
    ])
