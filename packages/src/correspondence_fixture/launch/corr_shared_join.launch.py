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

"""Shared join: corr_sub_2's join holds the tag for corr_fanin_relay."""

# Run with CORR_SCENARIO=two_publishers. corr_publisher publishes /corr/a and
# /corr/b every tick. corr_sub_2 reads /corr/a with a long take timeout.
# corr_fanin_relay reads /corr/a and /corr/b and republishes /corr/b on /corr/j,
# which corr_relay_sink reads (remapped from /corr/r).

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='correspondence_fixture',
            executable='corr_publisher',
            name='corr_publisher',
            output='screen',
        ),
        Node(
            package='correspondence_fixture',
            executable='corr_subscriber_a',
            name='corr_sub_2',
            output='screen',
        ),
        Node(
            package='correspondence_fixture',
            executable='corr_fanin_relay',
            name='corr_fanin_relay',
            output='screen',
        ),
        Node(
            package='correspondence_fixture',
            executable='corr_relay_sink',
            name='corr_relay_sink',
            output='screen',
            remappings=[('/corr/r', '/corr/j')],
        ),
    ])
