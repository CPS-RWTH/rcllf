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

"""Causality loop: all links d = 0, the /corr/pong callback publishes a ping."""

# colcon lf-generate refuses this system with a zero-delay causality loop error.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='correspondence_fixture',
            executable='corr_ping_echo',
            name='corr_ping_echo',
            output='screen',
        ),
        Node(
            package='correspondence_fixture',
            executable='corr_pong',
            name='corr_pong',
            output='screen',
        ),
    ])
