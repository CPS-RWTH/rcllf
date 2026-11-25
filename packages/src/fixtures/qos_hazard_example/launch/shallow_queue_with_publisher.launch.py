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

"""Minimal launch isolating RCLLF-H002 (depth-1 subscription queue)."""

# RCLLF-H002 only fires when a depth-1 subscription's topic also has an in-model
# publisher, so the standalone ``shallow_queue_subscriber_node.launch.py`` never
# triggers it. ``bringup.launch.py`` does, but it also starts
# ``message_filters_subscriber_node``, whose RCLLF-H003 is ERROR-severity and
# makes the generate hazard-gate reject the whole graph — masking H002's own WARN
# outcome.
#
# This launch starts exactly the publisher and the depth-1 subscriber, so
# ``qos.shallow_queue``'s playground case can assert WARN in isolation.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qos_hazard_example',
            executable='hazard_publisher_node',
            name='hazard_publisher_node',
            output='screen',
        ),
        Node(
            package='qos_hazard_example',
            executable='shallow_queue_subscriber_node',
            name='shallow_queue_subscriber_node',
            output='screen',
        ),
    ])
