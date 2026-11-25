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
Publisher and subscriber with node names of 40 characters or more.

lfc keeps only the tail of long generated C names, so under
``--federate per-node`` a federate named after a long node did not compile.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_example',
            executable='publisher_node',
            name='publisher_node_with_a_name_of_forty_or_more_chars',
            output='screen',
        ),
        Node(
            package='pubsub_example',
            executable='subscriber_node',
            name='subscriber_node_with_a_name_of_forty_or_more_chars',
            output='screen',
        ),
    ])
