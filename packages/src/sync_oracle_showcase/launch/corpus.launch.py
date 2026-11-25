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
Sync corpus showcase: one corpus player, five synchronizers.

The player reads the corpus named by SYNC_CORPUS. Every synchronizer instance
sees the same rows; lf_eval.sync_oracle compares each instance's RCLLF_SYNC
stream with oracle O1 at the instance's policy and queue size.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# (executable, node name, queue size). q 1 evicts the just-inserted entry, q 2
# and q 3 are the smallest sizes where clear-old and overflow interleave
# (plan constants).
SYNCS = [
    ('exact_sync_node', 'exact_q1', 1),
    ('exact_sync_node', 'exact_q2', 2),
    ('exact_sync_node', 'exact_q3', 3),
    ('approx_sync_node', 'approx_q2', 2),
    ('approx_sync_node', 'approx_q3', 3),
]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sync_oracle_showcase', executable='corpus_player',
            name='corpus_player', output='screen'),
    ] + [
        Node(
            package='sync_oracle_showcase', executable=exe, name=name, output='screen',
            parameters=[{'queue_size': q}])
        for exe, name, q in SYNCS
    ])
