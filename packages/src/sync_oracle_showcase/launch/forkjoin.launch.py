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
Fork/join showcase (corpus P3).

fork_source plays the corpus on /fork/src, branch_0 and branch_1 copy each
row after 0 ms and 8 ms of work (SYNC_WORK_SWAP=1 swaps them), and join, an
unmodified ExactTime(3), re-joins the copies by stamp.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sync_oracle_showcase', executable='corpus_player',
            name='fork_source', output='screen',
            remappings=[('/sync/in0', '/fork/src'), ('/sync/in1', '/fork/unused')]),
        # 8 ms: the slow branch's first copy trails the fast one's by at least
        # 8 ms, and 4 rows x 8 ms = 32 ms fit in the 50 ms tick (plan constants).
        Node(
            package='sync_oracle_showcase', executable='branch_worker',
            name='branch_0', output='screen',
            parameters=[{'work_ms': 0, 'work_ms_swapped': 8}],
            remappings=[('/fork/out', '/fork/b0')]),
        Node(
            package='sync_oracle_showcase', executable='branch_worker',
            name='branch_1', output='screen',
            parameters=[{'work_ms': 8, 'work_ms_swapped': 0}],
            remappings=[('/fork/out', '/fork/b1')]),
        Node(
            package='sync_oracle_showcase', executable='exact_sync_node',
            name='join', output='screen', parameters=[{'queue_size': 3}],
            remappings=[('/sync/in0', '/fork/b0'), ('/sync/in1', '/fork/b1')]),
    ])
