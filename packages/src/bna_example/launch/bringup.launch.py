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

"""Start the anchor sensor, the candidate sensor and the BNA fusion node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def _node(executable, name):
    return Node(package='bna_example', executable=executable, name=name, output='screen')


def generate_launch_description():
    return LaunchDescription([
        _node('anchor_sensor_node', 'anchor_sensor'),
        _node('candidate_sensor_node', 'candidate_sensor'),
        _node('bna_fusion_node', 'bna_fusion'),
    ])
