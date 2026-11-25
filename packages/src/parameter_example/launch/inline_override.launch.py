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
Launch-order fixture.

``launch_ros`` puts ``parameters=[...]`` on the node command line in list
order and writes every inline dict to a parameter file. That file is keyed by
the node's full name only when the launch file gives ``name=`` and
``namespace=``, and by ``/**`` otherwise. rcl lets a node-keyed section beat a
``/**`` section created before it.

- ``node_d`` has no namespace. The file in the middle (key ``/node_d``) beats
  both inline dicts: ``robot_name`` is ``node_d_from_file`` and
  ``update_rate`` stays 7 despite the later inline 8.
- ``node_f`` has a namespace. Its inline dict is keyed ``/fq/node_f`` like
  the file, so the later inline ``update_rate`` wins (8) and ``robot_name``
  is ``node_f_from_file``.

The lfc run of this file must log the same values.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    override = PathJoinSubstitution(
        [FindPackageShare('parameter_example'), 'config', 'd', 'override.yaml'])
    return LaunchDescription([
        Node(
            package='parameter_example',
            executable='parameter_node',
            name='node_d',
            output='screen',
            parameters=[{'robot_name': 'inline_first'}, override, {'update_rate': 8}],
        ),
        Node(
            package='parameter_example',
            executable='parameter_node',
            name='node_f',
            namespace='fq',
            output='screen',
            parameters=[{'robot_name': 'inline_first'}, override, {'update_rate': 8}],
        ),
    ])
