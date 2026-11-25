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
Parameter-precedence fixture (resolved configuration model).

Two instances of ``parameter_node`` (declares ``robot_name``, ``max_speed``,
``update_rate``, ``debug_mode``; C++ defaults in ``src/parameter_node.cpp``)
exercise four precedence axes, each independently:

- A1 (C++ default vs params file): ``max_speed`` — only set in ``a/params.yaml``.
- A2 (earlier vs later params file): ``update_rate`` — set in both
  ``a/params.yaml`` (7) and ``a/late.yaml`` (42, listed later); later wins.
- A3 (params file vs inline ``parameters=`` dict): ``debug_mode``. The file
  says ``false`` and the inline dict says ``true``. The file wins (verified
  against a live ``ros2 param get``). launch_ros keeps the list order, but
  ``node_a`` has no ``namespace=``, so the inline dict's generated params
  file is keyed ``/**``. rcl applies every ``/**`` value before the
  ``node_a`` section, which ``a/params.yaml`` created after its own ``/**``
  section, so the file's ``node_a`` value wins.
- A4 (``/**`` wildcard vs an explicit node key, in the same file):
  ``robot_name`` — ``a/params.yaml`` sets it under both ``/**`` and
  ``node_a``; the explicit node key wins.

``node_b`` loads only ``b/params.yaml``, which is a different file from
``a/params.yaml`` despite sharing the same basename ("params.yaml"), living
in a different directory — this is the same-basename cache collision case
(maintainer decision P2): the two nodes must resolve different ``robot_name``
values from their own same-named file, not share one cached copy.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('parameter_example')
    a_params = PathJoinSubstitution([pkg_share, 'config', 'a', 'params.yaml'])
    a_late = PathJoinSubstitution([pkg_share, 'config', 'a', 'late.yaml'])
    b_params = PathJoinSubstitution([pkg_share, 'config', 'b', 'params.yaml'])

    return LaunchDescription([
        Node(
            package='parameter_example',
            executable='parameter_node',
            name='node_a',
            output='screen',
            parameters=[a_params, a_late, {'debug_mode': True}],
        ),
        Node(
            package='parameter_example',
            executable='parameter_node',
            name='node_b',
            output='screen',
            parameters=[b_params],
        ),
    ])
