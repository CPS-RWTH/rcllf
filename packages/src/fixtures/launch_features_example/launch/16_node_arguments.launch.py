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
A Node's arguments= and ros_arguments= remaps and -p rules.

The first node is named ``arg_node`` but renamed by ``-r __node:=renamed`` in
``arguments``. launch_ros puts ``arguments`` before its own ``-r
__node:=arg_node`` and rcl takes the first matching rule, so the node runs
as ``/renamed``. Its ``feature_output`` is remapped to ``/chat2`` there.
``-p timer_period_ms:=250`` comes before ``parameters=``, so the inline 300
wins. ``-p renamed:message_prefix:=from_args`` targets the node by its
name after remapping and applies; the rules for ``feature_node`` (its name in
the code) and ``arg_node`` (its launch name) match no node and are ignored.

``listener`` takes its namespace from ``-r __ns:=/grp``, which beats
``namespace='/ignored'``, so its topics are ``/grp/feature_input`` and
``/grp/feature_output``. It does not subscribe to ``/chat2``: the analyzer
lists topic endpoints by class name, so two ``feature_node`` instances on one
edge cannot be told apart yet.

``ros_node`` is renamed to ``x`` by ``ros_arguments``, which launch_ros also
puts before its own ``-r __node:=ros_node``, so it runs as ``/ra/x`` with
``timer_period_ms`` 700.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='launch_features_example',
            executable='feature_node',
            name='arg_node',
            arguments=[
                '--ros-args', '-r', '__node:=renamed', '-r', 'feature_output:=chat2',
                '-p', 'timer_period_ms:=250', '-p', 'renamed:message_prefix:=from_args',
                '-p', 'feature_node:message_prefix:=code_name',
                '-p', 'arg_node:timer_period_ms:=1',
            ],
            parameters=[{'timer_period_ms': 300}],
            output='screen',
        ),
        Node(
            package='launch_features_example',
            executable='feature_node',
            name='listener',
            namespace='/ignored',
            arguments=['--ros-args', '-r', '__ns:=/grp'],
            output='screen',
        ),
        Node(
            package='launch_features_example',
            executable='feature_node',
            name='ros_node',
            namespace='/ra',
            ros_arguments=['-r', '__node:=x', '-p', 'timer_period_ms:=700'],
            output='screen',
        ),
    ])
