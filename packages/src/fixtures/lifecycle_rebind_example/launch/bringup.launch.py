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

# Lifecycle unbind and re-bind fixture. Under rcllf all five
# nodes run in one binary, so the supervisor drives the sleeper in-process.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='lifecycle_rebind_example', executable=exe, name=name, output='screen')
        for exe, name in (
            ('ticker_node', 'ticker'),
            ('sleeper_node', 'sleeper'),
            ('supervisor_node', 'supervisor'),
            ('listener_node', 'listener'),
            ('responder_node', 'responder'),
        )
    ])
