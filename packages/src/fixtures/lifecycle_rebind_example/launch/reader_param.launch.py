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

# reader.launch.py with the twin reader, whose second subscription's
# topic comes from the launch parameter twin_topic.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='lifecycle_rebind_example', executable='feeder_node', name='feeder',
             output='screen'),
        Node(package='lifecycle_rebind_example', executable='param_reader_node', name='reader',
             output='screen', parameters=[{'twin_topic': '/rebind/feed'}]),
        Node(package='lifecycle_rebind_example', executable='driver_node', name='driver',
             output='screen'),
    ])
