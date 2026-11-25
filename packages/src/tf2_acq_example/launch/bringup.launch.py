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


"""Start the /tf source and the acquisition sampler."""

from launch import LaunchDescription
from launch_ros.actions import Node


def _node(executable, name):
    return Node(package='tf2_acq_example', executable=executable, name=name, output='screen')


def generate_launch_description():
    return LaunchDescription([
        _node('tf_source_node', 'tf_source'),
        _node('acq_sampler_node', 'acq_sampler'),
    ])
