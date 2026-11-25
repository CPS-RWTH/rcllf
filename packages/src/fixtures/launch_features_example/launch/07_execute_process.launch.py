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

# ExecuteProcess launching a non-ROS process.

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['echo', 'launch_features_example ready'],
            name='readiness_check',
            output='screen',
        ),
        Node(
            package='launch_features_example',
            executable='feature_node',
            name='exec_process_node',
            output='screen',
        ),
    ])
