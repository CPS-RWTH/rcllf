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

# DeclareLaunchArgument with choices=[].

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    timer_period_arg = DeclareLaunchArgument(
        'timer_period_ms',
        default_value='500',
        choices=['200', '500', '1000'],
        description='Timer period in milliseconds.',
    )

    return LaunchDescription([
        timer_period_arg,
        Node(
            package='launch_features_example',
            executable='feature_node',
            name='choices_node',
            output='screen',
            parameters=[{'timer_period_ms': LaunchConfiguration('timer_period_ms')}],
        ),
    ])
