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

# Helper used by 15_nested_conditional_include.launch.py.
# This file itself contains a conditional IncludeLaunchDescription, producing
# depth-2 conditional nesting.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_nested_arg = DeclareLaunchArgument(
        'enable_nested',
        default_value='true',
        description='Enable the nested primary node inclusion.',
    )

    primary_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('launch_features_example'),
                'launch',
                '_primary.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('enable_nested')),
    )

    return LaunchDescription([enable_nested_arg, primary_include])
