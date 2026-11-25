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

# An included file (_helper_nested) that itself contains a conditional
# IncludeLaunchDescription — depth-2 conditional nesting.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_nested_arg = DeclareLaunchArgument(
        'enable_nested',
        default_value='true',
        description='Forwarded to helper to conditionally include the primary node.',
    )

    helper_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('launch_features_example'),
                'launch',
                '_helper_nested.launch.py',
            ])
        ),
        launch_arguments={
            'enable_nested': LaunchConfiguration('enable_nested'),
        }.items(),
    )

    return LaunchDescription([enable_nested_arg, helper_include])
