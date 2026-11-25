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

# IncludeLaunchDescription with IfCondition (conditional include).
# IncludeLaunchDescription with UnlessCondition (explicit).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_primary_arg = DeclareLaunchArgument(
        'enable_primary',
        default_value='true',
        description='Start primary node when true, secondary node when false.',
    )

    pkg_share = FindPackageShare('launch_features_example')

    primary_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', '_primary.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('enable_primary')),
    )

    secondary_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', '_secondary.launch.py'])
        ),
        condition=UnlessCondition(LaunchConfiguration('enable_primary')),
    )

    return LaunchDescription([enable_primary_arg, primary_include, secondary_include])
