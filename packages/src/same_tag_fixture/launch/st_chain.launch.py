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

"""Same-tag test CHAIN: relay of ONE split in pre -> /b -> relay."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start tick, the stage node(s), sink and msink."""
    return LaunchDescription([
        Node(
            package='same_tag_fixture',
            executable='st_source',
            name='tick',
            output='screen',
        ),
        Node(
            package='same_tag_fixture',
            executable='st_stage_in',
            name='pre',
            output='screen',
        ),
        Node(
            package='same_tag_fixture',
            executable='st_stage_out',
            name='relay',
            output='screen',
        ),
        Node(
            package='same_tag_fixture',
            executable='st_sink',
            name='sink',
            output='screen',
        ),
        Node(
            package='same_tag_fixture',
            executable='st_multi_sink',
            name='msink',
            output='screen',
        ),
    ])
