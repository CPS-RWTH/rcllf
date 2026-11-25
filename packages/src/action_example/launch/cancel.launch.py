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

# The action server and a client that cancels its goal after the
# third feedback sample. The node name stays short: lfc truncates generated C
# names to 40 characters, and a federate whose instance path ends in the same
# long name twice does not compile.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('action_example')

    server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'action_server.launch.py')
        )
    )

    return LaunchDescription([
        server_launch,
        Node(
            package='action_example',
            executable='action_cancel_client_node',
            name='cancel_client',
            namespace='action_example',
            output='screen',
        ),
    ])
