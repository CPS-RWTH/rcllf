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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    service_pkg_dir = get_package_share_directory('service_example')
    blocking_pkg_dir = get_package_share_directory('blocking_service_client_example')

    server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(service_pkg_dir, 'launch', 'service_server.launch.py')
        )
    )

    spin_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(blocking_pkg_dir, 'launch', 'blocking_spin_client.launch.py')
        )
    )

    future_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(blocking_pkg_dir, 'launch', 'blocking_future_client.launch.py')
        )
    )

    return LaunchDescription([
        server_launch,
        spin_client_launch,
        future_client_launch,
    ])
