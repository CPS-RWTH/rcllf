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

"""Top-level launch file starting the publisher and all hazard subscriber nodes."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(launch_file: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qos_hazard_example'),
                'launch',
                launch_file,
            ]),
        ]),
    )


def generate_launch_description():
    return LaunchDescription([
        _include('hazard_publisher_node.launch.py'),
        _include('best_effort_subscriber_node.launch.py'),
        _include('sensor_data_subscriber_node.launch.py'),
        _include('shallow_queue_subscriber_node.launch.py'),
        _include('message_filters_subscriber_node.launch.py'),
        _include('message_filters_cache_node.launch.py'),
        _include('chained_qos_subscriber_node.launch.py'),
    ])
