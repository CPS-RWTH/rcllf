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

# A lifecycle subscriber whose subscription is made in on_configure
# and reset in on_cleanup, fed by a lifecycle publisher and moved through
# configure, activate, cleanup, configure, activate by the driver.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='lifecycle_rebind_example', executable=exe, name=name, output='screen')
        for exe, name in (
            ('feeder_node', 'feeder'),
            ('reader_node', 'reader'),
            ('driver_node', 'driver'),
        )
    ])
