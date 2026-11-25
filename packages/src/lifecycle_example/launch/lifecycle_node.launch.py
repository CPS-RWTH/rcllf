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

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    # The node drives itself to `active` in main() (configure, then activate),
    # so the launch file sends no ChangeState events: a configure request would
    # arrive at an already active node and fail.
    lifecycle_node = LifecycleNode(
        package='lifecycle_example',
        executable='lifecycle_node',
        name='lifecycle_node',
        namespace='',
        output='screen',
    )

    return LaunchDescription([
        lifecycle_node,
    ])
