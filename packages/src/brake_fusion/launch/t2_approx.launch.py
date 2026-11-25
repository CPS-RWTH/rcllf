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

"""
T2 with the message_filters approximate fusion.

t2_container.launch.py with fusion:=approximate, so `colcon lf-analyze` gives the
policy its own system (t2_approx). Every other launch argument passes through
with the t2_container default.

rcllf_workers pins the LF worker count of the rcllf program generated from
this file (colcon lf-generate reads its default into the
`workers` target property). A stock launch ignores it. Unpinned, reactor-c
runs min(host cores, 8) workers: 2 on the 2-vCPU CI runner, 8 on a desk host.
On an idle 24-core desk host (10 runs per count) 8 workers broke the
200 ms worker-lag bound in 2 of 10 runs (exit 3, 235.6 and 350.9 ms); 2, 4 and
5 workers (5 = the four /sim/state drains plus the step reaction of a tag) never
did, with a worst event lag of 75.7, 203.7 and 160.5 ms. One worker falls behind
on the runner (341 ms). Two is the smallest count that runs the drains
side by side, and it gives the desk host the runner's schedule.
RCLLF_LF_ARGS="-w N" at launch overrides the pin.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    t2 = PathJoinSubstitution(
        [FindPackageShare('brake_fusion'), 'launch', 't2_container.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'rcllf_workers', default_value='2',
            description='LF worker threads of the generated rcllf program; '
                        'RCLLF_LF_ARGS="-w N" at launch overrides it'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([t2]),
            launch_arguments=[('fusion', 'approximate')],
        ),
    ])
