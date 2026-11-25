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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot_1',
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
    )

    use_multi_service_arg = DeclareLaunchArgument(
        'use_multi_service',
        default_value='false',
        description='Launch multi_service_example (server + client)',
    )

    use_action_full_arg = DeclareLaunchArgument(
        'use_action_full',
        default_value='false',
        description='Launch action_full_example (server + client with cancel/rejection)',
    )

    use_reentrant_fanout_arg = DeclareLaunchArgument(
        'use_reentrant_fanout',
        default_value='false',
        description='Launch reentrant_fanout_example (publisher + reentrant worker)',
    )

    # Namespaced group containing pubsub and timer examples
    namespaced_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('pubsub_example'),
                    'launch',
                    'bringup.launch.py',
                ])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('timer_example'),
                    'launch',
                    'timer_node.launch.py',
                ])
            ),
        ),
    ])

    # Service example at root level (no namespace)
    service_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('service_example'),
                'launch',
                'bringup.launch.py',
            ])
        ),
    )

    # Parameter example with system_params.yaml override
    parameter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('parameter_example'),
                'launch',
                'parameter_node.launch.py',
            ])
        ),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                FindPackageShare('system_bringup'),
                'config',
                'system_params.yaml',
            ]),
        }.items(),
    )

    # Conditional sim timer node
    sim_timer_node = Node(
        package='timer_example',
        executable='timer_node',
        name='sim_timer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim')),
    )

    # Optional: multi_service_example
    multi_service_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('multi_service_example'),
                'launch',
                'bringup.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_multi_service')),
    )

    # Optional: action_full_example
    action_full_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('action_full_example'),
                'launch',
                'bringup.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_action_full')),
    )

    # Optional: reentrant_fanout_example
    reentrant_fanout_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('reentrant_fanout_example'),
                'launch',
                'bringup.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_reentrant_fanout')),
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_arg,
        use_multi_service_arg,
        use_action_full_arg,
        use_reentrant_fanout_arg,
        namespaced_group,
        service_bringup,
        parameter_launch,
        sim_timer_node,
        multi_service_bringup,
        action_full_bringup,
        reentrant_fanout_bringup,
    ])
