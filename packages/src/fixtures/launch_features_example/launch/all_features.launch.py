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

# Comprehensive bringup: includes all 15 numbered launch files, covering every
# launch construct tracked by colcon lf-analyze.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('launch_features_example')

    # Shared arguments forwarded to the relevant included files
    enable_primary_arg = DeclareLaunchArgument(
        'enable_primary',
        default_value='true',
        description='Passed to 03_conditions: use primary node.',
    )
    timer_period_arg = DeclareLaunchArgument(
        'timer_period_ms',
        default_value='500',
        choices=['200', '500', '1000'],
        description='Passed to 02_arg_with_choices and 12_opaque_function.',
    )
    use_mt_arg = DeclareLaunchArgument(
        'use_mt',
        default_value='false',
        description='Passed to 09_set_launch_configuration: use MT container.',
    )
    enable_group_arg = DeclareLaunchArgument(
        'enable_group',
        default_value='true',
        description='Passed to 13_group_action_features: enable conditional group.',
    )
    enable_inner_arg = DeclareLaunchArgument(
        'enable_inner',
        default_value='true',
        description='Passed to 14_nested_group_action: enable inner group.',
    )
    enable_nested_arg = DeclareLaunchArgument(
        'enable_nested',
        default_value='true',
        description='Passed to 15_nested_conditional_include: enable nested include.',
    )

    def inc(filename, args=None):
        src = PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', filename])
        )
        if args:
            return IncludeLaunchDescription(src, launch_arguments=args.items())
        return IncludeLaunchDescription(src)

    return LaunchDescription([
        enable_primary_arg,
        timer_period_arg,
        use_mt_arg,
        enable_group_arg,
        enable_inner_arg,
        enable_nested_arg,
        # namespace + remappings on same node
        inc('01_node_namespace_remap.launch.py'),
        # DeclareLaunchArgument with choices
        inc('02_arg_with_choices.launch.py',
            {'timer_period_ms': LaunchConfiguration('timer_period_ms')}),
        # IncludeLaunchDescription with IfCondition / UnlessCondition
        inc('03_conditions.launch.py',
            {'enable_primary': LaunchConfiguration('enable_primary')}),
        # respawn + respawn_delay
        inc('04_respawn.launch.py'),
        # multiple YAML parameter files
        inc('05_multi_param_files.launch.py'),
        # SetEnvironmentVariable + AppendEnvironmentVariable
        inc('06_environment_variables.launch.py'),
        # ExecuteProcess
        inc('07_execute_process.launch.py'),
        # TimerAction
        inc('08_timer_action.launch.py'),
        # SetLaunchConfiguration conditional switching
        inc('09_set_launch_configuration.launch.py',
            {'use_mt': LaunchConfiguration('use_mt')}),
        # component_container_mt
        inc('10_container_mt.launch.py'),
        # LoadComposableNodes targeting external container
        inc('11_load_composable_nodes.launch.py'),
        # OpaqueFunction
        inc('12_opaque_function.launch.py',
            {'timer_period_ms': LaunchConfiguration('timer_period_ms')}),
        # conditional GroupAction, SetParameter, scoped=False
        inc('13_group_action_features.launch.py',
            {'enable_group': LaunchConfiguration('enable_group')}),
        # nested GroupAction
        inc('14_nested_group_action.launch.py',
            {'enable_inner': LaunchConfiguration('enable_inner')}),
        # depth-2 conditional include nesting
        inc('15_nested_conditional_include.launch.py',
            {'enable_nested': LaunchConfiguration('enable_nested')}),
    ])
