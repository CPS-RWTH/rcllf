# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='param_topic_pkg',
            executable='param_topic_node',
            name='param_topic_node',
            parameters=[{'output_topic': 'fixture_cmd'}],
        ),
    ])
