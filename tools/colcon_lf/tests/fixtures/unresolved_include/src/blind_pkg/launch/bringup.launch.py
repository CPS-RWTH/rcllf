# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='blind_pkg', executable='blind_node', name='blind_node'),
    ])
