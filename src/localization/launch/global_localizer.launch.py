#!/usr/bin/env python3

"""
Launch file for global localization node

This launch file starts the global localization node with parameters (initial pose)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('localization')

    # Get launch arguments
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    # Declare launch arguments
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='1.0')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0')

    global_localizer_node = Node(
        package='localization',
        executable='global_localizer_node.py',
        name='global_localizer_node',
        output='screen',
        parameters=[{
            'x': x,
            'y': y,
            'yaw': yaw
        }]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(global_localizer_node)

    return ld