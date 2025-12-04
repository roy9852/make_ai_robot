#!/usr/bin/env python3

"""
Launch file for odom localization node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    odom_localizer_node = Node(
        package='localization',
        executable='odom_localizer_node.py',
        name='odom_localizer_node',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(odom_localizer_node)

    return ld
