#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('localization')
    map_file = os.path.join(pkg_dir, 'maps', 'map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # 1. Map Server (Nav2 Map Server는 사용 허용된다고 가정)
        # 만약 이것도 안 되면 utils.py의 map_to_pcd 기능을 확장해서 map.yaml을 읽는 노드를 짜야 함
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename': map_file}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        # 2. Odom Localizer (ICP + IMU) - 님이 만든 거
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'odom_localizer.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 3. Global Localizer (Custom Particle Filter) - 방금 만든 거
        # 이제 nav2_amcl을 쓰지 않고 직접 만든 노드를 씁니다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'global_localizer.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x': x,
                'y': y,
                'yaw': yaw
            }.items(),
        ),
    ])