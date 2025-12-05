#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 찾기
    loc_pkg_dir = get_package_share_directory('localization')       # 내 패키지 (런치 파일용)
    sim_pkg_dir = get_package_share_directory('go1_simulation')     # 남의 패키지 (맵 파일용)
    
    # 2. 맵 파일 경로 설정 (go1_simulation 패키지 안의 maps 폴더를 가리킴)
    # 내 맵으로 하고 싶다면 maps에다가 내 지도 파일을 넣고 경로를 바꾸면 됨
    map_file = os.path.join(sim_pkg_dir, 'maps', 'hospital.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # 1. Map Server
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

        # 2. Odom Localizer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc_pkg_dir, 'launch', 'odom_localizer.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 3. Global Localizer
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc_pkg_dir, 'launch', 'global_localizer.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x': x,
                'y': y,
                'yaw': yaw
            }.items(),
        ),
    ])