'''
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
    # [추가] use_sim_time 설정을 받아옵니다.
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='1.0')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0')
    
    # [추가] use_sim_time 인자를 선언하고 기본값을 'true'로 설정합니다.
    # 이렇게 하면 별도 설정 없이 실행해도 시뮬레이션 시간을 사용합니다.
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    global_localizer_node = Node(
        package='localization',
        executable='global_localizer_node.py', 
        name='global_localizer_node',
        output='screen',
        parameters=[{
            'x': x,
            'y': y,
            'yaw': yaw,
            'use_sim_time': use_sim_time # <--- 핵심: 이 부분이 추가되어야 TF_OLD_DATA 에러가 사라집니다.
        }]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_use_sim_time_cmd) # [추가] 인자 등록
    ld.add_action(global_localizer_node)

    return ld
'''

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Launch Arguments 선언
    return LaunchDescription([
        # (1) 시뮬레이션 시간 사용 여부 (기본값 true)
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # (2) 초기 위치 설정
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # 2. 노드 실행
        Node(
            package='localization',
            executable='global_localizer_node.py',
            name='global_localizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'), 
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw')
            }]
        )
    ])