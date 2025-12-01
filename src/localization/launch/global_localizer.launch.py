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

'''
#수정예정인 코드
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

# ---------------------------
# [수정/추가 예시]
# - 자동 보정 기능을 넣으려면 launch 인자 대신 노드에서 실시간 pose를 받아서 처리
# - launch 파일은 그대로 두고, 노드에서 동적으로 pose를 갱신
# ---------------------------
'''