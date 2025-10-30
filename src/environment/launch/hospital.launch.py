import os
import sys

import launch
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('aws_robomaker_hospital_world')
    world_file_name = "hospital.world"
    world = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # Gazebo 모델 경로 설정
    models_path = os.path.join(pkg_share, 'models')
    fuel_models_path = os.path.join(pkg_share, 'fuel_models')
    
    # 기존 GZ_SIM_RESOURCE_PATH와 시스템 기본 경로 추가
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_default_paths = '/usr/share/gz'
    
    if gz_resource_path:
        gz_resource_path = f"{models_path}:{fuel_models_path}:{gz_default_paths}:{gz_resource_path}"
    else:
        gz_resource_path = f"{models_path}:{fuel_models_path}:{gz_default_paths}"
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # ROS2 Jazzy uses Gazebo (formerly Ignition)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world, '-r'],
        output='screen'
    )

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[world, ''],
          description='SDF world file'),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        set_gz_resource_path,
        gz_sim
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
