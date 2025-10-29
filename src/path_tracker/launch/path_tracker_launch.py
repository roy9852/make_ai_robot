from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the launch arguments
    algo_arg = 'mppi'

    # Get package directories using FindPackageShare
    # visky_pkg_dir = get_package_share_directory('visky')
    path_tracker_pkg_dir = get_package_share_directory('path_tracker')
    
    # Create configuration file paths using PathJoinSubstitution
    # visky_topic_config = PathJoinSubstitution([visky_pkg_dir, 'config', 'topic_names_config.yaml'])
    # visky_config = PathJoinSubstitution([visky_pkg_dir, 'config', 'config.yaml'])
    # platform_config = os.path.join(path_tracker_pkg_dir, 'config', f'{platform_arg}.yaml')
    algo_config = os.path.join(path_tracker_pkg_dir, 'config', f'{algo_arg}.yaml')
    print(algo_config)
    
    # Create node
    path_tracker_node = Node(
        package='path_tracker',
        executable='path_tracker_node',
        name='path_tracker',
        output='screen',
        parameters=[
            algo_config,
        ]
    )

    return LaunchDescription([
        path_tracker_node
    ])