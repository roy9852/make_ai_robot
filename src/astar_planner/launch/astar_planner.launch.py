import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('astar_planner')
    
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_dir, 'maps', 'example_map.txt'),
        description='Path to the map file'
    )
    
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='1.5',
        description='Robot start X position'
    )
    
    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value='1.5',
        description='Robot start Y position'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='1.0',
        description='Map resolution in meters'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Use Gazebo mode (true) or virtual simulator mode (false)'
    )
    
    # Simulator Node
    simulator_node = Node(
        package='astar_planner',
        executable='simulator_node',
        name='simulator_node',
        output='screen',
        parameters=[{
            'use_gazebo': LaunchConfiguration('use_gazebo'),
            'map_file': LaunchConfiguration('map_file'),
            'resolution': LaunchConfiguration('resolution'),
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
        }]
    )
    
    # Path Planner Node
    path_planner_node = Node(
        package='astar_planner',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
        }]
    )
    
    # RViz Node
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'astar_planner.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        map_file_arg,
        start_x_arg,
        start_y_arg,
        resolution_arg,
        use_rviz_arg,
        use_gazebo_arg,
        simulator_node,
        path_planner_node,
        rviz_node,
    ])

