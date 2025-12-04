"""
Perception Launch File

This launch file starts the perception node.
The node subscribes to camera and image information and publishes the image with 
bounding boxes, box labels and distances, and speech information.

Usage:
    ros2 launch perception perception_launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for perception node.
    
    Returns:
        LaunchDescription: Launch description containing the perceptionr node
    """
    #Get perception package directory
    perception_pkg_dir = get_package_share_directory('perception')
    #Path to Perception configuration file
    config_file = os.path.join(perception_pkg_dir, 'config', 'perception_config.yaml')
    #Path to Models directory
    models_dir = os.path.join(perception_pkg_dir, 'models')
    
    # Create path tracker node
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception',
        output='screen',
        parameters=[
            config_file,
            {'model_dir': models_dir}
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        perception_node
    ])