#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # List of all 12 joint controllers
    controllers = [
        'joint_state_broadcaster',
        'FR_hip_controller',
        'FR_thigh_controller',
        'FR_calf_controller',
        'FL_hip_controller',
        'FL_thigh_controller',
        'FL_calf_controller',
        'RR_hip_controller',
        'RR_thigh_controller',
        'RR_calf_controller',
        'RL_hip_controller',
        'RL_thigh_controller',
        'RL_calf_controller',
    ]
    
    # Create launch commands for each controller
    controller_spawners = []
    for controller in controllers:
        controller_spawners.append(
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
                output='screen'
            )
        )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add all controller spawners
    for spawner in controller_spawners:
        ld.add_action(spawner)
    
    return ld
