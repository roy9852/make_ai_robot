#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch Gazebo world with the robot.
    Start sensors (RGB camera, depth camera, IMU, etc.) and motors (joint controllers) for the robot.
    Publish pointcloud from the RGB-D cameras (face and top)
    Depending on the launch arguments, it can also use GT pose of the robot for localization.
    """
    # Constants for paths to different files and folders
    go1_pkg_name = 'go1_simulation'
    world_pkg_name = 'aws_robomaker_hospital_world'

    # Default values for launch arguments
    default_robot_name = 'go1'
    models_dir_name = 'models'
    fuel_models_dir_name = 'fuel_models'
    worlds_dir_name = 'worlds'
    default_world_file_name = 'hospital.world'    

    # Path to the ROS-Gazebo bridge configuration file
    ros_gz_bridge_config_file_name = 'config/ros_gz_bridge.yaml'

    # Set the path to different files and folders
    # Go1 simulation package
    go1_pkg_share_dir = FindPackageShare(package=go1_pkg_name).find(go1_pkg_name)
    default_ros_gz_bridge_config_file = os.path.join(go1_pkg_share_dir, ros_gz_bridge_config_file_name)
    # Gazebo simulation package
    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')    
    # Gazebo world and model packge
    world_pkg_share_dir = FindPackageShare(package=world_pkg_name).find(world_pkg_name)
    models_dir = os.path.join(world_pkg_share_dir, models_dir_name)
    fuel_models_dir = os.path.join(world_pkg_share_dir, fuel_models_dir_name)

    # Export GZ_SIM_RESOURCE_PATH
    # Without GZ_SIM_RESOURCE_PATH, Gazebo will not find the models and fuel models.
    gz_resource_path = f"{models_dir}:{fuel_models_dir}:/opt/ros/jazzy/share"

    # Launch configuration variables
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file_name = LaunchConfiguration('world_file_name')
    use_gt_pose = LaunchConfiguration('use_gt_pose')
    use_gpu = LaunchConfiguration('use_gpu')
    spawn_delay = LaunchConfiguration('spawn_delay')


    # Set the pose configuration variables
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='The name for the robot')    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_file_name_cmd = DeclareLaunchArgument(
        name='world_file_name',
        default_value=default_world_file_name,
        description='World file name (e.g., hospital.world, empty.world, cafe.world)')

    declare_use_gt_pose_cmd = DeclareLaunchArgument(
        name='use_gt_pose',
        default_value='false',
        description='Flag to enable using GT pose for localization')

    declare_use_gpu_cmd = DeclareLaunchArgument(
        name='use_gpu',
        default_value='true',
        description='Flag to enable using GPU for rendering')
    
    declare_spawn_delay_cmd = DeclareLaunchArgument(
        name='spawn_delay',
        default_value='50.0',
        description='Delay (sec) before spawning robot into Gazebo')

    # Pose arguments
    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='x component of initial position, meters')

    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='1.0',
        description='y component of initial position, meters')

    declare_z_cmd = DeclareLaunchArgument(
        name='z',
        default_value='0.5',
        description='z component of initial position, meters')

    declare_roll_cmd = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')

    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')

    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='yaw angle of initial orientation, radians')

    # Set Gazebo to use GPU
    # Force OpenGL to prefer the dedicated GPU
    set_gpu_render_cmd = SetEnvironmentVariable(
        name='MESA_DEDICATE_DEVICE_ID',
        value='0', 
        condition=IfCondition(use_gpu)
    )

    set_nv_offload_cmd = SetEnvironmentVariable(
        name='__NV_PRIME_RENDER_OFFLOAD',
        value='1',
        condition=IfCondition(use_gpu)
    )

    set_glx_vendor_cmd = SetEnvironmentVariable(
        name='__GLX_VENDOR_LIBRARY_NAME',
        value='nvidia',
        condition=IfCondition(use_gpu)
    )

    # Set Gazebo resource path
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )    

    # Launch Gazebo world
    # 'world_file' changes with launch argument 'world_file_name'
    world_file = PathJoinSubstitution([world_pkg_share_dir, worlds_dir_name, world_file_name])
    start_gazebo_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r ', world_file])]
    )


    # Bridge ROS topics and Gazebo messages for establishing communication
    # This enables sensors (Gazebo -> ROS) and motors (ROS -> Gazebo). 
    # Sensor configs are defined in 'config/ros_gz_bridge.yaml'
    # Motor configs are defined in 'urdf/control/go1_ros2_control.urdf.xacro', and 'urdf/control/gazebo_sim_ros2_control.urdf.xacro'
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_ros_gz_bridge_config_file,
        }],
        output='screen'
    )

    # Spawn the robot in ROS2
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(go1_pkg_share_dir, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Load the ROS 2 controllers (joint controllers)
    # Joint controller itself is independent of Gazebo. 
    # It is only responsible for sending commands to the motor topic in ROS2.
    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(go1_pkg_share_dir, 'launch', 'load_ros2_controllers.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn the robot in Gazebo
    # Based on the '/robot_description' topic, Gazebo will spawn the robot in the world.
    # It also connect ROS2 joint command from the joint controllers to the motor topic in Gazebo.
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ])
    
    delayed_gazebo_ros_spawner_cmd = TimerAction(
        period=spawn_delay,
        actions=[start_gazebo_ros_spawner_cmd]
    )   

    # Publish the pointcloud from the depth camera (face and top)
    go1_pointcloud_publisher_cmd = Node(
        package='go1_simulation',
        executable='publish_pointcloud.py',
        output='screen',
    )        
        
    # Publish the GT pose of the robot
    # When use_gt_pose is true, set comparison to false (publishes to _gt topics)
    go1_gt_pose_publisher_cmd = Node(
        package='go1_simulation',
        executable='go1_gt_pose_publisher.py',
        output='screen',
        parameters=[{'comparison': False}],
        condition=IfCondition(use_gt_pose)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_name_cmd)
    ld.add_action(declare_use_gt_pose_cmd)
    ld.add_action(declare_use_gpu_cmd)
    ld.add_action(declare_spawn_delay_cmd)

    # Add pose arguments
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add the actions to the launch description
    ld.add_action(set_gpu_render_cmd)
    ld.add_action(set_nv_offload_cmd)
    ld.add_action(set_glx_vendor_cmd)
    ld.add_action(set_gz_resource_path)
    ld.add_action(start_gazebo_world_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(load_controllers_cmd)
    # ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(delayed_gazebo_ros_spawner_cmd)
    ld.add_action(go1_pointcloud_publisher_cmd)
    ld.add_action(go1_gt_pose_publisher_cmd)

    return ld
