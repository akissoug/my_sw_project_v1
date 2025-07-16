#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directory
    pkg_dir = FindPackageShare('seawings_mission_management')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'mission_params.yaml']),
        description='Path to mission configuration file'
    )
    
    vehicle_arg = DeclareLaunchArgument(
        'vehicle',
        default_value='iris',
        description='Vehicle model for simulation'
    )
    
    # Include PX4 SITL launch
    px4_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'px4_sitl_gazebo.launch.py'])
        ),
        launch_arguments={
            'vehicle': LaunchConfiguration('vehicle'),
            'world': 'empty',
            'headless': 'false'
        }.items()
    )
    
    # Include mission management launch (delayed to allow PX4 to start)
    mission_launch = TimerAction(
        period=180.0,  # Wait 10 seconds for PX4 to start
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_dir, 'launch', 'seawings_mission.launch.py'])
                ),
                launch_arguments={
                    'config_file': LaunchConfiguration('config_file'),
                    'namespace': 'seawings'
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        vehicle_arg,
        LogInfo(msg=['Starting complete SEAWINGS system...']),
        px4_sitl_launch,
        mission_launch,
        LogInfo(msg=['Complete SEAWINGS system launched'])
    ])