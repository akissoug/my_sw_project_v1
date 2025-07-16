#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('seawings_mission_management')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'mission_params.yaml'),
        description='Path to mission configuration file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='seawings',
        description='ROS namespace for the mission nodes'
    )
    
    # Configuration file path
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    
    # Power Monitor Node
    power_monitor_node = Node(
        package='seawings_mission_management',
        executable='power_monitor',
        name='power_monitor',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0
    )
    
    # Fault Detector Node
    fault_detector_node = Node(
        package='seawings_mission_management',
        executable='fault_detector',
        name='fault_detector',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0
    )
    
    # Mission Supervisor Node
    mission_supervisor_node = Node(
        package='seawings_mission_management',
        executable='mission_supervisor',
        name='mission_supervisor',
        namespace=namespace,
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0
    )
    
    # Launch description
    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        LogInfo(msg=['Launching SEAWINGS Mission Management System...']),
        power_monitor_node,
        fault_detector_node,
        mission_supervisor_node,
        LogInfo(msg=['SEAWINGS Mission Management System launched successfully'])
    ])