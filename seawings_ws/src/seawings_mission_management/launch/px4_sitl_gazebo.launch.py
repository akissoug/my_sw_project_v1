#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    vehicle_arg = DeclareLaunchArgument(
        'vehicle',
        default_value='iris',
        description='Vehicle model for simulation, WIG craft'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world file'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    
    # Configuration
    vehicle = LaunchConfiguration('vehicle')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    
    # PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gazebo'],
        cwd='/home/soug/PX4-Autopilot',  # check it again
        output='screen',
        emulate_tty=True
    )
    
    # MAVROS Node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[{
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0'
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Micro XRCE-DDS Agent
    xrce_dds_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        vehicle_arg,
        world_arg,
        headless_arg,
        LogInfo(msg=['Starting PX4 SITL with Gazebo...']),
        px4_sitl,
        mavros_node,
        xrce_dds_agent,
        LogInfo(msg=['PX4 SITL simulation started'])
    ])