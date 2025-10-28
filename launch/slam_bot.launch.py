#!/usr/bin/env python3
"""
SLAM Bot Launch File

Minimal ROS2 launch file for autonomous SLAM robot.
Launches the slam_bot node with configuration parameters.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generate launch description for SLAM bot.
    
    Returns:
        LaunchDescription with all nodes and configurations
    """
    
    # Get package share directory
    pkg_share = FindPackageShare('autonomous_slam_robot_ros2').find('autonomous_slam_robot_ros2')
    
    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('autonomous_slam_robot_ros2'),
        'config',
        'params.yaml'
    ])
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=config_file,
        description='Full path to the ROS2 parameters file to use'
    )
    
    # Create SLAM bot node
    slam_bot_node = Node(
        package='autonomous_slam_robot_ros2',
        executable='slam_bot',
        name='slam_bot_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/scan', '/scan'),
            ('/odom', '/odom'),
            ('/map', '/map')
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    
    # Add nodes
    ld.add_action(slam_bot_node)
    
    return ld
