#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('kiss_matcher_ros')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'relocalization_config.yaml'),
        description='Path to the relocalization configuration file'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to the prior PCD map file'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/velodyne_points',
        description='Point cloud topic to subscribe to'
    )
    
    # Relocalization node
    relocalization_node = Node(
        package='kiss_matcher_ros',
        executable='relocalization_node',
        name='relocalization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'map_file_path': LaunchConfiguration('map_file'),
                'scan_topic': LaunchConfiguration('scan_topic'),
            }
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        map_file_arg,
        scan_topic_arg,
        relocalization_node,
    ])