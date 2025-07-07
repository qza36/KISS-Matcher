#!/usr/bin/env python3
"""
Example launch file for KISS-Matcher relocalization with sample data
This demonstrates how to use the relocalization node with nav2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('kiss_matcher_ros')
    
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_dir, '..', '..', 'data', 'sample_map.pcd'),
        description='Path to the prior PCD map file'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/velodyne_points',
        description='Point cloud topic to subscribe to'
    )
    
    auto_relocalize_arg = DeclareLaunchArgument(
        'auto_relocalize',
        default_value='false',
        description='Enable automatic relocalization on startup'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Relocalization node
    relocalization_node = Node(
        package='kiss_matcher_ros',
        executable='relocalization_node',
        name='relocalization_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'relocalization_config.yaml'),
            {
                'map_file_path': LaunchConfiguration('map_file'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'enable_auto_relocalization': LaunchConfiguration('auto_relocalize'),
            }
        ]
    )
    
    # Static transform publisher for example (replace with your robot's transforms)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne']
    )
    
    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'relocalization.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        map_file_arg,
        scan_topic_arg,
        auto_relocalize_arg,
        use_rviz_arg,
        static_tf_node,
        relocalization_node,
        rviz_node,
    ])