import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Just the essential transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # Minimal SLAM configuration
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                
                # Force map publishing
                'publish_map': True,
                'map_publish_interval': 1.0,
                'resolution': 0.05,
                
                # Add these critical parameters
                'map_update_interval': 1.0,
                'transform_publish_period': 0.05,
                'map_update_periodicity': 1.0,
                'enable_interactive_mode': False,
                
                # Add scan matching parameters
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_time_interval': 0.1,
                'transform_timeout': 0.5,
                
                # Debug parameters
                'debug_logging': True,
                'throttle_scans': 1
            }]
        )
    ]) 