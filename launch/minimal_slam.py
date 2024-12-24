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

        # SLAM Toolbox with online_async mode
        Node(
            package='slam_toolbox',
            executable='online_async_launch.py',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'localization',
                
                # Basic parameters
                'publish_map': True,
                'resolution': 0.05,
                'max_laser_range': 20.0,
                
                # Performance parameters
                'map_update_interval': 1.0,
                'transform_timeout': 0.2,
                'update_timing': True,
                'enable_interactive_mode': True,
                
                # Scan Matching Parameters
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                
                # Debug
                'debug_logging': True
            }]
        ),

        # Add RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('motor_controller'),
                'config',
                'slam_view.rviz'
            )]
        )
    ]) 