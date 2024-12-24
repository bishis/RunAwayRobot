import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Basic transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

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

        # SLAM Toolbox with minimal configuration
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # Try sync instead of async
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',

                # Critical parameters
                'publish_map': True,
                'map_publish_interval': 1.0,
                'resolution': 0.05,
                'max_laser_range': 20.0,

                # Debug parameters
                'debug_logging': True,
                'throttle_scans': 1,

                # Performance parameters
                'transform_timeout': 1.0,
                'map_update_interval': 5.0,
                'max_update_rate': 10.0,

                # Matching parameters
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_travel_distance': 0.5,
                'minimum_travel_heading': 0.5,
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 20.0,

                # Loop closure parameters
                'loop_search_maximum_distance': 10.0,
                'do_loop_closing': True,
                'loop_match_minimum_chain_size': 10,
                'loop_match_maximum_variance_coarse': 3.0,
                'loop_match_minimum_response_coarse': 0.35,
                'loop_match_minimum_response_fine': 0.45,

                # Publishing parameters
                'publish_frame_transforms': True,
                'resolution_closure_threshold': 0.1,
                'minimum_time_interval': 0.5
            }]
        ),

        # RViz
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