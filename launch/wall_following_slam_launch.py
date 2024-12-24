import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('motor_controller')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    
    return LaunchDescription([
        # 1. Launch RPLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py')
            )
        ),

        # 2. Launch RF2O Laser Odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'freq': 20.0
            }]
        ),

        # 3. Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # 4. Launch SLAM Toolbox with Hector-like parameters
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',

                # Map parameters (from Hector)
                'publish_map': True,
                'resolution': 0.025,  # Higher resolution like Hector
                'map_publish_interval': 0.5,
                'map_update_interval': 0.5,
                'map_size': 2048,  # Larger map size

                # Scan matching parameters
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_travel_distance': 0.05,
                'minimum_travel_heading': 0.1,
                'max_laser_range': 30.0,
                'transform_timeout': 0.1,
                'update_factor_free': 0.4,  # From Hector
                'update_factor_occupied': 0.9,  # From Hector
                'publish_period': 0.5,

                # Loop closure
                'enable_interactive_mode': False,
                'loop_search_maximum_distance': 5.0,
                'do_loop_closing': True
            }]
        ),

        # 5. Launch Robot Controller
        Node(
            package='motor_controller',
            executable='wall_follower',
            name='mobile_robot_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'safety_radius': 0.3,
                'detection_distance': 0.5,
                'turn_speed': 1.0,
                'linear_speed': 0.3,
            }]
        )
    ])
