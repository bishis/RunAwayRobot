import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # Static Transform Publisher for map->odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Static Transform Publisher for base_link->laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # Launch RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),

        # Launch Hector SLAM
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'pub_map_odom_transform': True,
                'scan_topic': '/scan',
                'map_resolution': 0.05,
                'map_size': 2048,
                'map_start_x': 0.5,
                'map_start_y': 0.5,
                'map_update_distance_thresh': 0.1,
                'map_update_angle_thresh': 0.1,
                'map_pub_period': 1.0,
                'update_factor_free': 0.4,
                'update_factor_occupied': 0.9,
                'laser_min_dist': 0.1,
                'laser_max_dist': 30.0,
                'laser_z_min_value': -1.0,
                'laser_z_max_value': 1.0,
            }]
        ),

        # Launch Mobile Robot Controller Node
        Node(
            package='motor_controller',
            executable='wall_follower',
            name='mobile_robot_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'safety_radius': 0.3,
                'detection_distance': 0.5,
                'turn_speed': 1.0,
                'linear_speed': 0.3,
            }]
        ),
    ])
