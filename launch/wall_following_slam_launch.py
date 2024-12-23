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

        # Launch ICP Odometry Node
        Node(
            package='icp_localization_ros',
            executable='icp_localization',
            name='icp_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tf': True,
                'scan_topic': '/scan',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'max_iterations': 30,
                'tolerance': 0.001,
                'max_correspondence_distance': 0.1,
                'max_translation': 0.1,
                'max_rotation': 0.2,
            }]
        ),

        # Launch Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': 'map.yaml',
                'topic_name': 'map',
                'frame_id': 'map',
                'resolution': 0.05,
            }]
        ),

        # Launch Map Saver
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'save_map_timeout': 5.0,
                'free_thresh_default': 0.25,
                'occupied_thresh_default': 0.65,
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
