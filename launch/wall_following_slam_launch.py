import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the path to config files
    pkg_dir = get_package_share_directory('motor_controller')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_config.yaml')
    
    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        use_sim_time_arg,

        # Launch RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),

        # TF Static Transform Publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_init',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        ),

        # Launch SLAM Toolbox with explicit parameters
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'transform_timeout': 0.2,
                'map_update_interval': 2.0,
                'publish_period': 1.0,
                'max_update_rate': 10.0,
                'enable_interactive_mode': False,
                'debug_logging': True,
                'throttle_scans': 1,
                'publish_frame_transforms': True
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
