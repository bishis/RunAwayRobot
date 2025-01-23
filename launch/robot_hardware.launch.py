import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    return LaunchDescription([
        # Network setup
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),

        # Include the working RPLIDAR launch configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rplidar_ros'),
                           'launch', 'rplidar_a1_launch.py')
            ])
        ),

        # Hardware controller for motors
        Node(
            package='motor_controller',
            executable='hardware_controller',
            name='hardware_controller',
            parameters=[{
                'wheel_separation': 0.24,     # Distance between wheels
                'max_linear_speed': 0.1,      # Max 0.1 m/s
                'max_angular_speed': 2.0,     # Increased max turning speed
                'linear_threshold': 0.01,     # Lower threshold
                'angular_threshold': 0.02,    # Lower threshold for more responsive turning
                'forward_min_duty': 0.090,    # Increased minimum power (85%)
                'forward_max_duty': 0.100,    # Maximum forward
                'reverse_min_duty': 0.060,    # More aggressive reverse
                'neutral_duty': 0.0725,       # Neutral position
                'speed_exponent': 1.0,        # Linear response
            }],
            output='screen'
        ),

        # Basic transform publisher for lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        )
    ])
