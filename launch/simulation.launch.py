from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    return LaunchDescription([
        # Navigation controller to handle cmd_vel
        Node(
            package='motor_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[{
                'max_linear_speed': 0.1,
                'max_angular_speed': 1.0,
                'linear_threshold': 0.01,
                'angular_threshold': 0.02,
            }],
            output='screen'
        ),

        # Robot simulator with LIDAR
        Node(
            package='motor_controller',
            executable='robot_simulator',
            name='robot_simulator',
            parameters=[{
                'wheel_separation': 0.24,
                'max_linear_speed': 0.1,
                'max_angular_speed': 1.0,
                'laser_range': 5.0,        # 5m range
                'laser_samples': 360,      # 1-degree resolution
                'laser_noise': 0.01,       # 1cm of noise
            }],
            output='screen'
        ),

        # Static transform for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_broadcaster',
            arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'laser']
        ),
    ]) 