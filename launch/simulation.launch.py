from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('motor_controller')
    
    return LaunchDescription([
        # Navigation controller
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
    ]) 